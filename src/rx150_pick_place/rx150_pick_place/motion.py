#!/usr/bin/env python3
"""
motion — lớp chấp hành chuyển động qua MoveIt cho các node pick-place.

Gộp bản MoveGroup primitive từng bị copy-paste ở 3 node, và sửa các điểm
KHÔNG đạt chuẩn công nghiệp của bản cũ:

  1. Bản cũ coi error_code −4 (CONTROL_FAILED) là THÀNH CÔNG cho MỌI nhóm khớp.
     Nghĩa là: tay KHÔNG tới đích mà chuỗi pick-place vẫn chạy tiếp → kẹp vào
     không khí, thả vật ra ngoài giá. Ở đây: CONTROL_FAILED không được tin,
     mà đối chiếu joint_states thực tế với đích (verify_reached) rồi mới quyết định.
  2. Bản cũ `res_future.result()` có thể là None (server chết giữa goal) →
     AttributeError, thoát ra exception handler tận ngoài worker loop.
  3. Bản cũ không có retry, không huỷ được goal đang chạy (không có E-stop).
  4. Bản cũ mọi bước là joint-space goal ⇒ đoạn hạ xuống kẹp / cắm ống là đường
     cong trong không gian. Ở đây có move_linear_ee(): nội suy THẲNG trong không
     gian Descartes (IK từng waypoint) — chuẩn cho pha approach/insert/retract.
"""
import math
import threading
import time

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory,
)
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .kinematics import ARM_JOINTS

# error_code → tên, để log ra chữ thay vì số (−1, −4 … không ai nhớ được)
ERROR_NAMES = {
    getattr(MoveItErrorCodes, n): n
    for n in dir(MoveItErrorCodes) if n.isupper() and isinstance(getattr(MoveItErrorCodes, n), int)
}


def error_name(code):
    return f'{ERROR_NAMES.get(code, "UNKNOWN")}({code})'


class JointStateMonitor:
    """Theo dõi joint_states — dùng để (a) seed IK, (b) verify pose đã tới,
    (c) kiểm tra ngón kẹp có vật hay không. Có kiểm tra ĐỘ CŨ của dữ liệu:
    verify bằng số liệu chết 5 s trước còn tệ hơn không verify."""

    def __init__(self, node, topic='/rx150/joint_states', callback_group=None,
                 joint_names=None):
        self._node = node
        self._names = list(joint_names or ARM_JOINTS)
        self._lock = threading.Lock()
        self._pos = {}
        self._stamp = 0.0
        node.create_subscription(JointState, topic, self._cb, 10,
                                 callback_group=callback_group)
        self.topic = topic

    def _cb(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._pos[name] = float(pos)
            self._stamp = time.monotonic()

    def age(self):
        with self._lock:
            return math.inf if self._stamp == 0.0 else time.monotonic() - self._stamp

    def position(self, joint):
        with self._lock:
            return self._pos.get(joint)

    def arm_positions(self):
        """[5] theo thứ tự ARM_JOINTS, hoặc None nếu chưa đủ dữ liệu."""
        with self._lock:
            if not all(n in self._pos for n in self._names):
                return None
            return [self._pos[n] for n in self._names]

    def wait_ready(self, timeout=5.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.arm_positions() is not None:
                return True
            time.sleep(0.05)
        self._node.get_logger().error(
            f'Không nhận được {self.topic} sau {timeout:.0f}s — kiểm tra xs_sdk / namespace.')
        return False


class MoveItExecutor:
    """Gửi goal tới move_group và XÁC NHẬN kết quả. Thread-safe cho 1 worker."""

    def __init__(self, node, kin, js: JointStateMonitor, *,
                 base_frame='rx150/base_link', arm_group='interbotix_arm',
                 callback_group=None, goal_tolerance_rad=0.01,
                 verify_tolerance_rad=0.06, planning_time_s=5.0,
                 execution_timeout_s=30.0, planning_attempts=5, retries=1,
                 dry_run=False, linear_step_m=0.006, use_linear=True):
        self._node = node
        self._log = node.get_logger()
        self.kin = kin
        self.js = js
        self.base_frame = base_frame
        self.arm_group = arm_group
        self.goal_tol = float(goal_tolerance_rad)
        self.verify_tol = float(verify_tolerance_rad)
        self.planning_time = float(planning_time_s)
        self.exec_timeout = float(execution_timeout_s)
        self.planning_attempts = int(planning_attempts)
        self.retries = int(retries)
        self.dry_run = bool(dry_run)
        self.linear_step = float(linear_step_m)
        self.use_linear = bool(use_linear)

        self._move = ActionClient(node, MoveGroup, 'move_action',
                                  callback_group=callback_group)
        self._exec = ActionClient(node, ExecuteTrajectory, 'execute_trajectory',
                                  callback_group=callback_group)
        self._handle_lock = threading.Lock()
        self._handle = None
        self._abort = threading.Event()
        self._linear_unavailable = False
        # dry_run: không có joint_states thật ⇒ giữ tư thế "ảo" để chạy thử được
        # TRỌN chuỗi (kể cả các đoạn đi thẳng và pha RECOVERY), không phải dừng
        # ở bước đầu tiên vì thiếu dữ liệu.
        self._virtual = None

    def set_virtual_joints(self, joints):
        self._virtual = list(joints)

    def current_joints(self):
        """Tư thế dùng để seed IK / tính đoạn đi thẳng."""
        if self.dry_run:
            return list(self._virtual) if self._virtual is not None else None
        return self.js.arm_positions()

    # ── E-stop / huỷ ────────────────────────────────────────────────────
    def request_abort(self):
        """Huỷ goal đang chạy và bắt mọi bước sau đó fail ngay (E-stop mềm)."""
        self._abort.set()
        with self._handle_lock:
            handle = self._handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
                self._log.warn('Đã gửi cancel tới goal MoveIt đang chạy.')
            except Exception as exc:                       # noqa: BLE001
                self._log.warn(f'Cancel goal thất bại: {exc}')

    def clear_abort(self):
        self._abort.clear()

    @property
    def aborted(self):
        return self._abort.is_set()

    # ── hạ tầng ─────────────────────────────────────────────────────────
    def wait_ready(self, timeout=10.0):
        if self.dry_run:
            return True
        if not self._move.wait_for_server(timeout_sec=timeout):
            self._log.error(
                "move_action chưa sẵn sàng — move_group chưa chạy? "
                "(T1: ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py)")
            return False
        return self.js.wait_ready(timeout=timeout)

    def _wait(self, future, timeout):
        """Executor spin node ở thread khác ⇒ poll ở đây. Thoát sớm khi E-stop."""
        t0 = time.monotonic()
        while not future.done():
            if not self._node.context.ok():
                return False
            if self._abort.is_set():
                self._log.warn('Bỏ chờ future vì E-stop.')
                return False
            if time.monotonic() - t0 > timeout:
                self._log.error(f'Future timeout sau {timeout:.1f}s.')
                return False
            time.sleep(0.01)
        return True

    # ── verify: đã tới đích thật chưa ───────────────────────────────────
    def verify_reached(self, joint_names, targets, tol=None, settle_s=0.15):
        """So joint_states thực tế với đích. (ok, sai số lớn nhất)."""
        tol = self.verify_tol if tol is None else float(tol)
        time.sleep(settle_s)
        if self.js.age() > 1.0:
            self._log.warn(f'joint_states cũ {self.js.age():.1f}s — không verify được.')
            return False, math.inf
        worst, worst_name = 0.0, ''
        for name, tgt in zip(joint_names, targets):
            actual = self.js.position(name)
            if actual is None:
                return False, math.inf
            err = abs(actual - float(tgt))
            if err > worst:
                worst, worst_name = err, name
        if worst > tol:
            self._log.warn(f'Sai số còn lại {worst_name}={math.degrees(worst):.2f}° '
                           f'> tol {math.degrees(tol):.2f}°.')
        return worst <= tol, worst

    # ── primitive: joint-space goal ─────────────────────────────────────
    def move_joints(self, joints, velocity_scale, label='MOVE', *, group=None,
                    joint_names=None, allowed_time=None, retries=None,
                    verify=True, verify_tol=None):
        group = group or self.arm_group
        joint_names = list(joint_names or ARM_JOINTS)
        targets = [float(v) for v in joints]
        allowed_time = self.planning_time if allowed_time is None else float(allowed_time)
        retries = self.retries if retries is None else int(retries)

        if self.dry_run:
            self._log.info(f'[DRY-RUN] {label}: {group} → '
                           f'{[round(math.degrees(t), 1) for t in targets]}° '
                           f'v={velocity_scale}')
            if group == self.arm_group and len(targets) == len(ARM_JOINTS):
                self._virtual = list(targets)
            return True

        for attempt in range(retries + 1):
            if self._abort.is_set():
                self._log.warn(f'{label}: bỏ qua vì E-stop.')
                return False
            code = self._send_move_goal(group, joint_names, targets,
                                        velocity_scale, allowed_time)
            if code == MoveItErrorCodes.SUCCESS:
                if not verify:
                    return True
                ok, _ = self.verify_reached(joint_names, targets, tol=verify_tol)
                if ok:
                    return True
                self._log.warn(f'{label}: MoveIt báo SUCCESS nhưng pose chưa đạt tolerance.')
            elif code in (MoveItErrorCodes.CONTROL_FAILED,
                          MoveItErrorCodes.TIMED_OUT) and verify:
                # Bridge fuzzy/hac báo tolerance violated là chuyện thường; ĐỪNG tin
                # mù (bản cũ nhận −4 là SUCCESS) — đối chiếu pose thực tế.
                ok, worst = self.verify_reached(joint_names, targets, tol=verify_tol)
                if ok:
                    self._log.warn(
                        f'{label}: {error_name(code)} nhưng pose thực tế đã đạt '
                        f'(sai số {math.degrees(worst):.2f}°) → chấp nhận.')
                    return True
                self._log.error(f'{label}: {error_name(code)} và pose CHƯA đạt.')
            else:
                self._log.error(f'{label}: {error_name(code)}.')

            if attempt < retries:
                self._log.warn(f'{label}: thử lại ({attempt + 2}/{retries + 1}).')
                time.sleep(0.3)
        return False

    def _send_move_goal(self, group, joint_names, targets, velocity_scale, allowed_time):
        """Trả error_code (int). FAILURE nếu không gửi/nhận được kết quả."""
        if not self._move.wait_for_server(timeout_sec=2.0):
            self._log.error('move_action server chưa sẵn sàng.')
            return MoveItErrorCodes.FAILURE

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.start_state.is_diff = True
        req.workspace_parameters.header.frame_id = self.base_frame
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        req.allowed_planning_time = float(allowed_time)
        req.num_planning_attempts = self.planning_attempts
        scale = max(0.01, min(1.0, float(velocity_scale)))
        req.max_velocity_scaling_factor = scale
        req.max_acceleration_scaling_factor = scale

        constraints = Constraints()
        for name, tgt in zip(joint_names, targets):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(tgt)
            jc.tolerance_above = self.goal_tol
            jc.tolerance_below = self.goal_tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal_future = self._move.send_goal_async(goal)
        if not self._wait(goal_future, timeout=5.0):
            return MoveItErrorCodes.FAILURE
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            self._log.error(f'MoveIt từ chối goal ({group}).')
            return MoveItErrorCodes.FAILURE

        with self._handle_lock:
            self._handle = handle
        try:
            res_future = handle.get_result_async()
            if not self._wait(res_future, timeout=allowed_time + self.exec_timeout):
                return MoveItErrorCodes.TIMED_OUT
            res = res_future.result()
            if res is None or res.result is None:
                self._log.error('MoveIt trả kết quả rỗng (server chết giữa goal?).')
                return MoveItErrorCodes.FAILURE
            return int(res.result.error_code.val)
        finally:
            with self._handle_lock:
                self._handle = None

    # ── primitive: đi THẲNG trong không gian Descartes ───────────────────
    def move_linear_ee(self, target, velocity_scale, label='LINEAR', *,
                       speed_mps=0.04, wrist_rotate=None, fallback=True,
                       verify=True):
        """Đi thẳng từ pose hiện tại tới target=(x, y, z, pitch), giữ hướng đổi
        tuyến tính. Dùng cho DESCEND / LIFT / INSERT / RETRACT.

        Cách làm: nội suy N waypoint trên đoạn thẳng → IK từng waypoint (seed
        bằng waypoint trước ⇒ không lật khuỷu) → gửi 1 quỹ đạo qua
        /execute_trajectory. Không có action đó (hoặc IK hụt 1 waypoint) thì
        fallback về joint goal qua move_group.
        """
        current = self.current_joints()
        if current is None:
            self._log.error(f'{label}: chưa có joint_states — không đi thẳng được '
                            f'(topic {self.js.topic}).')
            return False
        x0, y0, z0, p0 = self.kin.fk(current)
        x1, y1, z1, p1 = (float(v) for v in target)
        roll = current[4] if wrist_rotate is None else float(wrist_rotate)
        dist = math.dist((x0, y0, z0), (x1, y1, z1))

        traj = None
        if self.use_linear and not self._linear_unavailable and dist > 1e-4:
            traj = self._build_linear_traj(current, (x0, y0, z0, p0), (x1, y1, z1, p1),
                                           roll, dist, speed_mps, label)
        if traj is None:
            if not fallback:
                return False
            joints = self.kin.ik(x1, y1, z1, p1, seed=current, wrist_rotate=roll)
            if joints is None:
                self._log.error(f'{label}: IK fail — {self.kin.reach_report(x1, y1, z1, p1)}')
                return False
            return self.move_joints(joints, velocity_scale, label=label, verify=verify)

        if self.dry_run:
            self._log.info(f'[DRY-RUN] {label}: đi thẳng {dist * 100:.1f}cm '
                           f'({len(traj.points)} waypoint) → '
                           f'({x1:.3f},{y1:.3f},{z1:.3f}) pitch={math.degrees(p1):.1f}°')
            self._virtual = list(traj.points[-1].positions)
            return True

        ok = self._execute_traj(traj, label)
        if not ok and fallback and not self._abort.is_set():
            self._log.warn(f'{label}: đi thẳng thất bại → fallback joint goal.')
            joints = list(traj.points[-1].positions)
            return self.move_joints(joints, velocity_scale, label=label + '/fallback',
                                    verify=verify)
        if ok and verify:
            reached, _ = self.verify_reached(ARM_JOINTS, list(traj.points[-1].positions))
            return reached
        return ok

    def _build_linear_traj(self, current, start, goal, roll, dist, speed_mps, label):
        n = max(3, int(math.ceil(dist / max(self.linear_step, 1e-3))) + 1)
        duration = max(0.5, dist / max(float(speed_mps), 1e-3))
        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINTS)
        seed = list(current)
        for k in range(n):
            # ease-in/out: mượt gia tốc dù bridge chỉ nội suy tuyến tính vị trí
            frac = 0.5 * (1.0 - math.cos(math.pi * k / (n - 1)))
            pose = [a + (b - a) * frac for a, b in zip(start, goal)]
            if k == 0:
                joints = list(current)          # điểm đầu = pose ĐO ĐƯỢC (start tolerance)
            else:
                joints = self.kin.ik(pose[0], pose[1], pose[2], pose[3],
                                     seed=seed, wrist_rotate=roll)
                if joints is None:
                    self._log.warn(
                        f'{label}: waypoint {k}/{n - 1} không có IK '
                        f'({self.kin.reach_report(pose[0], pose[1], pose[2], pose[3])}) '
                        f'→ không đi thẳng được.')
                    return None
            seed = joints
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in joints]
            t = duration * k / (n - 1)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)
        return traj

    def _execute_traj(self, traj: JointTrajectory, label):
        if not self._exec.wait_for_server(timeout_sec=2.0):
            self._log.warn('execute_trajectory chưa sẵn sàng — tắt chế độ đi thẳng.')
            self._linear_unavailable = True
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = RobotTrajectory(joint_trajectory=traj)
        goal_future = self._exec.send_goal_async(goal)
        if not self._wait(goal_future, timeout=5.0):
            return False
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            self._log.warn(f'{label}: execute_trajectory từ chối goal.')
            return False
        with self._handle_lock:
            self._handle = handle
        try:
            duration = (traj.points[-1].time_from_start.sec
                        + traj.points[-1].time_from_start.nanosec * 1e-9)
            res_future = handle.get_result_async()
            if not self._wait(res_future, timeout=duration + self.exec_timeout):
                return False
            res = res_future.result()
            if res is None or res.result is None:
                return False
            code = int(res.result.error_code.val)
            if code == MoveItErrorCodes.SUCCESS:
                return True
            if code == MoveItErrorCodes.CONTROL_FAILED:
                ok, worst = self.verify_reached(ARM_JOINTS,
                                                list(traj.points[-1].positions))
                if ok:
                    self._log.warn(f'{label}: CONTROL_FAILED nhưng pose đã đạt '
                                   f'(sai số {math.degrees(worst):.2f}°).')
                    return True
            self._log.warn(f'{label}: execute_trajectory {error_name(code)}.')
            return False
        finally:
            with self._handle_lock:
                self._handle = None
