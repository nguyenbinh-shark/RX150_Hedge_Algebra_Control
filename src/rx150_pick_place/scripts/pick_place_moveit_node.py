#!/usr/bin/env python3
"""
pick_place_moveit_node — LAYER 2: QUYẾT ĐỊNH (rx150 pick-and-place qua MoveIt).

Đọc nhận diện (Layer 1) + cử chỉ tay → chọn vật → gọi MoveIt (Layer 3) gắp & thả.
TẤT CẢ chuyển động đi qua move_group (action 'move_action'); KHÔNG publish trực tiếp
/rx150/commands/* (sẽ đánh fuzzy_node). SDK `bot` CHỈ dùng làm IK-oracle
(bot.arm.set_ee_pose_components(execute=False) — arm.py:542-547 chỉ publish khi execute).

Luồng dữ liệu:
  /yolo/detected_objects (PoseArray, rx150/base_link)   ──► giữ pose mới nhất
  /hand_gesture/selected_target (Int32)                 ──► index vật đang được chỉ tay
  /hand_gesture/event ("ok_sign")                       ──► KÍCH gắp vật đang chọn

State machine (mỗi bước ARM = 1 goal MoveGroup nhóm interbotix_arm; gripper = nhóm
interbotix_gripper qua gripper_trajectory_bridge, hoặc PWM fallback):
  IDLE --ok_sign--> APPROACH(pre) -> DESCEND(grasp) -> GRASP -> LIFT(pre)
       -> TRANSPORT(pre-place) -> PLACE -> RELEASE -> RETREAT -> HOME -> IDLE

5-DoF: chỉ position + pitch là DOF định hướng đạt được (SDK ép yaw=atan2(y,x)).
Tham số: xem config/pick_place_params.yaml. Yêu cầu: T1 fuzzy_moveit.launch.py,
T2 pick_place.launch.py (xem README).
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String, Int32
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node, robot_shutdown, robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand

ROBOT_MODEL = 'rx150'
ROBOT_NAME = ROBOT_MODEL
ARM_GROUP = 'interbotix_arm'
GRIPPER_GROUP = 'interbotix_gripper'
ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
FINGER_JOINT = 'left_finger'
GRASP_FINGER = 0.015    # m — đóng (kẹp vật)
RELEASE_FINGER = 0.037  # m — mở hết
GRIP_PWM = 200.0        # PWM fallback — đóng
OPEN_PWM = -200.0       # PWM fallback — mở
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0]
OK_CODES = (1, -4)      # 1=SUCCESS, -4=CONTROL_FAILED (bridge báo tolerance nhưng đã xong)


class _LatestDetection:
    """Holder thread-safe cho PoseArray mới nhất từ yolo_detector_node."""

    def __init__(self):
        self._lock = threading.Lock()
        self._poses = []

    def set(self, poses):
        with self._lock:
            self._poses = list(poses)

    def get(self):
        with self._lock:
            return list(self._poses)


class PickPlaceMoveItNode(Node):
    def __init__(self, bot):
        super().__init__('pick_place_moveit')
        self.bot = bot

        # ---------------- parameters (default khớp config/pick_place_params.yaml) ----------------
        self.declare_parameter('approach_delta', 0.05)
        self.declare_parameter('grasp_pitch', 0.5)
        self.declare_parameter('finger_grasp_offset', 0.02)
        self.declare_parameter('velocity_scale_cruise', 0.3)
        self.declare_parameter('velocity_scale_delicate', 0.1)
        self.declare_parameter('place_x', 0.30)
        self.declare_parameter('place_y', 0.0)
        self.declare_parameter('place_z', 0.05)
        self.declare_parameter('place_pitch', 0.5)
        self.declare_parameter('home_joints', HOME_JOINTS)
        self.declare_parameter('detection_wait_s', 10.0)
        # Nguồn PoseArray vật thể: YOLO (mặc định) hoặc cluster_bridge (PCL) —
        # pose phải nằm trong frame rx150/base_link ở cả hai phía.
        self.declare_parameter('detection_topic', '/yolo/detected_objects')
        self.declare_parameter('use_gripper_bridge', True)
        self.declare_parameter('add_table_collision', True)
        self.declare_parameter('table_x', 0.30)
        self.declare_parameter('table_y', 0.0)
        self.declare_parameter('table_z', -0.02)   # tâm box; mặt bàn ~z=0 → đỉnh box dưới z=0
        self.declare_parameter('table_size_x', 0.80)
        self.declare_parameter('table_size_y', 0.80)
        self.declare_parameter('table_size_z', 0.10)

        # ---------------- state ----------------
        self._cb = ReentrantCallbackGroup()
        self._det = _LatestDetection()
        self._selected = -1
        self._lock = threading.Lock()
        self._busy = False
        self._pick_event = threading.Event()
        self._pick_idx = -1
        self._scene_done = False

        # ---------------- MoveGroup action client (arm + gripper cùng 'move_action') ----------------
        self._move = ActionClient(self, MoveGroup, 'move_action', callback_group=self._cb)
        self._apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self._cb)

        # ---------------- subscriptions Layer 1 ----------------
        self.create_subscription(
            PoseArray, self.get_parameter('detection_topic').value, self._poses_cb, 10,
            callback_group=self._cb)
        self.create_subscription(Int32, '/hand_gesture/selected_target', self._target_cb, 10,
                                 callback_group=self._cb)
        self.create_subscription(String, '/hand_gesture/event', self._event_cb, 10,
                                 callback_group=self._cb)

        # ---------------- worker: chạy state machine (không block executor) ----------------
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            'pick_place_moveit sẵn sàng. Chờ /hand_gesture/event="ok_sign" để gắp vật tại '
            '/hand_gesture/selected_target '
            f'(gripper_bridge={self.get_parameter("use_gripper_bridge").value}).')

    # ---------------- callbacks: chỉ cập nhật data, KHÔNG block ----------------
    def _poses_cb(self, msg: PoseArray):
        self._det.set(msg.poses)

    def _target_cb(self, msg: Int32):
        self._selected = int(msg.data)

    def _event_cb(self, msg: String):
        if msg.data != 'ok_sign':
            return
        with self._lock:
            if self._busy:
                self.get_logger().info('ok_sign bỏ qua — đang bận gắp.',
                                       throttle_duration_sec=2.0)
                return
            self._busy = True
            self._pick_idx = self._selected
        self._pick_event.set()
        self.get_logger().info(f'ok_sign → yêu cầu gắp object #{self._selected}')

    # ---------------- worker loop ----------------
    def _worker_loop(self):
        while rclpy.ok():
            self._pick_event.wait()
            self._pick_event.clear()
            idx = self._pick_idx
            try:
                self._run_pick(idx)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Pick thất bại (idx={idx}): {exc}')
            finally:
                with self._lock:
                    self._busy = False

    # ---------------- helper: block worker đến khi future xong ----------------
    def _wait_future(self, future, timeout=60.0):
        """Executor chính đang spin node ở thread khác → future xong ở đó; poll ở đây."""
        t0 = time.monotonic()
        while not future.done() and rclpy.ok():
            if time.monotonic() - t0 > timeout:
                self.get_logger().error(f'Future timeout sau {timeout:.0f}s.')
                return False
            time.sleep(0.01)
        return future.done()

    # ---------------- MoveGroup primitive (joint-space goal) ----------------
    def move_to_joint_target(self, group_name, joint_names, targets, velocity_scale,
                             allowed_time=5.0):
        """Gửi MoveGroup goal (JointConstraint mỗi khớp) → plan+execute qua move_group.
        Trả True nếu error_code ∈ {1 (SUCCESS), -4 (CONTROL_FAILED)}."""
        if not self._move.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('move_action server chưa sẵn sàng.')
            return False

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group_name
        req.start_state.is_diff = True
        req.workspace_parameters.header.frame_id = f'{ROBOT_NAME}/base_link'
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0
        req.allowed_planning_time = float(allowed_time)
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = float(velocity_scale)
        req.max_acceleration_scaling_factor = float(velocity_scale)

        c = Constraints()
        for jn, tgt in zip(joint_names, targets):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(tgt)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        goal_future = self._move.send_goal_async(goal)
        if not self._wait_future(goal_future):
            return False
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'MoveIt reject goal ({group_name}).')
            return False

        res_future = handle.get_result_async()
        if not self._wait_future(res_future, timeout=allowed_time + 15.0):
            return False
        res = res_future.result()
        code = res.result.error_code.val
        if code not in OK_CODES:
            self.get_logger().error(f'MoveIt fail ({group_name}) error_code={code}.')
            return False
        return True

    # ---------------- IK oracle (SDK, execute=False → KHÔNG publish) ----------------
    def _ik(self, x, y, z, pitch):
        joints, ok = self.bot.arm.set_ee_pose_components(
            x=float(x), y=float(y), z=float(z), pitch=float(pitch), execute=False)
        if not ok:
            self.get_logger().warn(f'IK fail x={x:.3f} y={y:.3f} z={z:.3f} pitch={pitch}.')
        return joints, ok

    # ---------------- gripper: bridge (MoveIt) hoặc PWM fallback ----------------
    def _gripper(self, grasp: bool):
        """grasp=True → kẹp (0.015); False → mở (0.037)."""
        if self.get_parameter('use_gripper_bridge').value:
            tgt = GRASP_FINGER if grasp else RELEASE_FINGER
            # velocity_scale thấp → quỹ đạo chậm → gripper_bridge giữ effort lâu hơn khi gắp
            return self.move_to_joint_target(
                GRIPPER_GROUP, [FINGER_JOINT], [tgt],
                self.get_parameter('velocity_scale_delicate').value, allowed_time=4.0)
        # fallback raw PWM (như test_pick_place_moveit.py): JointSingleCommand → xs_sdk
        cmd = JointSingleCommand(name='gripper', cmd=float(GRIP_PWM if grasp else OPEN_PWM))
        self.bot.core.pub_single.publish(cmd)
        time.sleep(2.0)
        return True

    # ---------------- HOME ----------------
    def _go_home(self):
        return self.move_to_joint_target(
            ARM_GROUP, ARM_JOINTS, list(self.get_parameter('home_joints').value),
            self.get_parameter('velocity_scale_cruise').value)

    # ---------------- planning scene: box bàn (ADD 1 lần) ----------------
    def _ensure_scene(self):
        if self._scene_done or not self.get_parameter('add_table_collision').value:
            return
        if not self._apply_scene.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '/apply_planning_scene chưa sẵn sàng — bỏ qua box bàn (dùng OctoMap).')
            self._scene_done = True
            return
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dim = [float(self.get_parameter('table_size_x').value),
                   float(self.get_parameter('table_size_y').value),
                   float(self.get_parameter('table_size_z').value)]
        pose = Pose()
        pose.position.x = float(self.get_parameter('table_x').value)
        pose.position.y = float(self.get_parameter('table_y').value)
        pose.position.z = float(self.get_parameter('table_z').value)
        pose.orientation.w = 1.0

        co = CollisionObject()
        co.header.frame_id = 'world'
        co.id = 'table'
        co.operation = CollisionObject.ADD
        co.primitives = [box]
        co.primitive_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]

        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self._apply_scene.call_async(req)
        self._wait_future(fut, timeout=5.0)
        self._scene_done = True
        self.get_logger().info('Đã thêm collision box "table" vào planning scene.')

    # ---------------- STATE MACHINE: pick 1 vật ----------------
    def _run_pick(self, idx):
        poses = self._det.get()
        if idx < 0 or idx >= len(poses):
            self.get_logger().warn(f'Không có detection hợp lệ tại index #{idx} '
                                   f'(có {len(poses)} pose). Bỏ qua.')
            return
        p = poses[idx]
        ox, oy, oz = float(p.position.x), float(p.position.y), float(p.position.z)

        pitch = float(self.get_parameter('grasp_pitch').value)
        delta = float(self.get_parameter('approach_delta').value)
        off = float(self.get_parameter('finger_grasp_offset').value)
        vcruise = float(self.get_parameter('velocity_scale_cruise').value)
        vdel = float(self.get_parameter('velocity_scale_delicate').value)
        px = float(self.get_parameter('place_x').value)
        py = float(self.get_parameter('place_y').value)
        pz = float(self.get_parameter('place_z').value)
        ppitch = float(self.get_parameter('place_pitch').value)

        self._ensure_scene()
        self.get_logger().info(f'==== PICK object #{idx} tại ({ox:.3f},{oy:.3f},{oz:.3f}) ====')

        def step(name, x, y, z, pi, vs):
            j, ok = self._ik(x, y, z, pi)
            if not ok:
                self.get_logger().error(f'{name}: IK fail — abort.')
                return False
            return self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, list(j), vs)

        # 1. approach (trên vật)
        if not step('APPROACH', ox, oy, oz + delta, pitch, vcruise):
            return self._abort()
        # 2. descend (gắp)
        if not step('DESCEND', ox, oy, oz - off, pitch, vdel):
            return self._abort()
        # 3. grasp
        if not self._gripper(grasp=True):
            return self._abort()
        # 4. lift
        if not step('LIFT', ox, oy, oz + delta, pitch, vcruise):
            return self._abort()
        # 5. transport (pre-place)
        if not step('TRANSPORT', px, py, pz + delta, ppitch, vcruise):
            return self._abort()
        # 6. place
        if not step('PLACE', px, py, pz, ppitch, vdel):
            return self._abort()
        # 7. release
        if not self._gripper(grasp=False):
            return self._abort()
        # 8. retreat (lùi 10cm theo phương ngang về gốc bàn)
        norm = math.hypot(px, py) or 1.0
        rx = px - 0.10 * (px / norm)
        ry = py - 0.10 * (py / norm)
        if not step('RETREAT', rx, ry, pz + delta, ppitch, vcruise):
            self.get_logger().warn('RETREAT fail — tiếp tục về home.')
        # 9. home
        self._go_home()
        self.get_logger().info('==== PICK-PLACE hoàn tất ====')

    def _abort(self):
        self.get_logger().warn('ABORT — mở gripper & về home.')
        self._gripper(grasp=False)
        self._go_home()
        return False


def main(args=None):
    rclpy.init(args=args)
    global_node = create_interbotix_global_node('pick_place_moveit_control')
    bot = InterbotixManipulatorXS(robot_model=ROBOT_MODEL, robot_name=ROBOT_NAME, node=global_node)
    robot_startup(global_node)
    # Gripper PWM mode: cần cho fallback raw PWM; gripper_bridge cũng xài PWM effort.
    try:
        bot.core.robot_set_operating_modes('single', 'gripper', 'pwm')
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('pick_place_moveit').warn(f'Không set gripper pwm: {exc}')

    node = PickPlaceMoveItNode(bot)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        robot_shutdown(global_node)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
