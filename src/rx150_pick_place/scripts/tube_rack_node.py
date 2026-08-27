#!/usr/bin/env python3
"""
tube_rack_node — Gắp ống nghiệm nằm ngang → cắm thẳng đứng lên giá nghiêng, phân theo màu.

Kiến trúc (Layer 2):
  - Input:  /yolo/detected_tubes (PoseArray, frame rx150/base_link)
            /yolo/tube_classes   (String JSON — index song song)
  - Motion: MoveIt Action 'move_action' (JointConstraint goal) → fuzzy/hac bridge
  - Gripper: gripper_trajectory_bridge (stall-aware) | PWM fallback
  - IK:     SDK bot.arm execute=False (oracle only — KHÔNG publish commands)

Yêu cầu T1:
  ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
      use_camera:=true use_camera_static_tf:=false
  (hoặc rx150_hac_controller hac_moveit.launch.py ...)

Chạy:
  ros2 launch rx150_pick_place tube_rack.launch.py
  ros2 launch rx150_pick_place tube_rack.launch.py dry_run:=true
  ros2 launch rx150_pick_place tube_rack.launch.py auto_start:=false
  (trigger tay:  ros2 service call /tube_rack/run std_srvs/srv/Trigger)
"""
import json
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node, robot_shutdown, robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand

# ─── constants ───────────────────────────────────────────────────────────────
ROBOT_MODEL = 'rx150'
ROBOT_NAME = ROBOT_MODEL
ARM_GROUP = 'interbotix_arm'
GRIPPER_GROUP = 'interbotix_gripper'
ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
FINGER_JOINT = 'left_finger'

GRASP_FINGER = 0.015     # m — đóng (kẹp vật)
RELEASE_FINGER = 0.037   # m — mở hết
GRIP_PWM = 200.0
OPEN_PWM = -200.0
OK_CODES = (1, -4)       # 1=SUCCESS, -4=CONTROL_FAILED (bridge tolerance)


# ═══════════════════════════════════════════════════════════════════════════
# Detection buffer — median ~5 frames cho mỗi ống (chống nhiễu 1 frame)
# ═══════════════════════════════════════════════════════════════════════════
class _DetectionBuffer:
    """Thread-safe buffer thu thập nhiều frame (PoseArray + classes) rồi tính median."""

    def __init__(self):
        self._lock = threading.Lock()
        self._poses = []
        self._classes = []

    def update_poses(self, poses):
        with self._lock:
            self._poses = list(poses)
            self._try_sync()

    def update_classes(self, classes_list):
        with self._lock:
            self._classes = list(classes_list)
            self._try_sync()

    def _try_sync(self):
        pass  # sync kiểm tra ở snapshot()

    def snapshot_once(self):
        """Trả list[dict] từ frame hiện tại (poses + classes cùng length)."""
        with self._lock:
            if len(self._poses) == 0 or len(self._classes) != len(self._poses):
                return None
            tubes = []
            for p, cls in zip(self._poses, self._classes):
                qz = p.orientation.z
                qw = p.orientation.w
                yaw = 2.0 * math.atan2(qz, qw)
                tubes.append({
                    'class': str(cls).lower(),
                    'x': float(p.position.x),
                    'y': float(p.position.y),
                    'z': float(p.position.z),
                    'yaw': float(yaw),
                })
            return tubes


# ═══════════════════════════════════════════════════════════════════════════
# Main node
# ═══════════════════════════════════════════════════════════════════════════
class TubeRackNode(Node):
    def __init__(self, bot):
        super().__init__('tube_rack')
        self.bot = bot
        self._cb = ReentrantCallbackGroup()

        # ── params ──────────────────────────────────────────────────────
        self.declare_parameter('home_joints', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('detection_wait_s', 10.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('fake_tubes', '[]')
        self.declare_parameter('approach_delta', 0.08)
        self.declare_parameter('tube_length', 0.10)
        self.declare_parameter('insert_depth', 0.03)
        self.declare_parameter('velocity_scale_cruise', 0.3)
        self.declare_parameter('velocity_scale_descend', 0.12)
        self.declare_parameter('velocity_scale_insert', 0.1)
        self.declare_parameter('velocity_scale_lift', 0.2)
        self.declare_parameter('grasp_pitch_ladder', [1.5708, 1.40, 1.25, 1.10, 0.95])
        self.declare_parameter('place_pitch_ladder', [0.0, 0.15, 0.3])
        self.declare_parameter('detection_topic', '/yolo/detected_tubes')
        self.declare_parameter('classes_topic', '/yolo/tube_classes')
        self.declare_parameter('use_gripper_bridge', True)
        self.declare_parameter('median_frames', 5)
        self.declare_parameter('median_collect_s', 1.0)
        self.declare_parameter('rack_filter_xy_m', 0.05)
        # Rack geometry
        self.declare_parameter('slot0_x', 0.24)
        self.declare_parameter('slot0_y', -0.01)
        self.declare_parameter('slot0_z', 0.10)
        self.declare_parameter('rack_angle_deg', 62.0)
        self.declare_parameter('slot_spacing', 0.12)
        self.declare_parameter('num_slots', 4)
        # Colour → slot priority
        self.declare_parameter('color_slot_map.pink', [0])
        self.declare_parameter('color_slot_map.blue', [1])
        self.declare_parameter('color_slot_map.green', [2])
        self.declare_parameter('color_slot_map.yellow', [3])
        # Collision box for rack
        self.declare_parameter('add_rack_collision', True)
        self.declare_parameter('rack_box_x', 0.30)
        self.declare_parameter('rack_box_y', -0.01)
        self.declare_parameter('rack_box_z', 0.10)
        self.declare_parameter('rack_box_size_x', 0.15)
        self.declare_parameter('rack_box_size_y', 0.10)
        self.declare_parameter('rack_box_size_z', 0.20)
        # Collision box for table
        self.declare_parameter('add_table_collision', True)
        self.declare_parameter('table_x', 0.30)
        self.declare_parameter('table_y', 0.0)
        self.declare_parameter('table_z', -0.05)
        self.declare_parameter('table_size_x', 0.80)
        self.declare_parameter('table_size_y', 0.80)
        self.declare_parameter('table_size_z', 0.10)

        # ── state ───────────────────────────────────────────────────────
        self._det = _DetectionBuffer()
        self._busy = False
        self._lock = threading.Lock()
        self._scene_done = False
        self._slot_occupied = {}  # slot_index → bool

        # ── MoveGroup action client ─────────────────────────────────────
        self._move = ActionClient(self, MoveGroup, 'move_action', callback_group=self._cb)
        self._apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self._cb)

        # ── subscriptions ───────────────────────────────────────────────
        self.create_subscription(
            PoseArray, self.get_parameter('detection_topic').value,
            self._poses_cb, 10, callback_group=self._cb)
        self.create_subscription(
            String, self.get_parameter('classes_topic').value,
            self._classes_cb, 10, callback_group=self._cb)

        # ── service: trigger manual run ─────────────────────────────────
        self.create_service(Trigger, '/tube_rack/run', self._run_srv_cb,
                            callback_group=self._cb)

        # ── status publisher ────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/tube_rack/status', 10)

        # ── worker thread ───────────────────────────────────────────────
        self._run_event = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        if self.get_parameter('auto_start').value:
            self.get_logger().info('auto_start=true → bắt đầu chu kỳ gắp ống ngay.')
            self._run_event.set()
        else:
            self.get_logger().info(
                'auto_start=false → chờ service /tube_rack/run để bắt đầu.')
        self.get_logger().info(
            f'tube_rack_node sẵn sàng '
            f'(dry_run={self.get_parameter("dry_run").value}).')

    # ── callbacks ───────────────────────────────────────────────────────
    def _poses_cb(self, msg: PoseArray):
        self._det.update_poses(msg.poses)

    def _classes_cb(self, msg: String):
        try:
            self._det.update_classes(json.loads(msg.data))
        except Exception:
            pass

    def _run_srv_cb(self, request, response):
        with self._lock:
            if self._busy:
                response.success = False
                response.message = 'Đang bận — vui lòng chờ chu kỳ hiện tại kết thúc.'
                return response
        self._run_event.set()
        response.success = True
        response.message = 'Chu kỳ gắp ống mới đã được kích hoạt.'
        return response

    # ── status ──────────────────────────────────────────────────────────
    def _publish_status(self, step, extra=None):
        data = {
            'step': step,
            'slot_occupied': {str(k): v for k, v in self._slot_occupied.items()},
        }
        if extra:
            data.update(extra)
        msg = String()
        msg.data = json.dumps(data)
        self._status_pub.publish(msg)

    # ── worker ──────────────────────────────────────────────────────────
    def _worker_loop(self):
        while rclpy.ok():
            self._run_event.wait()
            self._run_event.clear()
            with self._lock:
                self._busy = True
            try:
                self._run_cycle()
            except Exception as exc:
                self.get_logger().error(f'Chu kỳ thất bại: {exc}')
                import traceback
                self.get_logger().error(traceback.format_exc())
            finally:
                with self._lock:
                    self._busy = False

    # ═════════════════════════════════════════════════════════════════════
    #  RACK GEOMETRY
    # ═════════════════════════════════════════════════════════════════════
    def _compute_slots(self):
        """Tính tọa độ từng slot trên giá nghiêng từ tham số."""
        s0x = self.get_parameter('slot0_x').value
        s0y = self.get_parameter('slot0_y').value
        s0z = self.get_parameter('slot0_z').value
        angle = math.radians(self.get_parameter('rack_angle_deg').value)
        spacing = self.get_parameter('slot_spacing').value
        n = int(self.get_parameter('num_slots').value)

        slots = []
        for k in range(n):
            sx = s0x + k * spacing * math.cos(angle)
            sy = s0y + k * spacing * math.sin(angle)
            # giá nghiêng: slot sau cao hơn
            sz = s0z
            slots.append((sx, sy, sz))
        return slots

    def _color_to_preferred_slots(self, color):
        """Trả list slot index ưu tiên cho màu, fallback tới mọi slot."""
        param_name = f'color_slot_map.{color}'
        try:
            vals = self.get_parameter(param_name).value
            if vals:
                return [int(v) for v in vals]
        except Exception:
            pass
        # unknown / không có mapping → mọi slot
        n = int(self.get_parameter('num_slots').value)
        return list(range(n))

    def _choose_slot(self, color):
        """Chọn slot trống đầu tiên theo ưu tiên màu. Trả index hoặc None."""
        preferred = self._color_to_preferred_slots(color)
        n = int(self.get_parameter('num_slots').value)
        # thử slot ưu tiên trước
        for si in preferred:
            if si < n and not self._slot_occupied.get(si, False):
                return si
        # fallback: bất kỳ slot trống
        for si in range(n):
            if not self._slot_occupied.get(si, False):
                return si
        return None

    # ═════════════════════════════════════════════════════════════════════
    #  MEDIAN SNAPSHOT (chống nhiễu 1 frame)
    # ═════════════════════════════════════════════════════════════════════
    def _collect_median_snapshot(self):
        """Thu thập nhiều frame, tính median (x,y,z,yaw) cho từng ống."""
        n_frames = int(self.get_parameter('median_frames').value)
        collect_s = float(self.get_parameter('median_collect_s').value)
        dt = collect_s / max(n_frames, 1)

        frames = []
        for _ in range(n_frames):
            snap = self._det.snapshot_once()
            if snap:
                frames.append(snap)
            time.sleep(dt)

        if not frames:
            return []

        # lấy frame cuối cùng làm tham chiếu số lượng ống + class
        ref = frames[-1]
        n_tubes = len(ref)
        if n_tubes == 0:
            return []

        # median cho mỗi ống
        result = []
        for ti in range(n_tubes):
            xs, ys, zs, yaws = [], [], [], []
            for frame in frames:
                if ti < len(frame):
                    xs.append(frame[ti]['x'])
                    ys.append(frame[ti]['y'])
                    zs.append(frame[ti]['z'])
                    yaws.append(frame[ti]['yaw'])
            if not xs:
                continue
            xs.sort(); ys.sort(); zs.sort(); yaws.sort()
            mid = len(xs) // 2
            result.append({
                'class': ref[ti]['class'],
                'x': xs[mid], 'y': ys[mid], 'z': zs[mid], 'yaw': yaws[mid],
            })
        return result

    # ═════════════════════════════════════════════════════════════════════
    #  MoveGroup primitive (tái sử dụng từ pick_place_moveit_node)
    # ═════════════════════════════════════════════════════════════════════
    def _wait_future(self, future, timeout=60.0):
        t0 = time.monotonic()
        while not future.done() and rclpy.ok():
            if time.monotonic() - t0 > timeout:
                self.get_logger().error(f'Future timeout sau {timeout:.0f}s.')
                return False
            time.sleep(0.01)
        return future.done()

    def move_to_joint_target(self, group_name, joint_names, targets, velocity_scale,
                             allowed_time=8.0):
        """Gửi MoveGroup goal (JointConstraint) → plan+execute qua move_group.
        Trả True nếu error_code ∈ OK_CODES."""
        if self.get_parameter('dry_run').value:
            self.get_logger().info(
                f'[DRY-RUN] move_to_joint_target({group_name}) → '
                f'{[f"{t:.3f}" for t in targets]}  v={velocity_scale}')
            return True

        if not self._move.wait_for_server(timeout_sec=5.0):
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

    # ── IK oracle ───────────────────────────────────────────────────────
    def _ik(self, x, y, z, pitch):
        joints, ok = self.bot.arm.set_ee_pose_components(
            x=float(x), y=float(y), z=float(z), pitch=float(pitch), execute=False)
        if not ok:
            self.get_logger().warn(f'IK fail x={x:.3f} y={y:.3f} z={z:.3f} pitch={pitch:.3f}.')
        return joints, ok

    def _solve_ik_ladder(self, x, y, z, pitch_ladder):
        """Thử IK từ pitch ưu tiên → relaxed. Trả (joints, ok, pitch_used)."""
        for p in pitch_ladder:
            joints, ok = self._ik(x, y, z, p)
            if ok:
                return joints, True, p
        return None, False, pitch_ladder[0]

    # ── gripper ─────────────────────────────────────────────────────────
    def _gripper(self, grasp: bool):
        if self.get_parameter('dry_run').value:
            self.get_logger().info(f'[DRY-RUN] gripper → {"GRASP" if grasp else "OPEN"}')
            return True

        if self.get_parameter('use_gripper_bridge').value:
            tgt = GRASP_FINGER if grasp else RELEASE_FINGER
            return self.move_to_joint_target(
                GRIPPER_GROUP, [FINGER_JOINT], [tgt], 0.2, allowed_time=4.0)

        cmd = JointSingleCommand(
            name='gripper', cmd=float(GRIP_PWM if grasp else OPEN_PWM))
        self.bot.core.pub_single.publish(cmd)
        time.sleep(2.0)
        return True

    # ── home ────────────────────────────────────────────────────────────
    def _go_home(self):
        home = list(self.get_parameter('home_joints').value)
        return self.move_to_joint_target(
            ARM_GROUP, ARM_JOINTS, home,
            self.get_parameter('velocity_scale_cruise').value)

    # ── abort ───────────────────────────────────────────────────────────
    def _abort(self, reason=''):
        self.get_logger().warn(f'ABORT — mở gripper & về home. {reason}')
        self._gripper(grasp=False)
        self._go_home()
        return False

    # ── planning scene ──────────────────────────────────────────────────
    def _ensure_scene(self):
        if self._scene_done:
            return

        objects = []

        # Table collision box
        if self.get_parameter('add_table_collision').value:
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dim = [
                float(self.get_parameter('table_size_x').value),
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
            objects.append(co)

        # Rack collision box
        if self.get_parameter('add_rack_collision').value:
            box_r = SolidPrimitive()
            box_r.type = SolidPrimitive.BOX
            box_r.dim = [
                float(self.get_parameter('rack_box_size_x').value),
                float(self.get_parameter('rack_box_size_y').value),
                float(self.get_parameter('rack_box_size_z').value)]
            pose_r = Pose()
            pose_r.position.x = float(self.get_parameter('rack_box_x').value)
            pose_r.position.y = float(self.get_parameter('rack_box_y').value)
            pose_r.position.z = float(self.get_parameter('rack_box_z').value)
            pose_r.orientation.w = 1.0
            co_r = CollisionObject()
            co_r.header.frame_id = 'world'
            co_r.id = 'rack'
            co_r.operation = CollisionObject.ADD
            co_r.primitives = [box_r]
            co_r.primitive_poses = [pose_r]
            objects.append(co_r)

        if not objects:
            self._scene_done = True
            return

        if not self._apply_scene.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '/apply_planning_scene chưa sẵn sàng — bỏ qua collision boxes.')
            self._scene_done = True
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects

        req = ApplyPlanningScene.Request()
        req.scene = scene
        fut = self._apply_scene.call_async(req)
        self._wait_future(fut, timeout=5.0)
        self._scene_done = True
        names = [o.id for o in objects]
        self.get_logger().info(f'Đã thêm collision boxes: {names}')

    # ═════════════════════════════════════════════════════════════════════
    #  CHU KỲ CHÍNH: quét → gắp tuần tự → cắm giá
    # ═════════════════════════════════════════════════════════════════════
    def _run_cycle(self):
        self.get_logger().info('═══ BẮT ĐẦU CHU KỲ GẮP ỐNG NGHIỆM → GIÁ ═══')
        self._publish_status('START')

        # ── chuẩn bị ────────────────────────────────────────────────────
        self._ensure_scene()
        self._go_home()
        self._gripper(grasp=False)

        # ── fake_tubes mode ─────────────────────────────────────────────
        fake_json = self.get_parameter('fake_tubes').value
        fake_tubes = []
        if fake_json and fake_json != '[]':
            try:
                fake_tubes = json.loads(fake_json)
            except Exception:
                self.get_logger().warn('fake_tubes JSON không hợp lệ — bỏ qua.')

        # ── SCAN: thu thập detection ────────────────────────────────────
        if fake_tubes:
            tubes = []
            for ft in fake_tubes:
                tubes.append({
                    'class': str(ft.get('class', 'unknown')).lower(),
                    'x': float(ft['x']), 'y': float(ft['y']),
                    'z': float(ft['z']), 'yaw': float(ft.get('yaw', 0.0)),
                })
            self.get_logger().info(f'Dùng fake_tubes: {len(tubes)} ống.')
        else:
            wait_s = float(self.get_parameter('detection_wait_s').value)
            self.get_logger().info(f'Chờ detection tối đa {wait_s:.0f}s …')
            t0 = time.monotonic()
            tubes = []
            while time.monotonic() - t0 < wait_s:
                tubes = self._collect_median_snapshot()
                if tubes:
                    break
                time.sleep(0.5)

        if not tubes:
            self.get_logger().warn('Không phát hiện ống nghiệm nào — kết thúc.')
            self._publish_status('NO_TUBES')
            self._go_home()
            return

        # ── Lọc ống đã nằm trong vùng giá ──────────────────────────────
        slots = self._compute_slots()
        filter_r = float(self.get_parameter('rack_filter_xy_m').value)
        filtered = []
        for t in tubes:
            in_rack = False
            for sx, sy, _sz in slots:
                dx = t['x'] - sx
                dy = t['y'] - sy
                if math.hypot(dx, dy) < filter_r:
                    in_rack = True
                    break
            if not in_rack:
                filtered.append(t)
            else:
                self.get_logger().info(
                    f'Bỏ qua ống {t["class"]} tại ({t["x"]:.3f},{t["y"]:.3f}) '
                    f'— nằm trong vùng giá.')

        tubes = filtered
        self.get_logger().info(f'Số ống cần gắp: {len(tubes)}')
        for i, t in enumerate(tubes):
            self.get_logger().info(
                f'  [{i}] {t["class"]:>8}  x={t["x"]:.3f} y={t["y"]:.3f} '
                f'z={t["z"]:.3f} yaw={math.degrees(t["yaw"]):.1f}°')

        # ── Reset slot occupancy (scan lại) ─────────────────────────────
        self._slot_occupied = {}

        # ── Gắp từng ống ────────────────────────────────────────────────
        results = []
        grasp_ladder = list(self.get_parameter('grasp_pitch_ladder').value)
        place_ladder = list(self.get_parameter('place_pitch_ladder').value)
        delta = float(self.get_parameter('approach_delta').value)
        tube_len = float(self.get_parameter('tube_length').value)
        insert_depth = float(self.get_parameter('insert_depth').value)
        v_cruise = float(self.get_parameter('velocity_scale_cruise').value)
        v_descend = float(self.get_parameter('velocity_scale_descend').value)
        v_insert = float(self.get_parameter('velocity_scale_insert').value)
        v_lift = float(self.get_parameter('velocity_scale_lift').value)

        for ti, t in enumerate(tubes):
            self.get_logger().info(
                f'\n──── ỐNG [{ti+1}/{len(tubes)}] {t["class"]} '
                f'({t["x"]:.3f},{t["y"]:.3f},{t["z"]:.3f}) ────')
            self._publish_status('PICKING', {'tube_index': ti, 'tube_class': t['class']})

            ok = self._pick_and_place_one(
                t, slots, grasp_ladder, place_ladder,
                delta, tube_len, insert_depth,
                v_cruise, v_descend, v_insert, v_lift)
            results.append({'class': t['class'], 'ok': ok})
            if not ok:
                self.get_logger().warn(f'Ống [{ti+1}] thất bại — sang ống kế.')

        # ── Tổng kết ───────────────────────────────────────────────────
        n_ok = sum(1 for r in results if r['ok'])
        self.get_logger().info(
            f'═══ KẾT THÚC: {n_ok}/{len(results)} ống thành công ═══')
        for r in results:
            mark = '✓' if r['ok'] else '✗'
            self.get_logger().info(f'  {mark} {r["class"]}')
        self._publish_status('DONE', {'total': len(results), 'success': n_ok})
        self._go_home()

    # ═════════════════════════════════════════════════════════════════════
    #  GẮP 1 ỐNG → CẮM 1 SLOT
    # ═════════════════════════════════════════════════════════════════════
    def _pick_and_place_one(self, tube, slots, grasp_ladder, place_ladder,
                            delta, tube_len, insert_depth,
                            v_cruise, v_descend, v_insert, v_lift):
        tx, ty, tz, yaw = tube['x'], tube['y'], tube['z'], tube['yaw']
        color = tube['class']

        # ── Toán gắp (sort_tubes_by_color:305-327) ──────────────────────
        z_approach = max(tz + delta, 0.12)
        z_grasp = max(tz, 0.02)

        j_app, ok1, pitch_used = self._solve_ik_ladder(tx, ty, z_approach, grasp_ladder)
        j_grasp, ok2, _ = self._solve_ik_ladder(tx, ty, z_grasp, [pitch_used] + grasp_ladder)

        if not ok1 or not ok2:
            self.get_logger().error(
                f'IK fail tại ({tx:.3f},{ty:.3f}) — bỏ qua ống.')
            return self._abort('IK fail GRASP')

        # Wrist rotation: ngón vuông góc trục ống
        waist_angle = j_grasp[0]
        wrist_rot = yaw - waist_angle
        wrist_rot = math.atan2(math.sin(wrist_rot), math.cos(wrist_rot))

        j_app = list(j_app)
        j_grasp = list(j_grasp)
        j_app[4] = wrist_rot
        j_grasp[4] = wrist_rot

        self.get_logger().info(
            f'  IK ok: pitch={pitch_used:.2f} wrist_rot={math.degrees(wrist_rot):.1f}°')

        # ── Chọn slot ──────────────────────────────────────────────────
        slot_idx = self._choose_slot(color)
        if slot_idx is None:
            self.get_logger().warn(f'Hết slot trống cho màu {color} — bỏ qua.')
            return self._abort('Hết slot')
        sx, sy, sz = slots[slot_idx]
        self.get_logger().info(f'  → Slot {slot_idx} tại ({sx:.3f},{sy:.3f},{sz:.3f})')

        # ── Tính z_hover / z_insert cho việc đặt ống ────────────────────
        # Ống kẹp giữa thân → khi pitch=0 (thẳng đứng), đáy ống cách EE = tube_len/2
        z_hover = sz + tube_len / 2.0 + 0.03   # margin 3cm trên đỉnh slot
        z_insert_target = z_hover - insert_depth

        # IK cho slot (dùng place_pitch_ladder)
        j_hover, ok_h, pp_used = self._solve_ik_ladder(sx, sy, z_hover, place_ladder)
        j_insert, ok_i, _ = self._solve_ik_ladder(
            sx, sy, z_insert_target, [pp_used] + place_ladder)

        if not ok_h or not ok_i:
            self.get_logger().warn(
                f'IK fail tại slot {slot_idx} ({sx:.3f},{sy:.3f}) — bỏ qua.')
            return self._abort('IK fail PLACE')

        j_hover = list(j_hover)
        j_insert = list(j_insert)
        # Giữ wrist_rotate=0 cho place (ống thẳng đứng, không cần xoay)
        j_hover[4] = 0.0
        j_insert[4] = 0.0

        self.get_logger().info(
            f'  Place IK ok: place_pitch={pp_used:.2f} '
            f'z_hover={z_hover:.3f} z_insert={z_insert_target:.3f}')

        # ── Transport clearance: ống treo dọc dưới EE ──────────────────
        # Mọi hover phải cao hơn tube_len/2 + 0.03 (để đáy ống không chạm bàn/giá)
        z_transport_min = tube_len / 2.0 + 0.03

        # ═══ STATE MACHINE ═══════════════════════════════════════════════
        # 1. APPROACH — di chuyển tới trên ống
        self.get_logger().info('  [1/9] APPROACH (trên ống)')
        self._gripper(grasp=False)
        if not self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_app, v_cruise):
            return self._abort('APPROACH fail')

        # 2. DESCEND — hạ xuống vị trí kẹp
        self.get_logger().info('  [2/9] DESCEND (kẹp)')
        if not self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_grasp, v_descend):
            return self._abort('DESCEND fail')

        # 3. GRASP — kẹp ống
        self.get_logger().info('  [3/9] GRASP')
        if not self._gripper(grasp=True):
            return self._abort('GRASP fail')
        time.sleep(0.5)  # chờ stall

        # 4. LIFT — nhấc lên approach height
        self.get_logger().info('  [4/9] LIFT')
        if not self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_app, v_lift):
            return self._abort('LIFT fail')

        # 5. TRANSPORT → PLACE_HOVER — di chuyển tới trên slot
        self.get_logger().info(f'  [5/9] TRANSPORT → slot {slot_idx}')
        # IK cho transport: hover trên slot, EE đủ cao cho ống treo dọc
        z_transport = max(z_hover, z_transport_min + sz)
        j_trans, ok_t, pp_t = self._solve_ik_ladder(
            sx, sy, z_transport, place_ladder)
        if ok_t:
            j_trans = list(j_trans)
            j_trans[4] = 0.0
            if not self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_trans, v_cruise):
                return self._abort('TRANSPORT fail')
        else:
            # fallback: dùng j_hover trực tiếp
            if not self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_hover, v_cruise):
                return self._abort('TRANSPORT fail')

        # 6. PLACE_HOVER — hạ xuống hover position
        self.get_logger().info('  [6/9] PLACE_HOVER')
        if not self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_hover, v_descend):
            return self._abort('PLACE_HOVER fail')

        # 7. INSERT — hạ chậm xuống slot
        self.get_logger().info('  [7/9] INSERT')
        if not self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_insert, v_insert):
            return self._abort('INSERT fail')

        # 8. RELEASE — nhả ống
        self.get_logger().info('  [8/9] RELEASE')
        if not self._gripper(grasp=False):
            return self._abort('RELEASE fail')
        time.sleep(0.3)

        # 9. RÚT + RETREAT — nhấc THẲNG ĐỨNG qua đỉnh ống đã cắm, rồi rút ngang
        self.get_logger().info('  [9/9] RETRACT (thẳng đứng → ngang → home)')
        # Nhấc thẳng đứng: z >= slot_top + tube_length + margin
        z_clear = sz + tube_len + 0.02
        j_clear, ok_c, _ = self._solve_ik_ladder(sx, sy, z_clear, place_ladder)
        if ok_c:
            j_clear = list(j_clear)
            j_clear[4] = 0.0
            self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_clear, v_lift)

        # Retreat ngang (lùi về phía base) — chỉ SAU khi đã nhấc qua đỉnh ống
        norm = math.hypot(sx, sy) or 1.0
        rx = sx - 0.10 * (sx / norm)
        ry = sy - 0.10 * (sy / norm)
        j_retreat, ok_r, _ = self._solve_ik_ladder(rx, ry, z_clear, place_ladder)
        if ok_r:
            j_retreat = list(j_retreat)
            j_retreat[4] = 0.0
            self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, j_retreat, v_cruise)

        # HOME
        self._go_home()

        # Đánh dấu slot đã có ống
        self._slot_occupied[slot_idx] = True
        self.get_logger().info(
            f'  ✓ Ống {color} → slot {slot_idx} thành công!')
        self._publish_status('PLACED', {
            'tube_class': color, 'slot': slot_idx})
        return True


# ═══════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    global_node = create_interbotix_global_node('tube_rack_control')
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL, robot_name=ROBOT_NAME, node=global_node)
    robot_startup(global_node)

    # Gripper PWM mode: cần cho fallback; bridge cũng xài PWM effort
    try:
        bot.core.robot_set_operating_modes('single', 'gripper', 'pwm')
    except Exception as exc:
        rclpy.logging.get_logger('tube_rack').warn(
            f'Không set gripper pwm: {exc}')

    node = TubeRackNode(bot)
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
