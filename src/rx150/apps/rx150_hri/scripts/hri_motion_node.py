#!/usr/bin/env python3
"""
hri_motion_node — EXECUTOR chuyển động của chức năng HRI (rx150_hri).

Nhận lệnh "tới vị trí + kẹp" từ hri_task_node (hoặc pub thủ công) và thực thi qua
MoveIt (action 'move_action'). KHÔNG biết gì về detection/lựa chọn.
TẤT CẢ chuyển động arm đi qua move_group; KHÔNG publish trực tiếp /rx150/commands/*
(sẽ đánh fuzzy_node). SDK `bot` CHỈ dùng làm IK-oracle
(bot.arm.set_ee_pose_components(execute=False) — arm.py:542-547 chỉ publish khi execute).

Topic vào (xem hri_common.py):
  /hri/cmd_pose      (PoseStamped — frame rx150/base_link; pitch = 2·atan2(qy, qw))
  /hri/cmd_gripper   (Bool — True = kẹp, False = nhả)
  /hri/cmd_home      (Empty — về home_joints)
  /hri/set_vel_scale (Float32 — velocity scale MoveIt cho cmd_pose kế tiếp, 0.05..1.0)
Topic ra:
  /hri/status (String — "READY #0" / "POSE_DONE #3" / "POSE_FAILED #3" / "GRIP_DONE #4"
               / "HOME_DONE #5" / "REJECTED #6" / heartbeat "IDLE #<seq>" mỗi 2 s khi rảnh)

Đồng bộ: seq tăng đơn điệu cho MỖI lệnh hoàn tất/bị từ chối → hri_task gửi lệnh rồi
chờ status có seq > seq đã thấy. Primitive motion port nguyên từ
rx150_pick_place/scripts/pick_place_moveit_node.py (bản đã chạy ổn).
"""
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Bool, Empty, String, Float32
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node, robot_shutdown, robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand

import hri_common

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
OK_CODES = (1, -4)      # 1=SUCCESS, -4=CONTROL_FAILED (bridge báo tolerance nhưng đã xong)
QUEUE_DEPTH = 4         # số lệnh chờ tối đa (đầy → REJECTED)


class HriMotionNode(Node):
    def __init__(self, bot):
        super().__init__('hri_motion')
        self.bot = bot

        # ---------------- parameters (default khớp config/hri_params.yaml) ----------------
        self.declare_parameter('base_frame', 'rx150/base_link')
        self.declare_parameter('velocity_scale_cruise', 0.3)
        self.declare_parameter('velocity_scale_delicate', 0.1)
        self.declare_parameter('home_joints', [0.0, -1.80, 1.55, 0.8, 0.0])
        self.declare_parameter('use_gripper_bridge', True)
        self.declare_parameter('add_table_collision', True)
        self.declare_parameter('table_x', 0.30)
        self.declare_parameter('table_y', 0.0)
        self.declare_parameter('table_z', -0.02)
        self.declare_parameter('table_size_x', 0.80)
        self.declare_parameter('table_size_y', 0.80)
        self.declare_parameter('table_size_z', 0.10)
        self.declare_parameter('allowed_time_s', 5.0)   # thời gian plan+execute mỗi lệnh
        self.declare_parameter('cmd_queue_depth', QUEUE_DEPTH)

        self.base_frame = self.get_parameter('base_frame').value

        # ---------------- state ----------------
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._seq = 0                 # counter đơn điệu mỗi lệnh hoàn tất/bị từ chối
        self._vel_scale = float(self.get_parameter('velocity_scale_cruise').value)
        self._scene_done = False
        self._cmds = queue.Queue(maxsize=int(self.get_parameter('cmd_queue_depth').value))

        # ---------------- MoveGroup action client (arm + gripper cùng 'move_action') ----------------
        self._move = ActionClient(self, MoveGroup, 'move_action', callback_group=self._cb)
        self._apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self._cb)

        # ---------------- interface lệnh ----------------
        self.status_pub = self.create_publisher(String, '/hri/status', 10)
        self.create_subscription(PoseStamped, '/hri/cmd_pose', self._pose_cb, 10,
                                 callback_group=self._cb)
        self.create_subscription(Bool, '/hri/cmd_gripper', self._grip_cb, 10,
                                 callback_group=self._cb)
        self.create_subscription(Empty, '/hri/cmd_home', self._home_cb, 10,
                                 callback_group=self._cb)
        self.create_subscription(Float32, '/hri/set_vel_scale', self._vel_cb, 10,
                                 callback_group=self._cb)

        # ---------------- worker: thực thi lệnh tuần tự (không block executor) ----------------
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info('hri_motion sẵn sàng nhận lệnh /hri/cmd_*.')

    # ---------------- callbacks: chỉ xếp lệnh, KHÔNG block ----------------
    def _pose_cb(self, msg: PoseStamped):
        if msg.header.frame_id != self.base_frame:
            self._reject(f'cmd_pose sai frame "{msg.header.frame_id}" (cần {self.base_frame})')
            return
        p = msg.pose.position
        q = msg.pose.orientation
        pitch = hri_common.quat_to_pitch(q.x, q.y, q.z, q.w)
        self._enqueue(('pose', float(p.x), float(p.y), float(p.z), pitch))

    def _grip_cb(self, msg: Bool):
        self._enqueue(('grip', bool(msg.data)))

    def _home_cb(self, msg: Empty):  # noqa: ARG002
        self._enqueue(('home',))

    def _vel_cb(self, msg: Float32):
        # Áp dụng NGAY (không xếp hàng). An toàn về thứ tự: hri_task chỉ đổi vel
        # SAU khi nhận status lệnh trước → không đè vel của lệnh đang chờ.
        self._vel_scale = max(0.05, min(1.0, float(msg.data)))
        self.get_logger().info(f'vel_scale → {self._vel_scale:.2f}', throttle_duration_sec=1.0)

    def _enqueue(self, cmd):
        try:
            self._cmds.put_nowait(cmd)
        except queue.Full:
            self._reject('queue đầy')

    def _reject(self, reason):
        with self._lock:
            self._seq += 1
            n = self._seq
        self.get_logger().warn(f'REJECTED: {reason}')
        self.status_pub.publish(String(data=hri_common.build_status(hri_common.REJECTED, n)))

    def _publish_status(self, kind):
        self.status_pub.publish(String(data=hri_common.build_status(kind, self._seq)))

    # ---------------- worker loop ----------------
    def _worker_loop(self):
        self._ensure_scene()
        with self._lock:
            self._seq = 0
        self._publish_status(hri_common.READY)
        while rclpy.ok():
            try:
                cmd = self._cmds.get(timeout=2.0)
            except queue.Empty:
                self._publish_status(hri_common.IDLE)   # heartbeat, không tăng seq
                continue
            if cmd is None:
                break
            with self._lock:
                self._seq += 1
            kind, ok = self._execute(cmd)
            self._publish_status(kind if ok else kind + '_FAILED')

    def _execute(self, cmd):
        """Thực thi 1 lệnh → (kind_base, ok). kind_base như 'POSE' → 'POSE_DONE'/'POSE_FAILED'."""
        try:
            if cmd[0] == 'pose':
                _, x, y, z, pitch = cmd
                return 'POSE', self._exec_pose(x, y, z, pitch)
            if cmd[0] == 'grip':
                return 'GRIP', self._gripper(grasp=cmd[1])
            if cmd[0] == 'home':
                return 'HOME', self._go_home()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Lệnh {cmd[0]} exception: {exc}')
        return 'POSE', False

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

    # ---------------- lệnh pose: IK oracle → MoveIt ----------------
    def _exec_pose(self, x, y, z, pitch):
        joints, ok = self._ik(x, y, z, pitch)
        if not ok:
            return False
        at = float(self.get_parameter('allowed_time_s').value)
        return self.move_to_joint_target(ARM_GROUP, ARM_JOINTS, list(joints),
                                         self._vel_scale, allowed_time=at)

    # ---------------- IK oracle (SDK, execute=False → KHÔNG publish) ----------------
    def _ik(self, x, y, z, pitch):
        joints, ok = self.bot.arm.set_ee_pose_components(
            x=float(x), y=float(y), z=float(z), pitch=float(pitch), execute=False)
        if not ok:
            self.get_logger().warn(f'IK fail x={x:.3f} y={y:.3f} z={z:.3f} pitch={pitch:.3f}.')
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
        # fallback raw PWM: JointSingleCommand → xs_sdk
        cmd = JointSingleCommand(name='gripper', cmd=float(GRIP_PWM if grasp else OPEN_PWM))
        self.bot.core.pub_single.publish(cmd)
        time.sleep(2.0)
        return True

    # ---------------- HOME ----------------
    def _go_home(self):
        return self.move_to_joint_target(
            ARM_GROUP, ARM_JOINTS, list(self.get_parameter('home_joints').value),
            self.get_parameter('velocity_scale_cruise').value)

    # ---------------- planning scene: box bàn (ADD 1 lần khi khởi động) ----------------
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
        # shape_msgs/SolidPrimitive: field là `dimensions`, KHÔNG phải `dim`.
        # Message ROS 2 dùng __slots__ nên gán sai tên là AttributeError ngay ⇒
        # _ensure_scene() throw ở dòng đầu của _worker_loop và worker chết luôn
        # (node không bao giờ phát READY, mọi /hri/cmd_* rơi vào im lặng).
        box.dimensions = [float(self.get_parameter('table_size_x').value),
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


def main(args=None):
    rclpy.init(args=args)
    global_node = create_interbotix_global_node('hri_motion_control')
    bot = InterbotixManipulatorXS(robot_model=ROBOT_MODEL, robot_name=ROBOT_NAME, node=global_node)
    robot_startup(global_node)
    # Gripper PWM mode: cần cho fallback raw PWM; gripper_bridge cũng xài PWM effort.
    try:
        bot.core.robot_set_operating_modes('single', 'gripper', 'pwm')
    except Exception as exc:  # noqa: BLE001
        rclpy.logging.get_logger('hri_motion').warn(f'Không set gripper pwm: {exc}')

    node = HriMotionNode(bot)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._cmds.put(None)   # dừng worker
        node.destroy_node()
        robot_shutdown(global_node)
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
