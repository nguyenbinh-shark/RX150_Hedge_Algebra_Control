#!/usr/bin/env python3
"""
hri_motion_node — EXECUTOR chuyển động của chức năng HRI (rx150_hri).

Nhận lệnh "tới vị trí + kẹp" từ hri_task_node (hoặc pub thủ công) và thực thi qua
rx150_modules. KHÔNG biết gì về detection/lựa chọn.

GIAO DIỆN LỆNH KHÔNG ĐỔI (xem hri_common.py) — hri_task_node không phải sửa gì:
  Vào:  /hri/cmd_pose      (PoseStamped — frame base_frame; pitch = 2·atan2(qy, qw))
        /hri/cmd_gripper   (Bool — True = kẹp, False = nhả)
        /hri/cmd_home      (Empty — về home_joints)
        /hri/set_vel_scale (Float32 — velocity scale cho các cmd_pose SAU đó, 0.05..1.0)
  Ra:   /hri/status (String — "READY #0" / "POSE_DONE #3" / "POSE_FAILED #3" /
                     "GRIP_DONE #4" / "HOME_DONE #5" / "REJECTED #6" /
                     heartbeat "IDLE #<seq>" mỗi 2 s khi rảnh)

Đồng bộ: seq tăng đơn điệu cho MỖI lệnh hoàn tất/bị từ chối → hri_task gửi lệnh rồi
chờ status có seq > seq đã thấy.

BẢN NÀY BỎ ~250 DÒNG PRIMITIVE TỰ VIẾT, thay bằng rx150_modules (tầng Application
Support). Ba thứ đổi theo, đều là đổi tốt hơn:

  1. IK: bỏ IK-oracle qua InterbotixManipulatorXS.set_ee_pose_components(execute=False).
     SDK gọi mr.IKinSpace — Newton lặp với 3 seed cố định nên FAIL NGẪU NHIÊN ở các
     điểm biên. rx150_modules.kinematics là IK giải tích, tất định, có pytest phủ.
     Hệ quả phụ: node không còn cần SDK bot ⇒ không còn InterbotixManipulatorXS,
     robot_startup/robot_shutdown, và không còn nguy cơ hai xs_sdk trên một bus.

  2. Chấp hành: MoveItExecutor kiểm tra ĐÃ TỚI ĐÍCH THẬT (verify_reached) chứ không
     chỉ tin error_code, và có retry.

  3. Gripper: Gripper.close() XÁC NHẬN có vật trong ngón (đo left_finger) — kẹp
     trượt giờ trả GRIP_FAILED thay vì GRIP_DONE. Đặt verify_grasp:=false để về
     hành vi cũ khi diễn thử không có vật.
"""
import queue
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, String, Float32
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import OperatingModes

from rx150_modules.params import build_stack, declare_common, read_common, table_object

import hri_common

# Tham số riêng của node này (phần chung nằm trong rx150_modules.params.COMMON_DEFAULTS).
NODE_DEFAULTS = {
    'allowed_time_s': 5.0,       # s — plan+execute mỗi lệnh pose
    'cmd_queue_depth': 4,        # lệnh chờ tối đa; đầy → REJECTED
    'verify_grasp': True,        # False = coi kẹp là xong khi goal xong (hành vi cũ)
    # hri có tư thế home riêng, khai báo bằng home_joints trong hri_params.yaml.
    # rx150_modules mặc định lấy home từ home_xyz_pitch (0.18, 0, 0.18) và resolve_home()
    # sẽ GHI ĐÈ home_joints — cờ này để tắt đường đó. Bật lên nếu muốn khai home của hri
    # bằng toạ độ thay vì 5 số radian.
    'use_home_xyz_pitch': False,
    'idle_heartbeat_s': 2.0,     # chu kỳ phát IDLE khi rảnh
    # ---- phần cứng ----
    'commands_topic': '/rx150/commands/joint_single',
    'operating_modes_service': '/rx150/set_operating_modes',
    'set_gripper_pwm_mode': True,
}

class HriMotionNode(Node):
    def __init__(self):
        super().__init__('hri_motion')

        declare_common(self, NODE_DEFAULTS)
        self.cfg = read_common(self)
        for name in NODE_DEFAULTS:
            setattr(self.cfg, name, self.get_parameter(name).value)
        # Rỗng ⇒ resolve_home() giữ nguyên home_joints thô của hri_params.yaml.
        if not self.cfg.use_home_xyz_pitch:
            self.cfg.home_xyz_pitch = []

        self._cb = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(JointSingleCommand,
                                              self.cfg.commands_topic, 10)
        self.stack = build_stack(self, self.cfg, callback_group=self._cb,
                                 status_topic='~/task_status', publisher=self._cmd_pub)

        # ---------------- state ----------------
        self._lock = threading.Lock()
        self._seq = 0                 # counter đơn điệu mỗi lệnh hoàn tất/bị từ chối
        self._vel_scale = float(self.cfg.velocity_scale_cruise)
        self._scene_done = False
        self._cmds = queue.Queue(maxsize=int(self.cfg.cmd_queue_depth))

        # ---------------- interface lệnh (KHÔNG đổi) ----------------
        self.status_pub = self.create_publisher(String, '/hri/status', 10)
        self.create_subscription(PoseStamped, '/hri/cmd_pose', self._pose_cb, 10,
                                 callback_group=self._cb)
        self.create_subscription(Bool, '/hri/cmd_gripper', self._grip_cb, 10,
                                 callback_group=self._cb)
        self.create_subscription(Empty, '/hri/cmd_home', self._home_cb, 10,
                                 callback_group=self._cb)
        self.create_subscription(Float32, '/hri/set_vel_scale', self._vel_cb, 10,
                                 callback_group=self._cb)

        self._setup_gripper_mode()

        # ---------------- worker: thực thi tuần tự, không block executor ----------------
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info('hri_motion sẵn sàng nhận lệnh /hri/cmd_*.')

    # ---------------- phần cứng ----------------
    def _setup_gripper_mode(self):
        """Gripper phải ở PWM mode: cả đường bridge lẫn đường PWM trực tiếp đều cần."""
        if not self.cfg.set_gripper_pwm_mode or self.cfg.dry_run:
            return
        client = self.create_client(OperatingModes, self.cfg.operating_modes_service,
                                    callback_group=self._cb)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f'{self.cfg.operating_modes_service} chưa sẵn sàng — gripper có thể '
                'không ở PWM mode (xs_sdk chưa chạy?).')
            return
        client.call_async(OperatingModes.Request(cmd_type='single', name='gripper',
                                                 mode='pwm'))

    # ---------------- callbacks: chỉ xếp lệnh, KHÔNG block ----------------
    def _pose_cb(self, msg: PoseStamped):
        if msg.header.frame_id != self.cfg.base_frame:
            self._reject(f'cmd_pose sai frame "{msg.header.frame_id}" '
                         f'(cần {self.cfg.base_frame})')
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
        self.get_logger().info(f'vel_scale → {self._vel_scale:.2f}',
                               throttle_duration_sec=1.0)

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
        self.stack.executor.wait_ready(timeout=15.0)
        self._ensure_scene()
        with self._lock:
            self._seq = 0
        self._publish_status(hri_common.READY)
        heartbeat = float(self.cfg.idle_heartbeat_s)
        while rclpy.ok():
            try:
                cmd = self._cmds.get(timeout=heartbeat)
            except queue.Empty:
                self._publish_status(hri_common.IDLE)   # heartbeat, không tăng seq
                continue
            if cmd is None:
                break
            with self._lock:
                self._seq += 1
            kind, ok = self._execute(cmd)
            self._publish_status(kind + '_DONE' if ok else kind + '_FAILED')

    def _execute(self, cmd):
        """Thực thi 1 lệnh → (kind_base, ok). 'POSE' → 'POSE_DONE'/'POSE_FAILED'."""
        try:
            if cmd[0] == 'pose':
                _, x, y, z, pitch = cmd
                return 'POSE', self._exec_pose(x, y, z, pitch)
            if cmd[0] == 'grip':
                return 'GRIP', self._exec_gripper(grasp=cmd[1])
            if cmd[0] == 'home':
                return 'HOME', self._exec_home()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Lệnh {cmd[0]} exception: {exc}')
        return 'POSE', False

    # ---------------- các lệnh ----------------
    def _exec_pose(self, x, y, z, pitch):
        joints = self.stack.kin.ik(x, y, z, pitch)
        if joints is None:
            self.get_logger().warn(
                f'IK fail ({x:.3f},{y:.3f},{z:.3f}) pitch={pitch:.3f} — '
                f'{self.stack.kin.reach_report(x, y, z, pitch)}')
            return False
        return self.stack.executor.move_joints(
            joints, self._vel_scale, label='POSE',
            allowed_time=float(self.cfg.allowed_time_s))

    def _exec_gripper(self, grasp: bool):
        verify = bool(self.cfg.verify_grasp)
        if grasp:
            return self.stack.gripper.close(verify=verify)
        return self.stack.gripper.open(verify=verify)

    def _exec_home(self):
        return self.stack.executor.move_joints(
            list(self.cfg.home_joints), float(self.cfg.velocity_scale_cruise),
            label='HOME', allowed_time=float(self.cfg.allowed_time_s))

    # ---------------- planning scene: box bàn (ADD 1 lần khi khởi động) ----------------
    def _ensure_scene(self):
        if self._scene_done or not self.cfg.add_table_collision:
            self._scene_done = True
            return
        self.stack.scene.apply([table_object(self.stack.scene, self.cfg)],
                               label='vật cản tĩnh')
        self._scene_done = True


def main(args=None):
    rclpy.init(args=args)
    node = HriMotionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._cmds.put(None)   # dừng worker
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
