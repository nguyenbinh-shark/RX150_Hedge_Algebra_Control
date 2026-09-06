#!/usr/bin/env python3
"""
hri_task_node — CHỨC NĂNG HRI: logic đọc nhận diện + xử lý + lựa chọn + định trình tự.

Node KHÔNG biết MoveIt/robot — mọi chuyển động gửi qua executor hri_motion bằng topic
(xem hri_common.py): /hri/cmd_pose, /hri/cmd_gripper, /hri/cmd_home, /hri/set_vel_scale,
đồng bộ bằng /hri/status ("<KIND> #<seq>" — seq tăng đơn điệu mỗi lệnh hoàn tất).

Phát triển TỪNG BƯỚC (param `mode`):
  fixed  (BƯỚC 1 — đã làm) : gắp tại điểm cố định (pick_*) → nhả tại điểm cố định (place_*).
                             Không cần camera.
  camera (BƯỚC 2 — tới)    : đọc /yolo/detected_tubes (PoseArray base frame từ
                             rx150_perception) → chọn vật ổn định gần base nhất → gắp,
                             nhả tại place_*.
  (B3: gesture chọn vật qua /hand_gesture/*; B4: handover + RViz markers.)

Kích hoạt chu kỳ: auto_start=true → chạy `cycles` lần ngay khi executor READY;
auto_start=false → mỗi chu kỳ chờ topic /hri/start (std_msgs/Empty).
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, String, Float32

import hri_common


class HriTaskNode(Node):
    def __init__(self):
        super().__init__('hri_task')

        # ---------------- parameters (default khớp config/hri_params.yaml) ----------------
        self.declare_parameter('mode', 'fixed')
        self.declare_parameter('base_frame', 'rx150/base_link')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('cycles', 1)            # -1 = lặp vô hạn
        # ---- BƯỚC 1: điểm gắp / điểm nhả cố định (TUNE cho bàn thật) ----
        self.declare_parameter('pick_x', 0.25)
        self.declare_parameter('pick_y', 0.10)
        self.declare_parameter('pick_z', 0.05)
        self.declare_parameter('pick_pitch', 0.5)
        self.declare_parameter('place_x', 0.30)
        self.declare_parameter('place_y', -0.10)
        self.declare_parameter('place_z', 0.05)
        self.declare_parameter('place_pitch', 0.5)
        # ---- hình học lệnh ----
        self.declare_parameter('approach_delta', 0.05)      # m — chênh z khi tiếp cận/rút
        self.declare_parameter('finger_grasp_offset', 0.02) # m — hạ thêm khi kẹp
        self.declare_parameter('retreat_m', 0.10)           # m — lùi ngang sau khi nhả
        self.declare_parameter('velocity_scale_cruise', 0.3)
        self.declare_parameter('velocity_scale_delicate', 0.1)
        self.declare_parameter('status_timeout_s', 45.0)    # chờ status từng lệnh

        self.mode = str(self.get_parameter('mode').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.vcruise = float(self.get_parameter('velocity_scale_cruise').value)
        self.vdel = float(self.get_parameter('velocity_scale_delicate').value)
        self.status_timeout = float(self.get_parameter('status_timeout_s').value)

        # ---------------- state ----------------
        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._last_kind = None       # kind status mới nhất từ executor
        self._last_seq = -1          # seq status mới nhất (-1 = chưa nhận gì)
        self._start_event = threading.Event()

        # ---------------- interface với hri_motion ----------------
        self.pose_pub = self.create_publisher(PoseStamped, '/hri/cmd_pose', 10)
        self.grip_pub = self.create_publisher(Bool, '/hri/cmd_gripper', 10)
        self.home_pub = self.create_publisher(Empty, '/hri/cmd_home', 10)
        self.vel_pub = self.create_publisher(Float32, '/hri/set_vel_scale', 10)
        self.create_subscription(String, '/hri/status', self._status_cb, 10,
                                 callback_group=self._cb)
        self.create_subscription(Empty, '/hri/start', self._start_cb, 10,
                                 callback_group=self._cb)

        # ---------------- worker: chạy chu kỳ task (không block executor) ----------------
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'hri_task mode={self.mode}. auto_start={self.get_parameter("auto_start").value}, '
            f'cycles={self.get_parameter("cycles").value}; /hri/start (Empty) để kích hoạt.')

    # ---------------- callbacks: chỉ cập nhật state ----------------
    def _status_cb(self, msg: String):
        kind, seq = hri_common.parse_status(msg.data)
        if kind is None:
            self.get_logger().warn(f'Status lạ: "{msg.data}"')
            return
        with self._lock:
            if seq > self._last_seq:
                self._last_seq = seq
            self._last_kind = kind

    def _start_cb(self, msg: Empty):  # noqa: ARG002
        self._start_event.set()
        self.get_logger().info('Nhận /hri/start → kích hoạt 1 chu kỳ.')

    # ---------------- đồng bộ với executor ----------------
    def _snapshot(self):
        with self._lock:
            return self._last_seq

    def _wait_status(self, after_seq, kinds, timeout):
        """Chờ status có seq > after_seq và kind thuộc kinds. Trả True nếu *_DONE."""
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            with self._lock:
                seq, kind = self._last_seq, self._last_kind
            if seq > after_seq and kind in kinds:
                self.get_logger().info(f'Executor: {kind} #{seq}')
                return kind != hri_common.REJECTED and kind.endswith('_DONE')
            if time.monotonic() > deadline:
                self.get_logger().error(f'Timeout chờ status {kinds} sau {timeout:.0f}s.')
                return False
            time.sleep(0.02)
        return False

    def _wait_executor(self):
        """Chờ status đầu tiên (READY hoặc heartbeat IDLE) từ hri_motion."""
        while rclpy.ok():
            with self._lock:
                if self._last_seq >= 0 or self._last_kind is not None:
                    return True
            self.get_logger().warn('Chờ hri_motion sẵn sàng (chưa nhận /hri/status)...',
                                   throttle_duration_sec=10.0)
            time.sleep(0.5)
        return False

    # ---------------- primitive phát lệnh ----------------
    def _goto(self, x, y, z, pitch, vel, name=''):
        """Gửi 1 lệnh pose + chờ xong. vel = velocity scale cho lệnh này."""
        after = self._snapshot()
        self.vel_pub.publish(Float32(data=float(vel)))
        msg = PoseStamped()
        msg.header.frame_id = self.base_frame
        qx, qy, qz, qw = hri_common.pitch_to_quat(pitch)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pose_pub.publish(msg)
        self.get_logger().info(f'→ {name or "GOTO"} ({x:.3f},{y:.3f},{z:.3f}) pitch={pitch:.2f} vel={vel}')
        ok = self._wait_status(after, {hri_common.POSE_DONE, hri_common.POSE_FAILED,
                                       hri_common.REJECTED}, self.status_timeout)
        if not ok:
            self.get_logger().error(f'{name or "GOTO"} THẤT BẠI.')
        return ok

    def _grip(self, grasp: bool, name=''):
        after = self._snapshot()
        self.grip_pub.publish(Bool(data=grasp))
        self.get_logger().info(f'→ {name or "GRIP"} {"kẹp" if grasp else "nhả"}')
        return self._wait_status(after, {hri_common.GRIP_DONE, hri_common.GRIP_FAILED,
                                         hri_common.REJECTED}, self.status_timeout)

    def _home(self):
        after = self._snapshot()
        self.home_pub.publish(Empty())
        self.get_logger().info('→ HOME')
        return self._wait_status(after, {hri_common.HOME_DONE, hri_common.HOME_FAILED,
                                         hri_common.REJECTED}, self.status_timeout)

    # ---------------- worker loop ----------------
    def _worker_loop(self):
        if not self._wait_executor():
            return
        if self.mode != 'fixed':
            self.get_logger().error(
                f'mode="{self.mode}" chưa hỗ trợ (BƯỚC 2: camera). Chỉ có "fixed". Thoát task.')
            return
        cycles = int(self.get_parameter('cycles').value)
        auto = bool(self.get_parameter('auto_start').value)
        done = 0
        while rclpy.ok() and (cycles < 0 or done < cycles):
            if not auto:
                self._start_event.wait()
                if not rclpy.ok():
                    return
                self._start_event.clear()
            try:
                self._run_fixed_cycle()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Chu kỳ lỗi: {exc}')
            done += 1
            self.get_logger().info(f'==== HOÀN TÀT chu kỳ {done}'
                                   + ('' if cycles < 0 else f'/{cycles}') + ' ====')
            time.sleep(1.0)
        self.get_logger().info('hri_task kết thúc (hết chu kỳ).')

    # ---------------- BƯỚC 1: chu kỳ fixed pick → place ----------------
    def _run_fixed_cycle(self):
        px = float(self.get_parameter('pick_x').value)
        py = float(self.get_parameter('pick_y').value)
        pz = float(self.get_parameter('pick_z').value)
        ppitch = float(self.get_parameter('pick_pitch').value)
        qx = float(self.get_parameter('place_x').value)
        qy = float(self.get_parameter('place_y').value)
        qz = float(self.get_parameter('place_z').value)
        qpitch = float(self.get_parameter('place_pitch').value)
        delta = float(self.get_parameter('approach_delta').value)
        off = float(self.get_parameter('finger_grasp_offset').value)
        ret = float(self.get_parameter('retreat_m').value)

        self.get_logger().info(
            f'==== FIXED PICK ({px:.3f},{py:.3f},{pz:.3f}) → PLACE ({qx:.3f},{qy:.3f},{qz:.3f}) ====')

        def fail():
            self.get_logger().warn('ABORT chu kỳ — về home.')
            self._home()
            return False

        # 1. home
        if not self._home():
            return fail()
        # 2. approach (trên điểm gắp)
        if not self._goto(px, py, pz + delta, ppitch, self.vcruise, 'APPROACH'):
            return fail()
        # 3. descend (xuống kẹp)
        if not self._goto(px, py, pz - off, ppitch, self.vdel, 'DESCEND'):
            return fail()
        # 4. grasp
        if not self._grip(True, 'GRASP'):
            return fail()
        # 5. lift
        if not self._goto(px, py, pz + delta, ppitch, self.vcruise, 'LIFT'):
            return fail()
        # 6. transport (trên điểm nhả)
        if not self._goto(qx, qy, qz + delta, qpitch, self.vcruise, 'TRANSPORT'):
            return fail()
        # 7. place
        if not self._goto(qx, qy, qz, qpitch, self.vdel, 'PLACE'):
            return fail()
        # 8. release
        if not self._grip(False, 'RELEASE'):
            return fail()
        # 9. retreat (lùi ngang về phía base)
        norm = math.hypot(qx, qy) or 1.0
        if not self._goto(qx - ret * qx / norm, qy - ret * qy / norm, qz + delta,
                          qpitch, self.vcruise, 'RETREAT'):
            self.get_logger().warn('RETREAT fail — tiếp tục về home.')
        # 10. home
        self._home()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = HriTaskNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
