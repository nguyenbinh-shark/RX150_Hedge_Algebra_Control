#!/usr/bin/env python3
"""
pick_place_moveit_node — LAYER 2: gắp vật được CHỈ BẰNG CỬ CHỈ TAY rồi thả vào
vùng đặt cố định (rx150, qua MoveIt).

  /yolo/detected_tubes            (PoseArray, rx150/base_link) — vật thể
  /yolo/tube_classes              (String JSON)                — nhãn màu
  /hand_gesture/selected_target   (Int32)  — chỉ số vật đang được chỉ tay
  /hand_gesture/event == "ok_sign"         — KÍCH gắp vật đang chọn
  ~/status                        (String JSON) — state machine + thống kê
  ~/stop | ~/reset | ~/home | ~/pick | ~/open_gripper (std_srvs/Trigger)

Chuỗi (mỗi bước arm = 1 goal MoveGroup nhóm interbotix_arm; các đoạn tiếp cận /
hạ / rút đi THẲNG trong không gian Descartes):
  PLANNING (IK toàn chuỗi TRƯỚC khi động) → APPROACH → DESCEND → GRASP(+verify)
  → LIFT → TRANSPORT → PLACE → RELEASE → RETRACT → HOME

Toàn bộ phần chấp hành nằm ở thư viện rx150_pick_place.* (dùng chung với
tube_rack_node). Node này chỉ lo: chọn vật nào, thả ở đâu, khi nào.

Yêu cầu T1: ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py use_camera:=true
Chạy T2:    ros2 launch rx150_pick_place pick_place.launch.py
"""
import math
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import OperatingModes

from rx150_pick_place.params import build_stack, declare_common, read_common, table_object
from rx150_pick_place.skills import joint_deg
from rx150_pick_place.status import State

NODE_DEFAULTS = {
    # ---- vùng đặt vật (TUNE theo bàn thật) ----
    'place_x': 0.30,
    'place_y': 0.0,
    'place_z': 0.06,
    'place_approach_delta': 0.08,
    # ---- cử chỉ ----
    'selected_target_topic': '/hand_gesture/selected_target',
    'gesture_event_topic': '/hand_gesture/event',
    'trigger_event': 'ok_sign',
    'selection_timeout_s': 5.0,     # chỉ số chỉ tay cũ hơn ngần này → không nhận
    # ---- phần cứng ----
    'commands_topic': '/rx150/commands/joint_single',
    'operating_modes_service': '/rx150/set_operating_modes',
    'set_gripper_pwm_mode': True,
}


class PickPlaceMoveItNode(Node):
    def __init__(self):
        super().__init__('pick_place_moveit')
        declare_common(self, NODE_DEFAULTS)
        self.cfg = read_common(self)
        for name in NODE_DEFAULTS:
            setattr(self.cfg, name, self.get_parameter(name).value)

        self._cb = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(JointSingleCommand,
                                              self.cfg.commands_topic, 10)
        self.stack = build_stack(self, self.cfg, callback_group=self._cb,
                                 status_topic='~/status', publisher=self._cmd_pub)
        self.skill = self.stack.skill
        self.status = self.stack.status

        # ---- lựa chọn từ cử chỉ ----
        self._sel_lock = threading.Lock()
        self._selected = -1
        self._selected_t = 0.0
        self.create_subscription(Int32, self.cfg.selected_target_topic,
                                 self._target_cb, 10, callback_group=self._cb)
        self.create_subscription(String, self.cfg.gesture_event_topic,
                                 self._event_cb, 10, callback_group=self._cb)

        # ---- điều khiển vận hành ----
        for name, handler in (('~/pick', self._srv_pick), ('~/stop', self._srv_stop),
                              ('~/reset', self._srv_reset), ('~/home', self._srv_home),
                              ('~/open_gripper', self._srv_open)):
            self.create_service(Trigger, name, handler, callback_group=self._cb)

        self._job = threading.Event()
        self._job_idx = -1
        self._busy_lock = threading.Lock()
        self._busy = False
        self._scene_ready = False
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self._setup_gripper_mode()
        self.status.set(State.IDLE, 'chờ ok_sign')
        self.get_logger().info(
            f'pick_place_moveit sẵn sàng — chỉ tay chọn vật rồi ra hiệu '
            f'"{self.cfg.trigger_event}" (hoặc gọi service ~/pick). '
            f'dry_run={self.cfg.dry_run}, gripper_bridge={self.cfg.use_gripper_bridge}.')

    # ── phần cứng ───────────────────────────────────────────────────────
    def _setup_gripper_mode(self):
        """Gripper phải ở PWM mode (cả bridge lẫn fallback đều đẩy effort).

        Bản cũ làm việc này qua InterbotixManipulatorXS — kéo theo cả một node
        SDK, và constructor của nó GHI thanh ghi Profile_Velocity/Acceleration
        của các motor cánh tay (trong khi cánh tay đang do fuzzy_node giữ ở PWM).
        Ở đây chỉ gọi đúng 1 service.
        """
        if not self.cfg.set_gripper_pwm_mode or self.cfg.dry_run:
            return
        client = self.create_client(OperatingModes, self.cfg.operating_modes_service,
                                    callback_group=self._cb)
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f'{self.cfg.operating_modes_service} chưa sẵn sàng — không đặt được '
                'gripper về PWM mode (xs_sdk chưa chạy?).')
            return
        req = OperatingModes.Request(cmd_type='single', name='gripper', mode='pwm')
        client.call_async(req)
        self.get_logger().info('Đã yêu cầu gripper → PWM mode.')

    # ── callbacks cử chỉ ────────────────────────────────────────────────
    def _target_cb(self, msg: Int32):
        with self._sel_lock:
            self._selected = int(msg.data)
            self._selected_t = self.get_clock().now().nanoseconds * 1e-9

    def _event_cb(self, msg: String):
        if msg.data != self.cfg.trigger_event:
            return
        ok, reason = self._request_pick()
        if not ok:
            self.get_logger().warn(f'Bỏ qua {self.cfg.trigger_event}: {reason}',
                                   throttle_duration_sec=2.0)

    def _request_pick(self):
        with self._sel_lock:
            idx, stamp = self._selected, self._selected_t
        now = self.get_clock().now().nanoseconds * 1e-9
        if idx < 0:
            return False, 'chưa chỉ tay chọn vật nào.'
        if now - stamp > float(self.cfg.selection_timeout_s):
            return False, (f'lựa chọn đã cũ {now - stamp:.1f}s — chỉ tay lại vào vật '
                           'muốn gắp.')
        if self.stack.executor.aborted:
            return False, 'đang ở trạng thái STOP — gọi ~/reset trước.'
        with self._busy_lock:
            if self._busy:
                return False, 'đang bận thực hiện chu kỳ trước.'
            self._busy = True
            self._job_idx = idx
        self._job.set()
        return True, f'nhận lệnh gắp vật #{idx}'

    # ── services vận hành ───────────────────────────────────────────────
    def _srv_pick(self, _request, response):
        response.success, response.message = self._request_pick()
        return response

    def _srv_stop(self, _request, response):
        """E-stop mềm: huỷ goal MoveIt đang chạy, chặn mọi bước tiếp theo."""
        self.stack.executor.request_abort()
        self.status.set(State.ESTOP, 'người vận hành yêu cầu dừng')
        response.success = True
        response.message = 'Đã dừng. Gọi ~/reset để chạy lại.'
        return response

    def _srv_reset(self, _request, response):
        self.stack.executor.clear_abort()
        with self._busy_lock:
            self._busy = False
        self.status.set(State.IDLE, 'đã reset')
        response.success = True
        response.message = 'Sẵn sàng.'
        return response

    def _srv_home(self, _request, response):
        response.success = self.skill.go_home()
        response.message = 'Đã về home.' if response.success else 'Về home thất bại.'
        return response

    def _srv_open(self, _request, response):
        response.success = self.stack.gripper.open()
        response.message = 'Đã mở gripper.'
        return response

    # ── worker ──────────────────────────────────────────────────────────
    def _worker_loop(self):
        while rclpy.ok():
            self._job.wait(timeout=1.0)
            if not self._job.is_set():
                self.status.publish()          # heartbeat cho HMI
                continue
            self._job.clear()
            idx = self._job_idx
            try:
                self._run_cycle(idx)
            except Exception as exc:           # noqa: BLE001
                self.get_logger().error(f'Chu kỳ lỗi ngoài dự kiến (#{idx}): {exc}')
                import traceback
                self.get_logger().error(traceback.format_exc())
                self.status.fault(f'exception: {exc}')
                self.skill.recover('exception')
                self.status.end_cycle(False)
            finally:
                with self._busy_lock:
                    self._busy = False

    # ── planning scene ──────────────────────────────────────────────────
    def _ensure_scene(self, obstacles=()):
        objects = []
        if not self._scene_ready and self.cfg.add_table_collision:
            objects.append(table_object(self.stack.scene, self.cfg))
        for i, obj in enumerate(obstacles):
            objects.append(self.stack.scene.add_cylinder(
                f'obstacle_{i}', (obj['x'], obj['y'], obj['z']),
                self.cfg.object_length, self.cfg.object_radius))
        if objects:
            self.stack.scene.apply(objects, label='vật cản')
        self._scene_ready = True

    def _clear_obstacles(self, count):
        if count:
            self.stack.scene.apply([self.stack.scene.remove(f'obstacle_{i}')
                                    for i in range(count)], label='xoá vật cản')

    # ── 1 chu kỳ pick-place ─────────────────────────────────────────────
    def _run_cycle(self, idx):
        self.status.begin_cycle()
        self.status.set(State.SCANNING, f'lấy pose vật #{idx}')

        target, others = self._resolve_target(idx)
        if target is None:
            self.status.fault(f'không có detection hợp lệ cho vật #{idx}')
            self.status.end_cycle(False)
            return

        self.get_logger().info(
            f'==== PICK #{idx} [{target["cls"]}] tại '
            f'({target["x"]:.3f}, {target["y"]:.3f}, {target["z"]:.3f}) '
            f'yaw={math.degrees(target["yaw"]):.0f}° ====')

        obstacles = others if self.cfg.add_detected_obstacles else ()
        self._ensure_scene(obstacles)

        # ---- LẬP KẾ HOẠCH TOÀN CHUỖI TRƯỚC KHI ĐỘNG ----
        self.status.set(State.PLANNING)
        grasp = self.skill.plan_grasp(target['x'], target['y'], target['z'],
                                      yaw=target['yaw'])
        if grasp is None:
            self.status.fault('không lập được kế hoạch gắp (ngoài tầm / quá dốc)')
            self._clear_obstacles(len(obstacles))
            self.status.end_cycle(False)
            return

        place_hover = self.skill.plan_pose(
            self.cfg.place_x, self.cfg.place_y,
            self.cfg.place_z + self.cfg.place_approach_delta,
            self.cfg.place_pitch_ladder, seed=grasp.lift.joints, label='PLACE-HOVER')
        place = None
        if place_hover is not None:
            place = self.skill.plan_pose(
                self.cfg.place_x, self.cfg.place_y, self.cfg.place_z,
                [place_hover.pitch], wrist=place_hover.wrist,
                seed=place_hover.joints, label='PLACE')
        if place is None:
            # Fail-fast: chưa kẹp gì cả nên dừng ở đây là an toàn tuyệt đối.
            self.status.fault('vị trí thả không với tới — kiểm tra place_x/y/z')
            self._clear_obstacles(len(obstacles))
            self.status.end_cycle(False)
            return
        self.get_logger().info('Kế hoạch: ' + self.skill.describe(
            grasp.pre, grasp.grasp, grasp.lift, place_hover, place))

        # ---- CHẤP HÀNH ----
        if not self.skill.grasp_at(grasp):
            self.status.fault('gắp thất bại')
            self.skill.recover('grasp fail')
            self._clear_obstacles(len(obstacles))
            self.status.end_cycle(False)
            return

        if not self.skill.release_at(place, hover=place_hover):
            self.status.fault('thả thất bại')
            self.skill.recover('place fail')
            self._clear_obstacles(len(obstacles))
            self.status.end_cycle(False)
            return

        self.skill.go_home()
        self._clear_obstacles(len(obstacles))
        self.status.end_cycle(True)
        self.status.set(State.DONE, f'vật #{idx} [{target["cls"]}] — {self.status.summary()}')
        self.status.set(State.IDLE, 'chờ lệnh tiếp theo', log=False)

    def _resolve_target(self, idx):
        """Chỉ số chỉ tay → pose đã lọc median. Trả (vật, các vật còn lại)."""
        raw = self.stack.detection.snapshot()
        if raw is None:
            self.get_logger().error(
                'Detection không dùng được (cũ / sai frame / chưa có) — không gắp.')
            return None, []
        if idx >= len(raw):
            self.get_logger().error(
                f'Chỉ số #{idx} vượt số vật đang thấy ({len(raw)}) — vật có thể đã '
                'biến mất giữa lúc chỉ tay và lúc ra hiệu.')
            return None, []
        rough = raw[idx]
        refined = self.stack.detection.median_snapshot(
            frames=self.cfg.median_frames, collect_s=self.cfg.median_collect_s,
            assoc_radius=self.cfg.detection_assoc_radius_m,
            min_hits_ratio=self.cfg.detection_min_hits_ratio)
        if not refined:
            self.get_logger().error('Không thu đủ frame ổn định để gắp.')
            return None, []
        best, best_d = None, self.cfg.detection_assoc_radius_m * 2.0
        for obj in refined:
            d = math.hypot(obj['x'] - rough['x'], obj['y'] - rough['y'])
            if d < best_d:
                best, best_d = obj, d
        if best is None:
            self.get_logger().error(
                'Vật được chỉ không còn ổn định giữa các frame — không gắp.')
            return None, []
        others = [o for o in refined if o is not best]
        self.get_logger().info(
            f'Pose đã lọc {best["frames"]} frame, độ tản mát XY={best["spread_xy"] * 1000:.1f}mm.')
        return best, others

    # ── khởi động ───────────────────────────────────────────────────────
    def startup_checks(self):
        if not self.stack.executor.wait_ready(timeout=15.0):
            self.status.fault('MoveIt / joint_states chưa sẵn sàng')
            return False
        joints = self.stack.executor.current_joints()
        if joints is None:
            self.get_logger().warn(
                f'Chưa nhận được {self.cfg.joint_states_topic} — không verify được pose '
                'sau mỗi bước (kiểm tra xs_sdk / namespace).')
        else:
            self.get_logger().info(f'Tư thế hiện tại: {joint_deg(joints)}°')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceMoveItNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.startup_checks()
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.stack.executor.request_abort()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
