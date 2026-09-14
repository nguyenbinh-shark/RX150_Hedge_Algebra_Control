#!/usr/bin/env python3
"""
tube_rack_node — gắp ống nghiệm nằm trên bàn → cắm vào giá, phân theo màu.

  /yolo/detected_tubes (PoseArray)  /yolo/tube_classes (String JSON)
  ~/status (String JSON)            /tube_rack/run (Trigger — chạy 1 chu kỳ)
  ~/stop | ~/reset | ~/home | ~/open_gripper (Trigger)

Chu kỳ: SCAN → (mỗi ống) PLANNING → APPROACH → DESCEND → GRASP(+verify) → LIFT
→ TRANSPORT → INSERT → RELEASE → RETRACT → … → HOME + báo cáo.

HÌNH HỌC GIÁ — HAI NGUỒN, file hiệu chuẩn THẮNG:
  (a) rack_calib_file (mặc định rack_pose.yaml của rx150_perception) — do
      rack_calib_node đo bằng AprilTag dán trên giá. Tả được cả 4 lỗ ở 4 đỉnh
      hình chữ nhật + TRỤC LỖ thật. Có file thì (b) bị bỏ qua hoàn toàn.
      Hiệu chuẩn lại: ./rx150.sh rack-calib
  (b) slot0_* + rack_yaw_deg + slot_spacing dưới đây — đo bằng thước, chỉ tả
      được MỘT HÀNG slot thẳng. Đường dự phòng khi chưa dán tag lên giá.

  slot0_x/y/z   toạ độ MIỆNG LỖ slot đầu tiên trong frame rx150/base_link
  rack_yaw_deg  hướng của HÀNG slot trong mặt phẳng XY (0° = dọc trục +x)
  rack_tilt_deg độ nghiêng của TRỤC LỖ so với phương THẲNG ĐỨNG
                (0° = lỗ dựng đứng; giá nghiêng 62° so với mặt bàn ⇒ 28°)
  slot_dz       chênh cao giữa 2 slot liên tiếp (giá bậc thang), 0 nếu ngang bằng

  Bản cũ chỉ có `rack_angle_deg` và dùng nó làm GÓC XY (sx += k·spacing·cos,
  sy += k·spacing·sin) trong khi tài liệu lại gọi là "góc nghiêng của giá" —
  hai thứ khác hẳn nhau. Ở đây tách hẳn ra, và pitch lúc cắm được suy từ
  rack_tilt_deg chứ không phải một ladder đặt tay rời rạc.

  Node LUÔN kiểm tra mọi slot có với tới được không NGAY LÚC KHỞI ĐỘNG và in
  bảng — thay vì tới ống thứ 3 mới phát hiện slot nằm ngoài tầm.

Yêu cầu T1: ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py use_camera:=true
Chạy:       ros2 launch rx150_pick_place tube_rack.launch.py [dry_run:=true] [auto_start:=false]
"""
import json
import math
import os
import threading

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import OperatingModes

from rx150_modules.params import build_stack, declare_common, read_common, table_object
from rx150_modules.status import State

NODE_DEFAULTS = {
    # ---- vận hành ----
    'auto_start': True,
    'fake_tubes': '[]',            # JSON để thử IK không cần camera
    'pick_order': 'nearest_first',  # nearest_first | farthest_first | detection
    'max_consecutive_failures': 2,
    # ---- hình học giá ----
    # Rỗng = tắt hẳn đường hiệu chuẩn, chỉ dùng slot0_* bên dưới.
    'rack_calib_file': 'rack_pose.yaml',
    'slot0_x': 0.24,
    'slot0_y': -0.01,
    'slot0_z': 0.10,
    'rack_yaw_deg': 0.0,
    'rack_tilt_deg': 0.0,
    'slot_spacing': 0.05,
    'slot_dz': 0.0,
    'num_slots': 4,
    'rack_filter_xy_m': 0.05,
    'insert_depth': 0.03,
    'hover_clearance': 0.04,
    'tube_length': 0.10,
    # ---- màu → slot ----
    'color_slot_map.pink': [0],
    'color_slot_map.blue': [1],
    'color_slot_map.green': [2],
    'color_slot_map.yellow': [3],
    # ---- vật cản giá ----
    'add_rack_collision': True,
    'rack_box_x': 0.30,
    'rack_box_y': -0.01,
    'rack_box_z': 0.10,
    'rack_box_size_x': 0.15,
    'rack_box_size_y': 0.10,
    'rack_box_size_z': 0.20,
    # ---- phần cứng ----
    'commands_topic': '/rx150/commands/joint_single',
    'operating_modes_service': '/rx150/set_operating_modes',
    'set_gripper_pwm_mode': True,
}


class TubeRackNode(Node):
    def __init__(self):
        super().__init__('tube_rack')
        declare_common(self, NODE_DEFAULTS)
        # tube_rack gắp ống dài nên mặc định object_length khác — chỉ đổi khi
        # người dùng không tự đặt.
        self.cfg = read_common(self)
        for name in NODE_DEFAULTS:
            setattr(self.cfg, name, self.get_parameter(name).value)
        self.cfg.object_length = float(self.cfg.tube_length)

        self._cb = ReentrantCallbackGroup()
        self._cmd_pub = self.create_publisher(JointSingleCommand,
                                              self.cfg.commands_topic, 10)
        self.stack = build_stack(self, self.cfg, callback_group=self._cb,
                                 status_topic='~/status', publisher=self._cmd_pub)
        self.skill = self.stack.skill
        self.status = self.stack.status

        self.calib = self._load_rack_calib()
        self.slots = self._compute_slots()
        self._slot_occupied = {}
        self._scene_ready = False

        self.create_service(Trigger, '/tube_rack/run', self._srv_run,
                            callback_group=self._cb)
        for name, handler in (('~/stop', self._srv_stop), ('~/reset', self._srv_reset),
                              ('~/home', self._srv_home), ('~/open_gripper', self._srv_open)):
            self.create_service(Trigger, name, handler, callback_group=self._cb)

        self._run_event = threading.Event()
        self._busy_lock = threading.Lock()
        self._busy = False
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self._setup_gripper_mode()
        self.status.set(State.IDLE, 'chờ /tube_rack/run')

    # ── phần cứng ───────────────────────────────────────────────────────
    def _setup_gripper_mode(self):
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

    # ── hình học giá ────────────────────────────────────────────────────
    def _load_rack_calib(self):
        """Nạp rack_pose.yaml (rack_calib_node đo bằng AprilTag) nếu có.

        Thắng slot0_*/slot_spacing/rack_yaw_deg vì file này tả ĐÚNG 4 lỗ ở 4 đỉnh
        hình chữ nhật + trục lỗ thật, còn bộ tham số kia chỉ tả được một HÀNG
        slot thẳng. Không có file thì im lặng quay về đường đo bằng thước.

        Đọc THẲNG file chứ không tra TF `tube_rack`: hình học giá phải xác định
        ngay lúc khởi động (check_rack_reachable chạy trước khi có ai publish TF),
        và một chu kỳ gắp không được đổi mục tiêu giữa chừng vì camera rung.
        """
        raw = str(getattr(self.cfg, 'rack_calib_file', '') or '').strip()
        if not raw:
            return None
        path = raw
        if not os.path.isabs(path):
            try:
                path = os.path.join(get_package_share_directory('rx150_perception'),
                                    'config', raw)
            except PackageNotFoundError:
                self.get_logger().warn(
                    f'Không thấy package rx150_perception — bỏ qua rack_calib_file={raw}.')
                return None
        if not os.path.exists(path):
            self.get_logger().info(
                f'CHƯA hiệu chuẩn giá (không có {path}) ⇒ dùng slot0_*/slot_spacing/'
                'rack_yaw_deg đo bằng thước trong tube_rack_params.yaml. '
                'Hiệu chuẩn bằng camera: ./rx150.sh rack-calib')
            return None
        try:
            with open(path, 'r', encoding='utf-8') as handle:
                doc = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:          # noqa: BLE001
            self.get_logger().error(f'Đọc {path} hỏng ({exc}) — quay về slot0_*.')
            return None

        slots = [[float(c) for c in item] for item in (doc.get('slots') or [])]
        if not slots or any(len(s) != 3 for s in slots):
            self.get_logger().error(
                f'{path} thiếu/hỏng khoá `slots` — quay về slot0_*. Snap lại: '
                './rx150.sh rack-calib')
            return None
        base = str(doc.get('base_frame', ''))
        if base and base != str(self.cfg.base_frame):
            self.get_logger().error(
                f'{path} đo trong khung {base} nhưng node đang làm việc trong '
                f'{self.cfg.base_frame} — BỎ QUA file. Sửa base_frame rồi snap lại.')
            return None

        axis = [float(v) for v in (doc.get('slot_axis') or [0.0, 0.0, 1.0])]
        norm = math.sqrt(sum(v * v for v in axis)) or 1.0
        axis = tuple(v / norm for v in axis)
        self.cfg.num_slots = len(slots)
        self.cfg.rack_tilt_deg = float(doc.get('rack_tilt_deg', 0.0))
        box = doc.get('rack_box') or {}
        for key in ('x', 'y', 'z', 'size_x', 'size_y', 'size_z'):
            if key in box:
                setattr(self.cfg, f'rack_box_{key}', float(box[key]))
        meta = doc.get('measurement') or {}
        self.get_logger().info(
            f'Hình học giá lấy từ HIỆU CHUẨN: {path} '
            f'(tag {meta.get("tag_id", "?")}, {meta.get("samples", "?")} mẫu, '
            f'{meta.get("calibrated_at", "?")}) — slot0_*/slot_spacing/rack_yaw_deg '
            'trong tube_rack_params.yaml KHÔNG được dùng.')
        return {'slots': [tuple(s) for s in slots], 'axis': axis, 'path': path}

    def _compute_slots(self):
        """[(x, y, z_miệng_lỗ)] cho từng slot, từ slot0 + hướng hàng + bậc cao."""
        if self.calib:
            return list(self.calib['slots'])
        yaw = math.radians(float(self.cfg.rack_yaw_deg))
        spacing = float(self.cfg.slot_spacing)
        dz = float(self.cfg.slot_dz)
        return [(self.cfg.slot0_x + k * spacing * math.cos(yaw),
                 self.cfg.slot0_y + k * spacing * math.sin(yaw),
                 self.cfg.slot0_z + k * dz)
                for k in range(int(self.cfg.num_slots))]

    def _slot_axis(self, sx, sy):
        """Vector đơn vị dọc TRỤC LỖ (hướng lên) + pitch cần cho ee.

        Trục lỗ nghiêng rack_tilt_deg so với phương thẳng đứng, ngả ra xa gốc
        (theo phương bán kính của slot). Ống kẹp nằm dọc trục ee_z ⇒ pitch của
        ee đúng bằng góc nghiêng đó.
        """
        if self.calib:
            # Trục ĐO ĐƯỢC, không phải trục suy ra từ phương bán kính: giá đặt
            # lệch tâm thì hai thứ này khác nhau vài độ.
            axis = self.calib['axis']
            return axis, math.acos(max(-1.0, min(1.0, axis[2])))
        tilt = math.radians(float(self.cfg.rack_tilt_deg))
        azimuth = math.atan2(sy, sx)
        axis = (math.sin(tilt) * math.cos(azimuth),
                math.sin(tilt) * math.sin(azimuth),
                math.cos(tilt))
        return axis, tilt

    def _insert_pose(self, slot_index, depth=None):
        """Vị trí ee_gripper_link khi ống đã cắm sâu `insert_depth` vào slot.

        Ống được kẹp ở GIỮA thân ⇒ đáy ống cách ee một đoạn tube_length/2 dọc
        trục ống. Muốn đáy ống nằm dưới miệng lỗ `insert_depth`:
            p_ee = p_miệng_lỗ + trục · (tube_length/2 − insert_depth)
        """
        sx, sy, sz = self.slots[slot_index]
        axis, tilt = self._slot_axis(sx, sy)
        depth = self.cfg.insert_depth if depth is None else float(depth)
        along = float(self.cfg.tube_length) / 2.0 - depth
        return (sx + axis[0] * along, sy + axis[1] * along, sz + axis[2] * along), tilt

    def _place_ladder(self, tilt):
        """Ladder pitch quanh góc nghiêng của lỗ (không phải quanh 0).

        Khi có hiệu chuẩn, rack_tilt_deg là góc ĐO ĐƯỢC nên hầu như không bao giờ
        đúng bằng 0 (0,3–0,5° là chuyện thường) ⇒ nhánh "tôn trọng ladder trong
        YAML" bên dưới tự tắt và ladder được dựng quanh góc thật. Đó là ý muốn:
        số đo được sát thực tế hơn ladder đặt tay.
        """
        override = list(self.cfg.place_pitch_ladder or [])
        if override and abs(float(self.cfg.rack_tilt_deg)) < 1e-6:
            return override            # giá dựng đứng: tôn trọng ladder trong YAML
        return [tilt, tilt + 0.15, tilt - 0.15, tilt + 0.30]

    def check_rack_reachable(self):
        """In bảng slot với/không với tới — chạy lúc khởi động, KHÔNG di chuyển."""
        kin = self.stack.kin
        lines, unreachable = [], 0
        for i, (sx, sy, sz) in enumerate(self.slots):
            (ix, iy, iz), tilt = self._insert_pose(i)
            hover_z = iz + float(self.cfg.hover_clearance)
            ok_insert = bool(kin.ik_all(ix, iy, iz, tilt))
            ok_hover = bool(kin.ik_all(ix, iy, hover_z, tilt))
            mark = 'OK ' if (ok_insert and ok_hover) else 'HỎNG'
            if not (ok_insert and ok_hover):
                unreachable += 1
            lines.append(
                f'  slot {i}: miệng ({sx:+.3f},{sy:+.3f},{sz:.3f}) r={math.hypot(sx, sy):.3f}m '
                f'→ ee cắm ({ix:+.3f},{iy:+.3f},{iz:.3f}) pitch={math.degrees(tilt):.0f}° '
                f'[{mark}]' + ('' if ok_insert else
                               f'  ← {kin.reach_report(ix, iy, iz, tilt)}'))
        self.get_logger().info('Hình học giá (tầm với rx150 ≈ '
                               f'{kin.max_reach:.3f}m):\n' + '\n'.join(lines))
        if unreachable:
            fix = (f'giá đang ở xa quá — dời giá lại gần rồi hiệu chuẩn lại '
                   f'(./rx150.sh rack-calib); số hiện tại đến từ {self.calib["path"]}'
                   if self.calib else
                   'sửa slot0_*/slot_spacing/rack_yaw_deg trong tube_rack_params.yaml')
            self.get_logger().error(
                f'{unreachable}/{len(self.slots)} slot NGOÀI TẦM — {fix} trước khi chạy.')
        return unreachable == 0

    # ── chọn slot ───────────────────────────────────────────────────────
    def _preferred_slots(self, color):
        try:
            values = self.get_parameter(f'color_slot_map.{color}').value
            if values:
                return [int(v) for v in values]
        except Exception:                      # noqa: BLE001 — màu không có mapping
            pass
        return list(range(int(self.cfg.num_slots)))

    def _choose_slot(self, color):
        for index in self._preferred_slots(color):
            if index < len(self.slots) and not self._slot_occupied.get(index):
                return index
        for index in range(len(self.slots)):
            if not self._slot_occupied.get(index):
                return index
        return None

    def _mark_occupied_from_detections(self, tubes):
        """Ống đã nằm trong vùng 1 slot ⇒ slot đó coi như đã đầy (và không gắp lại).

        Bản cũ chỉ LOẠI ống đó khỏi danh sách gắp nhưng vẫn coi slot là trống →
        ống tiếp theo được cắm chồng lên chính chỗ đã có ống.

        Bản mới:
        - Z đủ cao (gần miệng slot) → coi là đã cắm, đánh dấu slot đầy.
        - Z thấp (trên mặt bàn) gần giá → bỏ luôn (false positive hoặc ống
          không thể gắp do va chạm collision rack).
        """
        radius = float(self.cfg.rack_filter_xy_m)
        z_margin = 0.03          # ống cần Z ≥ slot_z - margin mới coi là đang trên giá
        remaining = []
        for tube in tubes:
            near_slot = None      # slot gần nhất (XY)
            on_rack = False       # True nếu Z đủ cao → đang cắm
            for i, (sx, sy, sz) in enumerate(self.slots):
                if math.hypot(tube['x'] - sx, tube['y'] - sy) < radius:
                    near_slot = i
                    on_rack = tube['z'] >= sz - z_margin
                    break
            if near_slot is None:
                # Không gần slot nào → ống trên bàn, gắp bình thường
                remaining.append(tube)
            elif on_rack:
                # Ống đã cắm trong giá → đánh dấu slot đầy
                self._slot_occupied[near_slot] = True
                self.get_logger().info(
                    f'Slot {near_slot} đã có ống {tube["cls"]} — bỏ qua, đánh dấu đã đầy.')
            else:
                # Gần slot nhưng Z thấp → false positive hoặc ống dưới chân giá;
                # không gắp được (va collision rack) → BỎ
                self.get_logger().warn(
                    f'BỎ ống {tube["cls"]} gần slot {near_slot}: '
                    f'Z={tube["z"]:.3f} < {self.slots[near_slot][2] - z_margin:.3f} '
                    f'(nằm dưới giá, không gắp được — có thể là detection ma).')
        return remaining

    # ── services ────────────────────────────────────────────────────────
    def _srv_run(self, _request, response):
        if self.stack.executor.aborted:
            response.success, response.message = False, 'Đang STOP — gọi ~/reset trước.'
            return response
        with self._busy_lock:
            if self._busy:
                response.success, response.message = False, 'Đang chạy chu kỳ khác.'
                return response
        self._run_event.set()
        response.success, response.message = True, 'Đã kích hoạt chu kỳ.'
        return response

    def _srv_stop(self, _request, response):
        self.stack.executor.request_abort()
        self.status.set(State.ESTOP, 'người vận hành yêu cầu dừng')
        response.success, response.message = True, 'Đã dừng. ~/reset để chạy lại.'
        return response

    def _srv_reset(self, _request, response):
        self.stack.executor.clear_abort()
        with self._busy_lock:
            self._busy = False
        self.status.set(State.IDLE, 'đã reset')
        response.success, response.message = True, 'Sẵn sàng.'
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
            self._run_event.wait(timeout=1.0)
            if not self._run_event.is_set():
                self.status.publish()
                continue
            self._run_event.clear()
            with self._busy_lock:
                self._busy = True
            try:
                self._run_cycle()
            except Exception as exc:            # noqa: BLE001
                import traceback
                self.get_logger().error(traceback.format_exc())
                self.status.fault(f'exception: {exc}')
                self.skill.recover('exception')
            finally:
                with self._busy_lock:
                    self._busy = False

    def _ensure_scene(self):
        if self._scene_ready:
            return
        objects = []
        if self.cfg.add_table_collision:
            objects.append(table_object(self.stack.scene, self.cfg))
        if self.cfg.add_rack_collision:
            objects.append(self.stack.scene.add_box(
                'rack',
                (self.cfg.rack_box_x, self.cfg.rack_box_y, self.cfg.rack_box_z),
                (self.cfg.rack_box_size_x, self.cfg.rack_box_size_y,
                 self.cfg.rack_box_size_z)))
        if objects:
            self.stack.scene.apply(objects, label='vật cản tĩnh')
        self._scene_ready = True

    # ── 1 chu kỳ: quét → gắp lần lượt ───────────────────────────────────
    def _run_cycle(self):
        self.get_logger().info('═══ BẮT ĐẦU CHU KỲ GẮP ỐNG → GIÁ ═══')
        self._ensure_scene()
        self._slot_occupied = {}

        self.status.set(State.HOMING, 'về home trước khi quét')
        self.skill.go_home()
        self.skill.open_gripper()

        self.status.set(State.SCANNING)
        tubes = self._scan()
        tubes = self._mark_occupied_from_detections(tubes)
        if not tubes:
            self.status.set(State.DONE, 'không còn ống nào cần gắp')
            self.skill.go_home()
            return
        tubes = self._sort_tubes(tubes)

        self.get_logger().info(f'Số ống cần gắp: {len(tubes)}')
        for i, tube in enumerate(tubes):
            self.get_logger().info(
                f'  [{i}] {tube["cls"]:>8} x={tube["x"]:+.3f} y={tube["y"]:+.3f} '
                f'z={tube["z"]:.3f} yaw={math.degrees(tube["yaw"]):+.0f}° '
                f'(median {tube.get("frames", "?")} frame)')

        results, consecutive_fail = [], 0
        for i, tube in enumerate(tubes):
            if self.stack.executor.aborted:
                self.get_logger().warn('E-stop — dừng chu kỳ.')
                break
            self.get_logger().info(
                f'──── ỐNG [{i + 1}/{len(tubes)}] {tube["cls"]} ────')
            self.status.begin_cycle()
            ok = self._pick_one(tube)
            self.status.end_cycle(ok)
            results.append((tube['cls'], ok))
            if ok:
                consecutive_fail = 0
            else:
                consecutive_fail += 1
                self.skill.recover(f'ống {tube["cls"]} thất bại')
                if consecutive_fail >= int(self.cfg.max_consecutive_failures):
                    self.get_logger().error(
                        f'{consecutive_fail} ống liên tiếp thất bại — DỪNG chu kỳ để '
                        'người vận hành kiểm tra (đừng gắp mù tiếp).')
                    break

        n_ok = sum(1 for _, ok in results if ok)
        self.get_logger().info(f'═══ KẾT THÚC: {n_ok}/{len(results)} ống thành công ═══')
        for cls, ok in results:
            self.get_logger().info(f'  {"✓" if ok else "✗"} {cls}')
        self.status.set(State.DONE, self.status.summary(),
                        slots={str(k): v for k, v in self._slot_occupied.items()})
        self.skill.go_home()
        self.status.set(State.IDLE, 'chờ /tube_rack/run', log=False)

    def _scan(self):
        fake = str(self.cfg.fake_tubes or '[]')
        if fake.strip() not in ('', '[]'):
            try:
                items = json.loads(fake)
            except ValueError:
                self.get_logger().error('fake_tubes không phải JSON hợp lệ.')
                return []
            self.get_logger().warn(f'Dùng fake_tubes ({len(items)} ống) — bỏ qua camera.')
            return [{'cls': str(t.get('class', t.get('cls', 'unknown'))).lower(),
                     'x': float(t['x']), 'y': float(t['y']), 'z': float(t['z']),
                     'yaw': float(t.get('yaw', 0.0)), 'frames': 0} for t in items]
        self.get_logger().info(
            f'Chờ detection tối đa {float(self.cfg.detection_wait_s):.0f}s …')
        return self.stack.detection.wait_for_detections(
            self.cfg.detection_wait_s,
            frames=self.cfg.median_frames, collect_s=self.cfg.median_collect_s,
            assoc_radius=self.cfg.detection_assoc_radius_m,
            min_hits_ratio=self.cfg.detection_min_hits_ratio)

    def _sort_tubes(self, tubes):
        """Gắp ống GẦN trước: mỗi lần với ra xa hơn thì phía trong đã trống,
        tay không phải lia qua đầu ống chưa gắp."""
        order = str(self.cfg.pick_order)
        if order == 'detection':
            return tubes
        reverse = order == 'farthest_first'
        return sorted(tubes, key=lambda t: math.hypot(t['x'], t['y']), reverse=reverse)

    # ── gắp 1 ống → cắm 1 slot ──────────────────────────────────────────
    def _pick_one(self, tube):
        self.status.set(State.PLANNING, f'{tube["cls"]}')
        slot = self._choose_slot(tube['cls'])
        if slot is None:
            self.get_logger().warn(f'Hết slot trống cho màu {tube["cls"]} — bỏ ống này.')
            return False

        # --- kế hoạch gắp ---
        grasp = self.skill.plan_grasp(tube['x'], tube['y'], tube['z'], yaw=tube['yaw'])
        if grasp is None:
            return False

        # --- kế hoạch cắm (TRƯỚC khi kẹp: hỏng thì chưa cầm gì trong tay) ---
        (ix, iy, iz), tilt = self._insert_pose(slot)
        ladder = self._place_ladder(tilt)
        hover_z = iz + float(self.cfg.hover_clearance)
        # Ghé cột thẳng đứng trên tư thế trung chuyển TRƯỚC khi vươn ra giá
        # (key_point: (0.18, 0, z+0.08)) ⇒ đoạn quét ngang không đi qua đầu các
        # ống đã cắm. via=None thì chuỗi vẫn chạy, chỉ mất lớp bảo hiểm này.
        via = self.skill.plan_via(hover_z, seed=grasp.lift.joints, label='VIA-GIÁ')
        hover = self.skill.plan_pose(ix, iy, hover_z, ladder,
                                     seed=(via.joints if via else grasp.lift.joints),
                                     label='HOVER')
        insert = None
        if hover is not None:
            insert = self.skill.plan_pose(ix, iy, iz, [hover.pitch], wrist=hover.wrist,
                                          seed=hover.joints, label='INSERT')
        if insert is None:
            self.get_logger().error(
                f'Slot {slot} không với tới — bỏ ống này (chưa kẹp nên an toàn). '
                f'{self.stack.kin.reach_report(ix, iy, iz, tilt)}')
            return False
        self.get_logger().info(
            f'  Kế hoạch: {self.skill.describe(grasp.pre, grasp.grasp, grasp.lift, via, hover, insert)} '
            f'→ slot {slot}')

        # --- chấp hành ---
        if not self.skill.grasp_at(grasp):
            return False
        if not self.skill.release_at(insert, hover=hover, via=via, state=State.INSERT):
            return False

        self._slot_occupied[slot] = True
        self.status.set(State.DONE, f'ống {tube["cls"]} → slot {slot}',
                        slots={str(k): v for k, v in self._slot_occupied.items()})
        return True

    # ── khởi động ───────────────────────────────────────────────────────
    def startup(self):
        self.check_rack_reachable()
        if not self.stack.executor.wait_ready(timeout=15.0):
            self.status.fault('MoveIt / joint_states chưa sẵn sàng')
            return
        self.get_logger().info(
            f'tube_rack sẵn sàng (dry_run={self.cfg.dry_run}, '
            f'{len(self.slots)} slot, pick_order={self.cfg.pick_order}).')
        if self.cfg.auto_start:
            self.get_logger().info('auto_start=true → chạy chu kỳ ngay.')
            self._run_event.set()


def main(args=None):
    rclpy.init(args=args)
    node = TubeRackNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.startup()
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.stack.executor.request_abort()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
