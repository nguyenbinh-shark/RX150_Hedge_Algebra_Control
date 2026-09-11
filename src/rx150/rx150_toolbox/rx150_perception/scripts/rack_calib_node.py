#!/usr/bin/env python3
"""
rack_calib_node — HIỆU CHUẨN VỊ TRÍ GIÁ ĐỠ ỐNG NGHIỆM bằng AprilTag.

Cùng một khuôn với hiệu chuẩn camera, chỉ khác đối tượng đo:

    đo bằng camera  →  ghi ra YAML  →  node khác NẠP LẠI, không đo lại nữa

    camera:  ./rx150.sh calib      → static_transforms.yaml → static_trans_pub
    GIÁ:     ./rx150.sh rack-calib → rack_pose.yaml         → tube_rack_node

TẠI SAO CẦN:
  tube_rack_params.yaml tả giá bằng slot0_x/y/z + rack_yaw_deg + slot_spacing,
  tức MỘT HÀNG slot thẳng đo bằng thước. Giá thật có 4 lỗ ở 4 ĐỈNH một hình chữ
  nhật 120 × 50 mm — mô hình hàng thẳng không tả được, và mỗi lần xê dịch giá là
  phải bò ra đo tay lại. Dán một AprilTag lên giá thì camera đo hộ: một lần snap
  ra đủ 4 miệng lỗ + TRỤC LỖ trong rx150/base_link.

  Sai số của phép đo này = sai số calib camera + sai số đọc tag. Nên B2 (calib
  camera) PHẢI xong trước; hiệu chuẩn giá trên nền TF camera sai chỉ là chép lại
  cái sai đó vào một file khác.

HÌNH HỌC (mặc định — sửa trong config/rack_calib.yaml):
  Tag ở TRUNG ĐIỂM CẠNH DÀI GẦN của hình chữ nhật, 4 lỗ ở 4 đỉnh:

        xa   (2)─────────────────(3)      v = rect_short_m      = 0.050
              │                   │
              │         ▲ v       │
        gần   (0)───────┼───────(1)       u = ±rect_long_m/2 = ±0.060
                       TAG ──► u

  Chỉ số slot 0..3 ở trên chính là chỉ số mà color_slot_map trong
  tube_rack_params.yaml đang dùng (pink→0, blue→1, green→2, yellow→3).
  Muốn đổi thứ tự thì sửa slot_index_map, ĐỪNG đổi hình học.

  QUY ƯỚC KHUNG TAG của apriltag_ros (common_functions.cpp:260 — không phải quy
  ước "y down" của AprilTag gốc, nó đã đổi khung trước khi publish):
      "looking straight at the tag: x is right, y is up, z is towards you"
  Nên khi tag dán NGỬA trên mặt giá (mount: top_flat):
      +x_tag = dọc cạnh dài (u)     +y_tag = hướng sang cạnh xa (v)
      +z_tag = TRỤC LỖ, hướng lên   ⇒ ma trận mount = I
  Dán tag quay ngược 180° thì ĐỪNG đảo dấu lung tung: đặt yaw_offset_deg: 180.
  (Đảo một trục là phép PHẢN CHIẾU chứ không phải phép quay — không dán ra được
  tình huống đó; đảo cả hai trục thì đúng bằng quay 180°.)

  Tag dán trên MẶT ĐỨNG phía trước giá: mount: front_vertical, khi đó
  hole_z_offset_m = chiều cao từ tâm tag lên mặt phẳng miệng lỗ.

BA CHẾ ĐỘ (tham số `mode`):
  snap     đo N mẫu tag → trung vị vị trí + trung bình quaternion (Markley) →
           ghi rack_pose.yaml → phát luôn TF tĩnh để soi trong RViz.
  publish  KHÔNG cần camera: nạp rack_pose.yaml và phát TF tĩnh
           base_frame → tube_rack (+ tube_rack/slot0..3). Đây là "static_trans_pub
           của giá" — chỉ để nhìn/đo, tube_rack_node đọc thẳng file chứ không
           tra TF này.
  watch    đo liên tục và in ĐỘ LỆCH so với file đã lưu (mm / độ) — dùng để biết
           "giá có bị đụng xê dịch không" mà không phải hiệu chuẩn lại.

⚠️ TAG GIÁ VÀ TAG TAY GẮP CÙNG id 0 — AprilTag chỉ phân biệt tag bằng id, nên hai
cái này về nguyên tắc không tách được. Node tách bằng đường vòng: tag tay gắp là
CON của rx150/ee_arm_link nên robot_state_publisher luôn biết nó ở đâu trong
base_frame; ứng viên nào rơi trong ignore_radius_m (8 cm) quanh rx150/ar_tag_link
hoặc rx150/ee_gripper_link thì bị loại. Nhờ vậy hai tag cùng id 0 vẫn dùng chung
được, KHÔNG phải in tag mới.
  • Cần T1 chạy (không có robot_state_publisher thì không có frame để lọc).
  • Đừng để tay máy đứng ngay trên giá lúc snap — khi đó tag GIÁ nằm trong bán
    kính lọc và bị loại nhầm (node báo WARN đúng tình huống này).
  • Còn >1 ứng viên sau khi lọc ⇒ BỎ khung, không đoán.
Ngược lại, lúc ./rx150.sh calib (hiệu chuẩn camera) thì cất TAG GIÁ đi.

CHẠY (T1 + T2 đã lên, tag phải nằm trong khung hình):
  ./rx150.sh rack-calib      # snap + ghi file
  ./rx150.sh rack-watch      # kiểm tra trôi
  ./rx150.sh reach           # sau khi snap: 4 lỗ có với tới được không

KHÔNG bật driver camera ở đây (T1 đang giữ D435i) — node chỉ đọc detection có sẵn
trên /rack_tag/tag_detections do rack_tag.launch.py sinh ra.
"""

import json
import math
import os
import shutil
from datetime import datetime

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

# Ảnh chồng hình là phần TÙY CHỌN: thiếu cv_bridge/cv2 thì marker RViz vẫn chạy,
# chỉ mất khung ảnh. Không cho cả node chết vì một tính năng để nhìn.
try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except ImportError:                                       # pragma: no cover
    cv2 = None
    CvBridge = None
    _HAS_CV = False

RESULT_FILE = 'rack_pose.yaml'
PROBE_FILE = 'rack_calib.yaml'      # file cùng thư mục, chắc chắn có symlink install→src

# Ma trận R_tag_rack: các CỘT là (x̂, ŷ, ẑ) của khung GIÁ viết trong khung TAG.
#   x̂ = dọc cạnh dài (u)   ŷ = hướng ra cạnh xa (v)   ẑ = trục lỗ (hướng lên)
MOUNTS = {
    # tag dán ngửa trên mặt giá: khung giá ≡ khung tag
    'top_flat': np.eye(3),
    # tag dán trên mặt đứng phía trước: trục lỗ = +y_tag (lên), cạnh xa = −z_tag
    # (lùi vào trong giá). det = +1 nên vẫn thuận tay phải.
    'front_vertical': np.array([[1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0],
                                [0.0, -1.0, 0.0]]),
}

DEFAULTS = {
    'mode': 'snap',                        # snap | publish | watch
    # ── nguồn đo ────────────────────────────────────────────────────────
    'detections_topic': '/rack_tag/tag_detections',
    'tag_id': 1,
    'base_frame': 'rx150/base_link',
    'rack_frame': 'tube_rack',
    'samples': 30,
    'sample_period_s': 0.05,               # giãn mẫu ra để không lấy trùng 1 frame
    'sample_timeout_s': 25.0,
    # Tag GIÁ và tag CÁNH TAY cùng id 0. Tag cánh tay là con của
    # rx150/ee_arm_link nên vị trí của nó trong base_frame LUÔN biết được từ TF —
    # dùng chính điều đó để loại nó ra, khỏi phải in tag mới.
    'ignore_near_frames': '["rx150/ar_tag_link", "rx150/ee_gripper_link"]',
    'ignore_radius_m': 0.08,
    'warn_spread_mm': 3.0,
    'warn_spread_deg': 1.5,
    # ── hình học giá trong KHUNG TAG ───────────────────────────────────
    'mount': 'top_flat',                   # top_flat | front_vertical
    'rect_long_m': 0.120,
    'rect_short_m': 0.050,
    'hole_z_offset_m': 0.0,
    'yaw_offset_deg': 0.0,                 # tag dán quay ngược ⇒ 180
    'hole_offsets': '[]',                  # JSON [[u,v,h], …] — đè hẳn hình chữ nhật
    'slot_index_map': [0, 1, 2, 3],
    # ── hộp vật cản suy ra cho planning scene ──────────────────────────
    'box_margin_m': 0.01,
    'box_below_m': 0.10,                   # giá đặc bên dưới miệng lỗ
    'box_above_m': 0.0,                    # 0 = không chắn đường cắm từ trên xuống
    # ── đầu ra ─────────────────────────────────────────────────────────
    'output_path': '',                     # rỗng ⇒ <src>/config/rack_pose.yaml
    'publish_after_snap': True,
    'exit_after_snap': False,
    'watch_rate_hz': 1.0,
    # ── GUI: xem bằng mắt trước khi tin vào số ─────────────────────────
    'publish_markers': True,               # ~/markers → RViz (lỗ, viền giá, trục lỗ)
    'publish_debug_image': True,           # ~/image_debug → chiếu 4 lỗ NGƯỢC lên ảnh
    'image_topic': '/camera/camera/color/image_raw',
    'camera_info_topic': '/camera/camera/color/camera_info',
    'debug_rate_hz': 5.0,                  # đủ mượt để soi, không giành CPU với YOLO
    'marker_rate_hz': 2.0,
    'hole_radius_m': 0.009,                # bán kính lỗ, chỉ để VẼ
    'hole_depth_m': 0.06,                  # chiều sâu lỗ, chỉ để VẼ
}


# ───────────────────────────── SE(3) thuần numpy ───────────────────────────
# Không dùng tf2_geometry_msgs.do_transform_pose: chữ ký hàm đó đổi giữa các bản
# Humble (Pose vs PoseStamped). Tự nhân ma trận thì không bao giờ hỏng.

def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    """3x3 → (x, y, z, w). Shepperd: chọn nhánh theo phần tử trội, ổn định số."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


def make_T(p, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q)
    T[:3, 3] = p
    return T


def T_inv(T):
    """Nghịch đảo phép biến đổi thuần nhất (R,t) → (Rᵀ, −Rᵀt)."""
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def quat_mean(quats):
    """Trung bình quaternion (Markley): vector riêng ứng trị riêng lớn nhất của
    Σ qqᵀ. Xử lý đúng chuyện ±q là CÙNG một phép quay."""
    M = np.zeros((4, 4))
    for q in quats:
        q = np.asarray(q, dtype=float)
        q = q / np.linalg.norm(q)
        M += np.outer(q, q)
    w, v = np.linalg.eigh(M)
    q = v[:, int(np.argmax(w))]
    return q / np.linalg.norm(q)


def angle_between_R(R1, R2):
    c = (np.trace(R1.T @ R2) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rot_z(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class RackCalib(Node):
    def __init__(self):
        super().__init__('rack_calib')
        for name, value in DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in DEFAULTS}

        self.mode = str(self.p['mode']).strip().lower()
        if self.mode not in ('snap', 'publish', 'watch'):
            raise SystemExit(f"mode không hợp lệ: {self.mode!r} (snap | publish | watch)")
        if self.p['mount'] not in MOUNTS:
            raise SystemExit(f"mount không hợp lệ: {self.p['mount']!r} "
                             f"({' | '.join(MOUNTS)})")

        self.base_frame = str(self.p['base_frame'])
        self.rack_frame = str(self.p['rack_frame'])
        self.output_path = self._resolve_output_path()
        self.static_tf = StaticTransformBroadcaster(self)

        self.samples = []            # [(t, p_base(3), q_base(4))]
        self.frames_seen = 0
        self.id_seen = set()
        self.ambiguous = 0
        self.arm_tag_dropped = 0
        self.tf_fallback = 0
        self._last_take = 0.0
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._done = False
        self.saved = None
        self.live = None             # nghiệm ĐANG đo (snap/watch)
        self.saved_view = None       # nghiệm đã lưu trong file

        # TF listener dựng cho MỌI chế độ: ảnh chồng hình cần TF camera←đế kể cả
        # khi chỉ đang publish lại file cũ.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.mode == 'publish':
            self.saved = self._load_result(required=True)
            self.saved_view = self._view_from_saved(self.saved)
            self._publish_result(self.saved)
            self._setup_gui()
            self.get_logger().info(
                f'Đang phát TF tĩnh {self.base_frame} → {self.rack_frame} (+ 4 slot) '
                f'từ {self.output_path}. Ctrl+C để dừng.')
            return

        # snap/watch: cần đo ⇒ cần detection từ apriltag_ros
        try:
            from apriltag_ros.msg import AprilTagDetectionArray
        except ImportError as exc:                       # pragma: no cover
            raise SystemExit(
                'Thiếu apriltag_ros — source ~/apriltag_ws/install/setup.bash '
                '(source_all.sh đã làm việc này).') from exc
        self.create_subscription(AprilTagDetectionArray,
                                 str(self.p['detections_topic']), self._on_tags, 10)
        self._setup_gui()
        if self.mode == 'watch':
            self.saved = self._load_result(required=False)
            self.saved_view = self._view_from_saved(self.saved) if self.saved else None
            self.create_timer(1.0 / max(0.1, float(self.p['watch_rate_hz'])),
                              self._on_watch_tick)
            self.get_logger().info(
                f"watch: so pose đo được với {self.output_path if self.saved else '(chưa có file)'}"
                ' — Ctrl+C để dừng.')
        else:
            self.create_timer(0.5, self._on_snap_tick)
            self.get_logger().info(
                f"snap: cần {int(self.p['samples'])} mẫu tag id={int(self.p['tag_id'])} "
                f"trên {self.p['detections_topic']} → {self.output_path}")

    # ── đường ghi kết quả ───────────────────────────────────────────────
    def _resolve_output_path(self):
        """Trả ĐƯỜNG TRONG src/ nếu có thể.

        BẪY đã trả giá một lần với armtag: install/ ở workspace này dựng bằng
        --symlink-install, mỗi FILE config là một symlink về src/. File MỚI tạo
        thẳng trong install/ lại là file thật ⇒ `colcon build` xoá sạch. Nên bám
        theo symlink của một file chắc chắn tồn tại cùng thư mục để lần ra src/.
        """
        raw = str(self.p['output_path'] or '').strip()
        share_cfg = os.path.join(get_package_share_directory('rx150_perception'), 'config')
        # Chỉ đường MẶC ĐỊNH mới được chép thêm một bản vào share/. Người dùng chỉ
        # định output_path là muốn file nằm ĐÚNG chỗ đó, không phải hai chỗ.
        self.auto_path = not raw
        if raw:
            return raw if os.path.isabs(raw) else os.path.join(share_cfg, raw)
        target = os.path.join(share_cfg, RESULT_FILE)
        if os.path.islink(target):
            return os.path.realpath(target)
        probe = os.path.join(share_cfg, PROBE_FILE)
        if os.path.islink(probe):
            return os.path.join(os.path.dirname(os.path.realpath(probe)), RESULT_FILE)
        return target

    # ── thu mẫu ─────────────────────────────────────────────────────────
    def _on_tags(self, msg):
        self.frames_seen += 1
        want = int(self.p['tag_id'])
        hits = []
        for det in msg.detections:
            if len(det.id) != 1:
                continue
            self.id_seen.add(int(det.id[0]))
            if int(det.id[0]) == want:
                hits.append(det)
        if not hits:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_take < float(self.p['sample_period_s']):
            return

        # Quy hết ứng viên về base_frame TRƯỚC khi chọn: phải biết chúng nằm ở đâu
        # thì mới loại được cái nào là tag cánh tay.
        stamp = stamp_to_sec(msg.header.stamp)
        cam_frame = msg.header.frame_id or hits[0].pose.header.frame_id
        try:
            T_base_cam = self._lookup(self.base_frame, cam_frame, stamp)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return
        cands = []
        for det in hits:
            pos = det.pose.pose.pose.position
            ori = det.pose.pose.pose.orientation
            T = T_base_cam @ make_T(np.array([pos.x, pos.y, pos.z]),
                                    np.array([ori.x, ori.y, ori.z, ori.w]))
            cands.append(T)

        kept, dropped = self._drop_arm_tag(cands, stamp)
        if dropped:
            self.arm_tag_dropped += dropped
        # Tag GIÁ và tag CÁNH TAY cùng id 0 — AprilTag chỉ phân biệt bằng id. Lọc
        # theo TF ở trên gỡ được phần lớn; còn hơn một ứng viên thì KHÔNG đoán:
        # thà bỏ khung còn hơn lấy nhầm rồi cắm ống vào chỗ trống.
        if len(kept) > 1:
            self.ambiguous += 1
            self.get_logger().warn(
                f'Còn {len(kept)} tag id {want} sau khi lọc tag cánh tay — BỎ khung này. '
                'Cất bớt tag ra khỏi khung hình, hoặc in tag giá bằng id khác rồi sửa '
                'rack_tag.yaml + rack_calib.yaml.',
                throttle_duration_sec=3.0)
            return
        if not kept:
            self.get_logger().warn(
                f'Khung này chỉ thấy tag CÁNH TAY (id {want}), không thấy tag giá — '
                'đưa giá vào khung hình. Nếu tay máy đang che/đứng ngay trên giá thì '
                'chính tag giá bị lọc nhầm: dời tay máy đi, hoặc giảm ignore_radius_m.',
                throttle_duration_sec=5.0)
            return
        T = kept[0]
        self._last_take = now
        self.samples.append((now, T[:3, 3].copy(), R_to_quat(T[:3, :3])))

    def _drop_arm_tag(self, cands, stamp_sec):
        """Loại ứng viên nào nằm sát một frame đã biết là TAG CÁNH TAY.

        Tag trên tay gắp là con của rx150/ee_arm_link ⇒ robot_state_publisher luôn
        biết nó ở đâu trong base_frame. Ứng viên rơi trong bán kính ignore_radius_m
        quanh frame đó thì chính là nó, không phải giá. Nhờ vậy hai tag CÙNG id 0
        vẫn dùng chung được mà không phải in lại tag.

        Frame không có trong cây TF (URDF dựng với show_ar_tag:=false) thì bỏ qua,
        KHÔNG coi là lỗi.
        """
        try:
            frames = json.loads(str(self.p['ignore_near_frames'] or '[]'))
        except json.JSONDecodeError:
            self.get_logger().error('ignore_near_frames không phải JSON — bỏ qua bộ lọc.',
                                    throttle_duration_sec=30.0)
            return list(cands), 0
        radius = float(self.p['ignore_radius_m'])
        if not frames or radius <= 0.0:
            return list(cands), 0

        centres = []
        for frame in frames:
            try:
                centres.append(self._lookup(self.base_frame, str(frame), stamp_sec)[:3, 3])
            except RuntimeError:
                continue                     # frame không tồn tại: không phải lỗi
        if not centres:
            self.get_logger().warn(
                'Không tra được frame nào trong ignore_near_frames — bộ lọc tag cánh '
                'tay đang TẮT. Chạy T1 (robot_state_publisher) thì mới có các frame đó.',
                throttle_duration_sec=30.0)
            return list(cands), 0

        kept, dropped = [], 0
        for T in cands:
            p = T[:3, 3]
            if any(float(np.linalg.norm(p - c)) < radius for c in centres):
                dropped += 1
            else:
                kept.append(T)
        return kept, dropped

    def _lookup(self, target, source, stamp_sec):
        """TF target←source tại stamp ảnh; ngoại suy thì lùi về bản mới nhất và
        ĐẾM vào tf_fallback để báo cáo trung thực."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source,
                Time(seconds=int(stamp_sec), nanoseconds=int((stamp_sec % 1) * 1e9)),
                timeout=Duration(seconds=0.05))
        except Exception:                                  # noqa: BLE001
            try:
                tf = self.tf_buffer.lookup_transform(target, source, Time(),
                                                     timeout=Duration(seconds=0.2))
                self.tf_fallback += 1
            except Exception as exc:                       # noqa: BLE001
                raise RuntimeError(
                    f'TF {target}←{source}: {exc}. Chạy T2 (./rx150.sh t2) chưa?') from exc
        t, r = tf.transform.translation, tf.transform.rotation
        return make_T(np.array([t.x, t.y, t.z]), np.array([r.x, r.y, r.z, r.w]))

    # ── hình học: khung tag → 4 miệng lỗ ────────────────────────────────
    def _hole_offsets(self):
        """[(u, v, h)] trong KHUNG GIÁ, theo đúng thứ tự slot 0..3."""
        raw = str(self.p['hole_offsets'] or '[]').strip()
        try:
            explicit = json.loads(raw) if raw else []
        except json.JSONDecodeError as exc:
            raise SystemExit(f'hole_offsets không phải JSON hợp lệ: {exc}') from exc
        if explicit:
            holes = [tuple(float(c) for c in item) for item in explicit]
            if any(len(h) != 3 for h in holes):
                raise SystemExit('hole_offsets: mỗi phần tử phải là [u, v, h]')
        else:
            half = float(self.p['rect_long_m']) / 2.0
            far = float(self.p['rect_short_m'])
            h = float(self.p['hole_z_offset_m'])
            holes = [(-half, 0.0, h), (half, 0.0, h), (-half, far, h), (half, far, h)]
        order = [int(i) for i in (self.p['slot_index_map'] or [])]
        if len(order) != len(holes):
            # hole_offsets đè vào với số lỗ khác 4 mà slot_index_map vẫn để mặc
            # định thì bám theo số lỗ THẬT; chỉ báo lỗi khi người dùng thực sự
            # yêu cầu một thứ tự không khớp.
            if order and order != list(range(len(order))):
                raise SystemExit(f'slot_index_map có {len(order)} phần tử nhưng hình '
                                 f'học đang có {len(holes)} lỗ')
            order = list(range(len(holes)))
        if sorted(order) != list(range(len(holes))):
            raise SystemExit(
                f'slot_index_map {order} phải là hoán vị của 0..{len(holes) - 1}')
        return [holes[i] for i in order]

    def _R_tag_rack(self):
        return MOUNTS[self.p['mount']] @ rot_z(math.radians(float(self.p['yaw_offset_deg'])))

    def _solve(self, p_tag, q_tag):
        """Từ pose TAG trong base → pose GIÁ + 4 miệng lỗ + trục lỗ."""
        R_base_rack = quat_to_R(q_tag) @ self._R_tag_rack()
        holes = self._hole_offsets()
        slots = [tuple(np.asarray(p_tag) + R_base_rack @ np.array(h)) for h in holes]
        axis = R_base_rack @ np.array([0.0, 0.0, 1.0])
        axis = axis / np.linalg.norm(axis)
        long_edge = R_base_rack @ np.array([1.0, 0.0, 0.0])
        return {
            'R_base_rack': R_base_rack,
            'slots': slots,
            'axis': axis,
            'tilt_deg': math.degrees(math.acos(max(-1.0, min(1.0, float(axis[2]))))),
            'long_edge_yaw_deg': math.degrees(math.atan2(float(long_edge[1]),
                                                         float(long_edge[0]))),
        }

    def _rack_box(self, slots):
        """AABB bao 4 lỗ — planning scene của tube_rack chỉ nhận hộp THẲNG TRỤC,
        nên giá xoay yaw phải lấy bao hình chứ không phải hộp xoay theo."""
        m = float(self.p['box_margin_m'])
        xs = [s[0] for s in slots]
        ys = [s[1] for s in slots]
        zs = [s[2] for s in slots]
        z_lo = min(zs) - float(self.p['box_below_m'])
        z_hi = max(zs) + float(self.p['box_above_m'])
        return {
            'x': (min(xs) + max(xs)) / 2.0,
            'y': (min(ys) + max(ys)) / 2.0,
            'z': (z_lo + z_hi) / 2.0,
            'size_x': max(0.01, (max(xs) - min(xs)) + 2 * m),
            'size_y': max(0.01, (max(ys) - min(ys)) + 2 * m),
            'size_z': max(0.01, z_hi - z_lo),
        }

    # ── snap ────────────────────────────────────────────────────────────
    def _on_snap_tick(self):
        if self._done:
            return
        need = int(self.p['samples'])
        elapsed = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        if len(self.samples) >= need:
            self._done = True
            self._finish_snap()
            return
        if elapsed > float(self.p['sample_timeout_s']):
            self._done = True
            self._fail_snap()
            return
        if int(elapsed) % 3 == 0:
            self.get_logger().info(
                f'… {len(self.samples)}/{need} mẫu ({self.frames_seen} frame detector)',
                throttle_duration_sec=2.5)

    def _fail_snap(self):
        want = int(self.p['tag_id'])
        why = []
        if self.frames_seen == 0:
            why.append(f"KHÔNG có frame nào trên {self.p['detections_topic']} — "
                       'rack_tag.launch.py chưa chạy, hoặc detector không nhận được ảnh '
                       "(nhớ remap '~/image_rect', xem rack_tag.launch.py).")
        elif not self.id_seen:
            why.append('detector chạy nhưng KHÔNG thấy tag nào — tag ngoài khung hình, '
                       'quá nghiêng, hoặc sai tag_family.')
        elif want not in self.id_seen:
            why.append(f'thấy tag id {sorted(self.id_seen)} nhưng cấu hình đang đòi id '
                       f'{want} — sửa tag_id trong rack_calib.yaml VÀ rack_tag.yaml.')
        elif self.ambiguous:
            why.append(f'BỎ {self.ambiguous} khung vì sau khi lọc tag cánh tay vẫn còn '
                       f'NHIỀU tag id {want} — cất bớt tag ra khỏi khung rồi snap lại.')
        elif self.arm_tag_dropped:
            why.append(f'chỉ thấy TAG CÁNH TAY ({self.arm_tag_dropped} lần), không thấy '
                       'tag giá — đưa giá vào khung hình. Nếu tay máy đang đứng ngay '
                       'trên giá thì chính tag giá bị lọc nhầm: dời tay máy đi.')
        else:
            why.append('thấy tag nhưng không tra được TF — xem WARN phía trên.')
        self.get_logger().error(
            f'HỎNG: chỉ lấy được {len(self.samples)}/{int(self.p["samples"])} mẫu sau '
            f'{self.p["sample_timeout_s"]:.0f}s. ' + ' '.join(why))
        raise SystemExit(1)

    def _finish_snap(self):
        pos = np.array([s[1] for s in self.samples])
        quats = [s[2] for s in self.samples]
        p_tag = np.median(pos, axis=0)
        q_tag = quat_mean(quats)
        spread_mm = (pos.max(axis=0) - pos.min(axis=0)) * 1000.0
        R_mean = quat_to_R(q_tag)
        ang = [math.degrees(angle_between_R(R_mean, quat_to_R(q))) for q in quats]

        sol = self._solve(p_tag, q_tag)
        box = self._rack_box(sol['slots'])
        result = {
            'p_tag': p_tag, 'q_tag': q_tag, 'sol': sol, 'box': box,
            'spread_mm': spread_mm, 'angle_spread_deg': max(ang),
            'n': len(self.samples),
        }
        self.live = self._view_from_sol(p_tag, sol, 'LIVE')
        self._report(result)
        self._write_result(result)
        if bool(self.p['publish_after_snap']):
            self._publish_result(self._as_saved(result))
        if bool(self.p['exit_after_snap']):
            raise SystemExit(0)

    def _report(self, r):
        sol, box = r['sol'], r['box']
        lines = [
            '',
            '═══════════ HIỆU CHUẨN GIÁ ĐỠ ỐNG NGHIỆM ═══════════',
            f'  mẫu             : {r["n"]}  (TF lùi về bản mới nhất: {self.tf_fallback}'
            + (f', BỎ {self.ambiguous} khung vì trùng tag id' if self.ambiguous else '')
            + (f', lọc ra {self.arm_tag_dropped} lần tag CÁNH TAY'
               if self.arm_tag_dropped else '') + ')',
            f'  tag {int(self.p["tag_id"]):<3d}         : ({r["p_tag"][0]:+.4f}, '
            f'{r["p_tag"][1]:+.4f}, {r["p_tag"][2]:.4f}) m trong {self.base_frame}',
            f'  tản mát vị trí  : x {r["spread_mm"][0]:.1f} | y {r["spread_mm"][1]:.1f} | '
            f'z {r["spread_mm"][2]:.1f} mm (max−min)',
            f'  tản mát góc     : {r["angle_spread_deg"]:.2f}°',
            f'  trục lỗ         : ({sol["axis"][0]:+.3f}, {sol["axis"][1]:+.3f}, '
            f'{sol["axis"][2]:+.3f})  ⇒ nghiêng {sol["tilt_deg"]:.1f}° so với phương '
            'thẳng đứng (= pitch ee lúc cắm)',
            f'  cạnh dài hướng  : {sol["long_edge_yaw_deg"]:+.1f}° trong mặt phẳng XY',
            '  MIỆNG LỖ (m, khung ' + self.base_frame + '):',
        ]
        for i, s in enumerate(sol['slots']):
            lines.append(f'    slot {i}: ({s[0]:+.4f}, {s[1]:+.4f}, {s[2]:.4f})   '
                         f'r = {math.hypot(s[0], s[1]):.3f} m')
        lines.append(f'  hộp vật cản     : tâm ({box["x"]:+.3f}, {box["y"]:+.3f}, '
                     f'{box["z"]:.3f}) cỡ ({box["size_x"]:.3f}, {box["size_y"]:.3f}, '
                     f'{box["size_z"]:.3f})')
        lines.append('════════════════════════════════════════════════════')
        self.get_logger().info('\n'.join(lines))

        warn_mm = float(self.p['warn_spread_mm'])
        if float(np.max(r['spread_mm'])) > warn_mm:
            self.get_logger().warn(
                f'Tản mát {np.max(r["spread_mm"]):.1f} mm > {warn_mm:.1f} mm — tag rung, '
                'quá nghiêng, quá xa, hoặc ảnh mờ. Đưa tag gần/vuông góc camera hơn rồi '
                'snap lại; đừng lấy số này làm chuẩn.'
                + (' Tản mát HÀNG TRĂM mm thì không phải nhiễu ảnh: gần như chắc chắn '
                   'đang có HAI nguồn phát TF world↔camera nên TF nhảy giữa hai giá trị '
                   '(RUNBOOK §2). Kiểm: ros2 run tf2_ros tf2_echo '
                   'camera_color_optical_frame rx150/base_link — số phải ĐỨNG YÊN.'
                   if float(np.max(r['spread_mm'])) > 50.0 else ''))
        if r['angle_spread_deg'] > float(self.p['warn_spread_deg']):
            self.get_logger().warn(
                f'Tản mát góc {r["angle_spread_deg"]:.2f}° — hướng trục lỗ kém tin cậy. '
                'Tag nhỏ ⇒ hướng luôn nhiễu hơn vị trí; tăng size tag nếu cần trục chuẩn.')

    # ── ghi / đọc file kết quả ──────────────────────────────────────────
    def _as_saved(self, r):
        """Đưa kết quả snap về đúng dạng dict như khi đọc lại từ YAML."""
        sol = r['sol']
        q = R_to_quat(sol['R_base_rack'])
        return {
            'base_frame': self.base_frame,
            'rack_frame': self.rack_frame,
            'rack_pose': {'x': float(r['p_tag'][0]), 'y': float(r['p_tag'][1]),
                          'z': float(r['p_tag'][2]), 'qx': float(q[0]), 'qy': float(q[1]),
                          'qz': float(q[2]), 'qw': float(q[3])},
            'slot_axis': [float(v) for v in sol['axis']],
            'rack_tilt_deg': float(sol['tilt_deg']),
            'slots': [[float(c) for c in s] for s in sol['slots']],
            'rack_box': r['box'],
        }

    def _write_result(self, r):
        sol, box = r['sol'], r['box']
        q = R_to_quat(sol['R_base_rack'])
        p = r['p_tag']
        holes = self._hole_offsets()
        text = [
            '# rack_pose.yaml — VỊ TRÍ GIÁ ĐỠ ỐNG NGHIỆM đo bằng AprilTag.',
            '#',
            '# SINH TỰ ĐỘNG bởi rack_calib_node (mode snap) — ĐỪNG sửa tay: lần snap sau',
            '# ghi đè sạch. Muốn đổi hình học thì sửa config/rack_calib.yaml rồi snap lại.',
            '#',
            '# tube_rack_node ĐỌC THẲNG file này (tham số rack_calib_file) — nó KHÔNG tra',
            '# TF lúc chạy, nên giá xê dịch thì phải snap lại chứ số cũ không tự đúng.',
            '#',
            f'# Đo lúc {datetime.now().isoformat(timespec="seconds")} · '
            f'{r["n"]} mẫu · tag id {int(self.p["tag_id"])} '
            f'· tản mát ≤ {float(np.max(r["spread_mm"])):.1f} mm / '
            f'{r["angle_spread_deg"]:.2f}°',
            '',
            f'base_frame: {self.base_frame}',
            f'rack_frame: {self.rack_frame}',
            'measurement:',
            f'  tag_id: {int(self.p["tag_id"])}',
            f'  samples: {r["n"]}',
            f'  calibrated_at: \'{datetime.now().isoformat(timespec="seconds")}\'',
            f'  spread_mm: [{r["spread_mm"][0]:.2f}, {r["spread_mm"][1]:.2f}, '
            f'{r["spread_mm"][2]:.2f}]',
            f'  angle_spread_deg: {r["angle_spread_deg"]:.3f}',
            f'  tf_fallback: {self.tf_fallback}',
            'geometry:',
            f'  mount: {self.p["mount"]}',
            f'  rect_long_m: {float(self.p["rect_long_m"]):.4f}',
            f'  rect_short_m: {float(self.p["rect_short_m"]):.4f}',
            f'  hole_z_offset_m: {float(self.p["hole_z_offset_m"]):.4f}',
            f'  yaw_offset_deg: {float(self.p["yaw_offset_deg"]):.1f}',
            '  hole_offsets_uvh:   # trong khung GIÁ, theo thứ tự slot 0..N',
        ]
        for i, h in enumerate(holes):
            text.append(f'    - [{h[0]:+.4f}, {h[1]:+.4f}, {h[2]:+.4f}]   # slot {i}')
        text += [
            '',
            f'# {self.base_frame} → {self.rack_frame}: gốc ở TAG, +z = trục lỗ,',
            '# +x = dọc cạnh dài, +y = hướng ra cạnh xa.',
            'rack_pose:',
            f'  x: {p[0]:.6f}',
            f'  y: {p[1]:.6f}',
            f'  z: {p[2]:.6f}',
            f'  qx: {q[0]:.9f}',
            f'  qy: {q[1]:.9f}',
            f'  qz: {q[2]:.9f}',
            f'  qw: {q[3]:.9f}',
            '',
            '# Trục lỗ (đơn vị) và độ nghiêng so với phương thẳng đứng. tube_rack_node',
            '# dùng đúng góc này làm pitch của ee lúc cắm (pitch = 0 ⇒ ee_z thẳng đứng).',
            f'slot_axis: [{sol["axis"][0]:.6f}, {sol["axis"][1]:.6f}, {sol["axis"][2]:.6f}]',
            f'rack_tilt_deg: {sol["tilt_deg"]:.3f}',
            f'long_edge_yaw_deg: {sol["long_edge_yaw_deg"]:.3f}',
            '',
            f'# MIỆNG LỖ trong {self.base_frame} (m). Chỉ số = chỉ số slot mà',
            '# color_slot_map (tube_rack_params.yaml) đang trỏ tới.',
            'slots:',
        ]
        for i, s in enumerate(sol['slots']):
            text.append(f'  - [{s[0]:.6f}, {s[1]:.6f}, {s[2]:.6f}]   # slot {i}  '
                        f'r={math.hypot(s[0], s[1]):.3f}m')
        text += [
            '',
            '# Hộp vật cản THẲNG TRỤC bao 4 lỗ (planning scene). Nóc hộp mặc định nằm ở',
            '# mặt phẳng miệng lỗ để không chắn đường cắm từ trên xuống.',
            'rack_box:',
            f'  x: {box["x"]:.6f}',
            f'  y: {box["y"]:.6f}',
            f'  z: {box["z"]:.6f}',
            f'  size_x: {box["size_x"]:.6f}',
            f'  size_y: {box["size_y"]:.6f}',
            f'  size_z: {box["size_z"]:.6f}',
            '',
        ]
        blob = '\n'.join(text)
        os.makedirs(os.path.dirname(self.output_path) or '.', exist_ok=True)
        with open(self.output_path, 'w', encoding='utf-8') as handle:
            handle.write(blob)
        self.get_logger().info(f'Đã ghi {self.output_path}')

        # Ghi thêm bản trong install/ nếu đó chưa phải symlink về đúng file vừa
        # ghi: chưa `colcon build` lần nào sau khi thêm file này thì share/ KHÔNG
        # có nó, và tube_rack_node tra theo share/ sẽ không thấy gì.
        if not getattr(self, 'auto_path', True):
            return
        share_copy = os.path.join(get_package_share_directory('rx150_perception'),
                                  'config', RESULT_FILE)
        if os.path.realpath(share_copy) != os.path.realpath(self.output_path):
            try:
                shutil.copyfile(self.output_path, share_copy)
                self.get_logger().warn(
                    f'Đã chép tạm sang {share_copy} để dùng NGAY. Bản này bị `colcon '
                    'build` xoá — chạy ./tools/build.sh --packages-select rx150_perception '
                    'để install/ trỏ symlink về src/ cho vĩnh viễn.')
            except OSError as exc:                        # noqa: BLE001
                self.get_logger().warn(f'Không chép được sang share/: {exc}')

    def _load_result(self, required):
        if not os.path.exists(self.output_path):
            if required:
                raise SystemExit(
                    f'Chưa có {self.output_path} — chạy `./rx150.sh rack-calib` trước.')
            return None
        with open(self.output_path, 'r', encoding='utf-8') as handle:
            doc = yaml.safe_load(handle) or {}
        if 'rack_pose' not in doc or 'slots' not in doc:
            raise SystemExit(f'{self.output_path} thiếu khoá rack_pose/slots — file hỏng.')
        return doc

    # ── phát TF tĩnh ────────────────────────────────────────────────────
    def _publish_result(self, saved):
        base = str(saved.get('base_frame', self.base_frame))
        rack = str(saved.get('rack_frame', self.rack_frame))
        pose = saved['rack_pose']
        stamp = self.get_clock().now().to_msg()

        def tf(parent, child, x, y, z, q):
            msg = TransformStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.translation.x = float(x)
            msg.transform.translation.y = float(y)
            msg.transform.translation.z = float(z)
            msg.transform.rotation.x = float(q[0])
            msg.transform.rotation.y = float(q[1])
            msg.transform.rotation.z = float(q[2])
            msg.transform.rotation.w = float(q[3])
            return msg

        q_rack = (pose['qx'], pose['qy'], pose['qz'], pose['qw'])
        R = quat_to_R(np.array(q_rack))
        p = np.array([pose['x'], pose['y'], pose['z']])
        msgs = [tf(base, rack, p[0], p[1], p[2], q_rack)]
        for i, s in enumerate(saved['slots']):
            local = R.T @ (np.array([float(c) for c in s]) - p)   # lỗ trong khung giá
            msgs.append(tf(rack, f'{rack}/slot{i}', local[0], local[1], local[2],
                           (0.0, 0.0, 0.0, 1.0)))
        # StaticTransformBroadcaster phải nhận CẢ danh sách trong MỘT lần gọi:
        # gọi nhiều lần thì bản sau đè /tf_static của bản trước ở vài bản Humble.
        self.static_tf.sendTransform(msgs)

    # ── GUI: marker RViz + ảnh chồng hình ───────────────────────────────
    # Số in ra terminal không trả lời được câu hỏi quan trọng nhất: "4 cái lỗ tính
    # ra có TRÙNG 4 cái lỗ thật không". Chiếu NGƯỢC toạ độ lỗ lên chính khung ảnh
    # camera thì mắt trả lời trong một giây — vòng tròn nằm trên lỗ hay lệch sang
    # bên cạnh. Đây là kiểm tra end-to-end thật sự: nó đi qua cả TF hiệu chuẩn
    # camera, cả pose tag, cả hình học khai trong rack_calib.yaml.
    def _setup_gui(self):
        self.marker_pub = None
        self.debug_pub = None
        self.bridge = None
        self.cam_K = None
        self.cam_D = None
        self._last_debug = 0.0

        if bool(self.p['publish_markers']):
            self.marker_pub = self.create_publisher(MarkerArray, '~/markers', 1)
            self.create_timer(1.0 / max(0.2, float(self.p['marker_rate_hz'])),
                              self._publish_markers)
        if not bool(self.p['publish_debug_image']):
            return
        if not _HAS_CV:
            self.get_logger().warn(
                'Không import được cv_bridge/cv2 — bỏ ảnh chồng hình, marker RViz vẫn chạy.')
            return
        self.bridge = CvBridge()
        self.debug_pub = self.create_publisher(Image, '~/image_debug', 1)
        self.create_subscription(CameraInfo, str(self.p['camera_info_topic']),
                                 self._on_caminfo, 1)
        self.create_subscription(Image, str(self.p['image_topic']),
                                 self._on_image, qos_profile_sensor_data)

    @staticmethod
    def _R_from_z(axis):
        """Khung bất kỳ có +z trùng trục lỗ (chỉ để vẽ hình trụ/vòng tròn)."""
        z = np.asarray(axis, dtype=float)
        z = z / (np.linalg.norm(z) or 1.0)
        helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x = np.cross(helper, z)
        x = x / (np.linalg.norm(x) or 1.0)
        return np.column_stack([x, np.cross(z, x), z])

    def _view_from_sol(self, p_tag, sol, label):
        return {'origin': np.asarray(p_tag, dtype=float),
                'slots': [np.asarray(s, dtype=float) for s in sol['slots']],
                'axis': np.asarray(sol['axis'], dtype=float),
                'tilt_deg': float(sol['tilt_deg']), 'label': label}

    def _view_from_saved(self, doc):
        if not doc:
            return None
        pose = doc['rack_pose']
        return {'origin': np.array([pose['x'], pose['y'], pose['z']]),
                'slots': [np.array([float(c) for c in s]) for s in doc['slots']],
                'axis': np.array([float(v) for v in (doc.get('slot_axis') or [0, 0, 1])]),
                'tilt_deg': float(doc.get('rack_tilt_deg', 0.0)),
                'label': f'FILE {os.path.basename(self.output_path)}'}

    def _publish_markers(self):
        if self.marker_pub is None:
            return
        arr = MarkerArray()
        # File vẽ trước (cam), nghiệm đang đo vẽ sau (lục) — trùng nhau thì chỉ
        # thấy lục, lệch nhau thì thấy hai bộ, đúng thứ cần biết.
        if self.saved_view:
            arr.markers += self._markers_for(self.saved_view, 'saved',
                                             (1.0, 0.6, 0.1), 0.45, lifetime=0.0)
        if self.live:
            arr.markers += self._markers_for(self.live, 'live',
                                             (0.2, 1.0, 0.3), 0.85, lifetime=3.0)
        if arr.markers:
            self.marker_pub.publish(arr)

    def _markers_for(self, view, ns, rgb, alpha, lifetime):
        stamp = self.get_clock().now().to_msg()
        radius = float(self.p['hole_radius_m'])
        depth = float(self.p['hole_depth_m'])
        q = R_to_quat(self._R_from_z(view['axis']))
        out = []

        def base(kind, mid, sub_ns):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.base_frame
            m.ns = f'{ns}/{sub_ns}'
            m.id = mid
            m.type = kind
            m.action = Marker.ADD
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = alpha
            m.pose.orientation.w = 1.0
            m.lifetime.sec = int(lifetime)
            m.lifetime.nanosec = int((lifetime % 1) * 1e9)
            return m

        for i, sl in enumerate(view['slots']):
            centre = sl - view['axis'] * (depth / 2.0)      # thân lỗ nằm DƯỚI miệng
            cyl = base(Marker.CYLINDER, i, 'holes')
            cyl.pose.position.x, cyl.pose.position.y, cyl.pose.position.z = centre
            cyl.pose.orientation.x, cyl.pose.orientation.y = float(q[0]), float(q[1])
            cyl.pose.orientation.z, cyl.pose.orientation.w = float(q[2]), float(q[3])
            cyl.scale.x = cyl.scale.y = 2 * radius
            cyl.scale.z = depth
            out.append(cyl)

            txt = base(Marker.TEXT_VIEW_FACING, i, 'labels')
            tip = sl + view['axis'] * 0.025
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = tip
            txt.scale.z = 0.022
            txt.color.a = 1.0
            txt.text = str(i)
            out.append(txt)

        # Viền giá: với 4 lỗ thì 0-1-3-2-0 mới ra hình chữ nhật (0,1 là cạnh gần).
        order = [0, 1, 3, 2, 0] if len(view['slots']) == 4 else \
            list(range(len(view['slots']))) + [0]
        line = base(Marker.LINE_STRIP, 0, 'outline')
        line.scale.x = 0.004
        for i in order:
            pt = Point()
            pt.x, pt.y, pt.z = view['slots'][i]
            line.points.append(pt)
        out.append(line)

        arrow = base(Marker.ARROW, 0, 'axis')
        tail, head = Point(), Point()
        tail.x, tail.y, tail.z = view['origin']
        tip = view['origin'] + view['axis'] * 0.08
        head.x, head.y, head.z = tip
        arrow.points = [tail, head]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.006, 0.012, 0.0
        out.append(arrow)

        tag = base(Marker.SPHERE, 0, 'tag')
        tag.pose.position.x, tag.pose.position.y, tag.pose.position.z = view['origin']
        tag.scale.x = tag.scale.y = tag.scale.z = 0.012
        out.append(tag)
        return out

    # ── ảnh chồng hình ──────────────────────────────────────────────────
    def _on_caminfo(self, msg):
        self.cam_K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.cam_D = np.array(msg.d, dtype=float).reshape(1, -1)

    def _project(self, points_base, T_cam_base):
        """[(u, v) | None] — None khi điểm nằm SAU lưng camera (z ≤ 0)."""
        P = np.asarray(points_base, dtype=float).reshape(-1, 3)
        cam = (T_cam_base[:3, :3] @ P.T).T + T_cam_base[:3, 3]
        out = []
        for pt in cam:
            if pt[2] <= 1e-6:
                out.append(None)
                continue
            uv, _ = cv2.projectPoints(pt.reshape(1, 1, 3), np.zeros(3), np.zeros(3),
                                      self.cam_K, self.cam_D)
            out.append((int(round(float(uv[0, 0, 0]))), int(round(float(uv[0, 0, 1])))))
        return out

    def _on_image(self, msg):
        if self.debug_pub is None or self.cam_K is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_debug < 1.0 / max(1.0, float(self.p['debug_rate_hz'])):
            return
        self._last_debug = now
        try:
            T_base_cam = self._lookup(self.base_frame,
                                      msg.header.frame_id or 'camera_color_optical_frame',
                                      stamp_to_sec(msg.header.stamp))
        except RuntimeError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                          # noqa: BLE001
            self.get_logger().warn(f'cv_bridge: {exc}', throttle_duration_sec=5.0)
            return
        img = self._draw_overlay(img.copy(), T_inv(T_base_cam))
        out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        out.header = msg.header
        self.debug_pub.publish(out)

    def _draw_overlay(self, img, T_cam_base):
        hud = []
        if self.saved_view:
            self._draw_view(img, self.saved_view, T_cam_base, (20, 160, 255), 1)
        if self.live:
            self._draw_view(img, self.live, T_cam_base, (60, 230, 60), 2)

        if self.live:
            hud.append((f'LIVE dang do  |  nghieng {self.live["tilt_deg"]:.1f} do',
                        (60, 230, 60)))
        if self.saved_view:
            hud.append((f'{self.saved_view["label"]}  |  nghieng '
                        f'{self.saved_view["tilt_deg"]:.1f} do', (20, 160, 255)))
        if self.live and self.saved_view and \
                len(self.live['slots']) == len(self.saved_view['slots']):
            d = max(float(np.linalg.norm(a - b)) * 1000.0
                    for a, b in zip(self.live['slots'], self.saved_view['slots']))
            hud.append((f'lech live vs file: {d:.1f} mm',
                        (60, 230, 60) if d <= 5.0 else (0, 80, 255)))
        if not hud:
            hud.append(('chua co nghiem — dang cho tag id '
                        f'{int(self.p["tag_id"])}', (0, 200, 255)))
        for i, (text, colour) in enumerate(hud):
            y = 24 + 22 * i
            cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        colour, 1, cv2.LINE_AA)
        return img

    def _draw_view(self, img, view, T_cam_base, colour, thickness):
        radius_m = float(self.p['hole_radius_m'])
        side = self._R_from_z(view['axis'])[:, 0]          # hướng bất kỳ trong mặt lỗ
        pts = self._project(view['slots'], T_cam_base)
        rim = self._project([s + side * radius_m for s in view['slots']], T_cam_base)
        order = [0, 1, 3, 2, 0] if len(pts) == 4 else list(range(len(pts))) + [0]
        for a, b in zip(order, order[1:]):
            if pts[a] and pts[b]:
                cv2.line(img, pts[a], pts[b], colour, thickness, cv2.LINE_AA)
        for i, (centre, edge) in enumerate(zip(pts, rim)):
            if not centre:
                continue
            r = max(4, int(round(math.hypot(edge[0] - centre[0], edge[1] - centre[1])))
                    ) if edge else 6
            cv2.circle(img, centre, r, colour, thickness, cv2.LINE_AA)
            cv2.drawMarker(img, centre, colour, cv2.MARKER_CROSS, 8, 1)
            cv2.putText(img, str(i), (centre[0] + r + 3, centre[1] - r - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, str(i), (centre[0] + r + 3, centre[1] - r - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
        axis_pts = self._project([view['origin'], view['origin'] + view['axis'] * 0.06],
                                 T_cam_base)
        if all(axis_pts):
            cv2.arrowedLine(img, axis_pts[0], axis_pts[1], colour, thickness,
                            cv2.LINE_AA, tipLength=0.25)

    # ── watch ───────────────────────────────────────────────────────────
    def _on_watch_tick(self):
        # Mất tag quá 2 s ⇒ XOÁ nghiệm live. Giữ lại thì marker vẫn nằm im chỗ cũ
        # và người xem tưởng đang đo được, trong khi thực ra camera không thấy gì.
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.samples and now - self.samples[-1][0] > 2.0:
            self.live = None
        if not self.samples:
            self.get_logger().warn(
                f'Chưa thấy tag id {int(self.p["tag_id"])} trên '
                f'{self.p["detections_topic"]} ({self.frames_seen} frame).',
                throttle_duration_sec=5.0)
            return
        recent = self.samples[-min(len(self.samples), 10):]
        p_now = np.median(np.array([s[1] for s in recent]), axis=0)
        q_now = quat_mean([s[2] for s in recent])
        sol = self._solve(p_now, q_now)
        self.live = self._view_from_sol(p_now, sol, 'LIVE')
        if not self.saved:
            self.get_logger().info(
                f'tag ({p_now[0]:+.4f}, {p_now[1]:+.4f}, {p_now[2]:.4f}) · nghiêng '
                f'{sol["tilt_deg"]:.1f}° — chưa có file để so.')
            return
        old = np.array([[float(c) for c in s] for s in self.saved['slots']])
        new = np.array([list(s) for s in sol['slots']])
        if old.shape != new.shape:
            self.get_logger().warn('Số slot trong file khác cấu hình hiện tại — snap lại.')
            return
        d = np.linalg.norm(new - old, axis=1) * 1000.0
        tilt_old = float(self.saved.get('rack_tilt_deg', 0.0))
        self.get_logger().info(
            'lệch so với file: ' +
            ' '.join(f'slot{i} {v:5.1f}mm' for i, v in enumerate(d)) +
            f' | nghiêng {sol["tilt_deg"]:.1f}° (file {tilt_old:.1f}°)')
        if float(np.max(d)) > 5.0:
            self.get_logger().warn(
                f'Lệch {np.max(d):.1f} mm — giá đã bị xê dịch (hoặc calib camera trôi). '
                'Chạy ./rx150.sh rack-calib để hiệu chuẩn lại.')


def main():
    rclpy.init()
    node = None
    try:
        node = RackCalib()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit) as exc:
        code = exc.code if isinstance(exc, SystemExit) else 0
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return code or 0
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
