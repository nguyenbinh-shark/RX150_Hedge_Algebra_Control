#!/usr/bin/env python3
"""
yolo_tube_detector_node.py — Nhận diện Ống Nghiệm 3D + Góc Nghiêng (Keypoint, Segment & BBox).

Tính năng:
1. Hỗ trợ 3 loại Model YOLOv8:
   - Model Keypoint (YOLOv8-Pose): Tự động trích xuất Nắp (Keypoint 0) & Đáy (Keypoint 1).
   - Model Segmentation (YOLOv8-Seg): Dùng mask contour xác định trục ống + phân biệt
     đầu nắp/đáy bằng phân tích màu HSV tại 2 cực (nắp có màu bão hoà cao, đáy trong suốt).
   - Model Detection chuẩn (YOLOv8): Tự động tính góc nghiêng qua cv2.minAreaRect.
2. Đo độ sâu Z tại từng Keypoint và tâm vật thể từ RealSense Aligned Depth.
3. Tính toán vectơ trục 3D của ống nghiệm và quy đổi sang góc xoay Yaw (Quaternion)
   chính xác trong hệ tọa độ cánh tay robot (rx150/base_link).
4. Xuất các topic:
   - /yolo/detected_tubes: PoseArray chứa đầy đủ Position (X,Y,Z) VÀ Orientation (qx,qy,qz,qw).
   - /yolo/tube_classes: Danh sách JSON tên nhãn màu.
   - /yolo/image_debug: Ảnh vẽ Bounding Box, Keypoints/Mask, đường nối trục và góc độ.
   - /yolo/markers: Marker 3D (mũi tên hướng + text nhãn) xem trực quan trên RViz.

Tải máy: inference bị giới hạn bởi `detect_rate_hz` (mặc định 5 Hz) và chạy trên GPU nếu
`device:=cuda` khả dụng. Chạy full 30 Hz trên CPU sẽ chiếm hết core và làm đơ desktop.
"""

import json
import os
import math
import time
from contextlib import nullcontext as _nullcontext
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from ament_index_python.packages import get_package_share_directory

# torch đi kèm ultralytics. Giữ import ở module level để dùng cho hậu xử lý mask
# TRÊN GPU (F.max_pool2d) thay vì kéo mask về CPU rồi gọi cv2.morphologyEx.
try:
    import torch
    import torch.nn.functional as F
    _HAS_TORCH = True
except ImportError:                                   # pragma: no cover
    torch = None
    F = None
    _HAS_TORCH = False

# OpenCV mặc định lấy toàn bộ core cho resize/morphology/findContours. Phần này chạy CPU
# kể cả khi YOLO đã lên GPU, nên phải ghim lại để chừa core cho Xorg/gnome-shell.
cv2.setNumThreads(2)

# Hue trung tâm (thang OpenCV 0..179) của từng màu nắp ống nghiệm, dùng để chấm điểm
# xem đầu nào của trục là NẮP. Đáy ống trong suốt nên gần như không khớp hue nào.
CAP_HUE_CENTERS = {
    'blue': 110.0,
    'green': 60.0,
    'yellow': 27.0,
    'pink': 168.0,
}


class YoloTubeDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_tube_detector')

        # 1. Khai báo tham số
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('target_frame', 'rx150/base_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        # Giới hạn tần suất inference. Camera chạy 30 Hz nhưng tube_rack chỉ lấy
        # median_frames=5 trong median_collect_s=1.0 → 5 Hz là thừa đủ.
        self.declare_parameter('detect_rate_hz', 5.0)
        # 'cuda' (mặc định) hoặc 'cpu'. Tự rơi về cpu nếu torch không thấy GPU.
        self.declare_parameter('device', 'cuda')

        # ── Tham số ĐỘ CHÍNH XÁC của YOLO ────────────────────────────────────
        # imgsz: độ phân giải inference. 0 = dùng imgsz model được train (640).
        #   Tăng lên 800/960 giúp bắt ống nhỏ/xa tốt hơn (đo: 640 ra 3 ống, 800 ra 4)
        #   nhưng cũng dễ sinh detection trùng -> phải siết `iou` kèm theo.
        #   Chi phí: 640≈5.8 ms, 800≈12.9 ms, 960≈16.2 ms trên A4000.
        self.declare_parameter('imgsz', 0)
        # iou: ngưỡng NMS. Thấp hơn = gộp mạnh hơn các box chồng nhau (bớt trùng),
        #   nhưng 2 ống nằm sát/chạm nhau có thể bị gộp làm một. 0.7 là mặc định YOLO.
        self.declare_parameter('iou', 0.7)
        # max_det: chặn trên số detection. Đặt ~ số ống tối đa trên bàn để loại đuôi rác.
        self.declare_parameter('max_det', 30)
        # agnostic_nms: NMS bỏ qua nhãn lớp. Bật khi cùng một ống bị gán 2 màu khác nhau.
        self.declare_parameter('agnostic_nms', False)
        # retina_masks: trả mask ở ĐỘ PHÂN GIẢI ẢNH GỐC thay vì độ phân giải inference.
        #   Mask nét hơn -> trục/tâm chính xác hơn. BẮT BUỘC bật khi imgsz khác native,
        #   nên để 'auto' node tự bật giúp.
        self.declare_parameter('retina_masks', True)

        # ── Độ tin cậy của DEPTH ─────────────────────────────────────────────
        # Cửa sổ median quanh pixel. To hơn = chịu được lỗ depth tốt hơn nhưng dễ
        # ăn nhầm depth của nền khi lấy sát mép ống.
        self.declare_parameter('depth_window', 5)
        # Tỷ lệ pixel hợp lệ tối thiểu trong cửa sổ; dưới ngưỡng coi như không có depth.
        self.declare_parameter('depth_min_valid_ratio', 0.30)
        self.declare_parameter('depth_min_m', 0.10)
        self.declare_parameter('depth_max_m', 1.50)

        # ── Mask / contour ───────────────────────────────────────────────────
        self.declare_parameter('mask_open_kernel', 5)      # <3 = tắt opening
        self.declare_parameter('mask_min_area_px', 100.0)
        self.declare_parameter('bbox_pad_px', 4)
        # Tỷ lệ dọc trục để lấy 2 điểm cực (0.70 = 70% nửa chiều dài tính từ tâm).
        self.declare_parameter('endpoint_frac', 0.70)
        # Bỏ qua ống mà mask gần tròn: trục chính không xác định -> yaw vô nghĩa.
        # MẶC ĐỊNH 0.0 = TẮT. Gate này có thể loại oan nếu model segment ra mask ngắn,
        # nên hãy bật `log_gate_metrics:=true`, xem số thật rồi mới đặt ngưỡng (vd 1.6).
        self.declare_parameter('min_aspect_ratio', 0.0)

        # ── Phân biệt NẮP vs ĐÁY (HSV) ───────────────────────────────────────
        self.declare_parameter('cap_patch_radius', 14)
        self.declare_parameter('cap_sat_min', 60)          # ngưỡng bão hoà coi là "có màu"
        self.declare_parameter('cap_val_min', 50)          # ngưỡng sáng
        self.declare_parameter('cap_hue_tol', 12.0)        # sai số hue chấp nhận (thang 0..179)
        # Hai cực chấm điểm sát nhau nghĩa là không phân biệt nổi nắp/đáy -> yaw có thể
        # lệch đúng 180°. Dưới ngưỡng này thì bỏ hẳn ống đó thay vì đoán bừa.
        # MẶC ĐỊNH 0.0 = giữ nguyên hành vi cũ (không loại ai). Xem số thật qua
        # `log_gate_metrics` rồi nâng dần (vd 20.0) nếu thấy yaw hay lật 180°.
        self.declare_parameter('cap_score_margin', 0.0)
        # In tỷ lệ dài/rộng + điểm nắp/đáy của TỪNG detection -> dùng để chọn 2 ngưỡng
        # trên từ dữ liệu thật. Chỉ bật khi đang tune, log khá dày.
        self.declare_parameter('log_gate_metrics', False)

        # ── Làm mượt theo thời gian ──────────────────────────────────────────
        # EMA vị trí + yaw, khớp detection giữa các frame theo khoảng cách gần nhất.
        # 0 = tắt. 0.5 = trung bình động vừa phải, 0.8 = rất mượt nhưng chậm phản ứng.
        self.declare_parameter('smooth_alpha', 0.5)
        self.declare_parameter('smooth_match_dist_m', 0.04)
        # Số frame liên tiếp phải thấy trước khi publish -> loại detection nhấp nháy.
        # 2 ở 5 Hz nghĩa là chậm thêm ~200 ms trước pose đầu tiên. Đặt 1 để tắt.
        self.declare_parameter('min_hits', 2)
        self.declare_parameter('track_timeout_s', 1.0)

        # Tham số vùng làm việc giới hạn 3D (Workspace ROI Box)
        self.declare_parameter('enable_roi_box', True)
        self.declare_parameter('roi_x_min', 0.10)
        self.declare_parameter('roi_x_max', 0.35)
        self.declare_parameter('roi_y_min', -0.25)
        self.declare_parameter('roi_y_max', 0.25)
        self.declare_parameter('roi_z_min', -0.02)
        self.declare_parameter('roi_z_max', 0.25)
        # Hiển thị: khối đặc alpha thấp gần như vô hình trên nền pointcloud, nên vẽ thêm
        # khung dây 12 cạnh (giống CropBox của pc_filter ngày trước) mới nhìn ra hộp.
        self.declare_parameter('roi_fill_alpha', 0.10)   # 0 = bỏ khối đặc, chỉ còn khung dây
        self.declare_parameter('roi_edge_width', 0.004)  # bề dày đường (m); 0 = bỏ khung dây

        # File ROI riêng. Nạp SAU declare nên nó GHI ĐÈ giá trị đến từ params-file của
        # launch — chủ ý: vùng làm việc chỉ có MỘT nguồn sự thật, khỏi phải nhớ file nào
        # thắng file nào. Đặt roi_params_file:='' nếu muốn ROI đến từ launch/CLI.
        self.declare_parameter(
            'roi_params_file',
            os.path.join(
                get_package_share_directory('rx150_perception'),
                'config', 'roi_box_params.yaml',
            ),
        )
        self._load_roi_file(self.get_parameter('roi_params_file').value)

        model_p = self.get_parameter('model_path').value
        if not model_p:
            pkg_share = get_package_share_directory('rx150_perception')
            # Ưu tiên: segmentation best.pt > keypoint > detection (bbox)
            possible_paths = [
                os.path.join(pkg_share, 'models', 'best.pt'),          # Model Segmentation mới nhất
                os.path.join(pkg_share, 'models', 'key_point', 'best.pt'),
                os.path.join(pkg_share, 'models', 'best_color.pt'),    # Detection (bbox)
            ]
            for p in possible_paths:
                if os.path.isfile(p):
                    model_p = p
                    break
            if not model_p:
                model_p = os.path.join(pkg_share, 'models', 'best.pt')

        self.conf_thresh = float(self.get_parameter('conf_threshold').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.optical_frame = self.get_parameter('camera_optical_frame').value

        # ── Đọc tham số độ chính xác một lần (không đổi lúc chạy) ────────────
        g = self.get_parameter
        imgsz = int(g('imgsz').value)
        self.retina_masks = bool(g('retina_masks').value)
        # imgsz khác native mà không bật retina_masks thì mask trả về ở độ phân giải
        # inference; _contour_from_mask có quy đổi được nhưng mask thô hơn -> tự bật.
        if imgsz > 0 and not self.retina_masks:
            self.retina_masks = True
            self.get_logger().warn(
                f'imgsz={imgsz} != native → tự bật retina_masks để mask ở đúng độ phân '
                'giải ảnh gốc (trục/tâm chính xác hơn).')
        # Gói sẵn kwargs truyền thẳng cho ultralytics mỗi frame.
        self.infer_kwargs = {
            'conf': self.conf_thresh,
            'iou': float(g('iou').value),
            'max_det': int(g('max_det').value),
            'agnostic_nms': bool(g('agnostic_nms').value),
            'retina_masks': self.retina_masks,
        }
        if imgsz > 0:
            self.infer_kwargs['imgsz'] = imgsz

        self.depth_window = max(1, int(g('depth_window').value))
        self.depth_min_valid_ratio = float(g('depth_min_valid_ratio').value)
        self.depth_min_m = float(g('depth_min_m').value)
        self.depth_max_m = float(g('depth_max_m').value)

        self.mask_open_kernel = int(g('mask_open_kernel').value)
        self.mask_min_area_px = float(g('mask_min_area_px').value)
        self.bbox_pad_px = int(g('bbox_pad_px').value)
        self.endpoint_frac = float(g('endpoint_frac').value)
        self.min_aspect_ratio = float(g('min_aspect_ratio').value)

        self.cap_patch_radius = int(g('cap_patch_radius').value)
        self.cap_sat_min = int(g('cap_sat_min').value)
        self.cap_val_min = int(g('cap_val_min').value)
        self.cap_hue_tol = float(g('cap_hue_tol').value)
        self.cap_score_margin = float(g('cap_score_margin').value)
        self.log_gate_metrics = bool(g('log_gate_metrics').value)

        self.smooth_alpha = float(g('smooth_alpha').value)
        self.smooth_match_dist = float(g('smooth_match_dist_m').value)
        self.min_hits = int(g('min_hits').value)
        self.track_timeout_s = float(g('track_timeout_s').value)
        self._tracks = []          # [{'p':np.array(3), 'yaw':float, 'cls':str,
                                   #   'hits':int, 'last':float}]

        self.get_logger().info(
            f"Độ chính xác: imgsz={imgsz or 'native'} iou={self.infer_kwargs['iou']} "
            f"retina={self.retina_masks} max_det={self.infer_kwargs['max_det']} | "
            f'depth win={self.depth_window} [{self.depth_min_m}..{self.depth_max_m}]m | '
            f'smooth α={self.smooth_alpha} min_hits={self.min_hits}')

        # Chọn device: chỉ dùng CUDA khi torch thực sự thấy GPU, và nói rõ khi phải fallback.
        self.device = self._resolve_device(str(self.get_parameter('device').value).lower())

        # Throttle inference
        rate_hz = float(self.get_parameter('detect_rate_hz').value)
        self._min_period = (1.0 / rate_hz) if rate_hz > 0.0 else 0.0
        self._last_infer = 0.0
        self.get_logger().info(
            f'Inference throttle: {rate_hz:.1f} Hz (chu kỳ tối thiểu {self._min_period * 1e3:.0f} ms)')

        self.get_logger().info(f'Đang nạp model YOLOv8 từ: {model_p}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_p)
            self.get_logger().info(f'Nạp model thành công! Task: {self.model.task}, Classes: {self.model.names}')
            # Warm-up: lần inference đầu phải build cuDNN plan + cấp phát VRAM (~1-2 s).
            # Làm ngay lúc khởi động để frame thật đầu tiên không bị khựng.
            warm = np.zeros((480, 640, 3), np.uint8)
            t_warm = time.monotonic()
            with torch.inference_mode() if _HAS_TORCH else _nullcontext():
                for _ in range(2):
                    self.model(warm, device=self.device, verbose=False, **self.infer_kwargs)
            self.get_logger().info(f'Warm-up xong sau {(time.monotonic() - t_warm) * 1e3:.0f} ms.')
        except Exception as e:
            self.get_logger().error(f'Lỗi nạp YOLOv8: {e}. Vui lòng cài đặt: pip install ultralytics')
            self.model = None

        self.bridge = CvBridge()

        # 2. Quản lý TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 3. Publishers
        self.pub_poses = self.create_publisher(PoseArray, '/yolo/detected_tubes', 10)
        self.pub_classes = self.create_publisher(String, '/yolo/tube_classes', 10)
        self.pub_debug_img = self.create_publisher(Image, '/yolo/image_debug', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/yolo/markers', 10)

        # 4. Subscribers đồng bộ
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        sub_rgb = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos)
        sub_depth = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos)
        sub_info = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/color/camera_info', qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=10, slop=0.08)
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info('YOLO Tube Keypoint Detector đã sẵn sàng!')

    # Khoá đọc từ roi_box_params.yaml. enable_roi_box nằm trong đây vì bản cũ chỉ nạp
    # 6 khoá roi_* -> dòng enable_roi_box trong YAML là dòng chết, sửa cũng không ăn.
    ROI_FLOAT_KEYS = (
        'roi_x_min', 'roi_x_max',
        'roi_y_min', 'roi_y_max',
        'roi_z_min', 'roi_z_max',
    )

    def _load_roi_file(self, path):
        """Nạp hộp giới hạn vùng nhận diện (ROI) từ YAML riêng.

        Chỉ áp dụng những khoá thực sự có trong file; khoá thiếu giữ nguyên giá trị đã
        declare. Không tìm thấy file thì chỉ log info — ROI vẫn chạy bằng default.
        """
        # 'none' là lối thoát cho CLI: rcl không parse nổi `-p roi_params_file:=` (chuỗi
        # rỗng), nên phải có một sentinel gõ được. Qua launch thì '' cũng dùng được.
        if not path or str(path).strip().lower() == 'none':
            self.get_logger().info('roi_params_file tắt -> ROI lấy từ params-file/CLI.')
            return
        try:
            import yaml
            if not os.path.isfile(path):
                self.get_logger().info(f'Không có {path} -> ROI dùng giá trị mặc định.')
                return
            with open(path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            p_dict = (cfg.get('/**', {}) or {}).get('ros__parameters', {}) \
                or cfg.get('ros__parameters', {}) or {}

            params = [
                rclpy.parameter.Parameter(k, rclpy.Parameter.Type.DOUBLE, float(p_dict[k]))
                for k in self.ROI_FLOAT_KEYS if k in p_dict
            ]
            if 'enable_roi_box' in p_dict:
                params.append(rclpy.parameter.Parameter(
                    'enable_roi_box', rclpy.Parameter.Type.BOOL,
                    bool(p_dict['enable_roi_box'])))
            if not params:
                self.get_logger().warn(f'{path} không có khoá roi_* nào -> bỏ qua.')
                return
            self.set_parameters(params)

            g = self.get_parameter
            self.get_logger().info(
                f"ROI Box <- {path} | {'BẬT' if g('enable_roi_box').value else 'TẮT'} | "
                f"X[{g('roi_x_min').value:.3f}..{g('roi_x_max').value:.3f}] "
                f"Y[{g('roi_y_min').value:.3f}..{g('roi_y_max').value:.3f}] "
                f"Z[{g('roi_z_min').value:.3f}..{g('roi_z_max').value:.3f}] m "
                f"(frame {self.get_parameter('target_frame').value})"
            )
        except Exception as e:
            self.get_logger().warn(f'Không nạp được {path}: {e}')

    def _roi_markers(self, xmin, xmax, ymin, ymax, zmin, zmax):
        """Marker cho hộp ROI: khung dây 12 cạnh + (tuỳ chọn) khối đặc mờ.

        Chỉ khối đặc alpha 0.15 thì trên nền pointcloud gần như không thấy gì — khung dây
        mới là thứ nhìn ra được, giống CropBox của pc_filter.
        """
        out = []
        stamp = self.get_clock().now().to_msg()
        fill_a = float(self.get_parameter('roi_fill_alpha').value)
        edge_w = float(self.get_parameter('roi_edge_width').value)

        if fill_a > 0.0:
            box = Marker()
            box.header.frame_id = self.target_frame
            box.header.stamp = stamp
            box.ns = 'workspace_roi'
            box.id = 999
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = (xmin + xmax) / 2.0
            box.pose.position.y = (ymin + ymax) / 2.0
            box.pose.position.z = (zmin + zmax) / 2.0
            box.pose.orientation.w = 1.0
            box.scale.x = xmax - xmin
            box.scale.y = ymax - ymin
            box.scale.z = zmax - zmin
            box.color.r, box.color.g, box.color.b = 0.0, 0.8, 1.0
            box.color.a = fill_a
            out.append(box)

        if edge_w > 0.0:
            edges = Marker()
            edges.header.frame_id = self.target_frame
            edges.header.stamp = stamp
            edges.ns = 'workspace_roi_edges'
            edges.id = 998
            edges.type = Marker.LINE_LIST
            edges.action = Marker.ADD
            edges.pose.orientation.w = 1.0
            edges.scale.x = edge_w            # LINE_LIST chỉ dùng scale.x làm bề dày
            edges.color.r, edges.color.g, edges.color.b = 0.0, 0.9, 1.0
            edges.color.a = 0.9
            c = [(x, y, z) for x in (xmin, xmax)
                 for y in (ymin, ymax) for z in (zmin, zmax)]
            # index vào c: bit0=z, bit1=y, bit2=x. 12 cạnh = mỗi cặp lệch đúng 1 bit.
            for i, j in ((0, 1), (2, 3), (4, 5), (6, 7),
                         (0, 2), (1, 3), (4, 6), (5, 7),
                         (0, 4), (1, 5), (2, 6), (3, 7)):
                for k in (i, j):
                    edges.points.append(Point(x=c[k][0], y=c[k][1], z=c[k][2]))
            out.append(edges)

        return out

    def _resolve_device(self, requested: str) -> str:
        """Trả về device thực sự dùng được; cảnh báo to nếu phải rơi về CPU."""
        if not requested.startswith('cuda'):
            self.get_logger().warn(
                'device:=cpu — YOLO chạy trên CPU sẽ chiếm nhiều core; hãy giữ '
                'detect_rate_hz thấp để tránh làm đơ desktop.')
            return 'cpu'
        if not _HAS_TORCH:
            self.get_logger().error('Không import được torch → rơi về CPU.')
            return 'cpu'
        try:
            if torch.cuda.is_available():
                # Ảnh vào luôn cùng kích thước (640x480) nên để cuDNN benchmark một lần
                # rồi cache thuật toán conv nhanh nhất.
                torch.backends.cudnn.benchmark = True
                self.get_logger().info(
                    f'Inference trên GPU: {requested} ({torch.cuda.get_device_name(0)}), '
                    f'torch {torch.__version__}, cuDNN benchmark ON')
                return requested
            self.get_logger().error(
                f'Yêu cầu {requested} nhưng torch {torch.__version__} (build CUDA '
                f'{torch.version.cuda}) KHÔNG thấy GPU → rơi về CPU. YOLO trên CPU sẽ '
                'chiếm hết core và có thể làm đơ máy. Cài lại torch khớp driver NVIDIA '
                'hiện có (xem `nvidia-smi`).')
        except Exception as e:                        # pragma: no cover
            self.get_logger().error(f'Lỗi dò GPU ({e}) → rơi về CPU.')
        return 'cpu'

    def _get_depth_at(self, depth_image, u, v, window=None):
        """Lấy độ sâu Z (mét) bằng bộ lọc trung vị (Median) quanh pixel (u,v).

        Trả 0.0 khi cửa sổ có quá ít pixel hợp lệ (`depth_min_valid_ratio`) — mép ống
        và vùng bóng của D435i hay bị lỗ, median trên 1-2 pixel sót lại rất dễ sai.
        """
        window = self.depth_window if window is None else window
        h, w = depth_image.shape[:2]
        u_int, v_int = int(round(u)), int(round(v))

        half = window // 2
        v_min, v_max = max(0, v_int - half), min(h, v_int + half + 1)
        u_min, u_max = max(0, u_int - half), min(w, u_int + half + 1)

        patch = depth_image[v_min:v_max, u_min:u_max]
        if patch.size == 0:
            return 0.0
        val = patch[patch > 0]
        if len(val) < max(1, int(patch.size * self.depth_min_valid_ratio)):
            return 0.0
        return float(np.median(val)) / 1000.0

    def _depth_ok(self, z):
        """Depth có nằm trong dải làm việc khai báo không."""
        return self.depth_min_m <= z <= self.depth_max_m

    def _tracks_begin_frame(self, now):
        """Dọn track quá hạn và mở khoá cho vòng khớp của frame mới."""
        self._tracks = [t for t in self._tracks
                        if now - t['last'] <= self.track_timeout_s]
        for t in self._tracks:
            t['matched'] = False

    def _smooth_one(self, x, y, z, yaw, cls_name, now):
        """Khớp detection với track gần nhất rồi EMA vị trí + yaw.

        Mỗi frame YOLO chạy độc lập nên tâm ống rung vài mm và yaw rung vài độ ngay cả
        khi ống đứng yên. EMA theo track làm phẳng phần rung đó; `min_hits` loại luôn
        detection nhấp nháy chỉ xuất hiện 1 frame.

        Trả (x, y, z, yaw) đã mượt, hoặc None nếu track chưa đủ `min_hits`.
        """
        if self.smooth_alpha <= 0.0 and self.min_hits <= 1:
            return x, y, z, yaw

        p_new = np.array([x, y, z], dtype=float)
        best, best_d = None, self.smooth_match_dist
        for t in self._tracks:
            if t['matched'] or t['cls'] != cls_name:
                continue
            d = float(np.linalg.norm(t['p'] - p_new))
            if d < best_d:
                best, best_d = t, d

        if best is None:                       # ống mới xuất hiện
            self._tracks.append({'p': p_new, 'yaw': yaw, 'cls': cls_name,
                                 'hits': 1, 'last': now, 'matched': True})
            return (x, y, z, yaw) if self.min_hits <= 1 else None

        a = self.smooth_alpha
        best['p'] = a * best['p'] + (1.0 - a) * p_new
        # yaw là GÓC: EMA trên vector đơn vị, nếu không sẽ nhảy loạn khi đi qua ±pi.
        sin_y = a * math.sin(best['yaw']) + (1.0 - a) * math.sin(yaw)
        cos_y = a * math.cos(best['yaw']) + (1.0 - a) * math.cos(yaw)
        best['yaw'] = math.atan2(sin_y, cos_y)
        best['hits'] += 1
        best['last'] = now
        best['matched'] = True

        if best['hits'] < self.min_hits:
            return None
        q = best['p']
        return float(q[0]), float(q[1]), float(q[2]), float(best['yaw'])

    def _deproject_pixel_to_3d(self, u, v, z, fx, fy, cx, cy):
        """Chiếu pixel 2D (u,v) + độ sâu Z sang tọa độ 3D Camera."""
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return np.array([x, y, z])

    def _transform_point_to_base(self, p_cam, R_tf, T_tf):
        """Chuyển đổi điểm 3D từ Camera sang Robot Base."""
        return R_tf.dot(p_cam) + T_tf

    def _open_masks_gpu(self, masks_tensor):
        """Morphological OPEN k x k cho TẤT CẢ mask cùng lúc, ngay trên GPU.

        Thay cho `cv2.morphologyEx(..., MORPH_OPEN)` chạy trên CPU từng mask một.
        erode(x) = -maxpool(-x), dilate(x) = maxpool(x)  →  open = dilate(erode(x)).
        Trả về tensor uint8 (N,H,W) 0/255 vẫn nằm trên device của mask.
        """
        k = self.mask_open_kernel
        if k < 3:                                          # <3 = tắt opening
            return (masks_tensor > 0).to(torch.uint8).mul_(255)
        pad = k // 2
        x = masks_tensor.unsqueeze(1).float()              # (N,1,H,W)
        x = -F.max_pool2d(-x, k, stride=1, padding=pad)    # erode
        x = F.max_pool2d(x, k, stride=1, padding=pad)      # dilate
        return (x.squeeze(1) > 0.5).to(torch.uint8).mul_(255)

    def _contour_from_mask(self, opened, i, x1, y1, x2, y2, h, w):
        """Lấy contour lớn nhất của detection i, chỉ kéo về CPU đúng vùng bounding box.

        Crop trước khi transfer: một mask đầy 640x480 là 300 KB, vùng bbox của ống
        nghiệm thường chỉ ~7 KB — ít hơn hàng chục lần qua PCIe. Crop cũng thay luôn
        vai trò của box_mask + bitwise_and cũ (không loang sang ống kế bên).

        QUAN TRỌNG: mask KHÔNG phải lúc nào cũng cùng độ phân giải với ảnh gốc. Khi đặt
        `imgsz` khác native, ultralytics trả mask ở độ phân giải inference (vd imgsz=800
        -> mask 608x800 cho ảnh 480x640) trừ khi bật `retina_masks`. bbox thì luôn ở toạ
        độ ảnh gốc — nên phải quy đổi hai chiều, nếu không sẽ crop nhầm vùng.
        """
        mh, mw = int(opened.shape[1]), int(opened.shape[2])
        sx, sy = mw / float(w), mh / float(h)              # ảnh gốc -> mask
        pad = self.bbox_pad_px
        cx0 = max(0, int((int(x1) - pad) * sx))
        cy0 = max(0, int((int(y1) - pad) * sy))
        cx1 = min(mw, int(np.ceil((int(x2) + pad) * sx)))
        cy1 = min(mh, int(np.ceil((int(y2) + pad) * sy)))
        if cx1 <= cx0 or cy1 <= cy0:
            return None
        sub = opened[i, cy0:cy1, cx0:cx1]
        sub = sub.cpu().numpy() if _HAS_TORCH and torch.is_tensor(sub) else np.asarray(sub)
        cnts, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea).astype(np.float32)
        # Contour đang ở toạ độ vùng crop TRONG MASK -> dời + quy đổi về toạ độ ảnh gốc.
        c[:, 0, 0] = (c[:, 0, 0] + cx0) / sx
        c[:, 0, 1] = (c[:, 0, 1] + cy0) / sy
        return c

    def _cap_score(self, image, pt, cls_name, radius):
        """Chấm điểm 'giống nắp' cho một cực của trục ống.

        Nắp ống nghiệm y tế có màu đặc trưng (Blue, Green, Yellow, Pink) và độ bão hoà
        màu rất cao; đáy ống trong suốt nên bão hoà thấp và không khớp hue nào.
        """
        h, w = image.shape[:2]
        u, v = int(round(pt[0])), int(round(pt[1]))
        u0, u1 = max(0, u - radius), min(w, u + radius + 1)
        v0, v1 = max(0, v - radius), min(h, v + radius + 1)

        patch = image[v0:v1, u0:u1]
        if patch.size == 0:
            return 0.0

        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]
        hue = hsv[:, :, 0]

        # 1. Điểm độ đậm màu (Saturation) — thang 0..255
        sat_score = float(np.mean(sat))

        # 2. Điểm khớp màu sắc theo nhãn Class (nếu biết màu mong đợi)
        hue_score = 0.0
        center = CAP_HUE_CENTERS.get(str(cls_name).lower())
        if center is not None:
            colored = (sat > self.cap_sat_min) & (val > self.cap_val_min)
            if np.any(colored):
                # Khoảng cách hue vòng tròn (thang OpenCV 0..179)
                d = np.abs(hue[colored].astype(np.float32) - center)
                d = np.minimum(d, 180.0 - d)
                hue_score = 100.0 * float(np.mean(d < self.cap_hue_tol))

        return sat_score + hue_score

    def _detect_cap_endpoint(self, image, pt_a, pt_b, cls_name, radius=None):
        """Phân biệt chính xác đầu nào là NẮP (Cap) vs ĐÁY (Bottom) bằng phân tích HSV.

        Trả về ((cap_u, cap_v), (bot_u, bot_v), score_cap, score_bot). Chênh lệch
        score_cap - score_bot chính là độ tin cậy: sát nhau nghĩa là không phân biệt
        nổi hai đầu, và đoán sai sẽ làm yaw lệch đúng 180°.
        """
        radius = self.cap_patch_radius if radius is None else radius
        sc_a = self._cap_score(image, pt_a, cls_name, radius)
        sc_b = self._cap_score(image, pt_b, cls_name, radius)
        if sc_a >= sc_b:
            return (int(pt_a[0]), int(pt_a[1])), (int(pt_b[0]), int(pt_b[1])), sc_a, sc_b
        return (int(pt_b[0]), int(pt_b[1])), (int(pt_a[0]), int(pt_a[1])), sc_b, sc_a

    def image_callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        if self.model is None:
            return

        # Throttle: bỏ frame nếu chưa tới chu kỳ inference kế tiếp. Phải nằm trước mọi
        # xử lý nặng (convert ảnh, TF lookup, inference).
        now = time.monotonic()
        if self._min_period > 0.0 and (now - self._last_infer) < self._min_period:
            return
        self._last_infer = now
        self._tracks_begin_frame(now)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Lỗi convert ảnh: {e}')
            return

        # Tra cứu ma trận TF Camera -> Robot Base
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                rgb_msg.header.frame_id or self.optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            tx = tf_stamped.transform.translation.x
            ty = tf_stamped.transform.translation.y
            tz = tf_stamped.transform.translation.z
            qx = tf_stamped.transform.rotation.x
            qy = tf_stamped.transform.rotation.y
            qz = tf_stamped.transform.rotation.z
            qw = tf_stamped.transform.rotation.w

            # Ma trận xoay Camera -> Robot Base
            R_tf = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ])
            T_tf = np.array([tx, ty, tz])
        except Exception as ex:
            self.get_logger().debug(f'TF lookup fail: {ex}')
            return

        # Camera Intrinsics
        fx, cx = info_msg.k[0], info_msg.k[2]
        fy, cy = info_msg.k[4], info_msg.k[5]

        # Đọc tham số ROI Box
        enable_roi = bool(self.get_parameter('enable_roi_box').value)
        roi_xmin = float(self.get_parameter('roi_x_min').value)
        roi_xmax = float(self.get_parameter('roi_x_max').value)
        roi_ymin = float(self.get_parameter('roi_y_min').value)
        roi_ymax = float(self.get_parameter('roi_y_max').value)
        roi_zmin = float(self.get_parameter('roi_z_min').value)
        roi_zmax = float(self.get_parameter('roi_z_max').value)

        # Chạy YOLO Inference. inference_mode bỏ hẳn autograd bookkeeping (nhanh hơn no_grad).
        with torch.inference_mode() if _HAS_TORCH else _nullcontext():
            results = self.model(cv_image, device=self.device, verbose=False,
                                 **self.infer_kwargs)

        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        tube_classes = []
        marker_array = MarkerArray()
        # Chỉ dựng ảnh debug khi có người subscribe. Không ai xem thì bỏ qua toàn bộ
        # copy ảnh + vẽ + cv2_to_imgmsg (đều là CPU thuần, OpenCV pip không có CUDA).
        draw_debug = self.pub_debug_img.get_subscription_count() > 0
        debug_img = cv_image.copy() if draw_debug else None

        marker_id = 0

        img_h, img_w = cv_image.shape[:2]

        for r in results:
            # Kiểm tra loại model: Keypoints / Masks / Bounding Boxes
            has_keypoints = (hasattr(r, 'keypoints') and r.keypoints is not None and len(r.keypoints) > 0)
            has_masks = (hasattr(r, 'masks') and r.masks is not None and len(r.masks) > 0)
            boxes = r.boxes if (hasattr(r, 'boxes') and r.boxes is not None) else []

            num_detections = len(boxes)
            if has_keypoints:
                num_detections = max(num_detections, len(r.keypoints))

            # --- Gom mọi transfer GPU->CPU thành MỘT lần cho cả batch ---
            # Trước đây mỗi detection gọi .cpu() ba lần (cls/conf/xyxy) -> 3N điểm
            # đồng bộ CUDA mỗi frame, mỗi lần stall cả pipeline. Giờ là đúng 3 lần.
            cls_all = conf_all = xyxy_all = kp_all = None
            if len(boxes) > 0:
                cls_all = boxes.cls.to(torch.int32).cpu().numpy()
                conf_all = boxes.conf.cpu().numpy()
                xyxy_all = boxes.xyxy.round().to(torch.int32).cpu().numpy()
            if has_keypoints:
                kp_all = r.keypoints.xy.cpu().numpy()

            # Morphological opening cho TẤT CẢ mask cùng lúc, ngay trên GPU.
            masks_opened = None
            if has_masks and _HAS_TORCH:
                masks_opened = self._open_masks_gpu(r.masks.data)

            for i in range(num_detections):
                cls_name = 'tube'
                conf = 0.9
                x1 = y1 = x2 = y2 = 0
                has_box = len(boxes) > i

                if has_box:
                    cls_id = int(cls_all[i])
                    cls_name = self.model.names.get(cls_id, f'tube_{cls_id}')
                    conf = float(conf_all[i])
                    x1, y1, x2, y2 = (int(v) for v in xyxy_all[i])
                    if draw_debug:
                        cv2.rectangle(debug_img, (x1, y1), (x2, y2), (200, 200, 200), 1)

                p_center_base = None
                yaw_robot = 0.0

                # -------------------------------------------------------------
                # TRƯỜNG HỢP A: MODEL KEYPOINTS (YOLOv8-Pose: Nắp & Đáy)
                # -------------------------------------------------------------
                if has_keypoints and kp_all is not None and i < len(kp_all):
                    kp_data = kp_all[i]  # shape (K, 2), đã ở CPU từ transfer gộp
                    if len(kp_data) >= 2:
                        u_top, v_top = kp_data[0] # Keypoint 0: Nắp
                        u_bot, v_bot = kp_data[1] # Keypoint 1: Đáy

                        z_top = self._get_depth_at(depth_image, u_top, v_top)
                        z_bot = self._get_depth_at(depth_image, u_bot, v_bot)

                        # Nếu 1 trong 2 điểm mất depth, dùng depth trung bình
                        z_avg = max(z_top, z_bot)
                        if z_top <= 0.05: z_top = z_avg
                        if z_bot <= 0.05: z_bot = z_avg

                        if z_top > 0.05 and z_bot > 0.05:
                            # De-project 2 keypoints sang 3D
                            p_top_cam = self._deproject_pixel_to_3d(u_top, v_top, z_top, fx, fy, cx, cy)
                            p_bot_cam = self._deproject_pixel_to_3d(u_bot, v_bot, z_bot, fx, fy, cx, cy)

                            # Chuyển sang hệ Robot Base
                            p_top_base = self._transform_point_to_base(p_top_cam, R_tf, T_tf)
                            p_bot_base = self._transform_point_to_base(p_bot_cam, R_tf, T_tf)

                            # Tâm kẹp gắp (Midpoint)
                            p_center_base = (p_top_base + p_bot_base) / 2.0

                            # Vectơ hướng trục ống nghiệm (từ Nắp -> Đáy)
                            vec_tube = p_bot_base - p_top_base
                            yaw_robot = math.atan2(vec_tube[1], vec_tube[0])

                            # Vẽ Debug lên ảnh
                            pt1 = (int(u_top), int(v_top))
                            pt2 = (int(u_bot), int(v_bot))
                            if draw_debug:
                                cv2.circle(debug_img, pt1, 5, (0, 0, 255), -1) # Nắp: Đỏ
                                cv2.circle(debug_img, pt2, 5, (0, 255, 0), -1) # Đáy: Xanh lá
                                cv2.line(debug_img, pt1, pt2, (255, 255, 0), 2)

                # -------------------------------------------------------------
                # TRƯỜNG HỢP B: MODEL SEGMENTATION (Mask contour + nắp/đáy)
                # Dùng mask contour → minAreaRect → trục chính (chính xác hơn bbox).
                # Giới hạn mask trong Bounding Box + Morphological Opening để tách
                # các cầu dính khi 2 ống để gần nhau.
                # -------------------------------------------------------------
                if (p_center_base is None and has_masks and has_box
                        and masks_opened is not None and i < len(masks_opened)):
                    # Mask của ultralytics ĐÃ ở đúng độ phân giải ảnh gốc (480,640) và
                    # nằm trên GPU; opening đã làm cho cả batch ở trên. Ở đây chỉ cần
                    # kéo về CPU đúng vùng bbox rồi tìm contour (OpenCV pip không có CUDA).
                    contour = self._contour_from_mask(
                        masks_opened, i, x1, y1, x2, y2, img_h, img_w)
                    if contour is not None:
                        if cv2.contourArea(contour) >= self.mask_min_area_px:
                            # Vẽ mask contour (viền cyan) lên debug image
                            if draw_debug:
                                cv2.drawContours(
                                    debug_img, [np.round(contour).astype(np.int32)],
                                    -1, (255, 255, 0), 2)

                            # minAreaRect trên mask contour — chính xác hơn bbox crop
                            rect = cv2.minAreaRect(contour)
                            rect_center, rect_size, rect_angle = rect
                            rw, rh = rect_size

                            # Chuẩn hóa: trục dài = hướng ống
                            if rw < rh:
                                angle_rad = math.radians(rect_angle + 90)
                            else:
                                angle_rad = math.radians(rect_angle)

                            # Mask gần tròn (ống dựng đứng / nhìn từ trên xuống) thì trục
                            # chính không xác định — minAreaRect sẽ trả góc ngẫu nhiên.
                            long_s, short_s = max(rw, rh), max(min(rw, rh), 1e-6)
                            aspect = long_s / short_s
                            if self.min_aspect_ratio > 0.0 and aspect < self.min_aspect_ratio:
                                if self.log_gate_metrics:
                                    self.get_logger().info(
                                        f'[gate] {cls_name}: tỷ lệ {aspect:.2f} < '
                                        f'{self.min_aspect_ratio} → BỎ (yaw không tin được)')
                                continue

                            # --- Tính tâm 3D từ tâm mask ---
                            u_mc, v_mc = rect_center
                            z_mc = self._get_depth_at(depth_image, u_mc, v_mc)
                            if self._depth_ok(z_mc):
                                p_cam = self._deproject_pixel_to_3d(u_mc, v_mc, z_mc, fx, fy, cx, cy)
                                p_center_base = self._transform_point_to_base(p_cam, R_tf, T_tf)

                                # --- Phân biệt chính xác NẮP vs ĐÁY bằng HSV Color Matching & Saturation ---
                                dx = math.cos(angle_rad)
                                dy = math.sin(angle_rad)
                                half_len = max(rw, rh) / 2.0

                                ef = self.endpoint_frac
                                pa = (int(u_mc + half_len * ef * dx), int(v_mc + half_len * ef * dy))
                                pb = (int(u_mc - half_len * ef * dx), int(v_mc - half_len * ef * dy))

                                (cap_u, cap_v), (bot_u, bot_v), sc_cap, sc_bot = self._detect_cap_endpoint(
                                    cv_image, pa, pb, cls_name
                                )

                                # Hai đầu chấm điểm quá sát nhau = không phân biệt nổi
                                # nắp/đáy. Đoán bừa ở đây làm yaw lệch ĐÚNG 180°, tệ hơn
                                # nhiều so với việc bỏ qua ống này ở frame hiện tại.
                                margin = sc_cap - sc_bot
                                if self.log_gate_metrics:
                                    self.get_logger().info(
                                        f'[gate] {cls_name}: area={cv2.contourArea(contour):.0f} '
                                        f'tỷ_lệ={aspect:.2f} nắp={sc_cap:.0f} đáy={sc_bot:.0f} '
                                        f'margin={margin:.0f}')
                                if self.cap_score_margin > 0.0 and margin < self.cap_score_margin:
                                    if self.log_gate_metrics:
                                        self.get_logger().info(
                                            f'[gate] {cls_name}: margin {margin:.0f} < '
                                            f'{self.cap_score_margin:.0f} → BỎ (dễ lật yaw 180°)')
                                    continue

                                # De-project 2 cực (Nắp & Đáy) sang tọa độ 3D
                                z_cap = self._get_depth_at(depth_image, cap_u, cap_v)
                                z_bot = self._get_depth_at(depth_image, bot_u, bot_v)
                                if z_cap <= 0.05: z_cap = z_mc
                                if z_bot <= 0.05: z_bot = z_mc

                                p_cap_cam = self._deproject_pixel_to_3d(cap_u, cap_v, z_cap, fx, fy, cx, cy)
                                p_bot_cam = self._deproject_pixel_to_3d(bot_u, bot_v, z_bot, fx, fy, cx, cy)
                                p_cap_base = self._transform_point_to_base(p_cap_cam, R_tf, T_tf)
                                p_bot_base = self._transform_point_to_base(p_bot_cam, R_tf, T_tf)

                                # Vectơ hướng từ Nắp -> Đáy trong hệ tọa độ robot
                                vec_tube = p_bot_base - p_cap_base
                                yaw_robot = math.atan2(vec_tube[1], vec_tube[0])

                                # Vẽ Debug trực quan: Nắp=Đỏ, Đáy=Xanh lá, đường trục Cyan
                                if draw_debug:
                                    cv2.circle(debug_img, (cap_u, cap_v), 7, (0, 0, 255), -1)
                                    cv2.circle(debug_img, (bot_u, bot_v), 7, (0, 255, 0), -1)
                                    cv2.line(debug_img, (cap_u, cap_v), (bot_u, bot_v), (255, 255, 0), 2)
                                    cv2.putText(debug_img, 'CAP', (cap_u - 15, cap_v - 10),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                                    cv2.putText(debug_img, 'BOT', (bot_u - 15, bot_v - 10),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                # -------------------------------------------------------------
                # TRƯỜNG HỢP C: MODEL BOUNDING BOX (Tính góc qua minAreaRect)
                # Fallback khi không có keypoints hay masks.
                # -------------------------------------------------------------
                if p_center_base is None and has_box:
                    u_center = (x1 + x2) / 2.0
                    v_center = (y1 + y2) / 2.0
                    z_center = self._get_depth_at(depth_image, u_center, v_center)

                    if self._depth_ok(z_center):
                        p_cam = self._deproject_pixel_to_3d(u_center, v_center, z_center, fx, fy, cx, cy)
                        p_center_base = self._transform_point_to_base(p_cam, R_tf, T_tf)

                        # Cắt vùng ảnh tìm góc nghiêng minAreaRect
                        crop = cv_image[max(0, y1):min(cv_image.shape[0], y2), max(0, x1):min(cv_image.shape[1], x2)]
                        if crop.size > 0:
                            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                            cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            if cnts:
                                rect = cv2.minAreaRect(max(cnts, key=cv2.contourArea))
                                angle_2d = math.radians(rect[2])
                                yaw_robot = angle_2d

                # Nếu tính toán 3D thành công
                if p_center_base is not None:
                    x_rob, y_rob, z_rob = float(p_center_base[0]), float(p_center_base[1]), float(p_center_base[2])

                    # Kiểm tra xem vật thể có nằm trong Vùng Làm Việc (ROI Box) không
                    if enable_roi:
                        if not (roi_xmin <= x_rob <= roi_xmax and roi_ymin <= y_rob <= roi_ymax and roi_zmin <= z_rob <= roi_zmax):
                            continue  # Nằm ngoài vùng làm việc an toàn -> bỏ qua

                    # Làm mượt theo thời gian + chặn detection nhấp nháy (min_hits).
                    sm = self._smooth_one(x_rob, y_rob, z_rob, yaw_robot, cls_name, now)
                    if sm is None:
                        continue          # track chưa đủ số frame liên tiếp
                    x_rob, y_rob, z_rob, yaw_robot = sm

                    # Chuyển góc Yaw thành Quaternion (xoay quanh trục Z)
                    qz_out = math.sin(yaw_robot / 2.0)
                    qw_out = math.cos(yaw_robot / 2.0)

                    pose = Pose()
                    pose.position.x = x_rob
                    pose.position.y = y_rob
                    pose.position.z = z_rob
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = float(qz_out)
                    pose.orientation.w = float(qw_out)

                    pose_array.poses.append(pose)
                    tube_classes.append(cls_name)

                    # Vẽ chữ nhãn + góc Yaw lên ảnh Debug
                    deg = math.degrees(yaw_robot)
                    label_str = f"{cls_name} ({conf:.2f}) [{x_rob:.2f},{y_rob:.2f}] yaw:{deg:.0f}deg"
                    if draw_debug:
                        cv2.putText(debug_img, label_str, (x1, max(y1 - 6, 12)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

                    # Marker 3D dạng Mũi Tên (Arrow) biểu diễn hướng xoay trong RViz
                    marker_arrow = Marker()
                    marker_arrow.header.frame_id = self.target_frame
                    marker_arrow.header.stamp = pose_array.header.stamp
                    marker_arrow.ns = 'tube_orientation'
                    marker_arrow.id = marker_id
                    marker_arrow.type = Marker.ARROW
                    marker_arrow.action = Marker.ADD
                    marker_arrow.pose = pose
                    marker_arrow.scale.x = 0.08  # Chiều dài mũi tên (8cm)
                    marker_arrow.scale.y = 0.012 # Độ dày
                    marker_arrow.scale.z = 0.012
                    marker_arrow.color.r = 0.0
                    marker_arrow.color.g = 1.0
                    marker_arrow.color.b = 1.0
                    marker_arrow.color.a = 0.9
                    marker_array.markers.append(marker_arrow)

                    # Marker Text nhãn tên
                    marker_txt = Marker()
                    marker_txt.header.frame_id = self.target_frame
                    marker_txt.header.stamp = pose_array.header.stamp
                    marker_txt.ns = 'tube_labels'
                    marker_txt.id = marker_id + 100
                    marker_txt.type = Marker.TEXT_VIEW_FACING
                    marker_txt.action = Marker.ADD
                    marker_txt.pose.position.x = x_rob
                    marker_txt.pose.position.y = y_rob
                    marker_txt.pose.position.z = z_rob + 0.04
                    marker_txt.scale.z = 0.02
                    marker_txt.color.r = 1.0
                    marker_txt.color.g = 1.0
                    marker_txt.color.b = 1.0
                    marker_txt.color.a = 1.0
                    marker_txt.text = f"{cls_name}\n({deg:.0f}°)"
                    marker_array.markers.append(marker_txt)

                    marker_id += 1

        # Hiển thị Khung Hộp Vùng Làm Việc (Workspace ROI Box) trong RViz
        if enable_roi:
            marker_array.markers.extend(self._roi_markers(
                roi_xmin, roi_xmax, roi_ymin, roi_ymax, roi_zmin, roi_zmax))

        # Publish toàn bộ kết quả
        if len(pose_array.poses) > 0 or enable_roi:
            self.pub_poses.publish(pose_array)
            str_msg = String()
            str_msg.data = json.dumps(tube_classes)
            self.pub_classes.publish(str_msg)
            self.pub_markers.publish(marker_array)

        # Publish ảnh Debug (chỉ khi có subscriber — xem draw_debug ở trên)
        if not draw_debug:
            return
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = rgb_msg.header
            self.pub_debug_img.publish(debug_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloTubeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
