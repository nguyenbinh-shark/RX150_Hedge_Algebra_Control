#!/usr/bin/env python3
"""test_yolo_tube_detector.py — Bộ test TỔNG HỢP cho yolo_tube_detector_node.

Gom toàn bộ test đã dùng khi tối ưu 2026-08-27: helper CV, hậu xử lý mask trên GPU,
quy đổi độ phân giải mask, gate depth, EMA tracker, inference model thật, và callback
end-to-end (không cần camera/robot thật).

Chạy:
    source ~/interbotix_ws/source_all.sh
    python3 src/rx150/rx150_toolbox/rx150_perception/test/test_yolo_tube_detector.py           # test offline
    python3 src/rx150/rx150_toolbox/rx150_perception/test/test_yolo_tube_detector.py --bench   # + đo ms/frame
    python3 src/rx150/rx150_toolbox/rx150_perception/test/test_yolo_tube_detector.py --live    # + fake camera 30Hz
                                                                           #   đo nhịp topic thật

Thoát code 0 = tất cả pass; khác 0 = có test FAIL.
"""
import argparse
import importlib.util
import math
import os
import sys
import time
import traceback

import numpy as np
import cv2

cv2.setNumThreads(2)

HERE = os.path.dirname(os.path.abspath(__file__))
NODE_PATH = os.path.join(HERE, '..', 'scripts', 'yolo_tube_detector_node.py')
if not os.path.isfile(NODE_PATH):
    # chạy từ bản cài: node nằm cùng thư mục lib/rx150_perception
    NODE_PATH = os.path.join(HERE, 'yolo_tube_detector_node.py')
MODEL_PATH = os.path.join(HERE, '..', 'models', 'best.pt')
if not os.path.isfile(MODEL_PATH):
    # chạy từ bản cài (lib/rx150_perception) -> lấy model qua ament share
    try:
        from ament_index_python.packages import get_package_share_directory
        MODEL_PATH = os.path.join(
            get_package_share_directory('rx150_perception'), 'models', 'best.pt')
    except Exception:
        pass

# ── nạp module node (không chạy main) ────────────────────────────────────────
spec = importlib.util.spec_from_file_location('ytd', NODE_PATH)
ytd = importlib.util.module_from_spec(spec)
sys.modules['ytd'] = ytd
spec.loader.exec_module(ytd)
NodeCls = ytd.YoloTubeDetectorNode

try:
    import torch
    HAS_CUDA = torch.cuda.is_available()
except ImportError:
    torch = None
    HAS_CUDA = False


class Skip(Exception):
    """Ném để đánh dấu test bị bỏ qua (thiếu GPU/model/...)."""


# ─────────────────────────────────────────────────────────────────────────────
# Shim: dùng thẳng method của node mà không cần rclpy/model.
# ─────────────────────────────────────────────────────────────────────────────
def make_shim(**over):
    class S:
        pass
    for name in ('_cap_score', '_detect_cap_endpoint', '_get_depth_at', '_depth_ok',
                 '_deproject_pixel_to_3d', '_transform_point_to_base',
                 '_open_masks_gpu', '_contour_from_mask',
                 '_tracks_begin_frame', '_smooth_one'):
        setattr(S, name, getattr(NodeCls, name))
    n = S()
    # mặc định giống node
    n.mask_open_kernel = 5
    n.bbox_pad_px = 4
    n.depth_window = 5
    n.depth_min_valid_ratio = 0.30
    n.depth_min_m = 0.10
    n.depth_max_m = 1.50
    n.cap_patch_radius = 14
    n.cap_sat_min = 60
    n.cap_val_min = 50
    n.cap_hue_tol = 12.0
    n.endpoint_frac = 0.70
    n.smooth_alpha = 0.5
    n.smooth_match_dist = 0.04
    n.min_hits = 1
    n.track_timeout_s = 1.0
    n._tracks = []
    for k, v in over.items():
        setattr(n, k, v)
    return n


def scene(seed=1):
    """Ảnh tổng hợp 4 'ống nghiệm' màu nghiêng khác nhau trên nền bàn."""
    img = np.full((480, 640, 3), 70, np.uint8)
    img += (np.random.default_rng(seed).random((480, 640, 3)) * 35).astype(np.uint8)
    for c, (x, y), ang in zip([(180, 105, 255), (255, 90, 30), (60, 220, 240), (90, 220, 120)],
                              [(120, 240), (255, 250), (400, 235), (535, 245)],
                              [0, 20, -35, 12]):
        box = cv2.boxPoints(((x, y), (46, 150), ang)).astype(np.int32)
        cv2.fillPoly(img, [box], (205, 205, 205))
        dx, dy = np.sin(np.radians(ang)) * 62, -np.cos(np.radians(ang)) * 62
        cv2.circle(img, (int(x + dx), int(y + dy)), 23, c, -1)
    return img


# ─────────────────────────────────────────────────────────────────────────────
# A. Helper CV thuần (không cần ROS/GPU/model)
# ─────────────────────────────────────────────────────────────────────────────
def test_cap_endpoint_hsv():
    """Phân biệt nắp/đáy bằng HSV: 4 màu + trường hợp nắp nằm ở cực bên kia."""
    n = make_shim()
    # pink bên TRÁI
    img = np.full((200, 400, 3), 190, np.uint8)
    cv2.circle(img, (80, 100), 22, (180, 105, 255), -1)
    cap, bot, sc, sb = n._detect_cap_endpoint(img, (80, 100), (320, 100), 'pink')
    assert cap == (80, 100) and sc > sb, f'pink: {cap} {sc} {sb}'
    # blue bên PHẢI -> phải đảo
    img2 = np.full((200, 400, 3), 190, np.uint8)
    cv2.circle(img2, (320, 100), 22, (255, 90, 30), -1)
    cap2, _, _, _ = n._detect_cap_endpoint(img2, (80, 100), (320, 100), 'blue')
    assert cap2 == (320, 100), f'blue phải đảo: {cap2}'
    # yellow
    img3 = np.full((200, 400, 3), 190, np.uint8)
    cv2.circle(img3, (80, 100), 22, (60, 220, 240), -1)
    assert n._detect_cap_endpoint(img3, (80, 100), (320, 100), 'yellow')[0] == (80, 100)
    # class lạ: chỉ dựa saturation vẫn phải chọn đúng đầu có màu
    assert n._detect_cap_endpoint(img, (80, 100), (320, 100), 'Unknown')[0] == (80, 100)


def test_depth_median_and_valid_ratio():
    """Median depth + từ chối cửa sổ quá nhiều lỗ (depth_min_valid_ratio)."""
    n = make_shim()
    d = np.zeros((100, 100), np.uint16)
    d[48:53, 48:53] = 500                              # 25/25 hợp lệ
    assert abs(n._get_depth_at(d, 50, 50) - 0.5) < 1e-9
    assert n._get_depth_at(d, 10, 10) == 0.0           # vùng rỗng
    d2 = np.zeros((100, 100), np.uint16)
    d2[50, 50] = 500                                   # 1/25 = 4% < 30% -> từ chối
    assert n._get_depth_at(d2, 50, 50) == 0.0
    d3 = np.zeros((100, 100), np.uint16)
    d3[48:53, 48:51] = 500                             # 15/25 = 60% -> chấp nhận
    assert abs(n._get_depth_at(d3, 50, 50) - 0.5) < 1e-9
    assert not n._depth_ok(0.0) and n._depth_ok(0.6) and not n._depth_ok(2.0)


def test_deproject_and_transform():
    """Chiếu pixel->3D và đổi hệ toạ độ."""
    n = make_shim()
    p = n._deproject_pixel_to_3d(320.0, 240.0, 0.5, 600.0, 600.0, 320.0, 240.0)
    assert np.allclose(p, [0, 0, 0.5])
    out = n._transform_point_to_base(p, np.eye(3), np.array([1.0, 2.0, 3.0]))
    assert np.allclose(out, [1.0, 2.0, 3.5])


# ─────────────────────────────────────────────────────────────────────────────
# B. Hậu xử lý mask (torch; chạy GPU nếu có, CPU tensor nếu không)
# ─────────────────────────────────────────────────────────────────────────────
def _tensor(m):
    t = torch.from_numpy(m)
    return t.cuda() if HAS_CUDA else t


def test_gpu_opening_vs_cv2():
    """Opening bằng max_pool2d phải khớp cv2.morphologyEx MORPH_OPEN từng pixel."""
    if torch is None:
        raise Skip('không có torch')
    n = make_shim()
    m = np.zeros((1, 120, 160), np.uint8)
    cv2.rectangle(m[0], (40, 20), (70, 95), 1, -1)
    cv2.circle(m[0], (110, 60), 2, 1, -1)              # đốm nhiễu phải bị xoá
    op = n._open_masks_gpu(_tensor(m))
    c = n._contour_from_mask(op, 0, 40, 20, 70, 95, 120, 160)
    ref = cv2.morphologyEx(m[0] * 255, cv2.MORPH_OPEN,
                           cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    cnts, _ = cv2.findContours(ref, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    big = max(cnts, key=cv2.contourArea)
    assert abs(cv2.contourArea(c) - cv2.contourArea(big)) < 1e-6, \
        f'{cv2.contourArea(c)} vs {cv2.contourArea(big)}'
    # kernel < 3 = tắt opening -> đốm nhiễu còn nguyên
    n2 = make_shim(mask_open_kernel=0)
    op2 = n2._open_masks_gpu(_tensor(m))
    assert int(op2.sum().item()) // 255 > int(op.sum().item()) // 255


def test_contour_scaling_across_resolutions():
    """Mask KHÁC độ phân giải ảnh (imgsz != native, retina off) -> contour vẫn đúng
    toạ độ ảnh gốc. Đây là bug đã sửa 2026-08-27."""
    if torch is None:
        raise Skip('không có torch')
    n = make_shim()
    IMG_H, IMG_W = 480, 640
    for mh, mw in [(480, 640), (608, 800), (960, 1280)]:
        sx, sy = mw / IMG_W, mh / IMG_H
        m = np.zeros((1, mh, mw), np.uint8)
        cv2.rectangle(m[0], (int(200 * sx), int(150 * sy)),
                      (int(260 * sx), int(330 * sy)), 1, -1)
        c = n._contour_from_mask(n._open_masks_gpu(_tensor(m)),
                                 0, 200, 150, 260, 330, IMG_H, IMG_W)
        (ux, uy), _, _ = cv2.minAreaRect(c)
        err = math.hypot(ux - 230, uy - 240)
        assert err < 1.5, f'mask {mh}x{mw}: tâm lệch {err:.2f} px'


# ─────────────────────────────────────────────────────────────────────────────
# C. EMA tracker theo thời gian
# ─────────────────────────────────────────────────────────────────────────────
def test_tracker_smoothing_reduces_noise():
    """EMA phải giảm rung so với dữ liệu thô (5mm + 4°)."""
    rng = np.random.default_rng(7)
    TRUE_P, TRUE_Y = np.array([0.25, 0.05, 0.08]), 0.30
    results = {}
    for alpha in (0.0, 0.5, 0.8):
        n = make_shim(smooth_alpha=alpha, min_hits=1)
        errs = []
        t = 0.0
        for k in range(60):
            t += 0.2
            n._tracks_begin_frame(t)
            noisy = TRUE_P + rng.normal(0, 0.005, 3)
            out = n._smooth_one(*noisy, TRUE_Y + rng.normal(0, math.radians(4)), 'pink', t)
            if k >= 10:
                errs.append(np.linalg.norm(np.array(out[:3]) - TRUE_P))
        results[alpha] = np.mean(errs)
    assert results[0.5] < results[0.0], f'α=0.5 phải tốt hơn thô: {results}'
    assert results[0.8] < results[0.5], f'α=0.8 phải mượt hơn 0.5: {results}'


def test_min_hits_blocks_flicker():
    """Detection mới thấy 1 frame chưa được publish khi min_hits=2."""
    n = make_shim(min_hits=2)
    t = 0.2
    n._tracks_begin_frame(t)
    assert n._smooth_one(0.2, 0.0, 0.05, 0.0, 'blue', t) is None
    t = 0.4
    n._tracks_begin_frame(t)
    assert n._smooth_one(0.2, 0.0, 0.05, 0.0, 'blue', t) is not None


def test_yaw_ema_wraparound():
    """EMA yaw trên (sin,cos): đi qua ±pi không được nhảy về 0."""
    n = make_shim(min_hits=1)
    t, outs = 0.0, []
    for y in [math.pi - 0.05, -math.pi + 0.05, math.pi - 0.02, -math.pi + 0.03]:
        t += 0.2
        n._tracks_begin_frame(t)
        outs.append(math.degrees(n._smooth_one(0.2, 0.0, 0.05, y, 'green', t)[3]))
    assert all(abs(abs(v) - 180) < 5 for v in outs), f'yaw nhảy qua ±pi: {outs}'


def test_track_timeout():
    """Track quá hạn bị dọn -> ống quay lại bị coi là mới (min_hits áp lại)."""
    n = make_shim(min_hits=2, track_timeout_s=1.0)
    for t in (0.2, 0.4):
        n._tracks_begin_frame(t)
        n._smooth_one(0.2, 0.0, 0.05, 0.0, 'blue', t)
    assert len(n._tracks) == 1
    n._tracks_begin_frame(2.0)                          # 1.6s im lặng > timeout
    assert len(n._tracks) == 0
    assert n._smooth_one(0.2, 0.0, 0.05, 0.0, 'blue', 2.0) is None  # lại là track mới


# ─────────────────────────────────────────────────────────────────────────────
# D. Model thật + callback end-to-end (cần ultralytics + model; ROS đã source)
# ─────────────────────────────────────────────────────────────────────────────
_RCLPY = {'ok': False}


def _need_ros():
    try:
        import rclpy  # noqa: F401
    except ImportError:
        raise Skip('chưa source ROS (rclpy)')
    if not _RCLPY['ok']:
        import rclpy
        rclpy.init()
        _RCLPY['ok'] = True


def _need_model():
    if not os.path.isfile(MODEL_PATH):
        raise Skip(f'không thấy model {MODEL_PATH}')
    try:
        import ultralytics  # noqa: F401
    except ImportError:
        raise Skip('chưa cài ultralytics')


def build_node(**attr_over):
    """Dựng node thật: stub publisher, tiêm TF identity, tắt throttle."""
    import rclpy
    from geometry_msgs.msg import TransformStamped
    n = NodeCls()
    n.set_parameters([rclpy.parameter.Parameter(
        'enable_roi_box', rclpy.Parameter.Type.BOOL, False)])
    t = TransformStamped()
    t.header.frame_id = 'rx150/base_link'
    t.child_frame_id = 'camera_color_optical_frame'
    t.transform.rotation.w = 1.0
    t.transform.translation.z = 0.5
    n.tf_buffer.set_transform_static(t, 'test')
    n._min_period = 0.0
    for k, v in attr_over.items():
        setattr(n, k, v)
    cap = []
    n.pub_poses.publish = lambda m: cap.append(m)
    n.pub_classes.publish = n.pub_markers.publish = lambda m: None
    n.pub_debug_img.publish = lambda m: None
    n.pub_debug_img.get_subscription_count = lambda: 0
    return n, cap


def _msgs(img):
    from sensor_msgs.msg import CameraInfo
    from cv_bridge import CvBridge
    br = CvBridge()
    m = br.cv2_to_imgmsg(img, encoding='bgr8')
    m.header.frame_id = 'camera_color_optical_frame'
    d = br.cv2_to_imgmsg(np.full((480, 640), 600, np.uint16), encoding='16UC1')
    info = CameraInfo()
    info.width, info.height = 640, 480
    info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    return m, d, info


def test_full_callback_publishes_poses():
    """Callback đầy đủ trên ảnh 4 ống: phải ra >=3 pose, toạ độ hợp lý."""
    _need_ros(); _need_model()
    n, cap = build_node(smooth_alpha=0.0, min_hits=1)
    m, d, info = _msgs(scene())
    n.image_callback(m, d, info)
    n.destroy_node()
    assert cap, 'không publish gì'
    ps = cap[-1].poses
    assert len(ps) >= 3, f'chỉ {len(ps)} pose'
    for p in ps:
        assert 0.3 < p.position.z < 1.5 or abs(p.position.z - 1.1) < 0.3
        assert abs(p.orientation.z) <= 1.0 and abs(p.orientation.w) <= 1.0


def test_min_hits_end_to_end():
    """min_hits=2: frame đầu 0 pose, frame sau mới có."""
    _need_ros(); _need_model()
    n, cap = build_node(smooth_alpha=0.5, min_hits=2)
    m, d, info = _msgs(scene())
    n.image_callback(m, d, info)
    first = len(cap[-1].poses) if cap else 0
    n.image_callback(m, d, info)
    second = len(cap[-1].poses)
    n.destroy_node()
    assert first == 0, f'frame 1 phải 0 pose (được {first})'
    assert second >= 3, f'frame 2 phải có pose (được {second})'


def test_roi_box_filters_everything():
    """ROI box thu về 1cm quanh gốc -> mọi ống bị loại, poses rỗng."""
    _need_ros(); _need_model()
    import rclpy
    n, cap = build_node(smooth_alpha=0.0, min_hits=1)
    n.set_parameters([rclpy.parameter.Parameter('enable_roi_box', rclpy.Parameter.Type.BOOL, True)])
    for name, v in [('roi_x_min', 0.0), ('roi_x_max', 0.01), ('roi_y_min', 0.0),
                    ('roi_y_max', 0.01), ('roi_z_min', 0.0), ('roi_z_max', 0.01)]:
        n.set_parameters([rclpy.parameter.Parameter(name, rclpy.Parameter.Type.DOUBLE, v)])
    m, d, info = _msgs(scene())
    n.image_callback(m, d, info)
    n.destroy_node()
    assert cap and len(cap[-1].poses) == 0, 'ROI 1cm mà vẫn lọt pose'


def test_debug_image_gating():
    """Không có subscriber /yolo/image_debug -> không dựng/không publish ảnh debug."""
    _need_ros(); _need_model()
    n, _ = build_node(smooth_alpha=0.0, min_hits=1)
    dbg = []
    n.pub_debug_img.publish = lambda m: dbg.append(m)
    m, d, info = _msgs(scene())
    n.pub_debug_img.get_subscription_count = lambda: 0
    n.image_callback(m, d, info)
    assert len(dbg) == 0, 'publish debug dù không ai subscribe'
    n.pub_debug_img.get_subscription_count = lambda: 1
    n.image_callback(m, d, info)
    n.destroy_node()
    assert len(dbg) == 1, 'có subscriber mà không publish debug'


def test_imgsz_800_retina():
    """imgsz=800 + retina_masks: không vỡ, ra >= số pose của native."""
    _need_ros(); _need_model()
    n, cap = build_node(smooth_alpha=0.0, min_hits=1)
    n.infer_kwargs = dict(n.infer_kwargs, imgsz=800, iou=0.45, retina_masks=True)
    m, d, info = _msgs(scene())
    n.image_callback(m, d, info)
    n.destroy_node()
    assert cap and len(cap[-1].poses) >= 3, f'imgsz=800: {len(cap[-1].poses) if cap else 0} pose'


# ─────────────────────────────────────────────────────────────────────────────
# E. --bench: đo ms/frame của callback đầy đủ
# ─────────────────────────────────────────────────────────────────────────────
def run_bench():
    _need_ros(); _need_model()
    n, _ = build_node(smooth_alpha=0.0, min_hits=1)
    m, d, info = _msgs(scene())
    for _ in range(15):
        n.image_callback(m, d, info)
    if HAS_CUDA:
        torch.cuda.synchronize()
    t0 = time.perf_counter()
    K = 60
    for _ in range(K):
        n.image_callback(m, d, info)
    if HAS_CUDA:
        torch.cuda.synchronize()
    ms = (time.perf_counter() - t0) / K * 1000
    n.destroy_node()
    dev = 'GPU' if HAS_CUDA else 'CPU'
    print(f'\n[bench] callback đầy đủ ({dev}): {ms:6.2f} ms/frame '
          f'(mốc 2026-08-27: GPU ~8.5 ms, CPU cũ ~68 ms chỉ riêng inference)')
    print(f'[bench] ở  5 Hz: ~{ms * 5 / 10:.1f}% một core | ở 30 Hz: ~{ms * 30 / 10:.1f}%')


# ─────────────────────────────────────────────────────────────────────────────
# F. --live: fake camera 30 Hz + node thật trong 1 executor, đo nhịp topic
# ─────────────────────────────────────────────────────────────────────────────
def run_live(duration=12.0):
    _need_ros(); _need_model()
    import rclpy
    from rclpy.node import Node as RosNode
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.executors import MultiThreadedExecutor
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import PoseArray, TransformStamped
    from cv_bridge import CvBridge

    img = scene()
    br = CvBridge()
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=5)

    class FakeCam(RosNode):
        def __init__(self):
            super().__init__('fake_cam_test')
            self.p_rgb = self.create_publisher(Image, '/camera/camera/color/image_raw', qos)
            self.p_d = self.create_publisher(
                Image, '/camera/camera/aligned_depth_to_color/image_raw', qos)
            self.p_i = self.create_publisher(CameraInfo, '/camera/camera/color/camera_info', qos)
            self.depth = np.full((480, 640), 600, np.uint16)
            self.info = CameraInfo()
            self.info.width, self.info.height = 640, 480
            self.info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
            self.create_timer(1.0 / 30.0, self.tick)

        def tick(self):
            st = self.get_clock().now().to_msg()
            mr = br.cv2_to_imgmsg(img, encoding='bgr8')
            md = br.cv2_to_imgmsg(self.depth, encoding='16UC1')
            for mm in (mr, md):
                mm.header.stamp = st
                mm.header.frame_id = 'camera_color_optical_frame'
            self.info.header.stamp = st
            self.p_rgb.publish(mr)
            self.p_d.publish(md)
            self.p_i.publish(self.info)

    class Counter(RosNode):
        def __init__(self):
            super().__init__('rate_counter_test')
            self.stamps = []
            self.create_subscription(PoseArray, '/yolo/detected_tubes',
                                     lambda m: self.stamps.append(time.monotonic()), 10)

    det = NodeCls()
    t = TransformStamped()
    t.header.frame_id = 'rx150/base_link'
    t.child_frame_id = 'camera_color_optical_frame'
    t.transform.rotation.w = 1.0
    t.transform.translation.z = 0.5
    det.tf_buffer.set_transform_static(t, 'live_test')

    cam, cnt = FakeCam(), Counter()
    ex = MultiThreadedExecutor(num_threads=4)
    for nd in (cam, det, cnt):
        ex.add_node(nd)
    print(f'\n[live] chạy {duration:.0f}s: fake camera 30 Hz -> node thật -> đếm /yolo/detected_tubes')
    t_end = time.monotonic() + duration
    while time.monotonic() < t_end:
        ex.spin_once(timeout_sec=0.1)
    st = cnt.stamps
    for nd in (cam, det, cnt):
        nd.destroy_node()
    if len(st) < 3:
        print(f'[live] FAIL: chỉ nhận {len(st)} msg'); return False
    # bỏ 2s đầu (warm-up + min_hits)
    st = [x for x in st if x > st[0] + 2.0]
    gaps = [b - a for a, b in zip(st, st[1:])]
    rate = len(st) / (st[-1] - st[0]) if len(st) > 1 else 0
    print(f'[live] {len(st)} msg | {rate:.2f} Hz | gap min={min(gaps)*1e3:.0f} ms '
          f'max={max(gaps)*1e3:.0f} ms (throttle 5 Hz -> gap >= ~200 ms)')
    ok = rate <= 6.5 and min(gaps) >= 0.15
    print(f'[live] {"PASS" if ok else "FAIL"}: nhịp đúng throttle, không tràn 30 Hz')
    return ok


# ─────────────────────────────────────────────────────────────────────────────
TESTS = [
    ('A1 nắp/đáy HSV 4 màu + đảo cực', test_cap_endpoint_hsv),
    ('A2 depth median + valid_ratio', test_depth_median_and_valid_ratio),
    ('A3 deproject + transform', test_deproject_and_transform),
    ('B1 opening GPU == cv2.MORPH_OPEN', test_gpu_opening_vs_cv2),
    ('B2 mask khác độ phân giải ảnh', test_contour_scaling_across_resolutions),
    ('C1 EMA giảm rung', test_tracker_smoothing_reduces_noise),
    ('C2 min_hits chặn nhấp nháy', test_min_hits_blocks_flicker),
    ('C3 yaw EMA qua ±pi', test_yaw_ema_wraparound),
    ('C4 track timeout', test_track_timeout),
    ('D1 callback đầy đủ ra pose', test_full_callback_publishes_poses),
    ('D2 min_hits end-to-end', test_min_hits_end_to_end),
    ('D3 ROI box lọc hết', test_roi_box_filters_everything),
    ('D4 gate ảnh debug theo subscriber', test_debug_image_gating),
    ('D5 imgsz=800 + retina', test_imgsz_800_retina),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--bench', action='store_true', help='đo ms/frame callback đầy đủ')
    ap.add_argument('--live', action='store_true', help='fake camera 30Hz + đo nhịp topic')
    ap.add_argument('-k', metavar='TỪ_KHOÁ', help='chỉ chạy test có tên chứa từ khoá')
    args = ap.parse_args()

    print(f'model : {MODEL_PATH} ({"có" if os.path.isfile(MODEL_PATH) else "KHÔNG có"})')
    print(f'torch : {torch.__version__ if torch else "không có"} | '
          f'CUDA: {torch.cuda.get_device_name(0) if HAS_CUDA else "không"}\n')

    n_pass = n_fail = n_skip = 0
    for name, fn in TESTS:
        if args.k and args.k.lower() not in name.lower():
            continue
        t0 = time.perf_counter()
        try:
            fn()
            ms = (time.perf_counter() - t0) * 1000
            print(f'  PASS  {name}  ({ms:.0f} ms)')
            n_pass += 1
        except Skip as e:
            print(f'  SKIP  {name}  ({e})')
            n_skip += 1
        except Exception:
            print(f'  FAIL  {name}')
            traceback.print_exc()
            n_fail += 1

    live_ok = True
    if args.bench:
        try:
            run_bench()
        except Skip as e:
            print(f'[bench] SKIP ({e})')
    if args.live:
        try:
            live_ok = run_live()
        except Skip as e:
            print(f'[live] SKIP ({e})')

    if _RCLPY['ok']:
        import rclpy
        rclpy.shutdown()

    print(f'\nTỔNG: {n_pass} pass, {n_fail} fail, {n_skip} skip')
    sys.exit(0 if (n_fail == 0 and live_ok) else 1)


if __name__ == '__main__':
    main()
