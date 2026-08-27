#!/usr/bin/env python3
"""
test_tube_orientation.py — Test nhận diện hướng ống nghiệm (nắp/đáy) bằng model Segmentation.

Chạy ĐỘC LẬP, không cần ROS hay robot:
    python3 test_tube_orientation.py

Yêu cầu:
    - Camera RealSense cắm USB
    - pip install ultralytics pyrealsense2 opencv-python numpy

Hiển thị:
    - Viền mask contour (vàng) quanh mỗi ống
    - Chấm đỏ "Cap" = đầu nắp (sáng hơn), chấm xanh "Bot" = đầu đáy
    - Đường trục + góc yaw (°)
    - Nhãn màu ống + confidence

Nhấn 'q' để thoát.
"""
import math
import os
import sys

import cv2
import numpy as np

# ── Load model ──────────────────────────────────────────────────────────
MODEL_PATH = os.path.join(os.path.dirname(__file__), '..', 'models', 'best.pt')
if not os.path.isfile(MODEL_PATH):
    # Thử tìm ở vị trí tuyệt đối
    MODEL_PATH = os.path.expanduser(
        '~/interbotix_ws/src/rx150_perception/models/best.pt')
if not os.path.isfile(MODEL_PATH):
    print(f'❌ Không tìm thấy model tại {MODEL_PATH}')
    sys.exit(1)

print(f'Đang nạp model: {MODEL_PATH}')
from ultralytics import YOLO
model = YOLO(MODEL_PATH)
print(f'  Task: {model.task}, Classes: {model.names}')
CONF = 0.4

# ── Chọn device: ép lên GPU rời nếu có ─────────────────────────────────
# Để trống device thì ultralytics tự chọn, và nó im lặng rơi về CPU khi torch
# không khớp driver NVIDIA — CPU chậm hơn ~12x (68 vs 5.7 ms/frame) và ăn hết core.
try:
    import torch
    if torch.cuda.is_available():
        DEVICE = 'cuda'
        torch.backends.cudnn.benchmark = True
        print(f'  Device: cuda ({torch.cuda.get_device_name(0)}), torch {torch.__version__}')
    else:
        DEVICE = 'cpu'
        print(f'  ⚠ Device: CPU — torch {torch.__version__} (build CUDA {torch.version.cuda}) '
              'không thấy GPU. Cài lại torch khớp driver (xem `nvidia-smi`).')
except ImportError:
    DEVICE = 'cpu'
    print('  ⚠ Device: CPU — không import được torch.')

# ── Mở camera RealSense ────────────────────────────────────────────────
try:
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    align = rs.align(rs.stream.color)
    pipe.start(cfg)
    USE_RS = True
    print('✅ Camera RealSense đã mở (640×480, 30fps)')
except Exception as e:
    print(f'⚠️ Không mở được RealSense ({e}) — thử webcam thường (không có depth)')
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print('❌ Không mở được camera nào.')
        sys.exit(1)
    USE_RS = False
    print('✅ Webcam mở thành công (không có depth)')


def patch_brightness(image, u, v, window=15):
    """Brightness score = V_mean - 0.3*S_mean (nắp trắng: V cao, S thấp)."""
    h, w = image.shape[:2]
    half = window // 2
    v_min, v_max = max(0, v - half), min(h, v + half + 1)
    u_min, u_max = max(0, u - half), min(w, u + half + 1)
    patch = image[v_min:v_max, u_min:u_max]
    if patch.size == 0:
        return 0.0
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    val = float(np.mean(hsv[:, :, 2]))
    sat = float(np.mean(hsv[:, :, 1]))
    return val - sat * 0.3


def process_frame(frame):
    """Chạy YOLO + xác định hướng nắp/đáy cho mỗi ống."""
    results = model(frame, conf=CONF, device=DEVICE, verbose=False)
    display = frame.copy()
    info_lines = []

    for r in results:
        has_masks = (hasattr(r, 'masks') and r.masks is not None and len(r.masks) > 0)
        boxes = r.boxes if (hasattr(r, 'boxes') and r.boxes is not None) else []

        for i in range(len(boxes)):
            box = boxes[i]
            cls_id = int(box.cls[0].cpu().numpy())
            cls_name = model.names.get(cls_id, f'tube_{cls_id}')
            conf = float(box.conf[0].cpu().numpy())
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # Vẽ bbox mỏng
            cv2.rectangle(display, (x1, y1), (x2, y2), (180, 180, 180), 1)

            cap_u, cap_v, bot_u, bot_v = None, None, None, None
            yaw_deg = 0.0

            # ── Segmentation mask ───────────────────────────────────
            if has_masks and i < len(r.masks):
                # 1. Chuyển mask tensor sang ảnh nhị phân kích thước frame
                m_raw = r.masks[i].data[0].cpu().numpy()
                m_img = cv2.resize(m_raw, (frame.shape[1], frame.shape[0]))
                m_bin = (m_img > 0.5).astype(np.uint8) * 255

                # 2. Cắt mask theo Bounding Box (x1,y1,x2,y2) để không bị loang sang ống bên cạnh
                box_mask = np.zeros_like(m_bin)
                pad = 4
                box_mask[max(0, y1-pad):min(frame.shape[0], y2+pad),
                         max(0, x1-pad):min(frame.shape[1], x2+pad)] = 255
                m_bin = cv2.bitwise_and(m_bin, box_mask)

                # 3. Lọc nhiễu & tách cầu dính bằng Morphological Opening (Erosion + Dilation)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                m_bin = cv2.morphologyEx(m_bin, cv2.MORPH_OPEN, kernel)

                # 4. Tìm contour lớn nhất trong vùng bounding box
                cnts, _ = cv2.findContours(m_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if cnts:
                    contour = max(cnts, key=cv2.contourArea)

                    if cv2.contourArea(contour) >= 100:
                        # Vẽ mask contour
                        cv2.drawContours(display, [contour], -1, (0, 255, 255), 2)

                        # Filled mask bán trong suốt
                        overlay = display.copy()
                        cv2.fillPoly(overlay, [contour], (0, 200, 200))
                        cv2.addWeighted(overlay, 0.2, display, 0.8, 0, display)

                        # minAreaRect trên contour
                        rect = cv2.minAreaRect(contour)
                        rect_center, rect_size, rect_angle = rect
                        rw, rh = rect_size

                        # Trục dài = hướng ống
                        if rw < rh:
                            angle_rad = math.radians(rect_angle + 90)
                        else:
                            angle_rad = math.radians(rect_angle)

                        u_mc, v_mc = rect_center
                        dx = math.cos(angle_rad)
                        dy = math.sin(angle_rad)
                        half_len = max(rw, rh) / 2.0

                        # 2 cực (60% từ tâm)
                        pa_u = int(u_mc + half_len * 0.6 * dx)
                        pa_v = int(v_mc + half_len * 0.6 * dy)
                        pb_u = int(u_mc - half_len * 0.6 * dx)
                        pb_v = int(v_mc - half_len * 0.6 * dy)

                        # So sánh brightness
                        ba = patch_brightness(frame, pa_u, pa_v, 15)
                        bb = patch_brightness(frame, pb_u, pb_v, 15)

                        if ba >= bb:
                            cap_u, cap_v = pa_u, pa_v
                            bot_u, bot_v = pb_u, pb_v
                        else:
                            cap_u, cap_v = pb_u, pb_v
                            bot_u, bot_v = pa_u, pa_v

                        yaw_deg = math.degrees(math.atan2(
                            bot_v - cap_v, bot_u - cap_u))

            # ── Fallback: bbox minAreaRect ──────────────────────────
            if cap_u is None:
                crop = frame[max(0, y1):min(frame.shape[0], y2),
                             max(0, x1):min(frame.shape[1], x2)]
                if crop.size > 0:
                    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                    _, thresh = cv2.threshold(
                        gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                    cnts, _ = cv2.findContours(
                        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if cnts:
                        rect = cv2.minAreaRect(max(cnts, key=cv2.contourArea))
                        yaw_deg = rect[2]
                        cx_crop = int(rect[0][0]) + max(0, x1)
                        cy_crop = int(rect[0][1]) + max(0, y1)
                        cap_u, cap_v = cx_crop, cy_crop
                        bot_u, bot_v = cx_crop, cy_crop  # không phân biệt được

            # ── Vẽ kết quả ──────────────────────────────────────────
            if cap_u is not None and bot_u is not None:
                # Nắp (đỏ) + Đáy (xanh)
                cv2.circle(display, (cap_u, cap_v), 8, (0, 0, 255), -1)
                cv2.circle(display, (bot_u, bot_v), 8, (0, 255, 0), -1)
                # Trục
                cv2.line(display, (cap_u, cap_v), (bot_u, bot_v),
                         (0, 255, 255), 2)
                # Mũi tên hướng nắp→đáy
                cv2.arrowedLine(display, (cap_u, cap_v), (bot_u, bot_v),
                                (255, 0, 255), 2, tipLength=0.2)
                # Label
                cv2.putText(display, 'Cap', (cap_u - 20, cap_v - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
                cv2.putText(display, 'Bot', (bot_u - 20, bot_v - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

            # Info text trên bbox
            label = f'{cls_name} ({conf:.2f}) yaw:{yaw_deg:.0f}deg'
            cv2.putText(display, label, (x1, max(y1 - 8, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            info_lines.append(f'  [{i}] {cls_name:>8} conf={conf:.2f} yaw={yaw_deg:.0f}°')

    # Header
    n = len(info_lines)
    cv2.putText(display, f'Tubes: {n}  |  Press Q to quit',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return display, info_lines


# ── Main loop ───────────────────────────────────────────────────────────
print('\n══════════════════════════════════════════')
print('  TEST NHẬN DIỆN HƯỚNG ỐNG NGHIỆM')
print('  Nắp = chấm ĐỎ  |  Đáy = chấm XANH')
print('  Nhấn Q để thoát')
print('══════════════════════════════════════════\n')

try:
    while True:
        if USE_RS:
            frames = pipe.wait_for_frames()
            frames = align.process(frames)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
        else:
            ret, frame = cap.read()
            if not ret:
                break

        display, infos = process_frame(frame)

        # In thông tin lên terminal (mỗi 30 frame ~ 1s)
        if infos:
            for line in infos:
                print(line)

        cv2.imshow('Tube Orientation Test', display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass
finally:
    if USE_RS:
        pipe.stop()
    else:
        cap.release()
    cv2.destroyAllWindows()
    print('\nĐã thoát.')
