#!/usr/bin/env python3
"""Xem camera RealSense và kết quả YOLO realtime trong một cửa sổ OpenCV.

Chạy độc lập, không cần ROS. Nhấn ``q`` hoặc ``Esc`` để thoát.
Mặc định chỉ mở color stream vì YOLO không cần depth. Dùng ``--with-depth``
nếu muốn đồng thời kiểm tra depth stream.
"""

import argparse
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


DEFAULT_WEIGHTS = "/home/hust/interbotix_ws/src/rx150/rx150_toolbox/rx150_perception/models/best_color.pt"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", default=DEFAULT_WEIGHTS)
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--with-depth", action="store_true",
                        help="Mở thêm depth stream (không cần cho YOLO)")
    args = parser.parse_args()

    if not os.path.isfile(args.weights):
        parser.error(f"Không tìm thấy model: {args.weights}")

    print(f"Nạp model: {args.weights}")
    model = YOLO(args.weights)
    print(f"Classes: {model.names}")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    if args.with_depth:
        config.enable_stream(
            rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    try:
        pipeline.start(config)
    except Exception as exc:
        print(f"Không khởi động được RealSense: {exc}")
        return 1

    align = rs.align(rs.stream.color) if args.with_depth else None
    window = "YOLO camera test | q/Esc: thoat"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    previous = time.perf_counter()
    fps = 0.0
    try:
        while True:
            try:
                frameset = pipeline.wait_for_frames(timeout_ms=15000)
            except RuntimeError as exc:
                print(f"Không nhận được frame: {exc}")
                print("Kiểm tra camera có đang bị rs_launch/rviz/realsense-viewer "
                      "hoặc một tiến trình capture khác sử dụng không.")
                print("Dừng test. Hãy tắt tiến trình camera khác rồi chạy lại.")
                return 1
            frames = align.process(frameset) if align is not None else frameset
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame() if args.with_depth else None
            if not color_frame or (args.with_depth and not depth_frame):
                continue
            image = np.asanyarray(color_frame.get_data())
            result = model.predict(image, conf=args.confidence, verbose=False)[0]
            view = result.plot()
            now = time.perf_counter()
            instant = 1.0 / max(now - previous, 1e-6)
            previous = now
            fps = 0.9 * fps + 0.1 * instant if fps else instant
            count = len(result.boxes) if result.boxes is not None else 0
            cv2.putText(view, f"FPS: {fps:.1f}  detections: {count}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow(window, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
