#!/usr/bin/env python3
"""Bậc B2/B3 — TF camera↔robot có không, và detection có dùng được không?

Bài test này tồn tại vì MỘT lý do: khi thiếu TF `camera_color_optical_frame →
rx150/base_link`, `yolo_tube_detector` im lặng không publish gì, và triệu chứng
trông y hệt "model YOLO kém" — dẫn tới hàng giờ tune conf_threshold vô ích.
Nó tách bạch hai nguyên nhân đó ra.

Thứ tự kiểm (dừng sớm ở nguyên nhân gốc):
  1. TF camera → base_link tồn tại?      -> không thì mọi thứ dưới đây vô nghĩa
  2. TF có ổn định (không nhảy)?         -> nhảy = 2 nguồn cùng phát (RUNBOOK §2)
  3. /yolo/detected_tubes có dữ liệu?    -> không thì detector chưa chạy
  4. frame_id đúng rx150/base_link?      -> sai frame = gắp sai chỗ mà không ai biết
  5. Tuổi dữ liệu < detection_max_age_s? -> quá cũ thì node quyết định sẽ từ chối
  6. Pose nằm trong hộp ROI?             -> ngoài ROI = bị lọc, không phải không thấy

Phần cứng: cần camera + `static_trans_pub` (`./rx150.sh t1` rồi `t2`).
KHÔNG phát lệnh nào tới robot.

Dùng:
  python3 module_tests/run_test.py perception/tf_and_detection_test.py
  python3 module_tests/run_test.py perception/tf_and_detection_test.py -- --seconds 8
  python3 module_tests/run_test.py perception/tf_and_detection_test.py -- --allow-empty
"""
from __future__ import annotations

import argparse
import math
import sys
import time

CAM_FRAME = "camera_color_optical_frame"
BASE_FRAME = "rx150/base_link"

# Khớp mặc định của rx150_perception/config/roi_box_params.yaml. Chỉ dùng để
# BÁO CÁO (pose nằm trong/ngoài hộp), không phải để quyết định pass/fail.
ROI = dict(x=(-0.10, 0.45), y=(-0.15, 0.30), z=(-0.02, 0.25))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/yolo/detected_tubes")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--max-age", type=float, default=1.5,
                    help="khớp detection_max_age_s trong config task")
    ap.add_argument("--allow-empty", action="store_true",
                    help="không FAIL khi PoseArray rỗng (dùng khi bàn trống)")
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseArray
        from tf2_ros import Buffer, TransformListener
        import rclpy.time
    except ImportError as exc:
        print(f"FAIL: không import được rclpy/tf2_ros ({exc}).")
        print("      Hãy 'source ~/interbotix_ws/source_all.sh' trước.")
        return 1

    rclpy.init()
    node = Node("tf_and_detection_test")
    buf = Buffer()
    TransformListener(buf, node)
    msgs: list = []
    node.create_subscription(PoseArray, args.topic, msgs.append, 10)

    print(f"Nghe {args.topic} + TF trong {args.seconds:.1f}s ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)

    fails: list[str] = []

    # ── 1 + 2. TF ────────────────────────────────────────────────────
    def lookup():
        try:
            return buf.lookup_transform(BASE_FRAME, CAM_FRAME, rclpy.time.Time())
        except Exception as exc:                                  # noqa: BLE001
            return exc

    tf_a = lookup()
    if isinstance(tf_a, Exception):
        print(f"FAIL: không tra được TF {CAM_FRAME} -> {BASE_FRAME}")
        print(f"      {type(tf_a).__name__}: {tf_a}")
        print("      → Đây là nguyên nhân gốc số 1 của 'YOLO không nhận được gì'.")
        print("      → Đúng MỘT nguồn được phát TF này (RUNBOOK §2):")
        print("        static_trans_pub (chạy rx150_perception.launch.py)  ← thường dùng")
        print("        use_camera_static_tf:=true  (TF cứng, chưa hiệu chuẩn)")
        print("        use_handeye_publisher:=true (cần ~/.ros/easy_handeye2/*.yaml)")
        node.destroy_node()
        rclpy.shutdown()
        print("\n=> NO-GO: dừng ở bước 1, các bước sau vô nghĩa khi chưa có TF.")
        return 1

    ta = tf_a.transform.translation
    print(f"OK  : TF {CAM_FRAME} -> {BASE_FRAME} = "
          f"({ta.x:+.4f}, {ta.y:+.4f}, {ta.z:+.4f}) m")

    # Nhảy TF = 2 publisher cùng frame. Đo bằng cách tra lại sau 1 s.
    t1 = time.monotonic()
    while time.monotonic() - t1 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    tf_b = lookup()
    if not isinstance(tf_b, Exception):
        tb = tf_b.transform.translation
        drift = math.dist((ta.x, ta.y, ta.z), (tb.x, tb.y, tb.z))
        if drift > 1e-4:
            fails.append(f"TF nhảy {drift * 1000:.1f} mm/s")
            print(f"FAIL: TF dịch {drift * 1000:.2f} mm trong 1 s — nghi 2 nguồn "
                  f"cùng phát world<->camera (RUNBOOK §2).")
        else:
            print("OK  : TF đứng yên (một nguồn duy nhất).")

    # ── 3. có detection không ────────────────────────────────────────
    if not msgs:
        print(f"FAIL: không nhận message nào trên {args.topic}.")
        print("      → detector chưa chạy: ros2 node list | grep yolo_tube_detector")
        print("      → hoặc đang chạy HAI cái cùng tên (GPU đầy, pose nhảy).")
        node.destroy_node()
        rclpy.shutdown()
        print("\n=> NO-GO")
        return 1

    hz = (len(msgs) - 1) / args.seconds if args.seconds > 0 else 0.0
    print(f"OK  : {len(msgs)} message (~{hz:.1f} Hz; detect_rate_hz mặc định 5.0).")

    last = msgs[-1]

    # ── 4. frame_id ──────────────────────────────────────────────────
    if last.header.frame_id != BASE_FRAME:
        fails.append(f"frame_id = '{last.header.frame_id}'")
        print(f"FAIL: frame_id = '{last.header.frame_id}', cần '{BASE_FRAME}'.")
        print("      → node quyết định sẽ TỪ CHỐI toàn bộ detection (detection.py).")
    else:
        print(f"OK  : frame_id = {BASE_FRAME}")

    # ── 5. tuổi dữ liệu ──────────────────────────────────────────────
    stamp = last.header.stamp.sec + last.header.stamp.nanosec * 1e-9
    age = time.time() - stamp if stamp > 0 else float("nan")
    if not math.isnan(age):
        if age > args.max_age:
            fails.append(f"tuổi {age:.2f}s > {args.max_age}s")
            print(f"FAIL: tuổi dữ liệu {age:.2f}s > detection_max_age_s={args.max_age}s.")
        else:
            print(f"OK  : tuổi dữ liệu {age:.2f}s.")

    # ── 6. nội dung + ROI ────────────────────────────────────────────
    n = len(last.poses)
    if n == 0:
        msg = "PoseArray rỗng (không thấy ống nào)"
        if args.allow_empty:
            print(f"WARN: {msg} — bỏ qua vì --allow-empty.")
        else:
            fails.append(msg)
            print(f"FAIL: {msg}.")
            print("      Phân biệt 'không thấy' với 'thấy nhưng bị ROI lọc':")
            print("        ros2 param set /yolo_tube_detector enable_roi_box false")
            print("      Xem ảnh debug: RViz display YoloDebug (/yolo/image_debug)")
    else:
        print(f"OK  : {n} ống.")
        for i, p in enumerate(last.poses):
            q = p.position
            inside = all(lo <= v <= hi for v, (lo, hi) in
                         ((q.x, ROI["x"]), (q.y, ROI["y"]), (q.z, ROI["z"])))
            yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
            flag = "trong ROI" if inside else "NGOÀI ROI (mặc định)"
            print(f"      [{i}] ({q.x:+.3f}, {q.y:+.3f}, {q.z:+.3f}) m  "
                  f"yaw={math.degrees(yaw):+6.1f}°  {flag}")
            r = math.hypot(q.x, q.y)
            if r > 0.40:
                print(f"          ⚠ r = {r:.3f} m — ngoài tầm với thực tế của rx150.")

    node.destroy_node()
    rclpy.shutdown()

    if fails:
        print(f"\n=> NO-GO ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\n=> GO: bậc B2 + B3 đạt.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
