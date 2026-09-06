#!/usr/bin/env python3
"""Bậc B1 — driver Dynamixel có đang phát joint_states đúng và đủ không?

Kiểm 4 thứ, mỗi thứ là một nguyên nhân hỏng riêng biệt ở bậc trên:
  1. Topic có tồn tại và có dữ liệu   -> xs_sdk chạy chưa?
  2. Đủ 6 tên khớp                    -> motor_configs đúng chưa? bus đọc đủ ID chưa?
  3. Tần số ~100 Hz                   -> update_rate + tranh chấp bus
  4. position không toàn 0 / không NaN -> encoder đọc được thật

Phần cứng: cần robot đã cấp nguồn và MỘT xs_sdk đang chạy (`./rx150.sh t1`).
KHÔNG phát bất kỳ lệnh nào tới robot — chỉ nghe.

Topic:  /rx150/joint_states  (sensor_msgs/JointState)

Dùng:
  python3 module_tests/run_test.py hardware/joint_states_test.py
  python3 module_tests/run_test.py hardware/joint_states_test.py -- --seconds 5 --min-hz 50
"""
from __future__ import annotations

import argparse
import math
import sys
import time

ARM_JOINTS = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]
FINGER_JOINTS = ["left_finger", "right_finger"]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--topic", default="/rx150/joint_states")
    ap.add_argument("--seconds", type=float, default=3.0,
                    help="thời gian nghe để đo tần số")
    ap.add_argument("--min-hz", type=float, default=50.0,
                    help="ngưỡng FAIL (danh nghĩa 100 Hz; <50 là có vấn đề bus)")
    args = ap.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        print(f"FAIL: không import được rclpy/sensor_msgs ({exc}).")
        print("      Hãy 'source ~/interbotix_ws/source_all.sh' trước.")
        return 1

    rclpy.init()
    node = Node("joint_states_test")
    samples: list = []

    # SensorDataQoS: xs_sdk phát best-effort; subscriber reliable sẽ im lặng
    # không nhận gì — đúng cái bẫy QoS đã ghi trong PERCEPTION_GUIDE.
    node.create_subscription(JointState, args.topic,
                             lambda m: samples.append((time.monotonic(), m)),
                             qos_profile_sensor_data)

    print(f"Nghe {args.topic} trong {args.seconds:.1f}s ...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    fails: list[str] = []

    # ── 1. có dữ liệu không ──────────────────────────────────────────
    if not samples:
        print(f"FAIL: không nhận được message nào trên {args.topic}.")
        print("      → xs_sdk chưa chạy, hoặc chạy ở namespace khác.")
        print("      Kiểm: pgrep -af xs_sdk ; ros2 topic list | grep joint_states")
        return 1
    print(f"OK  : nhận {len(samples)} message.")

    last = samples[-1][1]
    names = list(last.name)

    # ── 2. đủ tên khớp ───────────────────────────────────────────────
    missing_arm = [j for j in ARM_JOINTS if j not in names]
    if missing_arm:
        fails.append(f"thiếu khớp tay: {missing_arm}")
        print(f"FAIL: thiếu khớp {missing_arm} — có: {names}")
    else:
        print(f"OK  : đủ 5 khớp tay {ARM_JOINTS}.")

    if not any(j in names for j in FINGER_JOINTS):
        fails.append("không có left_finger/right_finger")
        print(f"FAIL: không có ngón kẹp nào trong {names}")
        print("      → xác nhận kẹp (part-present) sẽ KHÔNG hoạt động.")
    else:
        present = [j for j in FINGER_JOINTS if j in names]
        print(f"OK  : có ngón kẹp {present}.")

    # ── 3. tần số ────────────────────────────────────────────────────
    span = samples[-1][0] - samples[0][0]
    hz = (len(samples) - 1) / span if span > 0 else 0.0
    if hz < args.min_hz:
        fails.append(f"tần số {hz:.1f} Hz < {args.min_hz}")
        print(f"FAIL: {hz:.1f} Hz (danh nghĩa 100). Nghi: 2 xs_sdk trên 1 bus, "
              f"hoặc latency_timer chưa về 1 ms.")
        print("      Kiểm: pgrep -af xs_sdk  (nhiều hơn 1 dòng là hỏng)")
    else:
        print(f"OK  : {hz:.1f} Hz.")

    # ── 4. giá trị có thật không ─────────────────────────────────────
    pos = list(last.position)
    if not pos:
        fails.append("trường position rỗng")
        print("FAIL: message không có position.")
    elif any(math.isnan(p) for p in pos):
        fails.append("position có NaN")
        print(f"FAIL: position có NaN: {pos}")
    else:
        deg = {n: round(math.degrees(p), 1)
               for n, p in zip(names, pos) if n in ARM_JOINTS}
        print(f"OK  : góc hiện tại (độ) = {deg}")
        finger = next((p for n, p in zip(names, pos) if n == "left_finger"), None)
        if finger is not None:
            state = ("ĐÓNG HẾT (kẹp không khí?)" if finger <= 0.017
                     else "MỞ" if finger >= 0.035 else "ở giữa (đang giữ vật?)")
            print(f"      left_finger = {finger:.4f} m — {state}")

    # ── 5. tuổi dữ liệu (stamp so với now) ───────────────────────────
    stamp = last.header.stamp.sec + last.header.stamp.nanosec * 1e-9
    if stamp > 0:
        age = time.time() - stamp
        if age > 1.0:
            print(f"WARN: stamp cũ {age:.1f}s — lệch đồng hồ hoặc dữ liệu đứng.")
        else:
            print(f"OK  : tuổi dữ liệu {age * 1000:.0f} ms.")

    if fails:
        print(f"\n=> NO-GO ({len(fails)}): " + "; ".join(fails))
        return 1
    print("\n=> GO: bậc B1 (phần joint_states) đạt.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
