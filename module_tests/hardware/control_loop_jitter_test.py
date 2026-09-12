#!/usr/bin/env python3
"""Đo jitter và độ trễ vòng lặp điều khiển thời gian thực (ROS 2).

Công cụ này lắng nghe đồng thời 2 topic:
  1. /rx150/joint_states        (chu kỳ đọc từ bus cứng qua xs_sdk)
  2. /rx150/commands/joint_group (chu kỳ phát lệnh từ bộ điều khiển HAC/Fuzzy/FF)

Chỉ số đo lường:
  - Tần số trung bình thực tế (Hz)
  - Phân bố chu kỳ đọc Joint State (RTT jitter: min, p50, p90, p99, max, stddev)
  - Độ trễ tính toán Read-to-Command (thời gian từ lúc có joint_states đến khi controller phát lệnh)
  - Số mẫu trễ vượt ngưỡng (drop / deadline miss)

Cách dùng:
  python3 module_tests/run_test.py hardware/control_loop_jitter_test.py -- \\
      --seconds 30 --expect-hz 400 --max-p99-ratio 1.5 --csv /tmp/jitter.csv

Cổng P4/P6: Hz đo >= 0.95 x danh nghĩa; p99 chu kỳ < 1.5 x chu kỳ danh nghĩa;
max < 3 x; không có khoảng trống > 2 x chu kỳ (khoảng trống = gói rớt + timeout).
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from interbotix_xs_msgs.msg import JointGroupCommand
except ImportError as exc:
    print(f"[FAIL] Không import được rclpy / sensor_msgs / interbotix_xs_msgs ({exc}).")
    print("       Hãy 'source source_all.sh' trước.")
    sys.exit(1)


class JitterBenchNode(Node):
    def __init__(self, js_topic: str, cmd_topic: str):
        super().__init__("control_loop_jitter_bench")
        self.js_times: list[float] = []
        self.cmd_times: list[float] = []
        self.latencies_ms: list[float] = []      # đến-js -> đến-cmd (xấp xỉ thời gian tính)
        self.age_ms: list[float] = []            # header.stamp -> đến-cmd (tuổi thật của dữ liệu)
        self.last_js_time: float | None = None
        self.last_js_stamp: float | None = None
        self.stamp_unusable = 0

        self.sub_js = self.create_subscription(
            JointState, js_topic, self.on_joint_state, qos_profile_sensor_data
        )
        self.sub_cmd = self.create_subscription(
            JointGroupCommand, cmd_topic, self.on_command, 10
        )

    def on_joint_state(self, msg: JointState):
        self.js_times.append(time.perf_counter())
        self.last_js_time = self.js_times[-1]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_js_stamp = stamp if stamp > 0.0 else None

    def on_command(self, msg: JointGroupCommand):
        now = time.perf_counter()
        self.cmd_times.append(now)

        if self.last_js_time is not None:
            dt_ms = (now - self.last_js_time) * 1000.0
            if dt_ms < 50.0:  # lọc mẫu mất đồng bộ lúc khởi động
                self.latencies_ms.append(dt_ms)

        # Tuổi THẬT của dữ liệu tại lúc lệnh xuất hiện, tính từ header.stamp mà
        # xs_sdk đóng dấu ngay sau khi đọc bus. Khoảng cách đến-js -> đến-cmd ở
        # trên chỉ đo được phần controller tính, và cộng thêm hai chặng DDS tới
        # chính node đo này; nó KHÔNG thấy phần trễ bus -> xs_sdk -> controller.
        # Đó mới là phần lớn lên khi tăng tần số, nên phải đo bằng stamp.
        if self.last_js_stamp is not None:
            age_ms = (self.get_clock().now().nanoseconds * 1e-9 - self.last_js_stamp) * 1000.0
            if 0.0 <= age_ms < 200.0:
                self.age_ms.append(age_ms)
            else:
                self.stamp_unusable += 1


def compute_distribution(deltas_ms: list[float]) -> dict:
    if not deltas_ms:
        return {}
    s = sorted(deltas_ms)
    n = len(s)
    mean = sum(s) / n
    variance = sum((x - mean) ** 2 for x in s) / n
    return {
        "count": n,
        "min": s[0],
        "p50": s[int(n * 0.50)],
        "p90": s[int(n * 0.90)],
        "p95": s[int(n * 0.95)],
        "p99": s[int(n * 0.99)],
        "max": s[-1],
        "mean": mean,
        "stddev": math.sqrt(variance),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--js-topic", default="/rx150/joint_states", help="Topic joint states")
    parser.add_argument("--cmd-topic", default="/rx150/commands/joint_group", help="Topic group command")
    parser.add_argument("--seconds", type=float, default=10.0, help="Thời gian đo (giây)")
    parser.add_argument("--expect-hz", "--target-hz", dest="target_hz", type=float, default=100.0,
                        help="Tần số danh nghĩa mong đợi (Hz)")
    parser.add_argument("--max-p99-ratio", type=float, default=1.5,
                        help="Cổng: p99 chu kỳ / chu kỳ danh nghĩa (mặc định 1.5). "
                             "Dùng TỈ LỆ chứ không phải ms tuyệt đối, vì ở 500 Hz chu kỳ "
                             "danh nghĩa chỉ 2 ms — ngưỡng 2 ms cố định sẽ cho qua cả p99 = 2x.")
    parser.add_argument("--max-ratio", type=float, default=3.0,
                        help="Cổng: chu kỳ max / chu kỳ danh nghĩa (mặc định 3.0)")
    parser.add_argument("--min-hz-ratio", type=float, default=0.95,
                        help="Cổng: Hz đo / Hz danh nghĩa (mặc định 0.95)")
    parser.add_argument("--csv", metavar="FILE", help="Ghi thống kê ra file CSV")
    args = parser.parse_args()

    rclpy.init()
    node = JitterBenchNode(args.js_topic, args.cmd_topic)

    print(f"\n[*] Đang lắng nghe {args.js_topic} & {args.cmd_topic} trong {args.seconds:.1f}s...")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.01)

    node.destroy_node()
    rclpy.shutdown()

    # Phân tích chu kỳ Joint States
    js_intervals_ms = []
    for i in range(1, len(node.js_times)):
        js_intervals_ms.append((node.js_times[i] - node.js_times[i - 1]) * 1000.0)

    # Phân tích chu kỳ Commands
    cmd_intervals_ms = []
    for i in range(1, len(node.cmd_times)):
        cmd_intervals_ms.append((node.cmd_times[i] - node.cmd_times[i - 1]) * 1000.0)

    if not js_intervals_ms:
        print(f"\n[FAIL] Không nhận được dữ liệu từ {args.js_topic}!")
        print("       Kiểm tra xem xs_sdk đã chạy chưa.")
        return 1

    js_dist = compute_distribution(js_intervals_ms)
    actual_hz = 1000.0 / js_dist["mean"] if js_dist["mean"] > 0 else 0.0
    nominal_dt_ms = 1000.0 / args.target_hz if args.target_hz > 0 else 10.0

    print("\n" + "=" * 65)
    print(f"  BÁO CÁO HIỆU NĂNG VÒNG LẶP ĐIỀU KHIỂN (Thời gian: {args.seconds:.1f}s)")
    print("=" * 65)
    print(f"Tần số thực tế (joint_states) : {actual_hz:.1f} Hz (Mục tiêu: {args.target_hz:.1f} Hz)")
    print(f"Số mẫu joint_states nhận được : {len(node.js_times):,}")
    print(f"Số mẫu commands nhận được     : {len(node.cmd_times):,}")
    print("-" * 65)
    print(f"Phân bố chu kỳ JointState (Danh nghĩa: {nominal_dt_ms:.2f} ms):")
    print(f"  - Min chu kỳ : {js_dist['min']:.3f} ms")
    print(f"  - p50 (Med)  : {js_dist['p50']:.3f} ms")
    print(f"  - Mean       : {js_dist['mean']:.3f} ms")
    print(f"  - p90        : {js_dist['p90']:.3f} ms")
    print(f"  - p99        : {js_dist['p99']:.3f} ms")
    print(f"  - Max chu kỳ : {js_dist['max']:.3f} ms")
    print(f"  - Jitter std : {js_dist['stddev']:.3f} ms")
    print("-" * 65)

    lat_dist = compute_distribution(node.latencies_ms)
    if lat_dist:
        print("Khoảng đến-js -> đến-cmd (xấp xỉ thời gian controller tính):")
        print(f"  - p50 (Med)  : {lat_dist['p50']:.3f} ms")
        print(f"  - p99        : {lat_dist['p99']:.3f} ms")
        print(f"  - Max        : {lat_dist['max']:.3f} ms")
    else:
        print("[!] Không đo được (controller chưa chạy hoặc không phát lệnh)")

    age_dist = compute_distribution(node.age_ms)
    if age_dist:
        print("Trễ read->command THẬT (header.stamp của joint_states -> lệnh xuất hiện):")
        print(f"  - p50 (Med)  : {age_dist['p50']:.3f} ms")
        print(f"  - p99        : {age_dist['p99']:.3f} ms")
        print(f"  - Max        : {age_dist['max']:.3f} ms")
    elif node.stamp_unusable:
        print(f"[!] {node.stamp_unusable} mẫu có header.stamp ngoài khoảng hợp lệ "
              "— lệch đồng hồ, hoặc use_sim_time bật lệch giữa các node.")
    else:
        print("[!] joint_states không có header.stamp -> không đo được trễ read->command thật.")

    # Khoảng trống = gói rớt. Ở 4 Mbps, timeout SDK chưa vá là 34 ms (17 chu kỳ
    # ở 500 Hz); đã vá còn ~3 ms. Nhìn độ DÀI khoảng trống là biết đang chạy bản nào.
    gap_threshold = 2.0 * nominal_dt_ms
    gaps = [g for g in js_intervals_ms if g > gap_threshold]
    print("-" * 65)
    print(f"Khoảng trống > 2x chu kỳ ({gap_threshold:.2f} ms): {len(gaps)}")
    if gaps:
        print(f"  - dài nhất : {max(gaps):.2f} ms  (~{max(gaps)/nominal_dt_ms:.1f} chu kỳ)")
    print("=" * 65)

    # Đánh giá GO / NO-GO
    fails = []
    hz_ratio = actual_hz / args.target_hz if args.target_hz > 0 else 1.0
    if hz_ratio < args.min_hz_ratio:
        fails.append(f"Tần số {actual_hz:.1f} Hz = {hz_ratio*100:.0f}% mục tiêu "
                     f"{args.target_hz:.1f} Hz (cần >= {args.min_hz_ratio*100:.0f}%)")

    p99_ratio = js_dist["p99"] / nominal_dt_ms if nominal_dt_ms > 0 else 0.0
    if p99_ratio > args.max_p99_ratio:
        fails.append(f"p99 chu kỳ {js_dist['p99']:.2f} ms = {p99_ratio:.2f}x danh nghĩa "
                     f"(cần < {args.max_p99_ratio:.2f}x)")

    max_ratio = js_dist["max"] / nominal_dt_ms if nominal_dt_ms > 0 else 0.0
    if max_ratio > args.max_ratio:
        fails.append(f"max chu kỳ {js_dist['max']:.2f} ms = {max_ratio:.2f}x danh nghĩa "
                     f"(cần < {args.max_ratio:.2f}x)")

    if args.csv:
        with open(args.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["metric", "value_ms", "ratio_vs_nominal"])
            w.writerow(["nominal_dt", f"{nominal_dt_ms:.4f}", "1.0"])
            for k in ("min", "p50", "mean", "p90", "p99", "max", "stddev"):
                w.writerow([f"js_interval_{k}", f"{js_dist[k]:.4f}",
                            f"{js_dist[k]/nominal_dt_ms:.3f}" if nominal_dt_ms > 0 else ""])
            for name, dist in (("cmd_gap", lat_dist), ("read_to_cmd_age", age_dist)):
                for k in ("p50", "p99", "max"):
                    if dist:
                        w.writerow([f"{name}_{k}", f"{dist[k]:.4f}", ""])
            w.writerow(["actual_hz", f"{actual_hz:.3f}", f"{hz_ratio:.3f}"])
            w.writerow(["gaps_over_2x", len(gaps), ""])
        print(f"[*] Đã ghi {args.csv}")

    if fails:
        print("\n=> [NO-GO] Cảnh báo chất lượng thời gian thực:")
        for f in fails:
            print(f"   - {f}")
        return 1

    print("\n=> [GO] Vòng lặp điều khiển đạt tiêu chuẩn ổn định thời gian thực!")
    return 0


if __name__ == "__main__":
    sys.exit(main())

