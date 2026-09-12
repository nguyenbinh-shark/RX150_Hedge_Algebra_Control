#!/usr/bin/env python3
"""Đo đạc hiệu năng bus Dynamixel (RTT / Jitter / Packet Loss) độc lập không qua ROS.

Công cụ này kết nối trực tiếp vào /dev/ttyDXL qua Dynamixel SDK, thực hiện chuỗi
GroupSyncRead (0x82) và GroupFastSyncRead (0x8A) liên tục để đo đạc chính xác:
  - Thời gian phản hồi vòng lặp (Round-Trip Time RTT: min, p50, p90, p99, max)
  - Tỷ lệ mất gói / lỗi truyền thông
  - Tần số giới hạn lý thuyết tối đa của bus cứng ($f_{max} = 1 / RTT_{mean}$)
  - So sánh trực tiếp Standard Sync Read vs Fast Sync Read

Cách dùng:
  python3 module_tests/hardware/dxl_bus_bench.py --help
  python3 module_tests/run_test.py hardware/dxl_bus_bench.py -- --baud 1000000 --n 5000
  python3 module_tests/run_test.py hardware/dxl_bus_bench.py -- \\
      --baud 4000000 --ids 1,2,3,4,5,6 --n 20000 --mode both --csv /tmp/bench_4m.csv

Cổng P0 (1 Mbps): p50 RTT ~ 2.0-2.6 ms, 0 lỗi / 5000.
Cổng P3 (4 Mbps): p99 < 2.5 ms, 0 lỗi / 20000. Lỗi > 0.01% -> tụt về 2 Mbps.
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import subprocess
import sys
import time

try:
    from dynamixel_sdk import (
        COMM_SUCCESS,
        GroupSyncRead,
        PacketHandler,
        PortHandler,
    )
except ImportError:
    print("[FAIL] Không tìm thấy dynamixel_sdk. Hãy 'source source_all.sh' trước.")
    sys.exit(1)

class PortHandlerLowLatency(PortHandler):
    """PortHandler với timeout gói tính theo latency_timer THẬT của FTDI.

    Python SDK hard-code LATENCY_TIMER = 16 (port_handler.py) và tính
        packet_timeout = tx_per_byte * len + LATENCY_TIMER*2 + 2
    Ở 4 Mbps con số đó ra ~34 ms: MỘT gói rớt cũng nuốt 34 ms, tức 17 chu kỳ ở
    500 Hz. Nếu không override, p99 mà bench này báo sẽ là p99 của timeout SDK
    chứ không phải của bus — đúng cái nó sinh ra để đo. udev rule của repo đặt
    latency_timer = 1 ms, nên đây mới là con số đúng.

    Đây là bản Python song sinh của PortHandlerLowLatency trong
    dynamixel_workbench_toolbox (patch vendor 0001) — giữ hai bên cùng công thức
    thì số đo mới so sánh được với hành vi thật của xs_sdk.
    """

    def __init__(self, port_name: str, latency_ms: float = 1.0):
        super().__init__(port_name)
        self.latency_ms = latency_ms

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (
            (self.tx_time_per_byte * packet_length) + (self.latency_ms * 2.0) + 1.0
        )


def read_sysfs_latency(device: str):
    """latency_timer thật mà kernel đang dùng, hoặc None nếu không đọc được."""
    try:
        name = os.path.basename(os.path.realpath(device))
        with open(f"/sys/bus/usb-serial/devices/{name}/latency_timer") as fh:
            return int(fh.read().strip())
    except Exception:
        return None


# Control Table
ADDR_PRESENT_POSITION = 132  # 4 bytes
LEN_PRESENT_POSITION = 4
ADDR_PRESENT_PWM_CUR_VEL_POS = 124  # 12 bytes: PWM(2) + Current(2) + Velocity(4) + Position(4)
LEN_PRESENT_PWM_CUR_VEL_POS = 12


def check_port_lock(device: str) -> bool:
    try:
        real_dev = os.path.realpath(device)
        out = subprocess.check_output(["fuser", real_dev], stderr=subprocess.DEVNULL)
        pids = [int(p) for p in out.decode().split() if p.isdigit()]
        if pids:
            print(f"[FAIL] Cổng {device} ({real_dev}) đang bị tiến trình PID {pids} chiếm giữ!")
            print("       Hãy tắt xs_sdk hoặc các tiến trình ROS trước khi chạy benchmark.")
            return True
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False
    except Exception:
        pass
    return False


def run_benchmark(
    port: PortHandler,
    packet: PacketHandler,
    motor_ids: list[int],
    addr: int,
    length: int,
    count: int,
    use_fast: bool,
) -> dict:
    mode_name = "Fast Sync Read (0x8A)" if use_fast else "Standard Sync Read (0x82)"
    print(f"\n[*] Đang chạy benchmark: {mode_name}")
    print(f"    - Số chu kỳ : {count:,}")
    print(f"    - Motor IDs : {motor_ids}")
    print(f"    - Address   : {addr} (length: {length} bytes)")

    group_read = GroupSyncRead(port, packet, addr, length)
    for m_id in motor_ids:
        group_read.addParam(m_id)

    rtts_us = []
    errors = 0
    consecutive_errors = 0
    max_consecutive_errors = 0

    # Warmup
    for _ in range(50):
        if use_fast:
            group_read.fastSyncRead()
        else:
            group_read.txRxPacket()

    t_start = time.perf_counter()
    for i in range(count):
        t0 = time.perf_counter_ns()
        if use_fast:
            res = group_read.fastSyncRead()
        else:
            res = group_read.txRxPacket()
        t1 = time.perf_counter_ns()

        if res == COMM_SUCCESS:
            rtts_us.append((t1 - t0) / 1000.0)  # us
            consecutive_errors = 0
        else:
            errors += 1
            consecutive_errors += 1
            if consecutive_errors > max_consecutive_errors:
                max_consecutive_errors = consecutive_errors

    total_time = time.perf_counter() - t_start
    group_read.clearParam()

    if not rtts_us:
        return {"error": "Tất cả các lượt đọc đều thất bại"}

    rtts_us.sort()
    n = len(rtts_us)
    mean_us = sum(rtts_us) / n
    p50_us = rtts_us[int(n * 0.50)]
    p90_us = rtts_us[int(n * 0.90)]
    p95_us = rtts_us[int(n * 0.95)]
    p99_us = rtts_us[int(n * 0.99)]
    min_us = rtts_us[0]
    max_us = rtts_us[-1]
    variance = sum((x - mean_us) ** 2 for x in rtts_us) / n
    stddev_us = math.sqrt(variance)

    max_freq_hz = 1_000_000.0 / mean_us if mean_us > 0 else 0.0

    return {
        "mode": mode_name,
        "count": count,
        "success": n,
        "errors": errors,
        "loss_pct": (errors / count) * 100.0,
        "max_streak": max_consecutive_errors,
        "total_time_s": total_time,
        "min_us": min_us,
        "mean_us": mean_us,
        "p50_us": p50_us,
        "p90_us": p90_us,
        "p95_us": p95_us,
        "p99_us": p99_us,
        "max_us": max_us,
        "stddev_us": stddev_us,
        "max_freq_hz": max_freq_hz,
    }


def print_stats(stats: dict):
    if "error" in stats:
        print(f"[FAIL] Lỗi: {stats['error']}")
        return

    print("\n" + "=" * 50)
    print(f"  KẾT QUẢ: {stats['mode']}")
    print("=" * 50)
    print(f"Số mẫu hoàn thành : {stats['success']:,} / {stats['count']:,}")
    print(f"Số lỗi truyền tin : {stats['errors']} ({stats['loss_pct']:.3f}%)")
    print(f"Lỗi liên tiếp max : {stats['max_streak']}")
    print(f"Tổng thời gian    : {stats['total_time_s']:.2f} s")
    print("-" * 50)
    print(f"Min RTT           : {stats['min_us']:.1f} µs  ({stats['min_us']/1000.0:.3f} ms)")
    print(f"Mean RTT          : {stats['mean_us']:.1f} µs  ({stats['mean_us']/1000.0:.3f} ms)")
    print(f"p50 (Median)      : {stats['p50_us']:.1f} µs  ({stats['p50_us']/1000.0:.3f} ms)")
    print(f"p90               : {stats['p90_us']:.1f} µs  ({stats['p90_us']/1000.0:.3f} ms)")
    print(f"p95               : {stats['p95_us']:.1f} µs  ({stats['p95_us']/1000.0:.3f} ms)")
    print(f"p99               : {stats['p99_us']:.1f} µs  ({stats['p99_us']/1000.0:.3f} ms)")
    print(f"Max RTT           : {stats['max_us']:.1f} µs  ({stats['max_us']/1000.0:.3f} ms)")
    print(f"StdDev Jitter     : {stats['stddev_us']:.1f} µs")
    print("-" * 50)
    print(f"Tần số bus tối đa : {stats['max_freq_hz']:.1f} Hz (lý thuyết)")
    print("=" * 50)


def print_comparison(s_std: dict, s_fast: dict):
    print("\n" + "=" * 68)
    print(f"{'Chỉ số':<24} | {'Standard (0x82)':<18} | {'Fast (0x8A)':<18}")
    print("-" * 68)
    print(f"{'Mean RTT':<24} | {s_std['mean_us']/1000.0:>10.3f} ms      | {s_fast['mean_us']/1000.0:>10.3f} ms")
    print(f"{'p50 (Median)':<24} | {s_std['p50_us']/1000.0:>10.3f} ms      | {s_fast['p50_us']/1000.0:>10.3f} ms")
    print(f"{'p99':<24} | {s_std['p99_us']/1000.0:>10.3f} ms      | {s_fast['p99_us']/1000.0:>10.3f} ms")
    print(f"{'Max RTT':<24} | {s_std['max_us']/1000.0:>10.3f} ms      | {s_fast['max_us']/1000.0:>10.3f} ms")
    print(f"{'StdDev':<24} | {s_std['stddev_us']:>10.1f} µs      | {s_fast['stddev_us']:>10.1f} µs")
    print(f"{'Packet Loss':<24} | {s_std['loss_pct']:>10.3f} %       | {s_fast['loss_pct']:>10.3f} %")
    print(f"{'Max Loop Rate':<24} | {s_std['max_freq_hz']:>10.1f} Hz      | {s_fast['max_freq_hz']:>10.1f} Hz")
    speedup = s_std['mean_us'] / s_fast['mean_us'] if s_fast['mean_us'] > 0 else 1.0
    print("-" * 68)
    print(f"-> Fast Sync Read giúp giảm RTT gấp {speedup:.2f}x ({s_std['mean_us'] - s_fast['mean_us']:.1f} µs/chu kỳ)!")
    print("=" * 68)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyDXL", help="Cổng serial (/dev/ttyDXL)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate (1000000 hoặc 4000000)")
    parser.add_argument("--count", "--n", "-n", dest="count", type=int, default=5000,
                        help="Số chu kỳ đo (mặc định 5000)")
    parser.add_argument("--ids", default="1,2,3,4,5,6",
                        help="Danh sách motor IDs (mặc định 1,2,3,4,5,6 — gồm cả gripper)")
    parser.add_argument("--csv", metavar="FILE",
                        help="Ghi bảng thống kê ra file CSV (một dòng mỗi chế độ)")
    parser.add_argument("--latency-ms", type=float, default=1.0,
                        help="latency_timer giả định khi tính timeout gói (mặc định 1.0; "
                             "đặt 16 để tái lập hành vi mặc định của SDK)")
    parser.add_argument("--mode", choices=["standard", "fast", "both"], default="both", help="Chế độ đọc")
    parser.add_argument("--read-mode", choices=["pos", "full"], default="pos",
                        help="'pos': 4 bytes Position, 'full': 12 bytes PWM+Current+Vel+Pos")
    args = parser.parse_args()

    motor_ids = [int(x.strip()) for x in args.ids.split(",") if x.strip()]
    addr = ADDR_PRESENT_PWM_CUR_VEL_POS if args.read_mode == "full" else ADDR_PRESENT_POSITION
    length = LEN_PRESENT_PWM_CUR_VEL_POS if args.read_mode == "full" else LEN_PRESENT_POSITION

    dev = args.port
    if not os.path.exists(dev):
        usb_ports = glob.glob("/dev/ttyUSB*")
        if usb_ports:
            dev = usb_ports[0]
        else:
            print(f"[FAIL] Không tìm thấy thiết bị {dev}!")
            return 1

    if check_port_lock(dev):
        return 1

    sysfs_latency = read_sysfs_latency(dev)
    if sysfs_latency is None:
        print(f"[WARN] Không đọc được latency_timer của {dev}; giả định {args.latency_ms} ms.")
    elif abs(sysfs_latency - args.latency_ms) > 0.01:
        print(f"[WARN] latency_timer thật = {sysfs_latency} ms nhưng bench tính theo "
              f"{args.latency_ms} ms.")
        print("       Timeout sẽ quá ngắn -> bão lỗi giả, triệu chứng GIỐNG HỆT lỗi baud.")
        print("       Sửa udev rule về 1 ms, hoặc chạy lại với --latency-ms "
              f"{sysfs_latency}.")
    else:
        print(f"[*] latency_timer = {sysfs_latency} ms (khớp giả định của bench).")

    port = PortHandlerLowLatency(dev, latency_ms=args.latency_ms)
    packet = PacketHandler(2.0)

    if not port.openPort():
        print(f"[FAIL] Không thể mở cổng {dev}!")
        return 1

    port.setBaudRate(args.baud)

    # Ping check
    print(f"[*] Kiểm tra kết nối tới các motor {motor_ids} tại {args.baud:,} bps...")
    for m_id in motor_ids:
        _, res, _ = packet.ping(port, m_id)
        if res != COMM_SUCCESS:
            print(f"[FAIL] Không ping được motor ID {m_id}!")
            port.closePort()
            return 1
    print("    -> Tất cả motor sẵn sàng.")

    std_stats = None
    fast_stats = None
    try:
        if args.mode == "both":
            std_stats = run_benchmark(port, packet, motor_ids, addr, length, args.count, use_fast=False)
            fast_stats = run_benchmark(port, packet, motor_ids, addr, length, args.count, use_fast=True)
            print_stats(std_stats)
            print_stats(fast_stats)
            print_comparison(std_stats, fast_stats)
        elif args.mode == "fast":
            fast_stats = run_benchmark(port, packet, motor_ids, addr, length, args.count, use_fast=True)
            print_stats(fast_stats)
        else:
            std_stats = run_benchmark(port, packet, motor_ids, addr, length, args.count, use_fast=False)
            print_stats(std_stats)
    finally:
        port.closePort()

    if args.csv:
        rows = [r for r in (std_stats, fast_stats) if r and "error" not in r]
        if rows:
            fields = ["mode", "baud", "latency_ms", "ids", "read_len", "count", "success",
                      "errors", "loss_pct", "max_streak", "min_us", "mean_us", "p50_us",
                      "p90_us", "p95_us", "p99_us", "max_us", "stddev_us", "max_freq_hz"]
            with open(args.csv, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
                w.writeheader()
                for r in rows:
                    w.writerow({**r, "baud": args.baud, "latency_ms": args.latency_ms,
                                "ids": args.ids, "read_len": length})
            print(f"\n[*] Đã ghi {len(rows)} dòng vào {args.csv}")
        else:
            print(f"\n[WARN] Không có kết quả hợp lệ để ghi vào {args.csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

