#!/usr/bin/env python3
"""Công cụ quản lý baudrate Dynamixel cho cánh tay RX150 (Protocol 2.0).

Chức năng:
  --scan                   : Quét các baudrate (57600, 1M, 2M, 3M, 4M) tìm motor IDs 1-10
  --set-baud <RATE>        : Đổi baudrate tất cả motor (hỗ trợ 1000000, 4000000, ...)
  --restore                : Đưa tất cả motor về 1000000 bps (1 Mbps)
  --ping                   : Ping tất cả motor tại baudrate hiện tại (--baud)
  --factory-reset --id <N> : Reset xuất xưởng motor N (về 57600 bps, ID 1)

Yêu cầu an toàn:
  - Tự động kiểm tra xung đột cổng (/dev/ttyDXL đang bị xs_sdk chiếm).
  - Tắt Torque Enable (addr 64) trước khi ghi EEPROM Baud_Rate (addr 8).
  - Xác nhận lại bằng ping ở baudrate mới sau khi đổi.
"""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys
import time

try:
    from dynamixel_sdk import (
        COMM_SUCCESS,
        PacketHandler,
        PortHandler,
    )
except ImportError:
    print("[FAIL] Chưa cài dynamixel_sdk hoặc chưa source ROS 2.")
    print("       Hãy chạy: source /opt/ros/humble/setup.bash hoặc source source_all.sh")
    sys.exit(1)

# Control Table Addresses (Protocol 2.0 - X-series: XM430, XL430, XC430)
ADDR_MODEL_NUMBER = 0      # 2 bytes
ADDR_FIRMWARE_VER = 6      # 1 byte
ADDR_ID = 7                # 1 byte
ADDR_BAUD_RATE = 8         # 1 byte (EEPROM)
ADDR_TORQUE_ENABLE = 64    # 1 byte (RAM)
ADDR_LED = 65              # 1 byte (RAM)
ADDR_PRESENT_POSITION = 132  # 4 bytes

# Baudrate Map per Dynamixel Protocol 2.0 Control Table
BAUD_MAP = {
    9600: 0,
    57600: 1,
    115200: 2,
    1000000: 3,
    2000000: 4,
    3000000: 5,
    4000000: 6,
    4500000: 7,
}
REV_BAUD_MAP = {v: k for k, v in BAUD_MAP.items()}

COMMON_BAUDRATES = [1000000, 4000000, 2000000, 3000000, 57600, 115200]
ARM_JOINT_IDS = [1, 2, 3, 4, 5, 6]  # 1: waist, 2: shoulder, 3: elbow, 4: wrist_angle, 5: wrist_rotate, 6: gripper


def check_port_in_use(device: str) -> bool:
    """Kiểm tra xem có tiến trình nào đang mở file device không."""
    try:
        real_dev = os.path.realpath(device)
        out = subprocess.check_output(["fuser", real_dev], stderr=subprocess.DEVNULL)
        pids = [int(p) for p in out.decode().split() if p.isdigit()]
        if pids:
            # Lấy tên tiến trình
            names = []
            for pid in pids:
                try:
                    with open(f"/proc/{pid}/cmdline", "rb") as f:
                        cmd = f.read().replace(b"\0", b" ").decode(errors="ignore").strip()
                        names.append(f"PID {pid} ({cmd})")
                except Exception:
                    names.append(f"PID {pid}")
            print(f"[FAIL] Thiết bị {device} ({real_dev}) đang bị tiến trình khác chiếm giữ:")
            for n in names:
                print(f"       - {n}")
            print("       Hãy tắt tiến trình (ví dụ: killall xs_sdk) trước khi thao tác bus trực tiếp.")
            return True
    except FileNotFoundError:
        pass
    except subprocess.CalledProcessError:
        # fuser trả exit code 1 nghĩa là không có tiến trình nào giữ cổng
        return False
    except Exception:
        pass
    return False


def get_model_name(model_num: int) -> str:
    models = {
        1020: "XL430-W250",
        1030: "XM430-W210",
        1040: "XM430-W350",
        1050: "XH430-W210",
        1060: "XH430-W350",
        1110: "XC430-W150",
        1120: "XC430-W240",
    }
    return models.get(model_num, f"Unknown ({model_num})")


def scan_bus(port: PortHandler, packet: PacketHandler, max_id: int = 10) -> dict[int, dict]:
    """Quét tất cả baudrate phổ biến tìm các motor."""
    found_motors = {}
    print(f"\n[*] Đang quét bus qua cổng: {port.port_name} (IDs 1..{max_id})...")

    for baud in COMMON_BAUDRATES:
        port.setBaudRate(baud)
        time.sleep(0.02)
        print(f"    - Thử baudrate {baud:,} bps...", end="", flush=True)
        detected_at_baud = 0

        for dxl_id in range(1, max_id + 1):
            if dxl_id in found_motors:
                continue
            model_num, dxl_comm_result, dxl_error = packet.ping(port, dxl_id)
            if dxl_comm_result == COMM_SUCCESS:
                # Đọc thêm firmware
                fw, _, _ = packet.read1ByteTxRx(port, dxl_id, ADDR_FIRMWARE_VER)
                found_motors[dxl_id] = {
                    "id": dxl_id,
                    "baud": baud,
                    "model_num": model_num,
                    "model_name": get_model_name(model_num),
                    "fw": fw,
                }
                detected_at_baud += 1

        if detected_at_baud > 0:
            print(f" -> Tìm thấy {detected_at_baud} motor(s)!")
        else:
            print(" (không có)")

    return found_motors


def print_motors_table(motors: dict[int, dict]):
    if not motors:
        print("\n[!] Không phát hiện được motor nào! Kiểm tra nguồn và cáp tín hiệu.")
        return
    print("\n" + "=" * 65)
    print(f"{'ID':<5} | {'Model':<15} | {'Model ID':<10} | {'Baudrate':<12} | {'FW Ver':<8}")
    print("-" * 65)
    for dxl_id in sorted(motors.keys()):
        info = motors[dxl_id]
        print(f"{info['id']:<5} | {info['model_name']:<15} | {info['model_num']:<10} | {info['baud']:<12,d} | {info['fw']:<8}")
    print("=" * 65)


def set_baudrate(port: PortHandler, packet: PacketHandler, target_baud: int) -> bool:
    if target_baud not in BAUD_MAP:
        print(f"[FAIL] Baudrate {target_baud} không hỗ trợ! Chọn một trong: {list(BAUD_MAP.keys())}")
        return False

    target_val = BAUD_MAP[target_baud]
    print(f"\n[*] Bắt đầu quy trình đổi baudrate sang {target_baud:,} bps (Address 8 = {target_val})...")

    # 1. Quét tìm vị trí hiện tại của các motor
    motors = scan_bus(port, packet, max_id=7)
    if not motors:
        print("[FAIL] Không tìm thấy motor nào để cấu hình!")
        return False

    print_motors_table(motors)

    already_at_target = [m_id for m_id, m in motors.items() if m["baud"] == target_baud]
    need_change = [m_id for m_id, m in motors.items() if m["baud"] != target_baud]

    if not need_change:
        print(f"\n[OK] Tất cả {len(motors)} motor đã ở baudrate {target_baud:,} bps. Không cần thay đổi.")
        return True

    print(f"\n[+] Số motor cần đổi baud: {len(need_change)} ({need_change})")

    # Nhóm các motor theo baudrate hiện tại của chúng để giao tiếp
    baud_groups: dict[int, list[int]] = {}
    for m_id in need_change:
        cur_baud = motors[m_id]["baud"]
        baud_groups.setdefault(cur_baud, []).append(m_id)

    success_ids = []
    for cur_baud, ids in baud_groups.items():
        port.setBaudRate(cur_baud)
        time.sleep(0.02)
        print(f"\n[*] Kết nối nhóm motor {ids} tại {cur_baud:,} bps:")

        for dxl_id in ids:
            # 1. Tắt Torque Enable (bắt buộc trước khi ghi EEPROM)
            res, err = packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            if res != COMM_SUCCESS:
                print(f"    - ID {dxl_id}: [FAIL] không tắt được Torque ({packet.getRxPacketError(err)})")
                continue

            # 2. Ghi Baud_Rate vào Address 8
            res, err = packet.write1ByteTxRx(port, dxl_id, ADDR_BAUD_RATE, target_val)
            if res != COMM_SUCCESS:
                print(f"    - ID {dxl_id}: [FAIL] không ghi được Baud_Rate ({packet.getRxPacketError(err)})")
            else:
                print(f"    - ID {dxl_id}: [OK] Đã ghi Baud_Rate = {target_val} ({target_baud:,} bps)")
                success_ids.append(dxl_id)
            time.sleep(0.02)

    # 2. Đổi baudrate cổng sang target_baud và ping kiểm tra lại
    print(f"\n[*] Đổi baudrate cổng host sang {target_baud:,} bps và xác nhận ping...")
    port.setBaudRate(target_baud)
    time.sleep(0.05)

    verified_ids = []
    for dxl_id in motors.keys():
        model_num, res, _ = packet.ping(port, dxl_id)
        if res == COMM_SUCCESS:
            verified_ids.append(dxl_id)
            # Nháy LED để báo hiệu trực quan
            packet.write1ByteTxRx(port, dxl_id, ADDR_LED, 1)
            time.sleep(0.05)
            packet.write1ByteTxRx(port, dxl_id, ADDR_LED, 0)
            print(f"    - ID {dxl_id}: [OK] Ping thành công tại {target_baud:,} bps! (Model: {get_model_name(model_num)})")
        else:
            print(f"    - ID {dxl_id}: [FAIL] Không ping được tại {target_baud:,} bps!")

    if len(verified_ids) == len(motors):
        print(f"\n[HOÀN TẤT] Tất cả {len(verified_ids)} motor đã chuyển sang {target_baud:,} bps thành công rực rỡ!")
        return True
    else:
        print(f"\n[CẢNH BÁO] Chỉ có {len(verified_ids)}/{len(motors)} motor phản hồi. Hãy chạy lại --scan để kiểm tra.")
        return False


def factory_reset(port: PortHandler, packet: PacketHandler, dxl_id: int):
    print(f"\n[*] CẢNH BÁO: Thực hiện factory reset cho ID {dxl_id}...")
    print("    Thao tác này sẽ đưa motor về ID 1 và Baudrate 57,600 bps!")
    ans = input("    Bạn có chắc chắn muốn tiếp tục? (y/N): ").strip().lower()
    if ans != "y":
        print("    Đã hủy thao tác.")
        return

    # Quét xem motor đang ở baudrate nào
    cur_baud = None
    for baud in COMMON_BAUDRATES:
        port.setBaudRate(baud)
        time.sleep(0.02)
        _, res, _ = packet.ping(port, dxl_id)
        if res == COMM_SUCCESS:
            cur_baud = baud
            break

    if cur_baud is None:
        print(f"[FAIL] Không ping thấy ID {dxl_id} ở bất kỳ baudrate nào!")
        return

    print(f"[+] Tìm thấy ID {dxl_id} tại {cur_baud:,} bps.")
    port.setBaudRate(cur_baud)
    time.sleep(0.02)

    # 0xFF: reset all values (including ID and Baudrate)
    # 0x01: reset all values except ID
    # 0x02: reset all values except ID and Baudrate
    res, err = packet.factoryReset(port, dxl_id, 0x01)
    if res == COMM_SUCCESS:
        print(f"[OK] Factory reset thành công! (Giữ ID {dxl_id}, baudrate đã về 57,600 bps).")
    else:
        print(f"[FAIL] Factory reset thất bại: {packet.getRxPacketError(err)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyDXL", help="Cổng serial Dynamixel (mặc định /dev/ttyDXL)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate cho thao tác --ping (mặc định 1000000)")
    parser.add_argument("--scan", action="store_true", help="Quét tìm tất cả motor ở các baudrate")
    parser.add_argument("--set-baud", type=int, metavar="RATE", help="Đổi baudrate của tất cả motor sang RATE (ví dụ: 4000000)")
    parser.add_argument("--restore", action="store_true", help="Khôi phục tất cả motor về 1,000,000 bps (1 Mbps)")
    parser.add_argument("--ping", action="store_true", help="Ping tất cả motor tại baudrate chỉ định (--baud)")
    parser.add_argument("--factory-reset", action="store_true", help="Factory reset motor")
    parser.add_argument("--id", type=int, help="Motor ID cho --factory-reset")
    args = parser.parse_args()

    # Kiểm tra thiết bị
    dev = args.port
    if not os.path.exists(dev):
        # Thử tìm các /dev/ttyUSB*
        usb_ports = glob.glob("/dev/ttyUSB*")
        if usb_ports:
            print(f"[!] Không tìm thấy {dev}, tự động dùng {usb_ports[0]}")
            dev = usb_ports[0]
        else:
            print(f"[FAIL] Thiết bị {dev} không tồn tại và không tìm thấy /dev/ttyUSB* nào!")
            print("       Hãy đảm bảo robot đã cắm cáp USB U2D2 vào máy tính.")
            return 1

    # Kiểm tra cổng có bị xs_sdk chiếm giữ không
    if check_port_in_use(dev):
        return 1

    port = PortHandler(dev)
    packet = PacketHandler(2.0)

    if not port.openPort():
        print(f"[FAIL] Không thể mở cổng serial {dev}!")
        print("       Kiểm tra quyền truy cập: sudo usermod -aG dialout $USER")
        return 1

    try:
        if args.restore:
            set_baudrate(port, packet, 1000000)
        elif args.set_baud is not None:
            set_baudrate(port, packet, args.set_baud)
        elif args.factory_reset:
            if args.id is None:
                print("[FAIL] Cần chỉ định --id <N> khi dùng --factory-reset")
                return 1
            factory_reset(port, packet, args.id)
        elif args.ping:
            port.setBaudRate(args.baud)
            print(f"\n[*] Ping các motor tại {args.baud:,} bps trên {dev}...")
            found = 0
            for dxl_id in range(1, 8):
                model_num, res, _ = packet.ping(port, dxl_id)
                if res == COMM_SUCCESS:
                    print(f"    - ID {dxl_id}: [OK] {get_model_name(model_num)} (Model {model_num})")
                    found += 1
                else:
                    print(f"    - ID {dxl_id}: (không phản hồi)")
            print(f"[*] Kết quả: tìm thấy {found} motor.")
        else:
            # Mặc định là scan
            motors = scan_bus(port, packet)
            print_motors_table(motors)
    finally:
        port.closePort()

    return 0


if __name__ == "__main__":
    sys.exit(main())

