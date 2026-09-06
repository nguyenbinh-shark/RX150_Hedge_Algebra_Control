#!/usr/bin/env python3
"""
rx150_friction_id — hiệu chuẩn friction feedforward (Coulomb + viscous) cho
1 khớp của hac_node (RX150) — B2 trong kế hoạch hiệu chuẩn.

Cách làm: các khớp khác đứng yên ở --rest-pose; khớp đang test được cho
"cruise" ở vận tốc không đổi (mặc định 0.1/0.2/0.4 rad/s, cả 2 hướng) bằng
cách hạ tạm max_velocities[joint] (live param, xem hac_node onParamChange)
rồi gửi setpoint xa để Ruckig sinh ra 1 đoạn vận tốc không đổi đủ dài
(trapezoid: accel ngắn -> cruise -> vẫn đang cruise khi script dừng thu mẫu
và quay về rest-pose). Trong cửa sổ cruise (lọc theo |v_đo - v_target| < tol):

    u − u_grav(q) = f_c · sign(qd) + f_v · qd

với u = hac/effort (PWM tổng), u_grav = hac/gravity (PWM đã scale) — CẢ HAI
đã publish sẵn, không cần đọc/ghi gì thêm ở hac_node. LSQ (numpy lstsq) ra
Coulomb (f_c) + viscous (f_v) + R².

max_velocities/max_accelerations GỐC được khôi phục khi xong (kể cả khi lỗi
giữa chừng, nhờ try/finally).

CHẠY (hac_node đang chạy, torque ON; khuyến nghị gains_file:=
rx150_hac_gains_safe.yaml để u_max/vận tốc thấp hơn trong lúc ID):
  ros2 run rx150_motion_common rx150_friction_id.py --joint shoulder

Kết quả KHÔNG tự ghi vào yaml (khác B0 signcheck) — 1 lần chạy chỉ fit 1
khớp, tự ghi đè cả mảng 5 phần tử dễ mất giá trị khớp khác đã hiệu chuẩn.
Sửa tay `friction_coulomb[i]`/`friction_viscous[i]` trong rx150_hac_gains.yaml
hoặc qua tab Gains của rx150_tuning_gui.py (Lưu vào yaml).

AN TOÀN: kiểm tra rest-pose không kẹp gì trước khi chạy. Ctrl-C giữa chừng an
toàn (hac_node giữ nguyên setpoint cuối). Nếu tay rung/kêu bất thường ở tốc độ
cao nhất, giảm --speeds hoặc u_max trong gains_safe trước khi thử lại.
"""

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tuning_lib as tl

import rclpy
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import Float64MultiArray

HAC_NODE = "/rx150/hac_node"
HAC_SETPOINT_TOPIC = "/rx150/hac/setpoint"
def _motor_yaml():
    """config/rx150_motor.yaml: ưu tiên bản đã cài, fallback cây nguồn.

    Không hardcode ~/interbotix_ws/src/... nữa — đường dẫn đó chết mỗi lần
    thư mục package đổi chỗ (và sai hẳn với ai clone ra chỗ khác).
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("rx150_motion_common"),
                         "config", "rx150_motor.yaml")
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "config", "rx150_motor.yaml")


MOTOR_YAML = _motor_yaml()
DEFAULT_REST_POSE = [0.0, -1.80, 1.55, 0.8, 0.0]
DEFAULT_SPEEDS = [0.1, 0.2, 0.4]
CRUISE_TOL_FRAC = 0.2       # |v_đo - v_target| < max(0.02, tol_frac * v_target)
FAR_MOVE_MARGIN = 1.0       # rad, khoảng cách "đích xa" (kẹp trong joint limits)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--joint", required=True, choices=tl.ARM_JOINTS)
    p.add_argument("--speeds", type=float, nargs="+", default=DEFAULT_SPEEDS)
    p.add_argument("--rest-pose", type=float, nargs=5, default=DEFAULT_REST_POSE,
                    metavar=("WAIST", "SHOULDER", "ELBOW", "WRIST_ANGLE", "WRIST_ROTATE"))
    p.add_argument("--cruise-duration", type=float, default=3.0, help="Giây thu mẫu / (tốc độ, hướng)")
    p.add_argument("--settle-duration", type=float, default=1.5, help="Giây chờ giữa các lượt")
    p.add_argument("--out-dir", default=os.path.expanduser("~/interbotix_ws/tuning_runs"))
    return p.parse_args()


def get_param(node, name, timeout=5.0):
    cli = node.create_client(GetParameters, f"{HAC_NODE}/get_parameters")
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetParameters.Request()
    req.names = [name]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return None
    pv = fut.result().values[0]
    if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(pv.double_array_value)
    if pv.type == ParameterType.PARAMETER_DOUBLE:
        return pv.double_value
    return None


def set_param_array(node, name, values, timeout=5.0):
    cli = node.create_client(SetParameters, f"{HAC_NODE}/set_parameters")
    if not cli.wait_for_service(timeout_sec=timeout):
        return False
    req = SetParameters.Request()
    p = ParamMsg()
    p.name = name
    p.value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
    p.value.double_array_value = [float(v) for v in values]
    req.parameters = [p]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return False
    return bool(fut.result().results[0].successful)


def main():
    args = parse_args()
    joint_idx = tl.ARM_JOINTS.index(args.joint)
    limits = tl.load_joint_limits_rad(MOTOR_YAML, margin=0.15)
    lo, hi = limits[args.joint]

    rclpy.init()
    node = rclpy.create_node("rx150_friction_id")
    pub = node.create_publisher(Float64MultiArray, HAC_SETPOINT_TOPIC, 10)
    recorder = tl.TelemetryRecorder(node, "/rx150/hac")

    def publish_pose(pose):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in pose]
        pub.publish(msg)

    def spin(seconds, hold_pose=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if hold_pose is not None:
                publish_pose(hold_pose)
            rclpy.spin_once(node, timeout_sec=0.02)

    rest = list(args.rest_pose)

    orig_vmax = get_param(node, "max_velocities")
    orig_amax = get_param(node, "max_accelerations")
    if orig_vmax is None or orig_amax is None:
        print("Không đọc được max_velocities/max_accelerations — hac_node có đang chạy không?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print(f"max_velocities/max_accelerations gốc: {orig_vmax} / {orig_amax} — sẽ khôi phục khi xong.")

    print(f"Về rest-pose {rest} ...")
    spin(3.0, hold_pose=rest)

    samples_v = []
    samples_u = []

    try:
        for speed in args.speeds:
            vmax = list(orig_vmax)
            vmax[joint_idx] = speed
            if not set_param_array(node, "max_velocities", vmax):
                print(f"  LỖI set max_velocities cho speed={speed} rad/s — bỏ qua tốc độ này")
                continue

            tol = max(0.02, CRUISE_TOL_FRAC * speed)
            for direction in (+1.0, -1.0):
                target = min(max(rest[joint_idx] + direction * FAR_MOVE_MARGIN, lo), hi)
                pose = list(rest)
                pose[joint_idx] = target
                print(f"[{args.joint}] speed={speed:.2f} dir={direction:+.0f} -> target {target:+.3f} rad")

                n_before = len(samples_v)
                t_end = time.monotonic() + args.cruise_duration
                while time.monotonic() < t_end:
                    publish_pose(pose)
                    rclpy.spin_once(node, timeout_sec=0.02)
                    vel = recorder.latest_velocities()[joint_idx]
                    if vel == vel and abs(abs(vel) - speed) < tol and vel * direction > 0:
                        pwm = recorder.latest_effort_pwm()[joint_idx]
                        grav = recorder.latest_gravity_pwm()[joint_idx]
                        if pwm == pwm and grav == grav:
                            samples_v.append(vel)
                            samples_u.append(pwm - grav)
                print(f"  -> {len(samples_v) - n_before} mẫu trong cửa sổ cruise (tol={tol:.3f} rad/s)")

                spin(args.settle_duration, hold_pose=rest)
    finally:
        print("Khôi phục max_velocities/max_accelerations gốc ...")
        set_param_array(node, "max_velocities", orig_vmax)
        set_param_array(node, "max_accelerations", orig_amax)
        spin(0.5, hold_pose=rest)

    v = np.array(samples_v, dtype=float)
    u = np.array(samples_u, dtype=float)
    n = len(v)
    print(f"\nTổng số mẫu cruise hợp lệ: {n}")
    if n < 10:
        print("CẢNH BÁO: quá ít mẫu — kết quả không đáng tin. Tăng --cruise-duration hoặc "
              "kiểm tra hac_node có đang publish hac/effort, hac/gravity, joint_states không.")
        node.destroy_node()
        rclpy.shutdown()
        return

    X = np.stack([np.sign(v), v], axis=1)
    beta, _resid, _rank, _sv = np.linalg.lstsq(X, u, rcond=None)
    f_c, f_v = float(beta[0]), float(beta[1])
    pred = X @ beta
    ss_res = float(np.sum((u - pred) ** 2))
    ss_tot = float(np.sum((u - np.mean(u)) ** 2))
    r2 = (1.0 - ss_res / ss_tot) if ss_tot > 1e-9 else float("nan")

    print(f"\n=== Kết quả friction fit — {args.joint} ===")
    print(f"  f_c (Coulomb) = {f_c:+.3f} PWM")
    print(f"  f_v (viscous) = {f_v:+.3f} PWM/(rad/s)")
    print(f"  R² = {r2:.3f}  (n={n} mẫu, {len(args.speeds)} tốc độ x 2 hướng)")
    if f_c < 0:
        print("  CẢNH BÁO: f_c âm — bất thường cho ma sát Coulomb (luôn cản chuyển động, "
              "cùng dấu sign(qd)). Kiểm tra lại joint/hướng trước khi dùng kết quả này.")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(args.out_dir, f"friction_identify_{args.joint}_{ts}")
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, "friction_fit.json"), "w") as fh:
        json.dump({
            "joint": args.joint, "joint_index": joint_idx, "speeds": args.speeds,
            "n_samples": n, "f_c": f_c, "f_v": f_v, "r2": r2, "timestamp": ts,
        }, fh, indent=2)
    with open(os.path.join(run_dir, "samples.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["velocity_rad_s", "u_minus_grav_pwm"])
        for vv, uu in zip(v, u):
            w.writerow([f"{vv:.6f}", f"{uu:.6f}"])
    print(f"\nĐã ghi: {run_dir}")
    print(f"Áp dụng: sửa friction_coulomb[{joint_idx}]={f_c:.3f} và "
          f"friction_viscous[{joint_idx}]={f_v:.3f} trong rx150_hac_gains.yaml — tay, hoặc "
          f"qua tab Gains của rx150_tuning_gui.py --ros-args -p target:=hac (rồi 'Lưu vào yaml').")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
