#!/usr/bin/env python3
"""
rx150_tuning_session — kịch bản kích thích LẶP LẠI ĐƯỢC cho HAC (hoặc
fuzzy/ff) để tune gain theo kiểu PID: mỗi lần đổi ĐÚNG MỘT gain thì chạy 1
session có --label, so sánh bằng rx150_run_compare.py.

3 mode:
  hold — giữ nguyên base-pose, dùng để xem trôi (steady-state droop).
  step — cộng --delta (rad) vào 1 khớp (hoặc 'all') tại thời điểm marker.
  sine — dao động 1 khớp quanh base-pose: amp*sin(2*pi*freq*t).

Output: ~/interbotix_ws/tuning_runs/<label>_<ts>/
  data.csv            — 46 cột (timestamp + 5 khớp x 9 field), mở được bằng
                         data_analysis/plot_control_csv.py
  gains_snapshot.yaml — gain ĐÃ ÁP DỤNG đọc lại từ node qua get_parameters
                         (không phải giá trị định gửi — chắc chắn khớp thực tế)
  metrics.json        — rise time, overshoot%, settling 2%, RMSE, ss_err, RMS PWM
                         cho từng khớp
  meta.json           — mode, joint, delta/amp/freq, marker (giây từ đầu CSV)
  plots/*.png         — position/error/pwm/velocity, 5 khớp

Ví dụ:
  ros2 run rx150_motion_common rx150_tuning_session.py \\
      --label smoke --target hac --mode step --joint shoulder --delta 0.1

An toàn: script CHỈ publish setpoint qua /rx150/{target}/setpoint — hac_node
tự lo TOTG (Ruckig) + saturation. Ctrl-C giữa chừng vẫn an toàn (hac_node giữ
nguyên setpoint cuối cùng khi mất publisher).
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tuning_lib as tl

import rclpy
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float64MultiArray

DEFAULT_BASE_POSE = [0.0, -1.80, 1.55, 0.8, 0.0]

# Tên tham số cần đọc lại cho gains_snapshot.yaml, theo target. Các tên
# friction_* chỉ tồn tại trên hac_node sau khi tích hợp Giai đoạn B (bước 10);
# nếu node chưa có, get_parameters trả PARAMETER_NOT_SET — được ghi là null.
SNAPSHOT_PARAMS = {
    "hac": ["a", "b", "c", "error_limit", "error_dot_limit", "u_max", "Gff",
            "gravity_sign", "friction_coulomb", "friction_viscous", "friction_eps"],
    "fuzzy": ["Ke", "Ked", "Ku", "u_max", "Gff", "gravity_sign"],
    "ff": ["Ke", "Ked", "Ku", "u_max", "Kv", "Ka"],
}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--label", required=True, help="Tên gợi nhớ cho session (vd: smoke, a_0.25)")
    p.add_argument("--target", default="hac", choices=["hac", "fuzzy", "ff"])
    p.add_argument("--mode", default="hold", choices=["hold", "step", "sine"])
    p.add_argument("--joint", default=None, choices=tl.ARM_JOINTS + ["all"],
                   help="Khớp bị kích thích (bắt buộc cho step/sine)")
    p.add_argument("--base-pose", type=float, nargs=5, default=DEFAULT_BASE_POSE,
                   metavar=("WAIST", "SHOULDER", "ELBOW", "WRIST_ANGLE", "WRIST_ROTATE"))
    p.add_argument("--delta", type=float, default=0.1, help="Step (rad), mode=step")
    p.add_argument("--amp", type=float, default=0.1, help="Biên độ sine (rad), mode=sine")
    p.add_argument("--freq", type=float, default=0.2, help="Tần số sine (Hz), mode=sine")
    p.add_argument("--pre-hold", type=float, default=2.0, help="Giây giữ base-pose trước khi kích thích")
    p.add_argument("--duration", type=float, default=5.0, help="Giây ghi dữ liệu sau marker")
    p.add_argument("--rate", type=float, default=50.0, help="Tần số publish setpoint (Hz)")
    p.add_argument("--out-dir", default=os.path.expanduser("~/interbotix_ws/tuning_runs"))
    args = p.parse_args()
    if args.mode in ("step", "sine") and args.joint is None:
        p.error(f"--joint bắt buộc cho mode={args.mode}")
    return args


def pose_at(base_pose, args, t):
    pose = list(base_pose)
    if args.mode == "hold" or args.joint is None:
        return pose
    idxs = range(len(pose)) if args.joint == "all" else [tl.ARM_JOINTS.index(args.joint)]
    for i in idxs:
        if args.mode == "step":
            pose[i] = base_pose[i] + args.delta
        elif args.mode == "sine":
            pose[i] = base_pose[i] + args.amp * math.sin(2.0 * math.pi * args.freq * t)
    return pose


def spin_for(node, seconds, period, on_tick=None):
    """Spin liên tục (đáp ứng nhanh callback) nhưng chỉ gọi on_tick() theo
    đúng nhịp `period` (1/--rate) — tách rời tần số publish khỏi tần số poll.
    """
    end = time.monotonic() + seconds
    next_tick = time.monotonic()
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.005)
        now = time.monotonic()
        if on_tick is not None and now >= next_tick:
            on_tick()
            next_tick += period


def read_gains_snapshot(node, target_node, names):
    cli = node.create_client(GetParameters, f"{target_node}/get_parameters")
    if not cli.wait_for_service(timeout_sec=5.0):
        return {"error": f"{target_node}/get_parameters không sẵn sàng"}
    req = GetParameters.Request()
    req.names = names
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if not fut.done() or fut.result() is None:
        return {"error": "get_parameters timeout"}
    resp = fut.result()
    out = {}
    from rcl_interfaces.msg import ParameterType
    for nm, pv in zip(names, resp.values):
        if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            out[nm] = list(pv.double_array_value)
        elif pv.type == ParameterType.PARAMETER_DOUBLE:
            out[nm] = pv.double_value
        else:
            out[nm] = None  # PARAMETER_NOT_SET hoặc kiểu khác
    return out


def col(rows, header, name):
    idx = header.index(name)
    return [r[idx] for r in rows]


def compute_metrics(rows, header, args):
    metrics = {}
    for j in tl.ARM_JOINTS:
        pos = col(rows, header, f"{j}_pos")
        err = col(rows, header, f"{j}_err")
        pwm = col(rows, header, f"{j}_pwm")
        entry = {
            "rmse_err": tl.rmse(err),
            "ss_err": tl.steady_state_error(pos, pos[-1]) if pos else float("nan"),
            "rms_pwm": tl.rms(pwm),
        }
        if args.mode == "step" and (args.joint == j or args.joint == "all"):
            t = col(rows, header, "timestamp")
            pos0 = args.base_pose[tl.ARM_JOINTS.index(j)]
            ref_final = pos0 + args.delta
            entry.update(tl.step_metrics(t, pos, ref_final, pos0=pos0))
        metrics[j] = entry
    return metrics


def save_plots(rows, header, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(out_dir, exist_ok=True)
    t = col(rows, header, "timestamp")

    def make(suffix_pairs, title, fname):
        fig, axes = plt.subplots(len(tl.ARM_JOINTS), 1, figsize=(10, 2.0 * len(tl.ARM_JOINTS)), sharex=True)
        fig.suptitle(title)
        for i, j in enumerate(tl.ARM_JOINTS):
            ax = axes[i]
            for suffix, label, style in suffix_pairs:
                colname = f"{j}_{suffix}"
                if colname in header:
                    ax.plot(t, col(rows, header, colname), style, linewidth=0.8, label=label)
            ax.set_ylabel(j, fontsize=8)
            ax.axhline(0.0, color="gray", linewidth=0.4, linestyle=":")
            if i == 0:
                ax.legend(fontsize=7, loc="upper right")
        axes[-1].set_xlabel("t (s)")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, fname), dpi=140)
        plt.close(fig)

    make([("pos", "q", "-"), ("ref_pos", "q_ref", "--")], "Position tracking", "position.png")
    make([("err", "error", "-")], "Tracking error", "error.png")
    make([("pwm", "total", "-"), ("grav", "gravity", "-"), ("fric", "friction", "-")], "PWM", "pwm.png")
    make([("vel", "vel", "-"), ("edot", "edot", "-")], "Velocity", "velocity.png")


def main():
    args = parse_args()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(args.out_dir, f"{args.label}_{ts}")
    os.makedirs(run_dir, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("rx150_tuning_session")
    setpoint_topic = f"/rx150/{args.target}/setpoint"
    target_node = f"/rx150/{args.target}_node"
    pub = node.create_publisher(Float64MultiArray, setpoint_topic, 10)
    recorder = tl.TelemetryRecorder(node, f"/rx150/{args.target}")

    period = 1.0 / args.rate

    def publish_pose(pose):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in pose]
        pub.publish(msg)

    node.get_logger().info(f"[{args.label}] pre-hold {args.pre_hold}s tại base-pose {args.base_pose}")

    def tick_pre():
        publish_pose(args.base_pose)

    spin_for(node, args.pre_hold, period, on_tick=tick_pre)

    node.get_logger().info(f"[{args.label}] bắt đầu ghi — mode={args.mode} joint={args.joint}")
    recorder.start()
    marker_elapsed = 0.0  # recorder vừa start() -> t0 reset, marker ngay tại 0

    t_excite0 = time.monotonic()

    def tick_excite():
        t = time.monotonic() - t_excite0
        publish_pose(pose_at(args.base_pose, args, t))

    spin_for(node, args.duration, period, on_tick=tick_excite)
    recorder.stop()

    rows = recorder.rows
    header = tl.csv_header()
    if not rows:
        node.get_logger().error("Không ghi được dữ liệu nào (joint_states không đến?) — hủy session.")
        rclpy.shutdown()
        sys.exit(1)

    recorder.write_csv(os.path.join(run_dir, "data.csv"))

    gains = read_gains_snapshot(node, target_node, SNAPSHOT_PARAMS[args.target])
    import yaml
    with open(os.path.join(run_dir, "gains_snapshot.yaml"), "w") as fh:
        yaml.safe_dump(gains, fh, default_flow_style=None, sort_keys=False)

    metrics = compute_metrics(rows, header, args)
    with open(os.path.join(run_dir, "metrics.json"), "w") as fh:
        json.dump(metrics, fh, indent=2)

    meta = {
        "label": args.label,
        "target": args.target,
        "mode": args.mode,
        "joint": args.joint,
        "delta": args.delta if args.mode == "step" else None,
        "amp": args.amp if args.mode == "sine" else None,
        "freq": args.freq if args.mode == "sine" else None,
        "base_pose": args.base_pose,
        "pre_hold_s": args.pre_hold,
        "duration_s": args.duration,
        "rate_hz": args.rate,
        "marker_elapsed_s": marker_elapsed,
        "timestamp": ts,
        "n_rows": len(rows),
    }
    with open(os.path.join(run_dir, "meta.json"), "w") as fh:
        json.dump(meta, fh, indent=2)

    save_plots(rows, header, os.path.join(run_dir, "plots"))

    node.get_logger().info(f"[{args.label}] xong — {len(rows)} dòng -> {run_dir}")
    print(run_dir)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
