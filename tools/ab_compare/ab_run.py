#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ab_run.py — chạy MỘT task cố định trên bộ điều khiển đang sống (hac | fuzzy)
và ghi dữ liệu theo đúng một khuôn, để ab_report.py so sánh hai bộ.

Cả hai node nhận setpoint như nhau (/rx150/<ctrl>/setpoint, Float64MultiArray)
và tự sinh quỹ đạo bằng Ruckig với max_velocities/max_accelerations của chính
nó. Giới hạn profile giống nhau thì quỹ đạo tham chiếu giống nhau — ab_report.py
kiểm lại điều đó từ params.yaml chụp ở đây, không tin vào file YAML trong src/.

Ghi ra:
    tuning_runs/ab_<session>/<ctrl>/params.yaml         tham số ĐANG CHẠY của node
    tuning_runs/ab_<session>/<ctrl>/trial_NN/data.csv   1 dòng / tick debug (50 Hz)
    tuning_runs/ab_<session>/<ctrl>/trial_NN/meta.json  task, segment, thống kê ghi

Vì sao 1 dòng / tick debug mà không phải / joint_states (400 Hz): node chỉ phát
reference/error/effort ở debug_publish_rate (50 Hz). Ghép ref 50 Hz với pos
400 Hz tạo sai số GIẢ tới v·20 ms (≈ 3° ở 2.6 rad/s). Dòng được ghi khi nhận
'<ctrl>/gravity' — topic chung cuối cùng node phát trong một tick — nên ref,
err, edot, effort trên cùng dòng thuộc CÙNG một chu kỳ điều khiển. pos/vel là
joint_states mới nhất (cũ ≤ 2.5 ms).

Dùng:
    ./rx150.sh ab-hac                          # terminal 1 (hoặc ab-fuzzy)
    ./rx150.sh ab-run --ctrl hac --task pick_cycle --session s1 --trials 5
    ./rx150.sh ab-run --task pick_cycle --check          # chỉ kiểm task, không cần ROS
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src/rx150/rx150_toolbox/rx150_motion_common/scripts"))
import tuning_lib as tl  # noqa: E402

ARM = tl.ARM_JOINTS
TASK_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tasks")
MOTOR_YAML = os.path.join(REPO, "src/rx150/rx150_toolbox/rx150_motion_common/config/rx150_motor.yaml")

# Tham số chụp lại để kiểm công bằng. Tên không có trên node -> null.
PARAMS_COMMON = ["loop_rate", "debug_publish_rate", "watchdog_timeout", "velocity_filter_tau",
                 "enable_profile", "max_velocities", "max_accelerations", "max_jerk", "sync_mode",
                 "u_max", "enable_gravity_comp", "Gff", "gravity_sign"]
PARAMS_CTRL = {
    "hac": ["a", "b", "c", "joint_gain_scale", "error_limit", "error_dot_limit", "gravity_model_source",
            "friction_coulomb", "friction_viscous", "friction_eps"],
    "fuzzy": ["Ke", "Ked", "Ku"],
}

# <ctrl>/timing do node phát ở mỗi tick debug, gộp cả cửa sổ (8 chu kỳ @400/50 Hz).
# Binary cũ chưa có topic này -> các cột NaN, report tự bỏ phần CPU.
TIMING_COLS = ["law_mean_ns", "law_max_ns", "cyc_mean_ns", "cyc_max_ns"]
CSV_COLS = (["t", "seg"]
            + [f"{j}_{s}" for s in ("pos", "vel", "ref_pos", "ref_vel", "err", "edot", "pwm", "grav")
               for j in ARM]
            + TIMING_COLS)


def find_pid(exe):
    """pid của tiến trình có argv[0] tên 'exe' (ros2 launch chạy .../lib/<pkg>/<exe>)."""
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        try:
            with open(f"/proc/{d}/cmdline", "rb") as fh:
                argv0 = fh.read().split(b"\0", 1)[0].decode(errors="ignore")
        except OSError:
            continue
        if os.path.basename(argv0) == exe:
            return int(d)
    return None


def cpu_ticks(pid):
    """utime + stime (clock ticks) của pid. Tách sau ')' vì tên tiến trình có thể chứa khoảng trắng."""
    try:
        with open(f"/proc/{pid}/stat") as fh:
            f = fh.read().rsplit(")", 1)[1].split()
        return int(f[11]) + int(f[12])
    except (OSError, ValueError, IndexError):
        return None


# ─────────────────────────── task ────────────────────────────

def load_task(name_or_path):
    path = name_or_path
    if not os.path.isfile(path):
        path = os.path.join(TASK_DIR, name_or_path + ("" if name_or_path.endswith(".yaml") else ".yaml"))
    if not os.path.isfile(path):
        sys.exit(f"không thấy task '{name_or_path}' (có: "
                 f"{', '.join(sorted(f[:-5] for f in os.listdir(TASK_DIR) if f.endswith('.yaml')))})")
    with open(path) as fh:
        task = yaml.safe_load(fh)
    task["path"] = os.path.abspath(path)
    task.setdefault("name", os.path.basename(path)[:-5])
    task.setdefault("start_hold", 2.0)
    return task


def ruckig_time(dq, vmax, amax):
    """Thời gian profile tam giác/thang cho 1 khớp (jerk vô hạn) — đủ để cảnh báo hold thiếu."""
    dq = abs(dq)
    if dq < 1e-9:
        return 0.0
    if dq <= vmax * vmax / amax:
        return 2.0 * math.sqrt(dq / amax)
    return dq / vmax + vmax / amax


def check_task(task, vmax=3.14, amax=5.0, z_min=0.05, margin=0.05):
    """Trả danh sách vấn đề. Rỗng = chạy được."""
    problems = []
    lim = tl.load_joint_limits_rad(MOTOR_YAML, margin=margin)
    poses = [("start", task["start"], None)] + [(w["label"], w["q"], w["hold"]) for w in task["waypoints"]]
    prev = None
    print(f"\nTask '{task['name']}'  ({task['path']})")
    print(f"  {'segment':<16}{'z EE [m]':>9}{'t_move ~[s]':>13}{'hold [s]':>10}")
    for label, q, hold in poses:
        if len(q) != 5:
            problems.append(f"{label}: cần 5 góc, có {len(q)}")
            continue
        for j, v in zip(ARM, q):
            lo, hi = lim[j]
            if not lo <= v <= hi:
                problems.append(f"{label}: {j}={v:.3f} ngoài [{lo:.3f}, {hi:.3f}] (đã trừ lề {margin})")
        z = tl.ee_height(q)
        if z < z_min and label != "start":
            problems.append(f"{label}: EE z={z:.3f} m < {z_min} m")
        t_move = max(ruckig_time(b - a, vmax, amax) for a, b in zip(prev, q)) if prev else 0.0
        move = next((w.get("move") for w in task["waypoints"] if w["label"] == label), None)
        if move and prev:
            # quintic: v_max = 1.875·Δq/T, a_max = 5.77·Δq/T² — phải nằm trong giới hạn Ruckig
            dmax = max(abs(b - a) for a, b in zip(prev, q))
            if 1.875 * dmax / move > vmax or 5.774 * dmax / move ** 2 > amax:
                problems.append(f"{label}: move {move:.1f}s vượt giới hạn Ruckig — ref sẽ không bám nội suy")
            t_move = move
        if hold is not None and hold < t_move + 1.0:
            problems.append(f"{label}: hold {hold:.1f}s < t_move {t_move:.2f}s + 1s — không đủ để đo xác lập")
        print(f"  {label:<16}{z:9.3f}{t_move:13.2f}{(hold if hold else task['start_hold']):10.1f}")
        prev = q
    total = task["start_hold"] + sum(w["hold"] for w in task["waypoints"])
    print(f"  tổng mỗi lượt ≈ {total:.0f} s (chưa tính đoạn về start)")
    return problems


# ─────────────────────────── ROS ────────────────────────────

def run(args, task):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    ns, ctrl = "/rx150", args.ctrl
    other = "fuzzy" if ctrl == "hac" else "hac"

    class Runner(Node):
        def __init__(self):
            super().__init__("ab_run")
            self.pub = self.create_publisher(Float64MultiArray, f"{ns}/{ctrl}/setpoint", 10)
            self.latest = {}
            self.rows = []
            self.recording = False
            self.seg = -1
            self.t0 = 0.0
            self.create_subscription(JointState, f"{ns}/joint_states",
                                     lambda m: self.latest.__setitem__("js", m), qos_profile_sensor_data)
            for k in ("reference", "error", "edot", "effort"):
                self.create_subscription(JointState, f"{ns}/{ctrl}/{k}",
                                         lambda m, k=k: self.latest.__setitem__(k, m), qos_profile_sensor_data)
            self.create_subscription(Float64MultiArray, f"{ns}/{ctrl}/timing",
                                     lambda m: self.latest.__setitem__("timing", list(m.data)),
                                     qos_profile_sensor_data)
            self.create_subscription(JointState, f"{ns}/{ctrl}/gravity", self.on_tick, qos_profile_sensor_data)

        @staticmethod
        def pick(msg, field):
            if msg is None:
                return [math.nan] * 5
            idx = {n: i for i, n in enumerate(msg.name)}
            arr = getattr(msg, field)
            return [arr[idx[j]] if j in idx and idx[j] < len(arr) else math.nan for j in ARM]

        def on_tick(self, grav):
            self.latest["gravity"] = grav
            if not self.recording:
                return
            L = self.latest
            self.rows.append([time.monotonic() - self.t0, self.seg]
                             + self.pick(L.get("js"), "position") + self.pick(L.get("js"), "velocity")
                             + self.pick(L.get("reference"), "position") + self.pick(L.get("reference"), "velocity")
                             + self.pick(L.get("error"), "position") + self.pick(L.get("edot"), "velocity")
                             + self.pick(L.get("effort"), "effort") + self.pick(grav, "effort")
                             + (L.get("timing", []) + [math.nan] * len(TIMING_COLS))[:len(TIMING_COLS)])

        def send(self, q):
            m = Float64MultiArray()
            m.data = [float(v) for v in q]
            self.pub.publish(m)

        def spin_publish(self, q, seconds, rate=50.0):
            end = time.monotonic() + seconds
            nxt = 0.0
            while time.monotonic() < end and rclpy.ok():
                if time.monotonic() >= nxt:
                    self.send(q)
                    nxt = time.monotonic() + 1.0 / rate
                rclpy.spin_once(self, timeout_sec=0.002)

        def stream_quintic(self, q0, q1, duration, rate=100.0):
            """Rải setpoint nội suy quintic (vận tốc & gia tốc = 0 ở hai đầu) — dùng cho
            waypoint có 'move'. Chậm hơn nhiều so với giới hạn Ruckig nên ref của node
            bám theo đường nội suy này, giống hệt nhau giữa hai bộ."""
            q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
            t_begin = time.monotonic()
            nxt = 0.0
            while rclpy.ok():
                tau = (time.monotonic() - t_begin) / duration
                if tau >= 1.0:
                    break
                if time.monotonic() >= nxt:
                    s = tau ** 3 * (10.0 - 15.0 * tau + 6.0 * tau ** 2)
                    self.send(q0 + (q1 - q0) * s)
                    nxt = time.monotonic() + 1.0 / rate
                rclpy.spin_once(self, timeout_sec=0.002)
            self.send(q1)

        def params(self, node_name, names):
            cli = self.create_client(GetParameters, f"{node_name}/get_parameters")
            if not cli.wait_for_service(timeout_sec=5.0):
                return None
            req = GetParameters.Request()
            req.names = names
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result() is None:
                return None
            out = {}
            for n, v in zip(names, fut.result().values):
                out[n] = {ParameterType.PARAMETER_DOUBLE: lambda: v.double_value,
                          ParameterType.PARAMETER_INTEGER: lambda: v.integer_value,
                          ParameterType.PARAMETER_BOOL: lambda: v.bool_value,
                          ParameterType.PARAMETER_STRING: lambda: v.string_value,
                          ParameterType.PARAMETER_DOUBLE_ARRAY: lambda: list(v.double_array_value),
                          }.get(v.type, lambda: None)()
            return out

    rclpy.init()
    node = Runner()
    try:
        # ── preflight: đúng MỘT bộ điều khiển đang sống ──
        time.sleep(1.0)
        names = {f"{n_ns.rstrip('/')}/{n}" for n, n_ns in node.get_node_names_and_namespaces()}
        if f"{ns}/{ctrl}_node" not in names:
            sys.exit(f"✗ không thấy {ns}/{ctrl}_node. Chạy './rx150.sh ab-{ctrl}' trước.")
        if f"{ns}/{other}_node" in names:
            sys.exit(f"✗ {ns}/{other}_node CŨNG đang chạy — hai bộ cùng ghi PWM. Tắt một bộ.")
        t_wait = time.monotonic() + 5.0
        while time.monotonic() < t_wait and not ("js" in node.latest and "gravity" in node.latest):
            rclpy.spin_once(node, timeout_sec=0.05)
        if "js" not in node.latest or "gravity" not in node.latest:
            sys.exit(f"✗ không nhận được joint_states hoặc {ctrl}/gravity sau 5 s.")
        rclpy.spin_once(node, timeout_sec=0.1)
        if "timing" not in node.latest:
            print(f"⚠ không có {ns}/{ctrl}/timing — binary cũ? Build lại để có số thời gian tính "
                  "(CPU% tiến trình vẫn được đo).")

        prm = node.params(f"{ns}/{ctrl}_node", PARAMS_COMMON + PARAMS_CTRL[ctrl])
        if prm is None:
            sys.exit("✗ không đọc được tham số node.")
        problems = check_task(task, vmax=min(prm["max_velocities"] or [3.14]),
                              amax=min(prm["max_accelerations"] or [5.0]))
        if problems:
            print("\nVẤN ĐỀ:\n  • " + "\n  • ".join(problems))
            if not args.force:
                sys.exit("Dừng trước khi cử động (--force để bỏ qua).")
        if not prm.get("enable_profile"):
            print("⚠ enable_profile=false: setpoint là BẬC, quỹ đạo sẽ khác hẳn bộ kia.")

        out = os.path.join(args.out_root, f"ab_{args.session}", ctrl)
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "params.yaml"), "w") as fh:
            yaml.safe_dump({"node": f"{ns}/{ctrl}_node", "captured": datetime.now().isoformat(timespec="seconds"),
                            "params": prm}, fh, sort_keys=False, default_flow_style=None, allow_unicode=True)
        existing = sorted(d for d in os.listdir(out) if d.startswith("trial_"))
        first = int(existing[-1][6:]) + 1 if existing else 1

        q_now = node.pick(node.latest["js"], "position")
        print(f"\n⚠ Tay máy sẽ CỬ ĐỘNG. {args.trials} lượt '{task['name']}' với {ctrl}. Ctrl+C để dừng.")
        print(f"  về start từ {np.round(q_now, 2).tolist()} …")
        node.spin_publish(task["start"], args.approach)

        for k in range(first, first + args.trials):
            tdir = os.path.join(out, f"trial_{k:02d}")
            node.spin_publish(task["start"], task["start_hold"])
            node.rows, node.seg = [], -1
            segs = []
            pid = find_pid(f"{ctrl}_node")
            c0 = cpu_ticks(pid) if pid else None
            node.t0 = time.monotonic()
            node.recording = True
            q_prev = task["start"]
            for i, w in enumerate(task["waypoints"]):
                node.seg = i
                segs.append({"idx": i, "label": w["label"], "q": w["q"], "hold": w["hold"],
                             "move": w.get("move"), "t_start": time.monotonic() - node.t0})
                if w.get("move"):
                    node.stream_quintic(q_prev, w["q"], w["move"])
                    node.spin_publish(w["q"], w["hold"] - w["move"])
                else:
                    node.spin_publish(w["q"], w["hold"])
                q_prev = w["q"]
                segs[-1]["t_end"] = time.monotonic() - node.t0
            node.recording = False
            c1 = cpu_ticks(pid) if pid else None

            rows = node.rows
            os.makedirs(tdir, exist_ok=True)
            with open(os.path.join(tdir, "data.csv"), "w", newline="") as fh:
                wr = csv.writer(fh)
                wr.writerow(CSV_COLS)
                for r in rows:
                    wr.writerow([f"{v:.6f}" if isinstance(v, float) else v for v in r])
            dur = segs[-1]["t_end"]
            meta = {"ctrl": ctrl, "trial": k, "session": args.session, "task": task, "segments": segs,
                    "start": task["start"], "n_rows": len(rows), "duration_s": dur,
                    "rate_hz": len(rows) / dur if dur > 0 else 0.0,
                    # % MỘT lõi, cả tiến trình node (luật + Ruckig + Pinocchio + DDS), trung bình cả lượt
                    "cpu_pct": (100.0 * (c1 - c0) / os.sysconf("SC_CLK_TCK") / dur
                                if c0 is not None and c1 is not None and dur > 0 else None),
                    "pid": pid,
                    "stamp": datetime.now().isoformat(timespec="seconds")}
            with open(os.path.join(tdir, "meta.json"), "w") as fh:
                json.dump(meta, fh, indent=2, ensure_ascii=False)
            err = np.array([r[2 + 4 * 5: 2 + 5 * 5] for r in rows], dtype=float)
            rmse = math.degrees(float(np.sqrt(np.nanmean(err ** 2)))) if len(rows) else math.nan
            cpu = f"{meta['cpu_pct']:.1f}% 1 lõi" if meta["cpu_pct"] is not None else "?"
            law = np.array([r[-len(TIMING_COLS)] for r in rows], dtype=float)
            law_s = f"{np.nanmean(law) / 1e3:.1f} µs" if len(rows) and np.isfinite(law).any() else "?"
            print(f"  lượt {k:02d}: {len(rows)} dòng ({meta['rate_hz']:.1f} Hz), RMSE e = {rmse:.3f}°, "
                  f"CPU {cpu}, khối luật {law_s} -> {tdir}")
            if meta["rate_hz"] < 0.8 * (prm.get("debug_publish_rate") or 50.0):
                print("  ⚠ tốc độ ghi thấp hơn debug_publish_rate đáng kể — máy bận hoặc rớt gói.")

        print("  về start …")
        node.spin_publish(task["start"], args.approach)
        print(f"\nXong. Tiếp: chạy bộ còn lại cùng --session {args.session}, rồi\n"
              f"  ./rx150.sh ab-report {os.path.relpath(os.path.dirname(out), os.getcwd())}")
    except KeyboardInterrupt:
        print("\nDừng. Lượt đang dở KHÔNG được ghi; node giữ setpoint cuối.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ctrl", choices=["hac", "fuzzy"])
    p.add_argument("--task", default="pick_cycle", help="tên trong tools/ab_compare/tasks/ hoặc đường dẫn .yaml")
    p.add_argument("--session", default=datetime.now().strftime("%Y%m%d"),
                   help="tên phiên so sánh; HAC và Fuzzy PHẢI dùng chung (mặc định: ngày)")
    p.add_argument("--trials", type=int, default=5)
    p.add_argument("--approach", type=float, default=4.0, help="giây đi từ tư thế hiện tại về start")
    p.add_argument("--out-root", default=os.path.join(REPO, "tuning_runs"))
    p.add_argument("--check", action="store_true", help="chỉ kiểm task (không cần ROS)")
    p.add_argument("--force", action="store_true")
    args = p.parse_args()

    task = load_task(args.task)
    if args.check:
        problems = check_task(task)
        print("\nOK" if not problems else "\nVẤN ĐỀ:\n  • " + "\n  • ".join(problems))
        sys.exit(1 if problems else 0)
    if not args.ctrl:
        p.error("--ctrl hac|fuzzy là bắt buộc khi chạy thật")
    run(args, task)


if __name__ == "__main__":
    main()
