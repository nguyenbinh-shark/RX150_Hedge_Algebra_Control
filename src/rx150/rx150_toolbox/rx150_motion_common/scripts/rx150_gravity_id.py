#!/usr/bin/env python3
"""
rx150_gravity_id — hiệu chuẩn gravity compensation cho hac_node (RX150).

4 mode (chạy tuần tự signcheck -> plan -> identify -> validate):

  signcheck — (B0, ~2 phút) PHẢI chạy với hac_node đang bật
              enable_gravity_comp:=false (PD thuần giữ pose, tránh giật nếu
              Gff/sign sai). Di chuyển 5 pose, tính correlation(effort đo,
              τ Pinocchio thô từ topic hac/gravity_torque — LUÔN được publish
              độc lập enable_gravity_comp/Gff/gravity_sign). Âm -> khuyến nghị
              đổi gravity_sign[i] = -1.

  plan      — sinh lưới pose an toàn (JSON) cho identify/validate: q2
              (shoulder) ∈ [-1.6,0], q3 (elbow) ∈ [0.6,1.8], q4 (wrist_angle)
              ∈ [-0.4,0.6], lọc trong joint limits (rx150_motor.yaml, margin
              0.2 rad) + EE cao >=5cm (FK nhẹ, tuning_lib.ee_height). Mỗi pose
              2 lượt di chuyển (tiếp cận từ 2 hướng, triệt stiction).

  identify  — (B1) chạy lưới, mỗi pose: tiếp cận, chờ settle (|qd|<0.02 rad/s
              trong 0.5s), lấy trung bình 1s (effort đo + PWM tổng hac/effort
              + q). Ridge-fit mô hình lượng giác — SONG SONG 2 đơn vị:
                - PWM (hac/effort): dùng để TRIỂN KHAI (hấp thụ sai số Gff +
                  khối lượng URDF) -> ghi thẳng vào rx150_gravity_model.yaml.
                - effort đo (joint_states.effort, Present_Load×2.69): mô hình
                  vật lý, dùng đối chiếu Gff-tương-đương với ngưỡng datasheet
                  XL430 (~590 PWM/N·m).
              80/20 train/held-out (seed cố định để tái lập). Thêm regressor
              hướng tiếp cận (d=±1) để tách thiên lệch stiction khỏi hệ số
              gravity — d KHÔNG được ghi vào fitted_gravity_coeffs triển khai.

  validate  — (B4) chạy lại lưới (hoặc --data <identify_data.csv> cũ, không
              cần động cơ) với 1 model .yaml, so sánh PWM dự đoán vs PWM đo
              thực tế -> residual RMS trước/sau hiệu chuẩn.

CHẠY (hac_node phải đang chạy, torque ON, PWM mode, gains_safe khuyến nghị):
  ros2 run rx150_motion_common rx150_gravity_id.py signcheck
  ros2 run rx150_motion_common rx150_gravity_id.py plan --out grid.json
  ros2 run rx150_motion_common rx150_gravity_id.py identify --grid grid.json
  ros2 run rx150_motion_common rx150_gravity_id.py validate \\
      --model src/rx150/controllers/rx150_hac_controller/config/rx150_gravity_model.yaml \\
      --grid grid.json

AN TOÀN: nạp `gains_file:=rx150_hac_gains_safe.yaml` trước khi chạy identify/
validate (u_max thấp hơn, max_velocities<=0.5 rad/s). Luôn ngồi trong tầm với,
sẵn sàng `ros2 service call /rx150/torque_enable ... enable: false` (SAU KHI
đỡ tay tay). KHÔNG Ctrl-C hac_node khi tay đang chịu lực.
"""

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tuning_lib as tl

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float64MultiArray

try:
    from interbotix_xs_msgs.srv import RegisterValues
except ImportError:
    RegisterValues = None

HAC_SETPOINT_TOPIC = "/rx150/hac/setpoint"
HAC_NODE = "/rx150/hac_node"


def _motor_yaml():
    """config/rx150_motor.yaml: ưu tiên bản đã cài, fallback cây nguồn.

    Bản cũ hardcode ~/interbotix_ws/src/rx150_motion_common/config/... — đường
    dẫn đó chết khi package dời vào src/rx150/rx150_toolbox/ (refactor IRROS),
    làm plan/identify/validate ném FileNotFoundError ngay dòng đầu. Cùng cách
    làm với rx150_friction_id.py.
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
DEFAULT_OUT_DIR = os.environ.get(
    "RX150_TUNING_RUNS",
    os.path.expanduser("~/RX150_Hedge_Algebra_Control/tuning_runs"
                       if os.path.isdir(os.path.expanduser("~/RX150_Hedge_Algebra_Control"))
                       else "~/interbotix_ws/tuning_runs")
)

SETTLE_VEL_THRESH = 0.02   # rad/s
SETTLE_WINDOW_S = 0.5
SETTLE_TIMEOUT_S = 8.0
SAMPLE_WINDOW_S = 1.0
GRAVITY_JOINTS = ["shoulder", "elbow", "wrist_angle"]
BASIS_SIZE = {"shoulder": 6, "elbow": 4, "wrist_angle": 2}
COEFF_OFFSET = {"shoulder": 0, "elbow": 6, "wrist_angle": 10}


# ─────────────────────────── CLI ───────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    sp0 = sub.add_parser("signcheck", help="B0 — kiểm tra dấu gravity_sign")
    sp0.add_argument("--apply", action="store_true",
                      help="Tự sửa gravity_sign trong config/rx150_hac_gains.yaml của rx150_hac_controller nếu phát hiện sai dấu")
    sp0.add_argument("--out-dir", default=DEFAULT_OUT_DIR)

    sp1 = sub.add_parser("plan", help="Sinh lưới pose an toàn (JSON)")
    sp1.add_argument("--out", default="rx150_gravity_grid.json")
    sp1.add_argument("--n-per-axis", type=int, default=5,
                      help="Điểm chia mỗi trục (5^3=125 tổ hợp -> ~60 pose an toàn sau lọc)")
    sp1.add_argument("--margin", type=float, default=0.2, help="Margin joint limit (rad)")
    sp1.add_argument("--min-ee-height", type=float, default=0.05, help="Độ cao EE tối thiểu (m)")

    sp2 = sub.add_parser("identify", help="B1 — chạy lưới, ridge-fit mô hình gravity")
    sp2.add_argument("--grid", required=True)
    sp2.add_argument("--limit", type=int, default=None, help="Giới hạn số lượt di chuyển (debug)")
    sp2.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    sp2.add_argument("--approach-margin", type=float, default=0.25)

    sp3 = sub.add_parser("validate", help="B4 — so residual RMS trước/sau hiệu chuẩn")
    sp3.add_argument("--model", required=True, help="rx150_gravity_model.yaml đã fit")
    sp3.add_argument("--grid", default=None, help="Lưới để chạy lại (bỏ qua nếu dùng --data)")
    sp3.add_argument("--data", default=None, help="identify_data.csv cũ (không cần động cơ)")
    sp3.add_argument("--limit", type=int, default=None)
    sp3.add_argument("--approach-margin", type=float, default=0.25)
    sp3.add_argument("--out-dir", default=DEFAULT_OUT_DIR)

    return p.parse_args()


# ─────────────────────────── ROS helpers ───────────────────────────

def get_param(node, target_node, name, timeout=5.0):
    cli = node.create_client(GetParameters, f"{target_node}/get_parameters")
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetParameters.Request()
    req.names = [name]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return None
    pv = fut.result().values[0]
    if pv.type == ParameterType.PARAMETER_BOOL:
        return pv.bool_value
    if pv.type == ParameterType.PARAMETER_DOUBLE:
        return pv.double_value
    if pv.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(pv.double_array_value)
    if pv.type == ParameterType.PARAMETER_STRING:
        return pv.string_value
    return None


def read_motor_temps(node, timeout=5.0):
    """Present_Temperature (°C) của 5 khớp arm, qua /rx150/get_motor_registers.
    Trả None nếu service không sẵn sàng (không chặn hiệu chuẩn — chỉ để
    provenance)."""
    if RegisterValues is None:
        return None
    cli = node.create_client(RegisterValues, "/rx150/get_motor_registers")
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = RegisterValues.Request()
    req.cmd_type = "group"
    req.name = "arm"
    req.reg = "Present_Temperature"
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return None
    return list(fut.result().values)


class HacDriver:
    """Bọc 1 rclpy Node: publish setpoint /rx150/hac/setpoint, cache telemetry
    hac/* + joint_states (qua tuning_lib.TelemetryRecorder), settle-detect,
    sample trung bình cửa sổ.
    """

    def __init__(self, node_name):
        self.node = rclpy.create_node(node_name)
        self.pub = self.node.create_publisher(Float64MultiArray, HAC_SETPOINT_TOPIC, 10)
        self.recorder = tl.TelemetryRecorder(self.node, "/rx150/hac")

    def publish_pose(self, pose):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in pose]
        self.pub.publish(msg)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def move_to(self, pose):
        self.publish_pose(pose)
        self.spin(0.05)

    def wait_settle(self, timeout=SETTLE_TIMEOUT_S, thresh=SETTLE_VEL_THRESH,
                     window=SETTLE_WINDOW_S, hold_pose=None):
        start = time.monotonic()
        good_since = None
        while time.monotonic() - start < timeout:
            if hold_pose is not None:
                self.publish_pose(hold_pose)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            vel = self.recorder.latest_velocities()
            if tl.is_settled(vel, thresh):
                if good_since is None:
                    good_since = time.monotonic()
                elif time.monotonic() - good_since >= window:
                    return True
            else:
                good_since = None
        return False

    def sample_window(self, duration=SAMPLE_WINDOW_S, hold_pose=None):
        acc = {"q": [], "measured_effort": [], "pwm": [], "tau": []}
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if hold_pose is not None:
                self.publish_pose(hold_pose)
            rclpy.spin_once(self.node, timeout_sec=0.02)
            acc["q"].append(self.recorder.latest_positions())
            acc["measured_effort"].append(self.recorder.latest_measured_load())
            acc["pwm"].append(self.recorder.latest_effort_pwm())
            acc["tau"].append(self.recorder.latest_gravity_torque())
        return {k: np.nanmean(np.array(v, dtype=float), axis=0).tolist() for k, v in acc.items()}


# ─────────────────────────── Pose grid ───────────────────────────

def generate_pose_grid(margin=0.2, n_per_axis=4,
                        shoulder_range=(-1.6, 0.0), elbow_range=(0.6, 1.8),
                        wrist_range=(-0.4, 0.6), min_ee_height=0.05):
    limits = tl.load_joint_limits_rad(MOTOR_YAML, margin=margin)
    q2_vals = np.linspace(*shoulder_range, n_per_axis)
    q3_vals = np.linspace(*elbow_range, n_per_axis)
    q4_vals = np.linspace(*wrist_range, n_per_axis)

    total = 0
    kept = []
    for q2 in q2_vals:
        for q3 in q3_vals:
            for q4 in q4_vals:
                total += 1
                q = [0.0, float(q2), float(q3), float(q4), 0.0]
                ok = True
                for name, val in zip(GRAVITY_JOINTS, (q2, q3, q4)):
                    lo, hi = limits.get(name, (-math.pi, math.pi))
                    if not (lo <= val <= hi):
                        ok = False
                        break
                if ok and tl.ee_height(q) < min_ee_height:
                    ok = False
                if ok:
                    kept.append(q)
    return kept, total


def cmd_plan(args):
    kept, total = generate_pose_grid(margin=args.margin, n_per_axis=args.n_per_axis,
                                      min_ee_height=args.min_ee_height)
    grid = []
    for q in kept:
        grid.append({"q": q, "approach_dir": "+"})
        grid.append({"q": q, "approach_dir": "-"})

    out = {
        "generated": datetime.now().isoformat(),
        "ranges": {"shoulder": [-1.6, 0.0], "elbow": [0.6, 1.8], "wrist_angle": [-0.4, 0.6]},
        "margin": args.margin,
        "min_ee_height": args.min_ee_height,
        "n_grid_total": total,
        "n_pose_kept": len(kept),
        "n_moves": len(grid),
        "poses": grid,
    }
    with open(args.out, "w") as fh:
        json.dump(out, fh, indent=2)
    print(f"Grid: {total} tổ hợp -> {len(kept)} pose an toàn (EE>={args.min_ee_height}m, "
          f"trong joint limits margin={args.margin}rad) -> {len(grid)} lượt di chuyển "
          f"(2 hướng/pose). Ghi: {args.out}")


# ─────────────────────────── B0 signcheck ───────────────────────────

def _patch_gravity_sign(yaml_path, recs):
    import re
    with open(yaml_path) as fh:
        text = fh.read()
    m = re.search(r"^(\s*)gravity_sign:\s*\[(.*?)\]\s*$", text, flags=re.MULTILINE)
    if not m:
        print(f"Không thấy dòng 'gravity_sign:' trong {yaml_path} — sửa tay.")
        return False
    current = [float(x) for x in m.group(2).split(",")]
    for jn, rec in recs.items():
        idx = tl.ARM_JOINTS.index(jn)
        if idx < len(current):
            current[idx] = float(rec["recommend_sign"])
    arr = "[" + ", ".join(f"{v:g}" for v in current) + "]"
    text = text[:m.start()] + f"{m.group(1)}gravity_sign: {arr}" + text[m.end():]
    with open(yaml_path, "w") as fh:
        fh.write(text)
    return True


def cmd_signcheck(args):
    check_node = rclpy.create_node("rx150_gravity_id_check")
    egc = get_param(check_node, HAC_NODE, "enable_gravity_comp")
    check_node.destroy_node()
    if egc is None:
        print("CẢNH BÁO: không đọc được enable_gravity_comp — hac_node có đang chạy không?")
    elif egc is True:
        print("LỖI: enable_gravity_comp đang TRUE. B0 BẮT BUỘC chạy với "
              "enable_gravity_comp:=false (relaunch hac_node với override này) để tránh "
              "giật tay nếu Gff/gravity_sign đang sai. Hủy.")
        return

    driver = HacDriver("rx150_gravity_id_signcheck")
    kept, _ = generate_pose_grid(n_per_axis=3)
    if len(kept) < 3:
        print("Không tạo đủ pose an toàn (kiểm tra joint limits / EE height) — hủy.")
        return
    step = max(1, len(kept) // 5)
    poses = kept[::step][:5]

    samples = {j: {"tau": [], "eff": []} for j in GRAVITY_JOINTS}
    for i, q in enumerate(poses):
        print(f"[{i + 1}/{len(poses)}] move -> {[f'{v:.2f}' for v in q]}")
        driver.move_to(q)
        if not driver.wait_settle(hold_pose=q):
            print("  cảnh báo: chưa settle trong timeout — lấy mẫu tạm")
        s = driver.sample_window(hold_pose=q)
        for jn in GRAVITY_JOINTS:
            idx = tl.ARM_JOINTS.index(jn)
            samples[jn]["tau"].append(s["tau"][idx])
            samples[jn]["eff"].append(s["measured_effort"][idx])

    print("\nKết quả B0 signcheck (correlation effort đo vs τ Pinocchio thô):")
    recs = {}
    for jn, d in samples.items():
        tau = np.array(d["tau"])
        eff = np.array(d["eff"])
        valid = np.isfinite(tau) & np.isfinite(eff)
        if valid.sum() >= 2 and np.std(tau[valid]) > 1e-9 and np.std(eff[valid]) > 1e-9:
            corr = float(np.corrcoef(tau[valid], eff[valid])[0, 1])
        else:
            corr = float("nan")
        if corr == corr and corr < 0:
            verdict = "ĐỔI gravity_sign -> -1"
            rec_sign = -1
        elif corr == corr:
            verdict = "OK (giữ +1)"
            rec_sign = 1
        else:
            verdict = "không xác định (dữ liệu không đủ biến thiên)"
            rec_sign = 1
        print(f"  {jn:<14} corr = {corr:+.3f}  -> {verdict}")
        recs[jn] = {"correlation": corr, "recommend_sign": rec_sign}

    os.makedirs(args.out_dir, exist_ok=True)
    out_path = os.path.join(args.out_dir, f"signcheck_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
    with open(out_path, "w") as fh:
        json.dump(recs, fh, indent=2)
    print(f"\nĐã ghi: {out_path}")

    if args.apply and any(v["recommend_sign"] < 0 for v in recs.values()):
        yaml_path = tl.find_src_config("rx150_hac_controller", "rx150_hac_gains.yaml")
        if yaml_path is None:
            print("Không tìm thấy config/rx150_hac_gains.yaml trong cây src — sửa gravity_sign tay theo recs trên.")
        elif _patch_gravity_sign(yaml_path, recs):
            print(f"Đã sửa gravity_sign trong {yaml_path} — cần colcon build + relaunch hac_node "
                  f"rồi chạy lại signcheck để xác nhận.")


# ─────────────────────────── B1 identify / B4 validate (chung) ──────

def _clip_approach(q, idxs, direction, margin, raw_limits):
    q2 = list(q)
    for idx in idxs:
        jn = tl.ARM_JOINTS[idx]
        lo, hi = raw_limits.get(jn, (-math.pi, math.pi))
        q2[idx] = min(max(q[idx] + direction * margin, lo), hi)
    return q2


def _drive_and_sample(driver, moves, approach_margin, temp_node=None):
    raw_limits = tl.load_joint_limits_rad(MOTOR_YAML, margin=0.05)
    idxs = [tl.ARM_JOINTS.index(j) for j in GRAVITY_JOINTS]

    temps_start = read_motor_temps(temp_node or driver.node)
    rows = []
    for i, mv in enumerate(moves):
        q = mv["q"]
        direction = 1.0 if mv["approach_dir"] == "+" else -1.0
        approach_q = _clip_approach(q, idxs, direction, approach_margin, raw_limits)

        print(f"[{i + 1}/{len(moves)}] approach {[f'{v:.2f}' for v in approach_q]} "
              f"-> target {[f'{v:.2f}' for v in q]} (dir {mv['approach_dir']})")
        driver.move_to(approach_q)
        driver.spin(1.0)
        driver.move_to(q)
        if not driver.wait_settle(hold_pose=q):
            print("  cảnh báo: chưa settle trong timeout — lấy mẫu tạm")
        s = driver.sample_window(hold_pose=q)
        rows.append({
            "q": s["q"], "target_q": q, "dir": direction,
            "measured_effort": s["measured_effort"], "pwm": s["pwm"], "tau": s["tau"],
        })
    temps_end = read_motor_temps(temp_node or driver.node)
    return rows, temps_start, temps_end


def _write_identify_csv(rows, path):
    import csv
    header = ["dir"]
    for prefix_ in ("q", "target_q", "measured_effort", "pwm", "tau"):
        for j in tl.ARM_JOINTS:
            header.append(f"{prefix_}_{j}")
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for r in rows:
            row = [r["dir"]]
            for prefix_ in ("q", "target_q", "measured_effort", "pwm", "tau"):
                row.extend(r[prefix_])
            w.writerow(row)


def _rows_from_identify_csv(path):
    header, csv_rows = tl.read_csv(path)
    rows = []
    for raw in csv_rows:
        d = dict(zip(header, raw))
        row = {"dir": d["dir"]}
        for prefix_ in ("q", "target_q", "measured_effort", "pwm", "tau"):
            row[prefix_] = [d[f"{prefix_}_{j}"] for j in tl.ARM_JOINTS]
        rows.append(row)
    return rows


def _fit_ridge(X, y, seed=42, test_frac=0.2, ridge_rel=1e-4):
    n = X.shape[0]
    rng = np.random.default_rng(seed)
    perm = rng.permutation(n)
    n_test = max(1, int(round(n * test_frac))) if n >= 5 else 0
    test_idx, train_idx = perm[:n_test], perm[n_test:]
    Xtr, ytr = X[train_idx], y[train_idx]
    Xte, yte = X[test_idx], y[test_idx]

    XtX = Xtr.T @ Xtr
    lam = ridge_rel * np.trace(XtX) / max(1, XtX.shape[0])
    beta = np.linalg.solve(XtX + lam * np.eye(XtX.shape[0]), Xtr.T @ ytr)

    def r2(X_, y_):
        if len(y_) == 0:
            return float("nan")
        pred = X_ @ beta
        ss_res = np.sum((y_ - pred) ** 2)
        ss_tot = np.sum((y_ - np.mean(y_)) ** 2)
        return float(1 - ss_res / ss_tot) if ss_tot > 1e-12 else float("nan")

    resid_tr = ytr - Xtr @ beta
    resid_te = yte - Xte @ beta
    return {
        "beta": beta.tolist(),
        "r2_train": r2(Xtr, ytr),
        "r2_heldout": r2(Xte, yte),
        "residual_rms_train": float(np.sqrt(np.mean(resid_tr ** 2))) if len(resid_tr) else float("nan"),
        "residual_rms_heldout": float(np.sqrt(np.mean(resid_te ** 2))) if len(resid_te) else float("nan"),
        "n_train": int(len(train_idx)),
        "n_test": int(len(test_idx)),
    }


def _basis(rows):
    m2 = np.array([r["q"][1] for r in rows])
    m3 = m2 + np.array([r["q"][2] for r in rows])
    m4 = m3 + np.array([r["q"][3] for r in rows])
    d = np.array([r["dir"] for r in rows])
    return {
        "shoulder": np.stack([np.cos(m2), np.sin(m2), np.cos(m3), np.sin(m3),
                              np.cos(m4), np.sin(m4), d], axis=1),
        "elbow": np.stack([np.cos(m3), np.sin(m3), np.cos(m4), np.sin(m4), d], axis=1),
        "wrist_angle": np.stack([np.cos(m4), np.sin(m4), d], axis=1),
    }


def fit_gravity_model(rows):
    bases = _basis(rows)
    pwm = np.array([r["pwm"] for r in rows])
    eff = np.array([r["measured_effort"] for r in rows])

    coeffs12_pwm = [0.0] * 12
    result = {"joints": {}, "n_samples": len(rows)}
    for jn, X in bases.items():
        idx = tl.ARM_JOINTS.index(jn)
        fit_pwm = _fit_ridge(X, pwm[:, idx])
        fit_eff = _fit_ridge(X, eff[:, idx])

        nb = BASIS_SIZE[jn]
        beta_pwm_basis = fit_pwm["beta"][:nb]
        beta_eff_basis = fit_eff["beta"][:nb]
        # PWM trên MỘT ĐƠN VỊ joint_states.effort (= Present_Load×2.69, mô-men
        # TƯƠNG ĐỐI — xem tuning_lib.latest_measured_load), KHÔNG phải PWM/N·m:
        # cả tử lẫn mẫu đều fit trên cùng bộ pose, mẫu ở đơn vị load thô. Muốn so
        # với ngưỡng datasheet XL430 (~590 PWM/N·m) phải quy đổi load -> N·m trước.
        pwm_per_load = [
            (bp / be) if abs(be) > 1e-9 else float("nan")
            for bp, be in zip(beta_pwm_basis, beta_eff_basis)
        ]
        coeffs12_pwm[COEFF_OFFSET[jn]:COEFF_OFFSET[jn] + nb] = beta_pwm_basis

        result["joints"][jn] = {
            "fit_pwm": fit_pwm,
            "fit_effort": fit_eff,
            "stiction_bias_pwm": fit_pwm["beta"][-1],
            "stiction_bias_effort": fit_eff["beta"][-1],
            "pwm_per_load_unit_per_term": pwm_per_load,
            "pwm_per_load_unit_mean_abs": float(np.nanmean(np.abs(pwm_per_load))),
        }

    result["fitted_gravity_coeffs_pwm"] = coeffs12_pwm
    return result


def _write_gravity_model_yaml(path, fit, provenance):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    arr = ", ".join(f"{v:.6g}" for v in fit["fitted_gravity_coeffs_pwm"])
    lines = [
        "# Sinh tự động bởi rx150_gravity_id.py (mode identify) — SỬA bằng cách chạy lại",
        "# identify, KHÔNG sửa tay hệ số. Xem model_fit.json cùng thư mục run để có đầy đủ",
        "# R²/residual/provenance.",
        f"# generated: {provenance['timestamp']}  n_samples: {fit['n_samples']}",
        f"# motor_temp_c_start: {provenance['motor_temp_c_start']}  "
        f"motor_temp_c_end: {provenance['motor_temp_c_end']}",
        f"# payload: {provenance['payload_note']}",
        "/rx150/hac_node:",
        "  ros__parameters:",
        '    gravity_model_source: "fitted"',
        f"    fitted_gravity_coeffs: [{arr}]",
    ]
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


def cmd_identify(args):
    with open(args.grid) as fh:
        grid = json.load(fh)
    moves = grid["poses"]
    if args.limit:
        moves = moves[:args.limit]
    if not moves:
        print("Grid rỗng — chạy `plan` trước.")
        return

    driver = HacDriver("rx150_gravity_id_identify")
    rows, temps_start, temps_end = _drive_and_sample(driver, moves, args.approach_margin)
    print(f"Nhiệt độ động cơ: start={temps_start}  end={temps_end}")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = os.path.join(args.out_dir, f"gravity_identify_{ts}")
    os.makedirs(run_dir, exist_ok=True)
    _write_identify_csv(rows, os.path.join(run_dir, "identify_data.csv"))

    fit = fit_gravity_model(rows)
    provenance = {
        "timestamp": ts,
        "n_poses": len(rows),
        "motor_temp_c_start": temps_start,
        "motor_temp_c_end": temps_end,
        "payload_note": "SỬA TAY: payload đang gắn khi hiệu chuẩn (RealSense/gripper/vật cầm).",
    }
    fit["provenance"] = provenance
    with open(os.path.join(run_dir, "model_fit.json"), "w") as fh:
        json.dump(fit, fh, indent=2)

    print(f"\n=== Kết quả fit ({run_dir}) ===")
    for jn in GRAVITY_JOINTS:
        r = fit["joints"][jn]
        print(f"{jn:<14} R²(pwm) train={r['fit_pwm']['r2_train']:.3f} "
              f"heldout={r['fit_pwm']['r2_heldout']:.3f}  "
              f"R²(eff) train={r['fit_effort']['r2_train']:.3f} "
              f"heldout={r['fit_effort']['r2_heldout']:.3f}  "
              f"PWM/đơn-vị-load~{r['pwm_per_load_unit_mean_abs']:.2f}")

    # stiction_bias_pwm là con số quyết định sàn sai số xác lập: nó BỊ LOẠI khỏi
    # model triển khai (đúng — nó phụ thuộc hướng tiếp cận, không phải trọng
    # lực), nên phần đó ở lại dưới dạng trễ. Kp = 2c/(3a) của mặt HAC.
    kp_node = rclpy.create_node("rx150_gravity_id_kp")
    a_p = get_param(kp_node, HAC_NODE, "a")
    c_p = get_param(kp_node, HAC_NODE, "c")
    kp_node.destroy_node()
    if a_p and c_p:
        kp = 2.0 * c_p / (3.0 * a_p)
        print(f"\nSàn sai số do ma sát tĩnh (Kp={kp:.0f} PWM/rad, KHÔNG hiệu chuẩn "
              f"gravity nào chạm tới — cần rx150_friction_id.py):")
        for jn in GRAVITY_JOINTS:
            bias = abs(fit["joints"][jn]["stiction_bias_pwm"])
            print(f"  {jn:<14} ±{bias:6.1f} PWM  ->  ±{math.degrees(bias / kp):.2f}° trễ "
                  f"theo hướng tiếp cận")

    model_path = tl.src_config_path("rx150_hac_controller", "rx150_gravity_model.yaml")
    if model_path:
        _write_gravity_model_yaml(model_path, fit, provenance)
        print(f"\nĐã ghi model triển khai: {model_path}")
        print("Cần: colcon build --packages-select rx150_hac_controller, rồi relaunch với "
              "gains_file bao gồm rx150_gravity_model.yaml (gravity_model_source:=fitted) "
              "để hac_node dùng model này.")
    else:
        print("\nKhông suy ra được src path cho rx150_gravity_model.yaml — dùng "
              "fitted_gravity_coeffs_pwm trong model_fit.json để tự ghi yaml.")


def _predict_fitted_pwm(rows, coeffs):
    m2 = np.array([r["q"][1] for r in rows])
    m3 = m2 + np.array([r["q"][2] for r in rows])
    m4 = m3 + np.array([r["q"][3] for r in rows])
    pred = np.zeros((len(rows), 5))
    c = coeffs
    pred[:, 1] = (c[0] * np.cos(m2) + c[1] * np.sin(m2) + c[2] * np.cos(m3) + c[3] * np.sin(m3) +
                  c[4] * np.cos(m4) + c[5] * np.sin(m4))
    pred[:, 2] = c[6] * np.cos(m3) + c[7] * np.sin(m3) + c[8] * np.cos(m4) + c[9] * np.sin(m4)
    pred[:, 3] = c[10] * np.cos(m4) + c[11] * np.sin(m4)
    return pred


def cmd_validate(args):
    with open(args.model) as fh:
        model_yaml = yaml.safe_load(fh)
    coeffs = model_yaml["/rx150/hac_node"]["ros__parameters"]["fitted_gravity_coeffs"]

    if args.data:
        rows = _rows_from_identify_csv(args.data)
        print(f"Dùng lại dữ liệu cũ: {args.data} ({len(rows)} pose) — không di chuyển tay.")
    else:
        if not args.grid:
            print("Cần --grid (hoặc --data để dùng lại dữ liệu cũ).")
            return
        with open(args.grid) as fh:
            grid = json.load(fh)
        moves = grid["poses"]
        if args.limit:
            moves = moves[:args.limit]
        driver = HacDriver("rx150_gravity_id_validate")
        rows, temps_start, temps_end = _drive_and_sample(driver, moves, args.approach_margin)
        print(f"Nhiệt độ động cơ: start={temps_start}  end={temps_end}")

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(args.out_dir, f"gravity_validate_{ts}")
        os.makedirs(run_dir, exist_ok=True)
        _write_identify_csv(rows, os.path.join(run_dir, "validate_data.csv"))

    pred = _predict_fitted_pwm(rows, coeffs)
    actual = np.array([r["pwm"] for r in rows])
    resid = actual - pred

    print("\n=== Validate: residual RMS (PWM) mô hình fitted vs PWM đo thực tế ===")
    out = {}
    for jn in GRAVITY_JOINTS:
        idx = tl.ARM_JOINTS.index(jn)
        rms = float(np.sqrt(np.nanmean(resid[:, idx] ** 2)))
        raw_pwm_rms = float(np.sqrt(np.nanmean(actual[:, idx] ** 2)))
        print(f"  {jn:<14} residual_rms={rms:.2f} PWM  (so với RMS PWM thô={raw_pwm_rms:.2f}) "
              f"— so với model_fit.json['joints']['{jn}']['fit_pwm']['residual_rms_*'] "
              f"TRƯỚC hiệu chuẩn để tính tỉ lệ giảm.")
        out[jn] = {"residual_rms_pwm": rms, "raw_pwm_rms": raw_pwm_rms}
    print(json.dumps(out, indent=2))


def main():
    args = parse_args()
    rclpy.init()
    try:
        if args.mode == "plan":
            cmd_plan(args)
        elif args.mode == "signcheck":
            cmd_signcheck(args)
        elif args.mode == "identify":
            cmd_identify(args)
        elif args.mode == "validate":
            cmd_validate(args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
