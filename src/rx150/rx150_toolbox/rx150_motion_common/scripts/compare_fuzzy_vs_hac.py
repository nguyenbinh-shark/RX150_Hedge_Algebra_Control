#!/usr/bin/env python3
"""
compare_fuzzy_vs_hac.py
=======================
So sánh offline hai bộ điều khiển Fuzzy PD và HAC:
  - Tác vụ 1: Vẽ mặt điều khiển 3D  (e, ed) → PWM
  - Tác vụ 2: Mô phỏng trajectory profile + so sánh PWM theo thời gian

Output:
  control_surface_3d.png   — 3 surface plots (Fuzzy, HAC, |Diff|)
  trajectory_comparison.png — 4 subplots (profile, PWM, error, ΔPWM)
  comparison_data.csv       — bảng số liệu chi tiết

Không cần ROS / robot thật / ruckig Python. Chạy:
    python3 compare_fuzzy_vs_hac.py
"""

import math
import os
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless backend — luôn ghi file, không cần display
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — cần cho projection='3d'

# ═══════════════════════════════════════════════════════════════════════
# 0. THÔNG SỐ TỪ YAML CONFIGS
# ═══════════════════════════════════════════════════════════════════════

JOINT_NAMES = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]
N_JOINTS = 5

# --- Fuzzy PD (rx150_fuzzy_gains.yaml) ---
FUZZY_Ke   = [2.0, 3.0, 3.0, 5.0, 2.0]
FUZZY_Ked  = [0.005, 5e-5, 0.005, 0.005, 5e-4]
FUZZY_Ku   = [600.0, 800.0, 800.0, 700.0, 700.0]
FUZZY_umax = [600.0, 600.0, 600.0, 600.0, 600.0]
FUZZY_Gff  = [150.0, 255.0, 255.0, 500.0, 200.0]
FUZZY_grav_sign = [1.0, 1.0, 1.0, 1.0, 1.0]

# --- HAC (rx150_hac_gains.yaml) ---
HAC_a = 0.3
HAC_b = 12.0
HAC_c = 1200
HAC_error_limit     = [0.5, 0.333, 0.333, 0.2, 0.5]
HAC_error_dot_limit = [3.14, 3.14, 3.14, 3.14, 3.14]
HAC_umax = [600.0, 600.0, 600.0, 600.0, 600.0]
HAC_Gff  = [150.0, 255.0, 255.0, 500.0, 200.0]
HAC_grav_sign = [1.0, 1.0, 1.0, 1.0, 1.0]

# --- Profile ---
Q_START = [0.0, 0.0, 0.0, 0.0, 0.0]
Q_TARGET = [0.0, -1.80, 1.55, 0.8, 0.0]
V_MAX = 3.14    # rad/s
A_MAX = 5.0     # rad/s²
DT = 0.01       # s (100 Hz)

# --- Output directory ---
# Ghi kết quả vào thư mục ĐANG ĐỨNG, không phải vào scripts/ (scripts/ chỉ chứa
# code; và khi cài rồi thì script nằm ở lib/, ghi vào đó là sai hẳn).
# Đổi chỗ ghi bằng biến môi trường RX150_PLOT_DIR nếu cần.
OUTPUT_DIR = os.environ.get("RX150_PLOT_DIR", os.getcwd())

# ═══════════════════════════════════════════════════════════════════════
# 1. FUZZY TYPE-1 EVAL (port trung thực từ fuzzy_type1.c)
# ═══════════════════════════════════════════════════════════════════════

def _mu_gauss(sigma, c, x):
    if sigma == 0.0:
        return 1.0 if x == c else 0.0
    d = (x - c) / sigma
    return math.exp(-0.5 * d * d)

def _mu_zmf(a, b, x):
    if a == b:
        return 1.0 if x <= a else 0.0
    if x <= a:
        return 1.0
    if x >= b:
        return 0.0
    m = 0.5 * (a + b)
    if x <= m:
        t = (x - a) / (b - a)
        return 1.0 - 2.0 * t * t
    t = (b - x) / (b - a)
    return 2.0 * t * t

def _mu_smf(a, b, x):
    if a == b:
        return 1.0 if x >= a else 0.0
    if x <= a:
        return 0.0
    if x >= b:
        return 1.0
    m = 0.5 * (a + b)
    if x <= m:
        t = (x - a) / (b - a)
        return 2.0 * t * t
    t = (b - x) / (b - a)
    return 1.0 - 2.0 * t * t

# MF types
MF_ZMF = 0
MF_GAUSSMF = 1
MF_SMF = 2

def _mf_eval(mf_type, params, x):
    if mf_type == MF_ZMF:
        return _mu_zmf(params[0], params[1], x)
    elif mf_type == MF_GAUSSMF:
        return _mu_gauss(params[0], params[1], x)
    elif mf_type == MF_SMF:
        return _mu_smf(params[0], params[1], x)
    return 0.0

# Input MFs (5 cho e, 5 cho ed) — giống nhau cho cả 2 input
_INPUT_MFS = [
    (MF_ZMF,     [-0.8, -0.4]),     # NegB / NB
    (MF_GAUSSMF, [0.15, -0.4]),     # NegS / NS
    (MF_GAUSSMF, [0.15,  0.0]),     # Ze   / Z
    (MF_GAUSSMF, [0.15,  0.4]),     # PosS / PS
    (MF_SMF,     [0.4,   0.8]),     # PosB / PB
]

# Output MFs (5) — giống input MFs
_OUTPUT_MFS = _INPUT_MFS

# 25 rules: (input1_mf_idx, input2_mf_idx) -> output_mf_idx  (1-indexed)
_RULES = [
    (1,1,1), (2,1,1), (3,1,1), (4,1,2), (5,1,4),
    (1,2,1), (2,2,1), (3,2,2), (4,2,4), (5,2,5),
    (1,3,1), (2,3,2), (3,3,3), (4,3,4), (5,3,5),
    (1,4,1), (2,4,2), (3,4,4), (4,4,5), (5,4,5),
    (1,5,2), (2,5,4), (3,5,5), (4,5,5), (5,5,5),
]

FUZZY_N = 201  # centroid discretization points

def fuzzy_type1_eval(e, ed):
    """Port trung thực của fuzzy_type1_eval() từ fuzzy_type1.c"""
    # Input memberships
    mu_e  = [_mf_eval(mf[0], mf[1], e)  for mf in _INPUT_MFS]
    mu_ed = [_mf_eval(mf[0], mf[1], ed) for mf in _INPUT_MFS]

    # Firing strength per rule (AND = min)
    firing = []
    for (i1, i2, _) in _RULES:
        f = min(mu_e[i1-1], mu_ed[i2-1])  # weight = 1.0
        firing.append(f)

    # Output universe
    out_lo, out_hi = -1.0, 1.0
    y_vals = np.linspace(out_lo, out_hi, FUZZY_N)

    # Precompute output MF values
    mu_out = np.zeros((5, FUZZY_N))
    for m in range(5):
        mf_type, mf_params = _OUTPUT_MFS[m]
        for k in range(FUZZY_N):
            mu_out[m, k] = _mf_eval(mf_type, mf_params, y_vals[k])

    # Aggregate (AGG = max, IMP = min)
    agg = np.zeros(FUZZY_N)
    for r_idx, (_, _, o_idx) in enumerate(_RULES):
        for k in range(FUZZY_N):
            imp_val = min(firing[r_idx], mu_out[o_idx-1, k])
            if imp_val > agg[k]:
                agg[k] = imp_val

    # Defuzzify (centroid)
    num = np.sum(y_vals * agg)
    den = np.sum(agg)
    return num / den if den > 1e-9 else 0.0

# Vectorized version for surface plot
def fuzzy_type1_eval_grid(E, ED):
    """Evaluate fuzzy on 2D grids E, ED (same shape). Returns same-shape array."""
    result = np.zeros_like(E)
    for i in range(E.shape[0]):
        for j in range(E.shape[1]):
            result[i, j] = fuzzy_type1_eval(float(E[i, j]), float(ED[i, j]))
    return result

# ═══════════════════════════════════════════════════════════════════════
# 2. HAC EVAL (port từ hac.c)
# ═══════════════════════════════════════════════════════════════════════

def hac_eval(x, xd, a=HAC_a, b=HAC_b, c=HAC_c):
    """(2c/3a)·x + (c/3b)·xd"""
    return (2.0 * c / (3.0 * a)) * x + (c / (3.0 * b)) * xd

def hac_eval_grid(E, ED, a=HAC_a, b=HAC_b, c=HAC_c):
    """Vectorized HAC on 2D grids."""
    return (2.0 * c / (3.0 * a)) * E + (c / (3.0 * b)) * ED

# ═══════════════════════════════════════════════════════════════════════
# 3. TRAPEZOIDAL VELOCITY PROFILE (thay Ruckig TOTG)
# ═══════════════════════════════════════════════════════════════════════

def trapezoidal_profile(q0, qf, v_max, a_max, dt):
    """
    Sinh profile vận tốc hình thang cho 1 khớp.
    Returns: arrays (t, q_ref, qdot_ref, qddot_ref)
    """
    dist = qf - q0
    sign = 1.0 if dist >= 0 else -1.0
    d = abs(dist)

    if d < 1e-12:
        # Không di chuyển
        return np.array([0.0]), np.array([q0]), np.array([0.0]), np.array([0.0])

    # Thời gian gia tốc
    t_acc = v_max / a_max
    # Quãng đường gia tốc
    d_acc = 0.5 * a_max * t_acc**2

    if 2 * d_acc >= d:
        # Triangular profile (không đạt v_max)
        t_acc = math.sqrt(d / a_max)
        t_cruise = 0.0
        v_peak = a_max * t_acc
    else:
        # Trapezoidal
        t_cruise = (d - 2 * d_acc) / v_max
        v_peak = v_max

    t_total = 2 * t_acc + t_cruise
    n_steps = int(math.ceil(t_total / dt)) + 1
    t = np.linspace(0, t_total, n_steps)

    q_ref = np.zeros(n_steps)
    qdot_ref = np.zeros(n_steps)
    qddot_ref = np.zeros(n_steps)

    for k in range(n_steps):
        tk = t[k]
        if tk <= t_acc:
            # Acceleration phase
            qddot_ref[k] = sign * a_max
            qdot_ref[k] = sign * a_max * tk
            q_ref[k] = q0 + sign * 0.5 * a_max * tk**2
        elif tk <= t_acc + t_cruise:
            # Cruise phase
            dt_c = tk - t_acc
            qddot_ref[k] = 0.0
            qdot_ref[k] = sign * v_peak
            q_ref[k] = q0 + sign * (0.5 * a_max * t_acc**2 + v_peak * dt_c)
        else:
            # Deceleration phase
            dt_d = tk - t_acc - t_cruise
            qddot_ref[k] = -sign * a_max
            qdot_ref[k] = sign * (v_peak - a_max * dt_d)
            q_ref[k] = q0 + sign * (0.5 * a_max * t_acc**2
                                     + v_peak * t_cruise
                                     + v_peak * dt_d - 0.5 * a_max * dt_d**2)

    # Đảm bảo vị trí cuối chính xác
    q_ref[-1] = qf
    qdot_ref[-1] = 0.0
    qddot_ref[-1] = 0.0

    return t, q_ref, qdot_ref, qddot_ref


def generate_multi_joint_profile(q_start, q_target, v_max, a_max, dt):
    """
    Sinh profile per-khớp (không đồng bộ — mỗi khớp xong sớm nhất có thể).
    Đệm zero-order hold cho các khớp ngắn hơn.
    Returns: t_common, q_refs[n_joints][n_steps], qdot_refs, qddot_refs
    """
    profiles = []
    t_max = 0.0
    for j in range(N_JOINTS):
        t_j, q_j, qd_j, qdd_j = trapezoidal_profile(
            q_start[j], q_target[j], v_max, a_max, dt)
        profiles.append((t_j, q_j, qd_j, qdd_j))
        if t_j[-1] > t_max:
            t_max = t_j[-1]

    # Common time vector
    n_steps = int(math.ceil(t_max / dt)) + 1
    t_common = np.linspace(0, t_max, n_steps)

    q_refs = np.zeros((N_JOINTS, n_steps))
    qdot_refs = np.zeros((N_JOINTS, n_steps))
    qddot_refs = np.zeros((N_JOINTS, n_steps))

    for j in range(N_JOINTS):
        t_j, q_j, qd_j, qdd_j = profiles[j]
        # Interpolate to common time grid
        q_refs[j] = np.interp(t_common, t_j, q_j)
        qdot_refs[j] = np.interp(t_common, t_j, qd_j)
        qddot_refs[j] = np.interp(t_common, t_j, qdd_j)
        # After this joint's profile ends, hold final value
        mask = t_common > t_j[-1]
        q_refs[j][mask] = q_j[-1]
        qdot_refs[j][mask] = 0.0
        qddot_refs[j][mask] = 0.0

    return t_common, q_refs, qdot_refs, qddot_refs


# ═══════════════════════════════════════════════════════════════════════
# 4. GRAVITY MODEL (đơn giản, dùng sin(q) — đủ cho so sánh offline)
# ═══════════════════════════════════════════════════════════════════════

# Approximate gravity torques (N·m) based on typical RX-150 link masses/lengths.
# τ_grav ≈ m·L·g·sin(q) per joint, simplified.
# Joint 0 (waist) has ~0 gravity torque (rotation about vertical axis).
_GRAV_COEFFS = [0.0, 0.65, 0.25, 0.08, 0.0]  # N·m scale factors

def gravity_torques(q):
    """Simple sin(q) gravity model. Returns list of torques (N·m)."""
    tau = [0.0] * N_JOINTS
    for j in range(N_JOINTS):
        tau[j] = _GRAV_COEFFS[j] * math.sin(q[j])
    return tau


# ═══════════════════════════════════════════════════════════════════════
# 5. COMPUTE PWM — cả 2 controller
# ═══════════════════════════════════════════════════════════════════════

def compute_fuzzy_pwm(e, ed, q_actual, joint_idx):
    """Tính PWM cho Fuzzy PD controller (1 khớp, 1 bước)."""
    j = joint_idx
    en  = max(-1.0, min(1.0, FUZZY_Ke[j] * e))
    edn = max(-1.0, min(1.0, FUZZY_Ked[j] * ed))
    un = fuzzy_type1_eval(en, edn)
    u = un * FUZZY_Ku[j]

    # Gravity compensation
    tau_g = gravity_torques(q_actual)
    grav_pwm = tau_g[j] * FUZZY_Gff[j] * FUZZY_grav_sign[j]
    u += grav_pwm

    u = max(-FUZZY_umax[j], min(FUZZY_umax[j], u))
    return u, grav_pwm


def compute_hac_pwm(e, ed, q_actual, joint_idx):
    """Tính PWM cho HAC controller (1 khớp, 1 bước)."""
    j = joint_idx
    e_sat  = max(-HAC_error_limit[j],     min(HAC_error_limit[j],     e))
    ed_sat = max(-HAC_error_dot_limit[j],  min(HAC_error_dot_limit[j], ed))
    un = hac_eval(e_sat, ed_sat, HAC_a, HAC_b, HAC_c)
    u = un  # HAC không nhân Ku

    # Gravity compensation
    tau_g = gravity_torques(q_actual)
    grav_pwm = tau_g[j] * HAC_Gff[j] * HAC_grav_sign[j]
    u += grav_pwm

    u = max(-HAC_umax[j], min(HAC_umax[j], u))
    return u, grav_pwm


# ═══════════════════════════════════════════════════════════════════════
# 6. TÁC VỤ 1: MẶT ĐIỀU KHIỂN 3D
# ═══════════════════════════════════════════════════════════════════════

def plot_control_surfaces():
    """
    Vẽ mặt điều khiển 3D cho mỗi khớp:
      x = sai số vị trí e (rad)
      y = sai số vận tốc ed (rad/s)
      z = đầu ra PWM cuối cùng gửi đến động cơ (KHÔNG cộng gravity — chỉ phần phản hồi)
    """
    print("=" * 60)
    print("TÁC VỤ 1: Vẽ mặt điều khiển 3D")
    print("=" * 60)

    # Chọn joint tiêu biểu để plot chi tiết
    focus_joints = [1]  # shoulder (khớp nặng nhất) — thay đổi nếu muốn plot tất cả

    for jidx in focus_joints:
        jname = JOINT_NAMES[jidx]
        print(f"\n  Joint {jidx}: {jname}")

        # Error range: quét rộng hơn giới hạn HAC một chút
        e_max = max(1.0 / FUZZY_Ke[jidx], HAC_error_limit[jidx]) * 1.2
        ed_max = max(1.0 / FUZZY_Ked[jidx] if FUZZY_Ked[jidx] > 0 else 5.0,
                     HAC_error_dot_limit[jidx]) * 1.2
        # Cap ed_max to avoid extreme values
        ed_max = min(ed_max, 5.0)

        n_grid = 101
        e_vals  = np.linspace(-e_max, e_max, n_grid)
        ed_vals = np.linspace(-ed_max, ed_max, n_grid)
        E, ED = np.meshgrid(e_vals, ed_vals)

        # --- Fuzzy PD surface ---
        print(f"    Computing Fuzzy PD surface ({n_grid}x{n_grid})...")
        Z_fuzzy = np.zeros_like(E)
        for i in range(n_grid):
            for k in range(n_grid):
                e_val = float(E[i, k])
                ed_val = float(ED[i, k])
                en  = max(-1.0, min(1.0, FUZZY_Ke[jidx] * e_val))
                edn = max(-1.0, min(1.0, FUZZY_Ked[jidx] * ed_val))
                un = fuzzy_type1_eval(en, edn)
                u = un * FUZZY_Ku[jidx]
                u = max(-FUZZY_umax[jidx], min(FUZZY_umax[jidx], u))
                Z_fuzzy[i, k] = u

        # --- HAC surface ---
        print(f"    Computing HAC surface ({n_grid}x{n_grid})...")
        E_sat = np.clip(E, -HAC_error_limit[jidx], HAC_error_limit[jidx])
        ED_sat = np.clip(ED, -HAC_error_dot_limit[jidx], HAC_error_dot_limit[jidx])
        Z_hac = hac_eval_grid(E_sat, ED_sat, HAC_a, HAC_b, HAC_c)
        Z_hac = np.clip(Z_hac, -HAC_umax[jidx], HAC_umax[jidx])

        # --- Difference ---
        Z_diff = Z_fuzzy - Z_hac

        # --- Plot ---
        fig = plt.figure(figsize=(22, 7))
        fig.suptitle(
            f"Mặt điều khiển 3D — Joint: {jname}\n"
            f"x = sai số vị trí e (rad)   |   y = sai số vận tốc ed (rad/s)   |   z = PWM",
            fontsize=14, fontweight="bold"
        )

        # Color limits
        vmin = min(Z_fuzzy.min(), Z_hac.min())
        vmax = max(Z_fuzzy.max(), Z_hac.max())

        # (a) Fuzzy PD
        ax1 = fig.add_subplot(131, projection="3d")
        surf1 = ax1.plot_surface(E, ED, Z_fuzzy, cmap="coolwarm",
                                  alpha=0.85, vmin=vmin, vmax=vmax,
                                  edgecolor='none', antialiased=True)
        ax1.set_xlabel("e (rad)", fontsize=10)
        ax1.set_ylabel("ed (rad/s)", fontsize=10)
        ax1.set_zlabel("PWM", fontsize=10)
        ax1.set_title("Fuzzy PD\n(phi tuyến)", fontsize=12, color="blue")
        ax1.view_init(elev=25, azim=-60)
        fig.colorbar(surf1, ax=ax1, shrink=0.5, pad=0.1)

        # (b) HAC
        ax2 = fig.add_subplot(132, projection="3d")
        surf2 = ax2.plot_surface(E, ED, Z_hac, cmap="coolwarm",
                                  alpha=0.85, vmin=vmin, vmax=vmax,
                                  edgecolor='none', antialiased=True)
        ax2.set_xlabel("e (rad)", fontsize=10)
        ax2.set_ylabel("ed (rad/s)", fontsize=10)
        ax2.set_zlabel("PWM", fontsize=10)
        ax2.set_title(f"HAC\n(tuyến tính: a={HAC_a}, b={HAC_b}, c={HAC_c})", fontsize=12, color="red")
        ax2.view_init(elev=25, azim=-60)
        fig.colorbar(surf2, ax=ax2, shrink=0.5, pad=0.1)

        # (c) Signed Difference (Fuzzy - HAC)
        diff_max = max(abs(float(Z_diff.min())), abs(float(Z_diff.max())), 1.0)
        ax3 = fig.add_subplot(133, projection="3d")
        surf3 = ax3.plot_surface(E, ED, Z_diff, cmap="bwr",
                                  alpha=0.85, vmin=-diff_max, vmax=diff_max,
                                  edgecolor='none', antialiased=True)
        ax3.set_xlabel("e (rad)", fontsize=10)
        ax3.set_ylabel("ed (rad/s)", fontsize=10)
        ax3.set_zlabel("ΔPWM", fontsize=10)
        ax3.set_title("Chênh lệch\n(Fuzzy − HAC)", fontsize=12, color="green")
        ax3.view_init(elev=25, azim=-60)
        fig.colorbar(surf3, ax=ax3, shrink=0.5, pad=0.1)

        plt.tight_layout(rect=[0, 0, 1, 0.92])
        out_path = os.path.join(OUTPUT_DIR, f"control_surface_3d_{jname}.png")
        plt.savefig(out_path, dpi=150, bbox_inches="tight")
        plt.close()
        print(f"    → Saved: {out_path}")

    # --- Surface plot cho tất cả 5 khớp (compact) ---
    print(f"\n  Generating compact 5-joint surface overview...")
    fig, axes = plt.subplots(2, N_JOINTS, figsize=(25, 10),
                             subplot_kw={"projection": "3d"})
    fig.suptitle(
        "Mặt điều khiển 3D — Tất cả 5 khớp\n"
        "Hàng trên: Fuzzy PD (phi tuyến)  |  Hàng dưới: HAC (tuyến tính)",
        fontsize=14, fontweight="bold"
    )

    for jidx in range(N_JOINTS):
        jname = JOINT_NAMES[jidx]
        e_max = max(1.0 / FUZZY_Ke[jidx], HAC_error_limit[jidx]) * 1.2
        ed_max = max(1.0 / FUZZY_Ked[jidx] if FUZZY_Ked[jidx] > 0 else 5.0,
                     HAC_error_dot_limit[jidx]) * 1.2
        ed_max = min(ed_max, 5.0)

        n_g = 51  # coarser grid for 5-joint overview
        e_v  = np.linspace(-e_max, e_max, n_g)
        ed_v = np.linspace(-ed_max, ed_max, n_g)
        Eg, EDg = np.meshgrid(e_v, ed_v)

        # Fuzzy
        Zf = np.zeros_like(Eg)
        for i in range(n_g):
            for k in range(n_g):
                en  = max(-1.0, min(1.0, FUZZY_Ke[jidx] * float(Eg[i, k])))
                edn = max(-1.0, min(1.0, FUZZY_Ked[jidx] * float(EDg[i, k])))
                un = fuzzy_type1_eval(en, edn)
                u = max(-FUZZY_umax[jidx], min(FUZZY_umax[jidx], un * FUZZY_Ku[jidx]))
                Zf[i, k] = u

        # HAC
        Es = np.clip(Eg, -HAC_error_limit[jidx], HAC_error_limit[jidx])
        EDs = np.clip(EDg, -HAC_error_dot_limit[jidx], HAC_error_dot_limit[jidx])
        Zh = np.clip(hac_eval_grid(Es, EDs), -HAC_umax[jidx], HAC_umax[jidx])

        vmin = min(Zf.min(), Zh.min())
        vmax = max(Zf.max(), Zh.max())

        ax_f = axes[0, jidx]
        ax_f.plot_surface(Eg, EDg, Zf, cmap="coolwarm", alpha=0.8,
                          edgecolor='none', vmin=vmin, vmax=vmax)
        ax_f.set_title(f"{jname}\nFuzzy PD", fontsize=9)
        ax_f.set_xlabel("e", fontsize=7)
        ax_f.set_ylabel("ed", fontsize=7)
        ax_f.set_zlabel("PWM", fontsize=7)
        ax_f.tick_params(labelsize=6)
        ax_f.view_init(elev=25, azim=-60)

        ax_h = axes[1, jidx]
        ax_h.plot_surface(Eg, EDg, Zh, cmap="coolwarm", alpha=0.8,
                          edgecolor='none', vmin=vmin, vmax=vmax)
        ax_h.set_title(f"{jname}\nHAC", fontsize=9)
        ax_h.set_xlabel("e", fontsize=7)
        ax_h.set_ylabel("ed", fontsize=7)
        ax_h.set_zlabel("PWM", fontsize=7)
        ax_h.tick_params(labelsize=6)
        ax_h.view_init(elev=25, azim=-60)

    plt.tight_layout(rect=[0, 0, 1, 0.92])
    out_path = os.path.join(OUTPUT_DIR, "control_surface_3d_all_joints.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"    → Saved: {out_path}")


# ═══════════════════════════════════════════════════════════════════════
# 7. TÁC VỤ 2: MÔ PHỎNG TRAJECTORY + SO SÁNH PWM
# ═══════════════════════════════════════════════════════════════════════

def simulate_trajectory():
    """
    Mô phỏng trajectory profile + plant đơn giản, tính PWM cho cả 2 controller.
    """
    print("\n" + "=" * 60)
    print("TÁC VỤ 2: Mô phỏng trajectory + so sánh PWM")
    print("=" * 60)

    # 1. Generate profile
    print("  Generating trapezoidal velocity profile...")
    t, q_refs, qdot_refs, qddot_refs = generate_multi_joint_profile(
        Q_START, Q_TARGET, V_MAX, A_MAX, DT)
    n_steps = len(t)
    print(f"    {n_steps} steps, t_max = {t[-1]:.3f} s")

    # 2. Simple plant simulation
    # Quán tính quy đổi (PWM → rad/s²) cho mỗi khớp
    J_SCALE = [20.0, 30.0, 25.0, 15.0, 10.0]

    # State arrays
    q_fuzzy  = np.zeros((N_JOINTS, n_steps))
    qd_fuzzy = np.zeros((N_JOINTS, n_steps))
    q_hac    = np.zeros((N_JOINTS, n_steps))
    qd_hac   = np.zeros((N_JOINTS, n_steps))

    pwm_fuzzy = np.zeros((N_JOINTS, n_steps))
    pwm_hac   = np.zeros((N_JOINTS, n_steps))
    grav_fuzzy_arr = np.zeros((N_JOINTS, n_steps))
    grav_hac_arr   = np.zeros((N_JOINTS, n_steps))
    err_fuzzy = np.zeros((N_JOINTS, n_steps))
    err_hac   = np.zeros((N_JOINTS, n_steps))
    edot_fuzzy = np.zeros((N_JOINTS, n_steps))
    edot_hac   = np.zeros((N_JOINTS, n_steps))

    # Init
    for j in range(N_JOINTS):
        q_fuzzy[j, 0] = Q_START[j]
        q_hac[j, 0]   = Q_START[j]

    print("  Simulating...")
    for k in range(n_steps - 1):
        for j in range(N_JOINTS):
            # --- Fuzzy PD ---
            e_f  = q_refs[j, k] - q_fuzzy[j, k]
            ed_f = qdot_refs[j, k] - qd_fuzzy[j, k]
            err_fuzzy[j, k] = e_f
            edot_fuzzy[j, k] = ed_f

            q_list = [q_fuzzy[jj, k] for jj in range(N_JOINTS)]
            u_f, g_f = compute_fuzzy_pwm(e_f, ed_f, q_list, j)
            pwm_fuzzy[j, k] = u_f
            grav_fuzzy_arr[j, k] = g_f

            # Plant update (Fuzzy)
            tau_g = gravity_torques(q_list)
            qdd = (u_f / J_SCALE[j]) - tau_g[j] * 9.81 / J_SCALE[j]
            qd_fuzzy[j, k+1] = qd_fuzzy[j, k] + qdd * DT
            q_fuzzy[j, k+1]  = q_fuzzy[j, k]  + qd_fuzzy[j, k] * DT

            # --- HAC ---
            e_h  = q_refs[j, k] - q_hac[j, k]
            ed_h = qdot_refs[j, k] - qd_hac[j, k]
            err_hac[j, k] = e_h
            edot_hac[j, k] = ed_h

            q_list_h = [q_hac[jj, k] for jj in range(N_JOINTS)]
            u_h, g_h = compute_hac_pwm(e_h, ed_h, q_list_h, j)
            pwm_hac[j, k] = u_h
            grav_hac_arr[j, k] = g_h

            # Plant update (HAC)
            tau_g_h = gravity_torques(q_list_h)
            qdd_h = (u_h / J_SCALE[j]) - tau_g_h[j] * 9.81 / J_SCALE[j]
            qd_hac[j, k+1] = qd_hac[j, k] + qdd_h * DT
            q_hac[j, k+1]  = q_hac[j, k]  + qd_hac[j, k] * DT

    # Fill last step
    for j in range(N_JOINTS):
        err_fuzzy[j, -1] = q_refs[j, -1] - q_fuzzy[j, -1]
        err_hac[j, -1]   = q_refs[j, -1] - q_hac[j, -1]
        edot_fuzzy[j, -1] = qdot_refs[j, -1] - qd_fuzzy[j, -1]
        edot_hac[j, -1]   = qdot_refs[j, -1] - qd_hac[j, -1]

    # 3. Plot
    print("  Plotting trajectory comparison...")
    focus_j = 1  # shoulder
    jname = JOINT_NAMES[focus_j]

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(
        f"So sánh Fuzzy PD vs HAC — Joint: {jname}\n"
        f"Profile: [{Q_START[focus_j]}] → [{Q_TARGET[focus_j]}] rad, "
        f"v_max={V_MAX}, a_max={A_MAX}, dt={DT}s",
        fontsize=14, fontweight="bold"
    )

    # (a) Profile + Tracking (đã ẩn q_ref theo yêu cầu)
    ax = axes[0, 0]
    # ax.plot(t, q_refs[focus_j], "k--", linewidth=2, label="q_ref (profile)")
    ax.plot(t, q_fuzzy[focus_j], "b-", linewidth=1.5, label="q_actual (Fuzzy PD)")
    ax.plot(t, q_hac[focus_j], "r-", linewidth=1.5, label="q_actual (HAC)")
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Vị trí (rad)")
    ax.set_title("(a) Profile vị trí + Tracking")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    # (b) PWM comparison
    ax = axes[0, 1]
    ax.plot(t, pwm_fuzzy[focus_j], "b-", linewidth=1.2, label="PWM Fuzzy PD", alpha=0.8)
    ax.plot(t, pwm_hac[focus_j], "r-", linewidth=1.2, label="PWM HAC", alpha=0.8)
    ax.axhline(y=FUZZY_umax[focus_j], color="gray", linestyle=":", alpha=0.5, label=f"u_max=±{FUZZY_umax[focus_j]}")
    ax.axhline(y=-FUZZY_umax[focus_j], color="gray", linestyle=":", alpha=0.5)
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("PWM")
    ax.set_title("(b) Đầu ra PWM gửi đến động cơ")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    # (c) Tracking error
    ax = axes[1, 0]
    ax.plot(t, err_fuzzy[focus_j], "b-", linewidth=1.2, label="|e| Fuzzy PD", alpha=0.8)
    ax.plot(t, err_hac[focus_j], "r-", linewidth=1.2, label="|e| HAC", alpha=0.8)
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Sai số vị trí (rad)")
    ax.set_title("(c) Tracking error")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)

    # (d) ΔPWM
    ax = axes[1, 1]
    delta_pwm = pwm_fuzzy[focus_j] - pwm_hac[focus_j]
    ax.plot(t, delta_pwm, "g-", linewidth=1.2)
    ax.axhline(y=0, color="gray", linestyle="-", alpha=0.3)
    ax.fill_between(t, delta_pwm, 0, alpha=0.15, color="green")
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("ΔPWM (Fuzzy − HAC)")
    ax.set_title("(d) Chênh lệch PWM")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    out_path = os.path.join(OUTPUT_DIR, "trajectory_comparison.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"    → Saved: {out_path}")

    # --- Multi-joint trajectory plot ---
    fig, axes = plt.subplots(N_JOINTS, 2, figsize=(16, 3.5 * N_JOINTS))
    fig.suptitle("So sánh Fuzzy PD vs HAC — Tất cả 5 khớp", fontsize=14, fontweight="bold")

    for j in range(N_JOINTS):
        jn = JOINT_NAMES[j]
        # Position tracking
        ax = axes[j, 0]
        ax.plot(t, q_refs[j], "k--", linewidth=1.5, label="q_ref")
        ax.plot(t, q_fuzzy[j], "b-", linewidth=1, label="Fuzzy", alpha=0.8)
        ax.plot(t, q_hac[j], "r-", linewidth=1, label="HAC", alpha=0.8)
        ax.set_ylabel(f"{jn}\n(rad)", fontsize=9)
        if j == 0:
            ax.set_title("Vị trí")
            ax.legend(loc="best", fontsize=8)
        if j == N_JOINTS - 1:
            ax.set_xlabel("Thời gian (s)")
        ax.grid(True, alpha=0.3)

        # PWM
        ax = axes[j, 1]
        ax.plot(t, pwm_fuzzy[j], "b-", linewidth=1, label="Fuzzy", alpha=0.8)
        ax.plot(t, pwm_hac[j], "r-", linewidth=1, label="HAC", alpha=0.8)
        ax.set_ylabel("PWM", fontsize=9)
        if j == 0:
            ax.set_title("PWM output")
            ax.legend(loc="best", fontsize=8)
        if j == N_JOINTS - 1:
            ax.set_xlabel("Thời gian (s)")
        ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = os.path.join(OUTPUT_DIR, "trajectory_all_joints.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"    → Saved: {out_path}")

    # 4. CSV export
    csv_path = os.path.join(OUTPUT_DIR, "comparison_data.csv")
    print(f"  Writing CSV: {csv_path}")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "t(s)", "joint", "q_ref", "qdot_ref", "qddot_ref",
            "q_fuzzy", "q_hac", "e_fuzzy", "e_hac",
            "ed_fuzzy", "ed_hac",
            "pwm_fuzzy", "pwm_hac", "delta_pwm",
            "grav_fuzzy", "grav_hac",
        ])
        for k in range(n_steps):
            for j in range(N_JOINTS):
                writer.writerow([
                    f"{t[k]:.4f}", JOINT_NAMES[j],
                    f"{q_refs[j,k]:.6f}", f"{qdot_refs[j,k]:.6f}", f"{qddot_refs[j,k]:.6f}",
                    f"{q_fuzzy[j,k]:.6f}", f"{q_hac[j,k]:.6f}",
                    f"{err_fuzzy[j,k]:.6f}", f"{err_hac[j,k]:.6f}",
                    f"{edot_fuzzy[j,k]:.6f}", f"{edot_hac[j,k]:.6f}",
                    f"{pwm_fuzzy[j,k]:.4f}", f"{pwm_hac[j,k]:.4f}",
                    f"{pwm_fuzzy[j,k]-pwm_hac[j,k]:.4f}",
                    f"{grav_fuzzy_arr[j,k]:.4f}", f"{grav_hac_arr[j,k]:.4f}",
                ])
    print(f"    → Saved: {csv_path}")

    return t, pwm_fuzzy, pwm_hac, err_fuzzy, err_hac


# ═══════════════════════════════════════════════════════════════════════
# 8. MAIN
# ═══════════════════════════════════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║  SO SÁNH FUZZY PD vs HAC — Offline Simulation          ║")
    print("║  Output: control_surface_3d_*.png                      ║")
    print("║          trajectory_comparison.png                     ║")
    print("║          trajectory_all_joints.png                     ║")
    print("║          comparison_data.csv                           ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()

    # Verify fuzzy eval
    test_val = fuzzy_type1_eval(0.3, 0.2)
    print(f"  Fuzzy eval verify: fuzzy_type1_eval(0.3, 0.2) = {test_val:.6f}")
    print(f"  HAC eval verify:   hac_eval(0.3, 0.2)         = {hac_eval(0.3, 0.2):.6f}")
    print()

    plot_control_surfaces()
    simulate_trajectory()

    print("\n✅ Hoàn tất! Kiểm tra output trong:")
    print(f"   {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
