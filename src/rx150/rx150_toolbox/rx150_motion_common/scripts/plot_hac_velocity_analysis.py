#!/usr/bin/env python3
"""
plot_hac_velocity_analysis.py
=============================
Vẽ đồ thị phân tích & so sánh tác động của ĐẦU VÀO VẬN TỐC (ÂM và DƯƠNG)
đến đầu ra điều khiển PWM của bộ HAC (và so sánh với Fuzzy PD).

Tạo 2 file hình ảnh:
  1. hac_velocity_curves.png  - Đồ thị cắt 2D theo ed và Mô phỏng 2 chiều (Tiến & Lùi)
  2. hac_4quadrants_surface.png - Mặt 3D phân tích 4 góc phần tư (e, ed)
"""

import math
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Thông số HAC ---
HAC_a = 0.3
HAC_b = 12.0
HAC_c = 1200.0
HAC_e_limit = 0.333
HAC_ed_limit = 3.14
HAC_umax = 600.0

# --- Thông số Fuzzy (dành cho shoulder) ---
FUZZY_Ke = 3.0
FUZZY_Ked = 5e-5
FUZZY_Ku = 800.0

def hac_eval(e, ed):
    e_sat = max(-HAC_e_limit, min(HAC_e_limit, e))
    ed_sat = max(-HAC_ed_limit, min(HAC_ed_limit, ed))
    un = (2.0 * HAC_c / (3.0 * HAC_a)) * e_sat + (HAC_c / (3.0 * HAC_b)) * ed_sat
    return max(-HAC_umax, min(HAC_umax, un))

def hac_eval_inverted_ed(e, ed):
    """Giả lập trường hợp bị đảo ngược sai dấu ed (-ed)"""
    e_sat = max(-HAC_e_limit, min(HAC_e_limit, e))
    ed_sat = max(-HAC_ed_limit, min(HAC_ed_limit, ed))
    un = (2.0 * HAC_c / (3.0 * HAC_a)) * e_sat - (HAC_c / (3.0 * HAC_b)) * ed_sat
    return max(-HAC_umax, min(HAC_umax, un))

def fuzzy_mock(e, ed):
    """Giả lập đơn giản Fuzzy PD (S-curve gain)"""
    en = max(-1.0, min(1.0, FUZZY_Ke * e))
    edn = max(-1.0, min(1.0, FUZZY_Ked * ed))
    # Phi tuyến dạng tanh
    un = math.tanh(en * 1.5 + edn * 1.5)
    u = un * FUZZY_Ku
    return max(-HAC_umax, min(HAC_umax, u))

# Ghi kết quả vào thư mục ĐANG ĐỨNG, không phải vào scripts/ (scripts/ chỉ chứa
# code; và khi cài rồi thì script nằm ở lib/, ghi vào đó là sai hẳn).
# Đổi chỗ ghi bằng biến môi trường RX150_PLOT_DIR nếu cần.
OUTPUT_DIR = os.environ.get("RX150_PLOT_DIR", os.getcwd())

def plot_velocity_curves():
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("PHÂN TÍCH TÁC ĐỘNG CỦA ĐẦU VÀO VẬN TỐC (ÂM & DƯƠNG) TRÊN HAC", fontsize=15, fontweight='bold', y=0.98)

    ed_vals = np.linspace(-3.14, 3.14, 400)

    # --- Subplot 1: e = 0 rad (Chỉ có vận tốc ed) ---
    ax1 = axes[0, 0]
    u_hac_correct = [hac_eval(0.0, ed) for ed in ed_vals]
    u_hac_inverted = [hac_eval_inverted_ed(0.0, ed) for ed in ed_vals]
    u_fuzzy = [fuzzy_mock(0.0, ed) for ed in ed_vals]

    ax1.plot(ed_vals, u_hac_correct, 'r-', linewidth=2.5, label='HAC chuẩn (Đúng dấu: +ed)')
    ax1.plot(ed_vals, u_hac_inverted, 'r--', linewidth=2.0, alpha=0.7, label='HAC bị đảo dấu (-ed)')
    ax1.plot(ed_vals, u_fuzzy, 'b-.', linewidth=2.0, label='Fuzzy PD')
    ax1.axhline(0, color='black', linestyle=':', alpha=0.5)
    ax1.axvline(0, color='black', linestyle=':', alpha=0.5)
    ax1.set_title("1. Đáp ứng PWM khi e = 0 rad (Chỉ có ed)", fontsize=11, fontweight='bold')
    ax1.set_xlabel("Sai số vận tốc ed (rad/s)")
    ax1.set_ylabel("PWM Output")
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.legend(loc='upper left', fontsize=9)

    # Annotate negative vs positive velocity regions
    ax1.annotate('ed < 0 (Vận tốc Âm)\n-> PWM Âm (Hãm / Phanh)', xy=(-1.5, -200), xytext=(-2.8, -500),
                 arrowprops=dict(facecolor='red', shrink=0.05, width=1, headwidth=6), fontsize=9, color='red')
    ax1.annotate('ed > 0 (Vận tốc Dương)\n-> PWM Dương (Trợ tốc)', xy=(1.5, 200), xytext=(0.5, 450),
                 arrowprops=dict(facecolor='green', shrink=0.05, width=1, headwidth=6), fontsize=9, color='green')

    # --- Subplot 2: e = +0.1 rad (Có vị trí dương) ---
    ax2 = axes[0, 1]
    u_hac_pos_e = [hac_eval(0.1, ed) for ed in ed_vals]
    u_hac_inv_pos_e = [hac_eval_inverted_ed(0.1, ed) for ed in ed_vals]

    ax2.plot(ed_vals, u_hac_pos_e, 'r-', linewidth=2.5, label='HAC chuẩn (+ed)')
    ax2.plot(ed_vals, u_hac_inv_pos_e, 'r--', linewidth=2.0, alpha=0.7, label='HAC bị đảo dấu (-ed)')
    ax2.axhline(0, color='black', linestyle=':', alpha=0.5)
    ax2.axvline(0, color='black', linestyle=':', alpha=0.5)
    ax2.set_title("2. Đáp ứng PWM khi e = +0.1 rad", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Sai số vận tốc ed (rad/s)")
    ax2.set_ylabel("PWM Output")
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.legend(loc='upper left', fontsize=9)

    # --- Subplot 3: Trajectory Forward (0 -> 1.5 rad) & Backward (1.5 -> 0 rad) ---
    ax3 = axes[1, 0]
    t = np.linspace(0, 4.0, 400)
    # Profile vị trí: 0 -> 1.5 rad (t=0..2s), 1.5 -> 0 rad (t=2..4s)
    q_ref = np.where(t <= 2.0, 0.75 * (1 - np.cos(np.pi * t / 2.0)), 0.75 * (1 + np.cos(np.pi * (t - 2.0) / 2.0)))
    qdot_ref = np.where(t <= 2.0, 0.75 * np.pi / 2.0 * np.sin(np.pi * t / 2.0), -0.75 * np.pi / 2.0 * np.sin(np.pi * (t - 2.0) / 2.0))

    ax3.plot(t, q_ref, 'k-', linewidth=2.0, label='Vị trí đặt q_ref (rad)')
    ax3.plot(t, qdot_ref, 'g--', linewidth=2.0, label='Vận tốc đặt qdot_ref (rad/s)')
    ax3.axhline(0, color='black', linestyle=':', alpha=0.5)
    ax3.axvline(2.0, color='gray', linestyle='--', alpha=0.7, label='Đảo chiều movement')
    ax3.set_title("3. Trajectory 2 Chiều: Lượt đi (v > 0) & Lượt về (v < 0)", fontsize=11, fontweight='bold')
    ax3.set_xlabel("Thời gian (s)")
    ax3.set_ylabel("Góc (rad) / Vận tốc (rad/s)")
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.legend(loc='upper right', fontsize=9)

    # --- Subplot 4: PWM tương ứng lượt đi & lượt về ---
    ax4 = axes[1, 1]
    # Giả lập vị trí thực tế bám theo có một ít sai số
    q_act = q_ref - 0.05 * np.sin(np.pi * t / 2.0)
    qdot_act = qdot_ref - 0.1 * np.cos(np.pi * t / 2.0)

    e_sim = q_ref - q_act
    ed_sim = qdot_ref - qdot_act

    pwm_hac_sim = [hac_eval(e_sim[i], ed_sim[i]) for i in range(len(t))]
    pwm_hac_inv_sim = [hac_eval_inverted_ed(e_sim[i], ed_sim[i]) for i in range(len(t))]

    ax4.plot(t, pwm_hac_sim, 'r-', linewidth=2.5, label='HAC PWM (Đúng dấu)')
    ax4.plot(t, pwm_hac_inv_sim, 'r--', linewidth=2.0, alpha=0.7, label='HAC PWM (Sai dấu)')
    ax4.axhline(0, color='black', linestyle=':', alpha=0.5)
    ax4.axvline(2.0, color='gray', linestyle='--', alpha=0.7)
    ax4.set_title("4. Tín hiệu PWM của HAC tương ứng 2 chiều chuyển động", fontsize=11, fontweight='bold')
    ax4.set_xlabel("Thời gian (s)")
    ax4.set_ylabel("PWM Command")
    ax4.grid(True, linestyle='--', alpha=0.6)
    ax4.legend(loc='upper right', fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_path1 = os.path.join(OUTPUT_DIR, "hac_velocity_curves.png")
    plt.savefig(out_path1, dpi=150)
    plt.close()
    print(f" Saved: {out_path1}")

def plot_4quadrants_surface():
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    e_vals = np.linspace(-0.5, 0.5, 100)
    ed_vals = np.linspace(-3.14, 3.14, 100)
    E, ED = np.meshgrid(e_vals, ed_vals)

    Z = np.zeros_like(E)
    for i in range(E.shape[0]):
        for j in range(E.shape[1]):
            Z[i, j] = hac_eval(E[i, j], ED[i, j])

    surf = ax.plot_surface(E, ED, Z, cmap='coolwarm', alpha=0.85, edgecolor='none')
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='PWM Output')

    ax.set_title("MẶT 3D BỘ DIỀU KHIỂN HAC — PHÂN TÍCH 4 GÓC PHẦN TƯ (e, ed)", fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel("Sai số vị trí e (rad)", labelpad=10)
    ax.set_ylabel("Sai số vận tốc ed (rad/s)", labelpad=10)
    ax.set_zlabel("PWM Output", labelpad=10)

    # Set view angle
    ax.view_init(elev=25, azim=-45)

    plt.tight_layout()
    out_path2 = os.path.join(OUTPUT_DIR, "hac_4quadrants_surface.png")
    plt.savefig(out_path2, dpi=150)
    plt.close()
    print(f" Saved: {out_path2}")

if __name__ == "__main__":
    print("Generating HAC velocity analysis plots...")
    plot_velocity_curves()
    plot_4quadrants_surface()
    print("Done!")
