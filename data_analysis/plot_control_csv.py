#!/usr/bin/env python3
"""
plot_control_csv.py — Vẽ đồ thị từ file CSV dữ liệu điều khiển.

Không cần ROS 2. Hoạt động với CSV từ bất kỳ bộ điều khiển nào
(fuzzy, PID, MPC...) miễn tuân thủ format cột chuẩn.

Cách dùng:
  python3 plot_control_csv.py fuzzy_data_20260812_153000.csv
  python3 plot_control_csv.py data.csv --joints waist shoulder
  python3 plot_control_csv.py data.csv --save output.png
  python3 plot_control_csv.py data.csv --start 2.0 --end 10.0
"""

import argparse
import sys
import os

# Thêm thư mục gốc module vào path để import utils
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib.pyplot as plt
from utils import load_csv, detect_joints, setup_style
from utils.csv_loader import get_data_summary
from utils.plot_styles import get_joint_color


def parse_args():
    parser = argparse.ArgumentParser(
        description='Vẽ đồ thị dữ liệu điều khiển từ file CSV.')
    parser.add_argument('csv_file', help='Đường dẫn file CSV')
    parser.add_argument('--joints', nargs='+', default=None,
                        help='Chỉ vẽ các khớp này (mặc định: tự detect)')
    parser.add_argument('--save', default=None,
                        help='Lưu hình vào file (ví dụ: output.png)')
    parser.add_argument('--no-show', action='store_true',
                        help='Không hiển thị cửa sổ đồ thị (chỉ lưu)')
    parser.add_argument('--start', type=float, default=None,
                        help='Thời gian bắt đầu (giây)')
    parser.add_argument('--end', type=float, default=None,
                        help='Thời gian kết thúc (giây)')
    parser.add_argument('--title', default=None,
                        help='Tiêu đề cho đồ thị (mặc định: tên file)')
    return parser.parse_args()


# ── Plot functions ───────────────────────────────────────────────────


def plot_position_tracking(df, joints, axes_list):
    """Vị trí thực vs reference cho mỗi khớp."""
    for i, joint in enumerate(joints):
        ax = axes_list[i]
        t = df['timestamp']
        color = get_joint_color(joint, i)
        pos_col = f'{joint}_pos'
        err_col = f'{joint}_err'

        if pos_col in df.columns:
            ax.plot(t, df[pos_col], color=color,
                    linewidth=0.8, label='actual')

        # ref = pos + error (vì error = ref - pos)
        if pos_col in df.columns and err_col in df.columns:
            ref = df[pos_col] + df[err_col]
            ax.plot(t, ref, '--', color='gray', linewidth=0.8,
                    alpha=0.8, label='reference (pos+err)')

        # Nếu có cột ref_pos riêng và không toàn NaN
        ref_col = f'{joint}_ref_pos'
        if ref_col in df.columns and df[ref_col].notna().any():
            ax.plot(t, df[ref_col], ':', color='black', linewidth=0.8,
                    alpha=0.7, label='ref (topic)')

        ax.set_ylabel(f'{joint}\n(rad)')
        ax.legend(loc='upper right')

    axes_list[-1].set_xlabel('Time (s)')


def plot_error(df, joints, axes_list):
    """Sai số theo thời gian."""
    for i, joint in enumerate(joints):
        ax = axes_list[i]
        t = df['timestamp']
        color = get_joint_color(joint, i)
        col = f'{joint}_err'
        if col in df.columns:
            ax.plot(t, df[col], color=color, linewidth=0.6)
            ax.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')
        ax.set_ylabel(f'{joint}\n(rad)')

    axes_list[-1].set_xlabel('Time (s)')


def plot_pwm(df, joints, axes_list):
    """PWM effort tách gravity comp + friction FF + phần controller thuần."""
    for i, joint in enumerate(joints):
        ax = axes_list[i]
        t = df['timestamp']
        color = get_joint_color(joint, i)
        pwm_col = f'{joint}_pwm'
        grav_col = f'{joint}_grav'
        fric_col = f'{joint}_fric'

        if pwm_col in df.columns:
            ax.plot(t, df[pwm_col], color=color,
                    linewidth=0.6, label='total PWM')

        has_grav = grav_col in df.columns and df[grav_col].notna().any()
        has_fric = fric_col in df.columns and df[fric_col].notna().any()

        if has_grav:
            ax.plot(t, df[grav_col], color='orange', linewidth=0.6,
                    alpha=0.7, label='gravity comp')
        if has_fric:
            ax.plot(t, df[fric_col], color='#9b59b6', linewidth=0.6,
                    alpha=0.7, label='friction FF')

        # controller-only = total - gravity - friction (0 nếu cột thiếu)
        if pwm_col in df.columns and (has_grav or has_fric):
            ctrl_only = df[pwm_col].copy()
            if has_grav:
                ctrl_only = ctrl_only - df[grav_col]
            if has_fric:
                ctrl_only = ctrl_only - df[fric_col]
            ax.plot(t, ctrl_only, color='green', linewidth=0.5,
                    alpha=0.6, label='controller only')

        ax.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')
        ax.set_ylabel(f'{joint}\n(PWM)')
        ax.legend(loc='upper right')

    axes_list[-1].set_xlabel('Time (s)')


def plot_velocity(df, joints, axes_list):
    """Vận tốc thực vs edot."""
    for i, joint in enumerate(joints):
        ax = axes_list[i]
        t = df['timestamp']
        color = get_joint_color(joint, i)
        vel_col = f'{joint}_vel'
        edot_col = f'{joint}_edot'

        if vel_col in df.columns:
            ax.plot(t, df[vel_col], color=color,
                    linewidth=0.6, label='velocity')
        if edot_col in df.columns:
            ax.plot(t, df[edot_col], color='red', linewidth=0.5,
                    alpha=0.6, label='edot (≈−vel)')

        ax.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')
        ax.set_ylabel(f'{joint}\n(rad/s)')
        ax.legend(loc='upper right')

    axes_list[-1].set_xlabel('Time (s)')


# ── Main ─────────────────────────────────────────────────────────────


def make_subplots(n_joints, title):
    fig, axes = plt.subplots(n_joints, 1, figsize=(12, 2.2 * n_joints),
                              sharex=True)
    if n_joints == 1:
        axes = [axes]
    fig.suptitle(title, fontsize=12, y=0.98)
    return fig, axes


def main():
    args = parse_args()
    setup_style()

    df = load_csv(args.csv_file, args.start, args.end)

    # Detect hoặc dùng danh sách khớp do user chỉ định
    all_joints = detect_joints(df)
    if args.joints:
        joints = [j for j in args.joints if j in all_joints]
        unknown = [j for j in args.joints if j not in all_joints]
        if unknown:
            print(f'WARNING: khớp không tìm thấy trong CSV: {unknown}',
                  file=sys.stderr)
    else:
        joints = all_joints

    if not joints:
        print('ERROR: không có khớp hợp lệ', file=sys.stderr)
        sys.exit(1)

    info = get_data_summary(df, joints)
    label = args.title or os.path.basename(args.csv_file)
    print(f'Data: {info["n_rows"]} rows, {info["duration"]:.2f}s, '
          f'{info["frequency"]:.0f} Hz')
    print(f'Joints: {joints}')

    n = len(joints)

    # --- 4 Figure windows ---
    fig1, ax1 = make_subplots(n, f'Position Tracking — {label}')
    plot_position_tracking(df, joints, ax1)
    fig1.tight_layout()

    fig2, ax2 = make_subplots(n, f'Tracking Error (e = ref − pos) — {label}')
    plot_error(df, joints, ax2)
    fig2.tight_layout()

    fig3, ax3 = make_subplots(n, f'Control Effort (PWM) — {label}')
    plot_pwm(df, joints, ax3)
    fig3.tight_layout()

    fig4, ax4 = make_subplots(n, f'Velocity & Error Derivative — {label}')
    plot_velocity(df, joints, ax4)
    fig4.tight_layout()

    if args.save:
        base = args.save.rsplit('.', 1)
        ext = base[1] if len(base) > 1 else 'png'
        name = base[0]
        for fig, suffix in [(fig1, 'position'), (fig2, 'error'),
                            (fig3, 'pwm'), (fig4, 'velocity')]:
            path = f'{name}_{suffix}.{ext}'
            fig.savefig(path, dpi=150, bbox_inches='tight')
        print(f'Saved: {name}_position.{ext}, {name}_error.{ext}, '
              f'{name}_pwm.{ext}, {name}_velocity.{ext}')

    if not args.no_show:
        plt.show()


if __name__ == '__main__':
    main()
