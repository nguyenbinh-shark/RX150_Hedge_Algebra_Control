#!/usr/bin/env python3
"""
rx150_run_compare — so sánh 2+ session (thư mục do rx150_tuning_session.py
tạo): overlay per-joint + bảng Δmetrics (console + markdown).

Khác với scripts/compare_fuzzy_vs_hac.py (hard-code so sánh fuzzy<->hac từ 1
CSV gộp, không align theo marker, không tính metrics) — tool này đọc N session
bất kỳ, tự align theo `marker_elapsed_s` trong meta.json, và dùng metrics.json
đã tính sẵn trong mỗi run (từ tuning_session.py).

Ví dụ:
  ros2 run rx150_motion_common rx150_run_compare.py \\
      ~/interbotix_ws/tuning_runs/a_0.3_20260827_101500 \\
      ~/interbotix_ws/tuning_runs/a_0.25_20260827_101800 \\
      --save compare_a
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tuning_lib as tl

METRIC_KEYS = ["rise_time", "overshoot_pct", "settling_time", "rmse_err", "ss_err", "rms_pwm"]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("run_dirs", nargs="+", help=">=2 thư mục run (rx150_tuning_session.py output)")
    p.add_argument("--joints", nargs="+", default=None, choices=tl.ARM_JOINTS,
                   help="Chỉ so sánh các khớp này (mặc định: tất cả)")
    p.add_argument("--save", default=None, help="Prefix file overlay .png (mặc định: không lưu)")
    p.add_argument("--md-out", default=None, help="Ghi bảng Δmetrics ra file markdown")
    p.add_argument("--no-show", action="store_true")
    return p.parse_args()


def load_run(run_dir):
    run_dir = os.path.abspath(os.path.expanduser(run_dir))
    with open(os.path.join(run_dir, "meta.json")) as fh:
        meta = json.load(fh)
    with open(os.path.join(run_dir, "metrics.json")) as fh:
        metrics = json.load(fh)
    header, rows = tl.read_csv(os.path.join(run_dir, "data.csv"))
    return {
        "dir": run_dir,
        "label": meta.get("label", os.path.basename(run_dir)),
        "meta": meta,
        "metrics": metrics,
        "header": header,
        "rows": rows,
    }


def col(run, name):
    idx = run["header"].index(name)
    return [r[idx] for r in run["rows"]]


def aligned_t(run):
    marker = run["meta"].get("marker_elapsed_s", 0.0)
    return [t - marker for t in col(run, "timestamp")]


def print_and_write_table(runs, joints, md_out):
    lines_md = ["| joint | metric | " + " | ".join(r["label"] for r in runs) +
                " | Δ(last-first) |", "|---" * (len(runs) + 3) + "|"]
    print(f"\n{'joint':<14}{'metric':<16}" + "".join(f"{r['label']:>14}" for r in runs) + f"{'Δ(last-first)':>16}")
    print("-" * (14 + 16 + 14 * len(runs) + 16))
    for j in joints:
        for key in METRIC_KEYS:
            vals = []
            for r in runs:
                v = r["metrics"].get(j, {}).get(key)
                vals.append(v)
            if all(v is None for v in vals):
                continue
            first = vals[0] if vals[0] is not None else float("nan")
            last = vals[-1] if vals[-1] is not None else float("nan")
            delta = (last - first) if (first == first and last == last) else float("nan")
            fmt = lambda v: f"{v:.4g}" if isinstance(v, (int, float)) and v == v else "—"
            print(f"{j:<14}{key:<16}" + "".join(f"{fmt(v):>14}" for v in vals) + f"{fmt(delta):>16}")
            lines_md.append(
                f"| {j} | {key} | " + " | ".join(fmt(v) for v in vals) + f" | {fmt(delta)} |")

    if md_out:
        with open(md_out, "w") as fh:
            fh.write("# rx150 run compare\n\n")
            fh.write("Runs: " + ", ".join(f"`{r['label']}` ({r['dir']})" for r in runs) + "\n\n")
            fh.write("\n".join(lines_md) + "\n")
        print(f"\nĐã ghi bảng markdown: {md_out}")


def make_overlay(runs, joints, save_prefix, show):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]

    for panel, cols in (
        ("position", [("pos", "-"), ("ref_pos", "--")]),
        ("error", [("err", "-")]),
        ("pwm", [("pwm", "-")]),
    ):
        fig, axes = plt.subplots(len(joints), 1, figsize=(10, 2.2 * len(joints)), sharex=True)
        if len(joints) == 1:
            axes = [axes]
        fig.suptitle(f"Compare — {panel}")
        for i, j in enumerate(joints):
            ax = axes[i]
            for ridx, r in enumerate(runs):
                t = aligned_t(r)
                color = colors[ridx % len(colors)]
                for suffix, style in cols:
                    colname = f"{j}_{suffix}"
                    if colname in r["header"]:
                        label = r["label"] if suffix in ("pos", "err", "pwm") else None
                        ax.plot(t, col(r, colname), style, color=color, linewidth=0.8,
                                alpha=0.9 if style == "-" else 0.5, label=label)
            ax.set_ylabel(j, fontsize=8)
            ax.axhline(0.0, color="gray", linewidth=0.4, linestyle=":")
            if i == 0:
                ax.legend(fontsize=7, loc="upper right")
        axes[-1].set_xlabel("t (s, đã align theo marker)")
        fig.tight_layout()
        if save_prefix:
            path = f"{save_prefix}_{panel}.png"
            fig.savefig(path, dpi=140)
            print(f"Đã lưu: {path}")
        if not show:
            plt.close(fig)

    if show:
        plt.show()


def main():
    args = parse_args()
    if len(args.run_dirs) < 2:
        print("ERROR: cần >=2 run_dirs để so sánh", file=sys.stderr)
        sys.exit(1)

    runs = [load_run(d) for d in args.run_dirs]
    joints = args.joints or tl.ARM_JOINTS

    print_and_write_table(runs, joints, args.md_out)
    make_overlay(runs, joints, args.save, show=not args.no_show and args.save is None)


if __name__ == "__main__":
    main()
