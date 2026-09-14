#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_law.py — so CHI PHÍ TÍNH của riêng luật điều khiển HAC vs Fuzzy, offline.

Biên dịch bench_law.c cùng CHÍNH hac.c và fuzzy_type1.c trong src/ (không chép,
không viết lại), ở hai mức tối ưu:
    -O0  đúng như build hiện tại (colcon không đặt CMAKE_BUILD_TYPE -> không cờ -O)
    -O2  như khi build Release
rồi chạy ghim vào một lõi (taskset) và ghi:

    tuning_runs/bench_law_<ts>/bench.csv   ns / lần gọi: median, p10, p90, min, mean
    tuning_runs/bench_law_<ts>/report.md   bảng, tỉ lệ, % ngân sách chu kỳ, môi trường đo
    tuning_runs/bench_law_<ts>/fig_bench.png

Đây là chi phí CỦA LUẬT. Chi phí cả node (Ruckig, Pinocchio, DDS) lấy từ
ab-run/ab-report (topic <ctrl>/timing và CPU% tiến trình) — hai con số trả lời hai
câu hỏi khác nhau, báo cáo nên đưa cả hai.

Dùng:
    ./rx150.sh ab-bench
    ./rx150.sh ab-bench --core 3 --rounds 500 --loop-rate 400
"""
import argparse
import csv
import os
import platform
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
HAC_DIR = os.path.join(REPO, "src/rx150/controllers/rx150_hac_controller/src/hac")
FUZ_DIR = os.path.join(REPO, "src/rx150/controllers/rx150_fuzzy_controller/src/fuzzy")
N_JOINTS = 5


def sh(cmd):
    return subprocess.run(cmd, check=True, capture_output=True, text=True).stdout


def read_first(path, default="?"):
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return default


def cpu_model():
    for line in read_first("/proc/cpuinfo", "").splitlines():
        if line.startswith("model name"):
            return line.split(":", 1)[1].strip()
    return platform.processor() or "?"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--opts", nargs="+", default=["-O0", "-O2"])
    p.add_argument("--core", type=int, default=2, help="lõi CPU để ghim (taskset)")
    p.add_argument("--rounds", type=int, default=300)
    p.add_argument("--loop-rate", type=float, default=400.0, help="để tính %% ngân sách chu kỳ")
    p.add_argument("--out", default=None)
    args = p.parse_args()

    if not shutil.which("gcc"):
        sys.exit("cần gcc")
    out = args.out or os.path.join(REPO, "tuning_runs", "bench_law_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    governor = read_first(f"/sys/devices/system/cpu/cpu{args.core}/cpufreq/scaling_governor")
    gcc_ver = sh(["gcc", "--version"]).splitlines()[0]
    pin = ["taskset", "-c", str(args.core)] if shutil.which("taskset") else []

    rows, checks = [], {}
    with tempfile.TemporaryDirectory() as tmp:
        for opt in args.opts:
            exe = os.path.join(tmp, "bench" + opt.replace("-", "_"))
            sh(["gcc", opt, "-std=c11", "-I", HAC_DIR, "-I", FUZ_DIR, os.path.join(HERE, "bench_law.c"),
                os.path.join(HERE, "fuzzy_fast.c"), os.path.join(HAC_DIR, "hac.c"),
                os.path.join(FUZ_DIR, "fuzzy_type1.c"), "-lm", "-o", exe])
            print(f"[{opt}] chạy {args.rounds} vòng trên lõi {args.core} …", flush=True)
            for line in sh(pin + [exe, str(args.rounds)]).strip().splitlines():
                if line.startswith("check,"):
                    _, name, maxdiff = line.split(",")
                    checks[(opt, name)] = float(maxdiff)
                    continue
                name, med, p10, p90, mn, mean = line.split(",")
                rows.append({"opt": opt, "law": name, "median_ns": float(med), "p10_ns": float(p10),
                             "p90_ns": float(p90), "min_ns": float(mn), "mean_ns": float(mean)})

    with open(os.path.join(out, "bench.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    budget_ns = 1e9 / args.loop_rate
    get = {(r["opt"], r["law"]): r for r in rows}
    L = ["# Chi phí tính luật điều khiển — HAC vs Fuzzy", "",
         f"- CPU: {cpu_model()} · lõi {args.core} · governor `{governor}`",
         f"- Trình dịch: {gcc_ver}",
         f"- Mã đo: `hac.c` và `fuzzy_type1.c` trong `src/` (không sao chép), {args.rounds} vòng xen kẽ",
         f"- Ngân sách chu kỳ ở {args.loop_rate:.0f} Hz = {budget_ns / 1e3:.0f} µs; node gọi luật {N_JOINTS} lần/chu kỳ",
         "", "| tối ưu | HAC [ns/lần] | Fuzzy [ns/lần] | Fuzzy / HAC | HAC ×5 khớp [µs] | Fuzzy ×5 khớp [µs] "
         "| Fuzzy % ngân sách | khung đo [ns] |", "|---|---|---|---|---|---|---|---|"]
    print(f"\n{'opt':<5}{'HAC ns':>10}{'Fuzzy ns':>12}{'Fuzzy/HAC':>11}{'Fuzzy ×5 µs':>13}{'% chu kỳ':>10}")
    for opt in args.opts:
        h, f, c = get[(opt, "hac")], get[(opt, "fuzzy")], get[(opt, "call")]
        ratio = f["median_ns"] / h["median_ns"]
        pct = 100.0 * N_JOINTS * f["median_ns"] / budget_ns
        L.append(f"| `{opt}` | {h['median_ns']:.2f} ({h['p10_ns']:.2f}–{h['p90_ns']:.2f}) | "
                 f"{f['median_ns']:,.0f} ({f['p10_ns']:,.0f}–{f['p90_ns']:,.0f}) | **{ratio:,.0f}×** | "
                 f"{N_JOINTS * h['median_ns'] / 1e3:.4f} | {N_JOINTS * f['median_ns'] / 1e3:.1f} | {pct:.2f} % | "
                 f"{c['median_ns']:.2f} |")
        print(f"{opt:<5}{h['median_ns']:>10.2f}{f['median_ns']:>12,.0f}{ratio:>10,.0f}×"
              f"{N_JOINTS * f['median_ns'] / 1e3:>13.1f}{pct:>9.2f}%")
    L += ["", "Số trong ngoặc: p10–p90 giữa các vòng. `khung đo` = cùng vòng lặp gọi một hàm rỗng; "
              "thời gian HAC đo được phần lớn là chính khung này, nên tỉ lệ thật còn lớn hơn con số trong bảng.",
          "", "Vì sao chênh nhiều: mỗi lần gọi, `fuzzy_type1_eval` tính 10 hàm thuộc đầu vào, 25 độ kích hoạt, "
              "**dựng lại bảng 7 hàm thuộc đầu ra × 201 điểm** (1 407 lần `mf_eval`, dù bảng là hằng số), gộp "
              "25 luật × 201 điểm (5 025 lần min/max) rồi giải mờ trọng tâm. `hac_eval` là 2 phép nhân và 1 phép "
              "cộng (mặt HAC tuyến tính theo a, b, c).",
          "", "⚠ Governor `powersave` làm tần số CPU dao động; muốn số ổn định hơn: "
              "`sudo cpupower frequency-set -g performance` trong lúc đo." if governor == "powersave" else "",
          "", "![bench](fig_bench.png)"]
    L += ["", "## Bản Fuzzy hiện tại có bị thiệt không?", "",
          "Hai biến thể trong `fuzzy_fast.c` giữ nguyên FIS (luật, hàm thuộc, AND=min, IMP=min, AGG=max, "
          "trọng tâm trên 201 điểm), chỉ bỏ phần tính lặp. `max |Δu|` = sai khác lớn nhất so với bản gốc "
          "trên lưới 401 × 401 của (e, ed) ∈ [−1.2, 1.2]² — phải bằng 0.", "",
          "| tối ưu | biến thể | ns / lần | nhanh hơn bản gốc | chậm hơn HAC | ×5 khớp, % chu kỳ | max \\|Δu\\| |",
          "|---|---|---|---|---|---|---|"]
    print(f"\n{'opt':<5}{'biến thể Fuzzy':<34}{'ns':>9}{'vs gốc':>9}{'vs HAC':>9}{'max|Δu|':>10}")
    for opt in args.opts:
        base, hac = get[(opt, "fuzzy")]["median_ns"], get[(opt, "hac")]["median_ns"]
        for law, label in (("fuzzy", "gốc (fis2c.py sinh)"), ("fuzzy_pre", "tính sẵn bảng MF đầu ra"),
                           ("fuzzy_merge", "+ gộp luật cùng hệ quả (25 → 7)")):
            m = get[(opt, law)]["median_ns"]
            d = checks.get((opt, law), 0.0)
            L.append(f"| `{opt}` | {label} | {m:,.0f} | {base / m:.1f}× | {m / hac:,.0f}× | "
                     f"{100 * N_JOINTS * m / budget_ns:.2f} % | {d:.1e} |")
            print(f"{opt:<5}{label:<34}{m:>9,.0f}{base / m:>8.1f}×{m / hac:>8,.0f}×{d:>10.1e}")
    with open(os.path.join(out, "report.md"), "w") as fh:
        fh.write("\n".join(L) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    x = range(len(args.opts))
    series = (("hac", "#1f77b4", "HAC"), ("fuzzy", "#d62728", "Fuzzy gốc"),
              ("fuzzy_pre", "#ff7f0e", "Fuzzy tính sẵn bảng"), ("fuzzy_merge", "#f2b134", "Fuzzy + gộp luật"))
    w = 0.8 / len(series)
    for k, (law, color, label) in enumerate(series):
        med = [get[(o, law)]["median_ns"] for o in args.opts]
        err = [[m - get[(o, law)]["p10_ns"] for o, m in zip(args.opts, med)],
               [get[(o, law)]["p90_ns"] - m for o, m in zip(args.opts, med)]]
        pos = [i + (k - (len(series) - 1) / 2) * w for i in x]
        bars = ax[0].bar(pos, med, w, yerr=err, capsize=2, color=color, label=label)
        for b, m in zip(bars, med):
            ax[0].text(b.get_x() + b.get_width() / 2, m * 1.25, f"{m:,.1f}" if m < 100 else f"{m:,.0f}",
                       ha="center", fontsize=6.5)
        ax[1].bar(pos, [100.0 * N_JOINTS * m / budget_ns for m in med], w, color=color, label=label)
    ax[0].set_yscale("log")
    ax[0].set_ylabel("ns / lần gọi (log)")
    ax[0].set_title("Một lần gọi luật (median, thanh = p10–p90)", fontsize=10)
    ax[1].set_ylabel("% chu kỳ")
    ax[1].set_title(f"{N_JOINTS} khớp / chu kỳ, so với ngân sách {budget_ns / 1e3:.0f} µs ({args.loop_rate:.0f} Hz)",
                    fontsize=10)
    for a in ax:
        a.set_xticks(list(x))
        a.set_xticklabels(args.opts)
        a.grid(axis="y", alpha=0.3, ls=":")
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fig_bench.png"), dpi=160)
    print(f"\n-> {out}/report.md")


if __name__ == "__main__":
    main()
