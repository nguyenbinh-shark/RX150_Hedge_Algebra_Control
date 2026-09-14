#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ab_report.py — xử lý số liệu phiên A/B do ab_run.py ghi, KHÔNG cần ROS.

Vào:  tuning_runs/ab_<session>/{hac,fuzzy}/trial_NN/{data.csv,meta.json} + params.yaml
Ra:   tuning_runs/ab_<session>/report/
        segments.csv    chỉ số từng (bộ, lượt, segment, khớp) — dạng dài, mở bằng pandas/Excel
        trials.csv      chỉ số gộp mỗi lượt (1 dòng / bộ / lượt)
        summary.csv     mean ± std mỗi bộ, Δ%, p-value Welch & Mann-Whitney
        per_joint.csv   mean ± std theo khớp
        report.md       bảng + kiểm công bằng tham số + ghi chú đọc số
        fig_tracking.png   q / e / u theo thời gian, lượt trung vị của mỗi bộ chồng lên nhau
        fig_trials.png     phân bố chỉ số theo lượt (box + điểm)
        fig_joints.png     cột theo khớp, thanh sai số = std giữa các lượt

Định nghĩa chỉ số (segment = từ lúc gửi waypoint tới hết 'hold'):
    rmse_e      RMS của e = q_ref − q do CHÍNH node tính (ref Ruckig, cùng tick)   [°]
    max_e       max |e|                                                           [°]
    ss_err      |q_target − q| trung bình trong --ss-window giây cuối segment     [°]
    settle      thời điểm (từ đầu segment) mà sau đó |q_target − q| ≤ băng mãi mãi [s]
                băng = max(2 % |Δq|, --band-deg); NaN = không bao giờ vào băng
    overshoot   phần vượt qua đích theo chiều chuyển động / |Δq|                  [%]
    rms_u       RMS PWM tổng (đã gồm bù trọng lực)                                [PWM]
    rms_u_fb    RMS phần phản hồi u − u_grav — phần thực sự do luật HAC/Fuzzy     [PWM]
    tv_u        tổng |Δu| / thời gian — đo rung / chattering lệnh                  [PWM/s]
    sat         % mẫu |u| ≥ 0.98·u_max                                            [%]
settle/overshoot/ss_err chỉ tính cho khớp có |Δq| ≥ --min-move (khớp đứng yên
thì không có đáp ứng để đo); rmse_e/max_e/u tính cho mọi khớp.

Dùng:
    ./rx150.sh ab-report tuning_runs/ab_s1
    ./rx150.sh ab-report tuning_runs/ab_s1 --drop-first 1      # bỏ lượt khởi động
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np
import pandas as pd
import yaml

ARM = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]
CTRLS = ["hac", "fuzzy"]
COLOR = {"hac": "#1f77b4", "fuzzy": "#d62728"}
LABEL = {"hac": "HAC", "fuzzy": "Fuzzy"}

# (cột, tên hiển thị, đơn vị, nhỏ hơn là tốt hơn)
TRIAL_METRICS = [
    ("rmse_e", "RMSE bám", "°", True),
    ("max_e", "Sai số bám lớn nhất", "°", True),
    ("ss_err", "Sai số xác lập TB", "°", True),
    ("ss_err_max", "Sai số xác lập xấu nhất", "°", True),
    ("settle", "Thời gian xác lập TB", "s", True),
    ("settle_fail", "Số lần không vào băng", "", True),
    ("overshoot", "Vọt lố TB", "%", True),
    ("rms_u", "RMS PWM tổng", "PWM", True),
    ("rms_u_fb", "RMS PWM phản hồi", "PWM", True),
    ("tv_u", "Biến thiên PWM", "PWM/s", True),
    ("sat", "Tỉ lệ bão hoà", "%", True),
    # chi phí tính — chỉ có khi node đã build với topic <ctrl>/timing
    ("law_us", "Khối luật 5 khớp TB", "µs", True),
    ("law_p99_us", "Khối luật p99", "µs", True),
    ("cyc_us", "Chu kỳ tính TB", "µs", True),
    ("cyc_p99_us", "Chu kỳ tính p99", "µs", True),
    ("cpu_pct", "CPU tiến trình", "% 1 lõi", True),
]

# tham số phải GIỐNG NHAU giữa hai bộ để quỹ đạo tham chiếu và điều kiện chạy như nhau
FAIRNESS = ["loop_rate", "debug_publish_rate", "velocity_filter_tau", "enable_profile",
            "max_velocities", "max_accelerations", "max_jerk", "sync_mode", "u_max",
            "enable_gravity_comp", "Gff", "gravity_sign", "gravity_model_source"]


# ─────────────────────────── đọc ────────────────────────────

def load_session(root):
    runs, params = [], {}
    for ctrl in CTRLS:
        pfile = os.path.join(root, ctrl, "params.yaml")
        if os.path.isfile(pfile):
            with open(pfile) as fh:
                params[ctrl] = (yaml.safe_load(fh) or {}).get("params", {})
        for tdir in sorted(glob.glob(os.path.join(root, ctrl, "trial_*"))):
            try:
                df = pd.read_csv(os.path.join(tdir, "data.csv"))
                with open(os.path.join(tdir, "meta.json")) as fh:
                    meta = json.load(fh)
            except (OSError, ValueError) as ex:
                print(f"  bỏ {tdir}: {ex}")
                continue
            if df.empty:
                print(f"  bỏ {tdir}: data.csv rỗng")
                continue
            runs.append({"ctrl": ctrl, "trial": meta["trial"], "dir": tdir, "df": df, "meta": meta})
    return runs, params


# ─────────────────────────── chỉ số ────────────────────────────

def segment_metrics(run, u_max, args):
    df, meta = run["df"], run["meta"]
    out = []
    prev_q = meta["start"]
    for seg in meta["segments"]:
        s = df[df["seg"] == seg["idx"]]
        if len(s) < 5:
            prev_q = seg["q"]
            continue
        t = s["t"].to_numpy() - seg["t_start"]
        dur = seg["t_end"] - seg["t_start"]
        for ji, j in enumerate(ARM):
            q = s[f"{j}_pos"].to_numpy()
            e = s[f"{j}_err"].to_numpy()
            u = s[f"{j}_pwm"].to_numpy()
            g = s[f"{j}_grav"].to_numpy()
            target, q0 = seg["q"][ji], prev_q[ji]
            dq = target - q0
            moving = abs(dq) >= args.min_move
            d = np.abs(target - q)
            row = {
                "ctrl": run["ctrl"], "trial": run["trial"], "seg": seg["idx"], "label": seg["label"],
                "joint": j, "dq_deg": math.degrees(dq), "moving": moving,
                "rmse_e": math.degrees(float(np.sqrt(np.nanmean(e ** 2)))),
                "max_e": math.degrees(float(np.nanmax(np.abs(e)))),
                "rms_u": float(np.sqrt(np.nanmean(u ** 2))),
                "rms_u_fb": float(np.sqrt(np.nanmean((u - np.nan_to_num(g)) ** 2))),
                "tv_u": float(np.nansum(np.abs(np.diff(u))) / dur) if dur > 0 else math.nan,
                "sat": 100.0 * float(np.mean(np.abs(u) >= 0.98 * u_max[ji])) if u_max else math.nan,
                "ss_err": math.nan, "settle": math.nan, "settle_fail": math.nan, "overshoot": math.nan,
            }
            if moving:
                tail = t >= (t[-1] - args.ss_window)
                row["ss_err"] = math.degrees(float(np.nanmean(d[tail])))
                band = max(0.02 * abs(dq), math.radians(args.band_deg))
                outside = np.flatnonzero(~(d <= band))
                if outside.size == 0:
                    row["settle"], row["settle_fail"] = float(t[0]), 0.0
                elif outside[-1] < len(t) - 1:
                    row["settle"], row["settle_fail"] = float(t[outside[-1] + 1]), 0.0
                else:
                    row["settle_fail"] = 1.0
                past = (q - target) * np.sign(dq)
                row["overshoot"] = 100.0 * max(0.0, float(np.nanmax(past))) / abs(dq)
            out.append(row)
        prev_q = seg["q"]
    return out


def timing_metrics(df):
    """Mỗi dòng = một cửa sổ debug. TB = trung bình các TB cửa sổ (cửa sổ đều độ dài);
    p99 lấy trên MAX của từng cửa sổ — thận trọng (chặn trên), không phải p99 từng chu kỳ."""
    if "law_mean_ns" not in df or not df["law_mean_ns"].notna().any():
        return {"law_us": math.nan, "law_p99_us": math.nan, "cyc_us": math.nan, "cyc_p99_us": math.nan}
    return {"law_us": df["law_mean_ns"].mean() / 1e3,
            "law_p99_us": float(np.nanpercentile(df["law_max_ns"], 99)) / 1e3,
            "cyc_us": df["cyc_mean_ns"].mean() / 1e3,
            "cyc_p99_us": float(np.nanpercentile(df["cyc_max_ns"], 99)) / 1e3}


def trial_metrics(run, seg_df):
    df = run["df"]
    e = df[[f"{j}_err" for j in ARM]].to_numpy()
    s = seg_df[(seg_df.ctrl == run["ctrl"]) & (seg_df.trial == run["trial"])]
    mv = s[s.moving]
    return {
        "ctrl": run["ctrl"], "trial": run["trial"],
        "rmse_e": math.degrees(float(np.sqrt(np.nanmean(e ** 2)))),
        "max_e": math.degrees(float(np.nanmax(np.abs(e)))),
        "ss_err": mv["ss_err"].mean(), "ss_err_max": mv["ss_err"].max(),
        "settle": mv["settle"].mean(), "settle_fail": mv["settle_fail"].sum(),
        "overshoot": mv["overshoot"].mean(),
        "rms_u": float(np.sqrt((s["rms_u"] ** 2).mean())),
        "rms_u_fb": float(np.sqrt((s["rms_u_fb"] ** 2).mean())),
        "tv_u": s["tv_u"].mean(), "sat": s["sat"].mean(),
        "rate_hz": run["meta"].get("rate_hz", math.nan),
        "cpu_pct": run["meta"].get("cpu_pct") if run["meta"].get("cpu_pct") is not None else math.nan,
        **timing_metrics(df),
    }


def compare(trials):
    import warnings
    try:
        from scipy import stats
        # 'catastrophic cancellation' khi hai bộ cho số gần trùng (vd. số lần không vào băng = 0) — vô hại
        warnings.filterwarnings("ignore", message="Precision loss occurred", category=RuntimeWarning)
    except ImportError:
        stats = None
    rows = []
    for col, name, unit, lower in TRIAL_METRICS:
        a = trials[trials.ctrl == "hac"][col].dropna().to_numpy()
        b = trials[trials.ctrl == "fuzzy"][col].dropna().to_numpy()
        if a.size == 0 and b.size == 0:
            continue  # vd. không có topic timing
        r = {"metric": col, "name": name, "unit": unit,
             "hac_mean": np.mean(a) if a.size else math.nan, "hac_std": np.std(a, ddof=1) if a.size > 1 else math.nan,
             "fuzzy_mean": np.mean(b) if b.size else math.nan,
             "fuzzy_std": np.std(b, ddof=1) if b.size > 1 else math.nan,
             "n_hac": a.size, "n_fuzzy": b.size, "p_welch": math.nan, "p_mwu": math.nan, "cohen_d": math.nan}
        r["delta_pct"] = (100.0 * (r["hac_mean"] - r["fuzzy_mean"]) / abs(r["fuzzy_mean"])
                          if r["fuzzy_mean"] not in (0.0,) and np.isfinite(r["fuzzy_mean"]) else math.nan)
        if a.size >= 2 and b.size >= 2:
            sp = math.sqrt(((a.size - 1) * a.var(ddof=1) + (b.size - 1) * b.var(ddof=1)) / (a.size + b.size - 2))
            r["cohen_d"] = (a.mean() - b.mean()) / sp if sp > 0 else math.nan
            if stats is not None and (a.var() > 0 or b.var() > 0):
                r["p_welch"] = float(stats.ttest_ind(a, b, equal_var=False).pvalue)
                r["p_mwu"] = float(stats.mannwhitneyu(a, b, alternative="two-sided").pvalue)
        if np.isfinite(r["hac_mean"]) and np.isfinite(r["fuzzy_mean"]) and r["hac_mean"] != r["fuzzy_mean"]:
            r["better"] = "HAC" if (r["hac_mean"] < r["fuzzy_mean"]) == lower else "Fuzzy"
        else:
            r["better"] = "="
        rows.append(r)
    return pd.DataFrame(rows)


def fairness(params):
    if set(params) != set(CTRLS):
        return [f"thiếu params.yaml của: {', '.join(c for c in CTRLS if c not in params)}"], []
    diffs, same = [], []
    for k in FAIRNESS:
        a, b = params["hac"].get(k), params["fuzzy"].get(k)
        if k == "gravity_model_source" and b is None:
            b = "pinocchio"  # fuzzy_node chỉ có đường Pinocchio + Gff
        (same if a == b else diffs).append((k, a, b))
    return diffs, same


# ─────────────────────────── vẽ ────────────────────────────

def median_trial(runs, trials, ctrl):
    t = trials[trials.ctrl == ctrl].sort_values("rmse_e")
    if t.empty:
        return None
    pick = int(t.iloc[len(t) // 2]["trial"])
    return next(r for r in runs if r["ctrl"] == ctrl and r["trial"] == pick)


def plot_tracking(runs, trials, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(len(ARM), 3, figsize=(15, 2.3 * len(ARM)), sharex=True)
    ref_drawn = False
    for ctrl in CTRLS:
        run = median_trial(runs, trials, ctrl)
        if run is None:
            continue
        # .to_numpy(): matplotlib 3.5 (apt) không nhận pandas Series 2.x
        df, c = run["df"], COLOR[ctrl]
        t = df["t"].to_numpy()
        for i, j in enumerate(ARM):
            if not ref_drawn:
                ax[i, 0].plot(t, np.degrees(df[f"{j}_ref_pos"].to_numpy()), "k--", lw=0.9, label="q_ref (Ruckig)")
            ax[i, 0].plot(t, np.degrees(df[f"{j}_pos"].to_numpy()), color=c, lw=1.0,
                          label=f"q {LABEL[ctrl]} (lượt {run['trial']})")
            ax[i, 1].plot(t, np.degrees(df[f"{j}_err"].to_numpy()), color=c, lw=0.9, label=LABEL[ctrl])
            ax[i, 2].plot(t, df[f"{j}_pwm"].to_numpy(), color=c, lw=0.8, label=LABEL[ctrl])
            ax[i, 0].set_ylabel(f"{j}\n[°]", fontsize=8)
        ref_drawn = True
        for s in run["meta"]["segments"]:
            for k in range(3):
                for i in range(len(ARM)):
                    ax[i, k].axvline(s["t_start"], color="0.85", lw=0.6, zorder=0)
    for k, title in enumerate(["Vị trí khớp", "Sai số bám e = q_ref − q [°]", "PWM tổng"]):
        ax[0, k].set_title(title, fontsize=10)
        ax[0, k].legend(fontsize=7, loc="best")
        ax[-1, k].set_xlabel("t [s]")
    for a in ax.flat:
        a.grid(alpha=0.3, ls=":")
        a.tick_params(labelsize=7)
    fig.suptitle("HAC vs Fuzzy — lượt có RMSE trung vị của mỗi bộ (vạch xám = bắt đầu segment)")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_trials(trials, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ms = [m for m in TRIAL_METRICS if m[0] != "settle_fail" and trials[m[0]].notna().any()]
    ncol = 5
    nrow = math.ceil(len(ms) / ncol)
    fig, ax = plt.subplots(nrow, ncol, figsize=(3.2 * ncol, 3.0 * nrow))
    rng = np.random.default_rng(0)
    for a, (col, name, unit, _) in zip(ax.flat, ms):
        data = [trials[trials.ctrl == c][col].dropna().to_numpy() for c in CTRLS]
        a.boxplot(data, widths=0.5, showfliers=False)
        for k, (c, d) in enumerate(zip(CTRLS, data)):
            a.scatter(k + 1 + rng.uniform(-0.12, 0.12, d.size), d, s=14, color=COLOR[c], zorder=3)
        a.set_xticks([1, 2])
        a.set_xticklabels([LABEL[c] for c in CTRLS])
        a.set_title(f"{name}" + (f" [{unit}]" if unit else ""), fontsize=9)
        a.grid(alpha=0.3, ls=":")
    for a in list(ax.flat)[len(ms):]:
        a.axis("off")
    fig.suptitle("Phân bố theo lượt (mỗi điểm = 1 lượt)")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_joints(pj, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cols = [("rmse_e", "RMSE bám [°]"), ("ss_err", "Sai số xác lập [°]"),
            ("settle", "Xác lập [s]"), ("rms_u_fb", "RMS PWM phản hồi")]
    fig, ax = plt.subplots(1, len(cols), figsize=(4.2 * len(cols), 3.4))
    x = np.arange(len(ARM))
    for a, (col, title) in zip(ax, cols):
        for k, c in enumerate(CTRLS):
            d = pj[pj.ctrl == c].set_index("joint").reindex(ARM)
            a.bar(x + (k - 0.5) * 0.38, d[f"{col}_mean"].to_numpy(), 0.38, yerr=d[f"{col}_std"].to_numpy(),
                  color=COLOR[c], capsize=3, label=LABEL[c])
        a.set_xticks(x)
        a.set_xticklabels(ARM, rotation=30, fontsize=8)
        a.set_title(title, fontsize=10)
        a.grid(axis="y", alpha=0.3, ls=":")
    ax[0].legend(fontsize=8)
    fig.suptitle("Theo khớp — cột = trung bình giữa các lượt, thanh = std")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


# ─────────────────────────── báo cáo ────────────────────────────

def fmt(v, nd=3):
    return "—" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f"{v:.{nd}g}"


def write_md(path, root, runs, summary, pj, diffs, same, task_names, args, params):
    L = [f"# So sánh HAC vs Fuzzy — `{os.path.basename(root)}`", ""]
    n = {c: sorted(r["trial"] for r in runs if r["ctrl"] == c) for c in CTRLS}
    L += [f"- Task: **{', '.join(task_names)}**",
          f"- Số lượt: HAC {len(n['hac'])} {n['hac']} · Fuzzy {len(n['fuzzy'])} {n['fuzzy']}"
          + (f" (đã bỏ {args.drop_first} lượt đầu mỗi bộ)" if args.drop_first else ""),
          f"- Băng xác lập: max(2 % |Δq|, {args.band_deg}°) · cửa sổ xác lập {args.ss_window} s "
          f"· khớp coi là chuyển động khi |Δq| ≥ {math.degrees(args.min_move):.1f}°", ""]

    L += ["## Kiểm công bằng", ""]
    if diffs:
        L += ["> ⚠ **Các tham số dưới đây KHÁC nhau giữa hai bộ** — khác biệt đo được có thể do chúng, "
              "không phải do luật điều khiển:", "", "| tham số | HAC | Fuzzy |", "|---|---|---|"]
        L += [f"| {k} | `{a}` | `{b}` |" for k, a, b in diffs]
    else:
        L += ["✓ Mọi tham số chung giống nhau: " + ", ".join(f"`{k}`" for k, _, _ in same) + "."]
    L += [""]

    L += ["## Kết quả theo lượt", "",
          "Δ% = (HAC − Fuzzy) / |Fuzzy|. Âm = HAC nhỏ hơn. Mọi chỉ số ở đây: **nhỏ hơn là tốt hơn**.", "",
          "| chỉ số | HAC (mean ± std) | Fuzzy (mean ± std) | Δ% | Cohen d | p Welch | p MWU | tốt hơn |",
          "|---|---|---|---|---|---|---|---|"]
    for _, r in summary.iterrows():
        u = f" [{r.unit}]" if r.unit else ""
        sig = " *" if np.isfinite(r.p_welch) and r.p_welch < 0.05 else ""
        L.append(f"| {r['name']}{u} | {fmt(r.hac_mean)} ± {fmt(r.hac_std, 2)} | "
                 f"{fmt(r.fuzzy_mean)} ± {fmt(r.fuzzy_std, 2)} | {fmt(r.delta_pct, 3)} | {fmt(r.cohen_d, 2)} | "
                 f"{fmt(r.p_welch, 2)}{sig} | {fmt(r.p_mwu, 2)} | {r.better} |")
    L += ["", "`*` p < 0.05 (Welch). Với n < 5 mỗi bộ, p-value chỉ mang tính tham khảo — "
              "Mann-Whitney với 3 vs 3 lượt không thể nhỏ hơn 0.1.", ""]

    S = summary.set_index("metric")
    if "law_us" in S.index or "cpu_pct" in S.index:
        def val(metric, ctrl):
            return float(S.loc[metric, f"{ctrl}_mean"]) if metric in S.index else math.nan

        budget = {c: 1e6 / (params.get(c) or {}).get("loop_rate", math.nan)
                  if (params.get(c) or {}).get("loop_rate") else math.nan for c in CTRLS}
        rows = [("Khối luật, 5 khớp [µs]", {c: val("law_us", c) for c in CTRLS}),
                ("Khối luật, 1 khớp [µs]", {c: val("law_us", c) / len(ARM) for c in CTRLS}),
                ("Chu kỳ tính [µs]", {c: val("cyc_us", c) for c in CTRLS}),
                ("Chu kỳ tính / ngân sách chu kỳ [%]", {c: 100.0 * val("cyc_us", c) / budget[c] for c in CTRLS}),
                ("Khối luật / ngân sách chu kỳ [%]", {c: 100.0 * val("law_us", c) / budget[c] for c in CTRLS}),
                ("CPU cả tiến trình [% 1 lõi]", {c: val("cpu_pct", c) for c in CTRLS})]
        L += ["## Chi phí tính toán", "",
              "| đại lượng | HAC | Fuzzy | Fuzzy / HAC |", "|---|---|---|---|"]
        for name, v in rows:
            ratio = v["fuzzy"] / v["hac"] if np.isfinite(v["hac"]) and v["hac"] > 0 else math.nan
            L.append(f"| {name} | {fmt(v['hac'])} | {fmt(v['fuzzy'])} | "
                     + (f"**{ratio:.3g}×**" if np.isfinite(ratio) else "—") + " |")
        L += ["",
              "- **Khối luật** = vòng từng khớp trong `onTimer`, đo bằng `steady_clock` trong node. Bên HAC khối "
              "này còn gồm bù ma sát (`tanh`) mà Fuzzy không có — tức là đo thiệt cho HAC.",
              "- **Chu kỳ tính** = từ sau watchdog tới khi publish lệnh: Ruckig + Pinocchio (hai bộ như nhau) + "
              "khối luật. Phần chung càng lớn thì tỉ lệ ở dòng này càng gần 1 — đó là thật, không phải lỗi đo.",
              "- **CPU cả tiến trình** (`/proc/<pid>/stat`) gồm cả DDS và publish debug; HAC phát 7 topic debug, "
              "Fuzzy 5.",
              "- Build hiện tại không bật tối ưu (`-O0`). Chi phí riêng của luật ở `-O0` và `-O2`, không nhiễu "
              "ROS: `./rx150.sh ab-bench`.", ""]

    L += ["## Theo khớp (trung bình giữa các lượt)", "",
          "| khớp | RMSE bám HAC / Fuzzy [°] | xác lập HAC / Fuzzy [°] | settle HAC / Fuzzy [s] "
          "| RMS PWM phản hồi HAC / Fuzzy |", "|---|---|---|---|---|"]
    for j in ARM:
        g = {c: pj[(pj.ctrl == c) & (pj.joint == j)] for c in CTRLS}
        if any(v.empty for v in g.values()):
            continue
        cell = lambda col: " / ".join(fmt(float(g[c][f"{col}_mean"].iloc[0])) for c in CTRLS)  # noqa: E731
        L.append(f"| {j} | {cell('rmse_e')} | {cell('ss_err')} | {cell('settle')} | {cell('rms_u_fb')} |")
    L += ["", "## Hình", "", "![tracking](fig_tracking.png)", "", "![trials](fig_trials.png)", "",
          "![joints](fig_joints.png)", "",
          "## Đọc số thế nào", "",
          "- `e` là sai số so với quỹ đạo Ruckig (ref), không phải so với đích. RMSE bám thấp mà "
          "sai số xác lập cao nghĩa là bám tốt khi chạy nhưng thiếu lực ở cuối (ma sát / bù trọng lực).",
          "- `RMS PWM phản hồi` = u − u_grav: phần do luật điều khiển tạo ra. So effort nên dùng cột "
          "này; PWM tổng bị bù trọng lực (giống nhau ở hai bộ) chi phối.",
          "- `Biến thiên PWM` cao kèm dao động nhỏ quanh đích = rung/limit-cycle.",
          "- Dữ liệu ở debug_publish_rate (thường 50 Hz): overshoot và max |e| có thể thấp hơn thực "
          "tế một chút vì đỉnh ngắn hơn 20 ms bị bỏ lỡ, như nhau ở cả hai bộ.",
          f"- File thô: `segments.csv` (từng segment × khớp), `trials.csv`, `summary.csv`, `per_joint.csv`."]
    with open(path, "w") as fh:
        fh.write("\n".join(L) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session_dir", help="tuning_runs/ab_<session>")
    p.add_argument("--out", default=None, help="mặc định <session_dir>/report")
    p.add_argument("--drop-first", type=int, default=0, help="bỏ N lượt đầu mỗi bộ (khởi động / nhiệt)")
    p.add_argument("--band-deg", type=float, default=0.5, help="băng xác lập tối thiểu [°]")
    p.add_argument("--ss-window", type=float, default=0.5, help="cửa sổ cuối segment để tính sai số xác lập [s]")
    p.add_argument("--min-move", type=float, default=math.radians(2.0), help="|Δq| tối thiểu [rad]")
    args = p.parse_args()

    # khớp không chạy trong segment -> cột settle/overshoot toàn NaN là CHỦ Ý, không cần cảnh báo
    np.seterr(invalid="ignore")
    import warnings
    warnings.filterwarnings("ignore", category=RuntimeWarning, message=".*(All-NaN|Mean of empty|invalid value).*")

    root = os.path.abspath(args.session_dir)
    out = os.path.abspath(args.out or os.path.join(root, "report"))
    runs, params = load_session(root)
    if args.drop_first:
        for c in CTRLS:
            firsts = sorted(r["trial"] for r in runs if r["ctrl"] == c)[:args.drop_first]
            runs = [r for r in runs if not (r["ctrl"] == c and r["trial"] in firsts)]
    have = {c for c in CTRLS if any(r["ctrl"] == c for r in runs)}
    if not runs:
        sys.exit(f"không có lượt nào trong {root}/{{hac,fuzzy}}/trial_*")
    if have != set(CTRLS):
        print(f"⚠ chỉ có dữ liệu của {', '.join(sorted(have))} — báo cáo sẽ thiếu một vế.")

    task_names = sorted({r["meta"]["task"]["name"] for r in runs})
    sigs = {json.dumps([s["q"] for s in r["meta"]["segments"]]) for r in runs}
    if len(sigs) > 1:
        sys.exit(f"✗ các lượt KHÔNG cùng một chuỗi waypoint (task: {task_names}) — không so sánh được. "
                 "Mỗi task một --session.")

    os.makedirs(out, exist_ok=True)
    seg_rows = []
    for r in runs:
        u_max = params.get(r["ctrl"], {}).get("u_max")
        seg_rows += segment_metrics(r, u_max, args)
    seg = pd.DataFrame(seg_rows)
    trials = pd.DataFrame([trial_metrics(r, seg) for r in runs]).sort_values(["ctrl", "trial"])
    summary = compare(trials)
    agg = seg.groupby(["ctrl", "trial", "joint"])[["rmse_e", "ss_err", "settle", "rms_u_fb"]].mean().reset_index()
    pj = agg.groupby(["ctrl", "joint"]).agg(["mean", "std"])
    pj.columns = [f"{a}_{b}" for a, b in pj.columns]
    pj = pj.drop(columns=[c for c in pj.columns if c.startswith("trial_")]).reset_index()
    diffs, same = fairness(params)

    seg.to_csv(os.path.join(out, "segments.csv"), index=False, float_format="%.6g")
    trials.to_csv(os.path.join(out, "trials.csv"), index=False, float_format="%.6g")
    summary.to_csv(os.path.join(out, "summary.csv"), index=False, float_format="%.6g")
    pj.to_csv(os.path.join(out, "per_joint.csv"), index=False, float_format="%.6g")
    plot_tracking(runs, trials, os.path.join(out, "fig_tracking.png"))
    plot_trials(trials, os.path.join(out, "fig_trials.png"))
    plot_joints(pj, os.path.join(out, "fig_joints.png"))
    write_md(os.path.join(out, "report.md"), root, runs, summary, pj, diffs, same, task_names, args, params)

    # ── tóm tắt ra terminal ──
    print(f"\n{'chỉ số':<28}{'HAC':>18}{'Fuzzy':>18}{'Δ%':>9}{'p':>8}  tốt hơn")
    for _, r in summary.iterrows():
        name = r["name"] + (f" [{r.unit}]" if r.unit else "")
        print(f"{name:<28}{fmt(r.hac_mean) + ' ± ' + fmt(r.hac_std, 2):>18}"
              f"{fmt(r.fuzzy_mean) + ' ± ' + fmt(r.fuzzy_std, 2):>18}{fmt(r.delta_pct, 3):>9}"
              f"{fmt(r.p_welch, 2):>8}  {r.better}")
    if diffs:
        print("\n⚠ THAM SỐ KHÁC NHAU giữa hai bộ (xem report.md):")
        for k, a, b in diffs:
            print(f"   {k}: HAC={a}  Fuzzy={b}")
    print(f"\n-> {out}/report.md")


if __name__ == "__main__":
    main()
