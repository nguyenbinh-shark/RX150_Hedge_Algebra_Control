#!/usr/bin/env python3
"""Report-quality figures (English labels) comparing HAC vs Fuzzy on the same reference.

Usage: ab_paper_figs.py HAC_SESSION_DIR FUZZY_SESSION_DIR OUT_DIR [--t0 16 --t1 36.5]
Example: ab_paper_figs.py tuning_runs/ab_slow3 tuning_runs/ab_slow1 tuning_runs/ab_slow3/figs_en
"""
import argparse, glob, json, os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter

JOINTS = ["waist", "shoulder", "elbow", "wrist_angle"]
JNAME = {"waist": "Waist", "shoulder": "Shoulder", "elbow": "Elbow", "wrist_angle": "Wrist pitch"}
COL = {"hac": "#1F6FB4", "fuzzy": "#C4382E", "ref": "#222222"}
LBL = {"hac": "HAC", "fuzzy": "Fuzzy"}
DT = 0.02

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10, "axes.titlesize": 11, "axes.labelsize": 10,
    "legend.fontsize": 9, "xtick.labelsize": 9, "ytick.labelsize": 9,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
    "grid.color": "#dddddd", "grid.linewidth": 0.6, "savefig.dpi": 220, "savefig.bbox": "tight",
})


def load(session, ctrl):
    trials = []
    for d in sorted(glob.glob(f"{session}/{ctrl}/trial_*")):
        df = pd.read_csv(f"{d}/data.csv")
        meta = json.load(open(f"{d}/meta.json"))
        trials.append((df, meta))
    if not trials:
        raise SystemExit(f"no trials in {session}/{ctrl}")
    return trials


def resample(trials, col, tgrid):
    return np.vstack([np.interp(tgrid, df.t.to_numpy(), df[col].to_numpy()) for df, _ in trials])


def seg_bounds(meta):
    return [(s["label"], s["t_start"], s["t_end"], s.get("move", 0.0)) for s in meta["segments"]]


def shade_segments(ax, segs, t0=None, t1=None, labels=False):
    for k, (lab, ts, te, mv) in enumerate(segs):
        if t1 is not None and (te < t0 or ts > t1):
            continue
        ax.axvspan(ts + mv, te, color="#000000", alpha=0.045, lw=0)   # hold phase
        ax.axvline(ts, color="#999999", lw=0.6, ls=":")
        if labels:
            xm = (max(ts, t0) + min(te, t1)) / 2 if t1 is not None else (ts + te) / 2
            ax.text(xm, 1.02, lab.replace("_", " "), transform=ax.get_xaxis_transform(),
                    ha="center", va="bottom", fontsize=8, color="#555555")


def band(ax, t, arr, ctrl, lw=1.4, label=True):
    m = arr.mean(0)
    ax.fill_between(t, arr.min(0), arr.max(0), color=COL[ctrl], alpha=0.18, lw=0)
    ax.plot(t, m, color=COL[ctrl], lw=lw, label=LBL[ctrl] if label else None)


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hac"); ap.add_argument("fuzzy"); ap.add_argument("out")
    ap.add_argument("--t0", type=float, default=16.0); ap.add_argument("--t1", type=float, default=36.5)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    data = {"hac": load(a.hac, "hac"), "fuzzy": load(a.fuzzy, "fuzzy")}
    tend = min(min(df.t.max() for df, _ in tr) for tr in data.values())
    tg = np.arange(0.0, tend, DT)
    segs = seg_bounds(data["hac"][0][1])
    R = {c: {} for c in data}
    for c, tr in data.items():
        for j in JOINTS:
            for q in ("pos", "ref_pos", "err", "pwm"):
                R[c][f"{j}_{q}"] = resample(tr, f"{j}_{q}", tg)
        for q in ("law_mean_ns",):
            R[c][q] = np.concatenate([df[q].dropna().to_numpy() for df, _ in tr])
    ref_diff = max(np.degrees(np.abs(R["hac"][f"{j}_ref_pos"].mean(0) - R["fuzzy"][f"{j}_ref_pos"].mean(0))).max()
                   for j in JOINTS)
    deg = np.degrees
    written = []

    def save(fig, name):
        for ext in ("png", "pdf"):
            fig.savefig(f"{a.out}/{name}.{ext}")
        plt.close(fig); written.append(name)

    # ---------- Fig 1: full-cycle tracking error overview ----------
    fig, axs = plt.subplots(len(JOINTS), 1, figsize=(10, 7.2), sharex=True)
    for ax, j in zip(axs, JOINTS):
        shade_segments(ax, segs, labels=(j == JOINTS[0]))
        for c in ("fuzzy", "hac"):
            band(ax, tg, deg(np.abs(R[c][f"{j}_err"])), c, lw=1.1)
        ax.set_ylabel(f"{JNAME[j]}\n|e| (deg)")
        ax.set_ylim(bottom=0)
        e_h, e_f = rms(deg(R["hac"][f"{j}_err"])), rms(deg(R["fuzzy"][f"{j}_err"]))
        ax.text(0.995, 0.93, f"RMSE  HAC {e_h:.2f}°   Fuzzy {e_f:.2f}°", transform=ax.transAxes,
                ha="right", va="top", fontsize=8.5,
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#cccccc", lw=0.6))
    for ax in axs:
        ax.axvspan(a.t0, a.t1, ymin=0, ymax=0.03, color="#B0761A", alpha=0.9, lw=0)
    axs[-1].set_xlabel("Time (s)")
    axs[-1].set_xlim(0, tend)
    axs[0].legend(loc="upper left", ncol=2, frameon=True)
    fig.suptitle("Absolute tracking error over the full slow pick-and-place cycle "
                 "(mean of 3 trials, band = min–max; grey = hold phase)", fontsize=11, y=0.955)
    fig.align_ylabels(axs)
    save(fig, "fig1_error_full_cycle")

    # ---------- Fig 2: characteristic window, position + error per joint ----------
    w = (tg >= a.t0) & (tg <= a.t1)
    tw = tg[w]
    fig, axs = plt.subplots(len(JOINTS), 2, figsize=(11, 8.2), sharex=True,
                            gridspec_kw={"width_ratios": [1, 1], "wspace": 0.22})
    for r, j in enumerate(JOINTS):
        axp, axe = axs[r]
        for ax in (axp, axe):
            shade_segments(ax, segs, a.t0, a.t1, labels=(r == 0))
        axp.plot(tw, deg(R["hac"][f"{j}_ref_pos"].mean(0)[w]), color=COL["ref"], lw=1.3, ls="--", label="Reference")
        for c in ("fuzzy", "hac"):
            axp.plot(tw, deg(R[c][f"{j}_pos"].mean(0)[w]), color=COL[c], lw=1.3, label=LBL[c])
            band(axe, tw, deg(R[c][f"{j}_err"][:, w]), c, lw=1.3)
        axe.axhline(0, color="#444444", lw=0.7)
        axp.set_ylabel(f"{JNAME[j]}\nangle (deg)")
        axe.set_ylabel("error (deg)")
        lo, hi = axe.get_ylim(); axe.set_ylim(lo, hi + 0.35 * (hi - lo))
        e_h, e_f = rms(deg(R["hac"][f"{j}_err"][:, w])), rms(deg(R["fuzzy"][f"{j}_err"][:, w]))
        axe.text(0.01, 0.96, f"RMSE  HAC {e_h:.2f}°  |  Fuzzy {e_f:.2f}°", transform=axe.transAxes,
                 ha="left", va="top", fontsize=8.5,
                 bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#cccccc", lw=0.6))
    axs[0, 0].legend(loc="lower right", frameon=True)
    axs[0, 0].set_title("Joint position", pad=16)
    axs[0, 1].set_title("Tracking error", pad=16)
    for ax in axs[-1]:
        ax.set_xlabel("Time (s)"); ax.set_xlim(a.t0, a.t1)
    fig.suptitle(f"Characteristic window {a.t0:g}–{a.t1:g} s: move above left bin → lower → raise\n"
                 "(error: mean of 3 trials, band = min–max; grey = hold phase)", fontsize=11.5, y=0.975)
    fig.align_ylabels(axs[:, 0]); fig.align_ylabels(axs[:, 1])
    save(fig, "fig2_window_position_error")

    # ---------- Fig 3: close-up of one move + hold (shoulder lowering), with control effort ----------
    seg = next(s for s in segs if s[0] == "down_L")
    z0, z1 = seg[1] - 0.5, seg[2]
    wz = (tg >= z0) & (tg <= z1)
    tz = tg[wz]
    fig, axs = plt.subplots(3, 1, figsize=(8.5, 7.4), sharex=True, gridspec_kw={"height_ratios": [1.3, 1, 1]})
    j = "shoulder"
    for ax in axs:
        ax.axvspan(seg[1] + seg[3], z1, color="#000000", alpha=0.045, lw=0)
        ax.axvline(seg[1] + seg[3], color="#999999", lw=0.6, ls=":")
    axs[0].plot(tz, deg(R["hac"][f"{j}_ref_pos"].mean(0)[wz]), color=COL["ref"], ls="--", lw=1.4, label="Reference")
    for c in ("fuzzy", "hac"):
        axs[0].plot(tz, deg(R[c][f"{j}_pos"].mean(0)[wz]), color=COL[c], lw=1.5, label=LBL[c])
        band(axs[1], tz, deg(R[c][f"{j}_err"][:, wz]), c, lw=1.5)
        band(axs[2], tz, R[c][f"{j}_pwm"][:, wz], c, lw=1.2)
    axs[1].axhline(0, color="#444444", lw=0.7)
    # steady-state annotation: mean error over last 1.5 s of the hold
    hs = (tz >= z1 - 1.5)
    for c, va in (("fuzzy", "bottom"), ("hac", "top")):
        ess = deg(R[c][f"{j}_err"][:, wz].mean(0)[hs]).mean()
        axs[1].annotate(f"{LBL[c]} steady-state ≈ {ess:+.2f}°", xy=(z1 - 1.2, ess),
                        xytext=(0, 14 if va == "bottom" else -14), textcoords="offset points",
                        ha="right", va=va, fontsize=9, color=COL[c], fontweight="bold")
    axs[0].set_ylabel("Shoulder angle (deg)")
    axs[1].set_ylabel("Tracking error (deg)")
    axs[2].set_ylabel("Command (PWM)")
    axs[0].legend(loc="lower right")
    axs[1].legend(loc="upper left")
    ymin, ymax = axs[0].get_ylim()
    axs[0].text(seg[1] + seg[3] / 2, 1.01, "move (3 s quintic)", transform=axs[0].get_xaxis_transform(),
                ha="center", va="bottom", fontsize=9, color="#555555")
    axs[0].text((seg[1] + seg[3] + z1) / 2, 1.01, "hold", transform=axs[0].get_xaxis_transform(),
                ha="center", va="bottom", fontsize=9, color="#555555")
    axs[-1].set_xlabel("Time (s)"); axs[-1].set_xlim(z0, z1)
    fig.suptitle("Close-up: shoulder lowering 0.35 rad and holding under gravity load\n"
                 "(mean of 3 trials, band = min–max)", fontsize=11.5, y=0.975)
    fig.align_ylabels(axs)
    save(fig, "fig3_shoulder_closeup")

    # ---------- Fig 4: metrics summary (per trial -> mean ± std) ----------
    def trial_metrics(trials):
        out = {j: {"rmse": [], "ss": []} for j in JOINTS}
        for df, meta in trials:
            for j in JOINTS:
                e = deg(df[f"{j}_err"].to_numpy())
                out[j]["rmse"].append(rms(e))
                ss = []
                for lab, ts, te, mv in seg_bounds(meta):
                    m = (df.t >= te - 1.5) & (df.t <= te)
                    ss.append(np.abs(e[m.to_numpy()]).mean())
                out[j]["ss"].append(np.mean(ss))
        return out

    M = {c: trial_metrics(tr) for c, tr in data.items()}
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.9), gridspec_kw={"width_ratios": [1.25, 1.25, 0.8], "wspace": 0.35})
    x = np.arange(len(JOINTS)); bw = 0.36
    for ax, key, title in ((axs[0], "rmse", "Tracking RMSE per joint"),
                           (axs[1], "ss", "Steady-state error per joint\n(|e| averaged over last 1.5 s of each hold)")):
        for k, c in enumerate(("hac", "fuzzy")):
            mu = [np.mean(M[c][j][key]) for j in JOINTS]; sd = [np.std(M[c][j][key], ddof=1) for j in JOINTS]
            bars = ax.bar(x + (k - 0.5) * bw, mu, bw, yerr=sd, capsize=3, color=COL[c], label=LBL[c],
                          error_kw=dict(lw=0.8, ecolor="#333333"))
            for b, v, s in zip(bars, mu, sd):
                ax.annotate(f"{v:.2f}", (b.get_x() + b.get_width() / 2, v + s),
                            xytext=(0, 3), textcoords="offset points", ha="center", fontsize=7.5)
        ax.set_xticks(x, [JNAME[j] for j in JOINTS]); ax.set_ylabel("deg"); ax.set_title(title)
        ax.grid(axis="x", visible=False); ax.margins(y=0.15)
    axs[0].legend(loc="upper right")
    # computation time of the control law (log scale)
    lm = {c: R[c]["law_mean_ns"] / 1e3 for c in data}
    vals = [np.mean(lm["hac"]), np.mean(lm["fuzzy"])]
    bars = axs[2].bar([0, 1], vals, 0.55, color=[COL["hac"], COL["fuzzy"]])
    axs[2].set_yscale("log"); axs[2].set_ylim(0.5, 1000)
    axs[2].yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
    for b, v in zip(bars, vals):
        axs[2].annotate(f"{v:.2f} µs" if v < 10 else f"{v:.0f} µs", (b.get_x() + b.get_width() / 2, v),
                        xytext=(0, 4), textcoords="offset points", ha="center", fontsize=9, fontweight="bold")
    axs[2].set_xticks([0, 1], ["HAC", "Fuzzy"]); axs[2].set_ylabel("µs per cycle (log)")
    axs[2].set_title("Control-law compute time\n(5 joints, mean per 400 Hz cycle)")
    axs[2].grid(axis="x", visible=False)
    fig.suptitle("Full-cycle summary over 3 trials each (error bars = ±1 std)", fontsize=11.5, y=1.04)
    save(fig, "fig4_metrics_summary")

    print(f"reference max difference HAC vs Fuzzy: {ref_diff:.3f} deg")
    for c in data:
        print(c, {j: (round(np.mean(M[c][j]['rmse']), 3), round(np.mean(M[c][j]['ss']), 3)) for j in JOINTS},
              f"law {np.mean(lm[c]):.2f} us")
    print("written:", ", ".join(f"{a.out}/{n}.png|pdf" for n in written))


if __name__ == "__main__":
    main()
