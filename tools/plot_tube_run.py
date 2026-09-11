#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""plot_tube_run.py — vẽ đồ thị từ thư mục do record_tube_run.py sinh ra.

    python3 tools/plot_tube_run.py tuning_runs/run1_20260911_210000

Sinh ra trong chính thư mục đó:
    traj3d.png       quỹ đạo 3D: ref / enc / tag + 4 lỗ giá + ống nhận diện
    traj_2d.png      cùng dữ liệu, chiếu XY (nhìn từ trên) và XZ (nhìn ngang)
    error_time.png   sai số theo thời gian, tô nền theo pha
    rms.png          RMS theo trục và theo khớp
    joints.png       ref vs enc từng khớp
    detections.png   toạ độ ống theo thời gian + tản mát XY
    summary.txt      bảng số (RMS, min/max, đếm mẫu)

HAI sai số KHÁC NHAU, đừng gộp:
    enc - ref   sai số BÁM        lỗi của bộ điều khiển
    tag - enc   sai số HIỆU CHUẨN encoder tưởng vs camera thấy
Cộng chúng lại là vô nghĩa; chúng có nguyên nhân và cách sửa khác hẳn nhau.
"""
import csv, json, math, os, sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

CLS_COLOR = {'pink': '#e83e8c', 'blue': '#2f6fed', 'green': '#1faa59',
             'yellow': '#e0a800', '': '#888888'}
C_REF, C_ENC, C_TAG = '#1f77b4', '#d62728', '#2ca02c'
ARM = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']


def read_csv(path):
    with open(path, encoding='utf-8') as fh:
        rows = list(csv.DictReader(fh))
    return rows


def col(rows, key):
    out = []
    for r in rows:
        v = r.get(key, '')
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            out.append(float('nan'))
    return np.array(out)


def finite(*arrs):
    """Mặt nạ các mẫu mà MỌI mảng đều hữu hạn."""
    m = np.ones(len(arrs[0]), dtype=bool)
    for a in arrs:
        m &= np.isfinite(a)
    return m


def rms(v):
    v = v[np.isfinite(v)]
    return float(np.sqrt(np.mean(v ** 2))) if len(v) else float('nan')


def phase_spans(t, phase):
    """[(t0, t1, tên pha)] các đoạn pha liên tiếp."""
    spans, i = [], 0
    while i < len(phase):
        j = i
        while j + 1 < len(phase) and phase[j + 1] == phase[i]:
            j += 1
        if phase[i]:
            spans.append((t[i], t[j], phase[i]))
        i = j + 1
    return spans


def shade(ax, spans, t):
    cmap = plt.get_cmap('tab20')
    names = sorted({s[2] for s in spans})
    for t0, t1, nm in spans:
        if t1 - t0 < 1e-6:
            continue
        ax.axvspan(t0, t1, color=cmap(names.index(nm) % 20), alpha=0.12, lw=0)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    d = sys.argv[1].rstrip('/')
    traj = read_csv(os.path.join(d, 'traj.csv'))
    dets = read_csv(os.path.join(d, 'det.csv'))
    with open(os.path.join(d, 'meta.json'), encoding='utf-8') as fh:
        meta = json.load(fh)

    if not traj:
        sys.exit('traj.csv rỗng — không có gì để vẽ.')

    t = col(traj, 't')
    phase = [r.get('phase', '') for r in traj]
    spans = phase_spans(t, phase)

    P = {}
    for src in ('ref', 'enc', 'tag'):
        P[src] = np.column_stack([col(traj, f'{src}_{ax}') for ax in 'xyz'])

    slots = np.array(meta.get('rack', {}).get('slots') or []).reshape(-1, 3) \
        if meta.get('rack', {}).get('slots') else np.zeros((0, 3))

    dx = col(dets, 'x'); dy = col(dets, 'y'); dz = col(dets, 'z')
    dt = col(dets, 't'); dcls = [r.get('cls', '') for r in dets]

    have = {k: bool(np.isfinite(P[k][:, 0]).any()) for k in P}
    lines = [(k, P[k], c, n) for k, c, n in
             (('ref', C_REF, 'ref (đặt)'), ('enc', C_ENC, 'enc (encoder)'),
              ('tag', C_TAG, 'tag (camera)')) if have[k]]

    # ── 1. quỹ đạo 3D ───────────────────────────────────────────────────
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    for k, A, c, nm in lines:
        m = np.isfinite(A[:, 0])
        ax.plot(A[m, 0], A[m, 1], A[m, 2], color=c, lw=1.4, label=nm,
                ls='--' if k == 'ref' else '-', alpha=0.9)
    if len(slots):
        ax.scatter(slots[:, 0], slots[:, 1], slots[:, 2], marker='s', s=90,
                   c='k', depthshade=False, label='lỗ giá')
        for i, s in enumerate(slots):
            ax.text(s[0], s[1], s[2] + 0.012, str(i), fontsize=9, ha='center')
    md = finite(dx, dy, dz)
    if md.any():
        for cl in sorted(set(dcls)):
            sel = md & np.array([c == cl for c in dcls])
            if sel.any():
                ax.scatter(dx[sel], dy[sel], dz[sel], marker='o', s=28,
                           c=CLS_COLOR.get(cl, '#888'), depthshade=False,
                           label=f'ống {cl or "?"}', alpha=0.65)
    ax.scatter([0], [0], [0], marker='^', s=70, c='k', label='base_link')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_zlabel('z [m]')
    ax.set_title(f"Quỹ đạo 3D — {meta.get('label', '')}")
    ax.legend(loc='upper left', fontsize=8)
    try:
        ax.set_box_aspect((1, 1, 0.6))
    except Exception:
        pass
    fig.tight_layout(); fig.savefig(os.path.join(d, 'traj3d.png'), dpi=130)
    plt.close(fig)

    # ── 2. chiếu XY + XZ ────────────────────────────────────────────────
    fig, axs = plt.subplots(1, 2, figsize=(13, 5.6))
    for aa, (i, j, li, lj, ttl) in zip(axs, [(0, 1, 'x', 'y', 'Nhìn từ TRÊN (XY)'),
                                             (0, 2, 'x', 'z', 'Nhìn NGANG (XZ)')]):
        for k, A, c, nm in lines:
            m = np.isfinite(A[:, i])
            aa.plot(A[m, i], A[m, j], color=c, lw=1.3, label=nm,
                    ls='--' if k == 'ref' else '-', alpha=0.9)
        if len(slots):
            aa.scatter(slots[:, i], slots[:, j], marker='s', s=80, c='k', label='lỗ giá')
            for n, s in enumerate(slots):
                aa.annotate(str(n), (s[i], s[j]), textcoords='offset points',
                            xytext=(5, 5), fontsize=9)
        if md.any():
            D = np.column_stack([dx, dy, dz])
            for cl in sorted(set(dcls)):
                sel = md & np.array([c == cl for c in dcls])
                if sel.any():
                    aa.scatter(D[sel, i], D[sel, j], s=26, alpha=0.6,
                               c=CLS_COLOR.get(cl, '#888'), label=f'ống {cl or "?"}')
        aa.scatter([0], [0], marker='^', s=70, c='k')
        aa.set_xlabel(f'{li} [m]'); aa.set_ylabel(f'{lj} [m]')
        aa.set_title(ttl); aa.grid(alpha=0.3); aa.axis('equal')
    axs[0].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(d, 'traj_2d.png'), dpi=130)
    plt.close(fig)

    # ── 3. sai số theo thời gian ────────────────────────────────────────
    pairs = []
    if have['ref'] and have['enc']:
        pairs.append(('enc − ref  (sai số BÁM)', P['enc'] - P['ref'], C_ENC))
    if have['tag'] and have['enc']:
        pairs.append(('tag − enc  (sai số HIỆU CHUẨN)', P['tag'] - P['enc'], C_TAG))

    if pairs:
        fig, axs = plt.subplots(len(pairs) + 1, 1, figsize=(12, 3.1 * (len(pairs) + 1)),
                                sharex=True)
        axs = np.atleast_1d(axs)
        for aa, (nm, E, c) in zip(axs, pairs):
            for k, lbl in enumerate('xyz'):
                aa.plot(t, E[:, k] * 1000, lw=0.9, label=f'Δ{lbl}')
            aa.plot(t, np.linalg.norm(E, axis=1) * 1000, lw=1.6, color='k',
                    label='‖Δ‖')
            shade(aa, spans, t)
            aa.set_ylabel('mm'); aa.set_title(nm, fontsize=10)
            aa.grid(alpha=0.3); aa.legend(fontsize=8, ncol=4)
        aa = axs[-1]
        for nm, E, c in pairs:
            aa.plot(t, np.linalg.norm(E, axis=1) * 1000, lw=1.4, label=nm)
        shade(aa, spans, t)
        aa.set_ylabel('‖Δ‖ [mm]'); aa.set_xlabel('t [s]')
        aa.set_title('So hai loại sai số', fontsize=10)
        aa.grid(alpha=0.3); aa.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(os.path.join(d, 'error_time.png'), dpi=130)
        plt.close(fig)

    # ── 4. RMS ──────────────────────────────────────────────────────────
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.6))
    labels, vals = [], []
    for nm, E, c in pairs:
        short = nm.split('(')[0].strip()
        for k, lbl in enumerate('xyz'):
            labels.append(f'{short}\nΔ{lbl}'); vals.append(rms(E[:, k]) * 1000)
        labels.append(f'{short}\n‖Δ‖'); vals.append(rms(np.linalg.norm(E, axis=1)) * 1000)
    if vals:
        b = axs[0].bar(range(len(vals)), vals,
                       color=[C_ENC] * 4 + [C_TAG] * 4 if len(vals) == 8 else None)
        axs[0].set_xticks(range(len(vals)))
        axs[0].set_xticklabels(labels, fontsize=7)
        axs[0].set_ylabel('RMS [mm]'); axs[0].set_title('RMS sai số vị trí ee')
        axs[0].grid(alpha=0.3, axis='y')
        for r, v in zip(b, vals):
            axs[0].text(r.get_x() + r.get_width() / 2, v, f'{v:.1f}',
                        ha='center', va='bottom', fontsize=7)
    else:
        axs[0].text(.5, .5, 'thiếu ref hoặc tag', ha='center', transform=axs[0].transAxes)

    jr = [rms(col(traj, f'err_{j}')) * 180 / math.pi for j in ARM]
    if np.isfinite(jr).any():
        b = axs[1].bar(range(5), jr, color='#6f42c1')
        axs[1].set_xticks(range(5)); axs[1].set_xticklabels(ARM, rotation=20, fontsize=8)
        axs[1].set_ylabel('RMS [°]'); axs[1].set_title('RMS sai số khớp (/hac/error)')
        axs[1].grid(alpha=0.3, axis='y')
        for r, v in zip(b, jr):
            axs[1].text(r.get_x() + r.get_width() / 2, v, f'{v:.2f}',
                        ha='center', va='bottom', fontsize=7)
    else:
        axs[1].text(.5, .5, 'không có /hac/error', ha='center', transform=axs[1].transAxes)
    fig.tight_layout(); fig.savefig(os.path.join(d, 'rms.png'), dpi=130)
    plt.close(fig)

    # ── 5. từng khớp ────────────────────────────────────────────────────
    fig, axs = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
    for i, j in enumerate(ARM):
        e = col(traj, f'enc_{j}') * 180 / math.pi
        r = col(traj, f'ref_{j}') * 180 / math.pi
        if np.isfinite(r).any():
            axs[i].plot(t, r, color=C_REF, ls='--', lw=1.2, label='ref')
        axs[i].plot(t, e, color=C_ENC, lw=1.0, label='enc')
        shade(axs[i], spans, t)
        axs[i].set_ylabel(f'{j}\n[°]', fontsize=8); axs[i].grid(alpha=0.3)
        if i == 0:
            axs[i].legend(fontsize=8, ncol=2)
    axs[-1].set_xlabel('t [s]')
    fig.suptitle('Khớp: ref vs encoder', y=0.995)
    fig.tight_layout(); fig.savefig(os.path.join(d, 'joints.png'), dpi=130)
    plt.close(fig)

    # ── 6. nhận diện ────────────────────────────────────────────────────
    if md.any():
        fig, axs = plt.subplots(1, 2, figsize=(13, 5))
        for k, lbl in zip(range(3), 'xyz'):
            axs[0].plot(dt[md], [dx, dy, dz][k][md], '.', ms=4, label=lbl)
        axs[0].set_xlabel('t [s]'); axs[0].set_ylabel('m'); axs[0].grid(alpha=0.3)
        axs[0].set_title('Toạ độ ống theo thời gian (ổn định = nhận diện tốt)')
        axs[0].legend(fontsize=8)

        for cl in sorted(set(dcls)):
            sel = md & np.array([c == cl for c in dcls])
            if sel.any():
                axs[1].scatter(dx[sel], dy[sel], s=30, alpha=0.6,
                               c=CLS_COLOR.get(cl, '#888'), label=f'ống {cl or "?"}')
        if len(slots):
            axs[1].scatter(slots[:, 0], slots[:, 1], marker='s', s=90, c='k',
                           label='lỗ giá')
            for n, s in enumerate(slots):
                axs[1].add_patch(plt.Circle((s[0], s[1]), 0.05, fill=False,
                                            ls=':', color='r', lw=1))
                axs[1].annotate(str(n), (s[0], s[1]), textcoords='offset points',
                                xytext=(6, 6), fontsize=9)
        axs[1].set_xlabel('x [m]'); axs[1].set_ylabel('y [m]'); axs[1].axis('equal')
        axs[1].grid(alpha=0.3); axs[1].legend(fontsize=8)
        axs[1].set_title('Tản mát XY — vòng đỏ = bán kính 5 cm bị coi "đã cắm"')
        fig.tight_layout(); fig.savefig(os.path.join(d, 'detections.png'), dpi=130)
        plt.close(fig)

    # ── 7. bảng số ──────────────────────────────────────────────────────
    L = []
    L.append(f"run          : {meta.get('label')}  ({meta.get('recorded_at')})")
    L.append(f"mẫu quỹ đạo  : {len(traj)} @ {meta.get('rate_hz')} Hz"
             f"  ({t[-1] - t[0]:.1f} s)")
    L.append(f"nguồn có mặt : " + ', '.join(k for k in ('ref', 'enc', 'tag') if have[k])
             + (''.join(f"  [THIẾU {k}]" for k in ('ref', 'enc', 'tag') if not have[k])))
    L.append('')
    for nm, E, c in pairs:
        n = np.linalg.norm(E, axis=1)
        n = n[np.isfinite(n)]
        L.append(f"{nm}")
        L.append(f"   RMS  x/y/z = {rms(E[:, 0])*1000:7.2f} /{rms(E[:, 1])*1000:7.2f} /"
                 f"{rms(E[:, 2])*1000:7.2f}  mm")
        L.append(f"   RMS  ‖Δ‖   = {rms(np.linalg.norm(E, axis=1))*1000:7.2f} mm"
                 f"   max = {n.max()*1000:7.2f} mm" if len(n) else "   (rỗng)")
        L.append('')
    if np.isfinite(jr).any():
        L.append('RMS sai số khớp (/hac/error):')
        for j, v in zip(ARM, jr):
            L.append(f'   {j:14s} {v:6.3f}°')
        L.append('')
    if md.any():
        named = sorted({c for c in dcls if c})
        blank = sum(1 for i, c in enumerate(dcls) if md[i] and not c)
        L.append(f'Nhận diện: {md.sum()} pose, {len(named)} nhãn '
                 f'({", ".join(named) or "—"})'
                 # '?' không phải một loại ống: là frame mà pose tới trước
                 # /yolo/tube_classes. Nhiều bất thường ⇒ hai topic lệch nhịp.
                 + (f'  [{blank} pose chưa kịp có nhãn]' if blank else ''))
        L.append(f'   tản mát x/y/z = {np.std(dx[md])*1000:.2f} / '
                 f'{np.std(dy[md])*1000:.2f} / {np.std(dz[md])*1000:.2f} mm')
        if len(slots):
            for cl in sorted(set(dcls)):
                sel = md & np.array([c == cl for c in dcls])
                if not sel.any():
                    continue
                cx, cy = dx[sel].mean(), dy[sel].mean()
                dd = np.hypot(slots[:, 0] - cx, slots[:, 1] - cy) * 1000
                near = int(np.argmin(dd))
                flag = '  ⚠️ < 50 mm ⇒ BỊ COI LÀ ĐÃ CẮM' if dd[near] < 50 else ''
                L.append(f'   ống {cl or "?":7s} ({cx:.3f},{cy:.3f})  '
                         f'gần nhất slot {near} = {dd[near]:.1f} mm{flag}')
    txt = '\n'.join(L)
    with open(os.path.join(d, 'summary.txt'), 'w', encoding='utf-8') as fh:
        fh.write(txt + '\n')
    print(txt)
    print(f"\n✅ Đồ thị đã ghi vào {d}/")


if __name__ == '__main__':
    main()
