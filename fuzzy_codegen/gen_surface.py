#!/usr/bin/env python3
"""Evaluate a .fis over a 2D (e, ed) grid and emit surface.json for the 3D
control-surface artifact. Evaluates the FIS natively in Python (source of
truth = the .fis), so it does not depend on the generated C.

Usage:
    python3 gen_surface.py [fuzzy_type1.fis] [-o surface.json] [-N 41]
"""
import argparse
import json
import math
import os

import fis2c  # reuse the parser

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FIS = os.path.join(HERE, 'fuzzy_type1.fis')
DEFAULT_OUT = os.path.join(HERE, 'surface.json')
N = 41              # grid resolution per axis (default, override with -N)
DEFUZZ_N = 201      # centroid discretization (matches generated C)


def mf_eval(mf, x):
    t = mf.mtype
    p = mf.params
    if t == 'gaussmf':
        s, c = p
        if s == 0:
            return 1.0 if x == c else 0.0
        d = (x - c) / s
        return math.exp(-0.5 * d * d)
    if t == 'zmf':
        a, b = p
        if a == b:
            return 1.0 if x <= a else 0.0
        if x <= a:
            return 1.0
        if x >= b:
            return 0.0
        m = 0.5 * (a + b)
        if x <= m:
            tt = (x - a) / (b - a)
            return 1.0 - 2.0 * tt * tt
        tt = (b - x) / (b - a)
        return 2.0 * tt * tt
    if t == 'smf':
        a, b = p
        if a == b:
            return 1.0 if x >= a else 0.0
        if x <= a:
            return 0.0
        if x >= b:
            return 1.0
        m = 0.5 * (a + b)
        if x <= m:
            tt = (x - a) / (b - a)
            return 2.0 * tt * tt
        tt = (b - x) / (b - a)
        return 1.0 - 2.0 * tt * tt
    if t == 'trimf':
        a, b, c = p
        if x <= a or x >= c:
            return 0.0
        if x <= b:
            return 1.0 if b == a else (x - a) / (b - a)
        return 1.0 if c == b else (c - x) / (c - b)
    if t == 'trapmf':
        a, b, c, d = p
        if x <= a or x >= d:
            return 0.0
        if b <= x <= c:
            return 1.0
        if x < b:
            return 1.0 if b == a else (x - a) / (b - a)
        return 1.0 if d == c else (d - x) / (d - c)
    if t == 'gbellmf':
        a, b, c = p
        if a == 0:
            return 1.0 if x == c else 0.0
        tt = abs((x - c) / a)
        return 1.0 / (1.0 + tt ** (2.0 * b))
    if t == 'pimf':
        a, b, c, d = p
        return min(mf_eval_z(('smf', [a, b]), x), mf_eval_z(('zmf', [c, d]), x))
    if t == 'sigmf':
        a, c = p
        return 1.0 / (1.0 + math.exp(-a * (x - c)))
    if t == 'constant':
        return p[0]
    return 0.0


def t_norm(a, b, method):
    return min(a, b) if method in ('min', 'minimum') else a * b


def s_norm(a, b, method):
    if method in ('max', 'maximum'):
        return max(a, b)
    return a + b - a * b  # probor


def eval_fis(fis, in_vals):
    mu_in = [[mf_eval(m, in_vals[i]) for m in v.mfs]
             for i, v in enumerate(fis.inputs)]
    firing = []
    for r in fis.rules:
        f = 1.0
        seen = False
        for ii, idx in enumerate(r.in_idx):
            if idx != 0:
                val = mu_in[ii][idx - 1]
                if not seen:
                    f, seen = val, True
                elif r.conn == 2:
                    f = s_norm(f, val, fis.or_m)
                else:
                    f = t_norm(f, val, fis.and_m)
        firing.append((f if seen else 1.0) * r.weight)
    out = []
    for o, ov in enumerate(fis.outputs):
        lo, hi = ov.range
        num = den = 0.0
        for k in range(DEFUZZ_N):
            y = lo + (hi - lo) * k / (DEFUZZ_N - 1)
            agg = 0.0
            for ri, r in enumerate(fis.rules):
                oi = r.out_idx[o] if o < len(r.out_idx) else 0
                if oi == 0:
                    continue
                muo = mf_eval(ov.mfs[oi - 1], y)
                imp = min(firing[ri], muo) if fis.imp_m in ('min', 'minimum') else firing[ri] * muo
                agg = s_norm(agg, imp, fis.agg_m)
            num += y * agg
            den += agg
        out.append(num / den if den > 1e-9 else 0.0)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Sample a .fis into surface.json.")
    ap.add_argument('fis', nargs='?', default=DEFAULT_FIS,
                    help="path to the .fis (default: %s)" % os.path.basename(DEFAULT_FIS))
    ap.add_argument('-o', '--out', default=DEFAULT_OUT,
                    help="output json (default: %s)" % os.path.basename(DEFAULT_OUT))
    ap.add_argument('-N', type=int, default=N,
                    help="grid resolution per axis (default: %d)" % N)
    args = ap.parse_args(argv)
    N_grid, OUT_PATH = args.N, args.out

    fis = fis2c.parse_fis(args.fis)
    e_lo, e_hi = fis.inputs[0].range
    ed_lo, ed_hi = fis.inputs[1].range
    es = [e_lo + (e_hi - e_lo) * i / (N_grid - 1) for i in range(N_grid)]
    eds = [ed_lo + (ed_hi - ed_lo) * i / (N_grid - 1) for i in range(N_grid)]
    Z = []
    zmin = zmax = 0.0
    for j, ed in enumerate(eds):
        row = []
        for i, e in enumerate(es):
            # eval_core expects inputs in FIS order: [e, ed]
            val = eval_fis(fis, [e, ed])[0]
            row.append(val)
            zmin = min(zmin, val)
            zmax = max(zmax, val)
        Z.append(row)
    data = {
        'name': fis.name,
        'xName': fis.inputs[0].name or 'e',
        'yName': fis.inputs[1].name or 'ed',
        'zName': fis.outputs[0].name or 'output',
        'xRange': [e_lo, e_hi],
        'yRange': [ed_lo, ed_hi],
        'zRange': [zmin, zmax],
        'axis': es,    # x ticks (e)
        'axisY': eds,  # y ticks (ed)
        'grid': Z,     # grid[j][i] = Z at (e_i, ed_j); j indexes ed, i indexes e
    }
    with open(OUT_PATH, 'w') as f:
        json.dump(data, f)
    print('surface: %dx%d, z in [%.4f, %.4f], wrote %s'
          % (N_grid, N_grid, zmin, zmax, OUT_PATH))
    # quick sanity prints
    print('f(0,0)        = % .5f' % eval_fis(fis, [0.0, 0.0])[0])
    print('f(-1,-1)      = % .5f' % eval_fis(fis, [-1.0, -1.0])[0])
    print('f( 1, 1)      = % .5f' % eval_fis(fis, [1.0, 1.0])[0])
    print('f( 0, 1)      = % .5f' % eval_fis(fis, [0.0, 1.0])[0])
    print('f( 1, 0)      = % .5f' % eval_fis(fis, [1.0, 0.0])[0])


if __name__ == '__main__':
    main()
