#!/usr/bin/env python3
"""Smoke test: hri_common.py helpers thuần (pitch↔quat, status parse/build).

Chạy bằng: python3 module_tests/run_test.py perception/hri_helpers_test.py
"""
import math
import sys
from pathlib import Path

# import hri_common từ package (cần source env)
try:
    # Cố gắng import sau khi source install/setup.bash
    from rx150_hri.scripts import hri_common
except ImportError as exc:
    # Fallback: import trực tiếp từ đường dẫn tương đối (cho case chưa source)
    root = (Path(__file__).resolve().parents[2]
            / 'src' / 'rx150' / 'apps' / 'rx150_hri' / 'scripts')
    if not (root / 'hri_common.py').exists():
        print(f'SKIP: không tìm thấy hri_common.py tại {root}', file=sys.stderr)
        print('  (thường do chưa source ~/interbotix_ws/install/setup.bash)', file=sys.stderr)
        sys.exit(0)   # SKIP (không phải FAIL) trong CI khi env chưa setup
    import importlib.util
    spec = importlib.util.spec_from_file_location('hri_common', root / 'hri_common.py')
    hri_common = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hri_common)


def test_pitch_quat_roundtrip() -> bool:
    """pitch → quat → pitch phải khớp (tolerance 1e-6)."""
    cases = [0.0, 0.5, -0.5, 0.8, -0.8, 1.0, -1.0]
    for p in cases:
        qx, qy, qz, qw = hri_common.pitch_to_quat(p)
        p2 = hri_common.quat_to_pitch(qx, qy, qz, qw)
        # qy = sin(pitch/2), qw = cos(pitch/2); quat_to_pitch = 2·atan2(qy, qw)
        # Rắc rối quanh ±π (do atan2) → normalize về [-π,π]
        # Lưu ý: roundtrip trong domain [-π/2, π/2] là chính xác.
        p2 = (p2 + math.pi) % (2 * math.pi) - math.pi   # normalize
        if abs(p2 - p) > 1e-6:
            print(f'FAIL: pitch {p} → quat ({qx},{qy},{qz},{qw}) → pitch {p2}')
            return False
    print('OK: pitch_quat_roundtrip')
    return True


def test_status_parse_build() -> bool:
    """build_status / parse_status phải đối xứng."""
    cases = [
        (hri_common.POSE_DONE, 3),
        (hri_common.POSE_FAILED, 5),
        (hri_common.READY, 0),
        (hri_common.IDLE, 12),
    ]
    for kind, seq in cases:
        text = hri_common.build_status(kind, seq)
        kind2, seq2 = hri_common.parse_status(text)
        if kind != kind2 or seq != seq2:
            print(f'FAIL: build_status({kind},{seq}) = "{text}" → parse=({kind2},{seq2})')
            return False
    print('OK: status_parse_build')
    return True


def test_status_parse_malformed() -> bool:
    """parse_status trả None/-1 cho chuỗi sai định dạng."""
    malformed = ['abc', 'POSE_DONE', '#3', 'POSE_DONE #abc', '']
    for s in malformed:
        kind, seq = hri_common.parse_status(s)
        if kind is not None or seq != -1:
            print(f'FAIL: parse_status("{s}") = ({kind},{seq}) mong (None,-1)')
            return False
    print('OK: status_parse_malformed')
    return True


def main() -> int:
    ok = True
    ok &= test_pitch_quat_roundtrip()
    ok &= test_status_parse_build()
    ok &= test_status_parse_malformed()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
