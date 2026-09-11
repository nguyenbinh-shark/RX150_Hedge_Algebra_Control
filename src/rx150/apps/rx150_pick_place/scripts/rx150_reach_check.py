#!/usr/bin/env python3
"""
rx150_reach_check — kiểm tra TẦM VỚI của rx150 mà KHÔNG cần robot, camera hay MoveIt.

Đa số lần "pick and place chạy không được" của cánh tay 5-DoF không phải lỗi
điều khiển mà là điểm cần tới nằm ngoài bao hình khả thi: rx150 chỉ chúc thẳng
đứng (pitch 90°) được khi bán kính r ≲ 0.29 m; càng ra xa càng phải nghiêng.
Công cụ này in ra bảng đó, và chấm điểm luôn các toạ độ trong file config.

  ros2 run rx150_pick_place rx150_reach_check.py
  ros2 run rx150_pick_place rx150_reach_check.py --point 0.30 0.0 0.06 --pitch 90
  ros2 run rx150_pick_place rx150_reach_check.py \
      --config $(ros2 pkg prefix rx150_pick_place)/share/rx150_pick_place/config/\
tube_rack_params.yaml
"""
import argparse
import math
import sys

from rx150_modules.kinematics import Rx150Kinematics

RADII = [0.15, 0.18, 0.20, 0.22, 0.25, 0.28, 0.30, 0.33, 0.36, 0.40]
HEIGHTS = [0.02, 0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25]


def print_envelope(kin):
    print(f'\nPITCH LỚN NHẤT (độ, + = chúc xuống) — tầm với tối đa {kin.max_reach:.3f} m')
    print('  "-" = không với tới với bất kỳ pitch nào trong [−45°, 90°]\n')
    print('   z\\r ' + ''.join(f'{r:7.2f}' for r in RADII))
    for z in HEIGHTS:
        cells = []
        for r in RADII:
            best = kin.max_pitch(r, 0.0, z)
            cells.append(f'{math.degrees(best):7.0f}' if best is not None else '      -')
        print(f'{z:7.2f}' + ''.join(cells))
    print('\n  → grasp_pitch_ladder nên bắt đầu từ 1.5708 rồi nới dần cho vùng r lớn.')


def check_point(kin, x, y, z, pitch_deg, label=''):
    pitch = math.radians(pitch_deg)
    ok = bool(kin.ik_all(x, y, z, pitch))
    best = kin.max_pitch(x, y, z)
    best_txt = '—' if best is None else f'{math.degrees(best):.0f}°'
    mark = 'OK  ' if ok else 'HỎNG'
    print(f'  [{mark}] {label:<18} ({x:+.3f},{y:+.3f},{z:.3f}) r={math.hypot(x, y):.3f}m '
          f'pitch={pitch_deg:.0f}°  pitch tối đa ở đây: {best_txt}')
    if not ok:
        print(f'          {kin.reach_report(x, y, z, pitch)}')
    return ok


def params_of(doc):
    """Lấy dict ros__parameters từ file YAML kiểu ROS 2 ('/**:' hoặc tên node)."""
    for value in doc.values():
        if isinstance(value, dict) and 'ros__parameters' in value:
            return value['ros__parameters']
    return {}


def load_rack_calib(params):
    """Hình học giá đo bằng AprilTag (rack_pose.yaml), nếu tube_rack_params trỏ tới.

    Bỏ qua bước này thì `./rx150.sh reach` chấm điểm đúng bộ số mà tube_rack_node
    KHÔNG dùng — báo GO cho một cái giá tưởng tượng.
    """
    import os
    import yaml
    raw = str(params.get('rack_calib_file', '') or '').strip()
    if not raw:
        return None
    path = raw
    if not os.path.isabs(path):
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('rx150_perception'),
                                'config', raw)
        except Exception:                                  # noqa: BLE001
            return None
    if not os.path.exists(path):
        return None
    with open(path, 'r', encoding='utf-8') as handle:
        doc = yaml.safe_load(handle) or {}
    slots = [[float(c) for c in item] for item in (doc.get('slots') or [])]
    if not slots:
        return None
    axis = [float(v) for v in (doc.get('slot_axis') or [0.0, 0.0, 1.0])]
    norm = math.sqrt(sum(v * v for v in axis)) or 1.0
    return {'slots': slots, 'axis': [v / norm for v in axis], 'path': path}


def check_config(kin, path):
    import yaml
    with open(path, 'r', encoding='utf-8') as handle:
        params = params_of(yaml.safe_load(handle) or {})
    if not params:
        print(f'Không đọc được ros__parameters trong {path}')
        return False
    print(f'\nKIỂM TRA CONFIG: {path}')
    all_ok = True

    if 'place_x' in params:
        px, py = float(params['place_x']), float(params['place_y'])
        pz = float(params['place_z'])
        ladder = params.get('place_pitch_ladder', [0.0])
        pitch_deg = math.degrees(float(ladder[0]))
        all_ok &= check_point(kin, px, py, pz, pitch_deg, 'vị trí thả')
        all_ok &= check_point(kin, px, py, pz + float(params.get('place_approach_delta', 0.08)),
                              pitch_deg, 'thả (hover)')

    calib = load_rack_calib(params)
    if calib:
        print(f'  (hình học giá lấy từ HIỆU CHUẨN {calib["path"]}, '
              'không phải slot0_* trong file này)')
        length = float(params.get('tube_length', 0.10))
        depth = float(params.get('insert_depth', 0.03))
        clearance = float(params.get('hover_clearance', 0.04))
        ax, ay, az = calib['axis']
        tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, az))))
        along = length / 2.0 - depth
        for k, (sx, sy, sz) in enumerate(calib['slots']):
            ix, iy, iz = sx + ax * along, sy + ay * along, sz + az * along
            all_ok &= check_point(kin, ix, iy, iz, tilt_deg, f'slot {k} (cắm)')
            all_ok &= check_point(kin, ix, iy, iz + clearance, tilt_deg,
                                  f'slot {k} (hover)')
    elif 'slot0_x' in params:
        n = int(params.get('num_slots', 4))
        yaw = math.radians(float(params.get('rack_yaw_deg', params.get('rack_angle_deg', 0.0))))
        tilt = math.radians(float(params.get('rack_tilt_deg', 0.0)))
        spacing = float(params.get('slot_spacing', 0.05))
        dz = float(params.get('slot_dz', 0.0))
        length = float(params.get('tube_length', 0.10))
        depth = float(params.get('insert_depth', 0.03))
        clearance = float(params.get('hover_clearance', 0.04))
        for k in range(n):
            sx = float(params['slot0_x']) + k * spacing * math.cos(yaw)
            sy = float(params['slot0_y']) + k * spacing * math.sin(yaw)
            sz = float(params['slot0_z']) + k * dz
            azimuth = math.atan2(sy, sx)
            along = length / 2.0 - depth
            ix = sx + math.sin(tilt) * math.cos(azimuth) * along
            iy = sy + math.sin(tilt) * math.sin(azimuth) * along
            iz = sz + math.cos(tilt) * along
            all_ok &= check_point(kin, ix, iy, iz, math.degrees(tilt), f'slot {k} (cắm)')
            all_ok &= check_point(kin, ix, iy, iz + clearance,
                                  math.degrees(tilt), f'slot {k} (hover)')

    if 'reject_x' in params:
        all_ok &= check_point(kin, float(params['reject_x']), float(params['reject_y']),
                              float(params['reject_z']), 0.0, 'chỗ loại bỏ')
    verdict = ('TẤT CẢ điểm trong config đều với tới được.' if all_ok else
               'CÓ điểm KHÔNG với tới — sửa config trước khi chạy robot.')
    print(f'\nKết luận: {verdict}')
    return all_ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', help='file YAML tham số cần kiểm tra')
    parser.add_argument('--point', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--pitch', type=float, default=90.0, help='độ (mặc định 90)')
    parser.add_argument('--no-envelope', action='store_true')
    args = parser.parse_args()

    kin = Rx150Kinematics()
    ok = True
    if not args.no_envelope:
        print_envelope(kin)
    if args.point:
        print()
        ok &= check_point(kin, *args.point, args.pitch, 'điểm yêu cầu')
    if args.config:
        ok &= check_config(kin, args.config)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
