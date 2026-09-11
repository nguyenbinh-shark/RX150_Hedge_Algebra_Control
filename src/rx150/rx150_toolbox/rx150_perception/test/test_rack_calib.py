#!/usr/bin/env python3
"""test_rack_calib.py — Kiểm HÌNH HỌC của rack_calib_node, KHÔNG cần camera/robot.

Phần dễ sai nhất của hiệu chuẩn giá không phải đọc tag mà là quy ước trục: khung
tag của apriltag_ros là "x phải, y LÊN, z ra phía người nhìn", không phải quy ước
"y down" của AprilTag gốc. Nhầm chỗ này thì 4 miệng lỗ ra sai đối xứng gương —
đúng chỗ nhìn bằng mắt trong RViz nhưng cắm ống trượt.

Bộ test này chốt các quy ước đó bằng số:
  - hình chữ nhật 120 × 50 với tag ở trung điểm cạnh dài gần
  - tag xoay / giá nghiêng ⇒ miệng lỗ và trục lỗ đi theo
  - yaw_offset_deg = 180 là phép QUAY (trục lỗ vẫn hướng lên), không phải phản chiếu
  - hộp vật cản bao đủ 4 lỗ, nóc đúng mặt phẳng miệng lỗ
  - cấu hình sai (slot_index_map không phải hoán vị) phải CHẾT, không nuốt im lặng

Chạy:
    source ~/interbotix_ws/source_all.sh
    ros2 run rx150_perception test_rack_calib.py
Thoát code 0 = tất cả pass.
"""
import importlib.util
import math
import os
import sys
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NODE_PATH = os.path.join(HERE, '..', 'scripts', 'rack_calib_node.py')
if not os.path.isfile(NODE_PATH):
    # chạy từ bản cài: node nằm cùng thư mục lib/rx150_perception
    NODE_PATH = os.path.join(HERE, 'rack_calib_node.py')

spec = importlib.util.spec_from_file_location('rack_calib_node', NODE_PATH)
RC = importlib.util.module_from_spec(spec)
spec.loader.exec_module(RC)

FAILS = []


def stub(**over):
    """RackCalib giả: chỉ có self.p — đủ cho toàn bộ phần hình học thuần."""
    params = dict(RC.DEFAULTS)
    params.update(over)
    node = SimpleNamespace(p=params)
    for name in ('_hole_offsets', '_R_tag_rack', '_solve', '_rack_box'):
        setattr(node, name, getattr(RC.RackCalib, name).__get__(node, SimpleNamespace))
    return node


def chk(name, got, want, tol=1e-9):
    got, want = np.asarray(got, dtype=float), np.asarray(want, dtype=float)
    ok = got.shape == want.shape and np.allclose(got, want, atol=tol)
    print(('  PASS  ' if ok else '  FAIL  ') + name)
    if not ok:
        print(f'          got  {np.round(got, 6).tolist()}')
        print(f'          want {np.round(want, 6).tolist()}')
        FAILS.append(name)


def main():
    ident = np.array([0., 0., 0., 1.])
    tag = np.array([0.26, 0.0, 0.10])

    print('1) 4 lỗ ở 4 đỉnh hình chữ nhật 120 × 50, tag ở trung điểm cạnh dài gần')
    chk('offsets (u, v, h)', stub()._hole_offsets(),
        [(-0.06, 0, 0), (0.06, 0, 0), (-0.06, 0.05, 0), (0.06, 0.05, 0)])

    print('2) tag ngửa, không xoay')
    sol = stub()._solve(tag, ident)
    chk('miệng lỗ', sol['slots'], [(0.20, 0, 0.10), (0.32, 0, 0.10),
                                   (0.20, 0.05, 0.10), (0.32, 0.05, 0.10)])
    chk('trục lỗ', sol['axis'], [0, 0, 1])
    chk('độ nghiêng', sol['tilt_deg'], 0.0)

    print('3) tag xoay 90° quanh z ⇒ cạnh dài nằm dọc +y')
    q90 = np.array([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    sol = stub()._solve(tag, q90)
    chk('miệng lỗ', sol['slots'], [(0.26, -0.06, 0.10), (0.26, 0.06, 0.10),
                                   (0.21, -0.06, 0.10), (0.21, 0.06, 0.10)])
    chk('hướng cạnh dài', sol['long_edge_yaw_deg'], 90.0, tol=1e-6)

    print('4) yaw_offset_deg = 180 (tag dán quay ngược) là phép QUAY')
    sol = stub(yaw_offset_deg=180.0)._solve(tag, ident)
    chk('miệng lỗ', sol['slots'], [(0.32, 0, 0.10), (0.20, 0, 0.10),
                                   (0.32, -0.05, 0.10), (0.20, -0.05, 0.10)])
    chk('trục lỗ VẪN hướng lên', sol['axis'], [0, 0, 1])

    print('5) giá nghiêng 20° ⇒ tilt_deg = pitch của ee lúc cắm')
    a = math.radians(20)
    sol = stub()._solve(tag, np.array([math.sin(a / 2), 0, 0, math.cos(a / 2)]))
    chk('độ nghiêng', sol['tilt_deg'], 20.0, tol=1e-6)
    chk('trục lỗ', sol['axis'], [0, -math.sin(a), math.cos(a)])

    print('6) mount front_vertical: (u, v, h) → (u, h, −v) trong khung tag')
    R = stub(mount='front_vertical')._R_tag_rack()
    chk('ánh xạ', R @ np.array([0.3, 0.7, 0.11]), [0.3, 0.11, -0.7])
    chk('det = +1 (thuận tay phải, không phản chiếu)', np.linalg.det(R), 1.0)

    print('7) hole_z_offset_m nâng miệng lỗ khỏi mặt phẳng tag')
    sol = stub(hole_z_offset_m=0.012)._solve(tag, ident)
    chk('z miệng lỗ', [s[2] for s in sol['slots']], [0.112] * 4)

    print('8) hole_offsets JSON đè hẳn hình chữ nhật')
    chk('offsets', stub(hole_offsets='[[0,0,0],[0.1,0,0.02]]')._hole_offsets(),
        [(0, 0, 0), (0.1, 0, 0.02)])

    print('9) slot_index_map hoán vị thứ tự slot (color_slot_map giữ nguyên)')
    chk('đảo cặp gần/xa', stub(slot_index_map=[2, 3, 0, 1])._hole_offsets(),
        [(-0.06, 0.05, 0), (0.06, 0.05, 0), (-0.06, 0, 0), (0.06, 0, 0)])

    print('10) hộp vật cản bao 4 lỗ, nóc đúng mặt phẳng miệng lỗ')
    box = stub()._rack_box([(0.20, 0, 0.10), (0.32, 0, 0.10),
                            (0.20, 0.05, 0.10), (0.32, 0.05, 0.10)])
    chk('tâm', [box['x'], box['y'], box['z']], [0.26, 0.025, 0.05])
    chk('kích thước', [box['size_x'], box['size_y'], box['size_z']], [0.14, 0.07, 0.10])

    print('11) cấu hình sai phải CHẾT ngay, không nuốt im lặng')
    try:
        stub(slot_index_map=[0, 0, 1, 2])._hole_offsets()
        print('  FAIL  slot_index_map trùng lặp bị bỏ qua')
        FAILS.append('slot_index_map trùng lặp')
    except SystemExit as exc:
        print(f'  PASS  slot_index_map trùng lặp → SystemExit: {exc}')
    try:
        stub(hole_offsets='[[0,0]]')._hole_offsets()
        print('  FAIL  hole_offsets thiếu thành phần bị bỏ qua')
        FAILS.append('hole_offsets thiếu thành phần')
    except SystemExit as exc:
        print(f'  PASS  hole_offsets thiếu thành phần → SystemExit: {exc}')

    print('12) trung bình quaternion (Markley) và khứ hồi R↔q')
    q = np.array([0.1, 0.2, 0.3, 0.9])
    q = q / np.linalg.norm(q)
    chk('±q là cùng một phép quay', RC.quat_to_R(RC.quat_mean([q, -q])),
        RC.quat_to_R(q), tol=1e-12)
    chk('R → q → R', RC.quat_to_R(RC.R_to_quat(RC.quat_to_R(q))),
        RC.quat_to_R(q), tol=1e-12)

    print(f'\nTỔNG: {"tất cả PASS" if not FAILS else f"FAIL: {FAILS}"}')
    return 1 if FAILS else 0


if __name__ == '__main__':
    sys.exit(main())
