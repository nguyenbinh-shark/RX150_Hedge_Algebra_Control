#!/usr/bin/env python3
"""Kiểm chứng IK/FK giải tích của rx150_modules.kinematics.

Chạy được KHÔNG CẦN robot:  python3 -m pytest src/rx150/rx150_toolbox/rx150_modules/test -q
Nếu có interbotix_xs_modules + modern_robotics thì đối chiếu luôn với
mr.FKinSpace (nguồn sự thật duy nhất về hình học rx150).
"""
import math

import pytest

from rx150_modules.kinematics import (
    ARM_JOINTS, Rx150Kinematics, wrap, wrist_rotate_for_axis,
)

GRID = [(x, y, z) for x in (0.15, 0.20, 0.25, 0.30) for y in (-0.10, 0.0, 0.12)
        for z in (0.03, 0.08, 0.15, 0.22)]
PITCHES = [math.radians(d) for d in (-30, 0, 30, 60, 85)]


@pytest.fixture(scope='module')
def kin():
    return Rx150Kinematics()


def _rzyx(yaw, pitch, roll):
    """Rz(yaw)·Ry(pitch)·Rx(roll) — quy ước hướng ee của rx150."""
    import numpy as np
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return rz @ ry @ rx


def test_ik_fk_roundtrip(kin):
    """Mọi nghiệm IK phải đưa ee về đúng pose yêu cầu (< 0.1 mm / 0.01°)."""
    n = 0
    for (x, y, z) in GRID:
        for pitch in PITCHES:
            for joints in kin.ik_all(x, y, z, pitch):
                fx, fy, fz, fp = kin.fk(joints)
                assert math.hypot(fx - x, fy - y) < 1e-4
                assert abs(fz - z) < 1e-4
                assert abs(wrap(fp - pitch)) < 2e-4
                n += 1
    assert n > 100, f'grid quá nhỏ để có ý nghĩa (chỉ {n} nghiệm)'


def test_ik_respects_joint_limits(kin):
    for (x, y, z) in GRID:
        for pitch in PITCHES:
            for joints in kin.ik_all(x, y, z, pitch):
                assert kin.limit_violation(joints) is None


def test_ik_seed_keeps_branch(kin):
    """APPROACH → DESCEND phải liên tục (không lật khuỷu ngay trên đầu vật)."""
    approach = kin.ik(0.24, 0.0, 0.16, math.radians(70))
    descend = kin.ik(0.24, 0.0, 0.06, math.radians(70), seed=approach)
    assert approach is not None and descend is not None
    assert max(abs(a - b) for a, b in zip(approach, descend)) < 1.2


def test_ik_seed_picks_nearest_of_two_branches(kin):
    """Ở pose có CẢ 2 nhánh, seed phải quyết định nhánh — nếu không, mỗi bước
    trong chuỗi có thể nhảy sang cấu hình khuỷu ngược lại."""
    # Giới hạn khớp rx150 khiến phần lớn pose chỉ còn 1 nhánh; đây là một pose
    # còn cả 2 (elbow-up / elbow-down) — tìm bằng quét lưới.
    target, pitch = (0.25, 0.0, 0.25), math.radians(50)
    both = kin.ik_all(*target, pitch)
    assert len(both) == 2, f'pose kiểm tra không còn 2 nhánh ({len(both)})'
    for want in both:
        got = kin.ik(*target, pitch, seed=want)
        assert max(abs(a - b) for a, b in zip(got, want)) < 1e-9


def test_unreachable_returns_none(kin):
    assert kin.ik(0.60, 0.0, 0.10, 0.0) is None          # quá tầm
    assert kin.ik_all(0.36, 0.0, 0.05, math.pi / 2) == []  # quá dốc ở xa
    assert 'quá dốc' in kin.reach_report(0.36, 0.0, 0.05, math.pi / 2)


def test_pitch_envelope_shrinks_with_radius(kin):
    """pitch=90° chỉ có ở gần; càng ra xa càng phải nghiêng — cơ sở của pitch ladder."""
    near = kin.max_pitch(0.22, 0.0, 0.05)
    far = kin.max_pitch(0.34, 0.0, 0.05)
    assert near is not None and far is not None
    assert near > math.radians(85) > far


def test_wrist_rotate_for_axis_at_top_down():
    """pitch = 90° ⇒ wrist_rotate = waist − yaw (KHÔNG phải yaw − waist)."""
    assert wrist_rotate_for_axis(0.0, 0.0) == pytest.approx(0.0)
    assert wrist_rotate_for_axis(0.3, 0.1) == pytest.approx(0.2)
    # yaw lệch > 90° ⇒ dùng nghiệm đối xứng mod pi thay vì xoay gần 180°
    assert abs(wrist_rotate_for_axis(0.0, 2.9)) <= math.pi / 2


def test_wrist_rotate_makes_fingers_perpendicular_to_tube():
    """Kiểm tra điều kiện kẹp thật: trục ee_y (phương ép của 2 ngón) ⟂ trục ống,
    ở MỌI pitch trong ladder — không chỉ ở 90°."""
    mr = pytest.importorskip('modern_robotics')
    mrd = pytest.importorskip('interbotix_xs_modules.xs_robot.mr_descriptions')
    import numpy as np
    des = getattr(mrd, 'rx150')
    kin = Rx150Kinematics()
    checked = 0
    for pitch_deg in (90, 80, 70, 60, 45, 30):
        for yaw in (0.0, 0.4, -0.6, 1.2):
            for (x, y) in ((0.25, 0.0), (0.20, 0.10), (0.22, -0.08)):
                pitch = math.radians(pitch_deg)
                waist = math.atan2(y, x)
                roll = wrist_rotate_for_axis(waist, yaw, pitch)
                joints = kin.ik(x, y, 0.06, pitch, wrist_rotate=roll)
                if joints is None:
                    continue
                T = mr.FKinSpace(des.M, des.Slist, np.array(joints, dtype=float))
                ee_y = T[:3, 1]
                axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
                assert abs(float(ee_y @ axis)) < 1e-6
                checked += 1
    assert checked > 30


def test_against_modern_robotics():
    """Đối chiếu FK giải tích với mr.FKinSpace trên chuỗi screw thật của rx150."""
    mr = pytest.importorskip('modern_robotics')
    mrd = pytest.importorskip('interbotix_xs_modules.xs_robot.mr_descriptions')
    import numpy as np
    des = getattr(mrd, 'rx150')
    kin = Rx150Kinematics()
    n = 0
    for (x, y, z) in GRID:
        for pitch in PITCHES:
            for joints in kin.ik_all(x, y, z, pitch, wrist_rotate=0.3):
                T = mr.FKinSpace(des.M, des.Slist, np.array(joints, dtype=float))
                assert abs(T[0, 3] - x) < 1e-5
                assert abs(T[1, 3] - y) < 1e-5
                assert abs(T[2, 3] - z) < 1e-5
                # hướng: R_ee = Rz(waist)·Ry(pitch)·Rx(wrist_rotate) — chính là
                # quy ước cho công thức wrist_rotate_for_axis()
                assert np.allclose(T[:3, :3], _rzyx(joints[0], pitch, joints[4]),
                                   atol=1e-5)
                n += 1
    assert n > 100


def test_arm_joint_order():
    assert ARM_JOINTS == ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
