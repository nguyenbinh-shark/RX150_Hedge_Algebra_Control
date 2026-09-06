#!/usr/bin/env python3
"""
Kiểm chứng phần "công thức chuyển động" áp từ key_point sang:

  1. home_xyz_pitch → home_joints (resolve_home) — tư thế trung chuyển THU GỌN
     thay cho [0,0,0,0,0] duỗi thẳng vượt qua giá.
  2. _build_joint_traj — trajectory joint-space của backend 'direct'
     (gửi thẳng FollowJointTrajectory, không qua move_group).
  3. Hình học lùi bán kính (retreat_radial) và cột trung chuyển (plan_via).

Cần msg của ROS (moveit_msgs/control_msgs) nên skip khi chạy ngoài workspace.
"""
import math

import pytest

from rx150_modules.kinematics import Rx150Kinematics

motion = pytest.importorskip('rx150_modules.motion',
                             reason='cần moveit_msgs/control_msgs từ ROS')
params = pytest.importorskip('rx150_modules.params')


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(('info', m))

    def warn(self, m):
        self.lines.append(('warn', m))

    def error(self, m):
        self.lines.append(('error', m))


class _Node:
    def __init__(self):
        self._log = _Log()

    def get_logger(self):
        return self._log


class _Cfg:
    pass


# ── 1. tư thế trung chuyển ──────────────────────────────────────────────
def test_resolve_home_dat_dung_toa_do():
    """home_joints phải là IK của home_xyz_pitch, FK quay lại đúng điểm đó."""
    kin = Rx150Kinematics()
    cfg = _Cfg()
    cfg.home_xyz_pitch = [0.18, 0.0, 0.18, 0.0]
    cfg.home_joints = [0.0] * 5
    params.resolve_home(_Node(), kin, cfg)

    x, y, z, pitch = kin.fk(cfg.home_joints)
    assert (x, y, z) == pytest.approx((0.18, 0.0, 0.18), abs=1e-6)
    assert pitch == pytest.approx(0.0, abs=1e-9)
    assert (cfg.home_x, cfg.home_y, cfg.home_z) == pytest.approx((0.18, 0.0, 0.18))
    assert kin.in_limits(cfg.home_joints)


def test_home_cu_duoi_thang_vuot_qua_gia():
    """Ghi lại LÝ DO đổi: [0,0,0,0,0] nằm xa hơn giá ⇒ về home là quét qua giá."""
    kin = Rx150Kinematics()
    x_cu = kin.fk([0.0] * 5)[0]
    assert x_cu > 0.35                      # tay duỗi thẳng
    assert x_cu > 0.26                      # xa hơn slot0_x của tube_rack_params
    cfg = _Cfg()
    cfg.home_xyz_pitch = [0.18, 0.0, 0.18, 0.0]
    cfg.home_joints = [0.0] * 5
    params.resolve_home(_Node(), kin, cfg)
    assert kin.fk(cfg.home_joints)[0] < 0.26  # tư thế mới nằm SAU LƯNG giá


def test_resolve_home_pose_khong_voi_toi_thi_giu_nguyen():
    kin = Rx150Kinematics()
    cfg = _Cfg()
    cfg.home_xyz_pitch = [1.5, 0.0, 0.18, 0.0]      # ngoài tầm với
    cfg.home_joints = [0.0] * 5
    node = _Node()
    params.resolve_home(node, kin, cfg)
    assert cfg.home_joints == [0.0] * 5
    assert any(lvl == 'error' for lvl, _ in node.get_logger().lines)


def test_resolve_home_rong_thi_dung_home_joints_tho():
    kin = Rx150Kinematics()
    cfg = _Cfg()
    cfg.home_xyz_pitch = []
    cfg.home_joints = [0.0, -0.5, 0.5, 0.0, 0.0]
    params.resolve_home(_Node(), kin, cfg)
    assert cfg.home_joints == [0.0, -0.5, 0.5, 0.0, 0.0]
    assert cfg.home_x == pytest.approx(kin.fk(cfg.home_joints)[0])


# ── 2. trajectory joint-space của backend 'direct' ──────────────────────
def _executor(joint_speed=0.6):
    """Chỉ cần .joint_speed — _build_joint_traj là hàm thuần, không đụng ROS."""
    ex = motion.MoveItExecutor.__new__(motion.MoveItExecutor)
    ex.joint_speed = joint_speed
    return ex


def test_joint_traj_dung_hai_dau_va_thoi_gian_tang_dan():
    ex = _executor()
    start = [0.0, -1.26, 1.38, -0.12, 0.0]
    goal = [0.5, -0.9, 1.0, 0.2, -0.3]
    traj = ex._build_joint_traj(motion.ARM_JOINTS, start, goal, 1.0)

    assert list(traj.joint_names) == motion.ARM_JOINTS
    assert len(traj.points) >= 3
    assert list(traj.points[0].positions) == pytest.approx(start)
    assert list(traj.points[-1].positions) == pytest.approx(goal)

    times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
             for p in traj.points]
    assert times[0] == pytest.approx(0.0)
    assert all(b > a for a, b in zip(times, times[1:]))


def test_joint_traj_moi_khop_don_dieu():
    """Ease-in/out dùng CHUNG một tham số ⇒ không khớp nào đi vòng."""
    ex = _executor()
    start = [0.0] * 5
    goal = [0.4, -0.3, 0.6, -0.2, 0.1]
    traj = ex._build_joint_traj(motion.ARM_JOINTS, start, goal, 1.0)
    for j in range(5):
        seq = [p.positions[j] for p in traj.points]
        direction = 1 if goal[j] > start[j] else -1
        assert all((b - a) * direction >= -1e-12 for a, b in zip(seq, seq[1:]))


def test_joint_traj_cham_hon_khi_velocity_scale_nho():
    ex = _executor()
    start, goal = [0.0] * 5, [1.2, 0.0, 0.0, 0.0, 0.0]

    def dur(scale):
        t = ex._build_joint_traj(motion.ARM_JOINTS, start, goal, scale)
        p = t.points[-1].time_from_start
        return p.sec + p.nanosec * 1e-9

    assert dur(0.3) > dur(1.0)
    # 1.2 rad ở 0.6 rad/s, scale 1.0 ⇒ 2 s
    assert dur(1.0) == pytest.approx(2.0, abs=0.05)


def test_joint_traj_khong_bao_gio_ngan_hon_san_toi_thieu():
    """Bước cực nhỏ vẫn phải có thời gian đủ để bridge stream setpoint."""
    ex = _executor()
    traj = ex._build_joint_traj(motion.ARM_JOINTS, [0.0] * 5,
                                [1e-4, 0.0, 0.0, 0.0, 0.0], 1.0)
    p = traj.points[-1].time_from_start
    assert p.sec + p.nanosec * 1e-9 >= 0.6


# ── 3. hình học lùi + cột trung chuyển ──────────────────────────────────
def test_lui_ban_kinh_giu_nguyen_z_va_giam_dung_khoang_cach():
    """retreat_radial thu r đúng `distance`, không đổi z — và với y=0 thì
    trùng đúng `x − distance` như key_point làm."""
    kin = Rx150Kinematics()
    x, y, z = 0.26, 0.0, 0.22
    r = math.hypot(x, y)
    k = (r - 0.05) / r
    assert (x * k, y * k) == pytest.approx((x - 0.05, 0.0))
    assert kin.ik_all(x * k, y * k, z, 0.0)          # điểm lùi với tới được

    x, y = 0.26, 0.075                                # giá lệch sang bên
    r = math.hypot(x, y)
    k = (r - 0.05) / r
    assert math.hypot(x * k, y * k) == pytest.approx(r - 0.05)
    assert kin.ik_all(x * k, y * k, z, 0.0)


def test_cot_trung_chuyen_thap_hon_moi_slot_va_voi_toi_duoc():
    """Cột (0.18, 0) ở độ cao hover phải với tới được — nếu không, plan_via
    trả None và chuỗi mất lớp bảo hiểm."""
    kin = Rx150Kinematics()
    hover_z = 0.10 + (0.10 / 2 - 0.03) + 0.04         # slot0_z + along + clearance
    assert kin.ik_all(0.18, 0.0, hover_z, 0.0)
    for sy in (-0.075, -0.025, 0.025, 0.075):
        assert math.hypot(0.18, 0.0) < math.hypot(0.26, sy)   # cột nằm PHÍA TRONG
