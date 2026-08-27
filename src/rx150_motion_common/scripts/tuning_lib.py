#!/usr/bin/env python3
"""
tuning_lib — thư viện dùng chung cho các script tuning/hiệu chuẩn HAC
(rx150_tuning_session.py, rx150_run_compare.py, rx150_gravity_id.py,
rx150_friction_id.py):

- TelemetryRecorder: ghi CSV schema 46 cột giống hệt data_analysis/csv_logger.py
  (timestamp + 5 khớp × {pos,vel,ref_pos,ref_vel,err,edot,pwm,grav,fric}) —
  file ra mở được bằng data_analysis/plot_control_csv.py.
- forward_kinematics_ee / ee_height: FK nhẹ (không cần URDF/Pinocchio) để lọc
  lưới pose an toàn theo độ cao end-effector.
- load_joint_limits_rad: đọc Min/Max_Position_Limit (tick) từ rx150_motor.yaml,
  đổi ra rad, trừ margin an toàn.
- step_metrics/rmse/rms/steady_state_error/is_settled: metrics dùng chung cho
  session + validate.

KHÔNG phải ROS node — các script tự tạo Node của mình và compose các helper ở
đây (TelemetryRecorder.__init__ nhận node đã tồn tại để add_subscription lên
nó, không tạo node riêng).
"""

import csv
import math
import os

import numpy as np
import yaml

ARM_JOINTS = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]


# ───────────────────────── src/ config file lookup ──────────────────────
# Lưu vào install/ thì colcon build lần sau sẽ ghi đè mất — mọi script ghi
# config (gains, gravity model) phải nhắm vào bản src/.

def find_src_config(package, filename):
    """Tìm file src/<package>/config/<filename> ĐÃ TỒN TẠI, suy ra từ
    install/<package>/share/<package>/config/<filename> (ament_index).
    Trả None nếu không tìm thấy (package chưa build, hoặc chỉ có ở install).
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        install_path = os.path.join(get_package_share_directory(package), "config", filename)
    except Exception:
        return None
    parts = install_path.replace(os.sep, "/").split("/")
    if "install" not in parts:
        return None
    ws_root = "/".join(parts[:parts.index("install")])
    candidate = os.path.join(ws_root, "src", package, "config", filename)
    if os.path.isfile(candidate):
        return candidate
    for root, _dirs, _files in os.walk(os.path.join(ws_root, "src")):
        if os.path.basename(root) == package:
            cand2 = os.path.join(root, "config", filename)
            if os.path.isfile(cand2):
                return cand2
    return None


def src_config_path(package, filename):
    """Tính đường dẫn src/<package>/config/<filename> để GHI MỚI (file có thể
    chưa tồn tại — dùng cho output do tool sinh, vd rx150_gravity_model.yaml).
    Trả None nếu không suy ra được ws_root (package chưa cài đặt lần nào).
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        install_path = os.path.join(get_package_share_directory(package), "config", filename)
    except Exception:
        return None
    parts = install_path.replace(os.sep, "/").split("/")
    if "install" not in parts:
        return None
    ws_root = "/".join(parts[:parts.index("install")])
    return os.path.join(ws_root, "src", package, "config", filename)


# ───────────────────────── CSV schema (46 cột) ──────────────────────────

def csv_header():
    header = ["timestamp"]
    for suffix in ("pos", "vel", "ref_pos", "ref_vel", "err", "edot", "pwm", "grav", "fric"):
        for j in ARM_JOINTS:
            header.append(f"{j}_{suffix}")
    return header


def read_csv(path):
    """Đọc lại file CSV (schema 46 cột hoặc bất kỳ) thành (header, rows) với
    rows là list[list[float]] — ô trống -> NaN. Dùng bởi rx150_run_compare.py.
    """
    with open(path, newline="") as fh:
        r = csv.reader(fh)
        header = next(r)
        rows = [
            [float(v) if v not in ("", None) else float("nan") for v in raw]
            for raw in r
        ]
    return header, rows


class TelemetryRecorder:
    """Subscribe joint_states + {prefix}/{reference,error,edot,effort,gravity,
    friction}; mỗi lần nhận joint_states (~100Hz) ghi 1 dòng nếu đang recording.

    `prefix` PHẢI là topic tuyệt đối, ví dụ '/rx150/hac' hoặc '/rx150/joint_states'
    cho joint_states — caller truyền prefix dạng '/rx150/hac' để build ra
    '/rx150/hac/reference' v.v; joint_states subscribe cố định '/rx150/joint_states'.
    """

    def __init__(self, node, prefix, joint_states_topic="/rx150/joint_states"):
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import JointState

        self._node = node
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self._latest = {}
        self._rows = []
        self._t0 = None
        self._recording = False

        node.create_subscription(JointState, joint_states_topic, self._on_js, qos)
        topics = {
            "ref": f"{prefix}/reference",
            "err": f"{prefix}/error",
            "edot": f"{prefix}/edot",
            "eff": f"{prefix}/effort",
            "grav": f"{prefix}/gravity",
            "fric": f"{prefix}/friction",
            # gravity_torque (N·m thô, độc lập Gff/gravity_sign) KHÔNG nằm
            # trong 46 cột CSV chuẩn — chỉ cache cho sampling điểm (gravity_id).
            "gtorque": f"{prefix}/gravity_torque",
        }
        for key, topic in topics.items():
            node.create_subscription(
                JointState, topic,
                lambda msg, k=key: self._latest.__setitem__(k, msg), qos)

    def start(self):
        self._recording = True
        self._t0 = None
        self._rows = []

    def stop(self):
        self._recording = False

    @staticmethod
    def _extract(msg, field):
        if msg is None:
            return [float("nan")] * len(ARM_JOINTS)
        name_map = {n: i for i, n in enumerate(msg.name)}
        arr = getattr(msg, field, [])
        out = []
        for j in ARM_JOINTS:
            idx = name_map.get(j)
            out.append(arr[idx] if idx is not None and idx < len(arr) else float("nan"))
        return out

    def _on_js(self, msg):
        self._latest["js"] = msg
        if not self._recording:
            return
        if self._t0 is None:
            self._t0 = self._node.get_clock().now()
        elapsed = (self._node.get_clock().now() - self._t0).nanoseconds * 1e-9

        row = [elapsed]
        row += self._extract(msg, "position")
        row += self._extract(msg, "velocity")
        row += self._extract(self._latest.get("ref"), "position")
        row += self._extract(self._latest.get("ref"), "velocity")
        row += self._extract(self._latest.get("err"), "position")
        row += self._extract(self._latest.get("edot"), "velocity")
        row += self._extract(self._latest.get("eff"), "effort")
        row += self._extract(self._latest.get("grav"), "effort")
        row += self._extract(self._latest.get("fric"), "effort")
        self._rows.append(row)

    def latest_positions(self):
        """Vị trí hiện tại 5 khớp (rad), NaN nếu chưa nhận joint_states."""
        return self._extract(self._latest.get("js"), "position")

    def latest_velocities(self):
        return self._extract(self._latest.get("js"), "velocity")

    def latest_effort_pwm(self):
        """PWM tổng (hac/effort), dùng làm chuẩn fit — KHÔNG phải joint_states.effort."""
        return self._extract(self._latest.get("eff"), "effort")

    def latest_measured_load(self):
        """joint_states.effort — Present_Load×2.69, đơn vị mô-men tương đối
        (KHÔNG phải PWM); dùng cho B0 signcheck / đối chiếu, không dùng để fit
        chính thức (đơn vị fit chính là PWM, xem module docstring của
        rx150_gravity_id.py)."""
        return self._extract(self._latest.get("js"), "effort")

    def latest_gravity_pwm(self):
        """hac/gravity — thành phần gravity ĐÃ scale PWM (Gff*sign hoặc fitted,
        tuỳ gravity_model_source hiện tại). Dùng cho friction ID: u - u_grav."""
        return self._extract(self._latest.get("grav"), "effort")

    def latest_gravity_torque(self):
        """hac/gravity_torque — τ Pinocchio thô (N·m), LUÔN được publish kể cả
        enable_gravity_comp=false, độc lập Gff/gravity_sign. Dùng cho B0/B1."""
        return self._extract(self._latest.get("gtorque"), "effort")

    def write_csv(self, path):
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(csv_header())
            for row in self._rows:
                w.writerow([f"{v:.6f}" if v == v else "" for v in row])

    @property
    def rows(self):
        return self._rows


# ───────────────────────── Forward kinematics (nhẹ) ──────────────────────
# Offset lấy trực tiếp từ rx150.urdf.xacro (joint origin xyz, TRƯỚC khi áp
# dụng phép quay của khớp đó). wrist_rotate quay quanh trục X của chính nó,
# nên không ảnh hưởng vị trí EE (chỉ ảnh hưởng roll) — vẫn giữ trong chuỗi để
# đúng ngữ nghĩa, kết quả vị trí sẽ bất biến theo góc này.

_LINK_OFFSETS = [
    (np.array([0.0, 0.0, 0.06566]), "z"),   # waist
    (np.array([0.0, 0.0, 0.03891]), "y"),   # shoulder
    (np.array([0.05, 0.0, 0.15]), "y"),     # elbow
    (np.array([0.15, 0.0, 0.0]), "y"),      # wrist_angle
    (np.array([0.065, 0.0, 0.0]), "x"),     # wrist_rotate
]
_EE_OFFSET = np.array([0.043, 0.0, 0.0])  # ee_arm_link, joint 'ee_arm' (fixed)


def _rot(axis, theta):
    c, s = math.cos(theta), math.sin(theta)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])  # z


def forward_kinematics_ee(q):
    """FK nhẹ (không cần URDF/Pinocchio) — trả (x, y, z) mét của ee_arm_link
    trong khung base_link cho q=[waist,shoulder,elbow,wrist_angle,wrist_rotate].
    Chỉ dùng để lọc lưới pose an toàn (độ cao EE) — KHÔNG dùng cho điều khiển.
    """
    p = np.zeros(3)
    R = np.eye(3)
    for (offset, axis), angle in zip(_LINK_OFFSETS, q):
        p = p + R @ offset
        R = R @ _rot(axis, angle)
    p = p + R @ _EE_OFFSET
    return p


def ee_height(q):
    return float(forward_kinematics_ee(q)[2])


# ───────────────────────── Joint limits từ rx150_motor.yaml ──────────────

_TICKS_PER_REV = 4096
_CENTER_TICK = 2048


def _tick_to_rad(tick):
    return (tick - _CENTER_TICK) * (2.0 * math.pi / _TICKS_PER_REV)


def load_joint_limits_rad(motor_yaml_path, margin=0.2, joints=ARM_JOINTS):
    """Đọc Min/Max_Position_Limit (tick, 12-bit, tick=2048 <-> 0 rad) từ
    rx150_motor.yaml, đổi ra rad, trừ margin (rad) mỗi bên để lưới pose ID tự
    động không chạm cữ cơ khí. Trả dict joint -> (lo, hi) rad.
    """
    with open(motor_yaml_path) as fh:
        cfg = yaml.safe_load(fh)
    motors = cfg.get("motors", {})
    limits = {}
    for j in joints:
        m = motors.get(j)
        if m is None:
            continue
        lo = _tick_to_rad(m["Min_Position_Limit"]) + margin
        hi = _tick_to_rad(m["Max_Position_Limit"]) - margin
        if lo > hi:
            lo, hi = hi, lo
        limits[j] = (lo, hi)
    return limits


# ───────────────────────── Metrics ──────────────────────

def step_metrics(t, pos, ref_final, pos0=None):
    """Rise time (10-90%), overshoot % (so với |Δ|), settling time (băng 2%)
    cho 1 step response. t, pos: mảng cùng chiều dài.
    """
    t = np.asarray(t, dtype=float)
    pos = np.asarray(pos, dtype=float)
    if pos0 is None:
        pos0 = pos[0]
    delta = ref_final - pos0
    if abs(delta) < 1e-9:
        return {"rise_time": float("nan"), "overshoot_pct": 0.0, "settling_time": 0.0}

    frac = (pos - pos0) / delta

    def first_cross(level):
        hits = np.flatnonzero(frac >= level) if delta > 0 else np.flatnonzero(frac <= level)
        return float(t[hits[0]]) if hits.size else float("nan")

    t10, t90 = first_cross(0.1), first_cross(0.9)
    rise = (t90 - t10) if (t10 == t10 and t90 == t90) else float("nan")

    if delta > 0:
        overshoot = max(0.0, (pos.max() - ref_final) / abs(delta) * 100.0)
    else:
        overshoot = max(0.0, (ref_final - pos.min()) / abs(delta) * 100.0)

    tol = 0.02 * abs(delta)
    settle_idx = None
    for i in range(len(pos)):
        if np.all(np.abs(pos[i:] - ref_final) <= tol):
            settle_idx = i
            break
    settling = (t[settle_idx] - t[0]) if settle_idx is not None else float("nan")

    return {
        "rise_time": float(rise),
        "overshoot_pct": float(overshoot),
        "settling_time": float(settling),
    }


def rmse(a, b=0.0):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float) if not np.isscalar(b) else b
    return float(np.sqrt(np.nanmean((a - b) ** 2)))


def rms(a):
    a = np.asarray(a, dtype=float)
    return float(np.sqrt(np.nanmean(a ** 2)))


def steady_state_error(pos, ref_final, window=20):
    tail = np.asarray(pos[-window:], dtype=float)
    return float(np.nanmean(tail) - ref_final)


def is_settled(vel_window, thresh=0.02):
    """True nếu |vel| < thresh cho toàn bộ cửa sổ (rad/s)."""
    return all(abs(v) == abs(v) and abs(v) < thresh for v in vel_window)
