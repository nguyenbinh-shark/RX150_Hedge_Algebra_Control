#!/usr/bin/env python3
"""
rx150_ee_tag_bench — đo vị trí THẬT của tay gắp bằng camera (AprilTag dán trên
gripper) và so với quỹ đạo mà robot tưởng nó đang đi.

Ba quỹ đạo của CÙNG một điểm (rx150/ee_gripper_link), trong cùng một khung
(mặc định rx150/base_link):

  cmd  — lệnh:     FK giải tích từ setpoint gửi cho controller (rx150_modules)
  enc  — encoder:  TF base_link -> ee_gripper_link (robot_state_publisher, tức
                   FK URDF từ joint_states đọc về từ Dynamixel)
  cam  — camera:   pose AprilTag do apriltag_ros đo, quy về ee_gripper_link qua
                   offset gắn tag X = T(ee -> tag) ĐƯỢC ƯỚC LƯỢNG TỪ DỮ LIỆU

Ba hiệu số trả lời ba câu khác nhau:
  cmd→enc  controller bám setpoint tốt đến đâu (quy sai số khớp ra mm Cartesian)
  enc→cam  cái ENCODER KHÔNG THẤY: võng khâu, rơ bánh răng, sai số hình học +
           sai số hiệu chuẩn camera. Đây là phần steady_state_bench mù tịt.
  cmd→cam  tổng — chính là sai số mà pick_place thực sự chịu khi với tới vật.

TẠI SAO KHÔNG CẦN BIẾT TAG DÁN CHÍNH XÁC Ở ĐÂU:
  X được fit từ chính các mẫu đứng yên (X_i = T_ref_ee_i⁻¹ · T_ref_tag_i, lấy
  trung vị vị trí + trung bình quaternion). Mọi sai lệch HẰNG — dán tag lệch,
  bias hiệu chuẩn camera — bị hấp thụ vào X. Phần dư sau khi trừ X là phần
  PHỤ THUỘC POSE, đúng thứ cần cho tuning. Đổi lại: bench này KHÔNG đo được
  bias tuyệt đối của toàn hệ, chỉ đo được độ KHÔNG NHẤT QUÁN giữa các pose.

Chế độ:
  hold     chạy qua bộ pose tĩnh (publish thẳng <ns>/setpoint như
           rx150_steady_state_bench.py), mỗi pose lấy N mẫu khi đã đứng yên
  watch    KHÔNG ra lệnh gì — chỉ ghi trong lúc MoveIt/pick_place đang chạy
  sweep    quét quỹ đạo chậm liên tục giữa các pose để so quỹ đạo ĐỘNG (trễ,
           vọt lố) chứ không chỉ điểm dừng
  analyze  tính lại + vẽ từ CSV đã ghi (không cần robot)

Yêu cầu đang chạy:
  T1  ./rx150.sh t1-hac   (robot + MoveIt + camera)
  T2  ./rx150.sh t2       (TF hiệu chuẩn world <-> camera từ static_transforms.yaml)
  T3  ./rx150.sh eetag    (apriltag_ros continuous detector -> /ee_tag/tag_detections)
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

try:
    from apriltag_ros.msg import AprilTagDetectionArray
except ImportError:                       # pragma: no cover - chỉ khi thiếu pkg
    print("Thiếu apriltag_ros — source install/setup.bash của workspace apriltag.")
    raise

ARM = ["waist", "shoulder", "elbow", "wrist_angle", "wrist_rotate"]

# Bộ pose mặc định: lấy từ rx150_steady_state_bench.py (để đối chiếu được với
# bảng sai số khớp đã đo) + vài pose xoay waist để tag lộ ra cho camera.
DEFAULT_POSES = {
    "home":       [0.0, -1.260, 1.381, -0.120, 0.0],
    "reach_far":  [0.0, -0.90,  1.00,  0.30, 0.0],
    "reach_low":  [0.0, -0.55,  0.95,  0.55, 0.0],
    "reach_side": [0.6, -0.90,  1.10,  0.20, -0.6],
    "side_neg":   [-0.6, -0.90, 1.10,  0.20, 0.6],
    "up_high":    [0.0, -1.50,  0.70, -0.20, 0.0],
}
SLEEP_POSE = [0.0, -1.80, 1.55, 0.80, 0.0]

# Bộ pose HIỆU CHUẨN (mode refine): 3 tầm với × 5 góc waist, wrist_rotate = 0.
# Hai đòi hỏi đánh nhau — tag phải LỘ ra cho camera, mà dữ liệu phải TRẢI RỘNG
# thì mới tách được xoay khỏi tịnh tiến. wrist_rotate giữ 0 vì chính nó là thứ
# lật mặt tag đi (reach_side ở bộ mặc định mất tag đúng vì wrist_rotate = -0.6);
# quét waist thì tag vẫn ngửa về phía camera mà EE đi được cả dải y.
CALIB_POSES = {}
for _wn, _w in [("wL", -0.50), ("wl", -0.25), ("w0", 0.0), ("wr", 0.25), ("wR", 0.50)]:
    for _cn, _arm in [("high", (-1.26, 1.381, -0.12)),
                      ("mid", (-0.90, 1.00, 0.30)),
                      ("low", (-0.55, 0.95, 0.55))]:
        CALIB_POSES[f"{_cn}_{_wn}"] = [_w, _arm[0], _arm[1], _arm[2], 0.0]

# Bộ pose HIỆU CHUẨN OFFSET TAG (mode tagoffset). Khác CALIB_POSES/grid ở chỗ
# QUÉT wrist_rotate: t_X chỉ tách được khỏi sai số tịnh tiến của extrinsic nhờ
# R_ee THAY ĐỔI giữa các pose — nếu mọi pose đều wrist_rotate=0 thì R_ee·t_X gần
# như hằng và lẫn hoàn toàn vào t_camera. Đo được trên chính dữ liệu đã ghi:
# grid27 (wrist_rotate≡0) cho std(t_X) = (1.20, 2.21, 2.28) mm, bộ này (0.91,
# 1.03, 1.04) mm — tốt hơn 2.2x ở trục xấu nhất với ít hơn 6 pose.
#
# Chọn bằng tìm kiếm rời rạc trên 2059 ứng viên (IK giải được × tag còn trong
# FOV × góc tới mặt tag ≤ 55° × cách camera 0.30–0.70 m), tối ưu trực tiếp
# std(t_X) lớn nhất lấy từ hiệp phương sai của bài toán 9 ẩn (6 extrinsic +
# t_X) — chứ không tối ưu độ phủ vị trí như grid_poses().
#
# z ≥ 0.12 m ở MỌI pose: nóc hộp giá đỡ nằm ở z ≈ 0.083 nên bộ này an toàn kể
# cả khi giá đang đặt trên bàn. Sửa bộ pose thì phải kiểm tra lại chặn này.
TAGCAL_POSES = {
    "r18z16w-29p69t-52": [-0.5, -0.4492, 0.1348, 1.5145, -0.9],
    "r18z20w0p69t34":    [0.0, -0.3279, -0.2755, 1.8034, 0.6],
    "r18z12w14p69t34":   [0.25, -0.4951, 0.4452, 1.2499, 0.6],
    "r18z20w14p69t0":    [0.25, -0.3279, -0.2755, 1.8034, 0.0],
    "r18z20w29p69t0":    [0.5, -0.3279, -0.2755, 1.8034, 0.0],
    "r22z12w-29p0t17":   [-0.5, -0.2367, 1.4818, -1.2451, 0.3],
    "r22z20w-29p34t17":  [-0.5, -0.6851, 0.4347, 0.8505, 0.3],
    "r22z25w-29p0t-69":  [-0.5, -0.9114, 0.8178, 0.0936, -1.2],
    "r22z25w29p0t-17":   [0.5, -0.9114, 0.8178, 0.0936, -0.3],
    "r26z12w-29p0t17":   [-0.5, -0.0596, 1.2156, -1.156, 0.3],
    "r26z16w29p0t-17":   [0.5, -0.3731, 1.1252, -0.7521, -0.3],
    "r26z20w29p17t-17":  [0.5, -0.5849, 0.6542, 0.2308, -0.3],
    "r30z12w-29p17t-69": [-0.5, -0.1265, 0.7937, -0.3672, -1.2],
    "r30z16w-29p52t-69": [-0.5, 0.0317, -0.242, 1.1103, -1.2],
    "r30z28w-29p0t0":    [-0.5, -0.36, 0.1761, 0.1839, 0.0],
    "r30z16w-14p0t-52":  [-0.25, -0.1339, 0.8627, -0.7288, -0.9],
    "r30z12w29p0t-17":   [0.5, 0.0995, 0.934, -1.0335, -0.3],
    "r30z20w29p0t-17":   [0.5, -0.2894, 0.7199, -0.4305, -0.3],
    "r30z25w29p0t52":    [0.5, -0.3727, 0.456, -0.0833, 0.9],
    "r30z25w29p0t69":    [0.5, -0.3727, 0.456, -0.0833, 1.2],
}

CSV_HEADER = (
    ["t", "stamp", "label", "moving"]
    + [f"cmd_{j}" for j in ARM]
    + [f"meas_{j}" for j in ARM]
    + ["enc_x", "enc_y", "enc_z", "enc_qx", "enc_qy", "enc_qz", "enc_qw"]
    + ["cam_x", "cam_y", "cam_z", "cam_qx", "cam_qy", "cam_qz", "cam_qw"]
    + ["fk_x", "fk_y", "fk_z", "fk_pitch"]
    # raw = pose tag trong khung CAMERA, trước khi nhân TF hiệu chuẩn. Giữ lại để
    # `refine` ước lượng lại chính cái TF đó (nếu chỉ lưu pose đã nhân TF thì sai
    # số của TF cũ đã bị nướng vào dữ liệu, không gỡ ra được).
    + ["raw_x", "raw_y", "raw_z", "raw_qx", "raw_qy", "raw_qz", "raw_qw"]
    # Tag GIÁ (id 0) đo từ CÙNG khung hình, cũng trong khung CAMERA (chưa nhân
    # TF hiệu chuẩn). rk_ok=0 khi khung đó không thấy tag giá. Có hai tag trong
    # một tấm ảnh thì hiệu hai vector KHÔNG đi qua t_camera nữa — xem dual_tag().
    + ["rk_ok", "rk_dt", "rk_x", "rk_y", "rk_z", "rk_qx", "rk_qy", "rk_qz", "rk_qw"]
)


# Quy ước xoay của ee_gripper_link, kiểm chứng trực tiếp với TF sống ngày
# 2026-09-11 (lệch 0.000°): R_ee = Rz(waist)·Ry(pitch)·Rx(wrist_rotate),
# pitch = shoulder + elbow + wrist_angle, pitch > 0 là CHÚC XUỐNG.

def Rz(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])


def Rx(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])


def R_ee_of(q):
    return Rz(q[0]) @ Ry(q[1] + q[2] + q[3]) @ Rx(q[4])


# ─────────────────────────── SE(3) helpers (numpy thuần) ───────────────────
# Không dùng tf2_geometry_msgs.do_transform_pose: chữ ký hàm đó đổi giữa các
# bản Humble (Pose vs PoseStamped) — tự nhân ma trận thì không bao giờ hỏng.

def quat_to_R(q):
    """q = (x, y, z, w) -> ma trận quay 3x3."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    """3x3 -> (x, y, z, w). Shepperd: chọn nhánh theo phần tử trội, ổn định số."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


def make_T(p, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_R(q)
    T[:3, 3] = p
    return T


def T_inv(T):
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def quat_mean(quats):
    """Trung bình quaternion (Markley): vector riêng ứng trị riêng lớn nhất của
    Σ qqᵀ. Xử lý đúng dấu ±q (cùng một phép quay)."""
    M = np.zeros((4, 4))
    for q in quats:
        q = np.asarray(q, dtype=float)
        q = q / np.linalg.norm(q)
        M += np.outer(q, q)
    w, v = np.linalg.eigh(M)
    q = v[:, int(np.argmax(w))]
    return q / np.linalg.norm(q)


def angle_between_R(R1, R2):
    """Góc quay của R1ᵀR2 (rad)."""
    c = (np.trace(R1.T @ R2) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# ────────────────────────────── FK từ lệnh ─────────────────────────────────

def make_fk():
    """FK giải tích tới ee_gripper_link (rx150_modules). Fallback: FK nhẹ trong
    tuning_lib (tới ee_arm_link — lệch 1 offset HẰNG, bị X hấp thụ hết nên vẫn
    dùng được để so hình dạng quỹ đạo)."""
    try:
        from rx150_modules.kinematics import Rx150Kinematics
        kin = Rx150Kinematics()
        return lambda q: kin.fk(list(q)), "rx150_modules.Rx150Kinematics.fk"
    except Exception as exc:                              # noqa: BLE001
        sys.stderr.write(f"[warn] không import được rx150_modules ({exc}); dùng FK tuning_lib\n")
        from tuning_lib import forward_kinematics_ee
        return (lambda q: tuple(forward_kinematics_ee(np.asarray(q))) + (q[1] + q[2] + q[3],),
                "tuning_lib.forward_kinematics_ee")


# ──────────────────────────────── Node ─────────────────────────────────────

class EeTagBench(Node):
    def __init__(self, args):
        super().__init__("ee_tag_bench")
        self.args = args
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.js = None                     # JointState mới nhất
        self.js_hist = deque(maxlen=600)   # (t_sec, pos[5], vel[5]) để tra theo stamp ảnh
        self.det = None                    # (stamp_sec, p, q) của tag quan tâm
        self.det_count = 0
        self.frame_count = 0
        self.cmd = None                    # setpoint đang giữ (do chính bench gửi)
        # Lịch sử setpoint để tra NGƯỢC theo dấu thời gian ảnh. Lúc đứng yên nó
        # bằng self.cmd; lúc đang chạy thì không: giữa lúc chụp ảnh và lúc bench
        # lấy mẫu trôi vài chục ms, ở 40 mm/s là ~2 mm — đủ để đọc nhầm TRỄ
        # thành sai lệch HẰNG, đúng thứ bài test này phải tách ra.
        self.cmd_hist = deque(maxlen=8000)
        self.rack_hist = deque(maxlen=300)   # (stamp, p, q) của tag GIÁ
        self.rack_count = 0

        self.create_subscription(JointState, args.joint_states_topic,
                                 self._on_js, qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray, args.tag_topic,
                                 self._on_tags, qos_profile_sensor_data)
        if getattr(args, "rack_tag_topic", None):
            self.create_subscription(AprilTagDetectionArray, args.rack_tag_topic,
                                     self._on_rack, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float64MultiArray, args.setpoint_topic, 10)

        self.fk, self.fk_src = make_fk()
        self.rows = []
        self.tf_fallback = 0

    # ── callbacks ────────────────────────────────────────────────────────
    def _on_js(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in ARM):
            return
        pos = [msg.position[idx[j]] for j in ARM]
        vel = ([msg.velocity[idx[j]] for j in ARM]
               if len(msg.velocity) == len(msg.name) else [0.0] * 5)
        self.js = (stamp_to_sec(msg.header.stamp), pos, vel)
        self.js_hist.append(self.js)

    def _on_tags(self, msg):
        self.frame_count += 1
        for d in msg.detections:
            if len(d.id) != 1 or d.id[0] != self.args.tag_id:
                continue
            p = d.pose.pose.pose.position
            o = d.pose.pose.pose.orientation
            self.det = (stamp_to_sec(msg.header.stamp),
                        np.array([p.x, p.y, p.z]),
                        np.array([o.x, o.y, o.z, o.w]),
                        msg.header.frame_id or d.pose.header.frame_id)
            self.det_count += 1
            return

    def _on_rack(self, msg):
        for d in msg.detections:
            if len(d.id) != 1 or d.id[0] != self.args.rack_tag_id:
                continue
            p = d.pose.pose.pose.position
            o = d.pose.pose.pose.orientation
            self.rack_hist.append((stamp_to_sec(msg.header.stamp),
                                   np.array([p.x, p.y, p.z]),
                                   np.array([o.x, o.y, o.z, o.w])))
            self.rack_count += 1
            return

    # ── tra cứu ──────────────────────────────────────────────────────────
    def _lookup(self, target, source, stamp_sec):
        """TF target<-source tại stamp ảnh; nếu ngoại suy thì lùi về bản mới nhất
        (đếm vào tf_fallback để báo cáo trung thực)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, Time(seconds=int(stamp_sec),
                                     nanoseconds=int((stamp_sec % 1) * 1e9)),
                timeout=Duration(seconds=0.05))
        except Exception:                                  # noqa: BLE001
            try:
                tf = self.tf_buffer.lookup_transform(target, source, Time(),
                                                     timeout=Duration(seconds=0.2))
                self.tf_fallback += 1
            except Exception as exc:                       # noqa: BLE001
                raise RuntimeError(f"TF {target}<-{source}: {exc}") from exc
        t = tf.transform.translation
        r = tf.transform.rotation
        return make_T(np.array([t.x, t.y, t.z]), np.array([r.x, r.y, r.z, r.w]))

    def _cmd_at(self, stamp_sec):
        """Setpoint ĐANG có hiệu lực lúc CHỤP ẢNH (nội suy tuyến tính)."""
        if not self.cmd_hist:
            return None
        if stamp_sec <= self.cmd_hist[0][0]:
            return list(self.cmd_hist[0][1])
        prev = self.cmd_hist[0]
        for cur in self.cmd_hist:
            if cur[0] >= stamp_sec:
                span = cur[0] - prev[0]
                f = 0.0 if span <= 1e-9 else (stamp_sec - prev[0]) / span
                return [a + (b - a) * f for a, b in zip(prev[1], cur[1])]
            prev = cur
        return list(prev[1])

    def _rack_at(self, stamp_sec):
        """Detection tag GIÁ gần dấu thời gian ảnh nhất. Hai detector đọc CÙNG
        một topic ảnh nên bình thường stamp trùng khít; dt lớn = một trong hai
        bỏ khung."""
        if not self.rack_hist:
            return None
        best = min(self.rack_hist, key=lambda r: abs(r[0] - stamp_sec))
        if abs(best[0] - stamp_sec) > self.args.rack_max_dt:
            return None
        return best

    def _js_at(self, stamp_sec):
        if not self.js_hist:
            return None
        best = min(self.js_hist, key=lambda s: abs(s[0] - stamp_sec))
        return best

    def take_sample(self, label, t0):
        """Một hàng dữ liệu từ detection MỚI NHẤT. None nếu chưa có/quá cũ."""
        det = self.det
        if det is None:
            return None
        stamp, p_cam, q_cam, cam_frame = det
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - stamp > self.args.max_age:
            return None
        try:
            T_ref_cam = self._lookup(self.args.ref_frame, cam_frame, stamp)
            T_ref_ee = self._lookup(self.args.ref_frame, self.args.ee_frame, stamp)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return None
        T_ref_tag = T_ref_cam @ make_T(p_cam, q_cam)

        js = self._js_at(stamp)
        meas = js[1] if js else [float("nan")] * 5
        vel = js[2] if js else [0.0] * 5
        moving = 1 if max(abs(v) for v in vel) > self.args.vel_thresh else 0
        cmd = self._cmd_at(stamp)
        if cmd is None:
            cmd = self.cmd if self.cmd is not None else meas
        fkx, fky, fkz, pitch = self.fk(cmd)

        rk = self._rack_at(stamp)
        rack_cols = ([1, rk[0] - stamp] + list(rk[1]) + list(rk[2])) if rk else \
                    [0, float("nan")] + [float("nan")] * 7

        row = ([now - t0, stamp, label, moving] + list(cmd) + list(meas)
               + list(T_ref_ee[:3, 3]) + list(R_to_quat(T_ref_ee[:3, :3]))
               + list(T_ref_tag[:3, 3]) + list(R_to_quat(T_ref_tag[:3, :3]))
               + [fkx, fky, fkz, pitch]
               + list(p_cam) + list(q_cam) + rack_cols)
        self.rows.append(row)
        return row

    # ── điều khiển ───────────────────────────────────────────────────────
    def send(self, q):
        self.cmd = list(q)
        self.cmd_hist.append((self.get_clock().now().nanoseconds * 1e-9, list(q)))
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q]
        self.pub.publish(msg)

    def creep(self, q0, q1, vel):
        """Trườn setpoint từ q0 tới q1 với tốc độ vel (rad/s) thay vì nhảy bậc.

        hac_moveit.launch.py ép enable_profile=False, nên setpoint bench gửi đi
        là BẬC THANG: HAC dồn hết u_max để đuổi, tay máy tới nơi với quán tính
        đáng kể. Trườn chậm thì lực quán tính ~0 — phần sai số còn lại là thứ
        KHÔNG do động lực học (trọng lực chưa bù hết, ma sát tĩnh, rơ).
        """
        span = max(abs(b - a) for a, b in zip(q0, q1))
        dur = span / max(vel, 1e-6)
        t_start = time.monotonic()
        while rclpy.ok():
            f = min(1.0, (time.monotonic() - t_start) / max(dur, 1e-6))
            self.send([a + (b - a) * f for a, b in zip(q0, q1)])
            rclpy.spin_once(self, timeout_sec=0.02)
            if f >= 1.0:
                return dur

    def hold(self, q, seconds, label=None, sample=False, t0=0.0):
        end = time.monotonic() + seconds
        n = 0
        while time.monotonic() < end and rclpy.ok():
            self.send(q)
            rclpy.spin_once(self, timeout_sec=0.02)
            if sample and self.take_sample(label, t0) is not None:
                n += 1
        return n


# ───────────────────────────── các chế độ chạy ─────────────────────────────

def preflight(node, t0, timeout=20.0):
    """Kiểm tra trước khi động vào robot: có ảnh? có tag? có TF? Trả (ok, thông báo).

    KIÊN NHẪN tới `timeout`: /tf_static là transient_local, listener thường mất
    vài giây mới ghép xong cây TF (cạnh camera→world do static_trans_pub phát ra
    sau cùng). Bản đầu chỉ chờ 5 s nên báo "not part of the same tree" ngay cả
    khi T2 đang chạy đúng — báo động giả làm dừng cả phiên đo.
    """
    deadline = time.monotonic() + timeout
    problems = []
    while time.monotonic() < deadline and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        problems = []
        if node.frame_count == 0:
            problems.append(f"không có message trên {node.args.tag_topic} "
                            "(detector chưa chạy? './rx150.sh eetag')")
        elif node.det is None:
            problems.append(f"detector chạy nhưng KHÔNG thấy tag id={node.args.tag_id} "
                            "(tag khuất/ngoài khung/sai id)")
        if node.js is None:
            problems.append(f"không có {node.args.joint_states_topic} (xs_sdk chưa chạy?)")
        if node.det is not None:
            try:
                node._lookup(node.args.ref_frame, node.det[3], node.det[0])
            except RuntimeError as exc:
                problems.append(f"thiếu TF camera→{node.args.ref_frame}: {exc} "
                                "(chưa chạy './rx150.sh t2' để nạp static_transforms.yaml?)")
            try:
                node._lookup(node.args.ref_frame, node.args.ee_frame, node.det[0])
            except RuntimeError as exc:
                problems.append(f"thiếu TF {node.args.ee_frame}: {exc}")
        if not problems:
            node.tf_fallback = 0          # lần lookup dò tìm không tính vào thống kê
            return True, []
    return False, problems


def grid_poses(radii=(0.16, 0.24, 0.31), heights=(0.06, 0.15, 0.24),
               waists=(-0.5, 0.0, 0.5), pitch_ladder=(0.0, 0.4, 0.8)):
    """Lưới pose sinh bằng IK giải tích, đặt EE vào các điểm (r, waist, z) ĐỊNH TRƯỚC.

    Lý do không liệt kê góc khớp bằng tay: refine chỉ tách được xoay khỏi tịnh
    tiến khi dữ liệu TRẢI RỘNG theo cả ba trục. Bộ CALIB_POSES chọn theo góc
    khớp phủ x vỏn vẹn 50 mm -> hiệu chỉnh pitch lẫn với dịch chuyển. Đặt điểm
    theo toạ độ Cartesian thì kiểm soát được tầm phủ trực tiếp.

    wrist_rotate = 0 (giữ mặt tag ngửa về camera); pitch lấy nấc đầu tiên IK giải
    được để pose ở rìa tầm với không bị loại sạch.
    """
    try:
        from rx150_modules.kinematics import Rx150Kinematics
    except Exception as exc:                                   # noqa: BLE001
        sys.exit(f"pose-set grid cần rx150_modules ({exc})")
    kin = Rx150Kinematics()
    out, seed = {}, None
    for r in radii:
        for z in heights:
            for w in waists:
                q = kin.ik_ladder(r * math.cos(w), r * math.sin(w), z,
                                  pitch_ladder, seed=seed, wrist_rotate=0.0)
                if isinstance(q, tuple):        # (joints, pitch) tuỳ phiên bản
                    q = q[0]
                if not q:
                    continue
                seed = q
                out[f"r{int(r*100)}_z{int(z*100)}_w{int(math.degrees(w))}"] = list(q)
    return out


def keepout_filter(poses, box, margin, label="giá"):
    """Bỏ pose mà TAY GẮP rơi vào hộp cấm nở thêm `margin`.

    Cần vì run_hold nhảy setpoint BẬC THANG giữa các pose: chỉ kiểm điểm đến là
    chưa đủ, nhưng loại sạch điểm đến nằm trong/ngay trên vùng cấm đã chặn được
    kiểu va chạm duy nhất mà lưới hiệu chuẩn gây ra trong thực tế (hạ thấp
    đúng chỗ giá đứng). Đoạn đi ngang vẫn phải tự giữ z đủ cao.
    """
    try:
        from rx150_modules.kinematics import Rx150Kinematics
    except Exception:                                      # noqa: BLE001
        return poses, []
    kin = Rx150Kinematics()
    top = box["z"] + box["size_z"] / 2.0 + margin
    out, dropped = {}, []
    for name, q in poses.items():
        x, y, z, _ = kin.fk(q)
        inside_xy = (abs(x - box["x"]) < box["size_x"] / 2.0 + margin
                     and abs(y - box["y"]) < box["size_y"] / 2.0 + margin)
        if inside_xy and z < top:
            dropped.append(f"{name} ({x:.3f},{y:.3f},{z:.3f})")
            continue
        out[name] = q
    if dropped:
        print(f"[keepout] bỏ {len(dropped)} pose chạm vùng cấm {label} "
              f"(nóc {top-margin:.3f} m + lề {margin:.3f} m):")
        for d in dropped:
            print(f"           {d}")
    return out, dropped


def pose_table(args):
    if args.pose_set == "grid":
        def _f(spec, default, scale=1.0):
            return (tuple(float(v) * scale for v in spec.split(",")) if spec else default)
        base = grid_poses(radii=_f(args.grid_radii, (0.16, 0.24, 0.31)),
                          heights=_f(args.grid_heights, (0.06, 0.15, 0.24)),
                          waists=_f(args.grid_waists_deg, (-0.5, 0.0, 0.5),
                                    math.pi / 180.0))
    elif args.pose_set == "calib":
        base = dict(CALIB_POSES)
    elif args.pose_set == "tagcal":
        base = dict(TAGCAL_POSES)
    elif args.pose_set == "all":
        base = {**DEFAULT_POSES, **CALIB_POSES}
    else:
        base = dict(DEFAULT_POSES)
    if args.poses:
        both = {**DEFAULT_POSES, **CALIB_POSES, **TAGCAL_POSES, **base}
        base = {k: both[k] for k in args.poses.split(",")}
    if getattr(args, "keepout_rack", False):
        import yaml
        path = resolve_config(args.rack_pose, "rack_pose.yaml")
        with open(path) as fh:
            box = (yaml.safe_load(fh) or {}).get("rack_box")
        if box:
            base, _ = keepout_filter(base, box, args.keepout_margin,
                                     label=os.path.basename(path))
        else:
            print(f"[keepout] {path} không có rack_box — bỏ qua bộ lọc")
    return base


def run_hold(node, args, t0):
    poses = pose_table(args)
    print(f"{'pose':<11} {'n':>4}  {'|cmd-enc|':>9} {'|enc-cam|':>9} {'|cmd-cam|':>9}   (mm, thô)")
    node.hold(SLEEP_POSE, 3.0)                     # điểm xuất phát chung
    if args.creep_vel > 0:
        print(f"tiếp cận CHẬM: nhảy tới cách {args.creep_dist:.3f} rad "
              f"(phía {'+' if args.creep_sign > 0 else '-'}), rồi trườn "
              f"{args.creep_vel:.3f} rad/s vào pose "
              f"(~{args.creep_dist/args.creep_vel:.1f} s/pose)")
    for name, q in poses.items():
        if args.creep_vel > 0:
            # Chỉ đoạn CUỐI đi chậm. Trườn cả quãng từ sleep thì mỗi pose mất
            # cả phút mà không thêm thông tin gì — quán tính chỉ quyết định lúc
            # DỪNG. Lệch cùng dấu trên mọi khớp ⇒ chiều tiếp cận xác định được.
            pre = [v - args.creep_sign * args.creep_dist for v in q]
            node.hold(pre, args.dwell)
            node.creep(pre, q, args.creep_vel)
        else:
            node.hold(q, args.dwell)               # đi tới + để Ruckig xong
        n = node.hold(q, args.settle, label=name, sample=True, t0=t0)
        tail = [r for r in node.rows if r[2] == name]
        msg = f"{name:<11} {n:>4}"
        if tail:
            a = np.array(tail, dtype=object)
            enc = np.array([[r[14], r[15], r[16]] for r in tail], dtype=float)
            cam = np.array([[r[21], r[22], r[23]] for r in tail], dtype=float)
            fk = np.array([[r[28], r[29], r[30]] for r in tail], dtype=float)
            msg += (f"  {1000 * np.linalg.norm(fk.mean(0) - enc.mean(0)):9.1f}"
                    f" {1000 * np.linalg.norm(enc.mean(0) - cam.mean(0)):9.1f}"
                    f" {1000 * np.linalg.norm(fk.mean(0) - cam.mean(0)):9.1f}")
        else:
            msg += "   (không thấy tag ở pose này)"
        print(msg)
    node.hold(SLEEP_POSE, 6.0)
    print("\nLưu ý: cột thô ở trên CHƯA trừ offset gắn tag X — số lớn là bình thường.\n"
          "Phần phân tích bên dưới mới là kết quả.")


def run_watch(node, args, t0):
    print(f"watch: ghi {args.duration:.0f}s, KHÔNG ra lệnh gì. Chạy task ở terminal khác. "
          "Ctrl+C để dừng sớm.")
    end = time.monotonic() + args.duration
    last = 0.0
    while time.monotonic() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        if now - last >= 1.0 / args.rate:
            last = now
            node.cmd = None                    # không có lệnh của mình -> cmd = meas
            node.take_sample("watch", t0)
        if len(node.rows) and len(node.rows) % 200 == 0:
            print(f"\r  mẫu: {len(node.rows)}", end="", flush=True)
    print()


def run_sweep(node, args, t0):
    """Quét chậm qua chuỗi pose, ghi liên tục -> so quỹ đạo ĐỘNG (trễ/vọt lố).
    Setpoint chỉ đổi target của Ruckig trong hac_node nên rải setpoint dày là
    an toàn: profile tự lo vận tốc/gia tốc."""
    names = args.poses.split(",") if args.poses else ["home", "reach_far", "reach_low", "home"]
    both = {**DEFAULT_POSES, **CALIB_POSES}
    seq = [both[n] for n in names]
    node.hold(seq[0], args.dwell)
    dt = 1.0 / args.rate
    for cyc in range(args.cycles):
        for i in range(len(seq) - 1):
            a, b = np.array(seq[i]), np.array(seq[i + 1])
            steps = max(2, int(np.max(np.abs(b - a)) / args.sweep_vel * args.rate))
            for k in range(steps + 1):
                q = a + (b - a) * (k / steps)
                node.send(q)
                t_end = time.monotonic() + dt
                while time.monotonic() < t_end and rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.005)
                node.take_sample(f"sweep{cyc}_{names[i]}->{names[i+1]}", t0)
            node.hold(seq[i + 1], args.settle, label=f"sweep{cyc}_{names[i+1]}_hold",
                      sample=True, t0=t0)
        print(f"  chu kỳ {cyc + 1}/{args.cycles} xong, mẫu: {len(node.rows)}")
    node.hold(SLEEP_POSE, 6.0)


# ────────────── quỹ đạo mô phỏng GẮP ỐNG NGHIỆM (mode pick) ─────────────────
# Khác `sweep` ở chỗ: sweep nội suy trong KHÔNG GIAN KHỚP giữa vài pose đặt tay,
# còn mode này đi đường THẲNG TRONG KHÔNG GIAN DESCARTES qua đúng chuỗi điểm mà
# pick_place đi khi gắp: treo trên lỗ -> hạ xuống -> dừng -> nhấc lên -> sang lỗ
# kế. Nhờ vậy sai số đo được quy thẳng ra mm ở đúng chỗ tay gắp phải tới.
#
# Tốc độ lấy từ tube_rack_params.yaml (approach 0.05, descend 0.025 m/s) — đó là
# "chậm vừa" mà bài test cần: đủ chậm để quán tính không chi phối, đủ nhanh để
# trễ của chuỗi ảnh còn hiện ra.
#
# GIỚI HẠN PHẢI BIẾT TRƯỚC: camera treo cao nhìn xuống, mà mặt tag hướng theo
# +z của ee_gripper_link. Gắp thật dùng pitch ~90° (chúc thẳng xuống) ⇒ mặt tag
# quay NGANG, camera nhìn gần như tiếp tuyến và AprilTag đo rất tồi. Nên mỗi
# điểm chốt tự chọn pitch LỚN NHẤT trong ladder mà góc tới mặt tag còn
# ≤ --max-incidence. Hình dạng đường đi giữ nguyên; chỉ độ chúc cổ tay giảm.


def _share_config(name):
    """Đường dẫn file config trong rx150_perception (install/share)."""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("rx150_perception"), "config", name)


def resolve_config(path, default_name=None):
    """Tên trần -> file trong share của rx150_perception; đường dẫn thì giữ nguyên."""
    if path is None:
        path = default_name
    if path is None:
        return None
    if os.path.sep in path or os.path.exists(path):
        return path
    try:
        return _share_config(path)
    except Exception:                                      # noqa: BLE001
        return path


def load_rack(path):
    """rack_pose.yaml -> miệng 4 lỗ trong rx150/base_link (do camera đo)."""
    import yaml
    with open(path) as fh:
        d = yaml.safe_load(fh)
    if not d or "slots" not in d:
        sys.exit(f"{path} thiếu khoá 'slots' — chạy './rx150.sh rack-calib' trước.")
    m = d.get("measurement") or {}
    rp = d.get("rack_pose") or {}
    return {"path": path,
            "slots": [np.array(s, float) for s in d["slots"]],
            "tag_xyz": np.array([rp.get("x", float("nan")), rp.get("y", float("nan")),
                                 rp.get("z", float("nan"))], float),
            "tilt_deg": float(d.get("rack_tilt_deg", 0.0)),
            "at": m.get("calibrated_at", "?"), "tag_id": m.get("tag_id", "?")}


def tag_incidence(q, kin, X, cam_pos):
    """Góc giữa PHÁP TUYẾN mặt tag và hướng nhìn của camera, độ.

    0° = camera nhìn vuông góc mặt tag (đo tốt nhất); 90° = nhìn tiếp tuyến
    (apriltag hoặc không thấy, hoặc cho pose lật nghiêng đầy nhiễu).
    """
    R = R_ee_of(q)
    p_tag = np.array(kin.fk(q)[:3]) + R @ X[:3, 3]
    n = R @ X[:3, :3] @ np.array([0.0, 0.0, 1.0])
    v = cam_pos - p_tag
    nv = float(np.linalg.norm(v))
    if nv < 1e-6:
        return 90.0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(n @ v) / nv))))


def _pitch_for(kin, xyz, ladder, wrist, seed, cam_pos, X, max_inc):
    """pitch LỚN NHẤT trong ladder vừa IK được vừa còn nhìn thấy tag.

    Trả (q, pitch, góc tới, thấy_tag). Khi không pitch nào thoả góc nhìn thì
    trả nghiệm IK đầu tiên kèm thấy_tag=False để lượt kiểm khô còn báo được.
    """
    fallback = None
    for p in ladder:
        q = kin.ik(float(xyz[0]), float(xyz[1]), float(xyz[2]), float(p),
                   seed=seed, wrist_rotate=wrist)
        if q is None:
            continue
        inc = tag_incidence(q, kin, X, cam_pos)
        if inc <= max_inc:
            return q, float(p), inc, True
        if fallback is None:
            fallback = (q, float(p), inc)
    if fallback is None:
        return None, None, None, False
    return fallback[0], fallback[1], fallback[2], False


def build_pick_path(args, kin, rack, cam_pos, X, q_start):
    """Chuỗi điểm chốt: (treo -> hạ -> dừng -> nhấc) cho từng lỗ, lặp --cycles."""
    ladder = [float(v) for v in args.pitch_ladder.split(",")]
    slots = ([int(v) for v in args.slots.split(",")] if args.slots
             else list(range(len(rack["slots"]))))
    keys, problems = [], []
    seed = list(q_start)
    for cyc in range(args.cycles):
        for i in slots:
            if i < 0 or i >= len(rack["slots"]):
                problems.append(f"slot {i}: ngoài danh sách ({len(rack['slots'])} lỗ)")
                continue
            mouth = rack["slots"][i]
            hov = mouth + np.array([0.0, 0.0, args.hover])
            low = mouth + np.array([0.0, 0.0, args.clearance])
            for tag, xyz, dwell, vel in (
                    ("hover", hov, args.dwell_hover, args.cart_vel),
                    ("low", low, args.dwell_point, args.descend_vel),
                    ("lift", hov, args.dwell_hover, args.descend_vel)):
                lbl = f"c{cyc}_{tag}{i}"
                if float(xyz[2]) < args.min_z:
                    problems.append(f"{lbl}: z={1000*xyz[2]:.0f} mm < min_z="
                                    f"{1000*args.min_z:.0f} mm")
                q, pit, inc, vis = _pitch_for(kin, xyz, ladder, args.wrist_rotate,
                                              seed, cam_pos, X, args.max_incidence)
                if q is None:
                    problems.append(f"{lbl}: IK FAIL tại ({xyz[0]:.3f}, {xyz[1]:.3f}, "
                                    f"{xyz[2]:.3f}) với ladder {ladder}")
                    continue
                if not vis:
                    problems.append(f"{lbl}: mặt tag nghiêng {inc:.0f}° so với camera "
                                    f"(ngưỡng {args.max_incidence:.0f}°) — camera sẽ đo tồi")
                keys.append({"label": lbl, "xyz": xyz, "q": q, "pitch": pit,
                             "inc": inc, "dwell": dwell, "vel": vel, "vis": vis,
                             "slot": i})
                seed = q
    return keys, problems


def segment_points(args, a, b):
    """(xyz, pitch, dt) dọc một đoạn thẳng. smoothstep ⇒ vận tốc bắt đầu và kết
    thúc bằng 0 (đỉnh = 1.5 × vận tốc trung bình), vì controller chạy với
    enable_profile=False nên KHÔNG có ai làm mượt hộ."""
    L = float(np.linalg.norm(b["xyz"] - a["xyz"]))
    dur = max(L / max(b["vel"], 1e-4), args.min_seg_time)
    n = max(2, int(round(dur * args.rate)))
    for k in range(1, n + 1):
        u = k / n
        s = u * u * (3.0 - 2.0 * u)
        yield (a["xyz"] + (b["xyz"] - a["xyz"]) * s,
               a["pitch"] + (b["pitch"] - a["pitch"]) * s,
               dur / n)


def dry_check(args, kin, keys, cam_pos, X):
    """Chạy KHÔ toàn tuyến trước khi động cơ nhúc nhích: IK từng bước, bước khớp
    lớn nhất, góc tới mặt tag nhỏ nhất. Rẻ, và bắt được đúng những lỗi mà lúc
    đang chạy thì đã muộn."""
    problems, seed = [], keys[0]["q"]
    n_pts, dur, max_step, worst_inc, z_min = 0, 0.0, 0.0, 0.0, 9.9
    for a, b in zip(keys, keys[1:]):
        for xyz, pit, dt in segment_points(args, a, b):
            q = kin.ik(float(xyz[0]), float(xyz[1]), float(xyz[2]), pit,
                       seed=seed, wrist_rotate=args.wrist_rotate)
            if q is None:
                problems.append(f"{a['label']}->{b['label']}: IK FAIL giữa đoạn tại "
                                f"({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) pitch "
                                f"{math.degrees(pit):.0f}°")
                break
            step = max(abs(x - y) for x, y in zip(q, seed))
            if step > max_step:
                max_step = step
            worst_inc = max(worst_inc, tag_incidence(q, kin, X, cam_pos))
            z_min = min(z_min, float(xyz[2]))
            seed, n_pts, dur = q, n_pts + 1, dur + dt
    if max_step > args.max_joint_step:
        problems.append(f"bước khớp lớn nhất {max_step:.3f} rad > --max-joint-step "
                        f"{args.max_joint_step:.3f} (lật nhánh IK?) — giảm --cart-vel "
                        f"hoặc tăng --rate")
    return problems, {"points": n_pts, "duration_s": dur, "max_joint_step_rad": max_step,
                      "worst_incidence_deg": worst_inc, "z_min_m": z_min}


def run_pick(node, args, t0):
    """Đi hết tuyến, ghi liên tục. Điểm dừng có nhãn 'dwell:', đoạn chạy 'move:'."""
    from rx150_modules.kinematics import Rx150Kinematics
    kin = Rx150Kinematics()

    xpath = resolve_config(args.tag_offset, None)
    X = load_tag_offset(xpath) if xpath else np.eye(4)
    if not args.tag_offset:
        print("[pick] ⚠ KHÔNG có --tag-offset: offset gắn tag sẽ bị fit từ chính dữ liệu,\n"
              "       mọi sai lệch HẰNG bị hấp thụ và câu hỏi 'có offset không' KHÔNG trả lời được.")

    det = node.det
    T_ref_cam = node._lookup(args.ref_frame, det[3], det[0])
    cam_pos = T_ref_cam[:3, 3]
    rack = load_rack(resolve_config(args.rack_pose, "rack_pose.yaml"))
    print(f"\nGiá: {rack['path']}\n  đo lúc {rack['at']} · tag id {rack['tag_id']} · "
          f"nghiêng {rack['tilt_deg']:.1f}°")
    print(f"Camera ở ({cam_pos[0]:.3f}, {cam_pos[1]:.3f}, {cam_pos[2]:.3f}) trong "
          f"{args.ref_frame}")

    q_start = list(node.js[1])
    keys, problems = build_pick_path(args, kin, rack, cam_pos, X, q_start)
    if not keys:
        sys.exit("không dựng được điểm chốt nào — xem lỗi bên trên")

    print(f"\nĐiểm chốt ({len(keys)}), miệng lỗ + treo {1000*args.hover:.0f} mm, "
          f"điểm thấp nhất cách miệng {1000*args.clearance:.0f} mm:")
    print(f"  {'điểm':<12} {'x':>7} {'y':>7} {'z':>7} {'r':>6} {'pitch':>6} "
          f"{'góc tag':>8} {'dừng':>5}")
    for k in keys:
        r = math.hypot(k["xyz"][0], k["xyz"][1])
        flag = "" if k["vis"] else "  ⚠tag khuất"
        print(f"  {k['label']:<12} {k['xyz'][0]:7.3f} {k['xyz'][1]:7.3f} "
              f"{k['xyz'][2]:7.3f} {r:6.3f} {math.degrees(k['pitch']):6.1f} "
              f"{k['inc']:7.1f}° {k['dwell']:5.1f}{flag}")

    dry, stats = dry_check(args, kin, keys, cam_pos, X)
    problems += dry
    print(f"\nKiểm khô: {stats['points']} điểm setpoint, ~{stats['duration_s']:.0f} s chạy, "
          f"bước khớp lớn nhất {stats['max_joint_step_rad']:.3f} rad, "
          f"góc tới mặt tag xấu nhất {stats['worst_incidence_deg']:.0f}°, "
          f"z thấp nhất {1000*stats['z_min_m']:.0f} mm")
    if problems:
        print("\nVẤN ĐỀ:")
        for p in problems:
            print(f"  • {p}")
        if not args.force:
            sys.exit("\nDừng TRƯỚC khi cử động. Sửa rồi chạy lại, hoặc --force nếu "
                     "biết mình làm gì.")
        print("\n--force: chạy tiếp bất chấp.")

    total = stats["duration_s"] + sum(k["dwell"] for k in keys) + 2 * args.approach_time
    print(f"\n⏱  tổng khoảng {total:.0f} s. Bắt đầu.\n")

    # Từ tư thế ĐANG ĐỨNG tới điểm chốt đầu: đi trong không gian KHỚP, chậm.
    # Đi Descartes từ chỗ bất kỳ có thể quét qua chính cái giá.
    node.creep(q_start, keys[0]["q"], args.joint_vel)
    node.hold(keys[0]["q"], args.dwell_hover, label=f"dwell:{keys[0]['label']}",
              sample=True, t0=t0)

    prev = keys[0]
    for k in keys[1:]:
        label = f"move:{prev['label']}->{k['label']}"
        seed = prev["q"]
        for xyz, pit, dt in segment_points(args, prev, k):
            q = kin.ik(float(xyz[0]), float(xyz[1]), float(xyz[2]), pit,
                       seed=seed, wrist_rotate=args.wrist_rotate)
            if q is None:
                continue
            seed = q
            node.send(q)
            t_end = time.monotonic() + dt
            while time.monotonic() < t_end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.005)
            node.take_sample(label, t0)
        n = node.hold(k["q"], k["dwell"], label=f"dwell:{k['label']}", sample=True, t0=t0)
        print(f"  {label:<34} -> dừng {k['label']:<12} {n:4d} mẫu "
              f"(tổng {len(node.rows)})")
        prev = k

    print("  về SLEEP…")
    node.creep(list(node.js[1]), SLEEP_POSE, args.joint_vel)
    node.hold(SLEEP_POSE, 3.0)


# ─────────────────────────────── phân tích ─────────────────────────────────

def merge_rows(paths):
    """Gộp nhiều lượt đo. Nhãn pose được thêm tiền tố theo lượt.

    Vì sao KHÔNG gộp chung nhãn: mỗi lượt có động lực học riêng (bậc thang, trườn
    chậm chiều dương, chiều âm), nên cùng một tên pose ở hai lượt là hai quan sát
    ĐỘC LẬP — giữ riêng thì số nhóm tăng gấp đôi/gấp ba, mà kiểm tra chéo và
    jackknife đều tính theo NHÓM. Gộp chung nhãn chỉ làm mịn dữ liệu chứ không
    làm chặt thêm khoảng tin cậy.
    """
    if len(paths) == 1:
        return load_rows(paths[0])
    out = []
    for i, p in enumerate(paths):
        rows = load_rows(p)
        tag = os.path.basename(os.path.dirname(os.path.abspath(p))) or f"r{i}"
        for r in rows:
            r["label"] = f"{i}:{r['label']}"
        out += rows
        print(f"  + {len(rows):5d} mẫu từ {tag}")
    print(f"  = {len(out)} mẫu từ {len(paths)} lượt")
    return out


def load_rows(path):
    with open(path) as fh:
        rd = csv.DictReader(fh)
        return list(rd)


def analyze(rows, args, meta=None):
    """rows: list dict (từ CSV) hoặc list list (đang chạy). Trả dict summary."""
    if rows and isinstance(rows[0], list):
        rows = [dict(zip(CSV_HEADER, r)) for r in rows]
    if not rows:
        return {"error": "không có mẫu nào"}

    def vec(r, pre):
        return np.array([float(r[f"{pre}_x"]), float(r[f"{pre}_y"]), float(r[f"{pre}_z"])])

    def quat(r, pre):
        return np.array([float(r[f"{pre}_qx"]), float(r[f"{pre}_qy"]),
                         float(r[f"{pre}_qz"]), float(r[f"{pre}_qw"])])

    static = [r for r in rows if int(float(r["moving"])) == 0]
    fit_rows = static if len(static) >= args.min_static else rows
    if len(static) < args.min_static:
        sys.stderr.write(f"[warn] chỉ {len(static)} mẫu đứng yên (<{args.min_static}); "
                         "fit offset tag trên TOÀN BỘ mẫu — kết quả kém tin cậy\n")

    # X = T(ee -> tag), trung vị vị trí + trung bình quaternion trên mẫu tĩnh
    Xs = [T_inv(make_T(vec(r, "enc"), quat(r, "enc"))) @ make_T(vec(r, "cam"), quat(r, "cam"))
          for r in fit_rows]
    X_fixed = resolve_config(getattr(args, "tag_offset", None), None)
    if X_fixed:
        # X đo sẵn bằng mode tagoffset. Fit tại chỗ HẤP THỤ mọi sai số hằng vào
        # X, nên số báo ra chỉ là độ KHÔNG NHẤT QUÁN. Đưa X cố định vào thì
        # enc→cam trở lại là sai lệch TUYỆT ĐỐI — đó là thứ pick_place chịu.
        X = load_tag_offset(X_fixed)
        spread = np.array([x[:3, 3] for x in Xs]) - X[:3, 3]
    else:
        X = np.eye(4)
        X[:3, 3] = np.median(np.array([x[:3, 3] for x in Xs]), axis=0)
        X[:3, :3] = quat_to_R(quat_mean([R_to_quat(x[:3, :3]) for x in Xs]))
        spread = np.array([x[:3, 3] for x in Xs]) - X[:3, 3]

    # cam -> vị trí ee_gripper_link theo camera
    out = []
    for r in rows:
        T_enc = make_T(vec(r, "enc"), quat(r, "enc"))
        T_cam_ee = make_T(vec(r, "cam"), quat(r, "cam")) @ T_inv(X)
        fk = np.array([float(r["fk_x"]), float(r["fk_y"]), float(r["fk_z"])])
        out.append({
            "t": float(r["t"]), "label": r["label"], "moving": int(float(r["moving"])),
            "enc": T_enc[:3, 3], "cam": T_cam_ee[:3, 3], "cmd": fk,
            "d_ang": math.degrees(angle_between_R(T_enc[:3, :3], T_cam_ee[:3, :3])),
        })

    def stats(pairs):
        d = np.array(pairs)
        n = np.linalg.norm(d, axis=1)
        return {
            "bias_mm": (1000 * d.mean(axis=0)).tolist(),
            "rms_mm": float(1000 * np.sqrt((n ** 2).mean())),
            "max_mm": float(1000 * n.max()),
            "std_axis_mm": (1000 * d.std(axis=0)).tolist(),
            "n": len(d),
        }

    stat_rows = [o for o in out if o["moving"] == 0] or out
    summary = {
        "n_samples": len(out), "n_static": len(stat_rows),
        "tag_offset_X": {
            "xyz_m": X[:3, 3].tolist(),
            "quat_xyzw": R_to_quat(X[:3, :3]).tolist(),
            "fit_spread_mm": (1000 * spread.std(axis=0)).tolist(),
            "source": X_fixed or "fit",
        },
        "cmd_vs_enc": stats([o["cmd"] - o["enc"] for o in stat_rows]),
        "enc_vs_cam": stats([o["enc"] - o["cam"] for o in stat_rows]),
        "cmd_vs_cam": stats([o["cmd"] - o["cam"] for o in stat_rows]),
        "orient_enc_vs_cam_deg": {
            "mean": float(np.mean([o["d_ang"] for o in stat_rows])),
            "max": float(np.max([o["d_ang"] for o in stat_rows])),
        },
        "per_label": {},
    }
    for lab in sorted({o["label"] for o in stat_rows}):
        sel = [o for o in stat_rows if o["label"] == lab]
        summary["per_label"][lab] = {
            "n": len(sel),
            "enc_vs_cam_mm": (1000 * np.mean([o["enc"] - o["cam"] for o in sel], axis=0)).tolist(),
            "cmd_vs_cam_mm": (1000 * np.mean([o["cmd"] - o["cam"] for o in sel], axis=0)).tolist(),
        }
    if meta:
        summary["meta"] = meta
    return summary, out


def print_summary(s):
    def line(name, d):
        b = d["bias_mm"]
        print(f"  {name:<10} bias=({b[0]:+6.1f},{b[1]:+6.1f},{b[2]:+6.1f})  "
              f"RMS={d['rms_mm']:6.2f}  max={d['max_mm']:6.2f}  (n={d['n']})")

    X = s["tag_offset_X"]
    sp = X["fit_spread_mm"]
    src = X.get("source", "fit")
    if src != "fit":
        print(f"\nOffset gắn tag X = T(ee_gripper_link -> tag), CỐ ĐỊNH từ {src}"
              f"\n  (enc→cam dưới đây là sai lệch TUYỆT ĐỐI, không phải chỉ độ "
              f"không nhất quán):")
    else:
        print(f"\nOffset gắn tag X = T(ee_gripper_link -> tag), ước lượng từ dữ liệu:")
    print(f"  xyz = ({X['xyz_m'][0]*1000:+.1f}, {X['xyz_m'][1]*1000:+.1f}, "
          f"{X['xyz_m'][2]*1000:+.1f}) mm   độ tản khi fit = "
          f"({sp[0]:.1f}, {sp[1]:.1f}, {sp[2]:.1f}) mm")
    print(f"\nSai lệch vị trí ee_gripper_link (mm, khung base_link, "
          f"{s['n_static']}/{s['n_samples']} mẫu đứng yên):")
    line("cmd→enc", s["cmd_vs_enc"])
    line("enc→cam", s["enc_vs_cam"])
    line("cmd→cam", s["cmd_vs_cam"])
    o = s["orient_enc_vs_cam_deg"]
    print(f"  hướng enc→cam: trung bình {o['mean']:.2f}°, lớn nhất {o['max']:.2f}°")
    if s["per_label"]:
        print("\nTheo từng pose (trung bình, mm):")
        print(f"  {'pose':<22} {'enc→cam (x,y,z)':>26} {'cmd→cam (x,y,z)':>26}")
        for lab, d in s["per_label"].items():
            e, c = d["enc_vs_cam_mm"], d["cmd_vs_cam_mm"]
            print(f"  {lab:<22} ({e[0]:+6.1f},{e[1]:+6.1f},{e[2]:+6.1f})"
                  f"        ({c[0]:+6.1f},{c[1]:+6.1f},{c[2]:+6.1f})")
    sd = s["enc_vs_cam"]["std_axis_mm"]
    tail = ("đây là phần PHỤ THUỘC POSE (bias hằng đã bị X hấp thụ)."
            if src == "fit" else "X cố định nên bias hằng NẰM TRONG cột bias ở trên.")
    print(f"\nĐộ tản enc→cam theo trục: ({sd[0]:.1f}, {sd[1]:.1f}, {sd[2]:.1f}) mm — {tail}")


def plot(out, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.array([o["t"] for o in out])
    fig, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=False)
    for i, axis in enumerate("xyz"):
        ax[i].plot(t, [o["cmd"][i] * 1000 for o in out], label="cmd (FK setpoint)", lw=1.0)
        ax[i].plot(t, [o["enc"][i] * 1000 for o in out], label="enc (TF joint_states)", lw=1.0)
        ax[i].plot(t, [o["cam"][i] * 1000 for o in out], ".", label="cam (AprilTag)", ms=2.5)
        ax[i].set_ylabel(f"{axis} [mm]")
        ax[i].grid(alpha=0.3)
    ax[0].legend(loc="upper right", fontsize=8)
    ax[0].set_title("Quỹ đạo ee_gripper_link: lệnh vs encoder vs camera")
    ax[3].plot([o["enc"][0] * 1000 for o in out], [o["enc"][1] * 1000 for o in out],
               label="enc", lw=1.0)
    ax[3].plot([o["cam"][0] * 1000 for o in out], [o["cam"][1] * 1000 for o in out],
               ".", label="cam", ms=2.5)
    ax[3].set_xlabel("x [mm]"), ax[3].set_ylabel("y [mm]")
    ax[3].set_title("Nhìn từ trên (XY)"), ax[3].legend(fontsize=8), ax[3].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"đồ thị: {path}")



# ────────────────────── hiệu chuẩn lại TF camera (refine) ──────────────────
# Nếu residual enc→cam là một HÀM TUYẾN TÍNH CỦA VỊ TRÍ (dy tỉ lệ với z, v.v.)
# thì thủ phạm không phải cơ khí robot mà là phép quay trong TF camera: một
# lần Snap Pose của armtag chỉ có một tư thế để giải, sai vài độ là thường.
# Tag gắn trên tay gắp cho ta HÀNG NGHÌN tư thế -> giải lại được cả extrinsic.
#
# Mô hình:  T_ref_cam · p_tag(camera)_i  =  R_ee_i · t_X + p_ee_i
# Ẩn: ω (rotvec của R_ref_cam, 3) + t_ref_cam (3) + t_X (3) = 9 tham số.
# Hướng của tag không vào residual vị trí nên không cần R_X.

def _kabsch(P, Q):
    """R, t sao cho R·P + t ≈ Q (hai đám mây điểm tương ứng)."""
    cp, cq = P.mean(axis=0), Q.mean(axis=0)
    H = (P - cp).T @ (Q - cq)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, cq - R @ cp


def _solve_tX(R_bc, t_bc, P_raw, R_ee, P_ee):
    """t_X tối ưu (tuyến tính) khi extrinsic đã cố định — dùng cho mốc 'trước'."""
    lhs = (R_bc @ P_raw.T).T + t_bc - P_ee
    A = R_ee.reshape(-1, 3)
    return np.linalg.lstsq(A, lhs.reshape(-1), rcond=None)[0]


def _fit_extrinsic(use, tX_fixed=None):
    """Giải (ω, t_ref_cam, t_X) bằng least-squares. Trả (params, R0, t0).

    tX_fixed: offset gắn tag đã đo riêng (mode tagoffset, bài AX=ZB). Cố định nó
    thì chỉ còn 6 ẩn — QUAN TRỌNG vì tịnh tiến của extrinsic và t_X đổi chác
    gần như 1:1 khi cả hai cùng tự do: fit 9 ẩn luôn ra residual đẹp nhưng chia
    phần sai lệch tuỳ ý giữa hai bên, nên không dùng để kết luận 'lệch offset'.
    """
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot
    P_raw, P_ref, P_ee, R_ee = _refine_arrays(use)
    R0, t0 = _kabsch(P_raw, P_ref)
    tX0 = (np.asarray(tX_fixed, float) if tX_fixed is not None
           else _solve_tX(R0, t0, P_raw, R_ee, P_ee))

    if tX_fixed is None:
        def resid(prm):
            R = Rot.from_rotvec(prm[0:3]).as_matrix()
            return ((R @ P_raw.T).T + prm[3:6]
                    - np.einsum("nij,j->ni", R_ee, prm[6:9]) - P_ee).ravel()

        x0 = np.concatenate([Rot.from_matrix(R0).as_rotvec(), t0, tX0])
        return least_squares(resid, x0, method="lm", max_nfev=20000).x, R0, t0

    off = np.einsum("nij,j->ni", R_ee, tX0) + P_ee

    def resid6(prm):
        R = Rot.from_rotvec(prm[0:3]).as_matrix()
        return ((R @ P_raw.T).T + prm[3:6] - off).ravel()

    x0 = np.concatenate([Rot.from_matrix(R0).as_rotvec(), t0])
    sol = least_squares(resid6, x0, method="lm", max_nfev=20000).x
    return np.concatenate([sol, tX0]), R0, t0


def _refine_arrays(use):
    P_raw = np.array([[float(r["raw_x"]), float(r["raw_y"]), float(r["raw_z"])] for r in use])
    P_ref = np.array([[float(r["cam_x"]), float(r["cam_y"]), float(r["cam_z"])] for r in use])
    P_ee = np.array([[float(r["enc_x"]), float(r["enc_y"]), float(r["enc_z"])] for r in use])
    R_ee = np.array([quat_to_R([float(r["enc_qx"]), float(r["enc_qy"]),
                                float(r["enc_qz"]), float(r["enc_qw"])]) for r in use])
    return P_raw, P_ref, P_ee, R_ee


def _eval_params(prm, use):
    """Residual (m) của một bộ tham số trên tập mẫu bất kỳ."""
    from scipy.spatial.transform import Rotation as Rot
    P_raw, _, P_ee, R_ee = _refine_arrays(use)
    return ((Rot.from_rotvec(prm[0:3]).as_matrix() @ P_raw.T).T + prm[3:6]
            - np.einsum("nij,j->ni", R_ee, prm[6:9]) - P_ee)


def _rms_mm(d):
    return float(1000 * np.sqrt((np.linalg.norm(d, axis=1) ** 2).mean()))


def cross_validate(use, tX_fixed=None):
    """Chia POSE (không phải mẫu) làm hai nửa, fit nửa này đo nửa kia.

    Bắt buộc phải chia theo pose: mẫu trong cùng một pose gần như trùng nhau,
    chia ngẫu nhiên theo mẫu thì tập kiểm tra không độc lập và mọi hiệu chỉnh
    đều 'tốt lên' — kể cả khi chỉ đang khớp nhiễu.
    """
    from scipy.spatial.transform import Rotation as Rot
    labels = sorted({r["label"] for r in use})
    if len(labels) < 6:
        return None
    halves = ([r for r in use if labels.index(r["label"]) % 2 == 0],
              [r for r in use if labels.index(r["label"]) % 2 == 1])
    out = []
    for train, test in (halves, halves[::-1]):
        prm, R0, t0 = _fit_extrinsic(train, tX_fixed)
        P_raw, _, P_ee, R_ee = _refine_arrays(test)
        old = np.concatenate([Rot.from_matrix(R0).as_rotvec(), t0,
                              np.asarray(tX_fixed, float) if tX_fixed is not None
                              else _solve_tX(R0, t0, P_raw, R_ee, P_ee)])
        ang = math.degrees(np.linalg.norm(Rot.from_matrix(
            R0.T @ Rot.from_rotvec(prm[0:3]).as_matrix()).as_rotvec()))
        out.append({"rms_old_mm": _rms_mm(_eval_params(old, test)),
                    "rms_new_mm": _rms_mm(_eval_params(prm, test)),
                    "rot_deg": ang})
    return out


def residual_structure(d, P_ee):
    """Dư sau refine còn phụ thuộc tầm với r / độ cao z không? Nếu có thì đó là
    thứ CƠ KHÍ (võng khi vươn xa), không phải hiệu chuẩn — encoder mù với nó."""
    r = np.hypot(P_ee[:, 0], P_ee[:, 1])
    A = np.c_[r, P_ee[:, 2], np.ones(len(P_ee))]
    out = {}
    for i, ax in enumerate("xyz"):
        c, *_ = np.linalg.lstsq(A, d[:, i] * 1000, rcond=None)
        out[ax] = {"d_dr_mm_per_m": float(c[0]), "d_dz_mm_per_m": float(c[1]),
                   "left_mm": float((d[:, i] * 1000 - A @ c).std()),
                   "raw_mm": float((d[:, i] * 1000).std())}
    return out


def refine(rows, args, out_dir=None):
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot

    if rows and isinstance(rows[0], list):
        rows = [dict(zip(CSV_HEADER, r)) for r in rows]
    if not rows or "raw_x" not in rows[0]:
        sys.exit("CSV thiếu cột raw_* (ghi bằng bản bench cũ) — cần đo lại.")
    use = [r for r in rows if int(float(r["moving"])) == 0] or rows
    P_raw = np.array([[float(r["raw_x"]), float(r["raw_y"]), float(r["raw_z"])] for r in use])
    P_ref = np.array([[float(r["cam_x"]), float(r["cam_y"]), float(r["cam_z"])] for r in use])
    P_ee = np.array([[float(r["enc_x"]), float(r["enc_y"]), float(r["enc_z"])] for r in use])
    R_ee = np.array([quat_to_R([float(r["enc_qx"]), float(r["enc_qy"]),
                                float(r["enc_qz"]), float(r["enc_qw"])]) for r in use])

    # extrinsic ĐANG DÙNG: suy ngược từ chính dữ liệu (cùng một TF áp cho mọi mẫu)
    R0, t0 = _kabsch(P_raw, P_ref)
    tX_path = resolve_config(getattr(args, "tag_offset", None), None)
    tX_fix = load_tag_offset(tX_path)[:3, 3] if tX_path else None
    tX0 = tX_fix if tX_fix is not None else _solve_tX(R0, t0, P_raw, R_ee, P_ee)
    if tX_fix is not None:
        print(f"\noffset gắn tag CỐ ĐỊNH từ {tX_path}: "
              f"({', '.join(f'{v:+.2f}' for v in 1000 * tX_fix)}) mm ⇒ chỉ giải 6 ẩn "
              f"extrinsic.")

    x0 = np.concatenate([Rot.from_matrix(R0).as_rotvec(), t0, tX0])
    before = _eval_params(x0, use)
    prm, _, _ = _fit_extrinsic(use, tX_fix)
    after = _eval_params(prm, use)

    R1 = Rot.from_rotvec(prm[0:3]).as_matrix()
    t1, tX1 = prm[3:6], prm[6:9]
    dR = Rot.from_matrix(R0.T @ R1)
    drpy = np.degrees(dR.as_euler("xyz"))

    rms = _rms_mm

    print(f"\nHiệu chuẩn lại TF camera trên {len(use)} mẫu đứng yên "
          f"({len({r['label'] for r in use})} nhóm pose):")
    print(f"  residual TRƯỚC (extrinsic đang dùng, t_X tối ưu): {rms(before):6.2f} mm RMS"
          f"   theo trục ({', '.join(f'{v:.2f}' for v in 1000*before.std(axis=0))})")
    print(f"  residual SAU  (giải lại cả extrinsic + t_X):      {rms(after):6.2f} mm RMS"
          f"   theo trục ({', '.join(f'{v:.2f}' for v in 1000*after.std(axis=0))})")
    print(f"  hiệu chỉnh xoay:  roll {drpy[0]:+.2f}°  pitch {drpy[1]:+.2f}°  yaw {drpy[2]:+.2f}°"
          f"   (tổng {math.degrees(np.linalg.norm(dR.as_rotvec())):.2f}°)")
    print(f"  hiệu chỉnh tịnh tiến: ({', '.join(f'{v:+.1f}' for v in 1000*(t1-t0))}) mm")
    print(f"  offset tag t_X: ({', '.join(f'{v:+.1f}' for v in 1000*tX1)}) mm "
          f"(trước: {', '.join(f'{v:+.1f}' for v in 1000*tX0)})")

    span = P_ee.max(axis=0) - P_ee.min(axis=0)
    narrow = " ⚠ HẸP (<100 mm): hiệu chỉnh xoay lẫn với tịnh tiến" if span.min() < 0.10 else ""
    print(f"  tầm phủ của dữ liệu: ({', '.join(f'{v*1000:.0f}' for v in span)}) mm theo x,y,z{narrow}")

    cv = cross_validate(use, tX_fix)
    if cv:
        print("\n  Kiểm tra chéo (fit nửa số pose, đo trên nửa CHƯA thấy):")
        for i, c in enumerate(cv):
            print(f"    lần {i+1}: hiệu chuẩn cũ {c['rms_old_mm']:5.2f} mm  →  mới "
                  f"{c['rms_new_mm']:5.2f} mm   (xoay {c['rot_deg']:.2f}°)")
        spread = abs(cv[0]["rot_deg"] - cv[1]["rot_deg"])
        biggest = max(c["rot_deg"] for c in cv)
        gain = np.mean([c["rms_old_mm"] - c["rms_new_mm"] for c in cv])
        if biggest < 0.6 or gain <= 0.0:
            # Cả hai nửa đều đòi sửa gần như KHÔNG, hoặc sửa xong lại tệ đi trên
            # pose chưa thấy: hiệu chuẩn hiện tại đã đúng, fit thêm chỉ là khớp nhiễu.
            verdict = ("hiệu chuẩn hiện tại ĐÃ ỔN — hiệu chỉnh còn lại chỉ là nhiễu, "
                       "đừng áp dụng")
        elif spread < 1.0:
            verdict = "hai nửa cho cùng một hiệu chỉnh ⇒ lệch THẬT, không phải khớp nhiễu"
        else:
            verdict = "hai nửa LỆCH NHAU nhiều ⇒ dữ liệu chưa đủ, đừng áp dụng"
        print(f"    chênh giữa hai lần: {spread:.2f}°, lợi trung bình {gain:+.2f} mm "
              f"— {verdict}")
    else:
        cv = []

    struct = residual_structure(after, P_ee)
    print("\n  Dư SAU refine còn phụ thuộc tư thế không (thứ encoder KHÔNG thấy):")
    for ax, v in struct.items():
        print(f"    d{ax}: theo tầm với {v['d_dr_mm_per_m']:+6.1f} mm/m, "
              f"theo độ cao {v['d_dz_mm_per_m']:+6.1f} mm/m   "
              f"còn lại {v['left_mm']:.2f} mm (thô {v['raw_mm']:.2f})")

    # đề xuất static_transforms.yaml (camera_color_optical_frame -> world).
    # Chỉ đúng khi ref_frame trùng world (world->rx150/base_link là identity trên
    # rx150) — nếu đổi ref_frame khác thì tự quy đổi, đừng chép mù.
    T_ref_cam = np.eye(4); T_ref_cam[:3, :3] = R1; T_ref_cam[:3, 3] = t1
    T_cam_ref = T_inv(T_ref_cam)
    q = R_to_quat(T_cam_ref[:3, :3])
    entry = {"frame_id": "camera_color_optical_frame", "child_frame_id": "world",
             "x": float(T_cam_ref[0, 3]), "y": float(T_cam_ref[1, 3]),
             "z": float(T_cam_ref[2, 3]), "qx": float(q[0]), "qy": float(q[1]),
             "qz": float(q[2]), "qw": float(q[3])}
    if out_dir:
        import yaml
        path = os.path.join(out_dir, "static_transforms_refined.yaml")
        with open(path, "w") as fh:
            yaml.safe_dump([entry], fh, sort_keys=True)
        print(f"\n  TF đề xuất: {path}")
        print("  (KHÔNG tự ghi đè bản đang dùng. Muốn áp dụng thì chép vào\n"
              "   src/rx150/rx150_toolbox/rx150_perception/config/static_transforms.yaml,\n"
              "   build lại, khởi động lại T2 — nó đổi cả toạ độ vật mà pick_place nhìn thấy.)")
    return {"rms_before_mm": rms(before), "rms_after_mm": rms(after),
            "d_rpy_deg": drpy.tolist(), "d_trans_mm": (1000 * (t1 - t0)).tolist(),
            "tX_mm": (1000 * tX1).tolist(), "entry": entry,
            "span_mm": (1000 * span).tolist(), "n": len(use),
            "cross_validation": cv, "residual_structure": struct}


# ─────────────────── hiệu chuẩn OFFSET GẮN TAG (mode tagoffset) ─────────────
# Bài toán: T_ref_cam · T_cam_tag_i = T_ref_ee_i · X, hai ẩn SE(3) (extrinsic
# camera và offset tag), đúng dạng robot-world/hand-eye AX = ZB.
#
# Vì sao không lấy thẳng X = T_ref_ee⁻¹·T_ref_tag như analyze(): công thức đó
# giả định extrinsic camera ĐÚNG, nên mọi sai số hiệu chuẩn camera chui hết vào
# X. Đo được trên dữ liệu thật: qua 6 lần chạy với 3 bản hiệu chuẩn camera khác
# nhau, X kiểu đó trượt 10 mm theo y, trong khi nghiệm đồng thời chỉ trượt 2.7 mm.
#
# Giải đồng thời cần R_ee THAY ĐỔI giữa các pose — xem chú thích TAGCAL_POSES.

def install_tag_offset(new_file, out_dir, verdict=""):
    """Cài bản vừa đo thành bản ĐANG DÙNG (có sao lưu bản cũ vào thư mục kết quả).

    Ghi vào MÃ NGUỒN chứ không phải bản build: `install/` ở workspace này là
    symlink về `src/`, nên đi theo symlink là ra đúng chỗ cần ghi. Ghi thẳng vào
    `install/.../share` mà nó là file thật thì `colcon build` lần sau xoá sạch.
    """
    import shutil
    if "CHƯA ĐỦ" in (verdict or ""):
        print("\n  ⚠ KHÔNG cài: kiểm tra chéo nói dữ liệu chưa đủ. Đo thêm pose rồi hãy cài.")
        return None
    ws = os.environ.get("RX150_WS", os.path.expanduser("~/interbotix_ws"))
    src = os.path.join(ws, "src", "rx150", "rx150_toolbox", "rx150_perception",
                       "config", "ee_tag_offset.yaml")
    share = None
    try:
        share = _share_config("ee_tag_offset.yaml")
    except Exception:                                      # noqa: BLE001
        pass
    target = os.path.realpath(share) if (share and os.path.islink(share)) else src
    os.makedirs(os.path.dirname(target), exist_ok=True)
    if os.path.isfile(target) and out_dir:
        bak = os.path.join(out_dir, "ee_tag_offset_previous.yaml")
        shutil.copyfile(target, bak)
        print(f"\n  bản cũ đã sao lưu: {bak}")
    shutil.copyfile(new_file, target)
    print(f"  ĐÃ CÀI: {target}")
    if share and not os.path.islink(share) and os.path.abspath(share) != os.path.abspath(target):
        shutil.copyfile(new_file, share)
        print(f"  (chép thêm vào bản build đang chạy: {share})")
    print("  ⇒ Khởi động lại T2 ('./rx150.sh t2') để frame rx150/ee_tag_link dùng số mới.")
    return target


def load_tag_offset(path):
    """Đọc ee_tag_offset.yaml -> ma trận 4x4 T(ee_frame -> tag)."""
    import yaml
    with open(path) as fh:
        d = yaml.safe_load(fh)
    return make_T([d["x"], d["y"], d["z"]], [d["qx"], d["qy"], d["qz"], d["qw"]])


def _groups(rows):
    """Gộp mẫu theo POSE. Mẫu trong cùng một pose gần như trùng nhau: coi mỗi
    mẫu là một quan sát độc lập sẽ thổi phồng độ tin cậy lên hàng chục lần."""
    out = []
    for lab in sorted({r["label"] for r in rows}):
        sel = [r for r in rows if r["label"] == lab]
        def m(pre):
            return np.array([np.mean([float(r[f"{pre}_{a}"]) for r in sel]) for a in "xyz"])
        def mq(pre):
            return quat_mean([[float(r[f"{pre}_q{a}"]) for a in "xyzw"] for r in sel])
        out.append({"label": lab, "n": len(sel),
                    "p_ee": m("enc"), "R_ee": quat_to_R(mq("enc")),
                    "p_raw": m("raw"), "R_raw": quat_to_R(mq("raw")),
                    "p_cam": m("cam")})
    return out


def _fit_AXZB(gs, w_rot=0.05):
    """Giải (T_ref_cam, X). w_rot = cánh tay đòn quy góc ra mét (0.05 m/rad)."""
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot

    P_raw = np.array([g["p_raw"] for g in gs])
    P_cam = np.array([g["p_cam"] for g in gs])
    R0, t0 = _kabsch(P_raw, P_cam)                 # extrinsic ĐANG dùng
    Xs = [T_inv(make_T(g["p_ee"], R_to_quat(g["R_ee"])))
          @ make_T(g["p_cam"], R_to_quat(np.array(R0) @ g["R_raw"])) for g in gs]
    RX0 = quat_to_R(quat_mean([R_to_quat(x[:3, :3]) for x in Xs]))
    tX0 = np.median(np.array([x[:3, 3] for x in Xs]), axis=0)

    def unpack(prm):
        return (R0 @ Rot.from_rotvec(prm[0:3]).as_matrix(), t0 + prm[3:6],
                RX0 @ Rot.from_rotvec(prm[6:9]).as_matrix(), tX0 + prm[9:12])

    def resid(prm):
        R_bc, t_bc, R_X, t_X = unpack(prm)
        e = []
        for g in gs:
            p_hat = R_bc @ g["p_raw"] + t_bc
            R_hat = R_bc @ g["R_raw"]
            p_prd = g["R_ee"] @ t_X + g["p_ee"]
            R_prd = g["R_ee"] @ R_X
            e.append(np.concatenate([
                p_hat - p_prd,
                w_rot * Rot.from_matrix(R_prd.T @ R_hat).as_rotvec()]))
        return np.concatenate(e)

    sol = least_squares(resid, np.zeros(12), method="lm", max_nfev=40000)
    R_bc, t_bc, R_X, t_X = unpack(sol.x)
    d = resid(sol.x).reshape(-1, 6)
    return {"R_bc": R_bc, "t_bc": t_bc, "R_X": R_X, "t_X": t_X,
            "pos_mm": float(1000 * np.sqrt((np.linalg.norm(d[:, :3], axis=1) ** 2).mean())),
            "rot_deg": float(math.degrees(np.sqrt(
                (np.linalg.norm(d[:, 3:] / w_rot, axis=1) ** 2).mean()))),
            "X0": (RX0, tX0), "jac": sol.jac}


def tag_offset(rows, args, out_dir=None):
    from scipy.spatial.transform import Rotation as Rot
    if rows and isinstance(rows[0], list):
        rows = [dict(zip(CSV_HEADER, r)) for r in rows]
    if not rows or "raw_x" not in rows[0]:
        sys.exit("CSV thiếu cột raw_* — cần đo lại bằng bản bench này.")
    use = [r for r in rows if int(float(r["moving"])) == 0] or rows
    gs = _groups(use)
    if len(gs) < 8:
        sys.exit(f"chỉ {len(gs)} pose — cần ≥8 để tách X khỏi extrinsic "
                 f"(chạy: eetag-hold --pose-set tagcal)")

    # Độ QUAN SÁT ĐƯỢC của t_X: từ hiệp phương sai của bài 9 ẩn (6 extrinsic +
    # t_X). Nếu mọi pose cùng một R_ee thì ma trận suy biến -> t_X vô nghĩa.
    F = np.zeros((9, 9))
    for g in gs:
        q = g["p_cam"] - np.mean([h["p_cam"] for h in gs], axis=0)
        J = np.hstack([-np.array([[0, -q[2], q[1]], [q[2], 0, -q[0]], [-q[1], q[0], 0]]),
                       np.eye(3), -g["R_ee"]])
        F += J.T @ J
    obs = np.sqrt(np.maximum(np.linalg.eigvalsh(np.linalg.inv(F)[6:9, 6:9]), 0))

    fit = _fit_AXZB(gs)
    t_X, R_X = fit["t_X"], fit["R_X"]
    sigma = fit["pos_mm"] * obs                 # std(t_X) thực tế, mm

    # Jackknife: bỏ từng pose ra fit lại. Không giả định gì về phân bố sai số.
    jk_t, jk_r = [], []
    for i in range(len(gs)):
        f = _fit_AXZB([g for j, g in enumerate(gs) if j != i])
        jk_t.append(f["t_X"])
        jk_r.append(math.degrees(np.linalg.norm(
            Rot.from_matrix(R_X.T @ f["R_X"]).as_rotvec())))
    jk_t = np.array(jk_t)
    jk_std = np.sqrt(len(gs) - 1) * jk_t.std(axis=0)     # hệ số jackknife

    # Kiểm tra chéo theo pose: fit nửa này, dự đoán vị trí tag ở nửa CHƯA thấy.
    labs = [g["label"] for g in gs]
    halves = ([g for g in gs if labs.index(g["label"]) % 2 == 0],
              [g for g in gs if labs.index(g["label"]) % 2 == 1])
    cv = []
    for train, test in (halves, halves[::-1]):
        f = _fit_AXZB(train)
        d = [f["R_bc"] @ g["p_raw"] + f["t_bc"] - (g["R_ee"] @ f["t_X"] + g["p_ee"])
             for g in test]
        cv.append({"pos_mm": float(1000 * np.sqrt(
            (np.linalg.norm(np.array(d), axis=1) ** 2).mean())),
            "t_X_mm": (1000 * f["t_X"]).tolist()})
    cv_gap = float(np.linalg.norm(np.array(cv[0]["t_X_mm"]) - np.array(cv[1]["t_X_mm"])))

    # X kiểu cũ (analyze): extrinsic coi như đúng. Để so.
    RX0, tX0 = fit["X0"]
    q_X = R_to_quat(R_X)
    rpy = np.degrees(Rot.from_matrix(R_X).as_euler("xyz"))

    print(f"\nOFFSET GẮN TAG  T({args.ee_frame} -> tag id {args.tag_id})")
    print(f"  giải ĐỒNG THỜI với extrinsic camera trên {len(gs)} pose, "
          f"{len(use)} mẫu đứng yên")
    print(f"\n  tịnh tiến t_X = ({', '.join(f'{v:+8.2f}' for v in 1000*t_X)}) mm")
    print(f"     ± (jackknife) ({', '.join(f'{v:8.2f}' for v in 1000*jk_std)}) mm")
    print(f"     ± (hiệp phương sai) ({', '.join(f'{v:8.2f}' for v in sigma)}) mm")
    print(f"  xoay    R_X   = roll {rpy[0]:+7.2f}°  pitch {rpy[1]:+7.2f}°  yaw {rpy[2]:+7.2f}°")
    print(f"     quat (x,y,z,w) = ({', '.join(f'{v:+.6f}' for v in q_X)})")
    print(f"     ± (jackknife) {np.sqrt(len(gs)-1)*np.std(jk_r):.2f}°")
    print(f"\n  dư sau khớp: vị trí {fit['pos_mm']:.2f} mm RMS, hướng {fit['rot_deg']:.2f}° RMS")
    print(f"  so với cách cũ (coi extrinsic là đúng): "
          f"({', '.join(f'{v:+.2f}' for v in 1000*(t_X-tX0))}) mm chênh lệch")
    print(f"\n  Kiểm tra chéo (fit nửa số pose, đo trên nửa CHƯA thấy):")
    for i, c in enumerate(cv):
        print(f"    lần {i+1}: dư {c['pos_mm']:5.2f} mm   t_X = "
              f"({', '.join(f'{v:+7.2f}' for v in c['t_X_mm'])}) mm")
    worst = max(sigma.max(), (1000 * jk_std).max())
    if cv_gap > 3.0 * worst or cv_gap > 6.0:
        verdict = (f"hai nửa lệch {cv_gap:.1f} mm — DỮ LIỆU CHƯA ĐỦ, đừng dùng số này")
    elif worst > 3.0:
        verdict = (f"hai nửa lệch {cv_gap:.1f} mm nhưng sai số còn {worst:.1f} mm — "
                   f"dùng tạm được, nên đo thêm pose")
    else:
        verdict = (f"hai nửa lệch {cv_gap:.1f} mm, sai số {worst:.1f} mm — ĐÁNG TIN")
    print(f"    {verdict}")

    res = {"t_X_mm": (1000 * t_X).tolist(), "quat_X_xyzw": q_X.tolist(),
           "rpy_X_deg": rpy.tolist(),
           "sigma_cov_mm": sigma.tolist(), "sigma_jackknife_mm": (1000 * jk_std).tolist(),
           "sigma_rot_jackknife_deg": float(np.sqrt(len(gs) - 1) * np.std(jk_r)),
           "resid_pos_mm": fit["pos_mm"], "resid_rot_deg": fit["rot_deg"],
           "t_X_naive_mm": (1000 * tX0).tolist(),
           "cross_validation": cv, "cv_gap_mm": cv_gap, "verdict": verdict,
           "n_poses": len(gs), "n_samples": len(use),
           "poses": [g["label"] for g in gs]}

    if out_dir:
        import yaml
        path = os.path.join(out_dir, "ee_tag_offset.yaml")
        entry = {
            "parent_frame": args.ee_frame, "child_frame": "ee_tag",
            "tag_id": int(args.tag_id),
            "x": float(t_X[0]), "y": float(t_X[1]), "z": float(t_X[2]),
            "qx": float(q_X[0]), "qy": float(q_X[1]),
            "qz": float(q_X[2]), "qw": float(q_X[3]),
            "measurement": {
                "calibrated_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "poses": len(gs), "samples": len(use),
                "sigma_mm": [round(float(v), 2) for v in np.maximum(sigma, 1000 * jk_std)],
                "resid_pos_mm": round(fit["pos_mm"], 2),
                "resid_rot_deg": round(fit["rot_deg"], 2),
                "verdict": verdict,
            },
        }
        with open(path, "w") as fh:
            fh.write("# ee_tag_offset.yaml — OFFSET GẮN TAG trên tay gắp, đo bằng\n"
                     "# rx150_ee_tag_bench.py tagoffset (giải đồng thời với extrinsic camera).\n"
                     "#\n"
                     "# SINH TỰ ĐỘNG — đừng sửa tay. Dán lại tag => phải đo lại.\n"
                     "# Dùng: eetag-hold/watch --tag-offset <file này> để analyze cho ra\n"
                     "# sai lệch TUYỆT ĐỐI thay vì chỉ độ không nhất quán giữa các pose.\n\n")
            yaml.safe_dump(entry, fh, sort_keys=True, default_flow_style=False,
                           allow_unicode=True)
        print(f"\n  Offset đã ghi: {path}")
        if getattr(args, "install", False):
            install_tag_offset(path, out_dir, verdict)
        else:
            print(f"  (KHÔNG tự cài — thêm --install, hoặc chép tay vào\n"
                  f"   src/rx150/rx150_toolbox/rx150_perception/config/ee_tag_offset.yaml)")
    return res


# ───────────── báo cáo bám quỹ đạo + tới điểm (mode pick / pickreport) ──────
# Ba câu hỏi tách bạch, mỗi câu một phép đo riêng:
#
#  1) TỚI ĐIỂM  — ở các điểm DỪNG, camera thấy tay gắp cách chỗ được lệnh bao
#     nhiêu mm. Với X cố định (--tag-offset) đây là sai lệch TUYỆT ĐỐI.
#  2) BÁM QUỸ ĐẠO — lúc ĐANG CHẠY, sai lệch tách làm hai phần: TRỄ (quy ra giây)
#     và phần còn lại. Không tách thì trễ 60 ms ở 40 mm/s đội lên thành 2.4 mm
#     và bị đọc nhầm là "lệch offset".
#  3) HAI TAG   — vector tay↔giá do camera đo trong CÙNG một khung hình so với
#     vector mà robot tin. Hiệu hai pose trong một ảnh KHÔNG đi qua phần tịnh
#     tiến của hiệu chuẩn hand-eye, nên nó kiểm được đúng đại lượng mà cú gắp
#     phải chịu, độc lập với t_camera.


def _pick_rows(rows, X, dedup=True):
    """CSV -> mảng gọn. cam = vị trí ee_gripper_link SUY TỪ TAG qua offset X.

    dedup: bench lấy mẫu nhanh hơn 30 fps của camera, nên MỘT khung ảnh bị ghi
    lại nhiều lần. Giữ nguyên thì trung bình bị kéo về phía khung nào sống lâu
    (đúng lúc detector trượt vài frame) và n bị thổi phồng — lúc dừng đã thấy
    293 'mẫu' cho 3 giây. Mỗi dấu thời gian ảnh = MỘT quan sát độc lập.
    """
    seen = set()
    out = []
    for r in rows:
        if dedup:
            key = (r["label"], round(float(r["stamp"]), 6))
            if key in seen:
                continue
            seen.add(key)
        T_cam_ee = make_T([float(r["cam_x"]), float(r["cam_y"]), float(r["cam_z"])],
                          [float(r["cam_qx"]), float(r["cam_qy"]),
                           float(r["cam_qz"]), float(r["cam_qw"])]) @ T_inv(X)
        out.append({
            "t": float(r["t"]), "stamp": float(r["stamp"]), "label": r["label"],
            "moving": int(float(r["moving"])),
            "enc": np.array([float(r["enc_x"]), float(r["enc_y"]), float(r["enc_z"])]),
            "cmd": np.array([float(r["fk_x"]), float(r["fk_y"]), float(r["fk_z"])]),
            "cam": T_cam_ee[:3, 3],
            # pose TAG trong khung base (đã nhân TF hiệu chuẩn) — cần để suy
            # ngược lại chính cái extrinsic đang dùng bằng Kabsch với raw.
            "tagbase": np.array([float(r["cam_x"]), float(r["cam_y"]),
                                 float(r["cam_z"])]),
            "R_enc": quat_to_R([float(r["enc_qx"]), float(r["enc_qy"]),
                                float(r["enc_qz"]), float(r["enc_qw"])]),
            "raw": np.array([float(r["raw_x"]), float(r["raw_y"]), float(r["raw_z"])]),
            "rk_ok": int(float(r.get("rk_ok", 0) or 0)),
            "rk": np.array([float(r.get("rk_x", "nan") or "nan"),
                            float(r.get("rk_y", "nan") or "nan"),
                            float(r.get("rk_z", "nan") or "nan")]),
        })
    return out


def _lag_fit(t, src, dst, span=0.30, step=0.002):
    """τ, bias để dst(t) ≈ src(t−τ) + bias. τ>0 ⇒ dst CHẬM hơn src τ giây."""
    order = np.argsort(t)
    t, src, dst = t[order], src[order], dst[order]
    best = None
    for tau in np.arange(-span, span + 1e-9, step):
        ts = t - tau
        m = (ts >= t[0]) & (ts <= t[-1])
        if int(m.sum()) < 20:
            continue
        sh = np.column_stack([np.interp(ts[m], t, src[:, i]) for i in range(3)])
        d = dst[m] - sh
        b = d.mean(axis=0)
        rms = float(1000 * np.sqrt((np.linalg.norm(d - b, axis=1) ** 2).mean()))
        raw = float(1000 * np.sqrt((np.linalg.norm(d, axis=1) ** 2).mean()))
        if best is None or rms < best["rms_mm"]:
            best = {"tau_s": float(tau), "bias_mm": (1000 * b).tolist(),
                    "rms_mm": rms, "rms_with_bias_mm": raw, "n": int(m.sum())}
    return best


def _line_dev(pts, a, b):
    """Khoảng cách từ mỗi điểm tới ĐOẠN THẲNG a→b (mm). Đây là độ cong của
    đường đi, không dính dáng gì tới nhanh/chậm hay trễ."""
    d = b - a
    L = float(np.linalg.norm(d))
    if L < 1e-9:
        return np.zeros(len(pts))
    u = d / L
    w = pts - a
    proj = np.clip(w @ u, 0.0, L)
    return 1000.0 * np.linalg.norm(w - proj[:, None] * u[None, :], axis=1)


def dual_tag(pk, R_bc, t_bc, X, rack):
    """Vector tay→giá: camera đo (một ảnh, hai tag) vs robot tin (encoder + file)."""
    sel = [o for o in pk if o["rk_ok"] == 1 and np.isfinite(o["rk"]).all()]
    if len(sel) < 10 or rack is None or not np.isfinite(rack["tag_xyz"]).all():
        return None
    v_cam = np.array([R_bc @ (o["rk"] - o["raw"]) for o in sel])
    v_rob = np.array([rack["tag_xyz"] - (o["enc"] + o["R_enc"] @ X[:3, 3]) for o in sel])
    d = v_cam - v_rob
    live = np.array([R_bc @ o["rk"] + t_bc for o in sel])
    stat = [o for o in sel if o["moving"] == 0]
    d_stat = (np.array([R_bc @ (o["rk"] - o["raw"]) - (rack["tag_xyz"] -
              (o["enc"] + o["R_enc"] @ X[:3, 3])) for o in stat])
              if len(stat) >= 10 else d)
    return {
        "n": len(sel), "n_static": len(stat),
        "mean_mm": (1000 * d_stat.mean(axis=0)).tolist(),
        "std_mm": (1000 * d_stat.std(axis=0)).tolist(),
        "rms_mm": float(1000 * np.sqrt((np.linalg.norm(d_stat, axis=1) ** 2).mean())),
        "rack_live_mean_m": live.mean(axis=0).tolist(),
        "rack_file_m": rack["tag_xyz"].tolist(),
        "rack_drift_mm": (1000 * (live.mean(axis=0) - rack["tag_xyz"])).tolist(),
        "rack_spread_mm": (1000 * live.std(axis=0)).tolist(),
    }


def pick_report(rows, args, out_dir=None, meta=None):
    if rows and isinstance(rows[0], list):
        rows = [dict(zip(CSV_HEADER, r)) for r in rows]
    if not rows:
        return {"error": "không có mẫu nào"}
    xpath = resolve_config(args.tag_offset, None)
    X = load_tag_offset(xpath) if xpath else np.eye(4)
    pk = _pick_rows(rows, X)
    rack = None
    try:
        rack = load_rack(resolve_config(args.rack_pose, "rack_pose.yaml"))
    except SystemExit:
        pass

    # extrinsic ĐANG dùng, suy ngược từ chính dữ liệu (một TF cho mọi mẫu)
    P_raw = np.array([o["raw"] for o in pk])
    P_ref = np.array([o["tagbase"] for o in pk])
    R_bc, t_bc = _kabsch(P_raw, P_ref)

    dwell = [o for o in pk if o["label"].startswith("dwell:")]
    move = [o for o in pk if o["label"].startswith("move:")]

    print(f"\n{'═'*74}\nBÁO CÁO BÁM QUỸ ĐẠO + TỚI ĐIỂM")
    print(f"{'═'*74}")
    print(f"offset gắn tag X: {xpath or 'KHÔNG CÓ (số dưới đây chỉ là độ không nhất quán)'}")
    if rack:
        print(f"giá: {rack['path']} (đo lúc {rack['at']})")
    print(f"mẫu: {len(pk)} tổng · {len(dwell)} ở điểm dừng · {len(move)} lúc đang chạy")

    res = {"n": len(pk), "n_dwell": len(dwell), "n_move": len(move),
           "tag_offset": xpath, "meta": meta or {}}

    # ── 1. TỚI ĐIỂM ────────────────────────────────────────────────────────
    labs = sorted({o["label"] for o in dwell},
                  key=lambda s: min(o["t"] for o in dwell if o["label"] == s))
    print(f"\n1) TỚI ĐIỂM — ở mỗi điểm dừng (mm, khung {args.ref_frame})")
    print(f"   {'điểm':<20} {'n':>4} {'|lệnh−cam|':>11} {'|lệnh−enc|':>11} "
           f"{'enc−cam (x, y, z)':>26}")
    pts = []
    for lab in labs:
        grp = [o for o in dwell if o["label"] == lab]
        t0lab = min(o["t"] for o in grp)
        late = [o for o in grp if o["t"] - t0lab >= args.settle_skip]
        sel = [o for o in late if o["moving"] == 0] or late or grp
        cam = np.mean([o["cam"] for o in sel], axis=0)
        enc = np.mean([o["enc"] for o in sel], axis=0)
        cmd = np.mean([o["cmd"] for o in sel], axis=0)
        e = enc - cam
        pts.append({"label": lab, "n": len(sel), "cmd_m": cmd.tolist(),
                    "enc_m": enc.tolist(), "cam_m": cam.tolist(),
                    "cmd_cam_mm": (1000 * (cmd - cam)).tolist(),
                    "cmd_enc_mm": (1000 * (cmd - enc)).tolist(),
                    "enc_cam_mm": (1000 * e).tolist()})
        print(f"   {lab[6:]:<20} {len(sel):>4} {1000*np.linalg.norm(cmd-cam):11.2f} "
              f"{1000*np.linalg.norm(cmd-enc):11.2f}   "
              f"({1000*e[0]:+7.2f},{1000*e[1]:+7.2f},{1000*e[2]:+7.2f})")
    if pts:
        ec = np.array([p["enc_cam_mm"] for p in pts])
        cc = np.array([p["cmd_cam_mm"] for p in pts])
        res["points"] = pts
        res["point_summary"] = {
            "enc_cam_mean_mm": ec.mean(axis=0).tolist(),
            "enc_cam_std_mm": ec.std(axis=0).tolist(),
            "cmd_cam_mean_mm": cc.mean(axis=0).tolist(),
            "cmd_cam_std_mm": cc.std(axis=0).tolist(),
            "cmd_cam_rms_mm": float(np.sqrt((np.linalg.norm(cc, axis=1) ** 2).mean())),
        }
        print(f"\n   PHẦN HẰNG   enc−cam = ({ec.mean(0)[0]:+6.2f},{ec.mean(0)[1]:+6.2f},"
              f"{ec.mean(0)[2]:+6.2f}) mm   ‖{np.linalg.norm(ec.mean(0)):.2f}‖")
        print(f"   PHẦN THAY ĐỔI theo điểm (std) = ({ec.std(0)[0]:6.2f},"
              f"{ec.std(0)[1]:6.2f},{ec.std(0)[2]:6.2f}) mm")
        print(f"   lệnh→cam: RMS {res['point_summary']['cmd_cam_rms_mm']:.2f} mm "
              f"— đây là sai số mà cú gắp thật sự chịu")
        ce = np.array([p["cmd_enc_mm"] for p in pts])
        res["point_summary"]["cmd_enc_mean_mm"] = ce.mean(axis=0).tolist()
        res["point_summary"]["cmd_enc_rms_mm"] = float(
            np.sqrt((np.linalg.norm(ce, axis=1) ** 2).mean()))
        print(f"   lệnh→enc: RMS {res['point_summary']['cmd_enc_rms_mm']:.2f} mm "
              f"— phần do CONTROLLER chưa tới nơi (không phải camera)")
        # Dư enc→cam còn phụ thuộc tầm với/độ cao không? Hằng thì hiệu chuẩn
        # nuốt được; phụ thuộc tư thế thì không — đó là võng cơ khí.
        Pe = np.array([p["enc_m"] for p in pts])
        rr = np.hypot(Pe[:, 0], Pe[:, 1])
        A = np.c_[rr, Pe[:, 2], np.ones(len(Pe))]
        st = {}
        for i, ax in enumerate("xyz"):
            c, *_ = np.linalg.lstsq(A, ec[:, i], rcond=None)
            st[ax] = {"d_dr_mm_per_m": float(c[0]), "d_dz_mm_per_m": float(c[1]),
                      "left_mm": float((ec[:, i] - A @ c).std()),
                      "raw_mm": float(ec[:, i].std())}
        res["point_summary"]["structure"] = st
        print("   cấu trúc phần dư enc→cam (thứ hiệu chuẩn KHÔNG sửa được):")
        for ax, v in st.items():
            print(f"     d{ax}: theo tầm với {v['d_dr_mm_per_m']:+7.1f} mm/m, theo độ cao "
                  f"{v['d_dz_mm_per_m']:+7.1f} mm/m  → còn lại {v['left_mm']:.2f} mm "
                  f"(thô {v['raw_mm']:.2f})")

    # ── 2. BÁM QUỸ ĐẠO ─────────────────────────────────────────────────────
    print(f"\n2) BÁM QUỸ ĐẠO — lúc đang chạy")
    if len(move) >= 40:
        t = np.array([o["stamp"] for o in move])
        cam = np.array([o["cam"] for o in move])
        enc = np.array([o["enc"] for o in move])
        cmd = np.array([o["cmd"] for o in move])
        f_ec = _lag_fit(t, enc, cam)
        f_ce = _lag_fit(t, cmd, enc)
        res["lag_enc_cam"], res["lag_cmd_enc"] = f_ec, f_ce
        for name, f, note in (("enc→cam", f_ec, "camera vs encoder"),
                              ("lệnh→enc", f_ce, "bám setpoint của controller")):
            if not f:
                continue
            b = f["bias_mm"]
            print(f"   {name:<9} trễ {1000*f['tau_s']:+6.0f} ms · lệch hằng "
                  f"({b[0]:+6.2f},{b[1]:+6.2f},{b[2]:+6.2f}) mm · dư {f['rms_mm']:5.2f} mm RMS"
                  f"  (chưa trừ trễ: {f['rms_with_bias_mm']:5.2f})   [{note}]")
        segs = []
        for lab in sorted({o["label"] for o in move}):
            sel = [o for o in move if o["label"] == lab]
            if len(sel) < 8:
                continue
            a, b = sel[0]["cmd"], sel[-1]["cmd"]
            dev_cam, dev_enc = _line_dev(np.array([o["cam"] for o in sel]), a, b), \
                               _line_dev(np.array([o["enc"] for o in sel]), a, b)
            segs.append({"label": lab, "n": len(sel), "len_mm": float(1000*np.linalg.norm(b-a)),
                         "dev_cam_rms_mm": float(np.sqrt((dev_cam**2).mean())),
                         "dev_cam_max_mm": float(dev_cam.max()),
                         "dev_enc_rms_mm": float(np.sqrt((dev_enc**2).mean()))})
        res["segments"] = segs
        if segs:
            dc = np.array([s["dev_cam_rms_mm"] for s in segs])
            de = np.array([s["dev_enc_rms_mm"] for s in segs])
            print(f"\n   Độ CONG so với đường thẳng nối hai đầu đoạn (không dính trễ):")
            print(f"   {'đoạn':<34} {'dài':>7} {'cam':>7} {'enc':>7}  (mm)")
            for s in segs:
                print(f"   {s['label'][5:]:<34} {s['len_mm']:7.1f} "
                      f"{s['dev_cam_rms_mm']:7.2f} {s['dev_enc_rms_mm']:7.2f}")
            print(f"   trung bình: camera {dc.mean():.2f} mm · encoder {de.mean():.2f} mm")
    else:
        print("   (không đủ mẫu lúc đang chạy)")

    # ── 3. HAI TAG ─────────────────────────────────────────────────────────
    dt = dual_tag(pk, R_bc, t_bc, X, rack)
    res["dual_tag"] = dt
    print(f"\n3) HAI TAG trong cùng khung hình (tag tay id {args.tag_id} ↔ tag giá "
          f"id {args.rack_tag_id})")
    if dt:
        m, sd = dt["mean_mm"], dt["std_mm"]
        dr = dt["rack_drift_mm"]
        print(f"   {dt['n']} mẫu có CẢ HAI tag ({dt['n_static']} lúc đứng yên)")
        print(f"   vector tay→giá, camera đo TRỪ robot tin:")
        print(f"     trung bình ({m[0]:+6.2f},{m[1]:+6.2f},{m[2]:+6.2f}) mm  "
              f"‖{np.linalg.norm(m):.2f}‖   độ tản ({sd[0]:.2f},{sd[1]:.2f},{sd[2]:.2f})")
        print(f"   tag giá đo trực tiếp lệch file rack_pose: "
              f"({dr[0]:+.2f},{dr[1]:+.2f},{dr[2]:+.2f}) mm")
        print(f"   ⇒ phần trung bình ở trên chính là độ lệch cú gắp phải chịu ở "
              f"chỗ giá,\n     và nó KHÔNG đi qua phần tịnh tiến của hiệu chuẩn hand-eye.")
    else:
        print("   (không đủ khung hình thấy cả hai tag — giá có nằm trong khung không?)")

    # ── phụ: tỉ lệ thấy tag ────────────────────────────────────────────────
    if meta and meta.get("total_frames"):
        print(f"\nKhung hình thấy tag tay: {meta['det_frames']}/{meta['total_frames']} "
              f"({100.0*meta['det_frames']/max(meta['total_frames'],1):.0f}%)"
              f" · TF phải lùi về bản mới nhất {meta.get('tf_fallback', 0)} lần")

    if out_dir:
        with open(os.path.join(out_dir, f"{args.label}_pickreport.json"), "w") as fh:
            json.dump(res, fh, indent=2)
        print(f"\nJSON: {os.path.join(out_dir, args.label + '_pickreport.json')}")
    return res


def plot_pick(rows, args, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if rows and isinstance(rows[0], list):
        rows = [dict(zip(CSV_HEADER, r)) for r in rows]
    xpath = resolve_config(args.tag_offset, None)
    X = load_tag_offset(xpath) if xpath else np.eye(4)
    pk = _pick_rows(rows, X)
    t = np.array([o["t"] for o in pk])
    fig, ax = plt.subplots(3, 2, figsize=(15, 11))
    for i, axis in enumerate("xyz"):
        a = ax[i][0]
        a.plot(t, [o["cmd"][i] * 1000 for o in pk], label="lệnh", lw=1.0)
        a.plot(t, [o["enc"][i] * 1000 for o in pk], label="encoder", lw=1.0)
        a.plot(t, [o["cam"][i] * 1000 for o in pk], ".", label="camera", ms=2.0)
        for o in pk:
            if o["label"].startswith("dwell:"):
                a.axvspan(o["t"], o["t"], color="0.85", lw=0.5)
        a.set_ylabel(f"{axis} [mm]"), a.grid(alpha=0.3)
    ax[0][0].legend(fontsize=8, loc="upper right")
    ax[0][0].set_title("Quỹ đạo mô phỏng gắp: lệnh vs encoder vs camera")
    ax[2][0].set_xlabel("t [s]")

    e_ec = np.array([1000 * (o["enc"] - o["cam"]) for o in pk])
    e_ce = np.array([1000 * (o["cmd"] - o["enc"]) for o in pk])
    ax[0][1].plot(t, np.linalg.norm(e_ce, axis=1), lw=0.9, label="|lệnh−enc| (controller)")
    ax[0][1].plot(t, np.linalg.norm(e_ec, axis=1), lw=0.9, label="|enc−cam| (camera vs robot)")
    ax[0][1].set_ylabel("mm"), ax[0][1].grid(alpha=0.3), ax[0][1].legend(fontsize=8)
    ax[0][1].set_title("Độ lớn sai lệch theo thời gian")
    for i, axis in enumerate("xyz"):
        ax[1][1].plot(t, e_ec[:, i], lw=0.8, label=f"d{axis}")
    ax[1][1].set_ylabel("enc−cam [mm]"), ax[1][1].grid(alpha=0.3), ax[1][1].legend(fontsize=8)
    ax[2][1].plot([o["enc"][0] * 1000 for o in pk], [o["enc"][1] * 1000 for o in pk],
                  lw=1.0, label="encoder")
    ax[2][1].plot([o["cam"][0] * 1000 for o in pk], [o["cam"][1] * 1000 for o in pk],
                  ".", ms=2.0, label="camera")
    ax[2][1].set_xlabel("x [mm]"), ax[2][1].set_ylabel("y [mm]")
    ax[2][1].set_title("Nhìn từ trên"), ax[2][1].grid(alpha=0.3), ax[2][1].legend(fontsize=8)
    ax[2][1].axis("equal")
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    print(f"đồ thị: {path}")


# ──────────────────────────────── main ─────────────────────────────────────

def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["hold", "watch", "sweep", "pick", "analyze",
                                    "refine", "tagoffset", "pickreport"])
    p.add_argument("csv", nargs="*",
                   help="cho mode analyze/refine/tagoffset. ĐƯA NHIỀU FILE để GỘP "
                        "nhiều lượt đo: nhãn pose được thêm tiền tố theo lượt nên "
                        "mỗi lượt thành một nhóm riêng — số nhóm tăng, kiểm tra "
                        "chéo và jackknife mới đủ chắc. MỘT lượt tagcal (20 pose) "
                        "thường KHÔNG đủ.")
    p.add_argument("--out", default=None, help="thư mục kết quả (mặc định tuning_runs/eetag_<ts>)")
    p.add_argument("--label", default="run")
    p.add_argument("--tag-topic", default="/ee_tag/tag_detections")
    p.add_argument("--tag-id", type=int, default=1,
                   help="id tag DÁN TRÊN TAY (khớp ee_tag.yaml; tag id 0 là của GIÁ)")
    p.add_argument("--ref-frame", default="rx150/base_link")
    p.add_argument("--ee-frame", default="rx150/ee_gripper_link")
    p.add_argument("--setpoint-topic", default="/rx150/hac/setpoint")
    p.add_argument("--joint-states-topic", default="/rx150/joint_states")
    p.add_argument("--poses", default=None, help="danh sách pose, phân cách dấu phẩy")
    p.add_argument("--pose-set", default="default",
                   choices=["default", "calib", "all", "grid", "tagcal"],
                   help="default = 6 pose đối chiếu với steady_state_bench; "
                        "calib = 15 pose theo góc khớp; "
                        "grid = lưới Cartesian sinh bằng IK (phủ rộng nhất, dùng cho refine); "
                        "tagcal = 20 pose QUÉT wrist_rotate, dùng cho mode tagoffset")
    p.add_argument("--dwell", type=float, default=5.0, help="giây chờ tới pose")
    p.add_argument("--settle", type=float, default=2.0, help="giây lấy mẫu ở pose")
    p.add_argument("--duration", type=float, default=60.0, help="watch: giây ghi")
    p.add_argument("--cycles", type=int, default=2, help="sweep: số chu kỳ")
    p.add_argument("--sweep-vel", type=float, default=0.15, help="sweep: rad/s")
    p.add_argument("--rate", type=float, default=30.0, help="Hz lấy mẫu (camera 30 fps)")
    p.add_argument("--max-age", type=float, default=0.4, help="giây: bỏ detection quá cũ")
    p.add_argument("--vel-thresh", type=float, default=0.02, help="rad/s: dưới ngưỡng = đứng yên")
    p.add_argument("--min-static", type=int, default=20)
    p.add_argument("--creep-vel", type=float, default=0.0,
                   help="rad/s: trườn chậm đoạn cuối vào pose thay vì nhảy bậc "
                        "(0 = bậc thang, mặc định cũ). Tách sai số do QUÁN TÍNH "
                        "khỏi sai số tĩnh thật.")
    p.add_argument("--creep-dist", type=float, default=0.08,
                   help="rad: dài đoạn trườn chậm")
    p.add_argument("--creep-sign", type=float, default=1.0,
                   help="+1/-1: tiếp cận từ phía nào. Chạy cả hai rồi so — ma "
                        "sát tĩnh làm sai lệch ĐỔI DẤU, trọng lực thì không.")
    p.add_argument("--tag-offset", default=None,
                   help="ee_tag_offset.yaml: dùng X CỐ ĐỊNH thay vì fit từ dữ liệu. "
                        "Cần cho mode watch — lúc task chạy, pose không trải đủ "
                        "rộng để fit X, mà fit sai thì sai số hằng bị giấu mất.")
    # ── tag GIÁ (id 0): đo CÙNG khung hình để so vector tay↔giá ────────────
    p.add_argument("--rack-tag-topic", default="/rack_tag/tag_detections",
                   help="detections của tag GIÁ. '' để tắt. Cần "
                        "'ros2 launch rx150_perception rack_calib.launch.py mode:=watch'")
    p.add_argument("--rack-tag-id", type=int, default=0)
    p.add_argument("--rack-max-dt", type=float, default=0.06,
                   help="giây: lệch dấu thời gian tối đa giữa hai detector thì vẫn "
                        "coi là cùng một khung hình")
    p.add_argument("--rack-pose", default=None,
                   help="rack_pose.yaml (mặc định: bản trong share của rx150_perception)")
    # ── quỹ đạo mô phỏng gắp (mode pick) ──────────────────────────────────
    p.add_argument("--slots", default=None, help="chỉ số lỗ, vd '1,0,3,2'")
    p.add_argument("--hover", type=float, default=0.08,
                   help="m: điểm treo cao hơn miệng lỗ bao nhiêu")
    p.add_argument("--clearance", type=float, default=0.015,
                   help="m: điểm THẤP NHẤT cách miệng lỗ bao nhiêu. Đây là chặn an "
                        "toàn — để 0 là đâm vào mặt giá nếu rack_pose lệch.")
    p.add_argument("--min-z", type=float, default=0.02, help="m: sàn tuyệt đối")
    p.add_argument("--cart-vel", type=float, default=0.05,
                   help="m/s đoạn đi ngang (approach_speed_mps của pick_place)")
    p.add_argument("--descend-vel", type=float, default=0.025,
                   help="m/s đoạn lên/xuống (descend_speed_mps của pick_place)")
    p.add_argument("--joint-vel", type=float, default=0.35,
                   help="rad/s cho đoạn đi trong không gian khớp (vào/ra tuyến)")
    p.add_argument("--min-seg-time", type=float, default=0.6, help="giây/đoạn tối thiểu")
    p.add_argument("--dwell-hover", type=float, default=1.5, help="giây dừng ở điểm treo")
    p.add_argument("--dwell-point", type=float, default=3.0,
                   help="giây dừng ở điểm thấp — dài hơn vì đây là số đo TỚI ĐIỂM")
    p.add_argument("--pitch-ladder",
                   default="1.5708,1.40,1.25,1.10,0.95,0.80,0.65,0.50,0.35,0.20",
                   help="rad, thử từ chúc nhất; lấy cái đầu tiên vừa IK được vừa "
                        "còn thấy tag")
    p.add_argument("--wrist-rotate", type=float, default=0.0,
                   help="rad. 0 = mặt tag ngửa lên phía camera treo cao")
    p.add_argument("--max-incidence", type=float, default=60.0,
                   help="độ: góc tới mặt tag lớn nhất còn chấp nhận")
    p.add_argument("--max-joint-step", type=float, default=0.12,
                   help="rad giữa hai setpoint liên tiếp — vượt là nghi lật nhánh IK")
    p.add_argument("--approach-time", type=float, default=6.0, help="giây, chỉ để ước tính")
    p.add_argument("--settle-skip", type=float, default=0.5,
                   help="giây đầu mỗi điểm dừng bị BỎ khi tính 'tới điểm' — bám "
                        "setpoint có trễ nên đoạn đầu vẫn đang đuổi, tính vào là "
                        "đọc nhầm quá độ thành sai số tĩnh")
    # ── lưới pose hiệu chuẩn ──────────────────────────────────────────────
    p.add_argument("--keepout-rack", action="store_true",
                   help="bỏ pose chạm hộp vật cản của GIÁ (rack_box trong rack_pose.yaml)")
    p.add_argument("--keepout-margin", type=float, default=0.06, help="m: lề vùng cấm")
    p.add_argument("--grid-radii", default=None, help="m, vd '0.16,0.24,0.31'")
    p.add_argument("--grid-heights", default=None, help="m, vd '0.06,0.15,0.24'")
    p.add_argument("--grid-waists-deg", default=None, help="độ, vd '-40,-20,0,20,40'")
    p.add_argument("--install", action="store_true",
                   help="mode tagoffset: cài luôn kết quả thành ee_tag_offset.yaml "
                        "đang dùng (sao lưu bản cũ). Từ chối nếu kiểm tra chéo nói "
                        "dữ liệu chưa đủ.")
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--force", action="store_true",
                   help="chạy hold/sweep/pick dù preflight hoặc kiểm khô báo lỗi")
    return p


def main():
    args = build_parser().parse_args()

    if args.mode in ("analyze", "refine", "tagoffset", "pickreport"):
        if not args.csv:
            sys.exit(f"mode {args.mode} cần đường dẫn CSV")
        if args.mode == "pickreport" and len(args.csv) > 1:
            sys.exit("pickreport chỉ nhận MỘT file CSV")
        rows = merge_rows(args.csv)
        args.csv = args.csv[0]
        if args.mode == "tagoffset":
            r = tag_offset(rows, args, out_dir=os.path.dirname(os.path.abspath(args.csv)))
            with open(os.path.splitext(args.csv)[0] + "_tagoffset.json", "w") as fh:
                json.dump(r, fh, indent=2)
            return
        if args.mode == "pickreport":
            d = os.path.dirname(os.path.abspath(args.csv))
            # nhãn lấy từ chính tên CSV, nếu không kết quả đọc lại sẽ mang tên
            # mặc định 'run' và đè lên nhau khi phân tích nhiều lượt trong một thư mục
            base = os.path.basename(os.path.splitext(args.csv)[0])
            args.label = base[:-5] if base.endswith("_pick") else base
            pick_report(rows, args, out_dir=d)
            if not args.no_plot:
                plot_pick(rows, args, os.path.join(d, f"{args.label}_pick.png"))
            return
        if args.mode == "refine":
            r = refine(rows, args, out_dir=os.path.dirname(os.path.abspath(args.csv)))
            with open(os.path.splitext(args.csv)[0] + "_refine.json", "w") as fh:
                json.dump(r, fh, indent=2)
            return
        s, out = analyze(rows, args)
        print_summary(s)
        if not args.no_plot:
            plot(out, os.path.splitext(args.csv)[0] + ".png")
        return

    out_dir = args.out or os.path.join(
        os.environ.get("RX150_WS", os.path.expanduser("~/interbotix_ws")),
        "tuning_runs", f"eetag_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = EeTagBench(args)
    t0 = node.get_clock().now().nanoseconds * 1e-9
    ok, problems = preflight(node, t0)
    for p in problems:
        print(f"[preflight] {p}")
    if args.mode == "pick" and node.rack_count == 0 and args.rack_tag_topic:
        print(f"[preflight] KHÔNG thấy tag GIÁ id={args.rack_tag_id} trên "
              f"{args.rack_tag_topic} — phần kiểm chéo hai tag sẽ trống "
              f"(bài đo quỹ đạo vẫn chạy bình thường).")
    if not ok and args.mode in ("hold", "sweep", "pick") and not args.force:
        print("Dừng — sửa các mục trên rồi chạy lại (hoặc --force nếu biết mình làm gì).")
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    print(f"FK lệnh: {node.fk_src}   tag: {args.tag_topic} id={args.tag_id}   "
          f"khung: {args.ref_frame}")

    try:
        if args.mode == "hold":
            run_hold(node, args, t0)
        elif args.mode == "watch":
            run_watch(node, args, t0)
        elif args.mode == "pick":
            run_pick(node, args, t0)
        else:
            run_sweep(node, args, t0)
    except KeyboardInterrupt:
        print("\n(dừng bằng Ctrl+C — vẫn ghi những gì đã đo)")

    csv_path = os.path.join(out_dir, f"{args.label}_{args.mode}.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        w.writerows(node.rows)
    print(f"\nCSV: {csv_path}  ({len(node.rows)} mẫu, "
          f"{node.det_count}/{node.frame_count} khung có tag, "
          f"{node.tf_fallback} lần TF phải lùi về bản mới nhất)")

    meta = {"mode": args.mode, "label": args.label, "fk": node.fk_src,
            "tag_topic": args.tag_topic, "tag_id": args.tag_id,
            "ref_frame": args.ref_frame, "det_frames": node.det_count,
            "total_frames": node.frame_count, "tf_fallback": node.tf_fallback,
            "rack_detections": node.rack_count}
    if node.rows and args.mode == "pick":
        pick_report(node.rows, args, out_dir=out_dir, meta=meta)
        if not args.no_plot:
            plot_pick(node.rows, args, os.path.join(out_dir, f"{args.label}_pick.png"))
    elif node.rows:
        s, out = analyze(node.rows, args, meta=meta)
        print_summary(s)
        json_path = os.path.join(out_dir, f"{args.label}_{args.mode}.json")
        with open(json_path, "w") as fh:
            json.dump(s, fh, indent=2)
        print(f"JSON: {json_path}")
        if not args.no_plot:
            plot(out, os.path.join(out_dir, f"{args.label}_{args.mode}.png"))
        if args.pose_set == "grid" and args.mode == "hold":
            # Lưới grid sinh ra CHỈ để giải lại extrinsic camera — chạy refine
            # luôn, khỏi phải nhớ gọi tay đúng file CSV vừa ghi.
            try:
                r = refine(node.rows, args, out_dir=out_dir)
                with open(os.path.splitext(csv_path)[0] + "_refine.json", "w") as fh:
                    json.dump(r, fh, indent=2)
            except SystemExit as exc:
                print(f"[refine] bỏ qua: {exc}")
        if args.pose_set == "tagcal" and args.mode == "hold":
            # Bộ pose này sinh ra CHỈ để giải offset tag — chạy luôn, khỏi phải
            # nhớ gọi tay mode tagoffset trên đúng file CSV vừa ghi.
            try:
                r = tag_offset(node.rows, args, out_dir=out_dir)
                with open(os.path.splitext(csv_path)[0] + "_tagoffset.json", "w") as fh:
                    json.dump(r, fh, indent=2)
            except SystemExit as exc:
                print(f"[tagoffset] bỏ qua: {exc}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
