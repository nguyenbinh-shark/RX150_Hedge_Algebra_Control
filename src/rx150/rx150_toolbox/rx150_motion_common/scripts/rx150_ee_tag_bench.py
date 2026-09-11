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
)


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

        self.create_subscription(JointState, args.joint_states_topic,
                                 self._on_js, qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray, args.tag_topic,
                                 self._on_tags, qos_profile_sensor_data)
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
        cmd = self.cmd if self.cmd is not None else meas
        fkx, fky, fkz, pitch = self.fk(cmd)

        row = ([now - t0, stamp, label, moving] + list(cmd) + list(meas)
               + list(T_ref_ee[:3, 3]) + list(R_to_quat(T_ref_ee[:3, :3]))
               + list(T_ref_tag[:3, 3]) + list(R_to_quat(T_ref_tag[:3, :3]))
               + [fkx, fky, fkz, pitch]
               + list(p_cam) + list(q_cam))
        self.rows.append(row)
        return row

    # ── điều khiển ───────────────────────────────────────────────────────
    def send(self, q):
        self.cmd = list(q)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q]
        self.pub.publish(msg)

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


def pose_table(args):
    if args.pose_set == "grid":
        base = grid_poses()
    elif args.pose_set == "calib":
        base = dict(CALIB_POSES)
    elif args.pose_set == "all":
        base = {**DEFAULT_POSES, **CALIB_POSES}
    else:
        base = dict(DEFAULT_POSES)
    if args.poses:
        both = {**DEFAULT_POSES, **CALIB_POSES, **base}
        return {k: both[k] for k in args.poses.split(",")}
    return base


def run_hold(node, args, t0):
    poses = pose_table(args)
    print(f"{'pose':<11} {'n':>4}  {'|cmd-enc|':>9} {'|enc-cam|':>9} {'|cmd-cam|':>9}   (mm, thô)")
    node.hold(SLEEP_POSE, 3.0)                     # điểm xuất phát chung
    for name, q in poses.items():
        node.hold(q, args.dwell)                   # đi tới + để Ruckig xong
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


# ─────────────────────────────── phân tích ─────────────────────────────────

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
    print(f"\nĐộ tản enc→cam theo trục: ({sd[0]:.1f}, {sd[1]:.1f}, {sd[2]:.1f}) mm — "
          "đây là phần PHỤ THUỘC POSE (bias hằng đã bị X hấp thụ).")


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


def _fit_extrinsic(use):
    """Giải (ω, t_ref_cam, t_X) bằng least-squares. Trả (params, R0, t0)."""
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot
    P_raw, P_ref, P_ee, R_ee = _refine_arrays(use)
    R0, t0 = _kabsch(P_raw, P_ref)
    tX0 = _solve_tX(R0, t0, P_raw, R_ee, P_ee)

    def resid(prm):
        R = Rot.from_rotvec(prm[0:3]).as_matrix()
        return ((R @ P_raw.T).T + prm[3:6]
                - np.einsum("nij,j->ni", R_ee, prm[6:9]) - P_ee).ravel()

    x0 = np.concatenate([Rot.from_matrix(R0).as_rotvec(), t0, tX0])
    return least_squares(resid, x0, method="lm", max_nfev=20000).x, R0, t0


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


def cross_validate(use):
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
        prm, R0, t0 = _fit_extrinsic(train)
        P_raw, _, P_ee, R_ee = _refine_arrays(test)
        old = np.concatenate([Rot.from_matrix(R0).as_rotvec(), t0,
                              _solve_tX(R0, t0, P_raw, R_ee, P_ee)])
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
    tX0 = _solve_tX(R0, t0, P_raw, R_ee, P_ee)

    x0 = np.concatenate([Rot.from_matrix(R0).as_rotvec(), t0, tX0])
    before = _eval_params(x0, use)
    prm, _, _ = _fit_extrinsic(use)
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

    cv = cross_validate(use)
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


# ──────────────────────────────── main ─────────────────────────────────────

def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["hold", "watch", "sweep", "analyze", "refine"])
    p.add_argument("csv", nargs="?", help="chỉ cho mode analyze/refine")
    p.add_argument("--out", default=None, help="thư mục kết quả (mặc định tuning_runs/eetag_<ts>)")
    p.add_argument("--label", default="run")
    p.add_argument("--tag-topic", default="/ee_tag/tag_detections")
    p.add_argument("--tag-id", type=int, default=0)
    p.add_argument("--ref-frame", default="rx150/base_link")
    p.add_argument("--ee-frame", default="rx150/ee_gripper_link")
    p.add_argument("--setpoint-topic", default="/rx150/hac/setpoint")
    p.add_argument("--joint-states-topic", default="/rx150/joint_states")
    p.add_argument("--poses", default=None, help="danh sách pose, phân cách dấu phẩy")
    p.add_argument("--pose-set", default="default", choices=["default", "calib", "all", "grid"],
                   help="default = 6 pose đối chiếu với steady_state_bench; "
                        "calib = 15 pose theo góc khớp; "
                        "grid = lưới Cartesian sinh bằng IK (phủ rộng nhất, dùng cho refine)")
    p.add_argument("--dwell", type=float, default=5.0, help="giây chờ tới pose")
    p.add_argument("--settle", type=float, default=2.0, help="giây lấy mẫu ở pose")
    p.add_argument("--duration", type=float, default=60.0, help="watch: giây ghi")
    p.add_argument("--cycles", type=int, default=2, help="sweep: số chu kỳ")
    p.add_argument("--sweep-vel", type=float, default=0.15, help="sweep: rad/s")
    p.add_argument("--rate", type=float, default=30.0, help="Hz lấy mẫu (camera 30 fps)")
    p.add_argument("--max-age", type=float, default=0.4, help="giây: bỏ detection quá cũ")
    p.add_argument("--vel-thresh", type=float, default=0.02, help="rad/s: dưới ngưỡng = đứng yên")
    p.add_argument("--min-static", type=int, default=20)
    p.add_argument("--no-plot", action="store_true")
    p.add_argument("--force", action="store_true",
                   help="chạy hold/sweep dù preflight báo lỗi")
    return p


def main():
    args = build_parser().parse_args()

    if args.mode in ("analyze", "refine"):
        if not args.csv:
            sys.exit(f"mode {args.mode} cần đường dẫn CSV")
        rows = load_rows(args.csv)
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
    if not ok and args.mode in ("hold", "sweep") and not args.force:
        print("Dừng — sửa các mục trên rồi chạy lại (hoặc --force nếu biết mình làm gì).")
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    print(f"FK lệnh: {node.fk_src}   tag: {args.tag_topic} id={args.tag_id}   "
          f"khung: {args.ref_frame}")

    try:
        if args.mode == "hold":
            run_hold(node, args, t0)
        elif args.mode == "watch":
            run_watch(node, args, t0)
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

    if node.rows:
        s, out = analyze(node.rows, args, meta={
            "mode": args.mode, "label": args.label, "fk": node.fk_src,
            "tag_topic": args.tag_topic, "tag_id": args.tag_id,
            "ref_frame": args.ref_frame, "det_frames": node.det_count,
            "total_frames": node.frame_count, "tf_fallback": node.tf_fallback,
        })
        print_summary(s)
        json_path = os.path.join(out_dir, f"{args.label}_{args.mode}.json")
        with open(json_path, "w") as fh:
            json.dump(s, fh, indent=2)
        print(f"JSON: {json_path}")
        if not args.no_plot:
            plot(out, os.path.join(out_dir, f"{args.label}_{args.mode}.png"))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
