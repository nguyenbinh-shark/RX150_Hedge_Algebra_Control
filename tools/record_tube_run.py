#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""record_tube_run.py — thu dữ liệu MỘT lần chạy tube_rack để vẽ đồ thị.

Ghi ra 3 file trong tuning_runs/<label>_<ts>/:
    traj.csv   quỹ đạo ee lấy mẫu đều: ref (đặt) / enc (encoder) / tag (camera)
    det.csv    mỗi pose ống YOLO nhận được, kèm nhãn màu
    meta.json  toạ độ 4 lỗ giá, rack_pose, thống kê, nguồn dữ liệu

BA nguồn quỹ đạo, phải phân biệt khi đọc đồ thị:
    ref  = /rx150/hac/reference  -> FK   Ruckig muốn tay ở đâu
    enc  = /rx150/joint_states   -> FK   encoder nói tay đang ở đâu
    tag  = TF base_link->ee_tag        camera NHÌN THẤY tay ở đâu
ref→enc là sai số BÁM (lỗi điều khiển). enc→tag là sai số HIỆU CHUẨN/cơ khí
(encoder tưởng một đằng, thực tế một nẻo) — hai thứ hoàn toàn khác nhau, và chỉ
tách được khi ghi cả ba.

tag rỗng (NaN) là BÌNH THƯỜNG nếu không chạy './rx150.sh eetag' hoặc tag bị tay
che. Cột tag_age cho biết mẫu tag cũ bao lâu; quá --tag-max-age thì ghi NaN chứ
không kéo dài giá trị cũ — kéo dài sẽ tạo ra đoạn quỹ đạo phẳng GIẢ.

Dùng:
    ./rx150.sh eetag                 # (tuỳ chọn) muốn có đường 'tag' thì cần
    python3 tools/record_tube_run.py --label run1
    # ... bấm chạy chu kỳ ở terminal task ...
    # Ctrl+C để dừng và ghi file
"""
import argparse, csv, json, math, os, sys, time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

ARM = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']


def stamp_to_sec(s):
    return s.sec + s.nanosec * 1e-9


def quat_to_mat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    """yaw quanh z của quaternion (x,y,z,w)."""
    R = quat_to_mat(*q)
    return math.atan2(R[1, 0], R[0, 0])


def load_yaml(path):
    import yaml
    with open(path, encoding='utf-8') as fh:
        return yaml.safe_load(fh)


def share(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


class Recorder(Node):
    def __init__(self, a):
        super().__init__('tube_run_recorder')
        self.a = a
        self.t0 = time.time()

        # FK — cùng nguồn mà tube_rack_node dùng, để số liệu so được với log task.
        from rx150_modules.kinematics import Rx150Kinematics
        self.kin = Rx150Kinematics()

        # offset gắn tag: T_base_gripper = T_base_eetag * T_offset^-1
        self.T_off_inv = None
        if a.tag_offset and os.path.exists(a.tag_offset):
            o = load_yaml(a.tag_offset)
            R = quat_to_mat(o['qx'], o['qy'], o['qz'], o['qw'])
            t = np.array([o['x'], o['y'], o['z']])
            self.T_off_inv = (R.T, -R.T @ t)
            self.get_logger().info(f"offset tag: {a.tag_offset}")
        else:
            self.get_logger().warn(
                'KHÔNG có ee_tag_offset.yaml — cột tag_* sẽ là vị trí TÂM TAG, '
                'không phải ee_gripper_link. Lệch một hằng số ~74 mm.')

        self.enc = None          # (t, q[5])
        self.ref = None          # (t, q[5])
        self.err = None          # (t, e[5])
        self.tag = None          # (t_ros, xyz)
        self.phase = ''
        self.detail = ''
        self.status_raw = None

        self.rows = []
        self.dets = []
        self.n_js = self.n_ref = self.n_tag = self.n_det = self.n_status = 0
        self.n_tag_msg = self.n_tf_fail = self.n_tf_fallback = 0
        self.last_tf_err = None
        self.classes = []

        self.tf_buf = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_lis = TransformListener(self.tf_buf, self, spin_thread=True)

        self.create_subscription(JointState, '/rx150/joint_states',
                                 self._on_js, qos_profile_sensor_data)
        # hac/* publish BEST_EFFORT. Subscriber RELIABLE sẽ KHÔNG nhận được gì và
        # rclpy chỉ WARN một dòng rồi im — cột ref_* ra NaN hết mà không rõ vì sao.
        self.create_subscription(JointState, a.ref_topic, self._on_ref,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, a.err_topic, self._on_err,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseArray, a.det_topic, self._on_det, 10)
        self.create_subscription(String, a.cls_topic, self._on_cls, 10)
        self.create_subscription(String, '/tube_rack/status', self._on_status, 10)

        if not a.no_tag:
            try:
                from apriltag_ros.msg import AprilTagDetectionArray
            except ImportError:
                self.get_logger().error(
                    'Thiếu apriltag_ros — chạy --no-tag, hoặc source workspace apriltag.')
                raise
            self.create_subscription(AprilTagDetectionArray, a.tag_topic,
                                     self._on_tag, qos_profile_sensor_data)

        self.create_timer(1.0 / a.rate, self._sample)

    # ── callbacks ───────────────────────────────────────────────────────
    def _jsvals(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in ARM):
            return None
        return [msg.position[idx[j]] for j in ARM]

    def _on_js(self, m):
        q = self._jsvals(m)
        if q:
            self.enc = (stamp_to_sec(m.header.stamp), q)
            self.n_js += 1

    def _on_ref(self, m):
        q = self._jsvals(m)
        if q:
            self.ref = (stamp_to_sec(m.header.stamp), q)
            self.n_ref += 1

    def _on_err(self, m):
        q = self._jsvals(m)
        if q:
            self.err = (stamp_to_sec(m.header.stamp), q)

    def _on_tag(self, m):
        """Lấy TF base_link->ee_tag TẠI dấu thời gian ẢNH, không phải 'mới nhất'.

        Tay chạy 50 mm/s mà lệch 40 ms là 2 mm — đủ để đọc nhầm TRỄ thành sai
        lệch hằng, đúng thứ phép đo này phải tách ra.
        """
        self.n_tag_msg += 1
        for d in m.detections:
            if len(d.id) != 1 or d.id[0] != self.a.tag_id:
                continue
            ts = rclpy.time.Time(seconds=int(stamp_to_sec(m.header.stamp)),
                                 nanoseconds=int((stamp_to_sec(m.header.stamp) % 1) * 1e9))
            # Tra ĐÚNG dấu thời gian ảnh trước: tay chạy 50 mm/s mà lệch 40 ms
            # là 2 mm, đủ để đọc nhầm TRỄ thành sai lệch hằng.
            #
            # Hụt thì LÙI về bản TF mới nhất thay vì bỏ mẫu — nhưng ĐẾM số lần
            # lùi (tf_fallback trong meta.json) vì mẫu lùi đã mất căn thời gian,
            # không dùng để kết luận sai lệch hằng được nữa. Cùng cách mà
            # rx150_ee_tag_bench.py làm.
            tr = None
            try:
                tr = self.tf_buf.lookup_transform(
                    self.a.base_frame, self.a.tag_frame, ts,
                    timeout=Duration(seconds=self.a.tf_timeout))
            except Exception as exc:
                self.last_tf_err = str(exc)
                try:
                    tr = self.tf_buf.lookup_transform(
                        self.a.base_frame, self.a.tag_frame, rclpy.time.Time())
                    self.n_tf_fallback += 1
                except Exception as exc2:
                    # KHÔNG nuốt im lặng: đúng cái bẫy ở §4.1 của
                    # docs/lenh_chay_tube_rack.md — tra TF hỏng mà chỉ log
                    # 'debug' thì cột tag_* ra NaN và không ai biết vì sao.
                    self.n_tf_fail += 1
                    self.last_tf_err = str(exc2)
                    self.get_logger().warn(
                        f'TF {self.a.base_frame}<-{self.a.tag_frame}: {exc2}',
                        throttle_duration_sec=5.0)
                    return
            p = tr.transform.translation
            q = tr.transform.rotation
            xyz = np.array([p.x, p.y, p.z])
            if self.T_off_inv is not None:
                R = quat_to_mat(q.x, q.y, q.z, q.w)
                Ri, ti = self.T_off_inv
                xyz = R @ ti + xyz          # gốc ee_gripper_link trong base_link
            self.tag = (stamp_to_sec(tr.header.stamp) or time.time(), xyz)
            self.n_tag += 1
            return

    def _on_cls(self, m):
        try:
            v = json.loads(m.data)
            self.classes = v if isinstance(v, list) else v.get('classes', [])
        except Exception:
            self.classes = []

    def _on_det(self, m):
        self.n_det += 1
        t = round(time.time() - self.t0, 4)
        cls = list(self.classes)
        for i, p in enumerate(m.poses):
            self.dets.append([
                t, i, len(m.poses),
                cls[i] if i < len(cls) else '',
                round(p.position.x, 6), round(p.position.y, 6), round(p.position.z, 6),
                round(math.degrees(yaw_of((p.orientation.x, p.orientation.y,
                                           p.orientation.z, p.orientation.w))), 3),
                m.header.frame_id])

    def _on_status(self, m):
        self.n_status += 1
        self.status_raw = m.data
        try:
            v = json.loads(m.data)
            self.phase = v.get('state', '')
            self.detail = v.get('detail', '')
        except Exception:
            pass

    # ── lấy mẫu đều ─────────────────────────────────────────────────────
    def _sample(self):
        t = round(time.time() - self.t0, 4)
        nan = float('nan')
        row = [t, self.phase]

        for src in (self.enc, self.ref):
            if src is None:
                row += [nan] * 9
            else:
                x, y, z, pitch = self.kin.fk(src[1])
                row += [round(x, 6), round(y, 6), round(z, 6),
                        round(math.degrees(pitch), 4)] + [round(v, 6) for v in src[1]]

        if self.tag is None:
            row += [nan, nan, nan, nan]
        else:
            age = time.time() - self.tag[0]
            if age > self.a.tag_max_age:
                row += [nan, nan, nan, round(age, 3)]
            else:
                row += [round(float(v), 6) for v in self.tag[1]] + [round(age, 3)]

        row += ([nan] * 5) if self.err is None else [round(v, 6) for v in self.err[1]]
        self.rows.append(row)


HEADER = (['t', 'phase']
          + ['enc_x', 'enc_y', 'enc_z', 'enc_pitch_deg']
          + [f'enc_{j}' for j in ARM]
          + ['ref_x', 'ref_y', 'ref_z', 'ref_pitch_deg']
          + [f'ref_{j}' for j in ARM]
          + ['tag_x', 'tag_y', 'tag_z', 'tag_age']
          + [f'err_{j}' for j in ARM])

DET_HEADER = ['t', 'i', 'n', 'cls', 'x', 'y', 'z', 'yaw_deg', 'frame_id']


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--label', default='tuberun')
    p.add_argument('--rate', type=float, default=50.0, help='Hz lấy mẫu quỹ đạo')
    p.add_argument('--duration', type=float, default=0.0, help='0 = tới khi Ctrl+C')
    p.add_argument('--out-dir', default='tuning_runs')
    p.add_argument('--ref-topic', default='/rx150/hac/reference',
                   help='dùng /rx150/fuzzy/reference nếu chạy bộ fuzzy')
    p.add_argument('--err-topic', default='/rx150/hac/error',
                   help='dùng /rx150/fuzzy/error nếu chạy bộ fuzzy')
    p.add_argument('--det-topic', default='/yolo/detected_tubes')
    p.add_argument('--cls-topic', default='/yolo/tube_classes')
    p.add_argument('--tag-topic', default='/ee_tag/tag_detections')
    p.add_argument('--tag-frame', default='ee_tag')
    p.add_argument('--tag-id', type=int, default=1)
    p.add_argument('--tag-max-age', type=float, default=0.25)
    p.add_argument('--tf-timeout', type=float, default=0.0,
                   help='chờ TF bao lâu cho mỗi lần tra (s). MẶC ĐỊNH 0 = KHÔNG '
                        'CHẶN: mỗi callback chờ đồng bộ sẽ làm nghẽn executor '
                        '(đo được: chờ 0.2s ⇒ chỉ 7/200 message tag được xử lý). '
                        'Hụt thì đã có đường lùi về TF mới nhất.')
    p.add_argument('--base-frame', default='rx150/base_link')
    p.add_argument('--no-tag', action='store_true', help='bỏ đường camera')
    p.add_argument('--tag-offset', default=None)
    p.add_argument('--rack-file', default=None)
    a = p.parse_args()

    cfg = share('rx150_perception') + '/config'
    if a.tag_offset is None:
        a.tag_offset = cfg + '/ee_tag_offset.yaml'
    if a.rack_file is None:
        a.rack_file = cfg + '/rack_pose.yaml'

    rclpy.init()
    node = Recorder(a)
    out = os.path.join(a.out_dir, f"{a.label}_{time.strftime('%Y%m%d_%H%M%S')}")
    os.makedirs(out, exist_ok=True)

    print(f"\n📈 Ghi vào {out}/   ({a.rate:g} Hz)")
    print(f"   ref={a.ref_topic}   tag={'TẮT' if a.no_tag else a.tag_topic}")
    print('   Ctrl+C để dừng và ghi file.\n')

    try:
        if a.duration > 0:
            end = time.time() + a.duration
            while rclpy.ok() and time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    with open(os.path.join(out, 'traj.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(HEADER)
        w.writerows(node.rows)
    with open(os.path.join(out, 'det.csv'), 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(DET_HEADER)
        w.writerows(node.dets)

    meta = {
        'label': a.label,
        'recorded_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'rate_hz': a.rate,
        'samples': len(node.rows),
        'counts': {'joint_states': node.n_js, 'reference': node.n_ref,
                   'tag': node.n_tag, 'tag_msgs': node.n_tag_msg,
                   'tf_fail': node.n_tf_fail,
                   'tf_fallback': node.n_tf_fallback,
                   'detections_msgs': node.n_det,
                   'detection_poses': len(node.dets), 'status': node.n_status},
        'topics': {'ref': a.ref_topic, 'det': a.det_topic,
                   'tag': None if a.no_tag else a.tag_topic},
        'tag_offset_file': a.tag_offset if node.T_off_inv is not None else None,
        'base_frame': a.base_frame,
        'last_status': node.status_raw,
        'last_tf_error': node.last_tf_err,
    }
    try:
        rk = load_yaml(a.rack_file)
        meta['rack'] = {'file': a.rack_file, 'slots': rk.get('slots'),
                        'rack_pose': rk.get('rack_pose'),
                        'rack_box': rk.get('rack_box'),
                        'rack_tilt_deg': rk.get('rack_tilt_deg'),
                        'measurement': rk.get('measurement')}
    except Exception as exc:
        meta['rack'] = {'file': a.rack_file, 'error': str(exc)}

    with open(os.path.join(out, 'meta.json'), 'w', encoding='utf-8') as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False)

    c = meta['counts']
    print(f"\n✅ {out}")
    print(f"   traj.csv  {len(node.rows)} mẫu")
    print(f"   det.csv   {len(node.dets)} pose ống ({c['detections_msgs']} message)")
    print(f"   meta.json 4 lỗ giá + thống kê")
    if c['reference'] == 0:
        print(f"   ⚠️  KHÔNG có message nào trên {a.ref_topic} — đường 'ref' sẽ rỗng.")
    if not a.no_tag and c['tag'] == 0:
        if c['tag_msgs'] == 0:
            print(f"   ⚠️  KHÔNG có message nào trên {a.tag_topic} — "
                  "cần './rx150.sh eetag'.")
        elif c['tf_fail']:
            print(f"   ⚠️  Thấy tag ({c['tag_msgs']} message) nhưng TRA TF HỎNG "
                  f"{c['tf_fail']} lần: {node.last_tf_err}")
        else:
            print(f"   ⚠️  Có {c['tag_msgs']} message nhưng không message nào "
                  f"chứa tag id {a.tag_id}.")
    if c['tf_fallback']:
        print(f"   ⓘ  {c['tf_fallback']}/{c['tag']} mẫu tag phải LÙI về TF mới nhất "
              f"(TF trễ hơn ảnh). Các mẫu đó mất căn thời gian — đừng dùng chúng "
              f"để kết luận sai lệch hằng.")
    if c['detection_poses'] == 0:
        print("   ⚠️  KHÔNG có pose ống nào — xem §4.1 docs/lenh_chay_tube_rack.md.")
    print(f"\n   Vẽ:  python3 tools/plot_tube_run.py {out}\n")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
