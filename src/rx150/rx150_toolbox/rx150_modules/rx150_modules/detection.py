#!/usr/bin/env python3
"""
detection — nguồn vật thể từ Layer 1 (YOLO), có kiểm tra chất lượng dữ liệu.

Bản cũ giữ đúng 1 PoseArray "mới nhất" rồi gắp theo nó. Vấn đề trên máy thật:
  * KHÔNG kiểm tra tuổi dữ liệu → camera/detector chết là tay vẫn lao xuống
    theo pose của 10 giây trước.
  * KHÔNG kiểm tra header.frame_id → detector đổi target_frame (hoặc pose ra ở
    frame camera) là toàn bộ toạ độ sai mà không ai biết.
  * classes ở topic RIÊNG, khớp nhau bằng chỉ số. Bản cũ chỉ so độ dài; nếu 2
    topic lệch nhịp thì ống xanh bị dán nhãn hồng → cắm sai slot.
  * median lấy theo CHỈ SỐ mảng, giả định thứ tự detection không đổi giữa các
    frame. Ở đây ghép theo KHOẢNG CÁCH XY (nearest-neighbour) nên đúng cả khi
    detector đổi thứ tự, và bỏ vật chỉ xuất hiện thoáng qua (chống nhấp nháy).

Lưu ý về yaw: yolo_tube_detector_node lấy góc từ cv2.minAreaRect (hệ ẢNH) rồi
publish thẳng thành quaternion quanh z. Nếu camera không thẳng trục với base_link
thì yaw đó lệch một hằng số — dùng yaw_offset_deg/invert_yaw để bù (đo bằng cách
đặt 1 ống dọc trục +x của robot rồi xem log yaw).
"""
import json
import math
import threading
import time


def wrap_axis(angle):
    """Trục ống đối xứng mod pi ⇒ đưa về (−pi/2, pi/2]."""
    a = math.atan2(math.sin(angle), math.cos(angle))
    if a > math.pi / 2:
        a -= math.pi
    elif a <= -math.pi / 2:
        a += math.pi
    return a


def _median(values):
    ordered = sorted(values)
    n = len(ordered)
    return ordered[n // 2] if n % 2 else 0.5 * (ordered[n // 2 - 1] + ordered[n // 2])


class DetectionSource:
    def __init__(self, node, *, poses_topic, classes_topic=None,
                 base_frame='rx150/base_link', max_age_s=1.5,
                 yaw_offset_deg=0.0, invert_yaw=False, callback_group=None,
                 class_sync_s=1.0):
        from geometry_msgs.msg import PoseArray
        from std_msgs.msg import String
        self._node = node
        self._log = node.get_logger()
        self.base_frame = base_frame
        self.max_age = float(max_age_s)
        self.yaw_offset = math.radians(float(yaw_offset_deg))
        self.invert_yaw = bool(invert_yaw)
        self.class_sync = float(class_sync_s)

        self._lock = threading.Lock()
        self._poses = []
        self._poses_t = 0.0
        self._frame = ''
        self._classes = []
        self._classes_t = 0.0
        self._frame_warned = False

        node.create_subscription(PoseArray, poses_topic, self._poses_cb, 10,
                                 callback_group=callback_group)
        if classes_topic:
            node.create_subscription(String, classes_topic, self._classes_cb, 10,
                                     callback_group=callback_group)
        self.poses_topic = poses_topic
        self.classes_topic = classes_topic

    # ── callbacks ───────────────────────────────────────────────────────
    def _poses_cb(self, msg):
        with self._lock:
            self._poses = list(msg.poses)
            self._frame = msg.header.frame_id
            self._poses_t = time.monotonic()

    def _classes_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            self._log.warn(f'{self.classes_topic}: JSON không hợp lệ.',
                           throttle_duration_sec=5.0)
            return
        with self._lock:
            self._classes = [str(c).lower() for c in data]
            self._classes_t = time.monotonic()

    # ── truy vấn ────────────────────────────────────────────────────────
    def age(self):
        with self._lock:
            return math.inf if self._poses_t == 0.0 else time.monotonic() - self._poses_t

    def snapshot(self):
        """Frame hiện tại đã kiểm tra frame_id / tuổi / đồng bộ nhãn.

        Trả list[dict(x, y, z, yaw, cls)] hoặc None (không dùng được).
        """
        now = time.monotonic()
        with self._lock:
            poses, frame = list(self._poses), self._frame
            poses_t, classes, classes_t = self._poses_t, list(self._classes), self._classes_t
        if poses_t == 0.0:
            return None
        age = now - poses_t
        if age > self.max_age:
            self._log.warn(f'{self.poses_topic}: dữ liệu cũ {age:.1f}s '
                           f'(> {self.max_age:.1f}s) — bỏ.', throttle_duration_sec=5.0)
            return None
        if frame and frame != self.base_frame:
            if not self._frame_warned:
                self._log.error(
                    f'{self.poses_topic} ở frame "{frame}" nhưng node làm việc trong '
                    f'"{self.base_frame}" — TỪ CHỐI dữ liệu (sửa target_frame của '
                    f'detector, đừng gắp theo toạ độ sai frame).')
                self._frame_warned = True
            return None
        if not poses:
            return []
        labels = None
        if self.classes_topic:
            if len(classes) == len(poses) and abs(classes_t - poses_t) <= self.class_sync:
                labels = classes
            else:
                self._log.warn(
                    f'Nhãn màu không đồng bộ (poses={len(poses)}, classes={len(classes)}, '
                    f'lệch {abs(classes_t - poses_t):.2f}s) — coi mọi ống là "unknown".',
                    throttle_duration_sec=5.0)
        out = []
        for idx, pose in enumerate(poses):
            yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
            yaw = yaw + self.yaw_offset
            if self.invert_yaw:
                yaw = -yaw
            out.append({
                'index': idx,
                'cls': labels[idx] if labels else 'unknown',
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'z': float(pose.position.z),
                'yaw': wrap_axis(yaw),
            })
        return out

    def median_snapshot(self, frames=5, collect_s=1.0, assoc_radius=0.03,
                        min_hits_ratio=0.6):
        """Gộp nhiều frame → toạ độ median cho từng vật (ghép theo khoảng cách XY).

        Vật không xuất hiện đủ min_hits_ratio số frame bị loại (detection nhấp nháy).
        """
        frames = max(1, int(frames))
        dt = float(collect_s) / frames
        collected = []
        for _ in range(frames):
            snap = self.snapshot()
            if snap:
                collected.append(snap)
            time.sleep(dt)
        if not collected:
            return []
        ref = collected[-1]
        min_hits = max(1, int(math.ceil(min_hits_ratio * len(collected))))
        result = []
        for obj in ref:
            xs, ys, zs, dyaws, labels = [], [], [], [], []
            for snap in collected:
                best, best_d = None, assoc_radius
                for cand in snap:
                    d = math.hypot(cand['x'] - obj['x'], cand['y'] - obj['y'])
                    if d < best_d:
                        best, best_d = cand, d
                if best is None:
                    continue
                xs.append(best['x'])
                ys.append(best['y'])
                zs.append(best['z'])
                dyaws.append(wrap_axis(best['yaw'] - obj['yaw']))
                labels.append(best['cls'])
            if len(xs) < min_hits:
                self._log.info(
                    f'Bỏ vật {obj["cls"]} tại ({obj["x"]:.3f},{obj["y"]:.3f}) — chỉ thấy '
                    f'{len(xs)}/{len(collected)} frame (detection nhấp nháy).')
                continue
            # nhãn: lấy nhãn xuất hiện nhiều nhất, không phải nhãn của 1 frame
            label = max(set(labels), key=labels.count)
            result.append({
                'cls': label,
                'x': _median(xs), 'y': _median(ys), 'z': _median(zs),
                'yaw': wrap_axis(obj['yaw'] + _median(dyaws)),
                'frames': len(xs),
                'spread_xy': round(max(
                    math.hypot(x - _median(xs), y - _median(ys))
                    for x, y in zip(xs, ys)), 4),
            })
        return result

    def wait_for_detections(self, timeout_s, **median_kwargs):
        """Chờ tới khi có ít nhất 1 vật (đã median), hoặc [] khi hết thời gian."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < float(timeout_s):
            objs = self.median_snapshot(**median_kwargs)
            if objs:
                return objs
            time.sleep(0.2)
        return []
