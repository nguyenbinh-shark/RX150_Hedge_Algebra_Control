#!/usr/bin/env python3
# hand_gesture_node.py — chọn vật bằng cử chỉ tay (MediaPipe Hands).
#
# Port từ key_point/task3_4tubes.py: pointing ray (landmark 5→7), is_pointing_to
# (khoảng cách điểm→tia < 8 cm), is_ok_sign (landmark 4↔8).
#
# Topic vào:
#   /yolo/detected_tubes (PoseArray base frame — từ yolo_detector_node)
#   /camera/camera/color/image_raw + /camera/camera/aligned_depth_to_color/image_raw
#   + CameraInfo
# Topic ra:
#   ~/selected_target (std_msgs/Int32 — index trong PoseArray được chỉ tay gần nhất)
#   ~/event           (std_msgs/String — "ok_sign" khi bắt được OK-sign)

import math
import os
import time

import numpy as np
import cv2

# MediaPipe Hands chạy inference trên CPU (XNNPACK) — build pip không có GPU delegate,
# và ép EGL context sang A4000 còn CHẬM hơn (đo: 12.4 ms vs 9.8 ms) vì inference vẫn ở
# CPU còn ảnh phải đi qua PCIe. Cách giảm tải duy nhất ở đây là throttle + ghim thread.
cv2.setNumThreads(2)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge
import message_filters
import tf2_ros
import tf2_geometry_msgs

try:
    import mediapipe as mp
except ImportError as exc:  # pragma: no cover
    try:
        from ament_index_python.packages import get_package_share_directory
        req_path = os.path.join(get_package_share_directory('rx150_perception'), 'requirements.txt')
        raise ImportError(
            f'mediapipe chưa cài. Chạy: pip install --user -r {req_path}'
        ) from exc
    except ImportError:
        raise ImportError(
            'mediapipe chưa cài. Chạy: pip install --user -r src/rx150_perception/requirements.txt'
        ) from exc


def _point_ray_distance(p, a, b):
    """Khoảng cách từ điểm p đến tia đi qua a theo hướng (b-a). Port is_pointing_to."""
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-9:
        return float(np.linalg.norm(p - a))
    t = float(np.dot(p - a, ab) / denom)
    closest = a + t * ab
    return float(np.linalg.norm(p - closest))


class HandGestureNode(Node):
    def __init__(self):
        super().__init__('hand_gesture')

        self.declare_parameter('base_frame', 'rx150/base_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('detection_topic', '/yolo/detected_tubes')
        # MediaPipe Hands ~9.8 ms/frame trên CPU. Ở 30 Hz là ~29% một core liên tục;
        # 5 Hz là quá đủ để bắt cử chỉ và chỉ còn ~5%.
        self.declare_parameter('gesture_rate_hz', 5.0)
        self.declare_parameter('pointing_threshold_m', 0.08)
        self.declare_parameter('ok_sign_px', 40.0)
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/aligned_depth_to_color/camera_info')

        self.base_frame = self.get_parameter('base_frame').value
        self.optical_frame = self.get_parameter('camera_optical_frame').value
        self.det_topic = self.get_parameter('detection_topic').value
        self.thresh = float(self.get_parameter('pointing_threshold_m').value)
        self.ok_px = float(self.get_parameter('ok_sign_px').value)

        rate_hz = float(self.get_parameter('gesture_rate_hz').value)
        self._min_period = (1.0 / rate_hz) if rate_hz > 0.0 else 0.0
        self._last_proc = 0.0

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.intrinsics = None
        self.latest_poses = []  # list[np.array(3)] trong base frame

        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=1, min_detection_confidence=0.6)

        self.target_pub = self.create_publisher(Int32, '~/selected_target', 10)
        self.event_pub = self.create_publisher(String, '~/event', 10)

        self.create_subscription(PoseArray, self.det_topic, self._poses_cb, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self._info_cb, 10)
        # Topic ảnh của RealSense là sensor data: BEST_EFFORT + depth nông. Để mặc định
        # (RELIABLE, depth 10) thì vừa lệch QoS với driver vừa dựng backlog khi CPU bận.
        img_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        color_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('image_topic').value, qos_profile=img_qos)
        depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('depth_topic').value, qos_profile=img_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], 10, 0.1)
        self.sync.registerCallback(self._sync_cb)
        self.get_logger().info('hand_gesture sẵn sàng.')

    def _poses_cb(self, msg):
        self.latest_poses = [np.array([p.position.x, p.position.y, p.position.z]) for p in msg.poses]

    def _info_cb(self, msg):
        if self.intrinsics is None:
            self.intrinsics = msg
            self.get_logger().info('Đã nhận CameraInfo.')

    def _deproject(self, u, v, z_m):
        k = self.intrinsics.k
        return (u - k[2]) * z_m / k[0], (v - k[5]) * z_m / k[4], z_m

    def _to_base(self, x, y, z, stamp):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.optical_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'No TF: {e}', throttle_duration_sec=3.0)
            return None
        from geometry_msgs.msg import PointStamped
        ps = PointStamped()
        ps.header.frame_id = self.optical_frame
        ps.header.stamp = stamp
        ps.point.x, ps.point.y, ps.point.z = float(x), float(y), float(z)
        # Gọi do_transform_point MỘT lần rồi đọc 3 thành phần — bản cũ gọi 3 lần cho
        # cùng một điểm (3x công việc, không đổi kết quả).
        out = tf2_geometry_msgs.do_transform_point(ps, tf).point
        return np.array([out.x, out.y, out.z])

    def _sync_cb(self, color_msg, depth_msg):
        if self.intrinsics is None:
            return
        # Throttle trước MỌI xử lý nặng (convert ảnh, cvtColor, MediaPipe).
        now = time.monotonic()
        if self._min_period > 0.0 and (now - self._last_proc) < self._min_period:
            return
        self._last_proc = now
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            depth_mm = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
        except Exception as e:  # noqa: BLE001
            return
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        if not res.multi_hand_landmarks:
            return
        lm = res.multi_hand_landmarks[0].landmark
        h, w = color.shape[:2]

        # OK-sign: thumb tip(4) ↔ index tip(8)
        if math.hypot((lm[4].x - lm[8].x) * w, (lm[4].y - lm[8].y) * h) < self.ok_px:
            self.event_pub.publish(String(data='ok_sign'))

        # Pointing ray: lm5 (base) → lm7 (tip)
        def pt3d(idx):
            ux, uy = int(lm[idx].x * w), int(lm[idx].y * h)
            patch = depth_mm[max(0, uy - 2):uy + 3, max(0, ux - 2):ux + 3]
            valid = patch[patch > 0]
            if valid.size == 0:
                return None
            z = float(np.median(valid)) / 1000.0
            x, y, _ = self._deproject(ux, uy, z)
            return self._to_base(x, y, z, color_msg.header.stamp)

        a = pt3d(5)
        b = pt3d(7)
        if a is None or b is None or not self.latest_poses:
            return

        # chọn object gần tia chỉ tay nhất
        best_idx, best_d = -1, self.thresh
        for i, p in enumerate(self.latest_poses):
            d = _point_ray_distance(p, a, b)
            if d < best_d:
                best_d, best_idx = d, i
        if best_idx >= 0:
            self.target_pub.publish(Int32(data=best_idx))
            self.get_logger().info(f'Chỉ tay → object #{best_idx} (d={best_d:.3f} m)',
                                   throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = HandGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
