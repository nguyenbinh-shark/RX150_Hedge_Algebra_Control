#!/usr/bin/env python3
"""
yolo_tube_detector_node.py — Nhận diện Ống Nghiệm 3D + Góc Nghiêng (Keypoint & BBox).

Tính năng:
1. Hỗ trợ cả 2 loại Model YOLOv8:
   - Model Keypoint (YOLOv8-Pose): Tự động trích xuất Nắp (Keypoint 0) & Đáy (Keypoint 1).
   - Model Detection chuẩn (YOLOv8): Tự động tính góc nghiêng qua cv2.minAreaRect.
2. Đo độ sâu Z tại từng Keypoint và tâm vật thể từ RealSense Aligned Depth.
3. Tính toán vectơ trục 3D của ống nghiệm và quy đổi sang góc xoay Yaw (Quaternion)
   chính xác trong hệ tọa độ cánh tay robot (rx150/base_link).
4. Xuất các topic:
   - /yolo/detected_tubes: PoseArray chứa đầy đủ Position (X,Y,Z) VÀ Orientation (qx,qy,qz,qw).
   - /yolo/tube_classes: Danh sách JSON tên nhãn màu.
   - /yolo/image_debug: Ảnh vẽ Bounding Box, Keypoints, đường nối trục và góc độ.
   - /yolo/markers: Marker 3D (mũi tên hướng + text nhãn) xem trực quan trên RViz.
"""

import json
import os
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import message_filters

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from ament_index_python.packages import get_package_share_directory


class YoloTubeDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_tube_detector')

        # 1. Khai báo tham số
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('target_frame', 'rx150/base_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')

        # Tham số vùng làm việc giới hạn 3D (Workspace ROI Box)
        self.declare_parameter('enable_roi_box', True)
        self.declare_parameter('roi_x_min', 0.10)
        self.declare_parameter('roi_x_max', 0.35)
        self.declare_parameter('roi_y_min', -0.25)
        self.declare_parameter('roi_y_max', 0.25)
        self.declare_parameter('roi_z_min', -0.02)
        self.declare_parameter('roi_z_max', 0.25)

        # Tự động nạp giá trị từ config/roi_box_params.yaml nếu tồn tại
        try:
            import yaml
            pkg_share = get_package_share_directory('rx150_perception')
            yaml_file = os.path.join(pkg_share, 'config', 'roi_box_params.yaml')
            if os.path.isfile(yaml_file):
                with open(yaml_file, 'r') as f:
                    cfg = yaml.safe_load(f)
                    p_dict = cfg.get('/**', {}).get('ros__parameters', {}) or cfg.get('ros__parameters', {})
                    if 'roi_x_min' in p_dict:
                        self.set_parameters([
                            rclpy.parameter.Parameter('roi_x_min', rclpy.Parameter.Type.DOUBLE, float(p_dict['roi_x_min'])),
                            rclpy.parameter.Parameter('roi_x_max', rclpy.Parameter.Type.DOUBLE, float(p_dict['roi_x_max'])),
                            rclpy.parameter.Parameter('roi_y_min', rclpy.Parameter.Type.DOUBLE, float(p_dict['roi_y_min'])),
                            rclpy.parameter.Parameter('roi_y_max', rclpy.Parameter.Type.DOUBLE, float(p_dict['roi_y_max'])),
                            rclpy.parameter.Parameter('roi_z_min', rclpy.Parameter.Type.DOUBLE, float(p_dict['roi_z_min'])),
                            rclpy.parameter.Parameter('roi_z_max', rclpy.Parameter.Type.DOUBLE, float(p_dict['roi_z_max'])),
                        ])
                        self.get_logger().info(f"Đã nạp ROI Box từ YAML: X[{p_dict['roi_x_min']}..{p_dict['roi_x_max']}], Y[{p_dict['roi_y_min']}..{p_dict['roi_y_max']}], Z[{p_dict['roi_z_min']}..{p_dict['roi_z_max']}]")
        except Exception as e:
            self.get_logger().warn(f'Không nạp được roi_box_params.yaml: {e}')

        model_p = self.get_parameter('model_path').value
        if not model_p:
            pkg_share = get_package_share_directory('rx150_perception')
            # Ưu tiên tìm model trong thư mục key_point hoặc models
            possible_paths = [
                os.path.join(pkg_share, 'models', 'best_keypoint.pt'),
                os.path.join(pkg_share, 'models', 'key_point', 'best.pt'),
                os.path.join(pkg_share, 'models', 'best_color.pt'),
            ]
            for p in possible_paths:
                if os.path.isfile(p):
                    model_p = p
                    break
            if not model_p:
                model_p = os.path.join(pkg_share, 'models', 'best_color.pt')

        self.conf_thresh = float(self.get_parameter('conf_threshold').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.optical_frame = self.get_parameter('camera_optical_frame').value

        self.get_logger().info(f'Đang nạp model YOLOv8 từ: {model_p}')
        try:
            from ultralytics import YOLO
            self.model = YOLO(model_p)
            self.get_logger().info(f'Nạp model thành công! Task: {self.model.task}, Classes: {self.model.names}')
        except Exception as e:
            self.get_logger().error(f'Lỗi nạp YOLOv8: {e}. Vui lòng cài đặt: pip install ultralytics')
            self.model = None

        self.bridge = CvBridge()

        # 2. Quản lý TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 3. Publishers
        self.pub_poses = self.create_publisher(PoseArray, '/yolo/detected_tubes', 10)
        self.pub_classes = self.create_publisher(String, '/yolo/tube_classes', 10)
        self.pub_debug_img = self.create_publisher(Image, '/yolo/image_debug', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/yolo/markers', 10)

        # 4. Subscribers đồng bộ
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        sub_rgb = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw', qos_profile=qos)
        sub_depth = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=qos)
        sub_info = message_filters.Subscriber(
            self, CameraInfo, '/camera/camera/color/camera_info', qos_profile=qos)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_rgb, sub_depth, sub_info], queue_size=10, slop=0.08)
        self.sync.registerCallback(self.image_callback)

        self.get_logger().info('YOLO Tube Keypoint Detector đã sẵn sàng!')

    def _get_depth_at(self, depth_image, u, v, window=5):
        """Lấy độ sâu Z (mét) bằng bộ lọc trung vị (Median) quanh pixel (u,v)."""
        h, w = depth_image.shape[:2]
        u_int, v_int = int(round(u)), int(round(v))
        
        half = window // 2
        v_min, v_max = max(0, v_int - half), min(h, v_int + half + 1)
        u_min, u_max = max(0, u_int - half), min(w, u_int + half + 1)

        patch = depth_image[v_min:v_max, u_min:u_max]
        val = patch[patch > 0]
        if len(val) == 0:
            return 0.0
        return float(np.median(val)) / 1000.0

    def _deproject_pixel_to_3d(self, u, v, z, fx, fy, cx, cy):
        """Chiếu pixel 2D (u,v) + độ sâu Z sang tọa độ 3D Camera."""
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return np.array([x, y, z])

    def _transform_point_to_base(self, p_cam, R_tf, T_tf):
        """Chuyển đổi điểm 3D từ Camera sang Robot Base."""
        return R_tf.dot(p_cam) + T_tf

    def image_callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        if self.model is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Lỗi convert ảnh: {e}')
            return

        # Tra cứu ma trận TF Camera -> Robot Base
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                rgb_msg.header.frame_id or self.optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            tx = tf_stamped.transform.translation.x
            ty = tf_stamped.transform.translation.y
            tz = tf_stamped.transform.translation.z
            qx = tf_stamped.transform.rotation.x
            qy = tf_stamped.transform.rotation.y
            qz = tf_stamped.transform.rotation.z
            qw = tf_stamped.transform.rotation.w

            # Ma trận xoay Camera -> Robot Base
            R_tf = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ])
            T_tf = np.array([tx, ty, tz])
        except Exception as ex:
            self.get_logger().debug(f'TF lookup fail: {ex}')
            return

        # Camera Intrinsics
        fx, cx = info_msg.k[0], info_msg.k[2]
        fy, cy = info_msg.k[4], info_msg.k[5]

        # Đọc tham số ROI Box
        enable_roi = bool(self.get_parameter('enable_roi_box').value)
        roi_xmin = float(self.get_parameter('roi_x_min').value)
        roi_xmax = float(self.get_parameter('roi_x_max').value)
        roi_ymin = float(self.get_parameter('roi_y_min').value)
        roi_ymax = float(self.get_parameter('roi_y_max').value)
        roi_zmin = float(self.get_parameter('roi_z_min').value)
        roi_zmax = float(self.get_parameter('roi_z_max').value)

        # Chạy YOLO Inference
        results = self.model(cv_image, conf=self.conf_thresh, verbose=False)

        pose_array = PoseArray()
        pose_array.header.frame_id = self.target_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()

        tube_classes = []
        marker_array = MarkerArray()
        debug_img = cv_image.copy()

        marker_id = 0

        for r in results:
            # Kiểm tra xem có Keypoints (Pose model) hay Bounding Boxes (Detect model)
            has_keypoints = (hasattr(r, 'keypoints') and r.keypoints is not None and len(r.keypoints) > 0)
            boxes = r.boxes if (hasattr(r, 'boxes') and r.boxes is not None) else []

            num_detections = len(r.keypoints) if has_keypoints else len(boxes)

            for i in range(num_detections):
                cls_name = 'tube'
                conf = 0.9

                if len(boxes) > i:
                    box = boxes[i]
                    cls_id = int(box.cls[0].cpu().numpy())
                    cls_name = self.model.names.get(cls_id, f'tube_{cls_id}')
                    conf = float(box.conf[0].cpu().numpy())
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (200, 200, 200), 1)

                p_center_base = None
                yaw_robot = 0.0

                # -------------------------------------------------------------
                # TRƯỜNG HỢP A: MODEL KEYPOINTS (YOLOv8-Pose: Nắp & Đáy)
                # -------------------------------------------------------------
                if has_keypoints:
                    kp_data = r.keypoints[i].xy[0].cpu().numpy() # shape (K, 2)
                    if len(kp_data) >= 2:
                        u_top, v_top = kp_data[0] # Keypoint 0: Nắp
                        u_bot, v_bot = kp_data[1] # Keypoint 1: Đáy

                        z_top = self._get_depth_at(depth_image, u_top, v_top)
                        z_bot = self._get_depth_at(depth_image, u_bot, v_bot)
                        
                        # Nếu 1 trong 2 điểm mất depth, dùng depth trung bình
                        z_avg = max(z_top, z_bot)
                        if z_top <= 0.05: z_top = z_avg
                        if z_bot <= 0.05: z_bot = z_avg

                        if z_top > 0.05 and z_bot > 0.05:
                            # De-project 2 keypoints sang 3D
                            p_top_cam = self._deproject_pixel_to_3d(u_top, v_top, z_top, fx, fy, cx, cy)
                            p_bot_cam = self._deproject_pixel_to_3d(u_bot, v_bot, z_bot, fx, fy, cx, cy)

                            # Chuyển sang hệ Robot Base
                            p_top_base = self._transform_point_to_base(p_top_cam, R_tf, T_tf)
                            p_bot_base = self._transform_point_to_base(p_bot_cam, R_tf, T_tf)

                            # Tâm kẹp gắp (Midpoint)
                            p_center_base = (p_top_base + p_bot_base) / 2.0

                            # Vectơ hướng trục ống nghiệm (từ Nắp -> Đáy)
                            vec_tube = p_bot_base - p_top_base
                            yaw_robot = math.atan2(vec_tube[1], vec_tube[0])

                            # Vẽ Debug lên ảnh
                            pt1 = (int(u_top), int(v_top))
                            pt2 = (int(u_bot), int(v_bot))
                            cv2.circle(debug_img, pt1, 5, (0, 0, 255), -1) # Nắp: Đỏ
                            cv2.circle(debug_img, pt2, 5, (0, 255, 0), -1) # Đáy: Xanh lá
                            cv2.line(debug_img, pt1, pt2, (255, 255, 0), 2)

                # -------------------------------------------------------------
                # TRƯỜNG HỢP B: MODEL BOUNDING BOX (Tính góc qua minAreaRect)
                # -------------------------------------------------------------
                if p_center_base is None and len(boxes) > i:
                    u_center = (x1 + x2) / 2.0
                    v_center = (y1 + y2) / 2.0
                    z_center = self._get_depth_at(depth_image, u_center, v_center)

                    if 0.1 <= z_center <= 1.5:
                        p_cam = self._deproject_pixel_to_3d(u_center, v_center, z_center, fx, fy, cx, cy)
                        p_center_base = self._transform_point_to_base(p_cam, R_tf, T_tf)

                        # Cắt vùng ảnh tìm góc nghiêng minAreaRect
                        crop = cv_image[max(0, y1):min(cv_image.shape[0], y2), max(0, x1):min(cv_image.shape[1], x2)]
                        if crop.size > 0:
                            gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                            cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            if cnts:
                                rect = cv2.minAreaRect(max(cnts, key=cv2.contourArea))
                                angle_2d = math.radians(rect[2])
                                yaw_robot = angle_2d

                # Nếu tính toán 3D thành công
                if p_center_base is not None:
                    x_rob, y_rob, z_rob = float(p_center_base[0]), float(p_center_base[1]), float(p_center_base[2])

                    # Kiểm tra xem vật thể có nằm trong Vùng Làm Việc (ROI Box) không
                    if enable_roi:
                        if not (roi_xmin <= x_rob <= roi_xmax and roi_ymin <= y_rob <= roi_ymax and roi_zmin <= z_rob <= roi_zmax):
                            continue  # Nằm ngoài vùng làm việc an toàn -> bỏ qua

                    # Chuyển góc Yaw thành Quaternion (xoay quanh trục Z)
                    qz_out = math.sin(yaw_robot / 2.0)
                    qw_out = math.cos(yaw_robot / 2.0)

                    pose = Pose()
                    pose.position.x = x_rob
                    pose.position.y = y_rob
                    pose.position.z = z_rob
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = float(qz_out)
                    pose.orientation.w = float(qw_out)

                    pose_array.poses.append(pose)
                    tube_classes.append(cls_name)

                    # Vẽ chữ nhãn + góc Yaw lên ảnh Debug
                    deg = math.degrees(yaw_robot)
                    label_str = f"{cls_name} ({conf:.2f}) [{x_rob:.2f},{y_rob:.2f}] yaw:{deg:.0f}deg"
                    cv2.putText(debug_img, label_str, (x1, max(y1 - 6, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

                    # Marker 3D dạng Mũi Tên (Arrow) biểu diễn hướng xoay trong RViz
                    marker_arrow = Marker()
                    marker_arrow.header.frame_id = self.target_frame
                    marker_arrow.header.stamp = pose_array.header.stamp
                    marker_arrow.ns = 'tube_orientation'
                    marker_arrow.id = marker_id
                    marker_arrow.type = Marker.ARROW
                    marker_arrow.action = Marker.ADD
                    marker_arrow.pose = pose
                    marker_arrow.scale.x = 0.08  # Chiều dài mũi tên (8cm)
                    marker_arrow.scale.y = 0.012 # Độ dày
                    marker_arrow.scale.z = 0.012
                    marker_arrow.color.r = 0.0
                    marker_arrow.color.g = 1.0
                    marker_arrow.color.b = 1.0
                    marker_arrow.color.a = 0.9
                    marker_array.markers.append(marker_arrow)

                    # Marker Text nhãn tên
                    marker_txt = Marker()
                    marker_txt.header.frame_id = self.target_frame
                    marker_txt.header.stamp = pose_array.header.stamp
                    marker_txt.ns = 'tube_labels'
                    marker_txt.id = marker_id + 100
                    marker_txt.type = Marker.TEXT_VIEW_FACING
                    marker_txt.action = Marker.ADD
                    marker_txt.pose.position.x = x_rob
                    marker_txt.pose.position.y = y_rob
                    marker_txt.pose.position.z = z_rob + 0.04
                    marker_txt.scale.z = 0.02
                    marker_txt.color.r = 1.0
                    marker_txt.color.g = 1.0
                    marker_txt.color.b = 1.0
                    marker_txt.color.a = 1.0
                    marker_txt.text = f"{cls_name}\n({deg:.0f}°)"
                    marker_array.markers.append(marker_txt)

                    marker_id += 1

        # Hiển thị Khung Hộp Vùng Làm Việc (Workspace ROI Box) trong RViz
        if enable_roi:
            roi_box_marker = Marker()
            roi_box_marker.header.frame_id = self.target_frame
            roi_box_marker.header.stamp = self.get_clock().now().to_msg()
            roi_box_marker.ns = 'workspace_roi'
            roi_box_marker.id = 999
            roi_box_marker.type = Marker.CUBE
            roi_box_marker.action = Marker.ADD
            roi_box_marker.pose.position.x = (roi_xmin + roi_xmax) / 2.0
            roi_box_marker.pose.position.y = (roi_ymin + roi_ymax) / 2.0
            roi_box_marker.pose.position.z = (roi_zmin + roi_zmax) / 2.0
            roi_box_marker.scale.x = roi_xmax - roi_xmin
            roi_box_marker.scale.y = roi_ymax - roi_ymin
            roi_box_marker.scale.z = roi_zmax - roi_zmin
            roi_box_marker.color.r = 0.0
            roi_box_marker.color.g = 0.8
            roi_box_marker.color.b = 1.0
            roi_box_marker.color.a = 0.15  # Hộp bán trong suốt màu Cyan
            marker_array.markers.append(roi_box_marker)

        # Publish toàn bộ kết quả
        if len(pose_array.poses) > 0 or enable_roi:
            self.pub_poses.publish(pose_array)
            str_msg = String()
            str_msg.data = json.dumps(tube_classes)
            self.pub_classes.publish(str_msg)
            self.pub_markers.publish(marker_array)

        # Publish ảnh Debug
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = rgb_msg.header
            self.pub_debug_img.publish(debug_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloTubeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
