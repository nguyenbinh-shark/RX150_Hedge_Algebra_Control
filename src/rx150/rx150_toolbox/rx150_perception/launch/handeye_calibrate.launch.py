# Hand-eye calibration eye-to-hand cho rx150: easy_handeye2 + AprilTag.
#
# Launch này KHÔNG include xsarm_control (tránh 2 xs_sdk cùng bus) và KHÔNG
# include rs_launch (camera đã do fuzzy_moveit.launch.py use_camera:=true chạy).
# => Phải chạy fuzzy_moveit.launch.py ở terminal khác TRƯỚC khi launch file này.
#
# Quy trình:
#   T1:  ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#            use_camera:=true use_camera_static_tf:=false use_moveit_rviz:=true
#        (use_camera_static_tf:=false để easy_handeye2 dummy sở hữu world->camera_link)
#   T2:  ros2 launch rx150_perception handeye_calibrate.launch.py
#   Trong rqt "easy_handeye2 calibration" GUI:
#     - di chuyển arm tới nhiều tư thế nghiêng khác nhau (qua MotionPlanning rviz)
#     - bấm "Take sample" >= 15-20 mẫu đa dạng
#     - chọn solver (vd Tsai-Lenz), "Compute", "Save calibration"
#   Sau đó: tắt cả 2 terminal, chạy lại fuzzy_moveit với use_handeye_publisher:=true.
#
# Tag (mặc định tag_0 = tag36h11 id=0, cạnh 50mm) phải dán cố định lên mặt trên
# gripper (gần rx150/ee_gripper_link). Xem config/apriltag_calib.yaml để đổi tag.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    apriltag_config = os.path.join(
        get_package_share_directory('rx150_perception'),
        'config', 'apriltag_calib.yaml')

    # AprilTag continuous detector = tracking source cho easy_handeye2.
    # Node sub `~/image_rect` & `~/camera_info`; remap về topic RealSense.
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        name='apriltag_ros_continuous_detector_node',
        output='screen',
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        parameters=[apriltag_config],
    )

    # easy_handeye2: handeye_server + rqt_calibrator (GUI lấy mẫu & giải).
    # eye_on_base => giải robot_base_frame -> tracking_base_frame (= world -> camera_link).
    handeye_calibrate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('easy_handeye2'),
            'launch', 'calibrate.launch.py')),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'calibration_type': LaunchConfiguration('calibration_type'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration('robot_effector_frame'),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'name', default_value='rx150_eob',
            description='tên calibration, dùng khi save/load (phải khớp handeye_publish).'),
        DeclareLaunchArgument(
            'calibration_type', default_value='eye_on_base',
            choices=('eye_in_hand', 'eye_on_base'),
            description='eye_on_base = eye-to-hand (camera cố định ngoài).'),
        DeclareLaunchArgument(
            'robot_base_frame', default_value='world',
            description='gốc robot. Nếu `tf2_echo world rx150/base_link` thất bại, '
                        'đổi thành rx150/base_link.'),
        DeclareLaunchArgument(
            'robot_effector_frame', default_value='rx150/ee_gripper_link',
            description='frame end-effector (tag gắn gần đây).'),
        DeclareLaunchArgument(
            'tracking_base_frame', default_value='camera_link',
            description='frame tham chiếu camera (rs driver publish optical chain từ đây).'),
        DeclareLaunchArgument(
            'tracking_marker_frame', default_value='tag_0',
            description='frame tag do apriltag publish; phải khớp tên tag trong apriltag_calib.yaml.'),
        apriltag_node,
        handeye_calibrate,
    ])
