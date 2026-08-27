# Launch Layer 1 (YOLO tube detector) + Layer 2 (node quyết định pick-place MoveIt).
#
# Yêu cầu T1 đã chạy (motion stack + camera + hand-eye):
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#       use_camera:=true rs_camera_pointcloud_enable:=true \
#       use_camera_static_tf:=false use_handeye_publisher:=true
#
# Chạy T2:
#   ros2 launch rx150_pick_place pick_place.launch.py
#
# NOTE: yolo_detector.launch.py và pcl_detector.launch.py đã bị xóa (commit 60a6dde).
#       Bây giờ chạy yolo_tube_detector_node.py trực tiếp. Nhánh PCL đã bỏ.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pp_share = get_package_share_directory('rx150_pick_place')
    params_file = os.path.join(pp_share, 'config', 'pick_place_params.yaml')

    enable_detector = LaunchConfiguration('enable_detector')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_detector', default_value='true',
            description='Chạy yolo_tube_detector_node (true) hoặc giả định đã chạy riêng (false).'),

        # ---- Layer 1: YOLO tube detector (chạy trực tiếp, không cần launch file) ----
        Node(
            package='rx150_perception',
            executable='yolo_tube_detector_node.py',
            name='yolo_tube_detector',
            output='screen',
            condition=IfCondition(enable_detector),
        ),

        # ---- Layer 2: node quyết định pick-place qua MoveIt ----
        Node(
            package='rx150_pick_place',
            executable='pick_place_moveit_node.py',
            name='pick_place_moveit',
            output='screen',
            # detection_topic nằm trong params_file (/yolo/detected_tubes) — không override
            # ở đây nữa, tránh lệch với topic mà yolo_tube_detector_node thực sự publish.
            parameters=[params_file],
        ),
    ])

