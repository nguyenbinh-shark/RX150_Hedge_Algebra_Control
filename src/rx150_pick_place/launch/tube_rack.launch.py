# tube_rack.launch.py — Self-contained launch: detector + tube_rack_node.
#
# Yêu cầu T1 đã chạy (motion stack + camera + hand-eye):
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#       use_camera:=true use_camera_static_tf:=false
#   (hoặc rx150_hac_controller hac_moveit.launch.py ...)
#
# Chạy:
#   ros2 launch rx150_pick_place tube_rack.launch.py
#   ros2 launch rx150_pick_place tube_rack.launch.py dry_run:=true
#   ros2 launch rx150_pick_place tube_rack.launch.py auto_start:=false
#   ros2 launch rx150_pick_place tube_rack.launch.py detector:=false  # nếu detector chạy riêng
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pp_share = get_package_share_directory('rx150_pick_place')
    params_file = os.path.join(pp_share, 'config', 'tube_rack_params.yaml')

    detector_flag = LaunchConfiguration('detector')
    dry_run = LaunchConfiguration('dry_run')
    auto_start = LaunchConfiguration('auto_start')

    return LaunchDescription([
        DeclareLaunchArgument(
            'detector', default_value='true',
            description='Chạy yolo_tube_detector_node (true) hoặc giả định đã chạy riêng (false).'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='Chỉ tính IK + log, không gửi MoveGroup goal.'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Tự chạy chu kỳ khi khởi động (false = chờ /tube_rack/run).'),

        # ── YOLO detector (từ rx150_perception, đã install qua CMakeLists) ──
        Node(
            package='rx150_perception',
            executable='yolo_tube_detector_node.py',
            name='yolo_tube_detector',
            output='screen',
            condition=IfCondition(detector_flag),
        ),

        # ── tube_rack_node ──
        Node(
            package='rx150_pick_place',
            executable='tube_rack_node.py',
            name='tube_rack',
            output='screen',
            parameters=[
                params_file,
                {'dry_run': dry_run, 'auto_start': auto_start},
            ],
        ),
    ])
