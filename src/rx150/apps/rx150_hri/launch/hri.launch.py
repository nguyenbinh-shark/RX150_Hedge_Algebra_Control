# Launch chức năng HRI: executor (hri_motion) + task (hri_task).
#
# BƯỚC 1 (điểm cố định — không cần camera): T1 chạy motion stack trước
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py use_camera:=false
# rồi T2 (cái này):
#   ros2 launch rx150_hri hri.launch.py mode:=fixed perception:=false
#
# BƯỚC 2 (camera — tới): T1 use_camera:=true + hand-eye TF, T2:
#   ros2 launch rx150_hri hri.launch.py mode:=camera perception:=true
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hri_share = get_package_share_directory('rx150_hri')
    perc_share = get_package_share_directory('rx150_perception')
    params_file = os.path.join(hri_share, 'config', 'hri_params.yaml')

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='fixed',
        description='Chế độ task: fixed (B1) | camera (B2 — tới)')
    perception_arg = DeclareLaunchArgument(
        'perception', default_value='false',
        description='true = spawn thêm YOLO detector (+ hand_gesture) của rx150_perception')

    perception_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perc_share, 'launch', 'yolo_detector.launch.py')),
        condition=IfCondition(LaunchConfiguration('perception')),
        launch_arguments={'enable_gesture': 'true'}.items(),
    )

    motion_node = Node(
        package='rx150_hri',
        executable='hri_motion_node.py',
        name='hri_motion',
        output='screen',
        parameters=[params_file],
    )
    task_node = Node(
        package='rx150_hri',
        executable='hri_task_node.py',
        name='hri_task',
        output='screen',
        parameters=[params_file, {'mode': LaunchConfiguration('mode')}],
    )

    return LaunchDescription([
        mode_arg,
        perception_arg,
        perception_include,
        motion_node,
        task_node,
    ])
