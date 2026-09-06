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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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

    # rx150_perception KHÔNG có yolo_detector.launch.py — bản cũ include đúng tên đó
    # nên perception:=true là chết ngay. Chạy thẳng hai node như
    # rx150_pick_place/launch/{tube_rack,pick_place}.launch.py vẫn đang làm, để hành
    # vi nhận diện y hệt dù bật detector ở đường nào.
    yolo_params_file = os.path.join(
        perc_share, 'config', 'yolo_detector_params.yaml')
    roi_params_file = os.path.join(
        perc_share, 'config', 'roi_box_params.yaml')

    detector_node = Node(
        package='rx150_perception',
        executable='yolo_tube_detector_node.py',
        name='yolo_tube_detector',
        output='screen',
        parameters=[yolo_params_file, {'roi_params_file': roi_params_file}],
        condition=IfCondition(LaunchConfiguration('perception')),
    )
    # name='hand_gesture' là BẮT BUỘC: node publish topic private (~/selected_target,
    # ~/event) nên tên node quyết định thành /hand_gesture/...
    gesture_node = Node(
        package='rx150_perception',
        executable='hand_gesture_node.py',
        name='hand_gesture',
        output='screen',
        condition=IfCondition(LaunchConfiguration('perception')),
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
        detector_node,
        gesture_node,
        motion_node,
        task_node,
    ])
