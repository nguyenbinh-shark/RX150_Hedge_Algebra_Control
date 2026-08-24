# Launch Layer 1 (YOLO + gesture) + Layer 2 (node quyết định pick-place MoveIt).
#
# Yêu cầu T1 đã chạy (motion stack + camera + hand-eye):
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#       use_camera:=true rs_camera_pointcloud_enable:=true \
#       use_camera_static_tf:=false use_handeye_publisher:=true
#
# Chạy T2 (cái này) — detector mặc định là YOLO:
#   ros2 launch rx150_pick_place pick_place.launch.py
#
# Detector PCL (cluster từ pointcloud, KHÔNG cần YOLO) — T2 phải chạy pc_filter:
#   T2: ros2 launch rx150_perception perception.launch.py   # CHỦ pc_filter (mutex)
#   T3: ros2 launch rx150_pick_place pick_place.launch.py detector:=pcl
#   Quét vật: ros2 service call /cluster_bridge/refresh std_srvs/srv/Trigger
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pp_share = get_package_share_directory('rx150_pick_place')
    perc_share = get_package_share_directory('rx150_perception')
    params_file = os.path.join(pp_share, 'config', 'pick_place_params.yaml')

    detector = LaunchConfiguration('detector')
    is_yolo = PythonExpression(["'", detector, "' == 'yolo'"])
    is_pcl = PythonExpression(["'", detector, "' == 'pcl'"])
    detection_topic = PythonExpression(
        ["'/pcl/detected_objects' if '", detector, "' == 'pcl' "
         "else '/yolo/detected_objects'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'detector', default_value='yolo', choices=['yolo', 'pcl'],
            description='nguồn phát hiện vật: yolo (/yolo/detected_objects) hay '
                        'pcl (/pcl/detected_objects từ cluster_bridge).'),
        # ---- Layer 1a: YOLO detector + hand_gesture (gesture BẬT để chọn vật) ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perc_share, 'launch', 'yolo_detector.launch.py')),
            launch_arguments={'enable_gesture': 'true'}.items(),
            condition=IfCondition(is_yolo),
        ),
        # ---- Layer 1b: PCL cluster_bridge + hand_gesture (pc_filter chạy riêng ở T2) ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(perc_share, 'launch', 'pcl_detector.launch.py')),
            launch_arguments={'enable_gesture': 'true'}.items(),
            condition=IfCondition(is_pcl),
        ),
        # ---- Layer 2: node quyết định pick-place qua MoveIt ----
        # dict tường minh sau file yaml → giá trị launch thắng default trong yaml.
        Node(
            package='rx150_pick_place',
            executable='pick_place_moveit_node.py',
            name='pick_place_moveit',
            output='screen',
            parameters=[params_file, {'detection_topic': detection_topic}],
        ),
    ])
