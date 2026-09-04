# Layer 1 (YOLO tube detector) + Layer 2 (pick-place theo cử chỉ tay qua MoveIt).
#
# Yêu cầu T1 đã chạy (motion stack + camera + hand-eye):
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#       use_camera:=true use_camera_static_tf:=false use_handeye_publisher:=true
#
# Chạy T2:
#   ros2 launch rx150_pick_place pick_place.launch.py
#   ros2 launch rx150_pick_place pick_place.launch.py dry_run:=true     # không động cơ
#   ros2 launch rx150_pick_place pick_place.launch.py enable_detector:=false
#   ros2 launch rx150_pick_place pick_place.launch.py enable_gesture:=false   # tự chạy riêng
#
# Kiểm tra tầm với TRƯỚC khi chạy (không cần robot):
#   ros2 run rx150_pick_place rx150_reach_check.py --config \
#       $(ros2 pkg prefix rx150_pick_place)/share/rx150_pick_place/config/pick_place_params.yaml
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = os.path.join(get_package_share_directory('rx150_pick_place'),
                               'config', 'pick_place_params.yaml')
    enable_detector = LaunchConfiguration('enable_detector')
    enable_gesture = LaunchConfiguration('enable_gesture')
    # ParameterValue(..., value_type=bool): nếu không ép kiểu, launch truyền
    # chuỗi "true" vào tham số đã khai báo kiểu bool ⇒ node chết vì
    # InvalidParameterTypeException.
    dry_run = ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_detector', default_value='true',
            description='Chạy yolo_tube_detector_node (false nếu detector chạy riêng).'),
        DeclareLaunchArgument(
            'enable_gesture', default_value='true',
            description='Chạy hand_gesture_node — NGUỒN của /hand_gesture/selected_target '
                        'và /hand_gesture/event. Không có nó thì node này không bao giờ '
                        'được kích (đứng ở "chưa chỉ tay chọn vật nào").'),
        DeclareLaunchArgument(
            'yolo_params_file', default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'), 'config',
                'yolo_detector_params.yaml']),
            description='Params YOLO (conf/imgsz/depth gate/EMA). `ros2 run` trần KHÔNG '
                        'nạp file này — chỉ qua launch mới có.'),
        DeclareLaunchArgument(
            'roi_params_file', default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'), 'config',
                'roi_box_params.yaml']),
            description='Hộp giới hạn vùng nhận diện (toạ độ base_link). Nạp SAU '
                        'yolo_params_file nên các khoá roi_* ở đây thắng.'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='true = chỉ lập kế hoạch + log, KHÔNG gửi goal tới robot.'),

        # ---- Layer 1: YOLO tube detector ----
        Node(
            package='rx150_perception',
            executable='yolo_tube_detector_node.py',
            name='yolo_tube_detector',
            output='screen',
            # Cùng bộ params mà fuzzy_moveit_perception.launch.py dùng — chạy detector
            # ở đây hay ở T1 thì hành vi nhận diện phải y hệt.
            parameters=[LaunchConfiguration('yolo_params_file'),
                        LaunchConfiguration('roi_params_file')],
            condition=IfCondition(enable_detector),
        ),

        # ---- Layer 1: chọn vật bằng cử chỉ tay (MediaPipe) ----
        # name='hand_gesture' là BẮT BUỘC: node publish topic private (~/selected_target,
        # ~/event) nên tên node quyết định thành /hand_gesture/... — khớp với
        # selected_target_topic / gesture_event_topic trong pick_place_params.yaml.
        Node(
            package='rx150_perception',
            executable='hand_gesture_node.py',
            name='hand_gesture',
            output='screen',
            condition=IfCondition(enable_gesture),
        ),

        # ---- Layer 2: quyết định pick-place ----
        Node(
            package='rx150_pick_place',
            executable='pick_place_moveit_node.py',
            name='pick_place_moveit',
            output='screen',
            parameters=[params_file, {'dry_run': dry_run}],
        ),
    ])
