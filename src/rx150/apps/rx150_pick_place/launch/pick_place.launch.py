# Layer 1 (YOLO tube detector) + Layer 2 (pick-place theo cử chỉ tay qua MoveIt).
#
# Yêu cầu T1 + T2 đã chạy (motion stack + camera, RỒI perception phát TF calib):
#   ./rx150.sh t1   &&   ./rx150.sh t2
# tương đương:
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#       use_camera:=true use_camera_static_tf:=false use_handeye_publisher:=false
#   ros2 launch rx150_perception rx150_perception.launch.py use_camera:=false
#
# TF `world <-> camera` CHỈ do static_trans_pub của rx150_perception phát.
# fuzzy_moveit.launch.py KHÔNG chạy node đó, và use_handeye_publisher:=true cần
# ~/.ros/easy_handeye2/rx150_eob.yaml (chưa có). Thiếu TF ⇒ YOLO im lặng.
# Kiểm: ros2 run tf2_ros tf2_echo camera_color_optical_frame rx150/base_link
#
# Chạy T3:
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
            description='Hộp giới hạn vùng nhận diện (toạ độ base_link). Truyền dạng '
                        'THAM SỐ roi_params_file: node tự mở file này rồi '
                        'set_parameters(), nên các khoá roi_* ở đây LUÔN thắng '
                        'params-file. Đặt \'none\' để ROI đến từ yolo_params_file/CLI.'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='true = chỉ lập kế hoạch + log, KHÔNG gửi goal tới robot.'),
        DeclareLaunchArgument(
            'motion_backend', default_value='moveit',
            description='moveit = qua move_group (OMPL + planning scene). '
                        'direct = bắn thẳng FollowJointTrajectory xuống '
                        'fuzzy_trajectory_bridge, KHÔNG planner (mô hình key_point).'),

        # ---- Layer 1: YOLO tube detector ----
        Node(
            package='rx150_perception',
            executable='yolo_tube_detector_node.py',
            name='yolo_tube_detector',
            output='screen',
            # Cùng bộ params mà fuzzy_moveit_perception.launch.py dùng — chạy detector
            # ở đây hay ở T1 thì hành vi nhận diện phải y hệt.
            #
            # roi_params_file đi vào dạng THAM SỐ (dict), không phải params-file: node
            # tự mở file trỏ bởi tham số đó rồi set_parameters(), nên đường tự-nạp LUÔN
            # thắng params-file. Truyền file ROI như params-file thì `roi_params_file:=`
            # tuỳ chỉnh sẽ bị chính default của node ghi đè, im lặng.
            parameters=[
                LaunchConfiguration('yolo_params_file'),
                {'roi_params_file': LaunchConfiguration('roi_params_file')},
            ],
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
            parameters=[params_file,
                        {'dry_run': dry_run,
                         'motion_backend': LaunchConfiguration('motion_backend')}],
        ),
    ])
