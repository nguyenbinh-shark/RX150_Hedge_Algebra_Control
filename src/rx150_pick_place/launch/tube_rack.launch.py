# Detector + tube_rack_node (gắp ống nghiệm → cắm lên giá, phân theo màu).
#
# Yêu cầu T1 đã chạy:
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#       use_camera:=true use_camera_static_tf:=false
#
# Chạy:
#   ros2 launch rx150_pick_place tube_rack.launch.py
#   ros2 launch rx150_pick_place tube_rack.launch.py dry_run:=true
#   ros2 launch rx150_pick_place tube_rack.launch.py auto_start:=false
#     → rồi kích tay: ros2 service call /tube_rack/run std_srvs/srv/Trigger
#   ros2 launch rx150_pick_place tube_rack.launch.py detector:=false
#
# Dừng khẩn cấp:  ros2 service call /tube_rack/stop  std_srvs/srv/Trigger
# Chạy lại:       ros2 service call /tube_rack/reset std_srvs/srv/Trigger
# Xem trạng thái: ros2 topic echo /tube_rack/status
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
                               'config', 'tube_rack_params.yaml')
    detector_flag = LaunchConfiguration('detector')
    # ParameterValue(..., value_type=bool): nếu không ép kiểu, launch truyền
    # chuỗi "true" vào tham số đã khai báo kiểu bool ⇒ node chết vì
    # InvalidParameterTypeException.
    dry_run = ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)
    auto_start = ParameterValue(LaunchConfiguration('auto_start'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'detector', default_value='true',
            description='Chạy yolo_tube_detector_node (false nếu detector chạy riêng).'),
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
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Tự chạy chu kỳ khi khởi động (false = chờ /tube_rack/run).'),

        Node(
            package='rx150_perception',
            executable='yolo_tube_detector_node.py',
            name='yolo_tube_detector',
            output='screen',
            condition=IfCondition(detector_flag),
        ),

        Node(
            package='rx150_pick_place',
            executable='tube_rack_node.py',
            name='tube_rack',
            output='screen',
            parameters=[params_file,
                        {'dry_run': dry_run, 'auto_start': auto_start}],
        ),
    ])
