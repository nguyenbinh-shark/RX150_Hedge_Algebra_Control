# Detector + tube_rack_node (gắp ống nghiệm → cắm lên giá, phân theo màu).
#
# Yêu cầu T1 + T2 đã chạy (motion stack + camera, RỒI perception phát TF calib):
#   ./rx150.sh t1   &&   ./rx150.sh t2
#
# TF `world <-> camera` CHỈ do static_trans_pub của rx150_perception phát;
# fuzzy_moveit.launch.py KHÔNG chạy node đó. Thiếu TF ⇒ YOLO im lặng, trông y hệt
# "model kém". Kiểm: ros2 run tf2_ros tf2_echo camera_color_optical_frame rx150/base_link
#
# Đi đường một-terminal (fuzzy_moveit_perception, đã có YOLO) thì PHẢI detector:=false.
#
# Chạy:
#   ros2 launch rx150_pick_place tube_rack.launch.py
#   ros2 launch rx150_pick_place tube_rack.launch.py dry_run:=true
#   ros2 launch rx150_pick_place tube_rack.launch.py auto_start:=false
#     → rồi kích tay: ros2 service call /tube_rack/run std_srvs/srv/Trigger
#   ros2 launch rx150_pick_place tube_rack.launch.py detector:=false
#
# Thử TOÀN CHUỖI khi KHÔNG có camera (ống giả, chỉ lập kế hoạch, không động cơ):
#   ros2 launch rx150_pick_place tube_rack.launch.py \
#       detector:=false dry_run:=true \
#       fake_tubes:='[{"x":0.20,"y":-0.15,"z":0.03,"yaw":0.0,"class":"pink"}]'
#   (KHÔNG truyền được bằng `ros2 run -p fake_tubes:=…`: rcl parse giá trị -p
#    theo YAML nên JSON array thành flow-sequence → "Unknown YAML event".)
#
# Chấp hành KHÔNG qua move_group (mô hình key_point — không planner, không
# tránh vật cản; an toàn nằm ở waypoint):
#   ros2 launch rx150_pick_place tube_rack.launch.py motion_backend:=direct
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
    # fake_tubes là CHUỖI JSON — ép value_type=str để launch không cố suy diễn
    # kiểu rồi biến '[{...}]' thành mảng.
    fake_tubes = ParameterValue(LaunchConfiguration('fake_tubes'), value_type=str)

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
            description='Hộp giới hạn vùng nhận diện (toạ độ base_link). Truyền dạng '
                        'THAM SỐ roi_params_file: node tự mở file này rồi '
                        'set_parameters(), nên các khoá roi_* ở đây LUÔN thắng '
                        'params-file. Đặt \'none\' để ROI đến từ yolo_params_file/CLI.'),
        DeclareLaunchArgument(
            'dry_run', default_value='false',
            description='true = chỉ lập kế hoạch + log, KHÔNG gửi goal tới robot.'),
        DeclareLaunchArgument(
            'auto_start', default_value='true',
            description='Tự chạy chu kỳ khi khởi động (false = chờ /tube_rack/run).'),
        DeclareLaunchArgument(
            'fake_tubes', default_value='[]',
            description='JSON danh sách ống giả để thử chuỗi khi không có camera. '
                        'Ví dụ: \'[{"x":0.2,"y":-0.15,"z":0.03,"yaw":0,"class":"pink"}]\''),
        DeclareLaunchArgument(
            'motion_backend', default_value='moveit',
            description='moveit = qua move_group (OMPL + planning scene). '
                        'direct = bắn thẳng FollowJointTrajectory xuống '
                        'fuzzy_trajectory_bridge, KHÔNG planner.'),

        # Cùng bộ params mà fuzzy_moveit_perception.launch.py dùng — chạy detector
        # ở đây hay ở T1 thì hành vi nhận diện phải y hệt.
        #
        # roi_params_file đi vào dạng THAM SỐ (dict), không phải params-file: node
        # tự mở file trỏ bởi tham số đó rồi set_parameters(), nên đường tự-nạp LUÔN
        # thắng params-file. Truyền file ROI như params-file thì `roi_params_file:=`
        # tuỳ chỉnh sẽ bị chính default của node ghi đè, im lặng.
        Node(
            package='rx150_perception',
            executable='yolo_tube_detector_node.py',
            name='yolo_tube_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('yolo_params_file'),
                {'roi_params_file': LaunchConfiguration('roi_params_file')},
            ],
            condition=IfCondition(detector_flag),
        ),

        Node(
            package='rx150_pick_place',
            executable='tube_rack_node.py',
            name='tube_rack',
            output='screen',
            parameters=[params_file,
                        {'dry_run': dry_run, 'auto_start': auto_start,
                         'fake_tubes': fake_tubes,
                         'motion_backend': LaunchConfiguration('motion_backend')}],
        ),
    ])
