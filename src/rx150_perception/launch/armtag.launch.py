# Launch ArmTag tuner & static transform publisher cho RX150 + RealSense D435i
# Sử dụng interbotix_perception_modules với tag36h11 ID 0 kích thước 30mm (0.03m).
#
# QUAN TRỌNG — ArmTag CẦN robot TF (rx150/base_link -> rx150/ee_gripper_link) từ
# robot_state_publisher + joint_states của driver. KHÔNG chạy standalone chỉ với camera:
# nếu thiếu robot TF, armtag upstream vẫn publish transform RÁC (get_transform trả ma trận
# đơn vị khi lookup fail) và GUI vẫn báo "Successfully found...".
#
# Quy trình calibration camera->arm (2 terminal):
#   T1: ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#           use_camera:=true use_camera_static_tf:=false use_handeye_publisher:=false
#       (camera + robot TF; tắt 2 nguồn static TF world->camera_link để tránh xung đột)
#   Preflight: ros2 run tf2_ros tf2_echo rx150/base_link rx150/ee_gripper_link
#       (phải in ra số trước khi Snap Pose; timeout = robot TF chưa có)
#   T2: ros2 launch rx150_perception armtag.launch.py use_armtag_tuner_gui:=true
#   Trong GUI: torque off arm để xoay tay vị tag cho camera thấy rõ:
#       ros2 service call /rx150/torque_enable interbotix_xs_msgs/srv/TorqueEnable \
#           "{cmd_type: 'group', name: 'arm', enable: false}"
#   torque ON lại, giữ yên tay, bấm 'Snap Pose' (nên tăng Num Samples ~10).
#
# Sau khi snap thành công: node static_trans_pub broadcast TF lên /tf_static VÀ lưu
# kết quả vào static_transforms.yaml (nằm trong install share — BỊ XÓA khi colcon
# build). Copy về src để giữ lâu dài:
#   cp $(ros2 pkg prefix rx150_perception)/share/rx150_perception/config/static_transforms.yaml \
#      ~/interbotix_ws/src/rx150_perception/config/
#
# Tái dùng TF đã lưu (không GUI, không cần thấy tag):
#   ros2 launch rx150_perception armtag.launch.py \
#       use_armtag_tuner_gui:=false load_static_transforms:=true
#
# MUTEX TF: chỉ MỘT trong 3 nguồn static world<->camera được bật tại một thời điểm:
#   (1) static_transforms.yaml (file này), (2) use_camera_static_tf (fuzzy_moveit),
#   (3) handeye publisher (fuzzy_moveit). Bật >=2 nguồn -> TF multiple parents / cycle.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tags_config = PathJoinSubstitution([
        FindPackageShare('rx150_perception'),
        'config', 'tags.yaml',
    ])

    default_static_transforms_path = PathJoinSubstitution([
        FindPackageShare('rx150_perception'),
        'config', 'static_transforms.yaml',
    ])

    rs_camera_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch', 'rs_launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('use_camera')),
        launch_arguments={
            'camera_name': 'camera',
            # realsense2_camera 4.58.3: param là 'rgb_camera.color_profile' và
            # 'depth_module.depth_profile' (tên cũ 'rgb_camera.profile' bị drop silently).
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'pointcloud.enable': 'true',
        }.items(),
    )

    armtag_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('interbotix_perception_modules'),
            'launch', 'armtag.launch.py',
        ])),
        launch_arguments={
            'tags_config': tags_config,
            'camera_frame': LaunchConfiguration('camera_frame'),
            'camera_color_topic': LaunchConfiguration('camera_color_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'arm_base_frame': LaunchConfiguration('arm_base_frame'),
            'arm_tag_frame': LaunchConfiguration('arm_tag_frame'),
            'use_armtag_tuner_gui': LaunchConfiguration('use_armtag_tuner_gui'),
            'position_only': LaunchConfiguration('position_only'),
        }.items(),
    )

    # armtag GUI publish kết quả lên topic '/static_transforms' (KHÔNG phải /tf);
    # static_trans_pub là node duy nhất tiêu thụ topic đó -> broadcast /tf_static + lưu yaml.
    static_tf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('interbotix_tf_tools'),
            'launch', 'static_transform_pub.launch.py',
        ])),
        launch_arguments={
            'load_transforms': LaunchConfiguration('load_static_transforms'),
            'save_transforms': LaunchConfiguration('save_transforms'),
            'transform_filepath': LaunchConfiguration('static_transforms_path'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-f', LaunchConfiguration('rviz_frame')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera', default_value='false',
            description='true = tự bật driver RealSense D435i (CHỈ khi fuzzy_moveit '
                        'chưa chạy camera; 2 driver tranh nhau thiết bị).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='true = mở cửa sổ RViz2 3D visualizer.'),
        DeclareLaunchArgument(
            'rviz_frame', default_value='world',
            description='fixed frame RViz (world tồn tại khi robot đang chạy).'),
        # Lưu ý upstream: 2 args dưới là "chết" trong chuỗi armtag — picture_snapper
        # không declare chúng; GUI lấy frame từ camera_info header và đọc param
        # '/apriltag/camera_info_topic' với default cứng. Giữ để tương thích API include.
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_color_optical_frame',
            description='frame optical của RealSense D435i.'),
        DeclareLaunchArgument(
            'camera_color_topic', default_value='/camera/camera/color/image_raw',
            description='topic image color D435i.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/color/camera_info',
            description='topic camera_info D435i.'),
        DeclareLaunchArgument(
            'arm_base_frame', default_value='world',
            description='frame gốc robot. Đặt world (không phải rx150/base_link) vì '
                        'URDF MoveIt đã có world->rx150/base_link; nếu đặt base_link '
                        'thì base_link có 2 parent sau khi snap.'),
        DeclareLaunchArgument(
            'arm_tag_frame', default_value='rx150/ee_gripper_link',
            description='vị trí dán tag36h11 ID 0 (30mm) trên tay robot '
                        '(xấp xỉ: tag phải dán ngay gốc frame này).'),
        DeclareLaunchArgument(
            'use_armtag_tuner_gui', default_value='true',
            description='mở GUI ArmTag Tuner để bấm Snap Pose.'),
        DeclareLaunchArgument(
            'position_only', default_value='false',
            description='chỉ tinh chỉnh vị trí position nếu đã có orientation sơ bộ.'),
        DeclareLaunchArgument(
            'load_static_transforms', default_value='false',
            description='true = static_trans_pub phát lại TF từ static_transforms.yaml '
                        'lúc khởi động (tắt khi đang snap để tránh replay edge cũ).'),
        DeclareLaunchArgument(
            'save_transforms', default_value='true',
            description='lưu kết quả Snap Pose vào static_transforms.yaml.'),
        DeclareLaunchArgument(
            'static_transforms_path', default_value=default_static_transforms_path,
            description='file lưu TF. Mặc định trong install share (bị xóa khi rebuild '
                        '— copy về src sau khi snap).'),
        rs_camera_include,
        armtag_include,
        static_tf_include,
        rviz_node,
    ])
