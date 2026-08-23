# Pipeline nhận diện PCL & AprilTag/ArmTag cho RX150 + RealSense D435i
# Tích hợp trực tiếp interbotix_perception_modules theo hướng dẫn Trossen Robotics.
#
# ĐIỀU KIỆN CHẠY: camera + robot TF phải đang chạy từ fuzzy_moveit.launch.py
# (use_camera:=true) — file này KHÔNG tự khởi động driver RealSense:
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py use_camera:=true
#
# Chạy PCL PointCloud Pipeline:
#   ros2 launch rx150_perception perception.launch.py
# Bật PCL Tuner GUI (Giao diện chỉnh tham số lọc CropBox, SAC Plane, Voxel, Cluster):
#   ros2 launch rx150_perception perception.launch.py enable_pipeline:=true use_pointcloud_tuner_gui:=true use_rviz:=true
# Lưu ý: nếu bật use_armtag:=true, xem runbook mutex TF trong armtag.launch.py.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_tags_config = PathJoinSubstitution([
        FindPackageShare('rx150_perception'),
        'config', 'tags.yaml',
    ])

    default_filter_params = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_perception'),
        'config', 'filter_params.yaml',
    ])

    pc_filter_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('interbotix_perception_modules'),
            'launch', 'pc_filter.launch.py',
        ])),
        launch_arguments={
            'filter_ns': LaunchConfiguration('filter_ns'),
            'filter_params': LaunchConfiguration('filter_params'),
            'enable_pipeline': LaunchConfiguration('enable_pipeline'),
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'use_pointcloud_tuner_gui': LaunchConfiguration('use_pointcloud_tuner_gui'),
        }.items(),
    )

    armtag_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('interbotix_perception_modules'),
            'launch', 'armtag.launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('use_armtag')),
        launch_arguments={
            'tags_config': LaunchConfiguration('tags_config'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'camera_color_topic': LaunchConfiguration('camera_color_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'arm_base_frame': LaunchConfiguration('arm_base_frame'),
            'arm_tag_frame': LaunchConfiguration('arm_tag_frame'),
            'use_armtag_tuner_gui': LaunchConfiguration('use_armtag_tuner_gui'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # file này giả định robot đang chạy từ fuzzy_moveit -> frame world luôn tồn tại.
        arguments=['-f', 'world'],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'filter_ns', default_value='pc_filter',
            description='namespace cho pointcloud_pipeline.'),
        DeclareLaunchArgument(
            'filter_params', default_value=default_filter_params,
            description='file filter params (voxel/crop/plane/cluster).'),
        DeclareLaunchArgument(
            'tags_config', default_value=default_tags_config,
            description='file cấu hình AprilTag (tag36h11 id=0 30mm).'),
        DeclareLaunchArgument(
            'enable_pipeline', default_value='false',
            description='true = chạy liên tục; false = chỉ chạy khi gọi service.'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/camera/camera/depth/color/points',
            description='topic pointcloud RealSense D435i.'),
        DeclareLaunchArgument(
            'camera_color_topic', default_value='/camera/camera/color/image_raw',
            description='topic image_raw RealSense D435i.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/color/camera_info',
            description='topic camera_info RealSense D435i.'),
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_color_optical_frame',
            description='frame optical camera D435i.'),
        DeclareLaunchArgument(
            'arm_base_frame', default_value='world',
            description='frame gốc robot. Đặt world (URDF đã có world->rx150/base_link; '
                        'đặt base_link sẽ tạo 2 parent sau khi snap).'),
        DeclareLaunchArgument(
            'arm_tag_frame', default_value='rx150/ee_gripper_link',
            description='frame dán AprilTag trên arm.'),
        DeclareLaunchArgument(
            'use_pointcloud_tuner_gui', default_value='false',
            description='hiện GUI tune filter online.'),
        DeclareLaunchArgument(
            'use_armtag', default_value='false',
            description='bật armtag launch cho AprilTag camera pose estimation.'),
        DeclareLaunchArgument(
            'use_armtag_tuner_gui', default_value='false',
            description='bật ArmTag Tuner GUI.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='mở giao diện RViz2 visualizer.'),
        pc_filter_include,
        armtag_include,
        rviz_node,
    ])
