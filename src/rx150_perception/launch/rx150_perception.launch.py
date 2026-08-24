# rx150_perception.launch.py — Pipeline nhận diện PCL cho RX150 + RealSense D435i
#
# Kiến trúc theo chuẩn Interbotix (interbotix_xsarm_perception).
# File này CHỈ khởi động perception nodes, KHÔNG khởi động robot hay camera.
#
# ĐIỀU KIỆN: Robot + Camera phải đang chạy từ fuzzy_moveit.launch.py:
#   T1: ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#           use_camera:=true use_camera_static_tf:=false
#
# Chạy perception:
#   T2: ros2 launch rx150_perception rx150_perception.launch.py
#
# Bật GUI tune pointcloud + snap ArmTag:
#   T2: ros2 launch rx150_perception rx150_perception.launch.py \
#           use_pointcloud_tuner_gui:=true use_armtag_tuner_gui:=true
#
# Chạy demo pick-place:
#   T3: cd src/rx150_perception/demos && python3 pick_place.py

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    filter_ns_launch_arg = LaunchConfiguration('filter_ns')
    filter_params_launch_arg = LaunchConfiguration('filter_params')
    use_pointcloud_tuner_gui_launch_arg = LaunchConfiguration('use_pointcloud_tuner_gui')
    enable_pipeline_launch_arg = LaunchConfiguration('enable_pipeline')
    cloud_topic_launch_arg = LaunchConfiguration('cloud_topic')

    tags_config_launch_arg = LaunchConfiguration('tags_config')
    camera_frame_launch_arg = LaunchConfiguration('camera_frame')
    apriltag_ns_launch_arg = LaunchConfiguration('apriltag_ns')
    camera_color_topic_launch_arg = LaunchConfiguration('camera_color_topic')
    camera_info_topic_launch_arg = LaunchConfiguration('camera_info_topic')
    armtag_ns_launch_arg = LaunchConfiguration('armtag_ns')
    ref_frame_launch_arg = LaunchConfiguration('ref_frame')
    arm_base_frame_launch_arg = LaunchConfiguration('arm_base_frame')
    arm_tag_frame_launch_arg = LaunchConfiguration('arm_tag_frame')
    use_armtag_tuner_gui_launch_arg = LaunchConfiguration('use_armtag_tuner_gui')
    position_only_launch_arg = LaunchConfiguration('position_only')

    load_transforms_launch_arg = LaunchConfiguration('load_transforms')
    transform_filepath_launch_arg = LaunchConfiguration('transform_filepath')

    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')

    # ---- 1. PointCloud Filter Pipeline (C++) ----
    pc_filter_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'launch',
                'pc_filter.launch.py',
            ])
        ]),
        launch_arguments={
            'filter_ns': filter_ns_launch_arg,
            'filter_params': filter_params_launch_arg,
            'enable_pipeline': enable_pipeline_launch_arg,
            'cloud_topic': cloud_topic_launch_arg,
            'use_pointcloud_tuner_gui': use_pointcloud_tuner_gui_launch_arg,
        }.items(),
    )

    # ---- 2. ArmTag (AprilTag detector + snap TF) ----
    armtag_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_perception_modules'),
                'launch',
                'armtag.launch.py',
            ])
        ]),
        launch_arguments={
            'tags_config': tags_config_launch_arg,
            'camera_frame': camera_frame_launch_arg,
            'apriltag_ns': apriltag_ns_launch_arg,
            'camera_color_topic': camera_color_topic_launch_arg,
            'camera_info_topic': camera_info_topic_launch_arg,
            'armtag_ns': armtag_ns_launch_arg,
            'ref_frame': ref_frame_launch_arg,
            'arm_base_frame': arm_base_frame_launch_arg,
            'arm_tag_frame': arm_tag_frame_launch_arg,
            'use_armtag_tuner_gui': use_armtag_tuner_gui_launch_arg,
            'position_only': position_only_launch_arg,
        }.items()
    )

    # ---- 3. Static Transform Publisher (load calibrated TF from YAML) ----
    static_transform_pub_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_tf_tools'),
                'launch',
                'static_transform_pub.launch.py',
            ])
        ]),
        launch_arguments={
            'load_transforms': load_transforms_launch_arg,
            'transform_filepath': transform_filepath_launch_arg,
        }.items()
    )

    # ---- 4. RViz (optional) ----
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-f', rviz_frame_launch_arg,
            '-d', rvizconfig_launch_arg,
        ],
        output={'both': 'log'},
        condition=IfCondition(use_rviz_launch_arg.perform(context)),
    )

    return [
        pc_filter_launch_include,
        armtag_launch_include,
        static_transform_pub_launch_include,
        rviz2_node,
    ]


def generate_launch_description():
    declared_arguments = []

    # ---- PointCloud Filter ----
    declared_arguments.append(
        DeclareLaunchArgument(
            'filter_ns',
            default_value='pc_filter',
            description='namespace where the pointcloud related nodes and parameters are located.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'filter_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'config',
                'filter_params.yaml'
            ]),
            description='file location of the parameters used to tune the perception pipeline filters.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_pointcloud_tuner_gui',
            default_value='false',
            choices=('true', 'false'),
            description='whether to show a GUI that a user can use to tune filter parameters.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_pipeline',
            default_value=LaunchConfiguration('use_pointcloud_tuner_gui'),
            choices=('true', 'false'),
            description=(
                'whether to enable the perception pipeline filters to run continuously; to save '
                'computer processing power, this should be set to false unless you are actively '
                'trying to tune the filter parameters; if false, the pipeline will only run if '
                'the get_cluster_positions ROS service is called.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cloud_topic',
            default_value='/camera/camera/depth/color/points',
            description='the absolute ROS topic name to subscribe to raw pointcloud data.',
        )
    )

    # ---- AprilTag / ArmTag ----
    declared_arguments.append(
        DeclareLaunchArgument(
            'tags_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'config',
                'tags.yaml'
            ]),
            description='parameter file location for the AprilTag configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_color_optical_frame',
            description='the camera frame in which the AprilTag will be detected.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'apriltag_ns',
            default_value='apriltag',
            description='namespace where the AprilTag related nodes and parameters are located.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_color_topic',
            default_value='/camera/camera/color/image_raw',
            description='the absolute ROS topic name to subscribe to color images.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/color/camera_info',
            description='the absolute ROS topic name to subscribe to the camera color info.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'armtag_ns',
            default_value='armtag',
            description='namespace where the Armtag related nodes and parameters are located.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ref_frame',
            default_value=LaunchConfiguration('camera_frame'),
            description=(
                'the reference frame that the armtag node should use when publishing a static '
                'transform for where the arm is relative to the camera.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_base_frame',
            default_value='world',
            description=(
                'the child frame for the static transform. Set to world when fuzzy_moveit '
                'uses use_world_frame:=true (URDF has world -> rx150/base_link).'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'arm_tag_frame',
            default_value='rx150/ar_tag_link',
            description='name of the frame on the arm where the AprilTag is located.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_armtag_tuner_gui',
            default_value='false',
            choices=('true', 'false'),
            description='whether to show a GUI to publish the ref_frame to arm_base_frame transform.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'position_only',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'whether only the position component of the detected AprilTag pose should be used.'
            ),
        )
    )

    # ---- Static Transform Publisher ----
    declared_arguments.append(
        DeclareLaunchArgument(
            'load_transforms',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'whether the static_trans_pub node should publish poses stored in '
                'static_transforms.yaml at startup.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'transform_filepath',
            default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'config',
                'static_transforms.yaml'
            ]),
            description=(
                'filepath to the static_transforms.yaml file used by the static_trans_pub node.'
            ),
        )
    )

    # ---- RViz ----
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            choices=('true', 'false'),
            description='launches RViz if set to true.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_frame',
            default_value='rx150/base_link',
            description='desired fixed frame in RViz.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'rviz',
                'rx150_perception.rviz'
            ]),
            description='filepath to the RViz config file.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

