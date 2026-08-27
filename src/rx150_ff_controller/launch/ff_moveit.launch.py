# BSD 3-Clause License
# Copyright (c) hust
#
# Launch MoveIt + ff PWM backend cho rx150.
#
# Kiến trúc:
#   xsarm_control (xs_sdk driver)  →  ff_node (PWM closed-loop, fuzzy PD + FF vel/acc)
#                                   →  ff_trajectory_bridge (action server)
#                                   →  move_group (MoveIt planning)
#                                   →  rviz2 (tuỳ chọn)
#
# MoveIt plan trajectory → bridge nội suy → ff_node bám setpoint bằng PWM.

import os

from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_common import (
    get_interbotix_xsarm_models,
)
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xsarm_semantic_robot_description_command,
    declare_interbotix_xsarm_robot_description_launch_arguments,
)
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
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def launch_setup(context, *args, **kwargs):

    robot_model_launch_arg = LaunchConfiguration('robot_model')
    robot_name_launch_arg = LaunchConfiguration('robot_name')
    use_sim_launch_arg = LaunchConfiguration('use_sim')
    load_configs_launch_arg = LaunchConfiguration('load_configs')
    use_moveit_rviz_launch_arg = LaunchConfiguration('use_moveit_rviz')
    rviz_config_file_launch_arg = LaunchConfiguration('rviz_config_file')
    robot_description_launch_arg = LaunchConfiguration('robot_description')

    # Camera (RealSense + hand-eye calibration) launch args
    use_camera_launch_arg = LaunchConfiguration('use_camera')
    rs_camera_pointcloud_enable_launch_arg = LaunchConfiguration('rs_camera_pointcloud_enable')
    rs_camera_align_depth_enable_launch_arg = LaunchConfiguration('rs_camera_align_depth_enable')
    rs_camera_rgb_profile_launch_arg = LaunchConfiguration('rs_camera_rgb_profile')
    rs_camera_depth_profile_launch_arg = LaunchConfiguration('rs_camera_depth_profile')
    rs_camera_serial_launch_arg = LaunchConfiguration('rs_camera_serial')
    use_camera_static_tf_launch_arg = LaunchConfiguration('use_camera_static_tf')
    use_handeye_publisher_launch_arg = LaunchConfiguration('use_handeye_publisher')

    robot_model = robot_model_launch_arg.perform(context)
    robot_name = robot_name_launch_arg.perform(context)

    # ------------------------------------------------------------------ #
    # 1. xsarm_control (xs_sdk driver — classic, NOT ros2_control)       #
    # ------------------------------------------------------------------ #
    xsarm_launch = os.path.join(
        get_package_share_directory('interbotix_xsarm_control'),
        'launch',
        'xsarm_control.launch.py')

    motor_configs = os.path.join(
        get_package_share_directory('rx150_motion_common'),
        'config',
        'rx150_motor.yaml')

    xsarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xsarm_launch),
        launch_arguments={
            'robot_model': robot_model,
            'robot_name': robot_name,
            'motor_configs': motor_configs,
            'load_configs': load_configs_launch_arg,
            'use_sim': use_sim_launch_arg,
            'use_rviz': 'false',
        }.items())

    # ------------------------------------------------------------------ #
    # 2. ff_node (PWM closed-loop controller, fuzzy PD + FF vel/acc)     #
    # ------------------------------------------------------------------ #
    ff_params = os.path.join(
        get_package_share_directory('rx150_ff_controller'),
        'config',
        'rx150_ff_gains.yaml')

    ff_node = Node(
        package='rx150_ff_controller',
        executable='ff_node',
        name='ff_node',
        namespace=robot_name,
        output='screen',
        parameters=[
            ff_params,
        ])

    # ------------------------------------------------------------------ #
    # 3. ff_trajectory_bridge (FollowJointTrajectory action server)      #
    # ------------------------------------------------------------------ #
    bridge_node = Node(
        package='rx150_motion_common',
        executable='rx150_trajectory_bridge.py',
        name='ff_trajectory_bridge',
        namespace=robot_name,
        output='screen',
        parameters=[{'setpoint_topic': 'ff/setpoint'}])

    # ------------------------------------------------------------------ #
    # 4. move_group (MoveIt planning)                                    #
    # ------------------------------------------------------------------ #
    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
    ])

    robot_description = {'robot_description': robot_description_launch_arg}

    robot_description_semantic = {
        'robot_description_semantic':
            construct_interbotix_xsarm_semantic_robot_description_command(
                robot_model=robot_model,
                config_path=config_path,
            ),
    }

    kinematics_config = PathJoinSubstitution([
        FindPackageShare('interbotix_xsarm_moveit'),
        'config',
        'kinematics.yaml',
    ])

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin':
                'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error':
                0.1,
        }
    }

    ompl_planning_pipeline_yaml_file = load_yaml(
        'interbotix_xsarm_moveit', 'config/ompl_planning.yaml')
    if ompl_planning_pipeline_yaml_file:
        ompl_planning_pipeline_config['move_group'].update(
            ompl_planning_pipeline_yaml_file)

    controllers_config = load_yaml(
        'interbotix_xsarm_moveit',
        f'config/controllers/{robot_model}_controllers.yaml')

    config_joint_limits = load_yaml(
        'interbotix_xsarm_moveit',
        f'config/joint_limits/{robot_model}_joint_limits.yaml')

    joint_limits = {
        'robot_description_planning': config_joint_limits,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # QUAN TRỌNG: moveit_manage_controllers = False vì không dùng ros2_control.
    # Bridge tự cung cấp action server, MoveIt chỉ cần gửi goal.
    trajectory_execution_parameters = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    use_octomap = LaunchConfiguration('use_octomap').perform(context).lower() == 'true'
    sensor_parameters = load_yaml('rx150_motion_common', 'config/sensors_3d.yaml') if use_octomap else {}

    # OctoMap reads /camera/camera/depth/color/points, which the driver only publishes when
    # rs_camera_pointcloud_enable is true. That default is now false (the stream is ~295 MB/s
    # and has no other consumer), so say so loudly instead of silently building an empty map.
    if use_octomap and rs_camera_pointcloud_enable_launch_arg.perform(context).lower() != 'true':
        print(
            '[WARN] use_octomap:=true but rs_camera_pointcloud_enable:=false -> move_group '
            'will never receive a pointcloud and the OctoMap stays empty. Relaunch with '
            'rs_camera_pointcloud_enable:=true.'
        )

    remappings = [
        (
            f'{robot_name}/get_planning_scene',
            f'/{robot_name}/get_planning_scene'
        ),
        (
            '/arm_controller/follow_joint_trajectory',
            f'/{robot_name}/arm_controller/follow_joint_trajectory'
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            f'/{robot_name}/gripper_controller/follow_joint_trajectory'
        ),
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description':
                        'robot_description',
                    'joint_state_topic':
                        f'/{robot_name}/joint_states',
                },
                'use_sim_time': False,
            },
            robot_description,
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            trajectory_execution_parameters,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits,
        ] + ([sensor_parameters] if (use_octomap and sensor_parameters) else []),
        remappings=remappings,
        output={'both': 'screen'},
    )

    # ------------------------------------------------------------------ #
    # 5. rviz (optional)                                                 #
    # ------------------------------------------------------------------ #
    moveit_rviz_node = Node(
        condition=IfCondition(use_moveit_rviz_launch_arg),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz_config_file_launch_arg,
            '-f', 'world',
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_config,
            {'use_sim_time': False},
        ],
        # Render on the discrete NVIDIA GPU. `prime-select` is `on-demand` on this machine,
        # so without these RViz falls back to the Intel iGPU, which shares RAM bandwidth
        # with the CPUs and drives the display -- point clouds there stall the compositor.
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__VK_LAYER_NV_optimus': 'NVIDIA_only',
        },
        remappings=remappings,
        output={'both': 'log'},
    )

    # ------------------------------------------------------------------ #
    # 6. Camera static TF — hardcoded `world -> camera_link` (Eye-to-hand)
    #    Dùng làm initial-guess KHI CHƯA calibration. Khi đã có hand-eye
    #    calibration, bật use_handeye_publisher:=true và use_camera_static_tf:=false
    #    để tránh 2 publisher cùng frame gây nhảy TF.
    # ------------------------------------------------------------------ #
    static_tf = Node(
        condition=IfCondition(use_camera_static_tf_launch_arg),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=['1.0', '0.0', '1.0', '0', '1.57', '0', 'world', 'camera_link']
    )

    # ------------------------------------------------------------------ #
    # 7. RealSense camera driver (optional)                              #
    #    publish_tf:=true → rs driver tự publish chuỗi                    #
    #    camera_link -> camera_color_frame -> camera_color_optical_frame  #
    #    (và depth frames). Không cần URDF camera riêng.                  #
    # ------------------------------------------------------------------ #
    rs_camera_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py')

    rs_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_camera_launch_path),
        condition=IfCondition(use_camera_launch_arg),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            'pointcloud.enable': rs_camera_pointcloud_enable_launch_arg,
            'align_depth.enable': rs_camera_align_depth_enable_launch_arg,
            'rgb_camera.color_profile': rs_camera_rgb_profile_launch_arg,
            'depth_module.depth_profile': rs_camera_depth_profile_launch_arg,
            'serial_no': rs_camera_serial_launch_arg,
            'publish_tf': 'true',
        }.items())

    # ------------------------------------------------------------------ #
    # 8. Hand-eye calibration TF publisher (optional)                    #
    #    Publish calibrated `world -> camera_link` từ easy_handeye2       #
    #    (xem handeye_publish.launch.py). Mutually exclusive với static_tf.#
    # ------------------------------------------------------------------ #
    handeye_publish_launch_path = os.path.join(
        get_package_share_directory('rx150_perception'),
        'launch',
        'handeye_publish.launch.py')

    handeye_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(handeye_publish_launch_path),
        condition=IfCondition(use_handeye_publisher_launch_arg))

    return [
        xsarm,
        ff_node,
        bridge_node,
        move_group_node,
        moveit_rviz_node,
        rs_camera,
        static_tf,
        handeye_publisher,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='rx150',
            choices=get_interbotix_xsarm_models(),
            description='model type of the Interbotix Arm such as `rx150`.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('robot_model'),
            description='name of the robot (typically equal to robot_model).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'uses xs_sdk_sim instead of the physical xs_sdk driver; useful for safe launch '
                'and MoveIt verification.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description="launches RViz with MoveIt's RViz configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'load_configs',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'Write motor register config to EEPROM at startup. Enable only after '
                'changing the motor config or replacing a motor.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'rviz',
                'xsarm_moveit.rviz'
            ]),
            description='file path to the config file RViz should load.',
        )
    )
    # external_srdf_loc: declare riêng vì helper declare_interbotix_xsarm_robot_description_*
    # KHÔNG khai báo nó, nhưng construct_interbotix_xsarm_semantic_robot_description_command
    # lại reference LaunchConfiguration('external_srdf_loc'). Thiếu -> launch fail ngay.
    declared_arguments.append(
        DeclareLaunchArgument(
            'external_srdf_loc',
            default_value=TextSubstitution(text=''),
            description=(
                'the file path to the custom semantic description file that you would like to '
                "include in the Interbotix robot's semantic description."
            ),
        )
    )

    # ------------------------------------------------------------------ #
    # Camera (RealSense + hand-eye calibration) arguments                #
    # ------------------------------------------------------------------ #
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_camera',
            default_value='true',
            choices=('true', 'false'),
            description='launch the RealSense camera driver (realsense2_camera rs_launch).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_pointcloud_enable',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'enable the colored pointcloud stream (rs_launch pointcloud.enable). OFF by '
                'default: XYZRGB 640x480x30 is ~295 MB/s and nothing subscribes to it in the '
                'normal flow. Turn it back on for use_octomap:=true or the PCL pipeline.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_octomap',
            default_value='false',
            choices=('true', 'false'),
            description='enable OctoMap 3D obstacle avoidance from pointcloud. Disabled by default.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_align_depth_enable',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'register depth onto color (rs_launch align_depth.enable); required for the '
                'colored `/depth/color/points` topic used by MoveIt OctoMap and the PCL pipeline.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_rgb_profile',
            default_value='640x480x30',
            description='color stream profile `<W>x<H>x<FPS>` (rs_launch rgb_camera.color_profile).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_depth_profile',
            default_value='640x480x30',
            description='depth stream profile `<W>x<H>x<FPS>` (rs_launch depth_module.depth_profile).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rs_camera_serial',
            default_value='',
            description='RealSense serial number; leave empty to auto-pick the first device.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_camera_static_tf',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'publish the hardcoded `world -> camera_link` static TF as an initial camera pose '
                'guess. Set to false once a hand-eye calibration is available.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_handeye_publisher',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'publish the calibrated `world -> camera_link` TF from easy_handeye2 via '
                'handeye_publish.launch.py. Mutually exclusive with use_camera_static_tf.'
            ),
        )
    )

    declared_arguments.extend(
        declare_interbotix_xsarm_robot_description_launch_arguments(
            show_gripper_bar='true',
            show_gripper_fingers='true',
            hardware_type='actual',
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])
