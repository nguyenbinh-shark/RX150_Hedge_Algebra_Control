# BSD 3-Clause License
# Copyright (c) hust
#
# Launch the Interbotix rx150 under xsarm_control and start the fuzzy controller.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from interbotix_xs_modules.xs_launch import declare_interbotix_xsarm_robot_description_launch_arguments


def generate_launch_description():
    load_configs = LaunchConfiguration('load_configs')
    robot_description = LaunchConfiguration('robot_description')

    # Action 1 — bring up the xsarm driver/stack for the rx150, using our motor config.
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
            'robot_model': 'rx150',
            'robot_name': 'rx150',
            'motor_configs': motor_configs,
            'load_configs': load_configs,
            'use_sim': 'false',
            'use_rviz': 'false',
        }.items())

    # Action 2 — fuzzy controller node (namespace 'rx150' -> relative topics become /rx150/...).
    fuzzy_params = os.path.join(
        get_package_share_directory('rx150_fuzzy_controller'),
        'config',
        'rx150_fuzzy_gains.yaml')

    fuzzy_node = Node(
        package='rx150_fuzzy_controller',
        executable='fuzzy_node',
        name='fuzzy_node',
        namespace='rx150',
        output='screen',
        parameters=[
            fuzzy_params,
            {'robot_description': robot_description}
        ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'load_configs',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'Write motor register config to EEPROM at startup. Enable only after '
                'changing the motor config or replacing a motor.'
            ),
        ),
        xsarm,
        fuzzy_node,
    ] + declare_interbotix_xsarm_robot_description_launch_arguments(
        show_gripper_bar='true',
        show_gripper_fingers='true',
        hardware_type='actual',
    ))
