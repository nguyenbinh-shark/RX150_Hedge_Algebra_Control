# BSD 3-Clause License
# Copyright (c) hust
#
# Launch the Interbotix rx150 under xsarm_control and start the HAC controller.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from interbotix_xs_modules.xs_launch import (
    declare_interbotix_xsarm_robot_description_launch_arguments,
)


def _launch_hac_node(context, robot_description, gains_file, gravity_model_file,
                     enable_gravity_comp):
    # Resolve LaunchConfiguration -> string tại thời điểm launch (không thể
    # if/else trực tiếp trên substitution chưa resolve).
    gains_file_str = context.perform_substitution(gains_file)
    gravity_model_file_str = context.perform_substitution(gravity_model_file)

    hac_params = os.path.join(
        get_package_share_directory('rx150_hac_controller'),
        'config',
        gains_file_str)

    node_params = [hac_params]
    if gravity_model_file_str:
        # File do rx150_gravity_id.py (mode identify) sinh ra — nạp SAU
        # gains_file để gravity_model_source/fitted_gravity_coeffs của nó đè
        # lên giá trị mặc định "pinocchio" trong gains_file.
        gravity_model_path = os.path.join(
            get_package_share_directory('rx150_hac_controller'),
            'config',
            gravity_model_file_str)
        node_params.append(gravity_model_path)
    node_params.append({'robot_description': robot_description})

    # enable_gravity_comp KHÔNG nằm trong onParamChange của hac_node (chỉ a/b/c,
    # u_max, Gff, friction_*, fitted_gravity_coeffs là live) => `ros2 param set`
    # không ăn, phải override lúc launch. Bước B0 signcheck của rx150_gravity_id.py
    # bắt buộc chạy với false, nếu không nó tự hủy.
    egc_str = context.perform_substitution(enable_gravity_comp)
    if egc_str:
        node_params.append({'enable_gravity_comp': egc_str.lower() == 'true'})

    hac_node = Node(
        package='rx150_hac_controller',
        executable='hac_node',
        name='hac_node',
        namespace='rx150',
        output='screen',
        parameters=node_params)

    return [hac_node]


def generate_launch_description():
    load_configs = LaunchConfiguration('load_configs')
    mode_configs = LaunchConfiguration('mode_configs')
    robot_description = LaunchConfiguration('robot_description')
    gains_file = LaunchConfiguration('gains_file')
    gravity_model_file = LaunchConfiguration('gravity_model_file')
    enable_gravity_comp = LaunchConfiguration('enable_gravity_comp')

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
            'mode_configs': mode_configs,
            'load_configs': load_configs,
            'use_sim': 'false',
            'use_rviz': 'false',
        }.items())

    # Action 2 — HAC controller node (namespace 'rx150' -> relative topics become /rx150/...).
    hac_node_setup = OpaqueFunction(
        function=lambda context: _launch_hac_node(
            context, robot_description, gains_file, gravity_model_file,
            enable_gravity_comp))

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
        DeclareLaunchArgument(
            'mode_configs',
            default_value=os.path.join(
                get_package_share_directory('rx150_motion_common'),
                'config',
                'rx150_modes_pwm.yaml'),
            description=(
                'File mode config truyền xuống xsarm_control. Mặc định là '
                'rx150_modes_pwm.yaml (arm lên THẲNG pwm, torque_enable false) chứ KHÔNG '
                'phải modes.yaml của upstream: file upstream cho arm lên ở operating_mode '
                'POSITION + torque_enable true, nên nếu tay máy đã bị xê dịch trong lúc '
                'torque tắt thì register Goal_Position còn giữ giá trị CŨ và cấp torque ở '
                'position mode làm tay máy GIẬT về đó, trong khoảng thời gian trước khi '
                'hac_node kịp chuyển sang pwm (hac_node.cpp:232).'
            ),
        ),
        DeclareLaunchArgument(
            'gains_file',
            default_value='rx150_hac_gains.yaml',
            description=(
                'Tên file trong rx150_hac_controller/config dùng làm gains chính. Dùng '
                'rx150_hac_gains_safe.yaml khi chạy rx150_gravity_id.py / rx150_friction_id.py '
                '(u_max + max_velocities/max_accelerations thấp hơn).'
            ),
        ),
        DeclareLaunchArgument(
            'gravity_model_file',
            default_value='',
            description=(
                'Tên file rx150_gravity_model.yaml (do rx150_gravity_id.py mode identify sinh '
                'ra, trong rx150_hac_controller/config) để nạp gravity_model_source=fitted + '
                'fitted_gravity_coeffs. Để trống -> dùng Pinocchio mặc định.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_gravity_comp',
            default_value='',
            choices=('', 'true', 'false'),
            description=(
                'Đè enable_gravity_comp của gains_file. Để trống -> giữ giá trị trong '
                'gains_file. BẮT BUỘC false khi chạy rx150_gravity_id.py mode signcheck '
                '(param này không live-tunable, ros2 param set sẽ không ăn).'
            ),
        ),
        xsarm,
        hac_node_setup,
    ] + declare_interbotix_xsarm_robot_description_launch_arguments(
        show_gripper_bar='true',
        show_gripper_fingers='true',
        hardware_type='actual',
    ))
