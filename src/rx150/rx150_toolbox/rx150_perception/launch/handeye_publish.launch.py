# Publish TF `world -> camera_link` từ kết quả hand-eye calibration đã lưu.
#
# Sau khi chạy handeye_calibrate.launch.py và bấm "Save calibration" trong rqt,
# kết quả nằm trong ~/.ros/easy_handeye2/<name>.yaml. Launch này spawn
# easy_handeye2/handeye_publisher (StaticTransformBroadcaster) để publish TF đó.
#
# Chạy (thay cho static_tf hardcode):
#   ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#        use_camera_static_tf:=false use_handeye_publisher:=true
#
# Lưu ý: 'name' phải khớp với tên đã dùng khi calibration (mặc định rx150_eob).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    name_arg = DeclareLaunchArgument('name', default_value='rx150_eob')

    handeye_publish = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('easy_handeye2'),
            'launch', 'publish.launch.py')),
        launch_arguments={
            'name': LaunchConfiguration('name'),
        }.items(),
    )

    return LaunchDescription([name_arg, handeye_publish])
