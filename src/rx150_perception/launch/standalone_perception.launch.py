# Launch Perception ĐỘC LẬP (Không cần kết nối Robot) cho RealSense D435i + AprilTag tag36h11 ID 0 (30mm)
#
# Chạy nhận diện PointCloud + AprilTag + mở giao diện 3D RViz2 tự động load sẵn hình ảnh & pointcloud:
#   ros2 launch rx150_perception standalone_perception.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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

    default_filter_params = PathJoinSubstitution([
        FindPackageShare('rx150_perception'),
        'config', 'filter_params.yaml',
    ])

    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('rx150_perception'),
        'config', 'standalone_perception.rviz',
    ])

    # 1. Driver RealSense D435i Camera
    rs_camera_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch', 'rs_launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('use_camera')),
        launch_arguments={
            'camera_name': 'camera',
            # realsense2_camera 4.58.3: 'rgb_camera.color_profile'/'depth_module.depth_profile'
            # (tên cũ 'rgb_camera.profile' bị drop silently).
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'pointcloud.enable': LaunchConfiguration('pointcloud_enable'),
        }.items(),
    )

    # 2. AprilTag Standalone Detector (tag36h11 ID 0 30mm)
    apriltag_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('interbotix_perception_modules'),
            'launch', 'apriltag.launch.py',
        ])),
        condition=IfCondition(LaunchConfiguration('use_apriltag')),
        launch_arguments={
            'tags_config': tags_config,
            'camera_frame': LaunchConfiguration('camera_frame'),
            'camera_color_topic': LaunchConfiguration('camera_color_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        }.items(),
    )

    # 3. PointCloud Filter Pipeline (CropBox, SACPlane, Voxel, Clustering)
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

    # 4. RViz2 Visualizer 3D tự động nạp standalone_perception.rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        # Render trên GPU rời NVIDIA. `prime-select` đang là `on-demand`, thiếu 3 biến này
        # thì RViz rơi về iGPU Intel — GPU đang xuất hình và dùng chung băng thông RAM với
        # CPU, vẽ point cloud ở đó là treo cả compositor.
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__VK_LAYER_NV_optimus': 'NVIDIA_only',
        },
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='tự động khởi chạy driver RealSense D435i.'),
        DeclareLaunchArgument(
            'use_apriltag', default_value='true',
            description='bật AprilTag detector đơn lẻ.'),
        DeclareLaunchArgument(
            'pointcloud_enable', default_value='false',
            choices=('true', 'false'),
            description=(
                'bật stream point cloud XYZRGB (~295 MB/s ở 640x480x30). Tắt mặc định; '
                'chỉ bật khi thực sự cần nhánh PCL hoặc xem cloud trong RViz.'),),
        DeclareLaunchArgument(
            'use_pointcloud_tuner_gui', default_value='false',
            description='hiện cửa sổ GUI chỉnh tham số lọc PointCloud (thêm 1 process Qt).'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='hiện giao diện 3D RViz2.'),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz_config,
            description='file cấu hình rviz.'),
        DeclareLaunchArgument(
            'enable_pipeline', default_value='false',
            description=(
                'bật pipeline lọc pointcloud chạy LIÊN TỤC. Upstream cố tình để false "to '
                'save computer processing power" — chuỗi PCL chạy đơn luồng trên mọi frame. '
                'Nếu false, pipeline chỉ chạy khi gọi service get_cluster_positions.'),),
        DeclareLaunchArgument(
            'filter_ns', default_value='pc_filter',
            description='namespace cho pointcloud pipeline.'),
        DeclareLaunchArgument(
            'filter_params', default_value=default_filter_params,
            description='file chứa thông số lọc filter_params.yaml.'),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/camera/camera/depth/color/points',
            description='topic pointcloud 3D từ RealSense D435i.'),
        DeclareLaunchArgument(
            'camera_color_topic', default_value='/camera/camera/color/image_raw',
            description='topic image raw.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/color/camera_info',
            description='topic camera info.'),
        DeclareLaunchArgument(
            'camera_frame', default_value='camera_color_optical_frame',
            description='frame optical camera D435i.'),
        rs_camera_include,
        apriltag_include,
        pc_filter_include,
        rviz_node,
    ])
