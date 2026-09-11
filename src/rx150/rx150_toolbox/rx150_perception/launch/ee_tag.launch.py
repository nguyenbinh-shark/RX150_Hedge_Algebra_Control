# Theo dõi LIÊN TỤC AprilTag dán trên tay gắp -> /ee_tag/tag_detections (30 fps).
#
# Dùng cho rx150_ee_tag_bench.py: so quỹ đạo camera đo được với quỹ đạo robot
# tự báo (encoder) và với lệnh gửi cho controller.
#
# Chạy (T3, sau khi T1 + T2 đã lên):
#   T1  ./rx150.sh t1-hac    robot + MoveIt + camera (driver RealSense ở đây)
#   T2  ./rx150.sh t2        TF hiệu chuẩn world <-> camera (static_transforms.yaml)
#   T3  ./rx150.sh eetag     launch này
#
# KHÔNG bật driver camera ở đây: T1 đã giữ thiết bị, hai driver tranh nhau D435i.
# Node này chỉ đọc ảnh color đã có trên topic.
#
# MUTEX TF không đổi: launch này chỉ thêm frame lá 'ee_tag' dưới
# camera_color_optical_frame, không đụng cạnh world <-> camera.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('rx150_perception'), 'config', 'ee_tag.yaml',
    ])

    detector = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        # name='ee_tag' => namespace riêng của node là /ee_tag, nên topic private
        # ~/tag_detections tự thành /ee_tag/tag_detections (mặc định của bench).
        name='ee_tag',
        output='screen',
        parameters=[config],
        # PHẢI remap '~/image_rect', KHÔNG phải 'image_rect': ContinuousDetector
        # subscribe topic PRIVATE (it_.subscribeCamera("~/image_rect", ...)), nên
        # remap tên trần không khớp gì cả — node chạy, không báo lỗi, và không
        # bao giờ nhận được một tấm ảnh nào.
        remappings=[
            ('~/image_rect', LaunchConfiguration('image_topic')),
            ('~/camera_info', LaunchConfiguration('camera_info_topic')),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/camera/color/image_raw',
            description='ảnh color D435i. Ảnh raw (chưa rectify) — D435i color '
                        'méo rất nhỏ; đây cũng là topic mà chuỗi hand-eye dùng.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/color/camera_info',
            description='camera_info khớp với image_topic.'),
        detector,
    ])
