# HIỆU CHUẨN VỊ TRÍ GIÁ ĐỠ ỐNG NGHIỆM bằng AprilTag — phần riêng, cùng khuôn với
# hiệu chuẩn TF camera:
#
#   camera:  armtag GUI  → static_transforms.yaml → static_trans_pub  (./rx150.sh calib)
#   GIÁ:     rack_calib  → rack_pose.yaml         → tube_rack_node    (./rx150.sh rack-calib)
#
# Chạy (T3, sau khi T1 + T2 đã lên):
#   T1  ./rx150.sh t1        robot + MoveIt + camera (driver RealSense ở đây)
#   T2  ./rx150.sh t2        TF hiệu chuẩn world <-> camera (static_transforms.yaml)
#   T3  ./rx150.sh rack-calib
#
#   ros2 launch rx150_perception rack_calib.launch.py                 # snap + ghi file
#   ros2 launch rx150_perception rack_calib.launch.py mode:=watch     # theo dõi trôi
#   ros2 launch rx150_perception rack_calib.launch.py mode:=publish   # chỉ phát TF
#   ros2 launch rx150_perception rack_calib.launch.py mode:=watch rviz:=true
#                                                     # ↑ KIỂM BẰNG MẮT (= ./rx150.sh rack-gui)
#
# GUI gồm hai thứ, đều bật sẵn:
#   /rack_calib/markers      marker 3D: 4 lỗ (hình trụ) + viền giá + trục lỗ + gốc tag
#   /rack_calib/image_debug  chiếu NGƯỢC 4 lỗ lên chính ảnh camera — vòng tròn nằm
#                            trên lỗ thật là ĐÚNG, lệch sang bên là SAI. Đây là kiểm
#                            tra end-to-end duy nhất đi qua cả TF camera lẫn pose tag.
#   Lục = nghiệm đang đo. Cam = nghiệm đã lưu trong rack_pose.yaml.
#
# KHÔNG bật driver camera ở đây: T1 đã giữ thiết bị, hai driver tranh nhau D435i.
# Launch này chỉ đọc ảnh color đã có trên topic.
#
# MUTEX TF không đổi: launch này chỉ thêm frame LÁ ('rack_tag' dưới
# camera_color_optical_frame, 'tube_rack' dưới rx150/base_link), không đụng cạnh
# world <-> camera vốn chỉ được phép có MỘT nguồn (static_trans_pub).
#
# mode:=publish KHÔNG cần camera lẫn detector — nó chỉ đọc rack_pose.yaml.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    if mode not in ('snap', 'publish', 'watch'):
        raise RuntimeError(f"mode không hợp lệ: {mode!r} (snap | publish | watch)")
    want_detector = (LaunchConfiguration('detector').perform(context).lower() == 'true'
                     and mode != 'publish')

    calib = Node(
        package='rx150_perception',
        executable='rack_calib_node.py',
        name='rack_calib',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('rx150_perception'),
                                  'config', 'rack_calib.yaml']),
            # Ảnh chồng hình đọc CÙNG topic mà detector đang đọc — nếu không thì
            # vòng tròn được vẽ lên một khung hình khác khung đã đo, lệch mà không
            # biết vì sao.
            {'mode': mode,
             'image_topic': LaunchConfiguration('image_topic'),
             'camera_info_topic': LaunchConfiguration('camera_info_topic')},
        ],
    )
    nodes = [calib]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            # name='rack_calib_rviz': KHÔNG trùng 'rviz2' của rx150_perception.launch.py
            # đang chạy ở T2 — hai node cùng tên trên một graph là hỏng.
            name='rack_calib_rviz',
            arguments=[
                '-f', 'rx150/base_link',
                '-d', PathJoinSubstitution([FindPackageShare('rx150_perception'),
                                            'rviz', 'rack_calib.rviz']).perform(context),
            ],
            # Render trên GPU rời NVIDIA — `prime-select` là `on-demand`, thiếu 3 biến
            # này thì RViz rơi về iGPU Intel (đang xuất hình, dùng chung băng thông RAM).
            additional_env={
                '__NV_PRIME_RENDER_OFFLOAD': '1',
                '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
                '__VK_LAYER_NV_optimus': 'NVIDIA_only',
            },
            output={'both': 'log'},
        ))
    if not want_detector:
        return nodes

    detector = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        # name='rack_tag' => namespace riêng của node là /rack_tag, nên topic
        # private ~/tag_detections tự thành /rack_tag/tag_detections (đúng cái
        # detections_topic mặc định trong rack_calib.yaml).
        name='rack_tag',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('rx150_perception'),
                                          'config', 'rack_tag.yaml'])],
        # PHẢI remap '~/image_rect', KHÔNG phải 'image_rect': ContinuousDetector
        # subscribe topic PRIVATE (it_.subscribeCamera("~/image_rect", …)), nên
        # remap tên trần không khớp gì cả — node chạy, không báo lỗi, và không
        # bao giờ nhận được một tấm ảnh nào.
        remappings=[
            ('~/image_rect', LaunchConfiguration('image_topic')),
            ('~/camera_info', LaunchConfiguration('camera_info_topic')),
        ],
    )
    return [detector] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='snap',
            description='snap = đo rồi ghi rack_pose.yaml | watch = đo và in độ lệch '
                        'so với file | publish = chỉ phát TF từ file (không cần camera).'),
        DeclareLaunchArgument(
            'detector', default_value='true',
            description='Chạy apriltag_ros continuous detector cho tag giá. false nếu '
                        'detector đã chạy ở terminal khác. mode:=publish tự bỏ qua.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Mở RViz kèm marker 4 lỗ + ảnh chồng hình (/rack_calib/'
                        'image_debug). Đây là cách KIỂM BẰNG MẮT xem lỗ tính ra có '
                        'trùng lỗ thật không. ./rx150.sh rack-gui = watch + rviz.'),
        DeclareLaunchArgument(
            'image_topic', default_value='/camera/camera/color/image_raw',
            description='ảnh color D435i. Ảnh raw (chưa rectify) — D435i color méo '
                        'rất nhỏ; đây cũng là topic mà chuỗi hand-eye dùng.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera/color/camera_info',
            description='camera_info khớp với image_topic.'),
        OpaqueFunction(function=_nodes),
    ])
