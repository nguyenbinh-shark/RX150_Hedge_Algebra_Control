# fuzzy_moveit_perception.launch.py — chạy TOÀN BỘ stack trong MỘT terminal.
#
# Gộp đúng SOP 2 terminal của docs/PERCEPTION_GUIDE.md (mục 2.1 + 2.2):
#   T1: fuzzy_moveit.launch.py     use_camera:=true rs_camera_pointcloud_enable:=true
#                                  use_camera_static_tf:=false use_handeye_publisher:=false
#   T2: rx150_perception.launch.py use_armtag_tuner_gui:=... use_rviz:=true
#   T3: ros2 run rx150_perception yolo_tube_detector_node.py   (phần xử lý ảnh)
#
# NHÁNH PCL ĐÃ BỎ (2026-08-28). Xử lý ảnh giờ hoàn toàn là YOLO: yolo_tube_detector_node
# tự deproject depth bằng camera_info nên KHÔNG cần /camera/camera/depth/color/points lẫn
# pc_filter. Không còn node nào trong rx150_perception/rx150_pick_place đọc /pc_filter/*
# (trừ demos/pick_place.py, bản port upstream Interbotix). Vì thế use_pointcloud_tuner_gui
# mặc định false và pc_filter không được dựng (use_pcl_pipeline).
#
# Nhưng rs_camera_pointcloud_enable vẫn để TRUE: đám mây điểm không còn phục vụ thuật toán
# nào, chỉ để NHÌN trong RViz (display RawPointCloud) cho dễ căn cảnh/soi depth. Đây là
# chi phí thuần hiển thị ~295 MB/s — đặt false là mất hình, không mất chức năng.
#
# Demo gắp/phân loại (demos/sort_tubes_by_color.py, demos/pick_place.py) KHÔNG nằm trong
# launch này — nó điều khiển tay thật nên phải gọi tay ở terminal khác:
#   python3 ~/interbotix_ws/src/rx150_perception/demos/sort_tubes_by_color.py
#
# Ba ràng buộc đã được khoá cứng ở đây (đừng đổi trừ khi biết rõ):
#   1. Camera CHỈ do nhánh fuzzy_moveit mở; perception nhận use_camera:=false.
#      Hai driver cùng một thiết bị -> "Device or resource busy".
#   2. TF `camera_color_optical_frame -> world` CHỈ do static_trans_pub của perception
#      phát (đọc config/static_transforms.yaml). fuzzy_moveit vì thế tắt cả
#      use_camera_static_tf lẫn use_handeye_publisher — 2 publisher cùng frame làm nhảy TF.
#   3. CHỈ MỘT RViz chạy. Hai RViz trùng tên node /rviz2, lại cộng stream pointcloud
#      ~295 MB/s, là công thức đơ máy. Mặc định dùng RViz của MoveIt (có panel
#      MotionPlanning) nhưng nạp config gộp rx150_moveit_perception.rviz — đã có sẵn
#      YoloMarkers + YoloDebug, nên không mất gì.
#      Muốn RViz perception nhẹ (không MoveIt): use_moveit_rviz:=false use_rviz:=true.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            choices=('true', 'false'),
            description='dùng xs_sdk_sim thay driver thật (không cần cắm robot).',
        ),
        DeclareLaunchArgument(
            'rs_camera_pointcloud_enable',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'stream XYZRGB /camera/camera/depth/color/points (~295 MB/s @640x480x30). YOLO '
                'KHÔNG cần topic này (nó đọc aligned_depth_to_color/image_raw, do arg riêng '
                'rs_camera_align_depth_enable điều khiển) — bật mặc định CHỈ để xem đám mây '
                'điểm trong RViz cho dễ căn cảnh, và để use_octomap:=true dùng được. Thấy '
                'RViz/máy ì thì đặt false, YOLO vẫn chạy đủ.'
            ),
        ),
        DeclareLaunchArgument(
            'use_moveit_rviz',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'RViz của MoveIt — có panel MotionPlanning, và nạp config gộp nên hiển thị '
                'luôn phần perception. Bật cái này thì để use_rviz:=false.'
            ),
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'rviz',
                'rx150_moveit_perception.rviz',
            ]),
            description=(
                'config cho RViz của MoveIt. Mặc định là bản gộp MotionPlanning + '
                'YoloMarkers + YoloDebug + Image. Display PCL đã gỡ; RawPointCloud còn nhưng '
                'tắt sẵn — tick lên khi bật lại rs_camera_pointcloud_enable.'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'RViz thứ hai của perception (không có MoveIt). Chỉ bật khi đã đặt '
                'use_moveit_rviz:=false — đừng chạy hai RViz cùng lúc.'
            ),
        ),
        DeclareLaunchArgument(
            'use_pointcloud_tuner_gui',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'GUI tune filter PCL; cũng bật enable_pipeline (pipeline chạy liên tục). Tắt '
                'mặc định vì nhánh PCL đã bỏ — cần thì bật KÈM rs_camera_pointcloud_enable:=true, '
                'không thì pipeline không có cloud để lọc.'
            ),
        ),
        DeclareLaunchArgument(
            'use_armtag_tuner_gui',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'GUI Snap Pose để hiệu chuẩn TF camera bằng AprilTag. Chỉ là một cửa sổ Qt nhỏ, '
                'gần như không tốn gì, nên để bật sẵn cho tiện snap lại. Tắt đi KHÔNG mất calib: '
                'pipeline armtag + static_trans_pub luôn chạy và vẫn phát TF đã lưu trong '
                'config/static_transforms.yaml. TF đó là BẮT BUỘC — YOLO lookup '
                'camera_color_optical_frame -> rx150/base_link để ra pose gắp.'
            ),
        ),
        DeclareLaunchArgument(
            'use_octomap',
            default_value='false',
            choices=('true', 'false'),
            description='OctoMap tránh vật cản 3D trong MoveIt.',
        ),
        DeclareLaunchArgument(
            'use_yolo',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'chạy yolo_tube_detector_node.py — nguồn của /yolo/detected_tubes + '
                '/yolo/tube_classes mà demos/sort_tubes_by_color.py subscribe.'
            ),
        ),
        DeclareLaunchArgument(
            'yolo_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'config',
                'yolo_detector_params.yaml',
            ]),
            description=(
                'params cho YOLO detector (conf/imgsz/depth gate/EMA...). `ros2 run` trần '
                'KHÔNG nạp file này — chạy qua launch mới có.'
            ),
        ),
        DeclareLaunchArgument(
            'roi_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'config',
                'roi_box_params.yaml',
            ]),
            description=(
                'hộp giới hạn VÙNG NHẬN DIỆN (toạ độ trong rx150/base_link). Detection ngoài '
                'hộp bị bỏ, không vào /yolo/detected_tubes. Nạp SAU yolo_params_file nên các '
                'khoá roi_* ở đây thắng. Đặt \'none\' để ROI đến từ yolo_params_file/CLI.'
            ),
        ),
        DeclareLaunchArgument(
            'perception_start_delay',
            default_value='5.0',
            description=(
                'giây chờ trước khi bật perception, cho RealSense + robot_state_publisher lên '
                'trước. Tránh apriltag/pc_filter spam warning lúc khởi động và dàn tải CPU.'
            ),
        ),
    ]

    # ---- Nhánh 1: robot + MoveIt + camera (Terminal 1 cũ) ----
    # Mọi arg nhạy cảm đều truyền TƯỜNG MINH: launch config của scope cha rò xuống file
    # được include, nên không dựa vào default bên trong file con.
    fuzzy_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rx150_fuzzy_controller'),
                'launch',
                'fuzzy_moveit.launch.py',
            ])
        ]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'use_camera': 'true',
            'rs_camera_pointcloud_enable': LaunchConfiguration('rs_camera_pointcloud_enable'),
            'use_camera_static_tf': 'false',
            'use_handeye_publisher': 'false',
            'use_moveit_rviz': LaunchConfiguration('use_moveit_rviz'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'),
            'use_octomap': LaunchConfiguration('use_octomap'),
        }.items(),
    )

    # ---- Nhánh 2: perception (Terminal 2 cũ) ----
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'launch',
                'rx150_perception.launch.py',
            ])
        ]),
        launch_arguments={
            'use_camera': 'false',
            'pointcloud_enable': 'false',
            # Nhánh PCL bám theo cùng một công tắc: bật tuner GUI thì mới dựng pc_filter.
            'use_pcl_pipeline': LaunchConfiguration('use_pointcloud_tuner_gui'),
            'use_pointcloud_tuner_gui': LaunchConfiguration('use_pointcloud_tuner_gui'),
            'use_armtag_tuner_gui': LaunchConfiguration('use_armtag_tuner_gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    # ---- Nhánh 3: YOLO detector (phần xử lý ảnh) ----
    # Nguồn của /yolo/detected_tubes + /yolo/tube_classes + /yolo/markers + /yolo/image_debug.
    # Model mặc định: share/rx150_perception/models/best.pt (segmentation) — node tự dò.
    # Vùng nhận diện (ROI box) đến từ roi_params_file; hộp cyan vẽ trong /yolo/markers.
    yolo_detector = Node(
        package='rx150_perception',
        executable='yolo_tube_detector_node.py',
        name='yolo_tube_detector',
        output='screen',
        # dict đứng sau params-file nên roi_params_file không bị file kia ghi đè.
        parameters=[
            LaunchConfiguration('yolo_params_file'),
            {'roi_params_file': LaunchConfiguration('roi_params_file')},
        ],
        condition=IfCondition(LaunchConfiguration('use_yolo')),
    )

    # scoped=True: DeclareLaunchArgument bên trong file được include KHÔNG rò ngược lên
    # scope cha (mặc định của launch là rò). Nếu không chặn, `use_camera:=true` do
    # fuzzy_moveit khai báo sẽ thành giá trị mặc định của cả nhánh perception -> 2 driver
    # RealSense cùng mở một thiết bị.
    return LaunchDescription(declared_arguments + [
        GroupAction([fuzzy_moveit], scoped=True),
        TimerAction(
            period=LaunchConfiguration('perception_start_delay'),
            actions=[GroupAction([perception], scoped=True), yolo_detector],
        ),
    ])
