# rx150_perception.launch.py — Perception cho RX150 + RealSense D435i (armtag calib + TF).
#
# Kiến trúc theo chuẩn Interbotix (interbotix_xsarm_perception).
# KHÔNG khởi động robot. Camera thì TUỲ CHỌN qua use_camera (mặc định false).
#
# Luồng chuẩn — camera do T1 cấp:
#   T1: ros2 launch rx150_hac_controller hac_moveit.launch.py use_camera_static_tf:=false
#       (hoặc rx150_fuzzy_controller fuzzy_moveit.launch.py ...)
#   T2: ros2 launch rx150_perception rx150_perception.launch.py
#
# Chạy ĐỘC LẬP (không có T1, hoặc T1 use_camera:=false) — camera + RViz đi cùng:
#   ros2 launch rx150_perception rx150_perception.launch.py \
#       use_camera:=true use_rviz:=true
#   LƯU Ý: use_camera:=true khi T1 cũng đang mở camera -> lỗi "Device or resource busy".
#
# VỊ TRÍ TAG TRÊN TAY GẮP: launch này phát frame `rx150/ee_tag_link` từ bản ĐO
# `config/ee_tag_offset.yaml` và trỏ armtag vào đó (arm_tag_frame:=auto). Mặc định
# cũ `rx150/ar_tag_link` là vị trí gá AR tag CHÍNH HÃNG khai cứng trong URDF của
# Interbotix — gá tự thiết kế thì sai, và sai bao nhiêu thì Snap Pose sai bấy nhiêu
# (phần xoay đi đúng 1:1). Đo lại vị trí tag: `./rx150.sh eetag-tagcal`.
#
# Snap Pose hiệu chuẩn ArmTag (cần robot ở T1 để có TF tay gắp):
#   ros2 launch rx150_perception rx150_perception.launch.py \
#       use_armtag_tuner_gui:=true use_rviz:=true
#
# Tuner PCL (chỉ khi cần nhánh point cloud; nhớ bật pointcloud ở nơi chạy camera):
#   ... use_pointcloud_tuner_gui:=true pointcloud_enable:=true use_camera:=true

import os

import yaml

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
    use_pcl_pipeline_launch_arg = LaunchConfiguration('use_pcl_pipeline')
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
    ee_tag_offset_file_launch_arg = LaunchConfiguration('ee_tag_offset_file')
    ee_tag_frame_launch_arg = LaunchConfiguration('ee_tag_frame')
    transform_filepath_launch_arg = LaunchConfiguration('transform_filepath')

    use_rviz_launch_arg = LaunchConfiguration('use_rviz')
    rviz_frame_launch_arg = LaunchConfiguration('rviz_frame')
    rvizconfig_launch_arg = LaunchConfiguration('rvizconfig')

    # ---- 0. RealSense camera driver (tuỳ chọn) ----
    # Mặc định TẮT vì luồng chuẩn là T1 (hac/fuzzy_moveit use_camera:=true) đã bật
    # camera rồi — 2 driver cùng mở 1 thiết bị sẽ lỗi "Device or resource busy".
    # Bật use_camera:=true khi chạy perception ĐỘC LẬP (calib, xem thử) không có T1
    # hoặc T1 chạy với use_camera:=false.
    rs_camera_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('use_camera')),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': 'camera',
            # realsense2_camera 4.58.3: tên param là 'rgb_camera.color_profile' /
            # 'depth_module.depth_profile' (tên cũ bị drop silently).
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'align_depth.enable': 'true',
            'pointcloud.enable': LaunchConfiguration('pointcloud_enable'),
        }.items(),
    )

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
        condition=IfCondition(use_pcl_pipeline_launch_arg),
    )

    # ---- 1b. Frame TAG TAY GẮP dựng từ bản ĐO (ee_tag_offset.yaml) ----
    #
    # Snap Pose tính  T_camera←base = T_camera←tag (đo bằng ảnh) ∘ T_tag←base (tra TF).
    # Vế thứ hai lấy ở đâu thì sai số chỗ đó chui NGUYÊN vào hiệu chuẩn camera —
    # riêng phần xoay đi đúng 1:1 bất kể lúc snap tay máy đứng ở tư thế nào
    # (số hiệu chỉnh là một phép liên hợp, mà liên hợp không đổi độ lớn góc).
    #
    # Mặc định của Interbotix là `rx150/ar_tag_link`, tức vị trí gá AR tag CHÍNH
    # HÃNG khai cứng trong ar_tag.urdf.xacro. Gá tự thiết kế thì con số đó không
    # còn đúng: đo 2026-09-11 lệch 4.82 mm và 2.09° so với bản đo từ dữ liệu.
    #
    # Nên nguồn sự thật là ee_tag_offset.yaml (đo bằng bài AX=ZB, mode tagoffset),
    # và nó được phát thành MỘT frame TF để cả armtag lẫn RViz cùng dùng. Không
    # đụng gì tới src/vendor/ nên kéo lại vendor không mất.
    #
    # Tên frame KHÔNG được trùng 'ee_tag' — detector trong ee_tag.launch.py đã phát
    # camera_color_optical_frame -> ee_tag; trùng tên là frame có HAI cha.
    ee_tag_offset_file = ee_tag_offset_file_launch_arg.perform(context)
    ee_tag_frame = ee_tag_frame_launch_arg.perform(context)
    arm_tag_frame_value = arm_tag_frame_launch_arg.perform(context)
    ee_tag_tf_node = None

    if os.path.isfile(ee_tag_offset_file):
        with open(ee_tag_offset_file) as fh:
            offset = yaml.safe_load(fh) or {}
        missing = [k for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw') if k not in offset]
        if missing:
            raise RuntimeError(f'{ee_tag_offset_file} thiếu khoá {missing} — '
                               f'đo lại bằng "./rx150.sh eetag-tagcal".')
        parent = offset.get('parent_frame', 'rx150/ee_gripper_link')
        meas = offset.get('measurement') or {}
        print(f'[rx150_perception] frame tag tay gắp {parent} -> {ee_tag_frame} '
              f'từ {os.path.basename(ee_tag_offset_file)} '
              f'(đo {meas.get("calibrated_at", "?")}, sai số {meas.get("sigma_mm", "?")} mm)')
        ee_tag_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ee_tag_frame_pub',
            arguments=[
                '--frame-id', parent, '--child-frame-id', ee_tag_frame,
                '--x', str(offset['x']), '--y', str(offset['y']), '--z', str(offset['z']),
                '--qx', str(offset['qx']), '--qy', str(offset['qy']),
                '--qz', str(offset['qz']), '--qw', str(offset['qw']),
            ],
            output={'both': 'log'},
        )
        if arm_tag_frame_value == 'auto':
            arm_tag_frame_value = ee_tag_frame
    elif arm_tag_frame_value == 'auto':
        arm_tag_frame_value = 'rx150/ar_tag_link'
        print(f'[rx150_perception] ⚠ KHÔNG thấy {ee_tag_offset_file} — armtag quay về '
              f'{arm_tag_frame_value} (vị trí gá CHÍNH HÃNG khai trong URDF). '
              f'Gá tự in thì Snap Pose sẽ sai đúng bằng độ lệch của gá; '
              f'chạy "./rx150.sh eetag-tagcal" để đo.')

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
            'arm_tag_frame': arm_tag_frame_value,
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

    # ---- 3b. Theo dõi GIÁ ngay trong T2 (tuỳ chọn) ----
    # Cùng node mà './rx150.sh rack-gui' chạy, nhưng rviz:=false vì T2 đã có RViz
    # riêng — hai node cùng tên 'rviz2' trên một graph là hỏng. Marker và ảnh
    # chồng hình vào thẳng RViz của T2 qua /rack_calib/markers và
    # /rack_calib/image_debug (hai Display đã có sẵn trong rx150_perception.rviz).
    #
    # mode:=watch CHỈ ĐO, không bao giờ ghi đè rack_pose.yaml — muốn ghi thì vẫn
    # phải './rx150.sh rack-calib'. An toàn khi task đang chạy.
    #
    # ⚠️ Bật cái này thì ĐỪNG chạy thêm './rx150.sh rack-gui' hay 'rack-calib' ở
    # terminal khác: cả hai đều dựng node tên 'rack_tag' và 'rack_calib', trùng
    # tên trên cùng graph. Tắt bớt một bên (detector:=false) hoặc tắt arg này.
    rack_watch_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'launch',
                'rack_calib.launch.py',
            ])
        ]),
        launch_arguments={
            'mode': 'watch',
            'rviz': 'false',
            'detector': LaunchConfiguration('rack_watch_detector'),
            'image_topic': LaunchConfiguration('rack_watch_image_topic'),
            'camera_info_topic': LaunchConfiguration('rack_watch_camera_info_topic'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_rack_watch')),
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
        # Render trên GPU rời NVIDIA — `prime-select` là `on-demand`, thiếu 3 biến này thì
        # RViz rơi về iGPU Intel (GPU đang xuất hình, dùng chung băng thông RAM với CPU).
        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__VK_LAYER_NV_optimus': 'NVIDIA_only',
        },
        output={'both': 'log'},
        condition=IfCondition(use_rviz_launch_arg.perform(context)),
    )

    # ee_tag_tf_node = None khi chưa có bản đo -> lọc bỏ
    return [n for n in (
        ee_tag_tf_node,
        rs_camera_launch_include,
        pc_filter_launch_include,
        armtag_launch_include,
        static_transform_pub_launch_include,
        rack_watch_launch_include,
        rviz2_node,
    ) if n is not None]


def generate_launch_description():
    declared_arguments = []

    # ---- Camera ----
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'launch the RealSense driver from THIS file. Keep false when T1 '
                '(hac/fuzzy_moveit) already runs the camera — two drivers on one '
                'device fail with "resource busy".'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'pointcloud_enable',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'enable the XYZRGB pointcloud stream (~295 MB/s) when use_camera:=true; '
                'only needed for the PCL pipeline / pointcloud tuner GUI.'
            ),
        )
    )

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
            'use_pcl_pipeline',
            default_value='true',
            choices=('true', 'false'),
            description=(
                'chạy pc_filter (pointcloud_pipeline C++). Đặt false nếu chỉ dùng YOLO — '
                'yolo_tube_detector tự deproject depth nên không đọc /pc_filter/*, để true chỉ '
                'tốn một process nằm không.'
            ),
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
            default_value='auto',
            description="frame trên tay máy coi là vị trí AprilTag. 'auto' = dùng "
                        "ee_tag_frame nếu có bản đo ee_tag_offset.yaml, không thì "
                        "quay về rx150/ar_tag_link (vị trí gá CHÍNH HÃNG khai cứng "
                        "trong URDF — sai nếu gá là hàng tự thiết kế).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ee_tag_offset_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rx150_perception'),
                'config',
                'ee_tag_offset.yaml',
            ]),
            description='bản ĐO của T(ee_gripper_link -> tag), sinh bởi '
                        '"./rx150.sh eetag-tagcal". Đây là NGUỒN SỰ THẬT duy nhất '
                        'cho vị trí tag; URDF không còn khai con số này nữa.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ee_tag_frame',
            default_value='rx150/ee_tag_link',
            description="tên frame TF dựng từ ee_tag_offset.yaml. ĐỪNG đặt là "
                        "'ee_tag': detector trong ee_tag.launch.py đã phát "
                        "camera_color_optical_frame -> ee_tag, trùng tên là frame "
                        "có HAI cha.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_armtag_tuner_gui',
            # MẶC ĐỊNH FALSE — trước đây là true và đó là một cái bẫy im lặng:
            # armtag_tuner_gui publish lên /static_transforms, static_trans_pub lưu
            # kết quả vào transform_filepath, mà install/ ở workspace này là SYMLINK
            # về src/ (colcon --symlink-install) ⇒ mỗi phiên 't2' thường ngày có thể
            # GHI ĐÈ static_transforms.yaml trong mã nguồn. Đo được 2026-09-09: bản
            # đầu phiên và bản sau khi khởi động lại T2 lệch nhau 9.75° và 30 mm, và
            # bản trong git HEAD lại khác cả hai — hiệu chuẩn 'trôi' chính là vì thế.
            # Phiên hiệu chuẩn ('./rx150.sh calib') vẫn truyền use_armtag_tuner_gui:=true.
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

    # ---- Theo dõi giá ----
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rack_watch',
            default_value='false',
            choices=('true', 'false'),
            description='Chạy rack_calib ở mode watch ngay trong T2: marker 4 lỗ + '
                        'ảnh chồng hình vào RViz của T2, và in độ lệch so với '
                        'rack_pose.yaml. KHÔNG ghi đè file. Bật cái này thì đừng '
                        'chạy thêm rack-gui/rack-calib ở terminal khác (trùng tên node).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rack_watch_detector',
            default_value='true',
            choices=('true', 'false'),
            description='false nếu detector tag giá (rack_tag) đã chạy ở nơi khác.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rack_watch_image_topic',
            default_value='/camera/camera/color/image_raw',
            description='ảnh cho detector tag giá + ảnh chồng hình.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rack_watch_camera_info_topic',
            default_value='/camera/camera/color/camera_info',
            description='camera_info khớp với rack_watch_image_topic.',
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

