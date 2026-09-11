#!/usr/bin/env python3

# Demo pick-and-place cho RX150 sử dụng pipeline PCL chuẩn Interbotix.
#
# Yêu cầu đã chạy:
#   T1: ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py \
#           use_camera:=true use_camera_static_tf:=false
#   T2: ros2 launch rx150_perception rx150_perception.launch.py
#
# Chạy demo:
#   T3: cd ~/interbotix_ws/src/rx150/rx150_toolbox/rx150_perception/demos && python3 pick_place.py

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

ROBOT_MODEL = 'rx150'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'


def main():
    # Tạo global node backend cho tất cả các API module
    global_node = create_interbotix_global_node()

    # Khởi tạo arm module + pointcloud + armtag interface
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        node=global_node,
    )
    pcl = InterbotixPointCloudInterface(
        node_inf=global_node,
    )
    armtag = InterbotixArmTagInterface(
        ref_frame=REF_FRAME,
        arm_tag_frame=ARM_TAG_FRAME,
        arm_base_frame=ARM_BASE_FRAME,
        node_inf=global_node,
    )

    # Khởi động API
    robot_startup(global_node)

    # Đặt tay về pose ngủ và mở gripper
    bot.arm.go_to_sleep_pose()
    bot.gripper.release()

    # Tìm TF camera -> base_link bằng AprilTag
    # (Nếu đã có static_transforms.yaml, bước này kiểm tra lại TF)
    armtag.find_ref_to_arm_base_transform()
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)

    # Lấy vị trí các cluster vật thể
    # Sắp xếp theo trục x (xa nhất trước) trong hệ tọa độ base_link
    success, clusters = pcl.get_cluster_positions(
        ref_frame=ARM_BASE_FRAME,
        sort_axis='x',
        reverse=True,
    )

    if success:
        print(f'Tìm thấy {len(clusters)} vật thể!')
        for i, cluster in enumerate(clusters):
            x, y, z = cluster['position']
            print(f'  Vật {i+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}')

            # Tiếp cận từ trên
            bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.05, pitch=0.5)
            # Hạ xuống gắp
            bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
            bot.gripper.grasp()
            # Nâng lên
            bot.arm.set_ee_pose_components(x=x, y=y, z=z + 0.05, pitch=0.5)
            # Di chuyển đến vị trí thả
            bot.arm.set_ee_pose_components(x=0.3, z=0.2)
            bot.gripper.release()
    else:
        print('Không tìm thấy cluster nào. Kiểm tra camera + filter params.')

    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()
    robot_shutdown(global_node)


if __name__ == '__main__':
    main()

