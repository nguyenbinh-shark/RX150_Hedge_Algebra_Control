#!/usr/bin/env python3
# hri_common.py — helper dùng chung giữa hri_task_node và hri_motion_node.
#
# Giao diện lệnh giữa 2 node (topic, test thủ công được bằng ros2 topic pub):
#   /hri/cmd_pose     (geometry_msgs/PoseStamped) — xyz + pitch đóng gói trong
#                     quaternion quanh trục Y (5-DoF: pitch là DOF định hướng duy nhất)
#   /hri/cmd_gripper  (std_msgs/Bool)             — True = kẹp, False = nhả
#   /hri/cmd_home     (std_msgs/Empty)            — về home_joints
#   /hri/set_vel_scale(std_msgs/Float32)          — velocity scale MoveIt cho các
#                                                     lệnh cmd_pose SAU đó (0..1)
#   /hri/status       (std_msgs/String)           — "<KIND> #<seq>" mỗi lệnh hoàn tất
#
# Các hàm ở đây THUẦN (không cần ROS runtime) để unit test được riêng.


def pitch_to_quat(pitch):
    """pitch (rad) → (qx, qy, qz, qw): quay quanh trục +Y của frame base_link."""
    import math
    half = 0.5 * float(pitch)
    return (0.0, math.sin(half), 0.0, math.cos(half))


def quat_to_pitch(qx, qy, qz, qw):
    """Nghịch đảo pitch_to_quat: pitch = 2·atan2(qy, qw). Bỏ qua qx/qz (không dùng)."""
    import math
    return 2.0 * math.atan2(float(qy), float(qw))


def build_status(kind, seq):
    """'POSE_DONE', 3 → 'POSE_DONE #3'."""
    return f'{kind} #{int(seq)}'


def parse_status(text):
    """'POSE_DONE #3' → ('POSE_DONE', 3); 'IDLE #3' → ('IDLE', 3).
    Không đúng định dạng → (None, -1)."""
    try:
        kind, _, seq = str(text).partition('#')
        kind = kind.strip()
        if not kind:
            return None, -1
        return kind, int(seq.strip())
    except ValueError:
        return None, -1


# Các kind status hri_motion có thể phát (task dựa vào này để đồng bộ):
READY = 'READY'    # executor khởi động xong (scene setup) — seq = 0
IDLE = 'IDLE'      # heartbeat khi rảnh (không tăng seq)
POSE_DONE = 'POSE_DONE'
POSE_FAILED = 'POSE_FAILED'
GRIP_DONE = 'GRIP_DONE'
GRIP_FAILED = 'GRIP_FAILED'
HOME_DONE = 'HOME_DONE'
HOME_FAILED = 'HOME_FAILED'
REJECTED = 'REJECTED'
