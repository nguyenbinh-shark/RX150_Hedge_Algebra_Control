#!/usr/bin/env python3
"""
params — bộ tham số CHUNG cho mọi node pick-place + hàm dựng cả stack.

Trước đây mỗi node tự declare_parameter một danh sách hơi khác nhau (cùng ý
nghĩa nhưng khác tên/khác default) nên chỉnh máy xong ở task này sang task kia
lại phải chỉnh lại. Ở đây 1 chỗ khai báo, 2 node dùng chung; node chỉ khai thêm
tham số RIÊNG của nó (giá đỡ, gesture…).
"""
from types import SimpleNamespace

from .detection import DetectionSource
from .gripper import FINGER_CLOSED, FINGER_JOINT, FINGER_OPEN, Gripper
from .kinematics import ARM_JOINTS, Rx150Kinematics
from .motion import JointStateMonitor, MoveItExecutor
from .scene import SceneManager
from .skills import PickPlaceSkill
from .status import TaskStatus

COMMON_DEFAULTS = {
    # ---- hệ / frame ----
    'robot_name': 'rx150',
    'base_frame': 'rx150/base_link',
    'joint_states_topic': '/rx150/joint_states',
    'arm_group': 'interbotix_arm',
    'gripper_group': 'interbotix_gripper',
    'dry_run': False,

    # ---- tư thế ----
    'home_joints': [0.0, 0.0, 0.0, 0.0, 0.0],
    # Ladder pitch: rx150 KHÔNG chúc thẳng đứng được ở mọi nơi (hết tầm từ
    # r ≈ 0.29 m — chạy `rx150_reach_check.py` để xem bảng). Ladder cho phép
    # nghiêng dần thay vì fail thẳng.
    'grasp_pitch_ladder': [1.5708, 1.40, 1.25, 1.10, 0.95],
    'place_pitch_ladder': [0.0, 0.15, 0.30, -0.15],

    # ---- hình học gắp ----
    'approach_delta': 0.06,      # m — độ cao pre-grasp trên điểm kẹp
    'grasp_z_offset': 0.0,       # m — hạ thêm dưới tâm vật (>0 = xuống sâu hơn)
    'retract_height': 0.08,      # m — rút thẳng đứng sau khi nhả
    'min_ee_z': 0.015,           # m — chặn dưới tuyệt đối cho ee (chống đâm bàn)
    'object_length': 0.10,       # m — kích thước vật để attach vào planning scene
    'object_radius': 0.0085,     # m

    # ---- tốc độ ----
    'velocity_scale_cruise': 0.3,
    'velocity_scale_delicate': 0.1,
    'approach_speed_mps': 0.05,
    'descend_speed_mps': 0.025,
    'insert_speed_mps': 0.02,
    'retract_speed_mps': 0.04,
    'use_linear_moves': True,
    'linear_step_m': 0.006,

    # ---- MoveIt ----
    'planning_time_s': 5.0,
    'planning_attempts': 5,
    'execution_timeout_s': 30.0,
    'goal_tolerance_rad': 0.01,
    'verify_tolerance_rad': 0.06,
    'move_retries': 1,

    # ---- gripper ----
    'use_gripper_bridge': True,
    'finger_closed_m': FINGER_CLOSED,
    'finger_open_m': FINGER_OPEN,
    'grasp_empty_margin_m': 0.0025,
    'grasp_settle_s': 0.4,
    'grasp_retries': 1,
    'regrasp_z_step': 0.008,
    'gripper_pwm_grasp': 250.0,
    'gripper_pwm_release': -250.0,

    # ---- xử lý sự cố ----
    'reject_x': 0.20,
    'reject_y': -0.20,
    'reject_z': 0.10,

    # ---- detection ----
    'detection_topic': '/yolo/detected_tubes',
    'classes_topic': '/yolo/tube_classes',
    'detection_wait_s': 10.0,
    'detection_max_age_s': 1.5,
    'median_frames': 5,
    'median_collect_s': 1.0,
    'detection_assoc_radius_m': 0.03,
    'detection_min_hits_ratio': 0.6,
    'detection_yaw_offset_deg': 0.0,
    'detection_invert_yaw': False,

    # ---- planning scene: mặt bàn ----
    'add_table_collision': True,
    'table_x': 0.30,
    'table_y': 0.0,
    'table_z': -0.05,
    'table_size_x': 0.80,
    'table_size_y': 0.80,
    'table_size_z': 0.10,
    'add_detected_obstacles': True,   # các vật KHÔNG gắp → vật cản (đừng gạt đổ)
}


def declare_common(node, overrides=None):
    """declare_parameter cho toàn bộ tham số chung (giá trị YAML sẽ ghi đè)."""
    defaults = dict(COMMON_DEFAULTS)
    defaults.update(overrides or {})
    for name, value in defaults.items():
        if not node.has_parameter(name):
            node.declare_parameter(name, value)
    return defaults


def read_common(node, names=None):
    """Đọc tham số → SimpleNamespace (cfg.approach_delta thay vì get_parameter…)."""
    names = names or COMMON_DEFAULTS.keys()
    cfg = SimpleNamespace()
    for name in names:
        setattr(cfg, name, node.get_parameter(name).value)
    return cfg


def build_stack(node, cfg, *, callback_group=None, status_topic='status',
                publisher=None):
    """Dựng đủ bộ: kinematics → joint monitor → MoveIt → gripper → scene → skill.

    Trả SimpleNamespace để node chỉ việc dùng stack.skill / stack.detection.
    """
    kin = Rx150Kinematics()
    js = JointStateMonitor(node, topic=cfg.joint_states_topic,
                           callback_group=callback_group, joint_names=ARM_JOINTS + [FINGER_JOINT])
    executor = MoveItExecutor(
        node, kin, js,
        base_frame=cfg.base_frame, arm_group=cfg.arm_group,
        callback_group=callback_group,
        goal_tolerance_rad=cfg.goal_tolerance_rad,
        verify_tolerance_rad=cfg.verify_tolerance_rad,
        planning_time_s=cfg.planning_time_s,
        execution_timeout_s=cfg.execution_timeout_s,
        planning_attempts=cfg.planning_attempts,
        retries=cfg.move_retries,
        dry_run=cfg.dry_run,
        linear_step_m=cfg.linear_step_m,
        use_linear=cfg.use_linear_moves,
    )
    executor.set_virtual_joints(cfg.home_joints)
    gripper = Gripper(
        node, executor, js, publisher=publisher,
        gripper_group=cfg.gripper_group, use_bridge=cfg.use_gripper_bridge,
        finger_closed=cfg.finger_closed_m, finger_open=cfg.finger_open_m,
        empty_margin=cfg.grasp_empty_margin_m, settle_s=cfg.grasp_settle_s,
        velocity_scale=cfg.velocity_scale_delicate,
        pwm_grasp=cfg.gripper_pwm_grasp, pwm_release=cfg.gripper_pwm_release,
        dry_run=cfg.dry_run,
    )
    scene = SceneManager(node, base_frame=cfg.base_frame, robot_name=cfg.robot_name,
                         callback_group=callback_group, dry_run=cfg.dry_run)
    status = TaskStatus(node, status_topic)
    detection = DetectionSource(
        node, poses_topic=cfg.detection_topic, classes_topic=cfg.classes_topic,
        base_frame=cfg.base_frame, max_age_s=cfg.detection_max_age_s,
        yaw_offset_deg=cfg.detection_yaw_offset_deg,
        invert_yaw=cfg.detection_invert_yaw, callback_group=callback_group)
    skill = PickPlaceSkill(node, kin, executor, gripper, scene, status, cfg)
    return SimpleNamespace(kin=kin, js=js, executor=executor, gripper=gripper,
                           scene=scene, status=status, detection=detection,
                           skill=skill)


def table_object(scene, cfg):
    return scene.add_box('table',
                         (cfg.table_x, cfg.table_y, cfg.table_z),
                         (cfg.table_size_x, cfg.table_size_y, cfg.table_size_z))
