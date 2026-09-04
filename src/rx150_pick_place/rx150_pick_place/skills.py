#!/usr/bin/env python3
"""
skills — các "verb" pick-place dùng chung cho mọi task node.

pick_place_moveit_node và tube_rack_node (và trước đây cả hri_motion_node) từng
copy nguyên khối 200 dòng MoveGroup + gripper + scene, rồi mỗi bản lệch nhau một
ít ⇒ sửa lỗi ở 1 bản không lan sang bản kia. Ở đây chỉ còn 1 bản:

  plan_pose / plan_grasp   — IK có pitch ladder + seed liên tục, KHÔNG di chuyển
  move_to / move_linear_to — chấp hành (joint goal / đi thẳng Descartes)
  grasp_at / release_at    — kẹp + XÁC NHẬN có vật, attach/detach planning scene
  retract_up / go_home     — rút thẳng đứng trước khi đi ngang (không quét vật)
  recover                  — xử lý sự cố: còn kẹp vật thì đem về chỗ loại bỏ

Nguyên tắc quan trọng: LẬP KẾ HOẠCH TOÀN CHUỖI TRƯỚC KHI ĐỘNG (plan_* không gửi
goal). Bản cũ gắp xong mới phát hiện IK của chỗ thả fail → tay đứng giữa không
trung, tay còn kẹp vật, rồi "về home" kéo vật đi theo.
"""
import math

from .kinematics import ARM_JOINTS, wrist_rotate_for_axis
from .status import State

GRASPED_OBJECT_ID = 'grasped_object'


class Waypoint:
    """1 pose đã có nghiệm IK (joints) — plan trước, chạy sau."""

    __slots__ = ('joints', 'x', 'y', 'z', 'pitch', 'wrist', 'label')

    def __init__(self, joints, x, y, z, pitch, wrist, label=''):
        self.joints = list(joints)
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.pitch, self.wrist = float(pitch), float(wrist)
        self.label = label

    @property
    def pose(self):
        return (self.x, self.y, self.z, self.pitch)

    def __repr__(self):
        return (f'{self.label}({self.x:.3f},{self.y:.3f},{self.z:.3f},'
                f'pitch={math.degrees(self.pitch):.0f}°)')


class GraspPlan:
    __slots__ = ('pre', 'grasp', 'lift', 'yaw')

    def __init__(self, pre, grasp, lift, yaw):
        self.pre, self.grasp, self.lift, self.yaw = pre, grasp, lift, yaw


class PickPlaceSkill:
    def __init__(self, node, kin, executor, gripper, scene, status, cfg):
        self.node = node
        self.log = node.get_logger()
        self.kin = kin
        self.exec = executor
        self.gripper = gripper
        self.scene = scene
        self.status = status
        self.cfg = cfg
        self._holding = False

    # ── trạng thái ──────────────────────────────────────────────────────
    @property
    def holding(self):
        return self._holding

    def current_joints(self):
        joints = self.exec.current_joints()
        if joints is None:
            self.log.warn('Chưa có joint_states — dùng home làm seed IK.')
            return list(self.cfg.home_joints)
        return joints

    # ── an toàn hình học ────────────────────────────────────────────────
    def clamp_z(self, z, label):
        """Chặn dưới độ cao ee — tránh đâm mặt bàn khi detection lệch/nhiễu depth."""
        if z < self.cfg.min_ee_z:
            self.log.warn(f'{label}: z={z:.3f}m dưới min_ee_z={self.cfg.min_ee_z:.3f}m '
                          f'→ nâng lên min_ee_z (detection lệch depth?).')
            return self.cfg.min_ee_z
        return z

    # ── lập kế hoạch (KHÔNG di chuyển) ──────────────────────────────────
    def plan_pose(self, x, y, z, pitch_ladder, *, wrist=0.0, seed=None, label='POSE',
                  align_yaw=None, quiet=False):
        """IK cho 1 pose, thử lần lượt pitch trong ladder. None nếu không với tới.

        align_yaw != None ⇒ wrist_rotate tính lại theo pitch thực tế chọn được
        (điều kiện kẹp ⟂ trục vật phụ thuộc pitch — xem wrist_rotate_for_axis).
        """
        z = self.clamp_z(z, label)
        seed = seed if seed is not None else self.current_joints()
        waist = math.atan2(y, x)
        for pitch in pitch_ladder:
            pitch = float(pitch)
            roll = (wrist_rotate_for_axis(waist, align_yaw, pitch)
                    if align_yaw is not None else float(wrist))
            joints = self.kin.ik(x, y, z, pitch, seed=seed, wrist_rotate=roll)
            if joints is not None:
                return Waypoint(joints, x, y, z, pitch, roll, label)
        if not quiet:
            self.log.error(
                f'{label}: không với tới ({x:.3f},{y:.3f},{z:.3f}) với pitch '
                f'{[round(math.degrees(float(p))) for p in pitch_ladder]}° — '
                f'{self.kin.reach_report(x, y, z, float(pitch_ladder[0]))}')
        return None

    def plan_grasp(self, x, y, z, *, yaw=None, pitch_ladder=None, approach_delta=None,
                   grasp_z_offset=None, seed=None):
        """Kế hoạch 3 pose pre-grasp → grasp → lift. None nếu bất kỳ pose nào fail.

        CÙNG 1 pitch cho cả 3 pose (seed liên tục) ⇒ hạ xuống và nhấc lên là
        chuyển động thẳng, không đổi cấu hình khuỷu ngay trên đầu vật.
        """
        ladder = list(pitch_ladder if pitch_ladder is not None else self.cfg.grasp_pitch_ladder)
        delta = self.cfg.approach_delta if approach_delta is None else float(approach_delta)
        offset = (self.cfg.grasp_z_offset if grasp_z_offset is None else float(grasp_z_offset))
        seed = seed if seed is not None else self.current_joints()
        z_grasp = self.clamp_z(z - offset, 'GRASP')
        z_pre = max(z_grasp + delta, z_grasp + 0.01)

        for pitch in ladder:
            pitch = float(pitch)
            pre = self.plan_pose(x, y, z_pre, [pitch], seed=seed, label='PRE-GRASP',
                                 align_yaw=yaw, quiet=True)
            if pre is None:
                continue
            grasp = self.plan_pose(x, y, z_grasp, [pitch], seed=pre.joints, label='GRASP',
                                   align_yaw=yaw, quiet=True)
            if grasp is None:
                continue
            lift = self.plan_pose(x, y, z_pre, [pitch], seed=grasp.joints, label='LIFT',
                                  align_yaw=yaw, quiet=True)
            if lift is None:
                continue
            if pitch != float(ladder[0]):
                self.log.warn(
                    f'Không gắp được ở pitch {math.degrees(float(ladder[0])):.0f}° '
                    f'(quá dốc so với tầm với ở r={math.hypot(x, y):.3f}m) — '
                    f'dùng {math.degrees(pitch):.0f}° trong ladder.')
            self.log.info(
                f'Kế hoạch gắp: pitch={math.degrees(pitch):.0f}° '
                f'wrist_rotate={math.degrees(grasp.wrist):.0f}° '
                f'z_pre={z_pre:.3f} z_grasp={z_grasp:.3f}')
            return GraspPlan(pre, grasp, lift, yaw)
        self.log.error(
            f'Không lập được kế hoạch gắp tại ({x:.3f},{y:.3f},{z:.3f}): '
            f'{self.kin.reach_report(x, y, z_grasp, float(ladder[0]))}')
        return None

    # ── chấp hành ───────────────────────────────────────────────────────
    def move_to(self, waypoint: Waypoint, velocity_scale=None, label=None):
        return self.exec.move_joints(
            waypoint.joints, velocity_scale or self.cfg.velocity_scale_cruise,
            label=label or waypoint.label or 'MOVE')

    def move_linear_to(self, waypoint: Waypoint, *, speed=None, label=None):
        return self.exec.move_linear_ee(
            waypoint.pose, self.cfg.velocity_scale_delicate,
            label=label or waypoint.label or 'LINEAR',
            speed_mps=speed if speed is not None else self.cfg.approach_speed_mps,
            wrist_rotate=waypoint.wrist)

    def go_home(self, velocity_scale=None):
        self.status.set(State.HOMING, log=False)
        return self.exec.move_joints(
            list(self.cfg.home_joints),
            velocity_scale or self.cfg.velocity_scale_cruise, label='HOME')

    def retract_up(self, height=None, *, speed=None, label='RETRACT'):
        """Rút THẲNG ĐỨNG lên trước khi đi ngang.

        Nhả vật xong mà đi ngang ngay là gạt đổ vật vừa đặt / cào vào giá — đây
        là lỗi hay gặp nhất khi chuỗi pick-place "gần đúng".
        """
        height = self.cfg.retract_height if height is None else float(height)
        joints = self.current_joints()
        x, y, z, pitch = self.kin.fk(joints)
        # Rút hết `height` thì tốt nhất, nhưng gần biên vùng làm việc có thể không
        # với tới — rút được BAO NHIÊU vẫn hơn là không rút gì rồi đi ngang.
        for scale in (1.0, 0.6, 0.3):
            dz = height * scale
            if dz < 0.01:
                break
            target = self.plan_pose(x, y, z + dz, [pitch], wrist=joints[4],
                                    seed=joints, label=label, quiet=True)
            if target is None:
                continue
            if scale < 1.0:
                self.log.warn(f'{label}: chỉ rút được {dz * 100:.0f}cm '
                              f'(yêu cầu {height * 100:.0f}cm) — sát biên vùng làm việc.')
            return self.move_linear_to(target,
                                       speed=speed or self.cfg.retract_speed_mps,
                                       label=label)
        self.log.error(f'{label}: không rút thẳng lên được ở '
                       f'({x:.3f},{y:.3f},{z:.3f}) — giữ nguyên vị trí.')
        return False

    # ── kẹp / nhả (có xác nhận + planning scene) ─────────────────────────
    def open_gripper(self):
        return self.gripper.open()

    def grasp_at(self, plan: GraspPlan, *, attach=True):
        """PRE-GRASP → hạ thẳng → kẹp + XÁC NHẬN → attach → nhấc thẳng."""
        self.status.set(State.APPROACH, f'{plan.pre}')
        if not self.gripper.open():
            self.log.warn('Không mở được gripper trước khi gắp — vẫn thử tiếp.')
        if not self.move_to(plan.pre, label='APPROACH'):
            return False

        self.status.set(State.DESCEND, f'{plan.grasp}')
        if not self.move_linear_to(plan.grasp, speed=self.cfg.descend_speed_mps,
                                   label='DESCEND'):
            return False

        self.status.set(State.GRASP)
        for attempt in range(int(self.cfg.grasp_retries) + 1):
            if self.gripper.close():
                break
            if attempt >= int(self.cfg.grasp_retries):
                self.log.error('GRASP thất bại sau khi thử lại — không có vật trong ngón.')
                return False
            # thử lại: mở ra, hạ thêm 1 nhịp nhỏ rồi kẹp lại (regrasp)
            step = float(self.cfg.regrasp_z_step)
            self.log.warn(f'Thử kẹp lại lần {attempt + 2}: hạ thêm {step * 1000:.0f}mm.')
            self.gripper.open()
            lower = self.plan_pose(plan.grasp.x, plan.grasp.y, plan.grasp.z - step,
                                   [plan.grasp.pitch], wrist=plan.grasp.wrist,
                                   seed=plan.grasp.joints, label='REGRASP')
            if lower is None or not self.move_linear_to(
                    lower, speed=self.cfg.descend_speed_mps, label='REGRASP'):
                return False
            plan.grasp = lower

        self._holding = True
        if attach:
            self.scene.attach_object(GRASPED_OBJECT_ID, self.cfg.object_length,
                                     self.cfg.object_radius)

        self.status.set(State.LIFT, f'{plan.lift}')
        if not self.move_linear_to(plan.lift, speed=self.cfg.retract_speed_mps,
                                   label='LIFT'):
            return False
        if self.gripper.holding() is False:
            self.log.error('Vật TUỘT khi nhấc lên (ngón đã đóng hết).')
            self._holding = False
            self.scene.detach_object(GRASPED_OBJECT_ID)
            return False
        return True

    def release_at(self, insert: Waypoint, *, hover: Waypoint = None,
                   speed=None, retract=True, state=None):
        """(hover →) hạ thẳng vào chỗ đặt → nhả → detach → rút thẳng lên."""
        if hover is not None:
            self.status.set(State.TRANSPORT, f'{hover}')
            if not self.move_to(hover, label='TRANSPORT'):
                return False
        self.status.set(state or State.PLACE, f'{insert}')
        # Vật đang kẹp PHẢI được phép chạm giá/bàn ở đúng pha này, nếu không
        # planner từ chối mọi quỹ đạo đưa ống vào trong lỗ.
        self.scene.set_collision_allowed(GRASPED_OBJECT_ID, True)
        try:
            if not self.move_linear_to(insert, speed=speed or self.cfg.insert_speed_mps,
                                       label='PLACE'):
                return False
            self.status.set(State.RELEASE)
            released = self.gripper.open()
        finally:
            self.scene.set_collision_allowed(GRASPED_OBJECT_ID, False)
        self._holding = False
        self.scene.detach_object(GRASPED_OBJECT_ID)
        if not released:
            self.log.warn('Nhả vật không chắc chắn — kiểm tra ngón kẹp.')
        if retract:
            self.status.set(State.RETRACT, log=False)
            if not self.retract_up():
                self.log.warn('Không rút thẳng lên được sau khi nhả — về home cẩn thận.')
        return released

    # ── xử lý sự cố ─────────────────────────────────────────────────────
    def recover(self, reason=''):
        """Đưa robot về trạng thái an toàn. Còn kẹp vật thì KHÔNG thả bừa tại chỗ."""
        self.status.set(State.RECOVERY, reason)
        if self.exec.aborted:
            # E-stop: KHÔNG tự di chuyển, và cũng không nhả vật (thả rơi vật ở vị
            # trí bất kỳ còn tệ hơn giữ nó). Người vận hành dùng ~/open_gripper.
            self.log.warn('E-stop đang bật — không tự di chuyển, không tự nhả vật. '
                          'Dùng service ~/open_gripper nếu cần lấy vật ra.')
            return False
        holding = self._holding or (self.gripper.holding() is True)
        self.retract_up(self.cfg.retract_height, label='RECOVERY/RETRACT')
        if holding:
            reject = self.plan_pose(self.cfg.reject_x, self.cfg.reject_y,
                                    self.cfg.reject_z, self.cfg.place_pitch_ladder,
                                    label='RECOVERY/REJECT')
            if reject is not None and self.move_to(reject, label='RECOVERY/REJECT'):
                self.gripper.open()
                self._holding = False
                self.scene.detach_object(GRASPED_OBJECT_ID)
                self.retract_up(label='RECOVERY/RETRACT2')
            else:
                self.log.error('Không tới được vị trí loại bỏ — GIỮ vật và về home. '
                               'Lấy vật ra bằng tay rồi gọi lại service ~/open_gripper.')
        else:
            self.gripper.open()
            self.scene.detach_object(GRASPED_OBJECT_ID)
        return self.go_home()

    # ── tiện ích: mô tả kế hoạch để log/HMI ─────────────────────────────
    @staticmethod
    def describe(*waypoints):
        return ' → '.join(str(w) for w in waypoints if w is not None)


def joint_deg(joints):
    return [round(math.degrees(v), 1) for v in joints]


__all__ = ['PickPlaceSkill', 'GraspPlan', 'Waypoint', 'GRASPED_OBJECT_ID',
           'ARM_JOINTS', 'joint_deg']
