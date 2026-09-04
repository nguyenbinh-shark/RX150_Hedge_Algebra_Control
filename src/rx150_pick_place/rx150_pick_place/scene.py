#!/usr/bin/env python3
"""
scene — planning scene: vật cản tĩnh + GẮN/THÁO vật đang kẹp.

Hai lỗi của bản cũ được sửa ở đây:
  1. `box.dim = [...]` — shape_msgs/SolidPrimitive KHÔNG có field `dim`
     (đúng là `dimensions`, và message ROS 2 dùng __slots__ ⇒ gán sai tên là
     AttributeError). Hậu quả thật: `_ensure_scene()` throw ngay dòng đầu, nên
     với add_table_collision:=true thì tube_rack_node chết đúng lúc mở chu kỳ
     (chưa hề gửi 1 goal nào) và hri_motion_node chết luôn worker thread.
  2. Bản cũ không bao giờ ATTACH vật vào gripper. Trong khi mang vật, planner
     coi tay là tay không ⇒ quét vật đang kẹp qua giá/bàn. Chuẩn công nghiệp:
     attach lúc kẹp, detach lúc nhả — planner tự tính vật là phần của robot.
"""
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

# Các link được phép "chạm" vật đã attach (không tính là collision).
DEFAULT_TOUCH_LINKS = [
    'left_finger_link', 'right_finger_link', 'gripper_link',
    'gripper_bar_link', 'gripper_prop_link', 'ee_gripper_link',
]


def _box(size_xyz):
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(v) for v in size_xyz]
    return prim


def _cylinder(height, radius):
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.CYLINDER
    prim.dimensions = [float(height), float(radius)]
    return prim


def _pose(xyz, quat=(0.0, 0.0, 0.0, 1.0)):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = (float(v) for v in quat)
    return pose


class SceneManager:
    def __init__(self, node, *, base_frame='rx150/base_link', robot_name='rx150',
                 ee_link='ee_gripper_link', callback_group=None, dry_run=False,
                 service_timeout=3.0):
        self._node = node
        self._log = node.get_logger()
        self.base_frame = base_frame
        self.ee_link = f'{robot_name}/{ee_link}'
        self.touch_links = [f'{robot_name}/{link}' for link in DEFAULT_TOUCH_LINKS]
        self.dry_run = bool(dry_run)
        self.service_timeout = float(service_timeout)
        self._client = node.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                          callback_group=callback_group)
        self._get_scene = node.create_client(GetPlanningScene, '/get_planning_scene',
                                             callback_group=callback_group)
        self._attached = set()
        self._world = set()

    # ── vật cản tĩnh ────────────────────────────────────────────────────
    def add_box(self, obj_id, xyz, size_xyz, frame=None):
        obj = CollisionObject()
        obj.header.frame_id = frame or self.base_frame
        obj.id = str(obj_id)
        obj.operation = CollisionObject.ADD
        obj.primitives = [_box(size_xyz)]
        obj.primitive_poses = [_pose(xyz)]
        return obj

    def add_cylinder(self, obj_id, xyz, height, radius, frame=None, quat=None):
        obj = CollisionObject()
        obj.header.frame_id = frame or self.base_frame
        obj.id = str(obj_id)
        obj.operation = CollisionObject.ADD
        obj.primitives = [_cylinder(height, radius)]
        obj.primitive_poses = [_pose(xyz, quat or (0.0, 0.0, 0.0, 1.0))]
        return obj

    def remove(self, obj_id, frame=None):
        obj = CollisionObject()
        obj.header.frame_id = frame or self.base_frame
        obj.id = str(obj_id)
        obj.operation = CollisionObject.REMOVE
        return obj

    def apply(self, objects, attached=None, label='scene'):
        """Gửi 1 diff planning scene. Trả True nếu service nhận."""
        if self.dry_run:
            self._log.info(f'[DRY-RUN] planning scene {label}')
            return True
        if not self._client.wait_for_service(timeout_sec=self.service_timeout):
            self._log.warn('/apply_planning_scene chưa sẵn sàng — bỏ qua cập nhật '
                           f'planning scene ({label}). Planner sẽ KHÔNG biết vật cản này.')
            return False
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = list(objects)
        if attached:
            scene.robot_state.attached_collision_objects = list(attached)
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._client.call_async(req)
        # node spin ở thread khác ⇒ poll ở đây (giống MoveItExecutor._wait)
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < self.service_timeout + 2.0:
            time.sleep(0.01)
        if not future.done():
            self._log.warn(f'/apply_planning_scene không trả lời ({label}).')
            return False
        result = future.result()
        if result is None or not getattr(result, 'success', False):
            self._log.warn(f'/apply_planning_scene từ chối diff ({label}).')
            return False
        for obj in objects:
            if obj.operation == CollisionObject.REMOVE:
                self._world.discard(obj.id)
            else:
                self._world.add(obj.id)
        detail = ', '.join(
            f'{o.id}:{"REMOVE" if o.operation == CollisionObject.REMOVE else "ADD"}'
            for o in objects)
        self._log.info(f'Planning scene {label}' + (f' [{detail}]' if detail else ''))
        return True

    # ── vật đang kẹp ────────────────────────────────────────────────────
    def attach_object(self, obj_id, height, radius, *, xyz=(0.0, 0.0, 0.0),
                      quat=(0.0, 0.0, 0.0, 1.0)):
        """Gắn 1 cylinder vào ee_link (toạ độ trong frame ee_link).

        Ống nghiệm kẹp ngang giữa 2 ngón nằm dọc trục ee_z ⇒ pose mặc định
        (identity) đã đúng: cylinder của shape_msgs dựng theo trục z.
        """
        obj_id = str(obj_id)
        aco = AttachedCollisionObject()
        aco.link_name = self.ee_link
        aco.touch_links = list(self.touch_links)
        aco.object = CollisionObject()
        aco.object.header.frame_id = self.ee_link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD
        aco.object.primitives = [_cylinder(height, radius)]
        aco.object.primitive_poses = [_pose(xyz, quat)]
        # MoveIt yêu cầu: attach = ADD vào robot_state + REMOVE khỏi world
        removals = [self.remove(obj_id)] if obj_id in self._world else []
        ok = self.apply(removals, attached=[aco], label=f'attach {obj_id}')
        if ok or self.dry_run:
            self._attached.add(obj_id)
        return ok

    def detach_object(self, obj_id, *, keep_in_world=False, world_pose=None,
                      height=None, radius=None):
        """Tháo vật khỏi gripper. keep_in_world=True → để lại làm vật cản tĩnh."""
        obj_id = str(obj_id)
        if obj_id not in self._attached and not self.dry_run:
            return True
        aco = AttachedCollisionObject()
        aco.link_name = self.ee_link
        aco.object = CollisionObject()
        aco.object.header.frame_id = self.ee_link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.REMOVE
        objects = []
        if keep_in_world and world_pose is not None and height and radius:
            objects.append(self.add_cylinder(obj_id, world_pose, height, radius))
        else:
            objects.append(self.remove(obj_id))
        ok = self.apply(objects, attached=[aco], label=f'detach {obj_id}')
        self._attached.discard(obj_id)
        return ok

    @property
    def attached_ids(self):
        return set(self._attached)

    # ── cho phép vật đang kẹp chạm vật cản (lúc cắm/đặt) ─────────────────
    def set_collision_allowed(self, obj_id, allowed):
        """Bật/tắt "vật này được phép chạm mọi thứ" trong ACM của planning scene.

        Cần cho pha CẮM/ĐẶT: ống đang kẹp phải chui vào trong lỗ của giá — mà
        giá là vật cản trong scene, nên planner sẽ từ chối mọi quỹ đạo đưa ống
        xuống. Chuẩn công nghiệp là nới ACM đúng cặp đó trong đúng pha, rồi trả
        lại ngay.

        LƯU Ý: gửi allowed_collision_matrix khác rỗng trong 1 diff sẽ THAY THẾ
        toàn bộ ACM (planning_scene.cpp:1185) — kể cả các disable_collisions của
        SRDF. Vì vậy phải ĐỌC ACM hiện tại rồi mới sửa, không được gửi ma trận
        tự chế.
        """
        obj_id = str(obj_id)
        if self.dry_run:
            self._log.info(f'[DRY-RUN] ACM: {obj_id} allowed={allowed}')
            return True
        acm = self._read_acm()
        if acm is None:
            self._log.warn('Không đọc được ACM — bỏ qua nới va chạm cho '
                           f'{obj_id} (pha đặt vật có thể bị planner từ chối).')
            return False
        names = list(acm.default_entry_names)
        values = list(acm.default_entry_values)
        if obj_id in names:
            index = names.index(obj_id)
            if allowed:
                values[index] = True
            else:
                names.pop(index)
                values.pop(index)
        elif allowed:
            names.append(obj_id)
            values.append(True)
        else:
            return True
        acm.default_entry_names = names
        acm.default_entry_values = values
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.allowed_collision_matrix = acm
        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self._client.call_async(req)
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < self.service_timeout + 2.0:
            time.sleep(0.01)
        ok = future.done() and future.result() is not None
        self._log.info(f'ACM: {obj_id} {"được phép" if allowed else "không được phép"} '
                       f'chạm vật cản ({"ok" if ok else "thất bại"}).')
        return ok

    def _read_acm(self):
        if not self._get_scene.wait_for_service(timeout_sec=self.service_timeout):
            return None
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self._get_scene.call_async(req)
        t0 = time.monotonic()
        while not future.done() and time.monotonic() - t0 < self.service_timeout + 2.0:
            time.sleep(0.01)
        if not future.done() or future.result() is None:
            return None
        return future.result().scene.allowed_collision_matrix
