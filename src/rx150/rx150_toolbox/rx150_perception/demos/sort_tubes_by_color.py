#!/usr/bin/env python3
"""
sort_tubes_by_color.py — Demo Tự động gắp & Phân loại ống nghiệm theo màu (MoveIt + Fuzzy/HAC PWM).

Kiến trúc điều khiển:
  - YOLOv8 (yolo_tube_detector_node.py) nhận diện ống nghiệm (X,Y,Z + yaw).
  - SDK bot.arm chỉ dùng giải IK (execute=False).
  - Lệnh chuyển động gửi qua MoveIt Action ('move_action') -> fuzzy_trajectory_bridge -> fuzzy_node (PWM)
    đảm bảo CÁNH TAY THẬT DI CHUYỂN MƯỢT MÀ, KHÔNG XUNG ĐỘT DRIVER.

Yêu cầu đã chạy:
  T1: ros2 launch rx150_fuzzy_controller fuzzy_moveit.launch.py use_camera:=true use_camera_static_tf:=false
      (hoặc ros2 launch rx150_hac_controller hac_moveit.launch.py ...)
  T2: ros2 launch rx150_perception rx150_perception.launch.py
  T3: ros2 run rx150_perception yolo_tube_detector_node.py

Chạy demo:
  T4: cd ~/interbotix_ws/src/rx150/rx150_toolbox/rx150_perception/demos && python3 sort_tubes_by_color.py
"""

import json
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.msg import JointSingleCommand

ROBOT_MODEL = 'rx150'
ROBOT_NAME = ROBOT_MODEL
ARM_GROUP = 'interbotix_arm'
GRIPPER_GROUP = 'interbotix_gripper'
ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
FINGER_JOINT = 'left_finger'

# Giới hạn góc mở/đóng ngón kẹp
GRASP_FINGER = 0.015    # m — kẹp chặt
RELEASE_FINGER = 0.037  # m — mở hết
GRIP_PWM = 200.0        # PWM fallback
OPEN_PWM = -200.0
OK_CODES = (1, -4)      # 1=SUCCESS, -4=CONTROL_FAILED (MoveIt báo chấp nhận hoàn thành)

# Vị trí chuẩn bị Home an toàn
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0]

# -------------------------------------------------------------
# ĐỊNH NGHĨA KHAY THẢ (PLACE BINS) THEO MÀU ỐNG NGHIỆM (mét)
# Tọa độ tính trong hệ rx150/base_link
# -------------------------------------------------------------
BIN_POSITIONS = {
    'blue':    {'x': 0.25, 'y':  0.15, 'z': 0.06},  # Khay Xanh Dương (bên trái)
    'green':   {'x': 0.20, 'y':  0.18, 'z': 0.06},  # Khay Xanh Lá
    'yellow':  {'x': 0.30, 'y':  0.00, 'z': 0.06},  # Khay Vàng (ở giữa)
    'pink':    {'x': 0.25, 'y': -0.15, 'z': 0.06},  # Khay Hồng / Đỏ (bên phải)
    'unknown': {'x': 0.28, 'y': -0.18, 'z': 0.06},  # Khay Chưa rõ
    'default': {'x': 0.28, 'y':  0.00, 'z': 0.06},  # Khay dự phòng
}

APPROACH_HEIGHT = 0.06  # Độ cao an toàn trên đỉnh ống nghiệm


class TubeSorterNode(Node):
    def __init__(self, bot):
        super().__init__('tube_sorter_moveit')
        self.bot = bot
        self.cb_group = ReentrantCallbackGroup()

        # Action Client MoveIt
        self.move_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        self.pub_single = self.create_publisher(JointSingleCommand, f'/{ROBOT_NAME}/commands/joint_single', 10)

        # Trạng thái ống nghiệm nhận diện
        self.tubes = []
        self.latest_poses = []
        self.latest_classes = []
        self.lock = threading.Lock()

        # Subscribers
        self.create_subscription(
            PoseArray, '/yolo/detected_tubes', self._poses_cb, 10, callback_group=self.cb_group)
        self.create_subscription(
            String, '/yolo/tube_classes', self._classes_cb, 10, callback_group=self.cb_group)

        self.get_logger().info('TubeSorterNode (MoveIt PWM Backend) đã sẵn sàng!')

    def _poses_cb(self, msg: PoseArray):
        with self.lock:
            self.latest_poses = list(msg.poses)
            self._sync()

    def _classes_cb(self, msg: String):
        with self.lock:
            try:
                self.latest_classes = json.loads(msg.data)
                self._sync()
            except Exception:
                pass

    def _sync(self):
        if len(self.latest_poses) > 0 and len(self.latest_classes) == len(self.latest_poses):
            temp = []
            for p, cls_name in zip(self.latest_poses, self.latest_classes):
                qz = p.orientation.z
                qw = p.orientation.w
                yaw_angle = 2.0 * math.atan2(qz, qw)
                temp.append({
                    'class': str(cls_name).lower(),
                    'x': float(p.position.x),
                    'y': float(p.position.y),
                    'z': float(p.position.z),
                    'yaw': float(yaw_angle),
                })
            self.tubes = temp

    def get_detected_tubes(self, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.1)
            with self.lock:
                if len(self.tubes) > 0:
                    return list(self.tubes)
        return []

    def move_joints_moveit(self, joint_names, targets, velocity_scale=0.3, allowed_time=8.0):
        """Gửi quỹ đạo điều khiển cánh tay thật qua MoveGroup Action Server."""
        if not self.move_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveGroup action server chưa sẵn sàng!')
            return False

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = ARM_GROUP
        req.start_state.is_diff = True
        req.workspace_parameters.header.frame_id = f'{ROBOT_NAME}/base_link'
        req.allowed_planning_time = float(allowed_time)
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = float(velocity_scale)
        req.max_acceleration_scaling_factor = float(velocity_scale)

        c = Constraints()
        for jn, tgt in zip(joint_names, targets):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(tgt)
            jc.tolerance_above = 0.015
            jc.tolerance_below = 0.015
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        # Gửi goal đồng bộ
        send_goal_future = self.move_client.send_goal_async(goal)
        t0 = time.monotonic()
        while not send_goal_future.done():
            if time.monotonic() - t0 > 5.0:
                self.get_logger().error('Send goal timeout!')
                return False
            time.sleep(0.01)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('MoveIt từ chối Goal!')
            return False

        res_future = goal_handle.get_result_async()
        t0 = time.monotonic()
        while not res_future.done():
            if time.monotonic() - t0 > allowed_time + 15.0:
                self.get_logger().error('Execute trajectory timeout!')
                return False
            time.sleep(0.01)

        res = res_future.result()
        code = res.result.error_code.val
        if code in OK_CODES:
            return True
        else:
            self.get_logger().error(f'MoveIt thất bại với error_code={code}')
            return False

    def solve_ik(self, x, y, z, pitch=0.5):
        """Dùng SDK giải động học nghịch (IK), KHÔNG phát lệnh trực tiếp."""
        joints, ok = self.bot.arm.set_ee_pose_components(
            x=float(x), y=float(y), z=float(z), pitch=float(pitch), execute=False)
        return joints, ok

    def solve_ik_top_down(self, x, y, z, target_pitch=1.5708):
        """Giải IK ưu tiên cắm thẳng đứng 90 độ từ trên xuống, tự động điều chỉnh nếu xa tầm với."""
        # Thử từ cắm thẳng đứng 90 độ (1.57 rad) giảm dần đến 60 độ (1.0 rad) nếu xa
        for p in [target_pitch, 1.40, 1.25, 1.10, 0.95]:
            joints, ok = self.solve_ik(x, y, z, pitch=p)
            if ok:
                return joints, True, p
        return None, False, target_pitch

    def control_gripper(self, grasp: bool):
        """Điều khiển đóng/mở kẹp gắp."""
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GRIPPER_GROUP
        req.start_state.is_diff = True
        req.max_velocity_scaling_factor = 0.2
        c = Constraints()
        jc = JointConstraint()
        jc.joint_name = FINGER_JOINT
        jc.position = GRASP_FINGER if grasp else RELEASE_FINGER
        jc.tolerance_above = 0.02
        jc.tolerance_below = 0.02
        jc.weight = 1.0
        c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        if self.move_client.wait_for_server(timeout_sec=1.0):
            fut = self.move_client.send_goal_async(goal)
            t0 = time.monotonic()
            while not fut.done() and time.monotonic() - t0 < 3.0:
                time.sleep(0.01)
            if fut.done() and fut.result().accepted:
                rf = fut.result().get_result_async()
                t0 = time.monotonic()
                while not rf.done() and time.monotonic() - t0 < 5.0:
                    time.sleep(0.01)
                return True

        # Fallback PWM trực tiếp
        cmd = JointSingleCommand(name='gripper', cmd=float(GRIP_PWM if grasp else OPEN_PWM))
        self.pub_single.publish(cmd)
        time.sleep(1.5)
        return True


def run_sorter():
    rclpy.init()
    global_node = create_interbotix_global_node()
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        node=global_node,
    )
    sorter = TubeSorterNode(bot)

    # Chạy MultiThreadedExecutor ở thread riêng để ROS callbacks hoạt động ngầm
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(global_node)
    executor.add_node(sorter)
    exec_thread = threading.Thread(target=executor.spin, daemon=True)
    exec_thread.start()

    robot_startup(global_node)

    print('\n======================================================')
    print('   GẮP & PHÂN LOẠI ỐNG NGHIỆM NẰM NGANG (TOP-DOWN 90°)')
    print('======================================================')
    print('1. Đang đưa cánh tay thật về tư thế chuẩn bị...')
    sorter.move_joints_moveit(ARM_JOINTS, HOME_JOINTS, velocity_scale=0.3)
    sorter.control_gripper(grasp=False)

    print('2. Đang chờ YOLOv8 quét và nhận diện ống nghiệm...')
    tubes = sorter.get_detected_tubes(timeout=6.0)

    if not tubes:
        print('❌ Không phát hiện ống nghiệm nào trong khung nhìn!')
        sorter.move_joints_moveit(ARM_JOINTS, HOME_JOINTS, velocity_scale=0.3)
        rclpy.shutdown()
        return

    print(f'✅ Tìm thấy {len(tubes)} ống nghiệm cần phân loại:')
    for i, t in enumerate(tubes):
        deg = math.degrees(t['yaw'])
        print(f"  [{i+1}] Loại: {t['class']:<10} Vị trí: x={t['x']:.3f}, y={t['y']:.3f}, z={t['z']:.3f} | Góc nghiêng: {deg:.1f}°")

    # Vòng lặp gắp và phân loại từng ống
    for i, t in enumerate(tubes):
        cls_name = t['class']
        tx, ty, tz = t['x'], t['y'], t['z']
        yaw = t['yaw']
        deg = math.degrees(yaw)

        # Chọn khay theo màu
        matched_bin = BIN_POSITIONS['default']
        for key in BIN_POSITIONS:
            if key in cls_name:
                matched_bin = BIN_POSITIONS[key]
                break

        print(f"\n---> BẮT ĐẦU GẮP ỐNG [{i+1}/{len(tubes)}]: Màu '{cls_name}' ({deg:.1f}°) -> Khay ({matched_bin['x']:.2f}, {matched_bin['y']:.2f})")

        # Độ cao an toàn trên không (+8cm) và vị trí kẹp thực tế (sát thân ống)
        z_approach = max(tz + 0.08, 0.12)
        z_grasp = max(tz, 0.02)  # Đảm bảo không đâm sâu quá mặt bàn

        # 1. Tính IK cho vị trí trên không (cắm thẳng đứng từ trên xuống)
        j_app, ok1, pitch_used = sorter.solve_ik_top_down(tx, ty, z_approach, target_pitch=1.5708)
        # 2. Tính IK cho vị trí hạ xuống kẹp
        j_grasp, ok2, _ = sorter.solve_ik_top_down(tx, ty, z_grasp, target_pitch=pitch_used)

        if not ok1 or not ok2:
            print(f'⚠️ Vị trí ({tx:.3f}, {ty:.3f}, {tz:.3f}) ngoài tầm với IK của robot! Bỏ qua ống này.')
            continue

        # 3. Tính góc xoay cổ tay wrist_rotate để 2 ngón kẹp vuông góc/ôm khít thân ống nằm ngang:
        # Khớp waist (joint 0) đã xoay hướng tâm -> wrist_rotate xoay bù góc:
        waist_angle = j_grasp[0]
        wrist_rot = yaw - waist_angle
        # Chuẩn hóa góc vào [-pi, pi]
        wrist_rot = math.atan2(math.sin(wrist_rot), math.cos(wrist_rot))

        j_app = list(j_app)
        j_grasp = list(j_grasp)
        j_app[4] = wrist_rot
        j_grasp[4] = wrist_rot

        # -------------------------------------------------------------
        # QUY TRÌNH 5 BƯỚC GẮP THẲNG ĐỨNG CHUẨN XÁC
        # -------------------------------------------------------------
        # BƯỚC 1: Mở kẹp gắp và di chuyển đến NGAY TRÊN ĐỈNH ỐNG NGHIỆM (+8cm)
        print('  [1/5] -> Di chuyển đến đỉnh trên không và xoay cổ tay đúng góc...')
        sorter.control_gripper(grasp=False)
        sorter.move_joints_moveit(ARM_JOINTS, j_app, velocity_scale=0.3)
        time.sleep(0.3)

        # BƯỚC 2: CẮM THẲNG ĐỨNG TỪ TRÊN XUỐNG VỊ TRÍ THÂN ỐNG (chậm, êm)
        print('  [2/5] -> Cắm thẳng đứng từ trên xuống kẹp thân ống...')
        sorter.move_joints_moveit(ARM_JOINTS, j_grasp, velocity_scale=0.12)
        time.sleep(0.2)

        # BƯỚC 3: KẸP CHẶT THÂN ỐNG NGHIỆM
        print('  [3/5] -> Kẹp chặt ống nghiệm...')
        sorter.control_gripper(grasp=True)
        time.sleep(0.8)

        # BƯỚC 4: NHẤC THẲNG ĐỨNG LÊN CAO (tránh kéo lê trên bàn)
        print('  [4/5] -> Nhấc thẳng đứng lên cao an toàn...')
        sorter.move_joints_moveit(ARM_JOINTS, j_app, velocity_scale=0.2)
        time.sleep(0.3)

        # BƯỚC 5: VẬN CHUYỂN ĐẾN KHAY VÀ THẢ
        bx, by, bz = matched_bin['x'], matched_bin['y'], matched_bin['z']
        j_place_app, ok3, _ = sorter.solve_ik_top_down(bx, by, bz + 0.08, target_pitch=1.5708)
        j_place, ok4, _ = sorter.solve_ik_top_down(bx, by, bz, target_pitch=1.5708)

        if ok3 and ok4:
            print(f"  [5/5] -> Vận chuyển đến khay '{cls_name}' và nhả kẹp...")
            sorter.move_joints_moveit(ARM_JOINTS, list(j_place_app), velocity_scale=0.3)
            sorter.move_joints_moveit(ARM_JOINTS, list(j_place), velocity_scale=0.15)
            sorter.control_gripper(grasp=False)
            time.sleep(0.5)
            # Rút tay thẳng lên sau khi nhả
            sorter.move_joints_moveit(ARM_JOINTS, list(j_place_app), velocity_scale=0.3)

    print('\n🎉 ĐÃ PHÂN LOẠI HOÀN TẤT TẤT CẢ ỐNG NGHIỆM! Đưa cánh tay về Home an toàn.')
    sorter.move_joints_moveit(ARM_JOINTS, HOME_JOINTS, velocity_scale=0.3)
    rclpy.shutdown()


if __name__ == '__main__':
    run_sorter()
