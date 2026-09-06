#!/usr/bin/env python3
"""
gripper_trajectory_bridge — FollowJointTrajectory action server cho MoveIt
điều khiển GRIPPER rx150 (option B: dịch quỹ đạo left_finger → effort PWM).

Vì gripper chạy ở PWM mode (stack này không có trajectory controller như
ros2_control), bridge nhận quỹ đạo `left_finger` (mét) từ MoveIt → dịch sang
effort PWM qua JointSingleCommand(name='gripper') → xs_sdk. Giữ model gắp chuẩn
của Trossen (effort cố định + dừng khi chạm limit, hoặc giữ lực khi stall do gắp
vật). Xem interbotix_xs_modules/.../xs_robot/gripper.py (grasp/release/gripper_state).

Namespace rx150 → action = /rx150/gripper_controller/follow_joint_trajectory
  (khớp rx150_controllers.yaml + remap trong fuzzy_moveit.launch.py:219-222).

Quy ước dấu (giống gripper.py:258,266):
  release (mở, finger → upper 0.037): effort DƯƠNG
  grasp  (kẹp, finger → lower 0.015): effort ÂM

Lưu ý quan trọng về "thành công" khi gắp: MoveIt đặt target Grasping=0.015.
Khi kẹp vật, finger không tới được 0.015 (vật chen giữa) → bridge coi "stall khi
đang đóng" = ĐÃ GẬP = SUCCESS (không phải GOAL_TOLERANCE_VIOLATED), và GIỮ effort
để duy trì lực kẹp trong khi vận chuyển.
"""

import collections
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand


class GripperTrajectoryBridge(Node):
    def __init__(self):
        super().__init__('gripper_trajectory_bridge')

        # ---------- parameters ----------
        self.declare_parameter('setpoint_rate', 50.0)
        self.declare_parameter('gripper_name', 'gripper')
        self.declare_parameter('finger_joint', 'left_finger')
        self.declare_parameter('grasp_effort', 250.0)    # |PWM| đóng (vd 150–350)
        self.declare_parameter('release_effort', 250.0)  # |PWM| mở
        self.declare_parameter('finger_lower_limit', 0.015)  # m (đóng hết)
        self.declare_parameter('finger_upper_limit', 0.037)  # m (mở hết)
        self.declare_parameter('default_goal_tolerance', 0.004)  # m
        self.declare_parameter('goal_time_margin', 0.5)         # s
        self.declare_parameter('stall_threshold', 0.0006)       # m — dịch < này = đứng yên
        self.declare_parameter('stall_time', 0.2)               # s đứng yên liên tục = đã gắp

        self._rate_hz = float(self.get_parameter('setpoint_rate').value)
        self._gripper_name = self.get_parameter('gripper_name').value
        self._finger = self.get_parameter('finger_joint').value
        self._grasp_effort = float(self.get_parameter('grasp_effort').value)
        self._release_effort = float(self.get_parameter('release_effort').value)
        self._lower = float(self.get_parameter('finger_lower_limit').value)
        self._upper = float(self.get_parameter('finger_upper_limit').value)
        self._default_tol = float(self.get_parameter('default_goal_tolerance').value)
        self._margin = float(self.get_parameter('goal_time_margin').value)
        self._stall_thr = float(self.get_parameter('stall_threshold').value)
        self._stall_time = float(self.get_parameter('stall_time').value)

        self._cb_group = ReentrantCallbackGroup()

        # ---------- publisher JointSingleCommand → xs_sdk ----------
        self._cmd_pub = self.create_publisher(
            JointSingleCommand, 'commands/joint_single', 10)

        # ---------- subscriber joint_states ----------
        self._latest_js = None
        self._js_lock = threading.Lock()
        self.create_subscription(
            JointState, 'joint_states', self._js_cb, 10,
            callback_group=self._cb_group)

        # ---------- action server (single goal, preempt) ----------
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'GripperTrajectoryBridge ready — action: '
            f'{self.get_namespace()}/gripper_controller/follow_joint_trajectory')

    # ------------------------------------------------------------------ #
    def _js_cb(self, msg: JointState):
        with self._js_lock:
            self._latest_js = msg

    def _finger_pos(self):
        """Trả vị trí left_finger (m) hiện tại, hoặc None nếu chưa có."""
        with self._js_lock:
            js = self._latest_js
        if js is None or self._finger not in js.name:
            return None
        return float(js.position[js.name.index(self._finger)])

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Gripper goal cancel requested')
        return CancelResponse.ACCEPT

    def _send(self, effort):
        msg = JointSingleCommand(name=self._gripper_name, cmd=float(effort))
        self._cmd_pub.publish(msg)

    def _zero(self):
        self._send(0.0)

    # ------------------------------------------------------------------ #
    def _execute_cb(self, goal_handle):
        """Dịch quỹ đạo left_finger → effort PWM, stall/limit-aware."""

        # --- preempt goal cũ ---
        with self._goal_lock:
            if (self._current_goal_handle is not None
                    and self._current_goal_handle.is_active):
                self.get_logger().warn('Preempting previous gripper goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle

        result = FollowJointTrajectory.Result()
        traj: JointTrajectory = goal_handle.request.trajectory

        # --- validate ---
        if self._finger not in traj.joint_names:
            msg = f'Trajectory phải chứa joint "{self._finger}"'
            self.get_logger().error(msg)
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = msg
            goal_handle.abort()
            return result
        if len(traj.points) == 0:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Trajectory rỗng'
            goal_handle.abort()
            return result

        fi = traj.joint_names.index(self._finger)
        times, desireds = [], []
        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            times.append(t)
            desireds.append(float(pt.positions[fi]))
        duration = times[-1]
        target = desireds[-1]

        current = self._finger_pos()
        if current is None:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Chưa nhận joint_states cho gripper'
            goal_handle.abort()
            return result

        tol = self._default_tol
        if abs(target - current) <= tol:
            self.get_logger().info('Gripper đã ở target — succeed ngay')
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            return result

        # --- hướng & effort ---
        grasp = target < current              # đóng (finger → lower)
        effort = -self._grasp_effort if grasp else self._release_effort
        direction = 'grasp' if grasp else 'release'
        self.get_logger().info(
            f'Gripper {direction}: target={target:.4f} current={current:.4f} '
            f'effort={effort:.0f} duration={duration:.3f}s')

        # --- stream loop ---
        rate = self.create_rate(self._rate_hz)
        start = self.get_clock().now()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = [self._finger]

        window = collections.deque()   # (elapsed, pos) trong stall_time gần nhất
        stalled = False
        reached = False                 # đã chạm limit (đóng/mở hết cỡ)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._zero()
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                self.get_logger().info('Gripper goal canceled')
                return result
            if not goal_handle.is_active:
                self.get_logger().info('Gripper goal preempted')
                return result

            now = self.get_clock().now()
            elapsed = (now - start).nanoseconds * 1e-9

            actual = self._finger_pos()
            if actual is None:
                rate.sleep()
                continue

            # gửi effort; nếu đã chạm limit → zero (mirror gripper_state)
            if (effort < 0 and actual <= self._lower) or (effort > 0 and actual >= self._upper):
                self._zero()
                reached = True
            else:
                self._send(effort)

            # feedback
            feedback.desired.positions = [target]
            feedback.desired.time_from_start = rclpy.duration.Duration(
                seconds=min(elapsed, duration)).to_msg()
            feedback.actual.positions = [actual]
            feedback.error.positions = [target - actual]
            goal_handle.publish_feedback(feedback)

            # hoàn tất theo hướng
            if grasp:
                if actual <= target + tol or reached:
                    break
            else:
                if actual >= target - tol or reached:
                    break

            # stall detect (chỉ khi đã có đủ dữ liệu): finger đứng yên dù có effort
            window.append((elapsed, actual))
            while window and elapsed - window[0][0] > self._stall_time:
                window.popleft()
            if (elapsed > self._stall_time and window
                    and (max(p for _, p in window) - min(p for _, p in window))
                    < self._stall_thr):
                stalled = True
                break

            if elapsed >= duration + self._margin:
                break

            rate.sleep()

        # ---------------- kết quả ----------------
        actual = self._finger_pos()
        if grasp:
            # gắp: tới limit HOẶC stall (chạm vật) => SUCCESS, GIỮ effort để giữ grip
            if (reached or stalled
                    or (actual is not None and actual <= target + tol)):
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = ''
                self.get_logger().info(
                    f'Grasp SUCCESS ({("stall/gripped" if stalled else "limit")}, '
                    f'finger={actual if actual is not None else float("nan"):.4f})')
                goal_handle.succeed()
            else:
                self._zero()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = f'Grasp chưa hoàn tất (finger={actual})'
                self.get_logger().warn(result.error_string)
                goal_handle.abort()
        else:
            # mở: tới upper => SUCCESS và zero PWM (không cần giữ lực)
            if reached or (actual is not None and actual >= target - tol):
                self._zero()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = ''
                self.get_logger().info(
                    f'Release SUCCESS (finger={actual if actual is None else actual:.4f})'
                    if actual is None else f'Release SUCCESS (finger={actual:.4f})')
                goal_handle.succeed()
            else:
                self._zero()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = f'Release chưa hoàn tất (finger={actual})'
                self.get_logger().warn(result.error_string)
                goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = GripperTrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
