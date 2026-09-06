#!/usr/bin/env python3
"""
rx150_trajectory_bridge — FollowJointTrajectory action server param-driven cho MoveIt,
stream setpoint xuống controller (fuzzy/ff) qua topic param.

MoveIt (OMPL + TOTP) plan trajectory → bridge nội suy tuyến tính theo thời gian
→ publish Float64MultiArray setpoint 100 Hz → controller bám PWM closed-loop.

Namespace rx150 → action = /rx150/arm_controller/follow_joint_trajectory
                  (khớp rx150_controllers.yaml trong interbotix_xsarm_moveit).

Parameters:
  node_name:        tên node (default 'rx150_trajectory_bridge')
  setpoint_topic:   topic setpoint (default 'setpoint', fuzzy → 'fuzzy/setpoint', ff → 'ff/setpoint')
  setpoint_rate:    tần suất stream setpoint (default 100 Hz)
  default_goal_tolerance: sai số mặc định (default 0.02 rad)
  goal_time_margin: thời gian chờ thêm sau khi kết thúc trajectory (default 0.5 s)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import threading
import math

# Thứ tự chuẩn 5 khớp arm rx150 (khớp rx150_controllers.yaml).
ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']


class Rx150TrajectoryBridge(Node):
    def __init__(self):
        # ---------- MUST initialize superclass FIRST ----------
        # Temporary node name to start, will be replaced by parameter
        super().__init__('rx150_trajectory_bridge_temp')

        # ---------- parameters ----------
        self.declare_parameter('node_name', 'rx150_trajectory_bridge')
        self.declare_parameter('setpoint_topic', 'setpoint')
        self.declare_parameter('setpoint_rate', 100.0)
        self.declare_parameter('default_goal_tolerance', 0.02)
        self.declare_parameter('goal_time_margin', 0.5)

        node_name = self.get_parameter('node_name').value
        setpoint_topic = self.get_parameter('setpoint_topic').value
        self._setpoint_rate = self.get_parameter('setpoint_rate').value
        self._default_tol = self.get_parameter('default_goal_tolerance').value
        self._goal_time_margin = self.get_parameter('goal_time_margin').value

        # ---------- callback group ----------
        self._cb_group = ReentrantCallbackGroup()

        # ---------- publisher ----------
        self._setpoint_pub = self.create_publisher(
            Float64MultiArray, setpoint_topic, 10)

        # ---------- subscriber joint_states ----------
        self._latest_js = None
        self._js_lock = threading.Lock()
        self.create_subscription(
            JointState, 'joint_states', self._js_callback, 10,
            callback_group=self._cb_group)

        # ---------- action server ----------
        # Chỉ cho phép 1 goal chạy; goal mới preempt goal cũ.
        self._goal_lock = threading.Lock()
        self._current_goal_handle = None

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'Rx150TrajectoryBridge ({node_name}) ready — action: '
            f'{self.get_namespace()}/arm_controller/follow_joint_trajectory, '
            f'setpoint_topic={setpoint_topic}')

    # ------------------------------------------------------------------ #
    # Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def _js_callback(self, msg: JointState):
        with self._js_lock:
            self._latest_js = msg

    def _goal_callback(self, goal_request):
        self.get_logger().info('Received new FollowJointTrajectory goal')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ #
    # Execute                                                            #
    # ------------------------------------------------------------------ #
    def _execute_callback(self, goal_handle):
        """Xử lý trajectory goal: nội suy & stream setpoint."""

        # --- preempt goal cũ nếu có ---
        with self._goal_lock:
            if (self._current_goal_handle is not None
                    and self._current_goal_handle.is_active):
                self.get_logger().warn('Preempting previous goal')
                self._current_goal_handle.abort()
            self._current_goal_handle = goal_handle

        result = FollowJointTrajectory.Result()
        traj: JointTrajectory = goal_handle.request.trajectory
        goal_tolerances = goal_handle.request.goal_tolerance  # list of JointTolerance

        # --- validate joint_names ---
        traj_names = list(traj.joint_names)
        arm_set = set(ARM_JOINTS)
        for jn in traj_names:
            if jn not in arm_set:
                msg = f'Joint "{jn}" is not in arm group {ARM_JOINTS}'
                self.get_logger().error(msg)
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                result.error_string = msg
                goal_handle.abort()
                return result

        if len(traj.points) == 0:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Trajectory has no points'
            goal_handle.abort()
            return result

        # --- build index map: traj joint order → arm standard order ---
        # traj_to_arm[i] = vị trí trong ARM_JOINTS của traj_names[i]
        traj_to_arm = [ARM_JOINTS.index(jn) for jn in traj_names]

        # Lấy positions cho mỗi point, quy đổi sang thứ tự arm chuẩn
        n_points = len(traj.points)
        times = []      # float seconds
        positions = []  # list of list[5]

        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            times.append(t)
            pos = [0.0] * 5
            for i, idx in enumerate(traj_to_arm):
                pos[idx] = pt.positions[i]
            positions.append(pos)

        duration = times[-1]
        self.get_logger().info(
            f'Trajectory: {n_points} points, duration={duration:.3f}s, '
            f'joints={traj_names}')

        # --- stream loop ---
        rate = self.create_rate(self._setpoint_rate)
        start = self.get_clock().now()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = ARM_JOINTS

        while rclpy.ok():
            # cancel check
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                goal_handle.canceled()
                result.error_code = result.SUCCESSFUL  # no error on cancel
                return result

            # check preempt (goal_handle no longer current)
            if not goal_handle.is_active:
                self.get_logger().info('Goal preempted')
                return result

            now = self.get_clock().now()
            elapsed = (now - start).nanoseconds * 1e-9

            # nội suy tuyến tính
            if elapsed <= 0.0:
                q = list(positions[0])
            elif elapsed >= duration:
                q = list(positions[-1])
            else:
                # tìm segment
                seg = 0
                for k in range(n_points - 1):
                    if times[k + 1] >= elapsed:
                        seg = k
                        break
                dt_seg = times[seg + 1] - times[seg]
                if dt_seg > 0.0:
                    alpha = (elapsed - times[seg]) / dt_seg
                else:
                    alpha = 1.0
                q = [
                    positions[seg][j] + alpha * (positions[seg + 1][j] - positions[seg][j])
                    for j in range(5)
                ]

            # publish setpoint
            msg = Float64MultiArray()
            msg.data = q
            self._setpoint_pub.publish(msg)

            # publish feedback
            feedback.desired.positions = q
            feedback.desired.time_from_start = rclpy.duration.Duration(
                seconds=min(elapsed, duration)).to_msg()
            with self._js_lock:
                js = self._latest_js
            if js is not None:
                actual = [0.0] * 5
                for i, name in enumerate(js.name):
                    if name in arm_set:
                        actual[ARM_JOINTS.index(name)] = js.position[i]
                feedback.actual.positions = actual
                feedback.error.positions = [
                    feedback.desired.positions[j] - actual[j] for j in range(5)
                ]
            goal_handle.publish_feedback(feedback)

            # kiểm tra hết thời gian
            if elapsed >= duration + self._goal_time_margin:
                break

            rate.sleep()

        # --- kiểm tra goal tolerance ---
        with self._js_lock:
            js = self._latest_js

        if js is None:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'No joint_states received — cannot verify goal'
            goal_handle.abort()
            return result

        # build tolerance map per joint
        tol_map = {}
        for gt in goal_tolerances:
            if gt.name in arm_set:
                tol_map[gt.name] = gt.position if gt.position > 0.0 else self._default_tol

        final_pos = positions[-1]
        actual_map = {}
        for i, name in enumerate(js.name):
            if name in arm_set:
                actual_map[name] = js.position[i]

        violated = []
        for idx, jname in enumerate(ARM_JOINTS):
            if jname not in actual_map:
                continue
            tol = tol_map.get(jname, self._default_tol)
            err = abs(final_pos[idx] - actual_map[jname])
            if err > tol:
                violated.append(f'{jname}: err={err:.4f} > tol={tol:.4f}')

        if violated:
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = 'Tolerance violated: ' + '; '.join(violated)
            self.get_logger().warn(result.error_string)
            goal_handle.abort()
        else:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = ''
            self.get_logger().info('Goal SUCCEEDED')
            goal_handle.succeed()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = Rx150TrajectoryBridge()
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
