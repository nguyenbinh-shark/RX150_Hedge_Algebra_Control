#!/usr/bin/env python3
"""
fuzzy_bridge_test — kiểm tra pipeline bridge mà KHÔNG cần move_group.

Gửi 2 goal FollowJointTrajectory tới /rx150/arm_controller/follow_joint_trajectory:
  1) ZERO-MOTION: target = vị trí hiện tại → bridge stream setpoint hằng,
     fuzzy_node giữ tay máy đứng yên. Mong đợi SUCCESSFUL.
  2) SMALL-MOTION: target = hiện tại + delta nhỏ trên 1 khớp (mặc định waist
     +0.10 rad ≈ 5.7°) → kiểm tra bám quỹ đạo thực sự. Mong đợi SUCCESSFUL.

Nếu cả hai PASS, chuỗi bridge → fuzzy_node → xs_sdk đã hoạt động đúng và sẵn
sàng cho move_group Plan & Execute.

Yêu cầu đang chạy: xsarm_control (xs_sdk) + fuzzy_node + fuzzy_trajectory_bridge.
CHÚ Ý: fuzzy_node torque ON + PWM kéo tay máy về home pose khi khởi động —
       chỉ chạy test khi tay máy đã ở vùng an toàn.

Cách dùng:
  ros2 run fuzzy_controller fuzzy_bridge_test                # delta mặc định 0.10 rad ở waist
  ros2 run fuzzy_controller fuzzy_bridge_test -- 0.05 elbow  # delta 0.05 rad ở khớp elbow
"""

import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']
ACTION_NAME = '/rx150/arm_controller/follow_joint_trajectory'
ACTION_STATUS_TOPIC = f'{ACTION_NAME}/_action/status'
MAX_DELTA_RAD = {              # delta tối đa cho phép (rad) để tránh nhập liệu nguy hiểm
    'waist': 0.30, 'shoulder': 0.25, 'elbow': 0.25, 'wrist_angle': 0.25, 'wrist_rotate': 0.30,
}
JOINT_LIMITS_RAD = {
    'waist': (-math.pi + 1e-5, math.pi - 1e-5),
    'shoulder': (math.radians(-106), math.radians(100)),
    'elbow': (math.radians(-102), math.radians(95)),
    'wrist_angle': (math.radians(-100), math.radians(123)),
    'wrist_rotate': (-math.pi + 1e-5, math.pi - 1e-5),
}
JOINT_LIMIT_MARGIN = 0.02      # rad — không test sát hard limit
ZERO_MOTION_DURATION = 2.0     # giây
SMALL_MOTION_DURATION = 3.0    # giây
TEST_JOINT_TOL = 0.02          # rad — tiêu chí nghiêm cho khớp đang được kiểm tra
HOLD_JOINT_TOL = 0.05          # rad — khớp chịu tải, controller chưa gravity-comp/integral
MIN_MOTION_FRACTION = 0.80     # phải quan sát được ít nhất 80% delta được yêu cầu


def parse_args(argv):
    delta = 0.10
    joint = 'waist'
    if argv:
        try:
            delta = float(argv[0])
            if len(argv) > 1:
                joint = argv[1]
        except ValueError:
            joint = argv[0]  # chỉ tên khớp, giữ delta mặc định
    if joint not in ARM_JOINTS:
        raise ValueError(f'joint "{joint}" không thuộc arm {ARM_JOINTS}')
    if not math.isfinite(delta):
        raise ValueError('delta phải là số hữu hạn')
    limit = MAX_DELTA_RAD[joint]
    if abs(delta) > limit:
        raise ValueError(f'|delta|={abs(delta):.3f} > giới hạn an toàn {limit:.2f} rad ở {joint}')
    return delta, joint


class BridgeTest(Node):
    def __init__(self, delta, joint):
        super().__init__('fuzzy_bridge_test')
        self._delta = delta
        self._joint = joint
        self._js = None
        self._ac = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.create_subscription(JointState, '/rx150/joint_states',
                                 self._js_cb, 10)

    def _js_cb(self, msg):
        self._js = msg

    def wait_for_state(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout
        while self._js is None and rclpy.ok():
            if self.get_clock().now().nanoseconds * 1e-9 > end:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_arm_pos()

    def current_arm_pos(self):
        if self._js is None:
            return None
        m = {n: p for n, p in zip(self._js.name, self._js.position)}
        try:
            positions = [m[j] for j in ARM_JOINTS]
        except KeyError as e:
            self.get_logger().error(f'joint_states thiếu khớp {e}')
            return None
        if not all(math.isfinite(position) for position in positions):
            self.get_logger().error('joint_states chứa NaN hoặc infinity')
            return None
        return positions

    def target_is_safe(self, joint, target):
        lower, upper = JOINT_LIMITS_RAD[joint]
        safe_lower = lower + JOINT_LIMIT_MARGIN
        safe_upper = upper - JOINT_LIMIT_MARGIN
        if safe_lower <= target <= safe_upper:
            return True
        self.get_logger().error(
            f'target {joint}={target:.3f} rad nằm ngoài vùng test an toàn '
            f'[{safe_lower:.3f}, {safe_upper:.3f}] rad')
        return False

    def make_goal(self, start_pos, end_pos, duration, label, strict_joint=None):
        """Tạo goal 2 điểm; chỉ pha motion mới siết tolerance khớp được test."""
        tol = [
            JointTolerance(
                name=j,
                position=TEST_JOINT_TOL if j == strict_joint else HOLD_JOINT_TOL,
            )
            for j in ARM_JOINTS
        ]
        p0 = JointTrajectoryPoint(positions=list(start_pos),
                                  time_from_start=Duration(seconds=0.0).to_msg())
        p1 = JointTrajectoryPoint(positions=list(end_pos),
                                  time_from_start=Duration(seconds=duration).to_msg())
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_JOINTS)
        goal.trajectory.points = [p0, p1]
        goal.goal_tolerance = tol
        goal.goal_time_tolerance = Duration(seconds=0.5).to_msg()
        tolerance_text = (
            f'tol({strict_joint})={TEST_JOINT_TOL:.3f}, tol(hold)={HOLD_JOINT_TOL:.3f}'
            if strict_joint else f'tol(hold)={HOLD_JOINT_TOL:.3f}'
        )
        self.get_logger().info(
            f'[{label}] gửi goal {duration:.1f}s: start={[round(v,3) for v in start_pos]} '
            f'-> end={[round(v,3) for v in end_pos]}; '
            f'{tolerance_text}')
        return goal

    def send_and_wait(self, goal, label, timeout=15.0):
        if not self._ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('action server không sẵn sàng sau 5s')
            return None
        if not self.action_server_is_unique():
            return None
        future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error(f'[{label}] goal bị REJECT')
            return None
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=timeout)
        if res_fut.done() and res_fut.result() is not None:
            return res_fut.result().result
        self.get_logger().error(f'[{label}] không nhận được result sau {timeout}s')
        cancel_fut = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_fut, timeout_sec=2.0)
        if cancel_fut.done():
            self.get_logger().warning(f'[{label}] đã yêu cầu hủy goal do timeout')
        return None

    def action_server_is_unique(self):
        """Status topic phải có đúng một publisher, tương ứng một action server."""
        for _ in range(10):
            servers = self.get_publishers_info_by_topic(ACTION_STATUS_TOPIC)
            if len(servers) == 1:
                return True
            if len(servers) > 1:
                names = [f'{info.node_namespace}/{info.node_name}' for info in servers]
                self.get_logger().error(
                    f'phát hiện {len(servers)} action servers cho {ACTION_NAME}: {names}')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(f'không tìm thấy status publisher cho {ACTION_NAME}')
        return False

    def state_within_tolerance(self, expected, actual, strict_joint=None):
        if actual is None:
            return False
        violations = []
        for index, joint in enumerate(ARM_JOINTS):
            tolerance = TEST_JOINT_TOL if joint == strict_joint else HOLD_JOINT_TOL
            error = abs(expected[index] - actual[index])
            if error > tolerance:
                violations.append(
                    f'{joint}: err={error:.4f} > tol={tolerance:.4f}')
        if violations:
            self.get_logger().error(
                'joint_states tự kiểm tra tolerance FAIL: ' + '; '.join(violations))
            return False
        return True

    def motion_was_observed(self, start, actual):
        if actual is None:
            return False
        index = ARM_JOINTS.index(self._joint)
        observed = actual[index] - start[index]
        required = abs(self._delta) * MIN_MOTION_FRACTION
        same_direction = observed * self._delta > 0.0
        ok = same_direction and abs(observed) >= required
        message = (
            f'chuyển động đo được {self._joint}: requested={self._delta:+.4f}, '
            f'observed={observed:+.4f}, required>={required:.4f} rad')
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return ok

    def run(self):
        cur = self.wait_for_state()
        if cur is None:
            self.get_logger().error('không nhận joint_states — xs_sdk chạy chưa?')
            return 1

        print('\n========================================================')
        print(f' Vị trí hiện tại: {dict(zip(ARM_JOINTS, [round(v,3) for v in cur]))}')
        print(f' Test motion   : {self._joint} += {self._delta:.3f} rad '
              f'({self._delta * 57.2958:+.2f}°)')
        print('========================================================\n')

        # --- Phase 1: zero-motion ---
        r1 = self.send_and_wait(self.make_goal(cur, cur, ZERO_MOTION_DURATION, 'zero'),
                                'zero-motion', timeout=ZERO_MOTION_DURATION + 8.0)
        zero_actual = self.current_arm_pos()
        zero_state_ok = self.state_within_tolerance(cur, zero_actual)
        self._report('zero-motion', r1, expected=cur, actual=zero_actual)
        if not self._ok(r1) or not zero_state_ok:
            self.get_logger().error('zero-motion FAIL — bỏ qua small-motion để bảo đảm an toàn')
            return 2

        # --- Phase 2: small-motion ---
        measured_after_zero = self.current_arm_pos()
        if measured_after_zero is None:
            self.get_logger().error('mất joint_states sau zero-motion')
            return 2
        joint_index = ARM_JOINTS.index(self._joint)
        # Giữ nguyên vector reference của zero-goal ở các khớp chịu tải. Nếu lấy
        # lại toàn bộ vị trí đo làm reference, PWM giữ tải bị reset lần hai và
        # elbow/shoulder có thể võng thêm trước khi sinh lại sai số giữ lực.
        start = list(cur)
        start[joint_index] = measured_after_zero[joint_index]
        end = list(start)
        end[joint_index] += self._delta
        if not self.target_is_safe(self._joint, end[joint_index]):
            return 2
        r2 = self.send_and_wait(self.make_goal(
                                    start, end, SMALL_MOTION_DURATION, 'small',
                                    strict_joint=self._joint),
                                'small-motion', timeout=SMALL_MOTION_DURATION + 8.0)
        final = self.current_arm_pos()
        self._report('small-motion', r2, expected=end, actual=final)
        small_state_ok = self.state_within_tolerance(
            end, final, strict_joint=self._joint)
        motion_ok = self.motion_was_observed(start, final)

        ok = self._ok(r1) and zero_state_ok and self._ok(r2) and small_state_ok and motion_ok
        print('\n========================================================')
        print(' KẾT QUẢ: ' + ('TẤ CẢ PASS ✓ — bridge pipeline OK' if ok
                               else 'CÓ FAIL ✗ — xem chi tiết trên'))
        print('========================================================\n')
        return 0 if ok else 2

    @staticmethod
    def _ok(result):
        return result is not None and \
            result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def _report(self, label, result, expected=None, actual=None):
        ok = self._ok(result)
        code = result.error_code if result else 'NO_RESULT'
        s = result.error_string if result else ''
        print(f'--- {label}: {"PASS ✓" if ok else "FAIL ✗"} (code={code} {s})')
        if expected is not None and actual is not None:
            err = [expected[i] - actual[i] for i in range(5)]
            print(f'    mong đợi : {[round(v,3) for v in expected]}')
            print(f'    thực tế  : {[round(v,3) for v in actual]}')
            print(f'    sai số   : {[round(v,4) for v in err]} rad')


def main():
    try:
        delta, joint = parse_args(sys.argv[1:])
    except (ValueError, IndexError) as e:
        print(f' tham số sai: {e}', file=sys.stderr)
        print(' dùng: fuzzy_bridge_test [delta_rad=0.10] [joint=waist]', file=sys.stderr)
        return 1

    rclpy.init()
    node = BridgeTest(delta, joint)
    try:
        return node.run()
    except KeyboardInterrupt:
        print('\n[ngắt]')
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
