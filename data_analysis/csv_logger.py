#!/usr/bin/env python3
"""
csv_logger — Ghi dữ liệu điều khiển ra file CSV (controller-agnostic).

Subscribe các topic điều khiển (error, effort, edot, reference, gravity)
từ bất kỳ bộ điều khiển nào (fuzzy, PID, MPC...), ghi 1 dòng CSV mỗi chu kỳ.

Cách dùng:
  # Mặc định: prefix='fuzzy' → subscribe fuzzy/error, fuzzy/effort, ...
  ros2 run fuzzy_controller fuzzy_csv_logger

  # Đổi prefix cho bộ điều khiển khác:
  ros2 run fuzzy_controller fuzzy_csv_logger --ros-args \
      -p controller_prefix:=pid

  # Tuỳ chỉnh file đầu ra:
  ros2 run fuzzy_controller fuzzy_csv_logger --ros-args \
      -p output_file:=/path/to/my_data.csv \
      -p flush_rate:=2.0

Ctrl+C để dừng → file CSV sẵn sàng cho pandas / matplotlib.
"""

import os
import csv
import signal
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState

ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle', 'wrist_rotate']


def build_topics(prefix):
    """Tạo dict topic từ prefix controller.

    Ví dụ: prefix='fuzzy' → 'fuzzy/error', 'fuzzy/effort', ...
           prefix='pid'   → 'pid/error',   'pid/effort',   ...
    """
    return {
        'js':   ('joint_states',          'pos_vel'),   # luôn cố định (từ xs_sdk)
        'ref':  (f'{prefix}/reference',   'pos_vel'),   # profile reference
        'err':  (f'{prefix}/error',       'pos'),       # error
        'edot': (f'{prefix}/edot',        'vel'),       # error derivative
        'eff':  (f'{prefix}/effort',      'eff'),       # control effort (PWM)
        'grav': (f'{prefix}/gravity',     'eff'),       # gravity compensation
        'fric': (f'{prefix}/friction',    'eff'),       # friction feedforward (chỉ HAC publish)
    }


class CsvLogger(Node):
    def __init__(self):
        super().__init__('csv_logger')

        # --- Parameters ---
        self.declare_parameter('controller_prefix', 'fuzzy')
        self._prefix = self.get_parameter('controller_prefix').value

        default_name = f'{self._prefix}_data_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.declare_parameter('output_file', default_name)
        self.declare_parameter('flush_rate', 1.0)

        self._output_file = self.get_parameter('output_file').value
        self._flush_rate = self.get_parameter('flush_rate').value

        # --- State ---
        self._lock = threading.Lock()
        self._start_time = None
        self._row_count = 0
        self._latest = {}  # key -> JointState msg (latest per topic)
        self._shutting_down = False

        # --- Build CSV header ---
        self._header = ['timestamp']
        # joint_states: pos + vel
        for j in ARM_JOINTS:
            self._header.append(f'{j}_pos')
        for j in ARM_JOINTS:
            self._header.append(f'{j}_vel')
        # reference: pos + vel
        for j in ARM_JOINTS:
            self._header.append(f'{j}_ref_pos')
        for j in ARM_JOINTS:
            self._header.append(f'{j}_ref_vel')
        # error: pos
        for j in ARM_JOINTS:
            self._header.append(f'{j}_err')
        # edot: vel
        for j in ARM_JOINTS:
            self._header.append(f'{j}_edot')
        # effort (PWM)
        for j in ARM_JOINTS:
            self._header.append(f'{j}_pwm')
        # gravity
        for j in ARM_JOINTS:
            self._header.append(f'{j}_grav')
        # friction (chỉ có giá trị khi controller là HAC; controller khác để trống)
        for j in ARM_JOINTS:
            self._header.append(f'{j}_fric')

        # --- Open CSV ---
        abs_path = os.path.abspath(self._output_file)
        self._csv_file = open(abs_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(self._header)
        self.get_logger().info(f'CSV output: {abs_path}')
        self.get_logger().info(f'Columns ({len(self._header)}): {", ".join(self._header[:6])} ...')

        # --- QoS: SensorData (best-effort, keep-last 1) phù hợp với publisher ---
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscribers ---
        # joint_states là topic chính (100 Hz từ xs_sdk), dùng nó làm trigger ghi CSV.
        # Các topic controller cũng ~100 Hz nhưng có thể lệch vài ms.
        topics = build_topics(self._prefix)
        self._sub_js = self.create_subscription(
            JointState, 'joint_states', self._on_joint_states, sensor_qos)

        for key, (topic_name, _) in topics.items():
            if key == 'js':
                continue  # đã subscribe ở trên
            self.create_subscription(
                JointState, topic_name,
                lambda msg, k=key: self._on_topic(k, msg),
                sensor_qos)

        # --- Flush timer ---
        if self._flush_rate > 0:
            period = 1.0 / self._flush_rate
            self._flush_timer = self.create_timer(period, self._flush)

        subscribed = [t for _, (t, _) in topics.items()]
        self.get_logger().info(
            f'CsvLogger ready — prefix="{self._prefix}", '
            f'topics={subscribed}, flush {self._flush_rate} Hz')

    def _extract_joint_values(self, msg, field):
        """Lấy giá trị 5 khớp ARM từ JointState msg, map theo tên khớp."""
        if msg is None:
            return [float('nan')] * 5

        # Build name -> index map
        name_map = {}
        for i, name in enumerate(msg.name):
            name_map[name] = i

        values = []
        for joint in ARM_JOINTS:
            idx = name_map.get(joint)
            if idx is None:
                values.append(float('nan'))
                continue
            if field == 'position' and idx < len(msg.position):
                values.append(msg.position[idx])
            elif field == 'velocity' and idx < len(msg.velocity):
                values.append(msg.velocity[idx])
            elif field == 'effort' and idx < len(msg.effort):
                values.append(msg.effort[idx])
            else:
                values.append(float('nan'))
        return values

    def _on_topic(self, key, msg):
        """Cache latest message cho mỗi topic."""
        with self._lock:
            self._latest[key] = msg

    def _on_joint_states(self, msg):
        """Trigger chính: khi nhận joint_states, ghi 1 dòng CSV."""
        with self._lock:
            self._latest['js'] = msg

            if self._start_time is None:
                self._start_time = self.get_clock().now()

            now = self.get_clock().now()
            elapsed = (now - self._start_time).nanoseconds * 1e-9

            row = [f'{elapsed:.6f}']

            # joint_states: position
            row.extend(self._fmt(v) for v in self._extract_joint_values(msg, 'position'))
            # joint_states: velocity
            row.extend(self._fmt(v) for v in self._extract_joint_values(msg, 'velocity'))

            # reference: position + velocity
            ref_msg = self._latest.get('ref')
            row.extend(self._fmt(v) for v in self._extract_joint_values(ref_msg, 'position'))
            row.extend(self._fmt(v) for v in self._extract_joint_values(ref_msg, 'velocity'))

            # error: position
            err_msg = self._latest.get('err')
            row.extend(self._fmt(v) for v in self._extract_joint_values(err_msg, 'position'))

            # edot: velocity
            edot_msg = self._latest.get('edot')
            row.extend(self._fmt(v) for v in self._extract_joint_values(edot_msg, 'velocity'))

            # effort (PWM)
            eff_msg = self._latest.get('eff')
            row.extend(self._fmt(v) for v in self._extract_joint_values(eff_msg, 'effort'))

            # gravity
            grav_msg = self._latest.get('grav')
            row.extend(self._fmt(v) for v in self._extract_joint_values(grav_msg, 'effort'))

            # friction (chỉ HAC)
            fric_msg = self._latest.get('fric')
            row.extend(self._fmt(v) for v in self._extract_joint_values(fric_msg, 'effort'))

            self._writer.writerow(row)
            self._row_count += 1

            if self._row_count % 500 == 0:
                self.get_logger().info(
                    f'Logged {self._row_count} rows ({elapsed:.1f}s)')

    @staticmethod
    def _fmt(v):
        """Format số: 6 chữ số thập phân, NaN nếu thiếu."""
        if v != v:  # NaN check
            return ''
        return f'{v:.6f}'

    def _flush(self):
        with self._lock:
            self._csv_file.flush()

    def shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True
        with self._lock:
            self._csv_file.flush()
            self._csv_file.close()
        abs_path = os.path.abspath(self._output_file)
        self.get_logger().info(
            f'CSV saved: {abs_path} ({self._row_count} rows)')


def main(args=None):
    rclpy.init(args=args)
    node = CsvLogger()

    # Graceful shutdown on SIGINT (Ctrl+C)
    def signal_handler(sig, frame):
        node.shutdown()
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
