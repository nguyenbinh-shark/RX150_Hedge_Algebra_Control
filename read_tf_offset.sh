#!/usr/bin/env bash
source /home/hust/RX150_500hz/source_all.sh >/dev/null 2>&1

python3 -c "
import rclpy, math, time
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion

rclpy.init()
node = Node('tf_offset_reader')
buf = Buffer()
listener = TransformListener(buf, node)

start = time.time()
while time.time() - start < 2.5:
    rclpy.spin_once(node, timeout_sec=0.1)

found = False
for ref_frame in ['rx150/ee_tag_link', 'rx150/ar_tag_link','rx150/ee_gripper_link', 'rx150/ee_arm_link']:
    try:
        t = buf.lookup_transform(ref_frame, 'ee_tag', rclpy.time.Time())
        dx = t.transform.translation.x * 1000.0
        dy = t.transform.translation.y * 1000.0
        dz = t.transform.translation.z * 1000.0
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        q = t.transform.rotation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        roll, pitch, yaw = [math.degrees(a) for a in rpy]

        print(f'=== KẾT QUẢ ĐO VỚI {ref_frame} ===')
        print(f'Lệch trục X : {dx:+.2f} mm')
        print(f'Lệch trục Y : {dy:+.2f} mm')
        print(f'Lệch trục Z : {dz:+.2f} mm')
        print(f'-> TỔNG LỆCH TÂM: {dist:.2f} mm')
        print(f'-> LỆCH GÓC     : Roll={roll:+.2f}°, Pitch={pitch:+.2f}°, Yaw={yaw:+.2f}°')
        print('-----------------------------------------')
        found = True
    except Exception as e:
        print(f'Không lookup được {ref_frame} -> ee_tag: {e}')

if not found:
    print('LỖI: Chưa tìm thấy frame ee_tag. Vui lòng đảm bảo ./rx150.sh eetag đang chạy và camera nhìn thấy tag!')

rclpy.shutdown()
"

