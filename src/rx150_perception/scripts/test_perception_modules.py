#!/usr/bin/env python3
"""
Script minh họa cách sử dụng các module từ interbotix_perception_modules
cho cánh tay RX150 + RealSense D435i + AprilTag tag36h11 ID 0 (30mm).
"""

import sys
import rclpy
from rclpy.node import Node
from interbotix_common_modules.common_robot.robot import InterbotixRobotNode

def main():
    rclpy.init(args=sys.argv)
    
    # Khởi tạo InterbotixRobotNode
    robot_node = InterbotixRobotNode(node_name='rx150_perception_test')
    robot_node.get_logger().info("Đã khởi tạo node kiểm tra interbotix_perception_modules...")

    try:
        # Import các module từ interbotix_perception_modules
        from interbotix_perception_modules.apriltag import InterbotixAprilTagInterface
        from interbotix_perception_modules.armtag import InterbotixArmTagInterface
        from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface

        robot_node.get_logger().info("Import thành công các module từ interbotix_perception_modules:")
        robot_node.get_logger().info(" - InterbotixAprilTagInterface")
        robot_node.get_logger().info(" - InterbotixArmTagInterface")
        robot_node.get_logger().info(" - InterbotixPointCloudInterface")
        
    except ImportError as e:
        robot_node.get_logger().error(f"Lỗi import interbotix_perception_modules: {e}")
    finally:
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
