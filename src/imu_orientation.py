#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

class IMUYawNode(Node):
    def __init__(self):
        super().__init__('imu_yaw_node')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        z = msg.orientation.z
        w = msg.orientation.w

        # Convert quaternion (z,w) to yaw angle (rad)
        yaw_rad = 2 * math.atan2(z, w)
        
        # Normalize angle to [-pi, pi]
        yaw_rad = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))

        yaw_deg = math.degrees(yaw_rad)

        self.get_logger().info(f'Yaw: {yaw_rad:.3f} rad | {yaw_deg:.2f} deg')

def main(args=None):
    rclpy.init(args=args)
    node = IMUYawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
