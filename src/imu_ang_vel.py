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
        ang_vel_z = msg.angular_velocity.z

        self.get_logger().info(f'Angular Velocity Z: {ang_vel_z:.3f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = IMUYawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
