#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')

        # Declare parameters with default values
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('w', 0.0)

        # Get parameters
        self.x = self.get_parameter('x').get_parameter_value().double_value
        self.y = self.get_parameter('y').get_parameter_value().double_value
        self.z = self.get_parameter('z').get_parameter_value().double_value
        self.w = self.get_parameter('w').get_parameter_value().double_value

        # Log parameters
        self.get_logger().info(f"Setting initial pose with parameters:")
        self.get_logger().info(f"  Position: ({self.x}, {self.y})")
        self.get_logger().info(f"  Orientation: ({self.z}, {self.w})")

        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_initial_pose)
        self.published = False

    def publish_initial_pose(self):
        if self.published:
            return  # Only publish once
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Set position (x, y, z)
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Set orientation (quaternion)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = self.z
        msg.pose.pose.orientation.w = self.w

        # Set covariance (6x6 matrix, 36 elements, here we set small uncertainty for x/y/theta)
        msg.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.0, 0, 0, 0,
            0, 0, 0, 0.0, 0, 0,
            0, 0, 0, 0, 0.0, 0,
            0, 0, 0, 0, 0, 0.06853891909122467  # yaw covariance
        ]

        self.publisher_.publish(msg)
        self.get_logger().info('Published initial pose.')
        self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPose()
    rclpy.spin_once(node, timeout_sec=2)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
