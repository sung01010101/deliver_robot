#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
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
        
        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store latest IMU quaternion data
        self.imu_quat_z = 0.0
        self.imu_quat_w = 1.0

    def imu_callback(self, msg):
        z = msg.orientation.z
        w = msg.orientation.w
        
        # Store quaternion components
        self.imu_quat_z = z
        self.imu_quat_w = w

        # Convert quaternion (z,w) to yaw angle (rad)
        yaw_rad = 2 * math.atan2(z, w)
        
        # Normalize angle to [-pi, pi]
        yaw_rad = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))

        yaw_deg = math.degrees(yaw_rad)

        self.get_logger().info(f'Yaw: {yaw_rad:.3f} rad | {yaw_deg:.2f} deg')
        
        # Publish TF transform
        self.publish_tf()
    
    def publish_tf(self):
        """Publish TF transform using IMU quaternion data"""
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'base_footprint'  # Child frame
        
        # Set translation (position) - assuming robot is at origin for IMU-only transform
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Set rotation using IMU quaternion directly
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = self.imu_quat_z
        t.transform.rotation.w = self.imu_quat_w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().debug(f'Published TF: z={self.imu_quat_z:.3f}, w={self.imu_quat_w:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = IMUYawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
