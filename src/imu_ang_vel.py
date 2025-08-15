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
        
        # Initialize orientation tracking
        self.current_yaw = 0.0  # Current yaw angle in radians
        self.prev_time = self.get_clock().now()
        self.first_message = True

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        ang_vel_z = msg.angular_velocity.z
        
        # Skip first message to establish timing
        if self.first_message:
            self.prev_time = current_time
            self.first_message = False
            self.get_logger().info(f'First IMU message received, starting integration')
            return
        
        # Calculate time delta
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        # Integrate angular velocity to get yaw angle
        if dt > 0:
            self.current_yaw += ang_vel_z * dt
            
            # Normalize angle to [-pi, pi]
            self.current_yaw = math.atan2(math.sin(self.current_yaw), math.cos(self.current_yaw))
        
        # Update previous time
        self.prev_time = current_time
        
        # Log information
        self.get_logger().info(f'Angular Velocity Z: {ang_vel_z:.3f} rad/s, '
                              f'Integrated Yaw: {self.current_yaw:.3f} rad ({math.degrees(self.current_yaw):.1f} deg)')
        
        # Publish TF transform
        self.publish_tf()
    
    def publish_tf(self):
        """Publish TF transform using integrated angular velocity"""
        t = TransformStamped()
        
        # Set header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'base_footprint'  # Child frame
        
        # Set translation (position) - assuming robot is at origin for angular velocity only transform
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Convert yaw angle to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.current_yaw / 2)
        t.transform.rotation.w = math.cos(self.current_yaw / 2)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().debug(f'Published TF: yaw={self.current_yaw:.3f} rad, '
                               f'quat(z={math.sin(self.current_yaw / 2):.3f}, '
                               f'w={math.cos(self.current_yaw / 2):.3f})')

def main(args=None):
    rclpy.init(args=args)
    node = IMUYawNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
