#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('demo_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.05  # publish every 0.05 seconds for smoother sine wave
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Sine wave parameters
        self.start_time = time.time()
        self.amplitude = 0.3  # maximum linear velocity (m/s)
        self.frequency = 0.1  # frequency in Hz (cycles per second)
        
        self.get_logger().info('demo cmd_vel publisher started with sine wave linear velocity.')

    def timer_callback(self):
        msg = Twist()
        
        # Calculate time elapsed since start
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Generate sine wave for linear velocity
        msg.linear.x = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed_time)
        msg.angular.z = 0.0  # No rotation
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x = {msg.linear.x:.3f}, angular.z = {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
