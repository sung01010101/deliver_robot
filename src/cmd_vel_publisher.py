#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1.0  # publish every second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('cmd_vel publisher started.')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2   # move forward with 0.2 m/s
        msg.angular.z = 0.5  # trun ccw with 0.5 rad/s
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: linear.x = {msg.linear.x}, angular.z = {msg.angular.z}')


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
