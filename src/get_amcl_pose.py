#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import csv
import os
from datetime import datetime

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        # Declare parameter for output directory
        self.declare_parameter('output_dir', os.path.expanduser('~/csv/amcl_pose/'))
        
        # Replace this with your actual topic name
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.listener_callback,
            10
        )

        # Get output directory parameter
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        
        # Create CSV file
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = f'pose_log_{now}.csv'
        self.csv_path = os.path.join(output_dir, self.csv_filename)

        # Write header
        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                'timestamp',
                'position_x', 'position_y', 'position_z',
                'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w',
                'covariance_0_0', 'covariance_0_1', '...', 'covariance_5_5'
            ])

        self.get_logger().info(f'Logging to: {self.csv_path}')

    def listener_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        cov = msg.pose.covariance  # a flat 6x6 matrix

        row = [
            timestamp,
            pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z, ori.w,
        ] + list(cov)

        with open(self.csv_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(row)

def main(args=None):
    rclpy.init(args=args)
    node = PoseLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
