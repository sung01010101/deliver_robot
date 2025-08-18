#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import os
from datetime import datetime


class GetScogNode(Node):
    def __init__(self):
        super().__init__('get_scog_node')
        
        # Declare parameters with default values
        self.declare_parameter('scog_topic', '/scog_data')
        self.declare_parameter('output_file', 'scog_data.csv')
        self.declare_parameter('output_directory', '~/Desktop')
        self.declare_parameter('add_timestamp', True)
        
        # Get parameters
        self.scog_topic = self.get_parameter('scog_topic').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value
        self.add_timestamp = self.get_parameter('add_timestamp').get_parameter_value().bool_value
        
        # Expand user directory
        self.output_directory = os.path.expanduser(self.output_directory)
        
        # Create output directory if it doesn't exist
        os.makedirs(self.output_directory, exist_ok=True)
        
        # Create full file path
        if self.add_timestamp:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            name, ext = os.path.splitext(self.output_file)
            self.output_file = f"{name}_{timestamp}{ext}"
        
        self.csv_file_path = os.path.join(self.output_directory, self.output_file)
        
        # Log parameters
        self.get_logger().info(f"SCOG Data Logger Parameters:")
        self.get_logger().info(f"  SCOG Topic: {self.scog_topic}")
        self.get_logger().info(f"  Output File: {self.csv_file_path}")
        self.get_logger().info(f"  Add Timestamp: {self.add_timestamp}")
        
        # Initialize CSV file with headers
        self.init_csv_file()
        
        # Create subscriber
        self.scog_sub = self.create_subscription(
            Float32MultiArray, 
            self.scog_topic, 
            self.scog_callback, 
            10
        )
        
        # Counter for logged messages
        self.message_count = 0
        
        # Start time for relative timestamp calculation
        self.start_time = None
        
        self.get_logger().info(f"SCOG data logger started. Saving to: {self.csv_file_path}")

    def init_csv_file(self):
        """Initialize CSV file with headers"""
        try:
            with open(self.csv_file_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write header row based on the data from esp32_scog_node.py
                headers = [
                    'timestamp',
                    'd_left',
                    'd_right',
                    'd_odom_theta',
                    'd_imu_theta',
                    'slip_ratio_ratio',
                    'denominator',
                    'slip_l',
                    'slip_r',
                    'd_left_corr',
                    'd_right_corr',
                    'd_theta',
                    'x',
                    'y',
                    'theta'
                ]
                writer.writerow(headers)
                
            self.get_logger().info(f"CSV file initialized with headers: {self.csv_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV file: {str(e)}")
            raise e

    def scog_callback(self, msg):
        """Callback to receive and save SCOG data"""
        try:
            # Get current timestamp
            current_time = self.get_clock().now()
            current_timestamp = current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1] * 1e-9
            
            # Set start time on first message
            if self.start_time is None:
                self.start_time = current_timestamp
                self.get_logger().info(f"Start time set: {self.start_time}")
            
            # Calculate relative timestamp (starts from 0)
            timestamp = current_timestamp - self.start_time
            
            # Ensure we have the expected number of data points
            if len(msg.data) != 14:
                self.get_logger().warn(f"Expected 14 data points, got {len(msg.data)}")
                return
            
            # Prepare data row
            data_row = [timestamp] + list(msg.data)
            
            # Write to CSV file
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(data_row)
            
            self.message_count += 1
            
            # Log progress every 100 messages
            if self.message_count % 100 == 0:
                self.get_logger().info(f"Logged {self.message_count} SCOG messages")
                
            # Debug log for first few messages
            if self.message_count <= 5:
                self.get_logger().info(f"SCOG Data #{self.message_count}: {msg.data}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to save SCOG data: {str(e)}")

    def destroy_node(self):
        """Clean up when shutting down"""
        self.get_logger().info(f"SCOG data logger shutting down. Total messages logged: {self.message_count}")
        self.get_logger().info(f"Data saved to: {self.csv_file_path}")
        super().destroy_node()


def main():
    rclpy.init()
    node = GetScogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received keyboard interrupt, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
