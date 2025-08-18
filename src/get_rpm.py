#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import csv
import os
from datetime import datetime


class GetScogNode(Node):
    def __init__(self):
        super().__init__('get_rpm_node')
        
        # Declare parameters with default values
        self.declare_parameter('rpm_topic', '/rpm_data')
        self.declare_parameter('output_file', 'rpm_data.csv')
        self.declare_parameter('output_directory', '~/Desktop')
        self.declare_parameter('add_timestamp', True)
        
        # Get parameters
        self.rpm_topic = self.get_parameter('rpm_topic').get_parameter_value().string_value
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
        self.get_logger().info(f"RPM Data Logger Parameters:")
        self.get_logger().info(f"  RPM Topic: {self.rpm_topic}")
        self.get_logger().info(f"  Output File: {self.csv_file_path}")
        self.get_logger().info(f"  Add Timestamp: {self.add_timestamp}")
        
        # Initialize CSV file with headers
        self.init_csv_file()
        
        # Create subscriber
        self.rpm_sub = self.create_subscription(
            Float32MultiArray, 
            self.rpm_topic, 
            self.rpm_callback, 
            10
        )
        
        # Counter for logged messages
        self.message_count = 0
        
        self.get_logger().info(f"SCOG data logger started. Saving to: {self.csv_file_path}")

    def init_csv_file(self):
        """Initialize CSV file with headers"""
        try:
            with open(self.csv_file_path, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write header row based on the data from esp32_scog_node.py
                headers = [
                    'timestamp',
                    'pwm_left',
                    'pwm_right',
                    'desired_rpm_left',
                    'desired_rpm_right',
                    'actual_rpm_left',
                    'actual_rpm_right',
                ]
                writer.writerow(headers)
                
            self.get_logger().info(f"CSV file initialized with headers: {self.csv_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV file: {str(e)}")
            raise e

    def rpm_callback(self, msg):
        """Callback to receive and save RPM data"""
        try:
            # Get current timestamp
            current_time = self.get_clock().now()
            timestamp = current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1] * 1e-9
            
            # Ensure we have the expected number of data points
            if len(msg.data) != 6:
                self.get_logger().warn(f"Expected 6 data points, got {len(msg.data)}")
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
                self.get_logger().info(f"Logged {self.message_count} RPM messages")

            # Debug log for first few messages
            if self.message_count <= 5:
                self.get_logger().info(f"RPM Data #{self.message_count}: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"Failed to save RPM data: {str(e)}")

    def destroy_node(self):
        """Clean up when shutting down"""
        self.get_logger().info(f"RPM data logger shutting down. Total messages logged: {self.message_count}")
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
