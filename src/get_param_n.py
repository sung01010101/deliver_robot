#!/usr/bin/env python3

"""
Experiment code to get slip ratio of two wheels
Slip ratio are then calculated with formula (8)(9) with below parameters.
Result slip ratio and velocity of two wheels then stored as a csv file.

- Target velocity: set by cmd_vel topic.
- Angular velocity: set by IMU.
- Wheel Base: fixed.
- Wheel velocity: measured by encoders.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial
import math
import csv
import os
from datetime import datetime


class Esp32SerialNode(Node):
    def __init__(self):
        super().__init__('esp32_serial_node')
        
        # Declare parameters with default values
        self.declare_parameter('encoder_cpr', 700.0)
        self.declare_parameter('motor_max_rpm', 330)
        self.declare_parameter('wheel_base', 0.465)
        self.declare_parameter('wheel_radius', 0.019)
        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('serial_timeout', 1.0)
        self.declare_parameter('serial_timer', 0.01)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('csv_filename', 'slip_ratio_data.csv')
        self.declare_parameter('output_directory', os.path.expanduser('~/csv/param_n/'))
        
        # Get parameters
        self.counts_per_rev = self.get_parameter('encoder_cpr').get_parameter_value().double_value
        self.motor_max_rpm = self.get_parameter('motor_max_rpm').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value
        self.serial_timer = self.get_parameter('serial_timer').get_parameter_value().double_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.csv_filename = self.get_parameter('csv_filename').get_parameter_value().string_value
        self.output_directory = self.get_parameter('output_directory').get_parameter_value().string_value

        # Calculate circumference
        self.circumference = 2 * math.pi * self.wheel_radius
        self.prev_left_count = 0
        self.prev_right_count = 0

        # declare experiments variables
        self.target_linear_vel = 0.0
        self.gyro_angular_vel = 0.0
        self.slip_ratio_l = 0.0
        self.slip_ratio_r = 0.0
        
        # Initialize CSV file for data logging
        self.initialize_csv_file()

        # Connect serial port
        try:
            self.serial_port = serial.Serial(
                self.serial_port_name, 
                self.serial_baudrate, 
                timeout=self.serial_timeout
            )
            self.get_logger().info(f"Successfully Connected to ESP32 on {self.serial_port_name}.")
            
            # Send reset message to ESP32
            reset_command = "RESET\n"
            self.serial_port.write(reset_command.encode('utf-8'))
            self.get_logger().info("Reset message sent to ESP32.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to ESP32: {str(e)}")
            raise e
        
        # Create publisher, subscriber and timer
        self.cmd_vel_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.timer = self.create_timer(self.serial_timer, self.read_serial_and_publish)

    def cmd_vel_callback(self, msg):
        self.target_linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # calculate velocity (m/s)
        left_speed = self.target_linear_vel - (angular_vel * self.wheel_base / 2)
        right_speed = self.target_linear_vel + (angular_vel * self.wheel_base / 2)
        
        # velocity to rpm
        left_rpm = self.constrain((left_speed / self.circumference) * 60, -self.motor_max_rpm, self.motor_max_rpm)
        right_rpm = self.constrain((right_speed / self.circumference) * 60, -self.motor_max_rpm, self.motor_max_rpm)
        
        # send data to ESP32
        # string format: "L100,R200\n" (Left: 100RPM, Right: 200RPM)
        command = f"L{left_rpm:.2f},R{right_rpm:.2f}\n"
        try:
            self.serial_port.write(command.encode('utf-8'))
            self.get_logger().debug(f"Sent to ESP32: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command to ESP32: {str(e)}")

    def imu_callback(self, msg):
        self.gyro_angular_vel = msg.angular_velocity.z

    def read_serial_and_publish(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            try:
                parts = line.split(',')
                left_encoder_count = int(parts[0])
                right_encoder_count = int(parts[1])
                pwm_l = float(parts[2])
                pwm_r = float(parts[3])
                input_rpm_l = float(parts[4])
                input_rpm_r = float(parts[5])
                output_rpm_l = float(parts[6])
                output_rpm_r = float(parts[7])
                # self.get_logger().info(f"Left: {left_encoder_count}, Right: {right_encoder_count}, PWM Left: {pwm_l}, PWM Right: {pwm_r},\
                #                         Input RPM Left: {input_rpm_l}, Input RPM Right: {input_rpm_r}, Output RPM Left: {output_rpm_l}, \
                #                         Output RPM Right: {output_rpm_r}")

                # save data only if input and output RPM are close (Ensure result is not affected by unstable speed)
                if input_rpm_l - output_rpm_l < 5 and input_rpm_r - output_rpm_r < 5:
                    
                    # distance change of left and right wheels
                    d_left = (left_encoder_count - self.prev_left_count) / self.counts_per_rev * self.circumference
                    d_right = (right_encoder_count - self.prev_right_count) / self.counts_per_rev * self.circumference

                    esp32_timer = 0.05  # second
                    vel_l = (d_left / self.serial_timer) / esp32_timer
                    vel_r = (d_right / self.serial_timer) / esp32_timer

                    # calculate slip ratio
                    if abs(vel_l) < abs(vel_r):
                        self.slip_ratio_r = 1 + (self.target_linear_vel + self.wheel_base / 2) / vel_r if vel_r != 0 else 0
                        self.slip_ratio_l = 1 - (self.target_linear_vel - self.wheel_base / 2) / vel_l if vel_l != 0 else 0
                    else:
                        self.slip_ratio_r = 1 - (self.target_linear_vel + self.wheel_base / 2) / vel_r if vel_r != 0 else 0
                        self.slip_ratio_l = 1 + (self.target_linear_vel - self.wheel_base / 2) / vel_l if vel_l != 0 else 0

                    # save data to csv
                    self.save_data_to_csv(self.slip_ratio_l, self.slip_ratio_r, vel_l, vel_r)
                    self.get_logger().info(f"Slip Ratio L: {self.slip_ratio_l}, R: {self.slip_ratio_r}, Vel L: {vel_l}, R: {vel_r}")

                # update encoder counts
                self.prev_left_count = left_encoder_count
                self.prev_right_count = right_encoder_count
            
            except ValueError:
                self.get_logger().warn(f"Receiving Invalid data format: {line}")
    
    def initialize_csv_file(self):
        """Initialize CSV file with headers"""
        # Create output directory if it doesn't exist
        os.makedirs(self.output_directory, exist_ok=True)
        
        # Create filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filepath = os.path.join(self.output_directory, f"{timestamp}_{self.csv_filename}")
        
        # Create CSV file and write headers
        try:
            with open(self.csv_filepath, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'slip_ratio_left', 'slip_ratio_right', 'velocity_left', 'velocity_right', 
                            'target_linear_vel', 'gyro_angular_vel']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
            self.get_logger().info(f"CSV file initialized: {self.csv_filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CSV file: {str(e)}")
    
    def save_data_to_csv(self, slip_ratio_l, slip_ratio_r, vel_l, vel_r):
        """Save slip ratio and velocity data to CSV file"""
        try:
            with open(self.csv_filepath, 'a', newline='') as csvfile:
                fieldnames = ['timestamp', 'slip_ratio_left', 'slip_ratio_right', 'velocity_left', 'velocity_right', 
                            'target_linear_vel', 'gyro_angular_vel']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write data row
                writer.writerow({
                    'timestamp': datetime.now().isoformat(),
                    'slip_ratio_left': slip_ratio_l,
                    'slip_ratio_right': slip_ratio_r,
                    'velocity_left': vel_l,
                    'velocity_right': vel_r,
                    'target_linear_vel': self.target_linear_vel,
                    'gyro_angular_vel': self.gyro_angular_vel
                })
        except Exception as e:
            self.get_logger().error(f"Failed to save data to CSV: {str(e)}")

    def constrain(self, val, min_val, max_val):
        """Constrain value between min and max"""
        return max(min_val, min(val, max_val))

    def destroy_node(self):
        """Clean up resources when shutting down"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main():
    rclpy.init()
    node = Esp32SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
