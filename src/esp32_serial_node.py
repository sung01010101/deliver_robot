#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math


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
        self.declare_parameter('timer_period', 0.01)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ekf_imu', True)
        
        # Get parameters
        self.counts_per_rev = self.get_parameter('encoder_cpr').get_parameter_value().double_value
        self.motor_max_rpm = self.get_parameter('motor_max_rpm').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.ekf_imu = self.get_parameter('ekf_imu').get_parameter_value().bool_value
        
        # Calculate circumference
        self.circumference = 2 * math.pi * self.wheel_radius
        
        # Log parameters
        self.get_logger().info(f"ESP32 Serial Node Parameters:")
        self.get_logger().info(f"  Encoder CPR: {self.counts_per_rev}")
        self.get_logger().info(f"  Motor Max RPM: {self.motor_max_rpm}")
        self.get_logger().info(f"  Wheel Base: {self.wheel_base} m")
        self.get_logger().info(f"  Wheel Radius: {self.wheel_radius} m")
        self.get_logger().info(f"  Serial Port: {self.serial_port_name}")
        self.get_logger().info(f"  Serial Baudrate: {self.serial_baudrate}")
        self.get_logger().info(f"  Use EKF: {self.ekf_imu}")
        
        # Initialize odometry data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left = 0
        self.prev_right = 0
        
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
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.timer = self.create_timer(self.timer_period, self.read_serial_and_publish)

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # calculate velocity (m/s)
        left_speed = linear_vel - (angular_vel * self.wheel_base / 2)
        right_speed = linear_vel + (angular_vel * self.wheel_base / 2)
        
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

    def read_serial_and_publish(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            try:
                parts = line.split(',')
                left = int(parts[0])
                right = int(parts[1])

                # debugging outputs
                pwm_l = float(parts[2])
                pwm_r = float(parts[3])
                input_rpm_l = float(parts[4])
                input_rpm_r = float(parts[5])
                output_rpm_l = float(parts[6])
                output_rpm_r = float(parts[7])
                # self.get_logger().info(f"Left: {left_encoder_count}, Right: {right_encoder_count}, PWM Left: {pwm_l}, PWM Right: {pwm_r},\
                #                         Input RPM Left: {input_rpm_l}, Input RPM Right: {input_rpm_r}, Output RPM Left: {output_rpm_l}, \
                #                         Output RPM Right: {output_rpm_r}")
                
                # calculate delta change
                d_left = (left - self.prev_left) / self.counts_per_rev * self.circumference
                d_right = (right - self.prev_right) / self.counts_per_rev * self.circumference

                self.prev_left, self.prev_right = left, right
                
                # calculate odometry values
                d_center = (d_left + d_right) / 2
                d_theta = (d_right - d_left) / self.wheel_base
                
                self.theta += d_theta
                self.x += d_center * math.cos(self.theta)
                self.y += d_center * math.sin(self.theta)
                
                # publish tf and odometry
                self.publish_odom()
                if not self.ekf_imu:
                    self.publish_tf()
                
            except ValueError:
                self.get_logger().warn(f"Receiving Invalid data: {line}")

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convert to quaternion
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2)
        t.transform.rotation.w = math.cos(self.theta / 2)
        
        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert to quaternion
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        self.odom_pub.publish(odom)

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
