#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion
import serial
import math


class Esp32ScogNode(Node):
    def __init__(self):
        super().__init__('esp32_scog_node')
        
        # Declare parameters with default values
        self.declare_parameter('encoder_cpr', 700.0)
        self.declare_parameter('motor_max_rpm', 330)
        self.declare_parameter('wheel_base', 0.465)
        self.declare_parameter('wheel_radius', 0.02)
        self.declare_parameter('base_slip_ratio', 0.0)
        self.declare_parameter('param_n', 0.5)
        self.declare_parameter('serial_port', '/dev/esp32')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('serial_timeout', 1.0)
        self.declare_parameter('serial_timer', 0.1)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        
        # Get parameters
        self.counts_per_rev = self.get_parameter('encoder_cpr').get_parameter_value().double_value
        self.motor_max_rpm = self.get_parameter('motor_max_rpm').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.base_slip_ratio = self.get_parameter('base_slip_ratio').get_parameter_value().double_value
        self.param_n = self.get_parameter('param_n').get_parameter_value().double_value
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value
        self.serial_timer = self.get_parameter('serial_timer').get_parameter_value().double_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        
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
        self.get_logger().info(f"  ----------------------------------------------------")
        self.get_logger().info(f"  Base Slip Ratio: {self.base_slip_ratio}")
        self.get_logger().info(f"  IMU Topic: {self.imu_topic}")
        self.get_logger().info(f"  Param N: {self.param_n}")
        
        # Initialize odometry data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left = 0
        self.prev_right = 0
        
        # Initialize IMU data
        self.imu_z = 0.0
        self.imu_w = 1.0
        self.imu_theta = 0.0
        self.accumulated_imu_theta = 0.0

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
        
        # Create IMU subscriber
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.get_logger().info(f"IMU subscriber created on topic: {self.imu_topic}")
        
        # Publisher for RPM data
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/rpm_data', 10)
        self.scog_pub = self.create_publisher(Float32MultiArray, '/scog_data', 10)
        
        # Timer for reading serial data
        self.serial_timer = self.create_timer(self.serial_timer, self.read_serial_and_publish)

        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # Publisher for tf transforms
        self.tf_broadcaster = TransformBroadcaster(self)

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

    def imu_callback(self, msg):
        """Callback to receive IMU orientation data"""
        # Store quaternion components directly
        x = msg.orientation.x
        y = msg.orientation.y
        self.imu_z = msg.orientation.z
        self.imu_w = msg.orientation.w
        
        # Store previous theta before updating
        prev_theta = self.imu_theta
        _, _, self.imu_theta = euler_from_quaternion([x, y, self.imu_z, self.imu_w])
        
        # Accumulate theta change for synchronization with encoder data
        d_theta = self.imu_theta - prev_theta
        self.accumulated_imu_theta += d_theta

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
                self.get_logger().info(f"Encoder: {left}, {right}, PWM: {pwm_l}, {pwm_r}, "
                                       f"Input RPM: {input_rpm_l}, {input_rpm_r}, "
                                       f"Output RPM: {output_rpm_l}, {output_rpm_r}")

                rpm_msg = Float32MultiArray()
                rpm_msg.data = [pwm_l, pwm_r, input_rpm_l, input_rpm_r, output_rpm_l, output_rpm_r]
                self.rpm_pub.publish(rpm_msg)
                
                # delta change of encoders and imu
                d_left = (left - self.prev_left) / self.counts_per_rev * self.circumference
                d_right = (right - self.prev_right) / self.counts_per_rev * self.circumference
                d_odom_theta = (d_right - d_left) / self.wheel_base
                
                # Use accumulated IMU theta change and reset accumulator
                d_imu_theta = self.accumulated_imu_theta
                self.accumulated_imu_theta = 0.0

                # formula 7: 計算 slip ratios 的比值
                if d_left == 0 or d_right == 0:
                    slip_ratio_ratio = 0.0
                else:
                    slip_ratio_ratio = -1 * self.signum(d_left * d_right) * (abs(d_right / d_left) ** self.param_n)
                
                # formula 3 + 7 合併後的公式, 解右輪 slip ratio
                denominator = (slip_ratio_ratio * d_left - d_right)

                if abs(denominator) < 0.001 or d_imu_theta < 0.005:
                    # 直行或數值不穩定時，假設小 slip（等量）
                    slip_r = self.base_slip_ratio
                    slip_l = self.base_slip_ratio
                else:
                    slip_r = (2 * self.wheel_base/2 * d_imu_theta + d_left - d_right) / denominator
                    slip_l = slip_ratio_ratio * slip_r

                # 限制 slip ratio 範圍
                slip_r = max(min(slip_r, 0.5), -0.5)
                slip_l = max(min(slip_l, 0.5), -0.5)

                # slip 修正後的 travel
                d_left_corr = d_left * (1 - slip_l)
                d_right_corr = d_right * (1 - slip_r)
                
                # update odometry
                d_center = (d_left_corr + d_right_corr) / 2.0
                d_theta = (d_right_corr - d_left_corr) / self.wheel_base

                self.theta += d_theta
                # self.theta = self.imu_theta
                self.x += d_center * math.cos(self.theta)
                self.y += d_center * math.sin(self.theta)
               
                # debugging outputs
                self.get_logger().info(f"Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}, ")
                
                scog_msg = Float32MultiArray()
                scog_msg.data = [d_left, d_right, d_odom_theta, d_imu_theta,
                                 slip_ratio_ratio, denominator, slip_l, slip_r,
                                 d_left_corr, d_right_corr, d_theta,
                                 self.x, self.y, self.theta]
                self.scog_pub.publish(scog_msg)

                # publish odometry
                self.publish_odom()
                self.publish_tf()

                # update previous values
                self.prev_left = left
                self.prev_right = right

            except ValueError:
                self.get_logger().warn(f"Receiving Invalid data: {line}")

    def signum(self, value):
        """
        Returns 1 for positive, -1 for negative, and 0 for zero.
        """
        return (value > 0) - (value < 0)

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
    node = Esp32ScogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
