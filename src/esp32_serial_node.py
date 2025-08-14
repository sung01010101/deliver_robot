#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
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
        self.declare_parameter('odom_timer', 0.01)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('use_imu', False)
        
        # Get parameters
        self.counts_per_rev = self.get_parameter('encoder_cpr').get_parameter_value().double_value
        self.motor_max_rpm = self.get_parameter('motor_max_rpm').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.serial_port_name = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        self.serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value
        self.odom_timer = self.get_parameter('odom_timer').get_parameter_value().double_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.use_imu = self.get_parameter('use_imu').get_parameter_value().bool_value
        
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
        self.get_logger().info(f"  IMU Topic: {self.imu_topic}")
        self.get_logger().info(f"  Use Imu for Rotation: {self.use_imu}")
        
        # Initialize odometry data
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left = 0
        self.prev_right = 0
        
        # Initialize IMU data
        self.imu_quat_z = 0.0
        self.imu_quat_w = 1.0
        self.imu_available = False
        
        # Initialize velocity data
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.prev_time = self.get_clock().now()
        
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
        
        # Create IMU subscriber if use_imu is True
        if self.use_imu:
            self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
            self.get_logger().info(f"IMU subscriber created on topic: {self.imu_topic}")
        
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        
        # Timer for reading serial data
        self.serial_timer = self.create_timer(0.01, self.read_serial_and_publish)  # 100Hz for serial reading

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
        self.imu_quat_z = msg.orientation.z
        self.imu_quat_w = msg.orientation.w
        
        if not self.imu_available:
            self.get_logger().info("IMU data is now available for odometry calculation")
        self.imu_available = True

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
                self.get_logger().info(f"Left: {left}, Right: {right}, PWM Left: {pwm_l}, PWM Right: {pwm_r},\
                                        Input RPM: {input_rpm_l}, {input_rpm_r}, Output RPM: {output_rpm_l}, {output_rpm_r}")

                # Get current time for velocity calculation
                current_time = self.get_clock().now()
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                
                # calculate delta change
                d_left = (left - self.prev_left) / self.counts_per_rev * self.circumference
                d_right = (right - self.prev_right) / self.counts_per_rev * self.circumference

                self.prev_left, self.prev_right = left, right
                
                # calculate odometry values
                d_center = (d_left + d_right) / 2
                d_theta = (d_right - d_left) / self.wheel_base
                
                # Update position and rotation
                if self.use_imu and self.imu_available:
                    # Use IMU for rotation, but still calculate encoder-based angular velocity for velocity estimation
                    encoder_d_theta = d_theta
                    # Convert IMU quaternion to theta for position calculation
                    imu_theta = 2 * math.atan2(self.imu_quat_z, self.imu_quat_w)
                    self.theta = imu_theta
                    
                    # For velocity calculation, use encoder-based angular velocity
                    if dt > 0:
                        self.vtheta = encoder_d_theta / dt
                else:
                    # Use encoder-based rotation (original behavior)
                    self.theta += d_theta
                    if dt > 0:
                        self.vtheta = d_theta / dt
                
                # Update linear position (always uses encoder data)
                self.x += d_center * math.cos(self.theta)
                self.y += d_center * math.sin(self.theta)
                
                # Calculate linear velocities
                if dt > 0:
                    self.vx = d_center / dt
                    self.vy = 0.0  # Differential drive robot doesn't move sideways
                
                # Update time
                self.prev_time = current_time
                
                # publish tf and odometry
                self.publish_odom()
                
                # Always publish TF transform (use IMU data when available and use_imu=True)
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
        
        # Use IMU quaternion directly if available and use_imu is true
        if self.use_imu and self.imu_available:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = self.imu_quat_z
            t.transform.rotation.w = self.imu_quat_w
        else:
            # Convert encoder-based theta to quaternion
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
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation - use IMU quaternion directly if available and use_imu is true
        if self.use_imu and self.imu_available:
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = self.imu_quat_z
            odom.pose.pose.orientation.w = self.imu_quat_w
        else:
            # Convert encoder-based theta to quaternion
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
