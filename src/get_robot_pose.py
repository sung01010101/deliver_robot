#!/usr/bin/env python3
"""
Robot pose publisher node.

Publishes the robot pose (in the map frame) to /robot_pose as a geometry_msgs/PoseStamped.
Falls back to odom->base_footprint if map->odom is not available.
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import tf_transformations
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_footprint'

class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')
        self.declare_parameter('map_frame', MAP_FRAME)
        self.declare_parameter('odom_frame', ODOM_FRAME)
        self.declare_parameter('base_frame', BASE_FRAME)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value or MAP_FRAME
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value or ODOM_FRAME
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value or BASE_FRAME
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value or 10.0

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.pub = self.create_publisher(PoseStamped, 'robot_pose', 10)
        timer_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Publishing robot pose at {self.publish_rate} Hz")

    def lookup_pose(self) -> Optional[PoseStamped]:
        # Try map -> base first
        try:
            tf_map_base = self.buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return self.transform_to_pose(tf_map_base, self.map_frame)
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Try chain map->odom and odom->base if direct unavailable
            try:
                tf_map_odom = self.buffer.lookup_transform(self.map_frame, self.odom_frame, rclpy.time.Time())
                tf_odom_base = self.buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
                # Compose the transforms manually
                pose_map_odom = self.transform_to_pose(tf_map_odom, self.map_frame)
                pose_odom_base = self.transform_to_pose(tf_odom_base, self.odom_frame)
                # Convert to homogeneous matrices
                mat_map_odom = pose_to_matrix(pose_map_odom.pose)
                mat_odom_base = pose_to_matrix(pose_odom_base.pose)
                mat_map_base = mat_map_odom @ mat_odom_base
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = self.map_frame
                # Extract translation
                pose.pose.position.x = mat_map_base[0, 3]
                pose.pose.position.y = mat_map_base[1, 3]
                pose.pose.position.z = mat_map_base[2, 3]
                # Extract quaternion
                quat = tf_transformations.quaternion_from_matrix(mat_map_base)
                pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                return pose
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warn(f"TF lookup failed: {e}")
                return None

    def transform_to_pose(self, transform, frame_id: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = frame_id
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def timer_callback(self):
        pose = self.lookup_pose()
        if pose:
            # Normalize quaternion (safety)
            q = pose.pose.orientation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if norm > 0.0:
                q.x /= norm; q.y /= norm; q.z /= norm; q.w /= norm
            self.pub.publish(pose)


def pose_to_matrix(pose) -> "list[list[float]]":
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    t = (pose.position.x, pose.position.y, pose.position.z)
    mat = tf_transformations.quaternion_matrix(q)
    mat[0, 3] = t[0]
    mat[1, 3] = t[1]
    mat[2, 3] = t[2]
    return mat


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
