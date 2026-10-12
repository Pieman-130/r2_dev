#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


WHEEL_RADIUS = 0.085       # meters
WHEEL_BASE = 0.250        # meters


class R2Odometry(Node):

    def __init__(self):
        super().__init__("r2_odometry")

        self.left_position = None
        self.right_position = None

        self.left_velocity = 0.0
        self.right_velocity = 0.0

        self.previous_left_position = None
        self.previous_right_position = None

        # Robot pose in odom frame
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_update_time = None

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.create_subscription(
            Float64,
            "/wheel/left_position",
            self.left_position_callback,
            10
        )

        self.create_subscription(
            Float64,
            "/wheel/right_position",
            self.right_position_callback,
            10
        )

        self.create_subscription(
            Float64,
            "/wheel/left_velocity",
            self.left_velocity_callback,
            10
        )

        self.create_subscription(
            Float64,
            "/wheel/right_velocity",
            self.right_velocity_callback,
            10
        )

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        # ---------------------------------------------------------
        # TF broadcaster
        # ---------------------------------------------------------

        self.tf_broadcaster = TransformBroadcaster(self)

        # Update odometry at 50 Hz.
        self.timer = self.create_timer(
            0.02,
            self.update_odometry
        )

        self.get_logger().info(
            "R2 differential-drive odometry started"
        )

        self.get_logger().info(
            f"Wheel radius: {WHEEL_RADIUS:.3f} m"
        )

        self.get_logger().info(
            f"Wheel separation: {WHEEL_BASE:.3f} m"
        )

    # -------------------------------------------------------------
    # Wheel callbacks
    # -------------------------------------------------------------

    def left_position_callback(self, msg):
        self.left_position = msg.data

    def right_position_callback(self, msg):
        self.right_position = msg.data

    def left_velocity_callback(self, msg):
        self.left_velocity = msg.data

    def right_velocity_callback(self, msg):
        self.right_velocity = msg.data

    # -------------------------------------------------------------
    # Odometry update
    # -------------------------------------------------------------

    def update_odometry(self):

        # We need both wheel positions before
        # we can calculate anything.
        if self.left_position is None:
            return

        if self.right_position is None:
            return

        now = self.get_clock().now()

        # ---------------------------------------------------------
        # First measurement establishes the starting point.
        # ---------------------------------------------------------

        if self.previous_left_position is None:

            self.previous_left_position = self.left_position
            self.previous_right_position = self.right_position
            self.last_update_time = now

            return

        # ---------------------------------------------------------
        # Wheel angular displacement
        # ---------------------------------------------------------

        delta_left_angle = (
            self.left_position -
            self.previous_left_position
        )

        delta_right_angle = (
            self.right_position -
            self.previous_right_position
        )

        self.previous_left_position = self.left_position
        self.previous_right_position = self.right_position

        # ---------------------------------------------------------
        # Convert wheel rotation to linear distance
        # ---------------------------------------------------------

        delta_left_distance = (
            WHEEL_RADIUS * delta_left_angle
        )

        delta_right_distance = (
            WHEEL_RADIUS * delta_right_angle
        )

        # ---------------------------------------------------------
        # Differential-drive kinematics
        # ---------------------------------------------------------

        delta_distance = (
            delta_left_distance +
            delta_right_distance
        ) / 2.0

        delta_yaw = (
            delta_right_distance -
            delta_left_distance
        ) / WHEEL_BASE

        # Midpoint heading gives better integration during turns.
        midpoint_yaw = self.yaw + delta_yaw / 2.0

        self.x += delta_distance * math.cos(midpoint_yaw)
        self.y += delta_distance * math.sin(midpoint_yaw)
        self.yaw += delta_yaw

        # Keep yaw bounded to -pi ... +pi.
        self.yaw = math.atan2(
            math.sin(self.yaw),
            math.cos(self.yaw)
        )

        # ---------------------------------------------------------
        # Publish
        # ---------------------------------------------------------

        self.publish_odometry(now)

    # -------------------------------------------------------------
    # Publish odometry + TF
    # -------------------------------------------------------------

    def publish_odometry(self, now):

        stamp = now.to_msg()

        # ---------------------------------------------------------
        # Odometry message
        # ---------------------------------------------------------

        odom = Odometry()

        odom.header.stamp = stamp
        odom.header.frame_id = "odom"

        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Yaw -> quaternion
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Differential-drive velocity
        linear_velocity = (
            WHEEL_RADIUS *
            (self.right_velocity + self.left_velocity) /
            2.0
        )

        angular_velocity = (
            WHEEL_RADIUS *
            (self.right_velocity - self.left_velocity) /
            WHEEL_BASE
        )

        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

        # ---------------------------------------------------------
        # TF: odom -> base_link
        # ---------------------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):

    rclpy.init(args=args)

    node = R2Odometry()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()