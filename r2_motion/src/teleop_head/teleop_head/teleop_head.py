import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
import math
import numpy as np

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (rad) to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class TeleopHeadBridge(Node):
    def __init__(self):
        super().__init__('teleop_head_bridge')

        # current orientation (Euler)
        self.yaw = 0.0    # left-right
        self.pitch = 0.0  # up-down
        self.roll = 0.0   # probably not used

        self.last_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Twist,
            '/r2_move/teleop_head',
            self.twist_callback,
            10
        )

        self.publisher = self.create_publisher(
            Quaternion,
            '/r2_move/head_position',
            10
        )

        # timer to update orientation
        self.timer = self.create_timer(0.02, self.update_orientation)  # 50 Hz

        self.current_twist = Twist()

    def twist_callback(self, msg):
        self.current_twist = msg

    def update_orientation(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # integrate angular velocities
        #self.roll  += self.current_twist.angular.x * dt
        self.pitch += self.current_twist.angular.y * dt
        self.yaw   += self.current_twist.angular.z * dt

        # optional: clamp pitch to prevent flipping
        self.pitch = max(-math.pi/2, min(math.pi/2, self.pitch))

        # convert to quaternion
        q = euler_to_quaternion(self.roll, self.pitch, self.yaw)

        # publish
        self.publisher.publish(q)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopHeadBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
