import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.base_publisher_ = self.create_publisher(Twist, '/r2_move/base', 10)
        self.head_publisher_ = self.create_publisher(Twist, '/r2_move/head', 10)


        

    def get_joy_input(self):


    def run_teleop(self):
        while rclpy.ok():
            twist_msg = Twist()

            self.base_publisher_.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    teleop_node.run_teleop()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
