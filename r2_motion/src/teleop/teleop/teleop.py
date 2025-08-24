import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys
import math

#TODO Setup max speed as a ros param
MAX_SPEED = 2.2 #m/s
MAX_ANGULAR_VELOCITY = 19.56 #rad/s

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.base_publisher_ = self.create_publisher(Twist, '/r2_move/base', 10)
        self.head_publisher_ = self.create_publisher(Twist, '/r2_move/head', 10)

        #setup and look for joystick
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No Joystick Detected!")
            sys.exit()

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

    def remove_deadspace(self,value):
        if value >= 0.05 or value <= -0.05:
            return value
        else:
            return 0.0


    def normalize(self,value1,value2):
        magnitude = math.hypot(value1, value2)
        if manitude > 1.0:
            norm1 /= magnitude
            norm2 /= mangitude
            return (norm1,norm2)
        else:
            return (value1,value2)


    def get_joy_input(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                #deadman switch: both L2 and R2 need to be held to send teleop cmds
                if self.joy.get_button(6) and self.joy.get_button(7):
                    lin_base = self.remove_deadspace(self.joy.get_axis(0))
                    ang_base = self.remove_deadspace(self.joy.get_axis(1))
                    lin_head = self.remove_deadspace(self.joy.get_axis(3))
                    ang_head = self.remove_deadspace(self.joy.get_axis(4))

                    #normallize
                    lin_base, ang_base = self.normalize(lin_base,ang_base)
                    lin_head, ang_head = self.normalize(lin_head,ang_head)
                else:
                    lin_base = 0.0
                    ang_base = 0.0
                    lin_head = 0.0
                    ang_head = 0.0
                
                return lin_base,ang_base,lin_head,ang_head
                    
        
    def run_teleop(self):
        while rclpy.ok():
            joy_state = self.get_joy_input()

            base_msg = Twist()
            base_msg.x = joy_state[0]*MAX_SPEED
            base_msg.z = joy_state[1]*MAX_ANGULAR_VELOCITY
            self.base_publisher_.publish(base_msg)

            head_msg = Twist()
            head_msg.x = joy_state[2]*

            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopNode()
    teleop_node.run_teleop()
    teleop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
