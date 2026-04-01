import sys
import struct
import usb.core
import usb.util
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Twist
import math

# name, resid, cmdid, length, type
PARAMETERS = {
    "VERSION": (48, 0, 4, "ro", "uint8"),
    "AEC_AZIMUTH_VALUES": (33, 75, 16 + 1, "ro", "radians"),
    "DOA_VALUE": (20, 18, 4 + 1, "ro", "uint16"),
}

class Microphone(Node):
    def __init__(self):
        super().__init__('microphone')

        self.mic_ang_pub = self.create_publisher(Twist, '/microphone/speec_angle', 10)

    def find_mic(self);
        


def main():
    print('Hi from microphone.')


if __name__ == '__main__':
    main()
