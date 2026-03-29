"""
robot_head_controller.py
Sends servo angle commands and receives telemetry (distance + pitch) from Arduino.

Packet formats:
  Command  (host → Arduino):  [0xAA, 0x01, angle_byte, xor_checksum]
  Telemetry (Arduino → host): [0xBB, dist_hi, dist_lo, pitch_hi, pitch_lo, xor_checksum]
"""

import serial
import struct
import time
import threading
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import Range

# ── Config ────────────────────────────────────────────────────────────────────
PORT      = "/dev/servo"      
BAUD      = 9600
CMD_HEADER = 0xAA
CMD_TYPE   = 0x01
TEL_HEADER = 0xBB
TEL_LEN    = 6          # bytes per telemetry packet

# Servo range matching the Arduino constants
SERVO_MIN    = 130
SERVO_MAX    = 220
SERVO_CENTER = 154

# Pitch angle limits corresponding to servo limits
PITCH_MIN    = -34   # corresponds to SERVO_MIN 130
PITCH_MAX    =  96   # corresponds to SERVO_MAX 220
PITCH_CENTER =   0   # corresponds to SERVO_CENTER 154

class Head(Node):
    '''Node for moving R2's head'''
    def __init__(self,pub_rate: int = PUB_RATE):
        super().__init__('Head')
      
        self.head_dist_publisher = self.create_publisher(
            Range,
            'head/dist_obj',
            10)

        self.head_pos_publisher = self.create_publisher(
            Float64,
            'head/actual_pos',
            10)

        self.head_move_subscriber = self.create_subscription(
            Float64,
            'head/desired_pos',
            self.move_head_callback,
            10)
        
        self.head_speed_subscriber = self.create_subscription(
            Float32,
            'head/desired_spd',
            self.move_head_speed_callback,
            10)
        
        self.get_logger().info("Head Motion control starting up.")
        self.ser = serial.Serial(PORT, BAUD, timeout=1)
        self.ser.reset_input_buffer()
        self.dist  = None
        self.pitch = None
        self.ser_buf  = bytearray() 

        self.get_logger().debug("Listening for Head telemetry.")
        self.reader = threading.Thread(target= self.telemetry_reader,daemon=True)
        self.reader.start()       
        
        self.head_pos_timer = self.create_timer(pub_rate, self.pub_head_pos)
        self.head_dis_timer = self.create_timer(pub_rate, self.pub_head_dis)

        self.move_speed = None


    def telemetry_reader(self):
        """Reads telemetry from distance sensor and mpu5060 IMU
        in the background"""
        while True:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            self.ser_buf.extend(chunk)

            # Scan for 0xBB header and extract complete packets
            while True:
                idx = self.ser_buf.find(TEL_HEADER)
                if idx == -1:
                    self.ser_buf.clear()
                    break
                if idx > 0:
                    del self.ser_buf[:idx]   # discard bytes before header

                if len(self.ser_buf) < TEL_LEN:
                    break                 # wait for more data

                pkt = bytes(self.ser_buf[:TEL_LEN])
                del self.ser_buf[:TEL_LEN]

                result = parse_telemetry(pkt)
                if result:
                    self.dist, self.pitch = result
    

    def pub_head_pos(self,):
        pass


    def pub_head_dis(self,):
        pass
     

    def pitch_to_servo(self, pitch_angle: float) -> int:
        """Map a real pitch angle to a servo value, clamped to safe range."""
        servo = np.interp(pitch_angle, [PITCH_MIN, PITCH_MAX], [SERVO_MIN, SERVO_MAX])
        return int(round(servo))


    # ── Checksum ──────────────────────────────────────────────────────────────────
    def xor_checksum(self, data: bytes) -> int:
        cs = 0
        for b in data:
            cs ^= b
        return cs


    # ── Command builder ───────────────────────────────────────────────────────────
    def build_servo_command(self, angle: int) -> bytes:
        if angle < SERVO_MIN:
            angle = SERVO_MIN
        elif angle > SERVO_MAX:
            angle = SERVO_MAX
        payload = bytes([CMD_HEADER, CMD_TYPE, angle])
        cs = xor_checksum(payload)
        return payload + bytes([cs])


    # ── Telemetry parser ──────────────────────────────────────────────────────────
    def parse_telemetry(self, pkt: bytes):
        """
        Returns (distance_m, pitch_deg) or None if checksum fails.
        pkt must be 6 bytes starting with 0xBB.
        """
        if len(pkt) != TEL_LEN or pkt[0] != TEL_HEADER:
            return None
        expected_cs = xor_checksum(pkt[:5])
        if pkt[5] != expected_cs:
            self.get_logger().warn(f"Bad telemetry checksum (got {pkt[5]:02X}, 
                                   expected {expected_cs:02X})")
            return None
        dist_mm   = struct.unpack(">H", pkt[1:3])[0]   # unsigned 16-bit
        pitch_raw = struct.unpack(">h", pkt[3:5])[0]   # signed 16-bit
        dist_m    = dist_mm / 1000.0
        pitch_deg = pitch_raw / 10.0
        return dist_m, pitch_deg


# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    head = Head()
    try:
        rclpy.spin(head)
    finally:
        head.destroy_node()
        rclpy.shutdown()
   
    while True:
        user_in = input("Pitch angle > ").strip().lower()

        if user_in == 'q':
            print("Exiting.")
            break

        if user_in == 'c':
            pitch_target = PITCH_CENTER
        else:
            try:
                pitch_target = float(user_in)   # float so e.g. 12.5 works
            except ValueError as e:
                print(f"  Parse error: {e!r} on input {user_in!r}")
                continue

        # Clamp pitch to valid range before converting
        pitch_target = max(PITCH_MIN, min(PITCH_MAX, pitch_target))
        angle = pitch_to_servo(pitch_target)

        cmd = build_servo_command(angle)
        ser.write(cmd)
        ser.flush()
        print(f"  → Pitch {pitch_target}° → servo value {angle}  (packet: {cmd.hex(' ').upper()})")

        time.sleep(0.3)
        if reader.dist is not None:
            print(f"  ← Distance: {reader.dist:.3f} m  |  Pitch: {reader.pitch:.1f}°")
        else:
            print("  ← Waiting for telemetry…")


if __name__ == "__main__":
    main()

    def main(args=None):
    rclpy.init(args=args)
    node = RobotHeadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()