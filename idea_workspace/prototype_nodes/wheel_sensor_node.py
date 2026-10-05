#!/usr/bin/env python3

import struct
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32


PORT = "/dev/SENS0"
BAUD = 115200

WHEEL_TRANSITIONS_PER_REV = 90.0


class WheelSensorNode(Node):

    def __init__(self):
        super().__init__("wheel_sensor")

        # ROS topics
        self.right_rpm_pub = self.create_publisher(
            Float32, "/wheel/right_rpm", 10
        )

        self.left_rpm_pub = self.create_publisher(
            Float32, "/wheel/left_rpm", 10
        )

        self.right_encoder_pub = self.create_publisher(
            Int32, "/wheel/right_encoder", 10
        )

        self.left_encoder_pub = self.create_publisher(
            Int32, "/wheel/left_encoder", 10
        )

        # Open sensor serial port
        try:
            self.ser = serial.Serial(
                PORT,
                BAUD,
                timeout=0.1
            )

            self.get_logger().info(
                f"Opened sensor port {PORT} at {BAUD} baud"
            )

        except Exception as e:
            self.get_logger().error(
                f"Could not open {PORT}: {e}"
            )
            raise

        self.buffer = bytearray()

        self.wheel_packets = 0
        self.environment_packets = 0
        self.bad_packets = 0

        self.last_report = time.monotonic()

        # Check the serial stream frequently.
        self.timer = self.create_timer(
            0.001,
            self.read_serial
        )

    def read_serial(self):

        try:
            data = self.ser.read(self.ser.in_waiting or 1)

            if data:
                self.buffer.extend(data)

            self.parse_buffer()

        except Exception as e:
            self.get_logger().error(
                f"Serial read error: {e}"
            )

    def parse_buffer(self):

        while True:

            # Need at least two bytes for a preamble.
            if len(self.buffer) < 2:
                return

            # Find AA 55 or AA 56
            start = -1

            for i in range(len(self.buffer) - 1):

                if self.buffer[i] == 0xAA and self.buffer[i + 1] in (0x55, 0x56):
                    start = i
                    break

            if start < 0:

                # Keep final AA in case it is beginning a packet.
                if self.buffer[-1] == 0xAA:
                    self.buffer = self.buffer[-1:]
                else:
                    self.buffer.clear()

                return

            if start > 0:
                del self.buffer[:start]

            # ---------------------------------------------------------
            # Environmental packet
            #
            # AA 55
            # 28-byte payload
            # checksum
            # 55 AA
            #
            # Total = 33 bytes
            # ---------------------------------------------------------

            if self.buffer[1] == 0x55:

                packet_length = 33

                if len(self.buffer) < packet_length:
                    return

                packet = bytes(self.buffer[:packet_length])

                if self.validate_environment_packet(packet):

                    self.environment_packets += 1
                    del self.buffer[:packet_length]

                else:

                    self.bad_packets += 1

                    # Discard first byte and try to resynchronize.
                    del self.buffer[0]

                continue

            # ---------------------------------------------------------
            # Wheel packet
            #
            # AA 56 LEN TYPE
            # 36-byte payload
            # checksum
            # 55 AA
            #
            # Total = 43 bytes
            # ---------------------------------------------------------

            if self.buffer[1] == 0x56:

                packet_length = 43

                if len(self.buffer) < packet_length:
                    return

                packet = bytes(self.buffer[:packet_length])

                if self.validate_wheel_packet(packet):

                    self.wheel_packets += 1

                    self.process_wheel_packet(packet)

                    del self.buffer[:packet_length]

                else:

                    self.bad_packets += 1

                    del self.buffer[0]

                continue

    def validate_environment_packet(self, packet):

        if packet[0] != 0xAA or packet[1] != 0x55:
            return False

        if packet[31] != 0x55 or packet[32] != 0xAA:
            return False

        checksum = 0

        for b in packet[2:30]:
            checksum ^= b

        return checksum == packet[30]

    def validate_wheel_packet(self, packet):

        if packet[0] != 0xAA or packet[1] != 0x56:
            return False

        if packet[2] != 36:
            return False

        if packet[41] != 0x55 or packet[42] != 0xAA:
            return False

        checksum = 0

        # Wheel payload begins at byte 4.
        for b in packet[4:40]:
            checksum ^= b

        return checksum == packet[40]

    def process_wheel_packet(self, packet):

        # Payload begins at byte 4.
        payload = packet[4:40]

        (
            timestamp_ms,
            right_period_us,
            left_period_us,
            right_count,
            left_count,
            right_rpm_x100,
            left_rpm_x100,
            right_age_ms,
            left_age_ms
        ) = struct.unpack("<9I", payload)

        right_rpm = right_rpm_x100 / 100.0
        left_rpm = left_rpm_x100 / 100.0

        # Publish RPM
        right_rpm_msg = Float32()
        right_rpm_msg.data = right_rpm
        self.right_rpm_pub.publish(right_rpm_msg)

        left_rpm_msg = Float32()
        left_rpm_msg.data = left_rpm
        self.left_rpm_pub.publish(left_rpm_msg)

        # Publish cumulative transition counts
        right_count_msg = Int32()
        right_count_msg.data = right_count
        self.right_encoder_pub.publish(right_count_msg)

        left_count_msg = Int32()
        left_count_msg.data = left_count
        self.left_encoder_pub.publish(left_count_msg)

        # Periodic diagnostic output
        now = time.monotonic()

        if now - self.last_report >= 1.0:

            self.get_logger().info(
                f"Wheel packets: {self.wheel_packets} | "
                f"Right: {right_rpm:.2f} RPM "
                f"count={right_count} | "
                f"Left: {left_rpm:.2f} RPM "
                f"count={left_count} | "
                f"Bad packets: {self.bad_packets}"
            )

            self.last_report = now


def main(args=None):

    rclpy.init(args=args)

    node = WheelSensorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.ser.close()
        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()