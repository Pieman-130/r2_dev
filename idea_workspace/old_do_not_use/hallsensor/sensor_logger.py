#!/usr/bin/env python3

import argparse
import csv
import struct
import sys
import time
from datetime import datetime

import serial


ENV_HEADER = b"\xAA\x55"
WHEEL_HEADER = b"\xAA\x56"
POSTAMBLE = b"\x55\xAA"

ENV_PACKET_LEN = 33
WHEEL_PACKET_LEN = 43

ENV_PAYLOAD_LEN = 28
WHEEL_PAYLOAD_LEN = 36


def xor_checksum(data: bytes) -> int:
    value = 0
    for b in data:
        value ^= b
    return value


class PacketParser:

    def __init__(self):
        self.buffer = bytearray()
        self.bad_packets = 0

    def feed(self, data: bytes):

        self.buffer.extend(data)
        packets = []

        while True:

            if len(self.buffer) < 2:
                break

            env_pos = self.buffer.find(ENV_HEADER)
            wheel_pos = self.buffer.find(WHEEL_HEADER)

            positions = [
                p for p in (env_pos, wheel_pos)
                if p >= 0
            ]

            if not positions:
                if self.buffer[-1] == 0xAA:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break

            start = min(positions)

            if start > 0:
                del self.buffer[:start]

            if len(self.buffer) < 2:
                break

            if self.buffer[:2] == ENV_HEADER:
                packet_len = ENV_PACKET_LEN
                payload_len = ENV_PAYLOAD_LEN
                packet_type = "environment"
                checksum_index = 30

            elif self.buffer[:2] == WHEEL_HEADER:
                packet_len = WHEEL_PACKET_LEN
                payload_len = WHEEL_PAYLOAD_LEN
                packet_type = "wheel"
                checksum_index = 40

            else:
                del self.buffer[0]
                continue

            if len(self.buffer) < packet_len:
                break

            packet = bytes(self.buffer[:packet_len])

            if packet[-2:] != POSTAMBLE:
                self.bad_packets += 1
                del self.buffer[0]
                continue

            # IMPORTANT:
            #
            # Environmental payload begins at byte 2:
            #   AA 55 [payload...]
            #
            # Wheel payload begins at byte 4:
            #   AA 56 LEN TYPE [payload...]
            #
            payload_start = 2 if packet_type == "environment" else 4

            payload = packet[
                payload_start:
                payload_start + payload_len
            ]

            received_checksum = packet[checksum_index]
            calculated_checksum = xor_checksum(payload)

            if received_checksum != calculated_checksum:
                self.bad_packets += 1
                del self.buffer[0]
                continue

            packets.append((packet_type, packet))

            del self.buffer[:packet_len]

        return packets


def decode_environment(packet: bytes):

    values = struct.unpack(
        "<7i",
        packet[2:30]
    )

    return {
        "packet_type": "environment",
        "arduino_timestamp_ms": "",

        "left_ultrasonic_us": values[0],
        "right_ultrasonic_us": values[1],
        "rear_ultrasonic_us": values[2],

        "front_cliff": values[3],
        "rear_cliff": values[4],

        "right_hall_high_us": values[5],
        "left_hall_high_us": values[6],

        "right_period_us": "",
        "left_period_us": "",

        "right_transition_count": "",
        "left_transition_count": "",

        "right_rpm": "",
        "left_rpm": "",

        "right_age_ms": "",
        "left_age_ms": "",
    }


def decode_wheel(packet: bytes):

    (
        timestamp_ms,
        right_period_us,
        left_period_us,
        right_count,
        left_count,
        right_rpm_x100,
        left_rpm_x100,
        right_age_ms,
        left_age_ms,
    ) = struct.unpack(
        "<9I",
        packet[4:40]
    )

    if right_age_ms == 0xFFFFFFFF:
        right_age = ""
    else:
        right_age = right_age_ms

    if left_age_ms == 0xFFFFFFFF:
        left_age = ""
    else:
        left_age = left_age_ms

    return {
        "packet_type": "wheel",
        "arduino_timestamp_ms": timestamp_ms,

        "left_ultrasonic_us": "",
        "right_ultrasonic_us": "",
        "rear_ultrasonic_us": "",

        "front_cliff": "",
        "rear_cliff": "",

        "right_hall_high_us": "",
        "left_hall_high_us": "",

        "right_period_us": right_period_us,
        "left_period_us": left_period_us,

        "right_transition_count": right_count,
        "left_transition_count": left_count,

        "right_rpm": right_rpm_x100 / 100.0,
        "left_rpm": left_rpm_x100 / 100.0,

        "right_age_ms": right_age,
        "left_age_ms": left_age,
    }


CSV_COLUMNS = [
    "host_time",
    "host_time_ms",
    "packet_type",
    "arduino_timestamp_ms",

    "left_ultrasonic_us",
    "right_ultrasonic_us",
    "rear_ultrasonic_us",

    "front_cliff",
    "rear_cliff",

    "right_hall_high_us",
    "left_hall_high_us",

    "right_period_us",
    "left_period_us",

    "right_transition_count",
    "left_transition_count",

    "right_rpm",
    "left_rpm",

    "right_age_ms",
    "left_age_ms",
]


def main():

    parser = argparse.ArgumentParser(
        description="Log Metro Mini Step 2 sensor packets to CSV."
    )

    parser.add_argument(
        "--port",
        default="/dev/SENS0",
        help="Serial port (default: /dev/SENS0)",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)",
    )

    parser.add_argument(
        "--output",
        default=None,
        help="CSV filename. If omitted, a timestamped filename is created.",
    )

    args = parser.parse_args()

    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"step2_sensor_log_{timestamp}.csv"

    print(f"Opening {args.port} at {args.baud} baud")
    print(f"Logging to {args.output}")
    print("Press Ctrl-C to stop.")
    print()

    try:
        ser = serial.Serial(
            args.port,
            args.baud,
            timeout=0.1,
        )

    except serial.SerialException as e:

        print(
            f"Could not open serial port: {e}",
            file=sys.stderr
        )

        return 1

    packet_parser = PacketParser()

    env_count = 0
    wheel_count = 0

    start_time = time.monotonic()

    with open(
        args.output,
        "w",
        newline=""
    ) as csv_file:

        writer = csv.DictWriter(
            csv_file,
            fieldnames=CSV_COLUMNS
        )

        writer.writeheader()

        try:

            while True:

                data = ser.read(256)

                if not data:
                    continue

                packets = packet_parser.feed(data)

                for packet_type, packet in packets:

                    host_now = time.time()

                    if packet_type == "wheel":

                        row = decode_wheel(packet)
                        wheel_count += 1

                    else:

                        row = decode_environment(packet)
                        env_count += 1

                    row["host_time"] = datetime.fromtimestamp(
                        host_now
                    ).isoformat(
                        timespec="milliseconds"
                    )

                    row["host_time_ms"] = int(
                        host_now * 1000
                    )

                    writer.writerow(row)
                    csv_file.flush()

                    if packet_type == "wheel":

                        print(
                            f"\rWheel packets: {wheel_count:6d} | "
                            f"Env: {env_count:5d} | "
                            f"R: {row['right_rpm']:7.2f} RPM | "
                            f"L: {row['left_rpm']:7.2f} RPM | "
                            f"RC: {row['right_transition_count']:7d} | "
                            f"LC: {row['left_transition_count']:7d}",
                            end="",
                            flush=True
                        )

        except KeyboardInterrupt:

            print("\n\nStopping logger...")

        finally:

            ser.close()

    elapsed = time.monotonic() - start_time

    print()
    print(f"Environmental packets: {env_count}")
    print(f"Wheel packets:         {wheel_count}")
    print(f"Bad packets:           {packet_parser.bad_packets}")
    print(f"Elapsed time:          {elapsed:.2f} s")

    if elapsed > 0:

        print(
            f"Environmental rate:    "
            f"{env_count / elapsed:.2f} Hz"
        )

        print(
            f"Wheel packet rate:     "
            f"{wheel_count / elapsed:.2f} Hz"
        )

    print(f"CSV saved:             {args.output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())