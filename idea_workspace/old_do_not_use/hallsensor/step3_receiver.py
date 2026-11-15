#!/usr/bin/env python3

import argparse
import csv
import struct
import sys
import time
from datetime import datetime

import serial


# ============================================================
# Packet definitions
# ============================================================

ENV_HEADER = b"\xAA\x55"
WHEEL_HEADER = b"\xAA\x56"
POSTAMBLE = b"\x55\xAA"

ENV_PACKET_LEN = 33
WHEEL_PACKET_LEN = 43

ENV_PAYLOAD_LEN = 28
WHEEL_PAYLOAD_LEN = 36


# ============================================================
# Checksum
# ============================================================

def xor_checksum(data: bytes) -> int:
    checksum = 0

    for b in data:
        checksum ^= b

    return checksum


# ============================================================
# Packet parser
# ============================================================

class PacketParser:

    def __init__(self):
        self.buffer = bytearray()

        self.good_environment = 0
        self.good_wheel = 0
        self.bad_packets = 0

    def feed(self, data: bytes):

        self.buffer.extend(data)

        packets = []

        while True:

            if len(self.buffer) < 2:
                break


            # ------------------------------------------------
            # Find next possible header
            # ------------------------------------------------

            env_pos = self.buffer.find(ENV_HEADER)
            wheel_pos = self.buffer.find(WHEEL_HEADER)

            positions = [
                p for p in (env_pos, wheel_pos)
                if p >= 0
            ]

            if not positions:

                # Keep a trailing AA in case it is the first
                # byte of a header split across reads.

                if self.buffer[-1] == 0xAA:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()

                break


            start = min(positions)

            if start > 0:
                del self.buffer[:start]


            # ------------------------------------------------
            # Determine packet type
            # ------------------------------------------------

            if self.buffer[:2] == ENV_HEADER:

                packet_type = "environment"
                packet_len = ENV_PACKET_LEN
                payload_start = 2
                payload_len = ENV_PAYLOAD_LEN
                checksum_index = 30

            elif self.buffer[:2] == WHEEL_HEADER:

                packet_type = "wheel"
                packet_len = WHEEL_PACKET_LEN
                payload_start = 4
                payload_len = WHEEL_PAYLOAD_LEN
                checksum_index = 40

            else:

                del self.buffer[0]
                continue


            # ------------------------------------------------
            # Wait until complete packet has arrived
            # ------------------------------------------------

            if len(self.buffer) < packet_len:
                break


            packet = bytes(
                self.buffer[:packet_len]
            )


            # ------------------------------------------------
            # Check postamble
            # ------------------------------------------------

            if packet[-2:] != POSTAMBLE:

                self.bad_packets += 1

                # Discard one byte and search again.
                del self.buffer[0]

                continue


            # ------------------------------------------------
            # Check checksum
            # ------------------------------------------------

            payload = packet[
                payload_start:
                payload_start + payload_len
            ]

            received_checksum = \
                packet[checksum_index]

            calculated_checksum = \
                xor_checksum(payload)


            if received_checksum != calculated_checksum:

                self.bad_packets += 1

                del self.buffer[0]

                continue


            # ------------------------------------------------
            # Valid packet
            # ------------------------------------------------

            packets.append(
                (packet_type, packet)
            )

            if packet_type == "environment":
                self.good_environment += 1
            else:
                self.good_wheel += 1


            del self.buffer[:packet_len]


        return packets


# ============================================================
# Decode environmental packet
# ============================================================

def decode_environment(packet):

    values = struct.unpack(
        "<7i",
        packet[2:30]
    )

    return {
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


# ============================================================
# Decode wheel packet
# ============================================================

def decode_wheel(packet):

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


# ============================================================
# CSV
# ============================================================

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


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Step 3 Metro Mini -> LattePanda "
            "diagnostic receiver"
        )
    )


    parser.add_argument(
        "--port",
        default="/dev/SENS0",
        help="Serial port (default: /dev/SENS0)"
    )


    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )


    parser.add_argument(
        "--output",
        default=None,
        help=(
            "CSV output filename. "
            "A timestamped filename is used if omitted."
        )
    )


    parser.add_argument(
        "--seconds",
        type=float,
        default=0,
        help=(
            "Stop automatically after this many seconds. "
            "0 means run until Ctrl-C."
        )
    )


    args = parser.parse_args()


    if args.output is None:

        timestamp = \
            datetime.now().strftime(
                "%Y%m%d_%H%M%S"
            )

        args.output = \
            f"step3_receiver_{timestamp}.csv"


    print(
        f"Opening {args.port} at {args.baud} baud"
    )

    print(
        f"Logging to {args.output}"
    )

    if args.seconds > 0:
        print(
            f"Capture duration: {args.seconds:.1f} seconds"
        )
    else:
        print(
            "Running until Ctrl-C"
        )

    print()


    try:

        ser = serial.Serial(
            args.port,
            args.baud,
            timeout=0.1
        )

    except serial.SerialException as e:

        print(
            f"Could not open serial port: {e}",
            file=sys.stderr
        )

        return 1


    # Clear bytes that may have been waiting before
    # the test began.

    ser.reset_input_buffer()


    packet_parser = PacketParser()


    start_time = time.monotonic()

    last_wheel_host_time = None
    last_wheel_arduino_time = None

    wheel_gaps = []

    rows_written = 0


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

                elapsed = \
                    time.monotonic() - start_time


                if (
                    args.seconds > 0
                    and elapsed >= args.seconds
                ):
                    break


                data = ser.read(256)


                if not data:
                    continue


                packets = \
                    packet_parser.feed(data)


                for packet_type, packet in packets:

                    host_now = time.monotonic()

                    host_wall = time.time()


                    if packet_type == "wheel":

                        row = \
                            decode_wheel(packet)


                        # ------------------------------------
                        # Check Arduino wheel-packet spacing
                        # ------------------------------------

                        arduino_time = \
                            row["arduino_timestamp_ms"]


                        if (
                            last_wheel_arduino_time
                            is not None
                        ):

                            delta = (
                                arduino_time
                                - last_wheel_arduino_time
                            )

                            wheel_gaps.append(delta)


                        last_wheel_arduino_time = \
                            arduino_time


                    else:

                        row = \
                            decode_environment(packet)


                    row["host_time"] = \
                        datetime.fromtimestamp(
                            host_wall
                        ).isoformat(
                            timespec="milliseconds"
                        )


                    row["host_time_ms"] = int(
                        host_wall * 1000
                    )


                    row["packet_type"] = \
                        packet_type


                    writer.writerow(row)
                    csv_file.flush()

                    rows_written += 1


                    # ----------------------------------------
                    # Live status
                    # ----------------------------------------

                    if packet_type == "wheel":

                        print(
                            f"\r"
                            f"Wheel: "
                            f"{packet_parser.good_wheel:7d} | "
                            f"Env: "
                            f"{packet_parser.good_environment:6d} | "
                            f"Bad: "
                            f"{packet_parser.bad_packets:4d} | "
                            f"R: "
                            f"{row['right_rpm']:6.2f} | "
                            f"L: "
                            f"{row['left_rpm']:6.2f} | "
                            f"RC: "
                            f"{row['right_transition_count']:7d} | "
                            f"LC: "
                            f"{row['left_transition_count']:7d}",
                            end="",
                            flush=True
                        )


        except KeyboardInterrupt:

            print(
                "\n\nStopping receiver..."
            )


        finally:

            ser.close()


    elapsed = \
        time.monotonic() - start_time


    print()
    print()
    print("========== Step 3 Results ==========")

    print(
        f"Elapsed time:          {elapsed:.2f} s"
    )

    print(
        f"Environmental packets: "
        f"{packet_parser.good_environment}"
    )

    print(
        f"Wheel packets:         "
        f"{packet_parser.good_wheel}"
    )

    print(
        f"Bad packets:           "
        f"{packet_parser.bad_packets}"
    )

    print(
        f"CSV rows:              "
        f"{rows_written}"
    )


    if elapsed > 0:

        print(
            f"Environmental rate:    "
            f"{packet_parser.good_environment / elapsed:.2f} Hz"
        )

        print(
            f"Wheel packet rate:     "
            f"{packet_parser.good_wheel / elapsed:.2f} Hz"
        )


    if wheel_gaps:

        print()
        print("Wheel Arduino timestamp spacing:")

        print(
            f"  Min:                 "
            f"{min(wheel_gaps)} ms"
        )

        print(
            f"  Max:                 "
            f"{max(wheel_gaps)} ms"
        )

        print(
            f"  Average:             "
            f"{sum(wheel_gaps) / len(wheel_gaps):.3f} ms"
        )


        abnormal = [
            gap for gap in wheel_gaps
            if gap != 20
        ]

        print(
            f"  Non-20ms intervals:   "
            f"{len(abnormal)}"
        )


    print()
    print(
        f"CSV saved to:          {args.output}"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())