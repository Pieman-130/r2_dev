#!/usr/bin/env python3

import argparse
import csv
import os
import re
import select
import serial
import sys
import time
from datetime import datetime


SENS_PORT = "/dev/SENS0"
LEONARDO_PORT = "/dev/ttyACM0"
BAUD = 115200


# ------------------------------------------------------------
# Leonardo CSV fields from the current balance sketch
# ------------------------------------------------------------

LEONARDO_FIELDS = [
    "arduino_time_ms",
    "pitch",
    "pitchRate",
    "rawGyroY",
    "gyroMin",
    "gyroMax",
    "gyroAvg",
    "gyroRange",
    "accelPitch",
    "controllerOutput",
    "targetMotor",
    "currentLeftMotor",
    "currentRightMotor",
    "balanceEnabled",
    "balanceFault",
    "faultCode",
]


# ------------------------------------------------------------
# Sensor packet parser
#
# Environment packet:
#   AA 55 + 28 byte payload + checksum + 55 AA
#
# Wheel packet:
#   AA 56 LEN TYPE + 36 byte payload + checksum + 55 AA
#
# Wheel payload:
#   timestamp_ms
#   right_period_us
#   left_period_us
#   right_transition_count
#   left_transition_count
#   right_rpm_x100
#   left_rpm_x100
#   right_age_ms
#   left_age_ms
# ------------------------------------------------------------

def parse_sens_buffer(buf):
    packets = []

    while True:
        # Find either packet preamble.
        candidates = [p for p in (buf.find(b"\xaa\x55"),
                                  buf.find(b"\xaa\x56"))
                      if p >= 0]

        if not candidates:
            # Keep the final AA in case it is the start of a preamble.
            if buf.endswith(b"\xaa"):
                buf = buf[-1:]
            else:
                buf = b""
            break

        start = min(candidates)

        if start > 0:
            buf = buf[start:]

        # Environment packet: 33 bytes total
        if buf.startswith(b"\xaa\x55"):
            total_len = 33
            packet_type = "environment"

        # Wheel packet: 43 bytes total
        elif buf.startswith(b"\xaa\x56"):
            if len(buf) < 4:
                break

            length = buf[2]
            msg_type = buf[3]

            if length != 36 or msg_type != 1:
                buf = buf[2:]
                continue

            total_len = 43
            packet_type = "wheel"

        else:
            buf = buf[1:]
            continue

        if len(buf) < total_len:
            break

        packet = buf[:total_len]
        buf = buf[total_len:]

        # Validate postamble.
        if packet[-2:] != b"\x55\xaa":
            continue

        if packet_type == "environment":
            payload = packet[2:30]
            checksum = packet[30]

        else:
            payload = packet[4:40]
            checksum = packet[40]

        calc = 0
        for b in payload:
            calc ^= b

        if calc != checksum:
            continue

        packets.append((packet_type, payload))

    return buf, packets


def decode_sensor_packet(packet_type, payload):
    values = [
        int.from_bytes(payload[i:i + 4], "little", signed=True)
        for i in range(0, 28, 4)
    ]

    if packet_type == "environment":
        return {
            "sensor_packet_type": "environment",
            "sensor_timestamp_ms": "",
            "right_period_us": "",
            "left_period_us": "",
            "right_transition_count": "",
            "left_transition_count": "",
            "right_rpm": "",
            "left_rpm": "",
            "right_age_ms": "",
            "left_age_ms": "",
            "right_encoder": values[5],
            "left_encoder": values[6],
            "right_ultrasonic": values[0],
            "left_ultrasonic": values[1],
            "rear_ultrasonic": values[2],
            "front_ir": values[3],
            "rear_ir": values[4],
        }

    # Wheel packet payload is nine uint32 values.
    w = [
        int.from_bytes(payload[i:i + 4], "little", signed=False)
        for i in range(0, 36, 4)
    ]

    return {
        "sensor_packet_type": "wheel",
        "sensor_timestamp_ms": w[0],
        "right_period_us": w[1],
        "left_period_us": w[2],
        "right_transition_count": w[3],
        "left_transition_count": w[4],
        "right_rpm": w[5] / 100.0,
        "left_rpm": w[6] / 100.0,
        "right_age_ms": w[7],
        "left_age_ms": w[8],
        "right_encoder": "",
        "left_encoder": "",
        "right_ultrasonic": "",
        "left_ultrasonic": "",
        "rear_ultrasonic": "",
        "front_ir": "",
        "rear_ir": "",
    }


def open_serial(path):
    try:
        return serial.Serial(
            path,
            BAUD,
            timeout=0,
            exclusive=True,
        )
    except TypeError:
        # Older pyserial versions may not support exclusive.
        return serial.Serial(
            path,
            BAUD,
            timeout=0,
        )


def main():
    parser = argparse.ArgumentParser(
        description="Log Leonardo balance telemetry and Metro Mini sensor/wheel data together."
    )

    parser.add_argument(
        "--seconds",
        type=float,
        default=0,
        help="Run time in seconds. 0 = run until Ctrl-C.",
    )

    parser.add_argument(
        "--output",
        default="",
        help="Output CSV filename. If omitted, a timestamped name is created.",
    )

    parser.add_argument(
        "--leonardo",
        default=LEONARDO_PORT,
        help=f"Leonardo serial port (default: {LEONARDO_PORT})",
    )

    parser.add_argument(
        "--sensor",
        default=SENS_PORT,
        help=f"Sensor serial port (default: {SENS_PORT})",
    )

    args = parser.parse_args()

    if not args.output:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"r2_balance_combined_{stamp}.csv"

    print("==============================================")
    print("R2 COMBINED BALANCE LOGGER")
    print("==============================================")
    print(f"Leonardo : {args.leonardo}")
    print(f"Sensors  : {args.sensor}")
    print(f"Output   : {args.output}")
    print(f"Duration : {'until Ctrl-C' if args.seconds <= 0 else str(args.seconds) + ' seconds'}")
    print()
    print("Starting...")
    print("")

    try:
        leonardo = open_serial(args.leonardo)
        sensor = open_serial(args.sensor)
    except Exception as e:
        print(f"ERROR opening serial port: {e}")
        sys.exit(1)

    # Give both devices a moment to settle.
    time.sleep(0.5)

    # Don't reset either device here. We want the robot's current state
    # and the current balance sketch to remain untouched.
    try:
        leonardo.reset_input_buffer()
        sensor.reset_input_buffer()
    except Exception:
        pass

    # The Leonardo CSV calls its first field arduino_time_ms.
    # Keep that exact name so DictWriter and the parsed telemetry agree.
    output_fields = [
        "host_time_s",
        "host_time_ms",

        # Leonardo
        "arduino_time_ms",
        "pitch",
        "pitchRate",
        "rawGyroY",
        "gyroMin",
        "gyroMax",
        "gyroAvg",
        "gyroRange",
        "accelPitch",
        "controllerOutput",
        "targetMotor",
        "currentLeftMotor",
        "currentRightMotor",
        "balanceEnabled",
        "balanceFault",
        "faultCode",

        # Metro / wheel sensor
        "sensor_packet_type",
        "sensor_timestamp_ms",
        "right_period_us",
        "left_period_us",
        "right_transition_count",
        "left_transition_count",
        "right_rpm",
        "left_rpm",
        "right_age_ms",
        "left_age_ms",
        "right_encoder",
        "left_encoder",

        # Environment sensors
        "right_ultrasonic",
        "left_ultrasonic",
        "rear_ultrasonic",
        "front_ir",
        "rear_ir",

        # Host-side timing
        "leonardo_receive_time_s",
        "sensor_receive_time_s",
    ]

    start = time.monotonic()
    last_status = start

    leonardo_buffer = b""
    sensor_buffer = b""

    latest_leonardo = {
        field: ""
        for field in LEONARDO_FIELDS
    }

    latest_sensor = {
        field: ""
        for field in [
            "sensor_packet_type",
            "sensor_timestamp_ms",
            "right_period_us",
            "left_period_us",
            "right_transition_count",
            "left_transition_count",
            "right_rpm",
            "left_rpm",
            "right_age_ms",
            "left_age_ms",
            "right_encoder",
            "left_encoder",
            "right_ultrasonic",
            "left_ultrasonic",
            "rear_ultrasonic",
            "front_ir",
            "rear_ir",
        ]
    }

    latest_leonardo_receive = ""
    latest_sensor_receive = ""

    leonardo_lines = 0
    leonardo_csv_lines = 0
    sensor_packets = 0
    sensor_bad_or_discarded = 0

    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=output_fields)
        writer.writeheader()

        try:
            while True:
                now = time.monotonic()
                elapsed = now - start

                if args.seconds > 0 and elapsed >= args.seconds:
                    break

                # ----------------------------------------------------
                # Read Leonardo
                # ----------------------------------------------------

                try:
                    n = leonardo.in_waiting
                    if n:
                        data = leonardo.read(n)
                        leonardo_buffer += data

                        while b"\n" in leonardo_buffer:
                            line, leonardo_buffer = leonardo_buffer.split(
                                b"\n", 1
                            )

                            line = line.strip()
                            if not line:
                                continue

                            leonardo_lines += 1

                            try:
                                text_line = line.decode(
                                    "ascii",
                                    errors="ignore"
                                )
                            except Exception:
                                continue

                            parts = [
                                x.strip()
                                for x in text_line.split(",")
                            ]

                            # We only care about the 16-column CSV
                            # telemetry lines from the current sketch.
                            if len(parts) == len(LEONARDO_FIELDS):
                                try:
                                    # Verify first field is numeric.
                                    float(parts[0])
                                except ValueError:
                                    continue

                                latest_leonardo = dict(
                                    zip(LEONARDO_FIELDS, parts)
                                )

                                latest_leonardo_receive = (
                                    time.monotonic() - start
                                )

                                leonardo_csv_lines += 1

                                # Log every valid Leonardo telemetry line.
                                row = {
                                    field: ""
                                    for field in output_fields
                                }

                                row["host_time_s"] = (
                                    time.monotonic() - start
                                )
                                row["host_time_ms"] = int(
                                    (time.monotonic() - start) * 1000
                                )

                                for field in LEONARDO_FIELDS:
                                    row[field] = latest_leonardo[field]

                                for field in latest_sensor:
                                    row[field] = latest_sensor[field]

                                row["leonardo_receive_time_s"] = (
                                    latest_leonardo_receive
                                )
                                row["sensor_receive_time_s"] = (
                                    latest_sensor_receive
                                )

                                writer.writerow(row)

                except (OSError, serial.SerialException) as e:
                    print(f"\nLeonardo serial error: {e}")
                    break

                # ----------------------------------------------------
                # Read sensor MCU
                # ----------------------------------------------------

                try:
                    n = sensor.in_waiting
                    if n:
                        data = sensor.read(n)
                        sensor_buffer += data

                        before_len = len(sensor_buffer)

                        sensor_buffer, packets = parse_sens_buffer(
                            sensor_buffer
                        )

                        # Anything consumed but not emitted as a valid
                        # packet is counted approximately here.
                        consumed = before_len - len(sensor_buffer)
                        sensor_bad_or_discarded += max(
                            0,
                            consumed - sum(
                                33 if p[0] == "environment" else 43
                                for p in packets
                            )
                        )

                        for packet_type, payload in packets:
                            latest_sensor = decode_sensor_packet(
                                packet_type,
                                payload
                            )

                            latest_sensor_receive = (
                                time.monotonic() - start
                            )

                            sensor_packets += 1

                except (OSError, serial.SerialException) as e:
                    print(f"\nSensor serial error: {e}")
                    break

                # ----------------------------------------------------
                # Status display
                # ----------------------------------------------------

                now = time.monotonic()

                if now - last_status >= 1.0:
                    last_status = now

                    pitch = latest_leonardo.get("pitch", "")
                    motor = latest_leonardo.get("currentLeftMotor", "")
                    rrpm = latest_sensor.get("right_rpm", "")
                    lrpm = latest_sensor.get("left_rpm", "")

                    print(
                        f"\r"
                        f"t={elapsed:6.1f}s  "
                        f"pitch={pitch:>7}  "
                        f"motor={motor:>4}  "
                        f"R={rrpm:>6} RPM  "
                        f"L={lrpm:>6} RPM  "
                        f"L-lines={leonardo_csv_lines:5}  "
                        f"S-packets={sensor_packets:5}",
                        end="",
                        flush=True,
                    )

                # Prevent busy spinning when both ports are quiet.
                time.sleep(0.001)

        except KeyboardInterrupt:
            print("\n\nStopped by user.")

    leonardo.close()
    sensor.close()

    duration = time.monotonic() - start

    print()
    print("==============================================")
    print("LOGGER COMPLETE")
    print("==============================================")
    print(f"Duration:             {duration:.2f} s")
    print(f"Leonardo lines:       {leonardo_lines}")
    print(f"Leonardo CSV records: {leonardo_csv_lines}")
    print(f"Sensor packets:       {sensor_packets}")
    print(f"Output file:          {os.path.abspath(args.output)}")
    print("")


if __name__ == "__main__":
    main()
