#!/usr/bin/env python3

import csv
import serial
import threading
import time
from datetime import datetime

PORT = "/dev/ttyACM0"
BAUD = 115200

COMMAND_INTERVAL = 0.10
OUTPUT_FILE = f"r2_manual_encoder_test.csv"

# Leonardo telemetry fields
FIELDS = [
    "time_ms",
    "pitch",
    "pitchRate",
    "accelPitch",
    "receivedLeftCommand",
    "receivedRightCommand",
    "targetLeftPWM",
    "targetRightPWM",
    "currentLeftPWM",
    "currentRightPWM",
    "rightRPM",
    "leftRPM",
    "rightSignedRPM",
    "leftSignedRPM",
    "rightTransitionCount",
    "leftTransitionCount",
    "rightPositionTransitions",
    "leftPositionTransitions",
    "rightPeriodUs",
    "leftPeriodUs",
]


def crc8(data):
    crc = 0x00

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


def make_packet(left, right):
    left = max(-100, min(100, int(left)))
    right = max(-100, min(100, int(right)))

    packet = bytearray([
        0xAA,
        0x55,
        4,
        0x01,
        left & 0xFF,
        right & 0xFF,
    ])

    packet.append(crc8(packet))

    return bytes(packet)


def command_sender(ser, command, running):
    while running.is_set():
        try:
            left, right = command
            ser.write(make_packet(left, right))
            ser.flush()
        except serial.SerialException:
            running.clear()
            break

        time.sleep(COMMAND_INTERVAL)


def telemetry_reader(ser, records, running):
    while running.is_set():
        try:
            line = ser.readline()

            if not line:
                continue

            text = line.decode("utf-8", errors="replace").strip()

            if not text:
                continue

            parts = text.split(",")

            if len(parts) != len(FIELDS):
                continue

            if parts[0] == "time_ms":
                continue

            try:
                # Convert everything to numeric values.
                row = [
                    int(parts[0]),
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                    int(parts[4]),
                    int(parts[5]),
                    int(parts[6]),
                    int(parts[7]),
                    int(parts[8]),
                    int(parts[9]),
                    float(parts[10]),
                    float(parts[11]),
                    float(parts[12]),
                    float(parts[13]),
                    int(parts[14]),
                    int(parts[15]),
                    int(parts[16]),
                    int(parts[17]),
                    int(parts[18]),
                    int(parts[19]),
                ]

                records.append(row)

            except ValueError:
                continue

        except serial.SerialException:
            running.clear()
            break


def print_latest(records):
    if not records:
        return

    r = records[-1]

    print(
        f"\r"
        f"pitch={r[1]:7.2f}°  "
        f"rx={r[4]:4d}/{r[5]:4d}  "
        f"target={r[6]:3d}/{r[7]:3d}  "
        f"PWM={r[8]:3d}/{r[9]:3d}  "
        f"RPM R/L={r[10]:6.2f}/{r[11]:6.2f}  "
        f"counts R/L={r[14]:5d}/{r[15]:5d}",
        end="",
        flush=True,
    )


def main():
    print("==============================================")
    print("R2 MANUAL MOTOR + ENCODER TEST")
    print("==============================================")
    print()
    print(f"Opening {PORT} at {BAUD}...")
    print()
    print("BALANCE CONTROL IS DISABLED.")
    print()
    print("Commands:")
    print("  25 25       forward")
    print(" -25 -25      reverse")
    print("  25 -25      turn")
    print("  0 0         stop")
    print("  q            quit")
    print()
    print("The encoder data is logged continuously.")
    print()

    try:
        ser = serial.Serial(
            PORT,
            BAUD,
            timeout=0.1
        )
    except serial.SerialException as e:
        print(f"ERROR opening serial port: {e}")
        return

    # Leonardo resets when USB serial opens.
    time.sleep(2)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    print("Connected.")
    print()

    command = [0, 0]
    records = []

    running = threading.Event()
    running.set()

    reader = threading.Thread(
        target=telemetry_reader,
        args=(ser, records, running),
        daemon=True,
    )

    sender = threading.Thread(
        target=command_sender,
        args=(ser, command, running),
        daemon=True,
    )

    reader.start()
    sender.start()

    last_print = time.monotonic()

    try:
        while running.is_set():

            # Display latest telemetry periodically.
            if time.monotonic() - last_print >= 0.5:
                print_latest(records)
                last_print = time.monotonic()

            # Use non-blocking-ish input with a short timeout isn't
            # portable with normal input(), so let the user enter
            # commands directly. Telemetry continues in the background.
            text = input("\n\nCommand L R (or q): ").strip()

            if text.lower() == "q":
                break

            parts = text.replace(",", " ").split()

            if len(parts) != 2:
                print("Enter two values, e.g. 25 25")
                continue

            try:
                left = int(parts[0])
                right = int(parts[1])
            except ValueError:
                print("Commands must be integers.")
                continue

            if not (-100 <= left <= 100 and -100 <= right <= 100):
                print("Commands must be between -100 and +100.")
                continue

            command[0] = left
            command[1] = right

            print(f"Command set to {left}, {right}")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        # Stop motors before closing.
        command[0] = 0
        command[1] = 0
        time.sleep(0.3)

        running.clear()

        reader.join(timeout=1)
        sender.join(timeout=1)

        ser.close()

    print()
    print("Saving telemetry...")

    with open(OUTPUT_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)
        writer.writerows(records)

    print(f"Saved {len(records)} telemetry records.")
    print(f"File: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
