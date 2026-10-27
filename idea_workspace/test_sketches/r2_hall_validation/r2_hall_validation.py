#!/usr/bin/env python3
"""R2 Hall validation CSV logger with interactive serial motor commands."""
import csv
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

PORT = "/dev/ttyACM0"
BAUD = 115200
FIELDS = [
    "host_time", "arduino_ms", "left_pwm", "right_pwm",
    "left_hall_state", "right_hall_state",
    "left_hall_transitions", "right_hall_transitions",
    "left_invalid_transitions", "right_invalid_transitions",
    "left_sc_transitions", "right_sc_transitions",
    "left_sc_level", "right_sc_level",
]
stop_event = threading.Event()
print_lock = threading.Lock()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    print("Available serial ports:")
    for p in list_ports.comports():
        print(f"  {p.device}: {p.description}")

    print(f"Opening {port} at {BAUD} baud. Close screen/other serial monitors first.")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1, write_timeout=1)
    except serial.SerialException as exc:
        print(f"Could not open serial port: {exc}")
        return 1

    time.sleep(2)  # Leonardo may reset when serial opens.
    ser.reset_input_buffer()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = Path(f"r2_hall_validation_{stamp}.csv")
    csv_file = csv_path.open("w", newline="", buffering=1)
    writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
    writer.writeheader()

    def reader():
        while not stop_event.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException as exc:
                with print_lock:
                    print(f"\nSerial read error: {exc}")
                stop_event.set()
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            parts = line.split(",")

            # Expected telemetry has 13 comma-separated values:
            # ms,leftPWM,rightPWM,Lstate,Rstate,LhallTransitions,RhallTransitions,
            # Linvalid,Rinvalid,LscTransitions,RscTransitions,LscLevel,RscLevel
            if len(parts) == 13:
                try:
                    values = [float(v) for v in parts]
                except ValueError:
                    with print_lock:
                        print(f"Arduino: {line}")
                    continue
                row = {"host_time": datetime.now().isoformat(timespec="milliseconds")}
                row.update(dict(zip(FIELDS[1:], values)))
                writer.writerow(row)
                with print_lock:
                    print(
                        f"t={values[0]:8.0f} ms | PWM L/R={values[1]:5.0f}/{values[2]:5.0f} | "
                        f"Hall L/R={int(values[3]):03b}/{int(values[4]):03b} | "
                        f"Hall Δ L/R={values[5]:7.0f}/{values[6]:7.0f} | "
                        f"invalid L/R={values[7]:5.0f}/{values[8]:5.0f} | "
                        f"SC Δ L/R={values[9]:7.0f}/{values[10]:7.0f}"
                    )
            else:
                with print_lock:
                    print(f"Arduino: {line}")

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    print(f"Logging to {csv_path.resolve()}")
    print("Commands (sent with newline): M 25 25 | M -25 -25 | STOP | q")
    print("Keep the robot securely supported, wheels off the ground, during motor tests.")

    try:
        while not stop_event.is_set():
            try:
                command = input("> ").strip()
            except EOFError:
                command = "q"
            if not command:
                continue
            if command.lower() in ("q", "quit", "exit"):
                try:
                    ser.write(b"STOP\n")
                    ser.flush()
                except serial.SerialException:
                    pass
                break
            try:
                ser.write((command + "\n").encode("ascii"))
                ser.flush()
                print(f"Sent: {command}")
            except (UnicodeEncodeError, serial.SerialException) as exc:
                print(f"Command send failed: {exc}")
    except KeyboardInterrupt:
        try:
            ser.write(b"STOP\n")
            ser.flush()
        except serial.SerialException:
            pass
    finally:
        stop_event.set()
        thread.join(timeout=1)
        csv_file.close()
        ser.close()
        print(f"CSV saved: {csv_path.resolve()}")
    return 0


main()