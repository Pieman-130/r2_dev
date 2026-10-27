#!/usr/bin/env python3
"""
R2 Hall decoder logger for r2_hall_decode_test.ino.

Expected Arduino telemetry (20 comma-separated fields):
ms,Lstate,Rstate,Lsigned,Rsigned,Lvalid,Rvalid,Linvalid,Rinvalid,
LperiodUs,RperiodUs,Lsc,Rsc,requestedL_pct,requestedR_pct,targetL_pwm,targetR_pwm,appliedL_pwm,appliedR_pwm,command_timed_out

Interactive commands are sent with a newline:
  M 25 25
  M -25 -25
  STOP
  q

Usage:
  python3 r2_hall_decode_pwm_logger.py
  python3 r2_hall_decode_pwm_logger.py /dev/ttyACM0
"""

import csv
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200

FIELDS = [
    "host_time",
    "arduino_ms",
    "left_hall_state",
    "right_hall_state",
    "left_signed_transitions",
    "right_signed_transitions",
    "left_valid_transitions",
    "right_valid_transitions",
    "left_invalid_transitions",
    "right_invalid_transitions",
    "left_transition_period_us",
    "right_transition_period_us",
    "left_sc_transitions",
    "right_sc_transitions",
    "requested_left_percent",
    "requested_right_percent",
    "target_left_pwm",
    "target_right_pwm",
    "applied_left_pwm",
    "applied_right_pwm",
    "command_timed_out",
]

stop_reader = threading.Event()
print_lock = threading.Lock()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT

    ports = list(list_ports.comports())
    if ports:
        print("Detected serial ports:")
        for p in ports:
            print(f"  {p.device}: {p.description}")
    else:
        print("No serial ports detected by pySerial.")

    print(f"Opening {port} at {BAUD} baud.")
    print("Close screen or any other serial monitor first.")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1, write_timeout=1)
    except serial.SerialException as exc:
        print(f"Could not open serial port: {exc}")
        return 1

    # The Leonardo may reset when the port is opened.
    time.sleep(2.0)
    ser.reset_input_buffer()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = Path(f"r2_hall_decode_{stamp}.csv")
    csv_file = csv_path.open("w", newline="", buffering=1)
    writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
    writer.writeheader()

    command_path = Path(f"r2_hall_commands_{stamp}.csv")
    command_file = command_path.open("w", newline="", buffering=1)
    command_writer = csv.DictWriter(command_file, fieldnames=["host_time", "command", "sent"])
    command_writer.writeheader()

    counts = {"telemetry": 0, "other": 0, "malformed": 0}

    def reader():
        while not stop_reader.is_set():
            try:
                raw = ser.readline()
            except serial.SerialException as exc:
                with print_lock:
                    print(f"\nSerial read error: {exc}")
                stop_reader.set()
                return

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != 20:
                counts["other"] += 1
                with print_lock:
                    print(f"Arduino: {line}")
                continue

            try:
                vals = [int(x.strip()) for x in parts]
            except ValueError:
                counts["malformed"] += 1
                with print_lock:
                    print(f"Unparsed line: {line}")
                continue

            row = {
                "host_time": datetime.now().isoformat(timespec="milliseconds"),
                **dict(zip(FIELDS[1:], vals)),
            }
            writer.writerow(row)
            counts["telemetry"] += 1

            with print_lock:
                print(
                    f"t={vals[0]:8d} ms | "
                    f"state L/R={vals[1]:03b}/{vals[2]:03b} | "
                    f"signed L/R={vals[3]:7d}/{vals[4]:7d} | "
                    f"valid L/R={vals[5]:7d}/{vals[6]:7d} | "
                    f"invalid L/R={vals[7]:5d}/{vals[8]:5d} | "
                    f"period µs L/R={vals[9]:7d}/{vals[10]:7d} | "
                    f"SC L/R={vals[11]:7d}/{vals[12]:7d} | "
                    f"request % L/R={vals[13]:4d}/{vals[14]:4d} | "
                    f"target PWM L/R={vals[15]:3d}/{vals[16]:3d} | "
                    f"applied PWM L/R={vals[17]:3d}/{vals[18]:3d} | "
                    f"timeout={vals[19]}"
                )

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()

    print(f"\nRecording CSV: {csv_path.resolve()}")
    print("Enter commands (newline sent automatically):")
    print("  M 25 25     low-power forward")
    print("  M -25 -25   low-power reverse")
    print("  STOP        stop motors")
    print("  q           stop motors, save CSV, exit")
    print("\nSuggested test: M 25 25, wait 2–3 s, STOP, wait 1 s,")
    print("M -25 -25, wait 2–3 s, STOP. Keep wheels off the ground.")
    print("The Arduino's command timeout is still active.\n")

    try:
        while not stop_reader.is_set():
            try:
                cmd = input("> ").strip()
            except EOFError:
                cmd = "q"

            if not cmd:
                continue
            if cmd.lower() in {"q", "quit", "exit"}:
                try:
                    ser.write(b"STOP\n")
                    ser.flush()
                    command_writer.writerow({"host_time": datetime.now().isoformat(timespec="milliseconds"), "command": "STOP", "sent": True})
                except serial.SerialException:
                    pass
                break

            try:
                ser.write((cmd + "\n").encode("ascii"))
                ser.flush()
                command_writer.writerow({"host_time": datetime.now().isoformat(timespec="milliseconds"), "command": cmd, "sent": True})
                print(f"Sent: {cmd}")
            except (UnicodeEncodeError, serial.SerialException) as exc:
                print(f"Could not send command: {exc}")

    except KeyboardInterrupt:
        try:
            ser.write(b"STOP\n")
            ser.flush()
        except serial.SerialException:
            pass
    finally:
        stop_reader.set()
        thread.join(timeout=1.0)
        csv_file.close()
        command_file.close()
        ser.close()
        print(f"\nSaved telemetry: {csv_path.resolve()}")
        print(f"Saved commands:  {command_path.resolve()}")
        print(
            f"Parsed telemetry rows: {counts['telemetry']}; "
            f"other lines: {counts['other']}; malformed rows: {counts['malformed']}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
