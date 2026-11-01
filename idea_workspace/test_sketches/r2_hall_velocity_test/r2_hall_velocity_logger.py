#!/usr/bin/env python3
"""Logger for r2_hall_velocity_period_preferred_test.ino (36 Arduino telemetry fields)."""
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
    "host_time", "arduino_ms", "left_hall_state", "right_hall_state",
    "left_signed_transitions", "right_signed_transitions",
    "left_valid_transitions", "right_valid_transitions",
    "left_invalid_transitions", "right_invalid_transitions",
    "left_transition_period_us", "right_transition_period_us",
    "left_sc_transitions", "right_sc_transitions",
    "requested_left_percent", "requested_right_percent",
    "target_left_pwm", "target_right_pwm", "applied_left_pwm", "applied_right_pwm",
    "command_timed_out", "left_rpm_count", "right_rpm_count",
    "left_rpm_period", "right_rpm_period",
    "left_rpm_hybrid", "right_rpm_hybrid",
    "left_rpm_filtered", "right_rpm_filtered",
    "left_velocity_hybrid_mps", "right_velocity_hybrid_mps",
    "left_velocity_filtered_mps", "right_velocity_filtered_mps",
    "left_velocity_mode", "right_velocity_mode",
    "left_transition_age_ms", "right_transition_age_ms",
]
ARDUINO_FIELDS = FIELDS[1:]
FLOAT_FIELDS = {
    "left_rpm_count", "right_rpm_count", "left_rpm_period", "right_rpm_period",
    "left_rpm_hybrid", "right_rpm_hybrid", "left_rpm_filtered", "right_rpm_filtered",
    "left_velocity_hybrid_mps", "right_velocity_hybrid_mps",
    "left_velocity_filtered_mps", "right_velocity_filtered_mps",
}
MODE_NAMES = {0: "ZERO/STALE", 1: "PERIOD", 2: "COUNT-FALLBACK"}
stop_reader = threading.Event()
print_lock = threading.Lock()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    ports = list(list_ports.comports())
    if ports:
        print("Detected serial ports:")
        for p in ports:
            print(f"  {p.device}: {p.description}")
    print(f"Opening {port} at {BAUD} baud. Close other serial monitors first.")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.1, write_timeout=1)
    except serial.SerialException as exc:
        print(f"Could not open serial port: {exc}")
        return 1

    time.sleep(2.0)  # Leonardo may reset when the port opens.
    ser.reset_input_buffer()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = Path(f"r2_hall_hybrid_velocity_{stamp}.csv")
    csv_file = csv_path.open("w", newline="", buffering=1)
    writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
    writer.writeheader()
    command_path = Path(f"r2_hall_hybrid_commands_{stamp}.csv")
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
            if len(parts) != 36:
                counts["other"] += 1
                with print_lock:
                    print(f"Arduino: {line}")
                continue
            try:
                vals = [float(x.strip()) for x in parts]
                row_values = {}
                for key, value in zip(ARDUINO_FIELDS, vals):
                    row_values[key] = value if key in FLOAT_FIELDS else int(value)
            except ValueError:
                counts["malformed"] += 1
                with print_lock:
                    print(f"Unparsed line: {line}")
                continue
            writer.writerow({"host_time": datetime.now().isoformat(timespec="milliseconds"), **row_values})
            counts["telemetry"] += 1
            # Array indexes below refer to the 36 Arduino fields (not host_time).
            left_mode = int(vals[32])
            right_mode = int(vals[33])
            with print_lock:
                print(
                    f"t={int(vals[0]):8d} ms | signed L/R={int(vals[3]):7d}/{int(vals[4]):7d} | "
                    f"PWM L/R={int(vals[17]):3d}/{int(vals[18]):3d} | "
                    f"RPM count={vals[20]:7.2f}/{vals[21]:7.2f} | "
                    f"period={vals[22]:7.2f}/{vals[23]:7.2f} | "
                    f"HYBRID={vals[24]:7.2f}/{vals[25]:7.2f} | "
                    f"filtered={vals[26]:7.2f}/{vals[27]:7.2f} | "
                    f"mode={MODE_NAMES.get(left_mode, left_mode)}/{MODE_NAMES.get(right_mode, right_mode)} | "
                    f"age={int(vals[34])}/{int(vals[35])} ms"
                )

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    print(f"\nRecording telemetry: {csv_path.resolve()}")
    print(f"Recording commands:  {command_path.resolve()}")
    print("Commands: M <left -100..100> <right -100..100>, STOP, q")
    print("Mode 0=ZERO/STALE, 1=preferred transition-period estimate, 2=count-window fallback (used only before a valid period exists).")
    print("Period-preferred hybrid velocity is telemetry only; it is NOT connected to the balance controller.")
    print("Keep robot supported / wheels off ground for motor tests.\n")
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
        print(f"Parsed telemetry rows: {counts['telemetry']}; other lines: {counts['other']}; malformed rows: {counts['malformed']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())