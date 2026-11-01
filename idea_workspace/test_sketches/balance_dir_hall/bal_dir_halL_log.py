#!/usr/bin/env python3
"""CSV logger for r2_balance_velocity_telemetry.ino (34 Arduino fields)."""
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
    "host_time", "time_ms", "pitch_deg", "pitch_rate_dps", "raw_gyro_y",
    "gyro_min", "gyro_max", "gyro_avg", "gyro_range", "accel_pitch_deg",
    "controller_output", "target_motor", "left_motor", "right_motor",
    "balance_enabled", "balance_fault", "fault_code",
    "left_signed_transitions", "right_signed_transitions",
    "left_valid", "left_invalid", "right_valid", "right_invalid",
    "left_period_us", "right_period_us", "left_rpm_period", "right_rpm_period",
    "left_rpm_filtered", "right_rpm_filtered",
    "left_velocity_filtered_mps", "right_velocity_filtered_mps",
    "left_velocity_mode", "right_velocity_mode",
    "left_transition_age_ms", "right_transition_age_ms",
]
ARDUINO_FIELDS = FIELDS[1:]
INT_FIELDS = {
    "time_ms", "target_motor", "left_motor", "right_motor", "balance_enabled",
    "balance_fault", "fault_code", "left_signed_transitions", "right_signed_transitions",
    "left_valid", "left_invalid", "right_valid", "right_invalid",
    "left_period_us", "right_period_us", "left_velocity_mode", "right_velocity_mode",
    "left_transition_age_ms", "right_transition_age_ms",
}
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
        ser = serial.Serial(port, BAUD, timeout=0.2, write_timeout=1)
    except serial.SerialException as exc:
        print(f"Could not open serial port: {exc}")
        return 1

    time.sleep(2.0)  # Leonardo may reset when serial port opens.
    ser.reset_input_buffer()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = Path(f"r2_balance_velocity_{stamp}.csv")
    f = path.open("w", newline="", buffering=1)
    writer = csv.DictWriter(f, fieldnames=FIELDS)
    writer.writeheader()
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
            if len(parts) != len(ARDUINO_FIELDS):
                counts["other"] += 1
                with print_lock:
                    print(f"Arduino: {line}")
                continue
            try:
                values = [float(x.strip()) for x in parts]
                row = {"host_time": datetime.now().isoformat(timespec="milliseconds")}
                for key, value in zip(ARDUINO_FIELDS, values):
                    row[key] = int(value) if key in INT_FIELDS else value
            except ValueError:
                counts["malformed"] += 1
                with print_lock:
                    print(f"Unparsed line: {line}")
                continue
            writer.writerow(row)
            counts["telemetry"] += 1
            with print_lock:
                print(
                    f"t={row['time_ms']:8d} ms | pitch={row['pitch_deg']:7.2f} deg | "
                    f"rate={row['pitch_rate_dps']:7.2f} deg/s | "
                    f"motor L/R={row['left_motor']:3d}/{row['right_motor']:3d} | "
                    f"velocity filt L/R={row['left_velocity_filtered_mps']:+.3f}/"
                    f"{row['right_velocity_filtered_mps']:+.3f} m/s | "
                    f"RPM L/R={row['left_rpm_filtered']:+.1f}/{row['right_rpm_filtered']:+.1f} | "
                    f"mode L/R={row['left_velocity_mode']}/{row['right_velocity_mode']} | "
                    f"balance={row['balance_enabled']} fault={row['balance_fault']}"
                )

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    print(f"\nRecording balance telemetry: {path.resolve()}")
    print("This logger is read-only; it sends no motor commands.")
    print("Press Ctrl+C to stop. Keep the robot in your normal safe test setup.")
    try:
        while not stop_reader.is_set():
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop_reader.set()
        thread.join(timeout=1.0)
        f.close()
        ser.close()
        print(f"\nSaved: {path.resolve()}")
        print(f"Parsed telemetry rows: {counts['telemetry']}; other lines: {counts['other']}; malformed rows: {counts['malformed']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())