#!/usr/bin/env python3
"""Logger for R2 high-resolution IMU/balance diagnostics V5 direction-hysteresis.

Reads D (every third IMU update) and T (250 ms Hall/velocity summary) CSV records.
V4 D records add cumulative IMU count, updates since the prior D record,
and elapsed time between D records. No commands are sent to the robot.
"""
import argparse
import csv
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

D_FIELDS = [
    "record", "seq", "timestamp_us", "dt_us", "raw_gyro_y_dps",
    "gyro_bias_dps", "pitch_rate_dps", "accel_pitch_deg", "fused_pitch_deg",
    "p_term", "d_term", "unsaturated_output", "controller_output",
    "target_motor", "selected_direction", "hysteresis_hold", "left_motor",
    "right_motor", "balance_enabled", "fault_code", "imu_updates_window",
    "queue_drops", "imu_total",
    "imu_updates_since_diag", "diag_interval_us",
]
T_FIELDS = [
    "record", "time_ms", "imu_updates_window", "left_signed", "right_signed",
    "left_valid", "left_invalid", "right_valid", "right_invalid",
    "left_period_us", "right_period_us", "left_rpm_period", "right_rpm_period",
    "left_rpm_filtered", "right_rpm_filtered", "left_velocity_mps",
    "right_velocity_mps", "left_mode", "right_mode", "left_age_ms",
    "right_age_ms", "diag_queue_drops", "summary_drops",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200,
                        help="USB CDC baud setting (default: 115200)")
    args = parser.parse_args()

    ports = list(list_ports.comports())
    if ports:
        print("Detected serial ports:")
        for p in ports:
            print(f"  {p.device}: {p.description}")
    print(f"Opening {args.port} at {args.baud}. Close Arduino Serial Monitor first.")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"Could not open serial port: {exc}", file=sys.stderr)
        return 1

    time.sleep(2.0)  # Leonardo may reset when the USB serial port opens.
    ser.reset_input_buffer()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    d_path = Path(f"r2_imu_hysteresis_{stamp}.csv")
    t_path = Path(f"r2_hall_hysteresis_{stamp}.csv")
    stop = threading.Event()
    counts = {"D": 0, "T": 0, "bad": 0, "other": 0}
    last_seq = None
    sequence_gaps = 0
    last_reported_queue_drops = 0
    lock = threading.Lock()

    # Use substantial file buffers so per-record disk flushes do not slow serial intake.
    with d_path.open("w", newline="", buffering=1024 * 1024) as df, \
         t_path.open("w", newline="", buffering=256 * 1024) as tf:
        dw = csv.DictWriter(df, fieldnames=D_FIELDS)
        tw = csv.DictWriter(tf, fieldnames=T_FIELDS)
        dw.writeheader()
        tw.writeheader()

        print(f"High-resolution IMU log: {d_path.resolve()}")
        print(f"Hall summary log:        {t_path.resolve()}")
        print("Press Ctrl+C to stop recording. This logger sends no motor commands.")
        try:
            while not stop.is_set():
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                parts = line.split(",")
                tag = parts[0]
                try:
                    if tag == "D" and len(parts) == len(D_FIELDS):
                        row = dict(zip(D_FIELDS, parts))
                        dw.writerow(row)
                        counts["D"] += 1
                        seq = int(parts[1])
                        if last_seq is not None and seq > last_seq + 1:
                            sequence_gaps += seq - last_seq - 1
                        last_seq = seq
                        queue_drops = int(parts[21])
                        last_reported_queue_drops = max(last_reported_queue_drops, queue_drops)
                        if counts["D"] % 250 == 0:
                            print(f"Diagnostic records: {counts['D']} | latest dt={parts[3]} us | "
                                  f"pitch={parts[8]} deg | output={parts[12]} | dir={parts[14]} | hold={parts[15]} | "
                                  f"seq gaps={sequence_gaps} | Arduino queue drops={last_reported_queue_drops} | "
                                  f"IMU total={parts[22]} | updates/D={parts[23]} | "
                                  f"D interval={parts[24]} us")
                    elif tag == "T" and len(parts) == len(T_FIELDS):
                        tw.writerow(dict(zip(T_FIELDS, parts)))
                        counts["T"] += 1
                    elif tag in ("D", "T"):
                        counts["bad"] += 1
                        print(f"Malformed {tag} record ({len(parts)} fields): {line}")
                    else:
                        counts["other"] += 1
                        print(f"Arduino: {line}")
                except (ValueError, csv.Error) as exc:
                    counts["bad"] += 1
                    print(f"Could not parse record: {exc}: {line}")
        except KeyboardInterrupt:
            print("\nStopping logger...")
        finally:
            stop.set()
            ser.close()
            df.flush()
            tf.flush()
            print(f"Saved {counts['D']} diagnostic samples and {counts['T']} Hall summaries.")
            print(f"Sequence gaps observed: {sequence_gaps}; latest Arduino queue-drop count: {last_reported_queue_drops}")
            print(f"Malformed records: {counts['bad']}; other lines: {counts['other']}")
            print(f"IMU CSV:  {d_path.resolve()}")
            print(f"Hall CSV: {t_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
