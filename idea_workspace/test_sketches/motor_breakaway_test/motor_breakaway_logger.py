#!/usr/bin/env python3
"""Interactive CSV logger for R2 motor breakaway test (Arduino Leonardo)."""
import argparse
import csv
import sys
import time
import threading
from datetime import datetime

try:
    import serial
except ImportError:
    print("Missing pyserial. Install with: python3 -m pip install pyserial")
    raise SystemExit(1)

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("port", help="Serial port, e.g. /dev/ttyACM0")
p.add_argument("--baud", type=int, default=115200)
p.add_argument("--out", default=None, help="CSV output filename")
a = p.parse_args()
out = a.out or f"r2_motor_breakaway_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

print(f"Opening {a.port} at {a.baud} baud...")
ser = serial.Serial(a.port, a.baud, timeout=0.15, write_timeout=1)
time.sleep(2.0)  # Leonardo resets when serial port opens
ser.reset_input_buffer()

f = open(out, "w", newline="", buffering=1)
w = csv.writer(f)
w.writerow(["host_time", "arduino_time_ms", "left_cmd", "right_cmd", "left_edges", "right_edges", "raw_line"])
print(f"Logging to {out}")
print("Commands: L 6, L 8, L 10 ...; R -6, etc.; STOP; STATUS; QUIT")
print("Use small increments and wait for edge counts to change before increasing PWM.")

stop_reader = threading.Event()
print_lock = threading.Lock()

def reader_loop():
    while not stop_reader.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except Exception:
            if stop_reader.is_set():
                break
            continue
        if not line:
            continue
        with print_lock:
            print("\n" + line)
        fields = line.split(",")
        if len(fields) == 6 and fields[0] == "T":
            w.writerow([datetime.now().isoformat(timespec="milliseconds"), *fields[1:], line])

reader = threading.Thread(target=reader_loop, daemon=True)
reader.start()
try:
    while True:
        try:
            cmd = input("r2> ").strip()
        except EOFError:
            cmd = "QUIT"
        if not cmd:
            continue
        if cmd.upper() in ("QUIT", "EXIT"):
            ser.write(b"STOP\n")
            time.sleep(0.15)
            break
        ser.write((cmd + "\n").encode("ascii", errors="ignore"))
except KeyboardInterrupt:
    print("\nCtrl+C: sending STOP")
    try:
        ser.write(b"STOP\n")
        time.sleep(0.15)
    except Exception:
        pass

finally:
    stop_reader.set()
    reader.join(timeout=0.5)
    f.close()
    ser.close()
    print(f"Saved {out}")
