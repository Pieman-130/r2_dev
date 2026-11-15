#!/usr/bin/env python3
"""
R2 balance + encoder observation logger.

The Leonardo uses ONLY the original balance controller in this test.
Encoder measurements are recorded but do not influence the motors.
"""

import csv
import datetime as dt
import time
import serial

PORT = "/dev/ttyACM0"
BAUD = 115200

FIELDS = [
    "time_ms", "pitch", "pitchRate", "rawGyroY",
    "gyroMin", "gyroMax", "gyroAvg", "gyroRange", "accelPitch",
    "controllerOutput", "wheelRPMMagnitude", "targetMotor",
    "currentLeftMotor", "currentRightMotor", "balanceEnabled",
    "balanceFault", "faultCode", "rightRPM", "leftRPM",
    "rightTransitionCount", "leftTransitionCount",
    "rightPositionTransitions", "leftPositionTransitions",
    "rightPeriodUs", "leftPeriodUs",
]

def parse_csv(line):
    parts = line.split(",")
    if len(parts) != len(FIELDS):
        return None
    try:
        return [float(x) for x in parts]
    except ValueError:
        return None

def main():
    filename = (
        "r2_balance_encoder_observation_"
        + dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        + ".csv"
    )
    print(f"Opening {PORT} at {BAUD} baud...")
    print(f"Logging to {filename}")
    print("ENCODERS ARE OBSERVATION ONLY — NO ENCODER FEEDBACK TO MOTORS")
    print("Press Ctrl-C to stop.\n")

    ser = serial.Serial(PORT, BAUD, timeout=0.25)
    time.sleep(2.0)
    ser.reset_input_buffer()

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(FIELDS)
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                row = parse_csv(line)
                if row is None:
                    if line:
                        print(line)
                    continue
                writer.writerow(row)
                f.flush()
                print(
                    f"pitch={row[1]:7.2f}  "
                    f"rate={row[2]:7.2f}  "
                    f"ctrl={row[9]:6.2f}  "
                    f"motor={row[12]:3.0f}/{row[13]:3.0f}  "
                    f"RPM R/L={row[17]:6.2f}/{row[18]:6.2f}  "
                    f"counts R/L={row[19]:5.0f}/{row[20]:5.0f}  "
                    f"bal={row[14]:.0f} fault={row[15]:.0f}"
                )
        except KeyboardInterrupt:
            print("\nStopping logger.")
        finally:
            ser.close()

if __name__ == "__main__":
    main()
