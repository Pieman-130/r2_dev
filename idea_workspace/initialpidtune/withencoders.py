import serial
import struct
import time
import csv
import threading
import math

# ============================================================
# Configuration
# ============================================================

BALANCE_PORT = "/dev/ttyACM0"
SENSOR_PORT = "/dev/SENS0"

BAUD_BALANCE = 115200
BAUD_SENSOR = 115200

OUTPUT_FILE = "balance_encoder_test_v2.csv"

# ============================================================
# Global sensor state
# ============================================================

sensor_lock = threading.Lock()

latest_sensor = None

sensor_packet_count = 0

right_encoder_last_value = None
left_encoder_last_value = None

right_encoder_last_change_time = None
left_encoder_last_change_time = None


# ============================================================
# Sensor packet parser
# ============================================================

def parse_sensor_packet(packet):
    """
    Sensor packet:

        AA 55
        28-byte payload
        checksum
        55 AA

    Payload = 7 little-endian int32 values:

        0 left ultrasonic
        1 right ultrasonic
        2 rear ultrasonic
        3 front-down IR
        4 rear-down IR
        5 right wheel encoder
        6 left wheel encoder
    """

    if len(packet) != 33:
        return None

    if packet[0] != 0xAA or packet[1] != 0x55:
        return None

    if packet[31] != 0x55 or packet[32] != 0xAA:
        return None

    payload = packet[2:30]
    checksum = packet[30]

    calc_checksum = 0
    for b in payload:
        calc_checksum ^= b

    if calc_checksum != checksum:
        return None

    values = struct.unpack("<7i", payload)

    return {
        "leftUltrasonic": values[0],
        "rightUltrasonic": values[1],
        "rearUltrasonic": values[2],
        "frontIR": values[3],
        "rearIR": values[4],
        "rightEncoder": values[5],
        "leftEncoder": values[6],
    }


# ============================================================
# RPM calculation
# ============================================================

def encoder_to_rpm(value):
    """
    Encoder conversion supplied by sensor MCU:

        RPM = 1 / ((data / 10000000) * 90) * 60

    Equivalent to approximately:

        RPM = 666,666.6667 / data
    """

    if value <= 0:
        return 0.0

    return 666666.6667 / value


# ============================================================
# Sensor reader thread
# ============================================================

def sensor_reader():
    global latest_sensor
    global sensor_packet_count
    global right_encoder_last_value
    global left_encoder_last_value
    global right_encoder_last_change_time
    global left_encoder_last_change_time

    ser = serial.Serial(
        SENSOR_PORT,
        BAUD_SENSOR,
        timeout=0.05
    )

    buffer = bytearray()

    print(f"Sensor reader connected to {SENSOR_PORT}")

    while True:

        data = ser.read(128)

        if data:
            buffer.extend(data)

        while len(buffer) >= 33:

            # Find packet preamble
            if buffer[0] != 0xAA or buffer[1] != 0x55:
                del buffer[0]
                continue

            packet = bytes(buffer[:33])

            parsed = parse_sensor_packet(packet)

            if parsed is None:
                # Bad packet; shift by one byte and try again
                del buffer[0]
                continue

            # Valid packet
            del buffer[:33]

            now = time.monotonic()

            sensor_packet_count += 1

            right_encoder = parsed["rightEncoder"]
            left_encoder = parsed["leftEncoder"]

            # Detect encoder value changes
            if right_encoder_last_value != right_encoder:
                right_encoder_last_change_time = now
                right_encoder_last_value = right_encoder

            if left_encoder_last_value != left_encoder:
                left_encoder_last_change_time = now
                left_encoder_last_value = left_encoder

            with sensor_lock:

                latest_sensor = {
                    **parsed,

                    "receive_time": now,

                    "packet_count": sensor_packet_count,

                    "rightRPM_raw":
                        encoder_to_rpm(right_encoder),

                    "leftRPM_raw":
                        encoder_to_rpm(left_encoder),

                    "right_encoder_last_change":
                        right_encoder_last_change_time,

                    "left_encoder_last_change":
                        left_encoder_last_change_time,
                }


# ============================================================
# Balance telemetry parser
# ============================================================

def parse_balance_line(line):

    """
    Expected balance telemetry:

    15 comma-separated fields:

    1  time_ms
    2  pitch
    3  pitchRate
    4  rawGyroY
    5  gyroMin
    6  gyroMax
    7  gyroAvg
    8  gyroRange
    9  accelPitch
    10 controllerOutput
    11 targetMotor
    12 currentLeftMotor
    13 currentRightMotor
    14 balanceEnabled
    15 balanceFault
    16 faultCode

    NOTE:
    This is actually 16 fields because left and right motor
    commands are logged separately.
    """

    parts = line.strip().split(",")

    if len(parts) != 16:
        return None

    try:

        return {
            "arduino_time_ms": int(parts[0]),

            "pitch": float(parts[1]),
            "pitchRate": float(parts[2]),
            "rawGyroY": float(parts[3]),

            "gyroMin": float(parts[4]),
            "gyroMax": float(parts[5]),
            "gyroAvg": float(parts[6]),
            "gyroRange": float(parts[7]),

            "accelPitch": float(parts[8]),

            "controllerOutput": float(parts[9]),

            "targetMotor": float(parts[10]),

            "currentLeftMotor": float(parts[11]),
            "currentRightMotor": float(parts[12]),

            "balanceEnabled": int(parts[13]),
            "balanceFault": int(parts[14]),
            "faultCode": int(parts[15]),
        }

    except ValueError:
        return None


# ============================================================
# Main logger
# ============================================================

def main():

    global latest_sensor

    # Start sensor reader
    sensor_thread = threading.Thread(
        target=sensor_reader,
        daemon=True
    )

    sensor_thread.start()

    time.sleep(0.5)

    balance_ser = serial.Serial(
        BALANCE_PORT,
        BAUD_BALANCE,
        timeout=0.1
    )

    print(f"Balance reader connected to {BALANCE_PORT}")
    print(f"Logging to {OUTPUT_FILE}")

    fieldnames = [

        # Timing
        "python_time_ms",
        "arduino_time_ms",

        # Balance state
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

        # Sensor packet information
        "sensorPacketCount",
        "sensorAgeMs",

        # Encoder values
        "rightEncoder",
        "leftEncoder",

        # RPM magnitude only
        "rightRPM_raw",
        "leftRPM_raw",

        # Encoder freshness
        "rightEncoderAgeMs",
        "leftEncoderAgeMs",

        # Whether encoder value changed since previous balance sample
        "rightEncoderUpdated",
        "leftEncoderUpdated",
    ]

    with open(
        OUTPUT_FILE,
        "w",
        newline=""
    ) as csvfile:

        writer = csv.DictWriter(
            csvfile,
            fieldnames=fieldnames
        )

        writer.writeheader()

        previous_packet_count = None
        previous_right_encoder = None
        previous_left_encoder = None

        while True:

            line = balance_ser.readline()

            if not line:
                continue

            try:
                line = line.decode(
                    "utf-8",
                    errors="ignore"
                )
            except Exception:
                continue

            balance = parse_balance_line(line)

            if balance is None:
                continue

            now = time.monotonic()

            # Copy sensor state safely
            with sensor_lock:

                sensor = (
                    dict(latest_sensor)
                    if latest_sensor is not None
                    else None
                )

            row = {}

            # ------------------------------------------------
            # Balance data
            # ------------------------------------------------

            row["python_time_ms"] = int(
                time.time() * 1000
            )

            row.update(balance)

            # ------------------------------------------------
            # Sensor data
            # ------------------------------------------------

            if sensor is None:

                row["sensorPacketCount"] = 0
                row["sensorAgeMs"] = ""

                row["rightEncoder"] = ""
                row["leftEncoder"] = ""

                row["rightRPM_raw"] = ""
                row["leftRPM_raw"] = ""

                row["rightEncoderAgeMs"] = ""
                row["leftEncoderAgeMs"] = ""

                row["rightEncoderUpdated"] = 0
                row["leftEncoderUpdated"] = 0

            else:

                packet_count = sensor["packet_count"]

                row["sensorPacketCount"] = packet_count

                # Age of newest sensor packet
                row["sensorAgeMs"] = (
                    now - sensor["receive_time"]
                ) * 1000.0

                right_encoder = sensor["rightEncoder"]
                left_encoder = sensor["leftEncoder"]

                row["rightEncoder"] = right_encoder
                row["leftEncoder"] = left_encoder

                # RPM magnitude only.
                # NO direction is inferred here.
                row["rightRPM_raw"] = sensor["rightRPM_raw"]
                row["leftRPM_raw"] = sensor["leftRPM_raw"]

                # Time since the encoder measurement itself changed
                if sensor["right_encoder_last_change"] is not None:
                    row["rightEncoderAgeMs"] = (
                        now -
                        sensor["right_encoder_last_change"]
                    ) * 1000.0
                else:
                    row["rightEncoderAgeMs"] = ""

                if sensor["left_encoder_last_change"] is not None:
                    row["leftEncoderAgeMs"] = (
                        now -
                        sensor["left_encoder_last_change"]
                    ) * 1000.0
                else:
                    row["leftEncoderAgeMs"] = ""

                # Did the actual encoder value change since
                # the previous balance telemetry row?
                if previous_right_encoder is None:
                    row["rightEncoderUpdated"] = 0
                else:
                    row["rightEncoderUpdated"] = int(
                        right_encoder != previous_right_encoder
                    )

                if previous_left_encoder is None:
                    row["leftEncoderUpdated"] = 0
                else:
                    row["leftEncoderUpdated"] = int(
                        left_encoder != previous_left_encoder
                    )

                previous_right_encoder = right_encoder
                previous_left_encoder = left_encoder

                previous_packet_count = packet_count

            writer.writerow(row)
            csvfile.flush()


# ============================================================
# Start
# ============================================================

if __name__ == "__main__":
    main()