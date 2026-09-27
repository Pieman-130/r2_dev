import serial
import threading
import time
import csv

PORT = "/dev/ttyACM0"
BAUD = 115200

OUTPUT_FILE = "motor_response_short_pulses.csv"

COMMAND_INTERVAL = 0.10

COMMAND_LEN = 5.0


# ============================================================
# CRC-8
# ============================================================

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


# ============================================================
# MOTOR PACKET
# ============================================================

def make_packet(left, right):

    left = max(-100, min(100, int(left)))
    right = max(-100, min(100, int(right)))

    packet = bytearray()

    packet.append(0xAA)
    packet.append(0x55)
    packet.append(4)
    packet.append(0x01)

    packet.append(left & 0xFF)
    packet.append(right & 0xFF)

    packet.append(crc8(packet))

    return bytes(packet)


# ============================================================
# SERIAL READER
# ============================================================

def serial_reader(ser, records, running):

    while running.is_set():

        try:
            line = ser.readline()

            if not line:
                continue

            text = line.decode(
                "utf-8",
                errors="replace"
            ).strip()

            parts = text.split(",")

            if len(parts) != 6:
                continue

            if parts[0] == "time_ms":
                continue

            try:
                records.append([
                    int(parts[0]),
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                    int(parts[4]),
                    int(parts[5])
                ])

            except ValueError:
                continue

        except serial.SerialException:
            break


# ============================================================
# COMMAND SENDER
# ============================================================

def command_sender(ser, command, running):

    while running.is_set():

        try:
            left, right = command

            ser.write(make_packet(left, right))
            ser.flush()

        except serial.SerialException:
            break

        time.sleep(COMMAND_INTERVAL)


# ============================================================
# SET COMMAND
# ============================================================

def set_command(command, left, right):

    command[0] = left
    command[1] = right


# ============================================================
# RUN TEST
# ============================================================

def run_test(command):

    print()
    print("========================================")
    print("       SHORT MOTOR PULSE TEST")
    print("========================================")
    print()
    print("Make sure the robot is secured so that")
    print("it can pitch forward safely.")
    print()
    print("Sequence:")
    print()
    print("  5 sec   stationary")
    print(f"  {COMMAND_LEN} s  forward")
    print("  5 sec   stationary")
    print(f"  {COMMAND_LEN} s  forward")
    print("  5 sec   stationary")
    print()
    print("Motor command: +25 / +25")
    print()

    input("Press ENTER when ready...")

    print()
    print("Starting in 3...")
    time.sleep(1)

    print("2...")
    time.sleep(1)

    print("1...")
    time.sleep(1)

    print("GO!")
    print()

    # --------------------------------------------------------
    # Initial stationary period
    # --------------------------------------------------------

    set_command(command, 0, 0)

    print("Stationary")
    time.sleep(5)

    # --------------------------------------------------------
    # Pulse 1
    # --------------------------------------------------------

    print("FORWARD PULSE 1")

    set_command(command, 25, 25)

    time.sleep(COMMAND_LEN)

    set_command(command, 0, 0)

    print("STOP")
    time.sleep(5)

    # --------------------------------------------------------
    # Pulse 2
    # --------------------------------------------------------

    #print("FORWARD PULSE 2")

    #set_command(command, 25, 25)

    #time.sleep(0.25)

    #set_command(command, 0, 0)

    #print("STOP")
    #time.sleep(5)

    # --------------------------------------------------------
    # Final stop
    # --------------------------------------------------------

    set_command(command, 0, 0)

    print()
    print("TEST COMPLETE")


# ============================================================
# MAIN
# ============================================================

def main():

    print(f"Opening {PORT}...")

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.1
    )

    # Leonardo resets when serial connection opens.
    time.sleep(2)

    print("Connected.")

    command = [0, 0]

    running = threading.Event()
    running.set()

    records = []

    reader = threading.Thread(
        target=serial_reader,
        args=(ser, records, running)
    )

    sender = threading.Thread(
        target=command_sender,
        args=(ser, command, running)
    )

    reader.start()
    sender.start()

    try:

        run_test(command)

    except KeyboardInterrupt:

        print()
        print("TEST INTERRUPTED")

    finally:

        # Stop motors
        set_command(command, 0, 0)

        time.sleep(0.2)

        # Stop threads
        running.clear()

        sender.join(timeout=1)
        reader.join(timeout=1)

        ser.close()

    # ========================================================
    # SAVE DATA
    # ========================================================

    print()
    print(f"Saving {len(records)} telemetry records...")

    with open(
        OUTPUT_FILE,
        "w",
        newline=""
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            "time_ms",
            "pitch",
            "pitchRate",
            "accelPitch",
            "left",
            "right"
        ])

        writer.writerows(records)

    print(f"Saved to: {OUTPUT_FILE}")
    print()


if __name__ == "__main__":
    main()