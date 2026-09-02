import serial
import time


# Change this to your Metro Mini's serial port.
#
# Windows example:
#   COM5
#
# macOS example:
#   /dev/cu.usbmodemXXXX
#
# Linux example:
#   /dev/ttyACM0

PORT = "/dev/ttyACM0"

BAUD = 115200


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

    left = max(-100, min(100, left))
    right = max(-100, min(100, right))

    # Convert signed values to bytes.
    left_byte = left & 0xFF
    right_byte = right & 0xFF

    command = 0x01

    # CRC covers CMD + LEFT + RIGHT
    crc_data = bytes([
        command,
        left_byte,
        right_byte
    ])

    checksum = crc8(crc_data)

    packet = bytes([
        0xAA,
        0x55,
        0x04,
        command,
        left_byte,
        right_byte,
        checksum
    ])

    return packet


def send_command(ser, left, right):

    packet = make_packet(left, right)

    print(
        f"LEFT={left:+4d} "
        f"RIGHT={right:+4d} "
        f"PACKET={packet.hex(' ')}"
    )

    ser.write(packet)


def main():

    print("Opening serial port...")

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.1
    )

    # Opening the serial port can reset an Arduino.
    time.sleep(2)

    print()
    print("Two-wheel robot motor test")
    print()
    print("Commands:")
    print("  l 20       Left motor forward 20%")
    print("  l -20      Left motor reverse 20%")
    print("  r 20       Right motor forward 20%")
    print("  r -20      Right motor reverse 20%")
    print("  both 20    Both forward 20%")
    print("  both -20   Both reverse 20%")
    print("  stop       Stop")
    print("  quit       Exit")
    print()

    left = 0
    right = 0

    try:

        while True:

            command = input("> ").strip().lower()

            if not command:
                continue

            if command == "quit":
                break

            elif command == "stop":
                left = 0
                right = 0

            else:

                parts = command.split()

                if len(parts) != 2:
                    print("Invalid command.")
                    continue

                try:
                    value = int(parts[1])
                except ValueError:
                    print("Speed must be an integer.")
                    continue

                value = max(-100, min(100, value))

                if parts[0] == "l":
                    left = value

                elif parts[0] == "r":
                    right = value

                elif parts[0] == "both":
                    left = value
                    right = value

                else:
                    print("Unknown command.")
                    continue

            send_command(ser, left, right)

    finally:

        # Always attempt to stop the robot before exiting.
        send_command(ser, 0, 0)

        time.sleep(0.5)

        ser.close()

        print("Serial port closed.")


if __name__ == "__main__":
    main()