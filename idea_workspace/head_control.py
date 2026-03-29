import serial
import struct
import threading
import time
import sys

PREAMBLE = 0xAA
POSTAMBLE = 0x55

PORT = "/dev/servo"   # Change if needed
BAUD = 9600


# ---------------- CRC ----------------
def compute_crc(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc


# ---------------- Send Command ----------------
def send_pitch(ser, pitch: float):
    payload = struct.pack("<f", pitch)  # little-endian float
    crc = compute_crc(payload)

    packet = struct.pack("<B", PREAMBLE)
    packet += payload
    packet += struct.pack("<B", crc)
    packet += struct.pack("<B", POSTAMBLE)

    ser.write(packet)


# ---------------- Read Telemetry ----------------
def read_telemetry(ser):
    while True:
        if ser.read(1) == bytes([PREAMBLE]):
            payload = ser.read(8)
            crc = ser.read(1)
            post = ser.read(1)

            if len(payload) != 8:
                continue

            if post != bytes([POSTAMBLE]):
                continue

            if compute_crc(payload) != crc[0]:
                print("Bad CRC — packet rejected")
                continue

            pitch, distance = struct.unpack("<ff", payload)

            print(f"Pitch: {pitch:7.2f} deg | Distance: {distance:7.1f} mm")
            time.sleep(1)


# ---------------- Main ----------------
def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        time.sleep(2)  # wait for Arduino reset
    except Exception as e:
        print("Serial error:", e)
        sys.exit(1)

    # Start telemetry reader thread
    t = threading.Thread(target=read_telemetry, args=(ser,), daemon=True)
    t.start()

    print("Enter desired pitch angle (-45 to +65). Ctrl+C to quit.\n")

    while True:
        try:
            user_input = input("Pitch> ")
            pitch = float(user_input)

            send_pitch(ser, pitch)

        except ValueError:
            print("Invalid number")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

    ser.close()


if __name__ == "__main__":
    main()
