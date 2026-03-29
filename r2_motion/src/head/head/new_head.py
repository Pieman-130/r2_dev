"""
robot_head_controller.py
Sends servo angle commands and receives telemetry (distance + pitch) from Arduino.

Packet formats:
  Command  (host → Arduino):  [0xAA, 0x01, angle_byte, xor_checksum]
  Telemetry (Arduino → host): [0xBB, dist_hi, dist_lo, pitch_hi, pitch_lo, xor_checksum]
"""

import serial
import struct
import time
import threading
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
PORT      = "/dev/servo"      # Change to your port, e.g. "/dev/ttyUSB0" on Linux
BAUD      = 9600
CMD_HEADER = 0xAA
CMD_TYPE   = 0x01
TEL_HEADER = 0xBB
TEL_LEN    = 6          # bytes per telemetry packet

# Servo range matching the Arduino constants
SERVO_MIN    = 130
SERVO_MAX    = 220
SERVO_CENTER = 154

# Pitch angle limits corresponding to servo limits
PITCH_MIN    = -34   # corresponds to SERVO_MIN 130
PITCH_MAX    =  96   # corresponds to SERVO_MAX 220
PITCH_CENTER =   0   # corresponds to SERVO_CENTER 154

def pitch_to_servo(pitch_angle: float) -> int:
    """Map a real pitch angle to a servo value, clamped to safe range."""
    servo = np.interp(pitch_angle, [PITCH_MIN, PITCH_MAX], [SERVO_MIN, SERVO_MAX])
    return int(round(servo))


# ── Checksum ──────────────────────────────────────────────────────────────────
def xor_checksum(data: bytes) -> int:
    cs = 0
    for b in data:
        cs ^= b
    return cs


# ── Command builder ───────────────────────────────────────────────────────────
def build_servo_command(angle: int) -> bytes:
    if angle < SERVO_MIN:
        angle = SERVO_MIN
    elif angle > SERVO_MAX:
        angle = SERVO_MAX
    payload = bytes([CMD_HEADER, CMD_TYPE, angle])
    cs = xor_checksum(payload)
    return payload + bytes([cs])


# ── Telemetry parser ──────────────────────────────────────────────────────────
def parse_telemetry(pkt: bytes):
    """
    Returns (distance_m, pitch_deg) or None if checksum fails.
    pkt must be 6 bytes starting with 0xBB.
    """
    if len(pkt) != TEL_LEN or pkt[0] != TEL_HEADER:
        return None
    expected_cs = xor_checksum(pkt[:5])
    if pkt[5] != expected_cs:
        print(f"[WARN] Bad telemetry checksum (got {pkt[5]:02X}, expected {expected_cs:02X})")
        return None
    dist_mm   = struct.unpack(">H", pkt[1:3])[0]   # unsigned 16-bit
    pitch_raw = struct.unpack(">h", pkt[3:5])[0]   # signed 16-bit
    dist_m    = dist_mm / 1000.0
    pitch_deg = pitch_raw / 10.0
    return dist_m, pitch_deg


# ── Telemetry reader (runs in background thread) ──────────────────────────────
class TelemetryReader(threading.Thread):
    def __init__(self, ser: serial.Serial):
        super().__init__(daemon=True)
        self.ser   = ser
        self.dist  = None
        self.pitch = None
        self._buf  = bytearray()

    def run(self):
        while True:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            self._buf.extend(chunk)

            # Scan for 0xBB header and extract complete packets
            while True:
                idx = self._buf.find(TEL_HEADER)
                if idx == -1:
                    self._buf.clear()
                    break
                if idx > 0:
                    del self._buf[:idx]   # discard bytes before header

                if len(self._buf) < TEL_LEN:
                    break                 # wait for more data

                pkt = bytes(self._buf[:TEL_LEN])
                del self._buf[:TEL_LEN]

                result = parse_telemetry(pkt)
                if result:
                    self.dist, self.pitch = result
                    self.pkt = pkt


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        time.sleep(2)           # let Arduino reset after connection
        ser.reset_input_buffer()

        reader = TelemetryReader(ser)
        reader.start()
        print("Connected. Commands: enter angle (130-220), 'c' for center, 'q' to quit.\n")

        while True:
            user_in = input("Pitch angle > ").strip().lower()

            if user_in == 'q':
                print("Exiting.")
                break

            if user_in == 'c':
                pitch_target = PITCH_CENTER
            else:
                try:
                    pitch_target = float(user_in)   # float so e.g. 12.5 works
                except ValueError as e:
                    print(f"  Parse error: {e!r} on input {user_in!r}")
                    continue

            # Clamp pitch to valid range before converting
            pitch_target = max(PITCH_MIN, min(PITCH_MAX, pitch_target))
            angle = pitch_to_servo(pitch_target)

            cmd = build_servo_command(angle)
            ser.write(cmd)
            ser.flush()
            print(f"  → Pitch {pitch_target}° → servo value {angle}  (packet: {cmd.hex(' ').upper()})")

            time.sleep(0.3)
            if reader.dist is not None:
                print(f"  ← Distance: {reader.dist:.3f} m  |  Pitch: {reader.pitch:.1f}°")
            else:
                print("  ← Waiting for telemetry…")


if __name__ == "__main__":
    main()