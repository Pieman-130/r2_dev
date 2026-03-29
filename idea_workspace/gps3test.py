import serial
import struct

PORT = "/dev/GPS0"
BAUD = 4800   # SiRF binary default
TIMEOUT = 1

ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)

def read_sirf_packet():
    # sync to start sequence A0 A2
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b == b'\xA0' and ser.read(1) == b'\xA2':
            break

    length_bytes = ser.read(2)
    if len(length_bytes) < 2:
        return None

    length = struct.unpack(">H", length_bytes)[0]
    payload = ser.read(length)
    checksum = ser.read(2)
    end = ser.read(2)

    if end != b'\xB0\xB3':
        return None

    return payload

def parse_mid_2(payload):
    # MID 2: Navigation Solution
    # See SiRF Binary Protocol Reference
    # Offsets are fixed
    try:
        lat = struct.unpack(">i", payload[1:5])[0] * 1e-7
        lon = struct.unpack(">i", payload[5:9])[0] * 1e-7
        alt = struct.unpack(">i", payload[9:13])[0] / 100.0
        fix = payload[13]
        sats = payload[14]

        return {
            "lat": lat,
            "lon": lon,
            "alt_m": alt,
            "fix": fix,
            "sats": sats
        }
    except Exception:
        return None

print("Reading SiRF binary data...")

while True:
    pkt = read_sirf_packet()
    if not pkt:
        continue

    mid = pkt[0]

    if mid == 2:
        nav = parse_mid_2(pkt)
        if nav:
            print(
                f"LAT={nav['lat']:.7f}, "
                f"LON={nav['lon']:.7f}, "
                f"ALT={nav['alt_m']:.1f}m, "
                f"FIX={nav['fix']}, "
                f"SATS={nav['sats']}"
            )
