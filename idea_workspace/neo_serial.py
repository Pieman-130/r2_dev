import serial, time, random

PORT = '/dev/neopixel'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

def send_led_frame(colors):
    """Send 16 RGB tuples as an LED frame update."""
    if len(colors) != 16:
        raise ValueError("Must send exactly 16 RGB values")

    cmd = 0x01
    data = bytearray([0xAA, cmd])
    checksum = cmd

    for r, g, b in colors:
        for val in (r, g, b):
            data.append(val)
            checksum = (checksum + val) & 0xFF

    data.append(checksum)
    ser.write(data)

def send_clear():
    """Clear the strip."""
    cmd = 0x02
    checksum = cmd & 0xFF
    ser.write(bytes([0xAA, cmd, checksum]))

def send_brightness(level):
    """Set brightness 0–255."""
    cmd = 0x03
    level = max(0, min(255, level))
    checksum = (cmd + level) & 0xFF
    ser.write(bytes([0xAA, cmd, level, checksum]))

# === Example usage ===
for i in range(10):
    frame = [[255,0,0] for _ in range(16)]
    send_led_frame(frame)
    time.sleep(.05)
    frame =[[0,255,0] for _ in range(16)]
    send_led_frame(frame)
    time.sleep(0.05) 
    frame =[[0,0,0] for _ in range(16)]
    send_led_frame(frame)
    time.sleep(0.05) 
    print("Arduino:", ser.readline().decode().strip())
'''
time.sleep(1)
send_brightness(80)
print("Set brightness")
print("Arduino:", ser.readline().decode().strip())
'''
#time.sleep(1)
#send_clear()
#send_clear()
#print("Cleared LEDs")
#print("Arduino:", ser.readline().decode().strip())