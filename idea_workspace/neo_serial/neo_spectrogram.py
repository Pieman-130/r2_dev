import subprocess
import numpy as np

import serial, time, random

PORT = '/dev/ttyACM0'
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

def fill_array(n, triple):
    """
    Fill a 16-element array based on the input number n (1–8).
    
    Parameters:
        n (int): Controls how many positions are filled.
        triple (list or tuple): 3-element array to fill with.
    
    Returns:
        list: 16-element array, each either [0,0,0] or the given triple.
    """
    if not (0 <= n <= 8):
        raise ValueError("n must be between 1 and 8")

    arr = [[0, 0, 0] for _ in range(16)]

    center = 7.5  # halfway between indices 7 and 8
    half_width = n - 0.5  # controls how far to expand around center

    # compute integer bounds of where to fill
    start = max(0, int(center - half_width + 1))
    end = min(16, int(center + half_width + 1))

    for i in range(start, end):
        arr[i] = list(triple)

    return arr


monitor_name = "alsa_output.usb-Generic_USB2.0_Device_20130100ph0-00.analog-stereo.2.monitor"

# Capture raw audio from monitor via parec
proc = subprocess.Popen(
    ["parec", "-d", monitor_name, "--format=s16le", "--rate=44100", "--channels=2"],
    stdout=subprocess.PIPE
)

CHUNK = 2048 * 2  # 2 frames per chunk
while True:
    raw_data = proc.stdout.read(CHUNK * 2 * 2)  # 2 bytes per sample * 2 channels
    if not raw_data:
        break
    #print(len(raw_data))
    #input("STOP")
    data = np.frombuffer(raw_data, dtype=np.int16)
    if data.any():
        led_len = 1
    else:
        led_len = 0
    magnitude = np.linalg.norm(data) / len(data)
    #magnitude = np.linalg.norm(data)
    #print(f"Volume magnitude: {magnitude:.2f}")
    led_len = magnitude * 0.16
    if led_len >= 1:
        led_len = 8


    frame = fill_array(led_len,[0,100,0])

    send_led_frame(frame)