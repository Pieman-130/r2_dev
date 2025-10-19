import subprocess
import numpy as np

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
    magnitude = np.linalg.norm(data) / len(data)
    #magnitude = np.linalg.norm(data)
    print(f"Volume magnitude: {magnitude:.2f}")
