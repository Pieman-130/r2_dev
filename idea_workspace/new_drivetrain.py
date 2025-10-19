import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
PREAMBLE = 0xAA

def send_motor_command(left_dir, left_speed, right_dir, right_speed):
    """
    left_dir/right_dir: 1 = forward, 0 = reverse
    left_speed/right_speed: 0–255
    """
    checksum = (PREAMBLE + left_dir + left_speed + right_dir + right_speed) & 0xFF
    packet = bytes([PREAMBLE, left_dir, left_speed, right_dir, right_speed, checksum])
    ser.write(packet)
    ser.flush()

# Example movement sequence
send_motor_command(1, 0, 0, 10)  # forward
print("Sent Forward")
time.sleep(1)

# Stop sending commands — Arduino failsafe will auto-stop after 1 second
