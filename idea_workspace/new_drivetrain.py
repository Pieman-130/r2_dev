import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
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
send_motor_command(1, 50, 1, 50)  # forward
time.sleep(1)

send_motor_command(0, 50, 0, 50)  # reverse
time.sleep(1)

send_motor_command(1, 50, 0, 50)  # spin in place
time.sleep(1)

# Stop sending commands — Arduino failsafe will auto-stop after 1 second
