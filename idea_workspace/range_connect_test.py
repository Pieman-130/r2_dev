from smbus2 import SMBus
import time

bus = SMBus(4)
addr = 0x29

bus.write_byte_data(addr, 0x00, 0x01)  # start ranging
time.sleep(0.05)

status = bus.read_byte_data(addr, 0x13)
print(f"Interrupt status: 0x{status:02X}")

while True:
    hi = bus.read_byte_data(addr, 0x1E)
    lo = bus.read_byte_data(addr, 0x1F)
    print("Raw distance:", (hi << 8) | lo)
    time.sleep(0.25)

    bus.write_byte_data(addr, 0x80, 0x00)
    time.sleep(0.01)
    bus.write_byte_data(addr, 0x80, 0x01)
    time.sleep(0.01)