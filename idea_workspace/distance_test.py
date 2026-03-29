import time
from smbus2 import SMBus

ADDR = 0x29
BUS = 4

bus = SMBus(BUS)

def write(reg, val):
    bus.write_byte_data(ADDR, reg, val)

def read(reg):
    return bus.read_byte_data(ADDR, reg)

def init():
    # Wait for boot
    for _ in range(100):
        if read(0xC0) == 0xEE:
            break
        time.sleep(0.01)
    else:
        raise RuntimeError("VL53L0X did not boot")

    # Mandatory init (minimal)
    write(0x88, 0x00)

    write(0x80, 0x01)
    write(0xFF, 0x01)
    write(0x00, 0x00)
    write(0x91, 0x3C)  # stop_variable
    write(0x00, 0x01)
    write(0xFF, 0x00)
    write(0x80, 0x00)

    time.sleep(0.05)

def read_distance():
    # Start single ranging
    write(0x00, 0x01)

    # Wait for range-complete interrupt (BIT 2)
    for _ in range(100):
        if read(0x13) & 0x04:
            break
        time.sleep(0.005)
    #else:
    #    return None

    # Read result
    hi = read(0x1E)
    lo = read(0x1F)
    distance = (hi << 8) | lo

    # Clear interrupt (MANDATORY)
    write(0x0B, 0x01)

    return distance

init()
print("VL53L0X ready")

while True:
    d = read_distance()
    print("Distance:", d)
    time.sleep(0.3)
