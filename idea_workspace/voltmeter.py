import smbus2
import time

#Number of cells in battery pack
CELLS = 7

# Default I2C address for INA260
INA260_ADDR = 0x40

# Register addresses
REG_BUS_VOLTAGE = 0x02  # Bus voltage register (mV)

# Open I2C bus (use 1 unless i2cdetect shows a different one)
bus = smbus2.SMBus(3)

estimated_voltage_curve = [
        (4.20, 100.0),
        (4.15, 98.0),
        (4.10, 95.0),
        (4.05, 92.0),
        (4.00, 90.0),
        (3.95, 85.0),
        (3.90, 80.0),
        (3.85, 75.0),
        (3.80, 70.0),
        (3.75, 65.0),
        (3.70, 60.0),
        (3.65, 55.0),
        (3.60, 50.0),
        (3.55, 45.0),
        (3.50, 40.0),
        (3.45, 35.0),
        (3.40, 30.0),
        (3.35, 25.0),
        (3.30, 20.0),
        (3.25, 15.0),
        (3.20, 10.0),
        (3.15, 5.0),
        (3.00, 0.0),
    ]


def read_voltage():
    # Read two bytes from the voltage register
    data = bus.read_i2c_block_data(INA260_ADDR, REG_BUS_VOLTAGE, 2)
    raw_voltage = (data[0] << 8) | data[1]
    voltage = raw_voltage * 1.25 / 1000.0  # Convert to volts (1.25mV/bit)
    return voltage

def estimate_charge(voltage):
    v_cell = float(voltage)/float(CELLS)

    if v_cell >= estimated_voltage_curve[0][0]:
        return 100.0
    if v_cell <= estimated_voltage_curve[-1][0]:
        return 0.0

    # find bracketing points and linearly interpolate
    for i in range(len(estimated_voltage_curve) - 1):
        v_high, soc_high = estimated_voltage_curve[i]
        v_low, soc_low = estimated_voltage_curve[i + 1]
        if v_high >= v_cell >= v_low:
            # fraction from low->high
            if v_high == v_low:
                return soc_low
            frac = (v_cell - v_low) / (v_high - v_low)
            soc = soc_low + frac * (soc_high - soc_low)
            return max(0.0, min(100.0, soc))

while True:
    voltage = read_voltage()
    charge = estimate_charge(voltage)
    #print(f"Voltage: {voltage:.3f} V")
    print(f"\rEstimated Charge: {charge:.0f}%",end="", flush=True)
    time.sleep(1)
