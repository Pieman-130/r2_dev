import serial
import csv
import time

PORT = "/dev/ttyACM0"
BAUD = 115200

OUTPUT_FILE = "balance_test.csv"


def main():

    print(f"Opening {PORT}...")

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.2
    )

    # Leonardo resets when serial opens.
    time.sleep(2)

    print("Connected.")
    print()
    print("Recording balance telemetry.")
    print("Press Ctrl+C to stop.")
    print()

    records = []

    try:

        while True:

            line = ser.readline()

            if not line:
                continue

            text = line.decode(
                "utf-8",
                errors="replace"
            ).strip()

            print(text)

            parts = text.split(",")

            # ------------------------------------------------
            # Expected 15 fields:
            #
            # 0   time_ms
            # 1   pitch
            # 2   pitchRate
            # 3   rawGyroY
            # 4   gyroMin
            # 5   gyroMax
            # 6   gyroAvg
            # 7   gyroRange
            # 8   accelPitch
            # 9   controllerOutput
            # 10  targetMotor
            # 11  currentMotor
            # 12  balanceEnabled
            # 13  balanceFault
            # 14  faultCode
            # ------------------------------------------------

            if len(parts) != 15:
                continue

            if parts[0] == "time_ms":
                continue

            try:

                record = [
                    int(parts[0]),       # time_ms
                    float(parts[1]),     # pitch
                    float(parts[2]),     # pitchRate
                    float(parts[3]),     # rawGyroY
                    float(parts[4]),     # gyroMin
                    float(parts[5]),     # gyroMax
                    float(parts[6]),     # gyroAvg
                    float(parts[7]),     # gyroRange
                    float(parts[8]),     # accelPitch
                    float(parts[9]),     # controllerOutput
                    int(parts[10]),      # targetMotor
                    int(parts[11]),      # currentMotor
                    int(parts[12]),      # balanceEnabled
                    int(parts[13]),      # balanceFault
                    int(parts[14])       # faultCode
                ]

                records.append(record)

            except ValueError:

                continue

    except KeyboardInterrupt:

        print()
        print("Stopping recording...")

    finally:

        ser.close()


    # --------------------------------------------------------
    # Save CSV
    # --------------------------------------------------------

    print()
    print(f"Saving {len(records)} records...")

    with open(
        OUTPUT_FILE,
        "w",
        newline=""
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            "time_ms",
            "pitch",
            "pitchRate",
            "rawGyroY",
            "gyroMin",
            "gyroMax",
            "gyroAvg",
            "gyroRange",
            "accelPitch",
            "controllerOutput",
            "targetMotor",
            "currentMotor",
            "balanceEnabled",
            "balanceFault",
            "faultCode"
        ])

        writer.writerows(records)

    print(f"Saved to: {OUTPUT_FILE}")


if __name__ == "__main__":
    main()