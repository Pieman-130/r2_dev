#!/usr/bin/env python3

import argparse
import time
from datetime import datetime

import serial


def main():

    parser = argparse.ArgumentParser(
        description="Capture raw Metro Mini serial data."
    )

    parser.add_argument(
        "--port",
        default="/dev/SENS0",
        help="Serial port (default: /dev/SENS0)"
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )

    parser.add_argument(
        "--output",
        default=None,
        help="Output .bin filename"
    )

    parser.add_argument(
        "--seconds",
        type=float,
        default=10,
        help="Capture duration in seconds (default: 10)"
    )

    args = parser.parse_args()

    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"step2_raw_{timestamp}.bin"

    print(f"Opening {args.port} at {args.baud} baud")
    print(f"Output: {args.output}")
    print(f"Duration: {args.seconds} seconds")
    print("Starting capture...")

    total_bytes = 0
    start = time.monotonic()

    try:
        with serial.Serial(
            args.port,
            args.baud,
            timeout=0.1
        ) as ser:

            # Give the serial connection a moment to settle.
            time.sleep(0.2)

            # Clear anything that was already buffered.
            ser.reset_input_buffer()

            with open(args.output, "wb") as f:

                while True:

                    elapsed = time.monotonic() - start

                    if elapsed >= args.seconds:
                        break

                    data = ser.read(256)

                    if data:
                        f.write(data)
                        f.flush()
                        total_bytes += len(data)

    except KeyboardInterrupt:
        print("\nCapture interrupted.")

    print()
    print("Capture complete.")
    print(f"Bytes captured: {total_bytes}")
    print(f"File: {args.output}")


if __name__ == "__main__":
    main()