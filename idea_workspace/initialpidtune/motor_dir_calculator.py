#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def to_float(value):
    try:
        return float(value)
    except (ValueError, TypeError):
        return math.nan


def rolling_average(values, window):
    result = []
    total = 0.0
    queue = []

    for value in values:
        if math.isnan(value):
            result.append(math.nan)
            continue

        queue.append(value)
        total += value

        if len(queue) > window:
            total -= queue.pop(0)

        result.append(total / len(queue))

    return result


def main():
    parser = argparse.ArgumentParser(
        description="Analyze R2 combined balance logs and estimate signed wheel velocity."
    )
    parser.add_argument("csv", help="Combined R2 balance CSV")
    parser.add_argument(
        "--window",
        type=int,
        default=10,
        help="Moving-average window in wheel samples (default: 10)",
    )
    parser.add_argument(
        "--deadband",
        type=float,
        default=1.0,
        help="Ignore motor commands between +/- this value when assigning direction "
             "(default: 1.0)",
    )
    args = parser.parse_args()

    path = Path(args.csv)

    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise SystemExit("CSV contains no rows.")

    # The current logger records Leonardo telemetry at ~20 Hz and sensor
    # packets at ~50 Hz. Each Leonardo row carries the latest sensor values.
    # We therefore use rows where a wheel RPM value is present.
    time_s = []
    pitch = []
    pitch_rate = []
    controller = []
    motor = []
    right_rpm = []
    left_rpm = []
    right_count = []
    left_count = []

    for row in rows:
        t = to_float(row.get("host_time_s"))
        p = to_float(row.get("pitch"))
        pr = to_float(row.get("pitchRate"))
        co = to_float(row.get("controllerOutput"))
        m = to_float(row.get("currentLeftMotor"))
        rr = to_float(row.get("right_rpm"))
        lr = to_float(row.get("left_rpm"))
        rc = to_float(row.get("right_transition_count"))
        lc = to_float(row.get("left_transition_count"))

        if math.isnan(t) or math.isnan(rr) or math.isnan(lr):
            continue

        time_s.append(t)
        pitch.append(p)
        pitch_rate.append(pr)
        controller.append(co)
        motor.append(m)
        right_rpm.append(rr)
        left_rpm.append(lr)
        right_count.append(rc)
        left_count.append(lc)

    if len(time_s) < 2:
        raise SystemExit("Not enough rows containing wheel RPM data.")

    # ------------------------------------------------------------
    # Provisional direction estimate
    #
    # The SC signal itself has no direction information.
    #
    # We use the motor command sign as a PROVISIONAL sign only.
    # This is explicitly diagnostic; it is not yet being fed into
    # the balance controller.
    #
    # Because both motor commands are normally equal in this test,
    # currentLeftMotor is sufficient to establish commanded direction.
    # When command is close to zero, hold the previous direction.
    # ------------------------------------------------------------

    direction = []
    last_direction = 0

    for m in motor:
        if math.isnan(m):
            direction.append(last_direction)
            continue

        if m > args.deadband:
            last_direction = 1
        elif m < -args.deadband:
            last_direction = -1

        direction.append(last_direction)

    signed_right = [
        rpm * d for rpm, d in zip(right_rpm, direction)
    ]
    signed_left = [
        rpm * d for rpm, d in zip(left_rpm, direction)
    ]

    signed_avg = [
        (r + l) / 2.0
        for r, l in zip(signed_right, signed_left)
    ]

    signed_avg_filtered = rolling_average(
        signed_avg,
        max(1, args.window)
    )

    # ------------------------------------------------------------
    # Basic statistics
    # ------------------------------------------------------------

    n = len(signed_avg)

    mean_pitch = sum(
        x for x in pitch if not math.isnan(x)
    ) / max(1, sum(not math.isnan(x) for x in pitch))

    mean_motor = sum(
        x for x in motor if not math.isnan(x)
    ) / max(1, sum(not math.isnan(x) for x in motor))

    mean_signed_rpm = sum(signed_avg) / n
    mean_filtered_rpm = sum(signed_avg_filtered) / n

    negative_fraction = sum(
        x < 0 for x in signed_avg
    ) / n

    positive_fraction = sum(
        x > 0 for x in signed_avg
    ) / n

    # ------------------------------------------------------------
    # Print summary
    # ------------------------------------------------------------

    print("==============================================")
    print("R2 PROVISIONAL SIGNED WHEEL ANALYSIS")
    print("==============================================")
    print(f"File:                    {path}")
    print(f"Wheel samples:            {n}")
    print(f"Moving-average window:    {args.window}")
    print()
    print("IMPORTANT:")
    print("The Hall SC signal has no direction information.")
    print("The sign below is inferred from motor-command direction.")
    print("It is diagnostic only and is NOT yet controller feedback.")
    print()
    print(f"Mean pitch:               {mean_pitch:.3f} deg")
    print(f"Mean motor command:       {mean_motor:.3f}")
    print(f"Mean provisional wheel:   {mean_signed_rpm:.3f} RPM")
    print(f"Mean filtered wheel:      {mean_filtered_rpm:.3f} RPM")
    print(f"Negative velocity fraction: {negative_fraction * 100:.1f}%")
    print(f"Positive velocity fraction: {positive_fraction * 100:.1f}%")
    print()

    # The useful question is the velocity while approximately upright.
    upright = [
        i for i, p in enumerate(pitch)
        if not math.isnan(p) and abs(p) <= 1.0
    ]

    if upright:
        upright_rpm = [signed_avg[i] for i in upright]
        upright_motor = [motor[i] for i in upright if not math.isnan(motor[i])]

        print("While |pitch| <= 1 degree:")
        print(f"  Samples:               {len(upright)}")
        print(f"  Mean signed wheel:     "
              f"{sum(upright_rpm) / len(upright_rpm):.3f} RPM")

        if upright_motor:
            print(f"  Mean motor command:    "
                  f"{sum(upright_motor) / len(upright_motor):.3f}")

    # ------------------------------------------------------------
    # Save derived CSV
    # ------------------------------------------------------------

    derived_path = path.with_name(path.stem + "_signed_velocity.csv")

    with derived_path.open("w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "host_time_s",
            "pitch",
            "pitchRate",
            "controllerOutput",
            "motorCommand",
            "rightRPM_magnitude",
            "leftRPM_magnitude",
            "provisionalDirection",
            "rightRPM_signed",
            "leftRPM_signed",
            "averageRPM_signed",
            "averageRPM_filtered",
        ])

        for values in zip(
            time_s,
            pitch,
            pitch_rate,
            controller,
            motor,
            right_rpm,
            left_rpm,
            direction,
            signed_right,
            signed_left,
            signed_avg,
            signed_avg_filtered,
        ):
            writer.writerow(values)

    print()
    print(f"Derived data written to:")
    print(f"  {derived_path}")

    # ------------------------------------------------------------
    # Plot 1: pitch + provisional signed velocity
    # ------------------------------------------------------------

    fig, ax1 = plt.subplots()

    ax1.plot(time_s, pitch, label="Pitch (deg)")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Pitch (deg)")

    ax2 = ax1.twinx()
    ax2.plot(
        time_s,
        signed_avg_filtered,
        label="Filtered provisional wheel velocity"
    )
    ax2.set_ylabel("Wheel velocity (RPM)")

    ax1.axhline(0.0)
    ax2.axhline(0.0)

    ax1.set_title("R2 Pitch vs Provisional Signed Wheel Velocity")
    fig.tight_layout()

    plot1 = path.with_name(path.stem + "_pitch_velocity.png")
    fig.savefig(plot1, dpi=150)
    plt.close(fig)

    # ------------------------------------------------------------
    # Plot 2: motor command + wheel velocity
    # ------------------------------------------------------------

    fig, ax1 = plt.subplots()

    ax1.plot(time_s, motor, label="Motor command")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Motor command")

    ax2 = ax1.twinx()
    ax2.plot(
        time_s,
        signed_avg_filtered,
        label="Filtered provisional wheel velocity"
    )
    ax2.set_ylabel("Wheel velocity (RPM)")

    ax1.axhline(0.0)
    ax2.axhline(0.0)

    ax1.set_title("Motor Command vs Provisional Signed Wheel Velocity")
    fig.tight_layout()

    plot2 = path.with_name(path.stem + "_motor_velocity.png")
    fig.savefig(plot2, dpi=150)
    plt.close(fig)

    # ------------------------------------------------------------
    # Plot 3: left/right wheel speeds
    # ------------------------------------------------------------

    fig, ax = plt.subplots()

    ax.plot(time_s, right_rpm, label="Right RPM magnitude")
    ax.plot(time_s, left_rpm, label="Left RPM magnitude")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("RPM magnitude")
    ax.set_title("R2 Wheel Speeds")
    ax.legend()
    fig.tight_layout()

    plot3 = path.with_name(path.stem + "_wheel_rpm.png")
    fig.savefig(plot3, dpi=150)
    plt.close(fig)

    print()
    print("Plots written:")
    print(f"  {plot1}")
    print(f"  {plot2}")
    print(f"  {plot3}")


if __name__ == "__main__":
    main()
