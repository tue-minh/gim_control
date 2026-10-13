#!/usr/bin/env python3
"""Plot a calibration log produced by impedance_tuner_node.

Usage: plot_calibration.py <calibration.csv> <output.png>

The CSV has the columns:
    timestamp, actual_position, target_position, actual_velocity, target_velocity,
    torque, position_error, velocity_error

The figure contains four stacked panels (position, velocity, torque, error)
sharing the same time axis.
"""
import csv
import sys


def read_csv(path):
    cols = {
        "timestamp": [],
        "actual_position": [],
        "target_position": [],
        "actual_velocity": [],
        "target_velocity": [],
        "torque": [],
        "position_error": [],
        "velocity_error": [],
    }
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in cols:
                cols[key].append(float(row[key]))
    return cols


def main():
    if len(sys.argv) != 3:
        print("usage: plot_calibration.py <calibration.csv> <output.png>", file=sys.stderr)
        return 2

    csv_path, png_path = sys.argv[1], sys.argv[2]

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # noqa: BLE001
        print(f"matplotlib is required: {exc}", file=sys.stderr)
        return 1

    d = read_csv(csv_path)
    if not d["timestamp"]:
        print("empty calibration file", file=sys.stderr)
        return 1

    t = d["timestamp"]
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 10))

    axes[0].plot(t, d["actual_position"], label="actual")
    axes[0].plot(t, d["target_position"], "--", label="target")
    axes[0].set_ylabel("position [rad]")
    axes[0].legend(loc="best")

    axes[1].plot(t, d["actual_velocity"], label="actual")
    axes[1].plot(t, d["target_velocity"], "--", label="target")
    axes[1].set_ylabel("velocity [rad/s]")
    axes[1].legend(loc="best")

    axes[2].plot(t, d["torque"], color="tab:red")
    axes[2].set_ylabel("torque [Nm]")

    axes[3].plot(t, d["position_error"], label="position error")
    axes[3].plot(t, d["velocity_error"], label="velocity error")
    axes[3].set_ylabel("error")
    axes[3].set_xlabel("time [s]")
    axes[3].legend(loc="best")

    for ax in axes:
        ax.grid(True, alpha=0.3)

    fig.suptitle(f"GIM arm calibration - {csv_path.split('/')[-1]}")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
