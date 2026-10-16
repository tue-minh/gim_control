#!/usr/bin/env python3
"""Plot an admittance calibration log produced by admittance_tuner_node.

Usage: plot_admittance.py <calibration.csv> <output.png>

The CSV has the columns:
    timestamp, joint, q_meas, q_ref, q_cmd, qd_meas, tau_ext, tau_meas, delta_q

The figure contains three panels:
  1. joint position (measured vs nominal reference vs commanded) with the compliance
     deflection delta_q on a secondary axis,
  2. external torque tau_ext and measured torque tau_meas,
  3. an admittance phase plot (tau_ext vs delta_q) whose slope is the effective stiffness.
"""
import csv
import sys


COLS = [
    "timestamp",
    "joint",
    "q_meas",
    "q_ref",
    "q_cmd",
    "qd_meas",
    "tau_ext",
    "tau_meas",
    "delta_q",
]


def read_csv(path):
    cols = {key: [] for key in COLS}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for key in cols:
                cols[key].append(float(row[key]))
    return cols


def effective_stiffness(tau, dq):
    """Least-squares slope of tau vs delta_q (Nm/rad)."""
    n = len(dq)
    if n < 2:
        return None
    mean_x = sum(dq) / n
    mean_y = sum(tau) / n
    var = sum((x - mean_x) ** 2 for x in dq)
    if var < 1e-12:
        return None
    cov = sum((x - mean_x) * (y - mean_y) for x, y in zip(dq, tau))
    return cov / var


def main():
    if len(sys.argv) != 3:
        print("usage: plot_admittance.py <calibration.csv> <output.png>", file=sys.stderr)
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

    fig = plt.figure(figsize=(11, 12))
    gs = fig.add_gridspec(3, 1, height_ratios=[1.0, 1.0, 1.2])
    ax0 = fig.add_subplot(gs[0])
    ax1 = fig.add_subplot(gs[1], sharex=ax0)
    ax2 = fig.add_subplot(gs[2])

    ax0.plot(t, d["q_meas"], label="q_meas")
    ax0.plot(t, d["q_ref"], "--", label="q_ref (nominal)")
    ax0.plot(t, d["q_cmd"], ":", label="q_cmd (ref + delta_q)")
    ax0.set_ylabel("position [rad]")
    ax0.legend(loc="best")

    ax0b = ax0.twinx()
    ax0b.plot(t, d["delta_q"], color="tab:green", alpha=0.7, label="delta_q")
    ax0b.set_ylabel("deflection [rad]", color="tab:green")
    ax0b.tick_params(axis="y", labelcolor="tab:green")

    ax1.plot(t, d["tau_ext"], color="tab:red", label="tau_ext (interaction)")
    ax1.plot(t, d["tau_meas"], color="tab:gray", alpha=0.7, label="tau_meas (motor)")
    ax1.axhline(0.0, color="k", linewidth=0.5)
    ax1.set_ylabel("torque [Nm]")
    ax1.legend(loc="best")

    ax2.plot(d["delta_q"], d["tau_ext"], color="tab:blue", linewidth=1.0, label="tau_ext vs delta_q")
    ax2.axhline(0.0, color="k", linewidth=0.5)
    ax2.axvline(0.0, color="k", linewidth=0.5)
    ax2.set_xlabel("delta_q [rad]")
    ax2.set_ylabel("tau_ext [Nm]")
    k_eff = effective_stiffness(d["tau_ext"], d["delta_q"])
    if k_eff is not None:
        ax2.set_title(f"admittance phase plot - empirical stiffness K_eff ~= {k_eff:.2f} Nm/rad")
    ax2.legend(loc="best")

    for ax in (ax0, ax1, ax2):
        ax.grid(True, alpha=0.3)
    ax0.set_xlabel("time [s]")
    ax1.set_xlabel("time [s]")

    joint = int(d["joint"][0]) if d["joint"] else 0
    fig.suptitle(f"GIM arm admittance - joint {joint} - {csv_path.split('/')[-1]}")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    plt.close(fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
