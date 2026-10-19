#!/usr/bin/env python3
"""Plot a multi-joint position trajectory log produced by position_trajectory_node.

Usage: plot_position_trajectory.py <trajectory.csv> <output.png>

The CSV contains full system info for the 3-DOF GIM arm in POSITION mode:
    timestamp,
    q_base_act, q_base_ref, qd_base_act, qd_base_ref, e_p_base, e_v_base, tau_meas_base,
    q_shoulder_act, q_shoulder_ref, qd_shoulder_act, qd_shoulder_ref, e_p_shoulder, e_v_shoulder, tau_meas_shoulder,
    q_elbow_act, q_elbow_ref, qd_elbow_act, qd_elbow_ref, e_p_elbow, e_v_elbow, tau_meas_elbow

The figure plots:
  - Joint positions (actual vs reference)
  - Joint velocities (actual vs reference)
  - Position tracking errors (q_ref - q_act)
  - Velocity tracking errors (qd_ref - qd_act)
  - Measured feedback torques (tau_meas)
  - Position tracking RMSE per joint
"""

import csv
import math
import os
import sys


JOINTS = ["base", "shoulder", "elbow"]
COLORS = {
    "base": "#1f77b4",       # blue
    "shoulder": "#2ca02c",   # green
    "elbow": "#ff7f0e",      # orange
}


def read_csv(path):
    data = {"t": []}
    for j in JOINTS:
        data[f"q_{j}_act"] = []
        data[f"q_{j}_ref"] = []
        data[f"qd_{j}_act"] = []
        data[f"qd_{j}_ref"] = []
        data[f"e_p_{j}"] = []
        data[f"e_v_{j}"] = []
        data[f"tau_meas_{j}"] = []

    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            data["t"].append(float(row["timestamp"]))
            for j in JOINTS:
                data[f"q_{j}_act"].append(float(row[f"q_{j}_act"]))
                data[f"q_{j}_ref"].append(float(row[f"q_{j}_ref"]))
                data[f"qd_{j}_act"].append(float(row[f"qd_{j}_act"]))
                data[f"qd_{j}_ref"].append(float(row[f"qd_{j}_ref"]))
                data[f"e_p_{j}"].append(float(row[f"e_p_{j}"]))
                data[f"e_v_{j}"].append(float(row[f"e_v_{j}"]))
                data[f"tau_meas_{j}"].append(float(row[f"tau_meas_{j}"]))
    return data


def rmse(values):
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def main():
    if len(sys.argv) < 3:
        print("Usage: plot_position_trajectory.py <trajectory.csv> <output.png>", file=sys.stderr)
        return 2

    csv_path = sys.argv[1]
    png_path = sys.argv[2]

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"matplotlib error: {exc}", file=sys.stderr)
        return 1

    if not os.path.isfile(csv_path):
        print(f"CSV file not found: {csv_path}", file=sys.stderr)
        return 1

    d = read_csv(csv_path)
    t = d["t"]
    if not t:
        print("Empty trajectory log file", file=sys.stderr)
        return 1

    fig, axes = plt.subplots(3, 2, figsize=(14, 11), sharex=False)

    # 1) Joint Positions (act vs ref)
    ax_pos = axes[0, 0]
    for j in JOINTS:
        c = COLORS[j]
        ax_pos.plot(t, d[f"q_{j}_act"], color=c, label=f"{j} act")
        ax_pos.plot(t, d[f"q_{j}_ref"], color=c, linestyle="--", alpha=0.7, label=f"{j} ref")
    ax_pos.set_ylabel("Position [rad]")
    ax_pos.set_title("Joint Positions (Actual vs Reference)")
    ax_pos.grid(True, alpha=0.3)
    ax_pos.legend(loc="upper right", ncol=3, fontsize=8)

    # 2) Joint Velocities (act vs ref)
    ax_vel = axes[0, 1]
    for j in JOINTS:
        c = COLORS[j]
        ax_vel.plot(t, d[f"qd_{j}_act"], color=c, label=f"{j} act")
        ax_vel.plot(t, d[f"qd_{j}_ref"], color=c, linestyle="--", alpha=0.7, label=f"{j} ref")
    ax_vel.set_ylabel("Velocity [rad/s]")
    ax_vel.set_title("Joint Velocities (Actual vs Reference)")
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend(loc="upper right", ncol=3, fontsize=8)

    # 3) Position Tracking Error (q_ref - q_act)
    ax_ep = axes[1, 0]
    for j in JOINTS:
        c = COLORS[j]
        err_rmse = rmse(d[f"e_p_{j}"])
        ax_ep.plot(t, d[f"e_p_{j}"], color=c, label=f"{j} (RMSE={err_rmse:.4f} rad)")
    ax_ep.set_ylabel("Position Error [rad]")
    ax_ep.set_title("Joint Position Errors (q_ref - q_act)")
    ax_ep.grid(True, alpha=0.3)
    ax_ep.legend(loc="upper right", fontsize=8)

    # 4) Velocity Tracking Error (qd_ref - qd_act)
    ax_ev = axes[1, 1]
    for j in JOINTS:
        c = COLORS[j]
        err_v_rmse = rmse(d[f"e_v_{j}"])
        ax_ev.plot(t, d[f"e_v_{j}"], color=c, label=f"{j} (RMSE={err_v_rmse:.4f} rad/s)")
    ax_ev.set_ylabel("Velocity Error [rad/s]")
    ax_ev.set_title("Joint Velocity Errors (qd_ref - qd_act)")
    ax_ev.grid(True, alpha=0.3)
    ax_ev.legend(loc="upper right", fontsize=8)

    # 5) Measured Feedback Torque (tau_meas)
    ax_tmeas = axes[2, 0]
    for j in JOINTS:
        c = COLORS[j]
        ax_tmeas.plot(t, d[f"tau_meas_{j}"], color=c, label=f"{j} meas")
    ax_tmeas.set_xlabel("Time [s]")
    ax_tmeas.set_ylabel("Torque [Nm]")
    ax_tmeas.set_title("Measured Feedback Torques (tau_meas)")
    ax_tmeas.grid(True, alpha=0.3)
    ax_tmeas.legend(loc="upper right", fontsize=8)

    # 6) Position tracking RMSE per joint
    ax_rmse = axes[2, 1]
    rmses = [rmse(d[f"e_p_{j}"]) for j in JOINTS]
    bars = ax_rmse.bar(JOINTS, rmses, color=[COLORS[j] for j in JOINTS], alpha=0.8)
    ax_rmse.set_xlabel("Joint")
    ax_rmse.set_ylabel("Position RMSE [rad]")
    ax_rmse.set_title("Position Tracking RMSE")
    ax_rmse.grid(True, axis="y", alpha=0.3)
    for bar, value in zip(bars, rmses):
        ax_rmse.text(bar.get_x() + bar.get_width() / 2.0, value, f"{value:.4f}",
                     ha="center", va="bottom", fontsize=8)

    basename = os.path.basename(csv_path)
    fig.suptitle(f"GIM Arm Multi-Joint Position Trajectory - {basename}", fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0.03, 1, 0.96])

    os.makedirs(os.path.dirname(os.path.abspath(png_path)), exist_ok=True)
    fig.savefig(png_path, dpi=130)
    plt.close(fig)
    print(f"Generated plot: {png_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
