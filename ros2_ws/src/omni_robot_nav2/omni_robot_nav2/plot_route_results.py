import csv
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def floats(rows, key):
    return [float(row[key]) for row in rows]


def main():
    results_dir = Path(sys.argv[1])
    summaries = [read_csv(path)[0] for path in sorted(
        results_dir.glob("trial_[0-9].csv")
    )]
    trajectories = {
        int(path.stem.split("_")[1]): read_csv(path)
        for path in results_dir.glob("trial_*_trajectory.csv")
    }
    colors = ["tab:blue", "tab:orange", "tab:green"]
    pickup = (-2.4844879133, 1.7525297249)
    drop = (-0.6351846679, -2.2289173701)
    rotation = (-1.0, -1.5)
    start = (-3.1510900301, -3.1363111209)

    fig, ax = plt.subplots()
    for index, trial in enumerate(sorted(trajectories)):
        rows = trajectories[trial]
        ax.plot(
            floats(rows, "x"), floats(rows, "y"),
            color=colors[index % len(colors)], label=f"trial {trial}"
        )
    ax.scatter(*start, marker="s", label="start")
    ax.scatter(*pickup, marker="o", label="pickup")
    ax.scatter(*rotation, marker="x", label="rotation")
    ax.scatter(*drop, marker="^", label="drop")
    ax.set(title="XY trajectories", xlabel="x [m]", ylabel="y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    fig.savefig(results_dir / "01_xy_trajectories.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    trials = [int(row["trial_id"]) for row in summaries]
    pickup_errors = [float(row["pickup_xy_error_m"]) for row in summaries]
    drop_errors = [float(row["drop_xy_error_m"]) for row in summaries]
    width = 0.35
    positions = range(len(trials))
    ax.bar([p - width / 2 for p in positions], pickup_errors,
           width, label="pickup")
    ax.bar([p + width / 2 for p in positions], drop_errors,
           width, label="drop")
    ax.axhline(0.10, color="red", linestyle="--", label="0.10 m")
    ax.set_xticks(list(positions), [f"trial {t}" for t in trials])
    ax.set(title="Position error", ylabel="XY error [m]")
    ax.grid(axis="y")
    ax.legend()
    fig.savefig(results_dir / "02_position_errors.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    labels = ["start→pickup", "3 s wait", "rotation", "pickup→drop"]
    for index, row in enumerate(summaries):
        values = [
            float(row["start_to_pickup_s"]),
            float(row["pickup_stop_wait_s"]),
            float(row["rotation_duration_s"]),
            float(row["rotation_to_drop_s"]),
        ]
        ax.plot(labels, values, marker="o", label=f"trial {row['trial_id']}")
    ax.set(title="Segment durations", ylabel="time [s]")
    ax.grid(True)
    ax.legend()
    fig.savefig(results_dir / "03_segment_durations.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    for index, trial in enumerate(sorted(trajectories)):
        rows = trajectories[trial]
        t = floats(rows, "t")
        speed = [
            math.hypot(float(row["vx"]), float(row["vy"])) for row in rows
        ]
        wz = floats(rows, "wz")
        ax.plot(t, speed, color=colors[index % len(colors)],
                label=f"trial {trial} translational")
        ax.plot(t, wz, color=colors[index % len(colors)],
                linestyle="--", label=f"trial {trial} angular")
    ax.axhline(0.0, color="black", linewidth=0.5)
    ax.set(title="Velocity time series", xlabel="time [s]",
           ylabel="velocity [m/s or rad/s]")
    ax.grid(True)
    ax.legend(fontsize="small")
    fig.savefig(results_dir / "04_velocity_timeseries.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots()
    for index, trial in enumerate(sorted(trajectories)):
        rows = trajectories[trial]
        ax.plot(
            floats(rows, "t"), floats(rows, "yaw"),
            color=colors[index % len(colors)], label=f"trial {trial}"
        )
    ax.axhline(math.pi, color="red", linestyle="--", label="π rad")
    ax.axhline(-math.pi, color="red", linestyle=":")
    ax.set(title="Yaw time series", xlabel="time [s]", ylabel="yaw [rad]")
    ax.grid(True)
    ax.legend()
    fig.savefig(results_dir / "05_yaw_timeseries.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
