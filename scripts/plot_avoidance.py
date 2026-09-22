"""Plot an avoidance CSV: path, braking envelope, velocity and lateral PID."""

import argparse
import csv
import math
from pathlib import Path
import sys


STATE_COLORS = {
    "CRUISE": "#21855b", "SLIDE_LEFT": "#7651b2", "SLIDE_RIGHT": "#176db8",
    "CLEARING": "#bd7517", "BRAKE": "#db493f", "BLOCKED": "#912747",
    "WAIT_SCAN": "#777777", "SLOW": "#e79a16", "TURN_LEFT": "#7651b2",
    "TURN_RIGHT": "#176db8",
}
REQUIRED_NUMERIC = ("elapsed_s", "front_m", "nearest_m", "north_m", "east_m")
OPTIONAL_NUMERIC = (
    "forward_m_s", "right_m_s", "actual_forward_m_s", "actual_right_m_s",
    "trigger_m", "stop_m", "lateral_m", "target_lateral_m", "command_lateral_m",
    "yaw_deg", "course_yaw_deg",
)


def read_log(path):
    """Keep legacy logs usable; absent newer telemetry is unknown, not zero."""
    rows = []
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        missing = set((*REQUIRED_NUMERIC, "state")) - set(reader.fieldnames or ())
        if missing:
            raise ValueError(f"Missing CSV columns: {', '.join(sorted(missing))}")
        for line_number, source in enumerate(reader, start=2):
            try:
                row = {key: float(source[key]) if source.get(key) else math.nan
                       for key in (*REQUIRED_NUMERIC, *OPTIONAL_NUMERIC)}
                row["state"] = source["state"].strip()
            except (ValueError, TypeError, AttributeError) as error:
                raise ValueError(f"Invalid CSV row at line {line_number}: {error}") from error
            if not math.isfinite(row["elapsed_s"]) or not row["state"]:
                raise ValueError(f"Missing time or state at CSV line {line_number}")
            if rows and row["elapsed_s"] < rows[-1]["elapsed_s"]:
                raise ValueError(f"Time moves backwards at CSV line {line_number}")
            rows.append(row)
    if not rows:
        raise ValueError("The CSV contains no flight samples")
    return rows


def render(rows, log_path, output, stop_distance=2.5, emergency_distance=0.85):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    def values(key):
        return [r.get(key, math.nan) for r in rows]

    def available(key):
        return any(math.isfinite(value) for value in values(key))

    positions = [r for r in rows if math.isfinite(r["east_m"]) and math.isfinite(r["north_m"])]
    if not positions:
        raise ValueError("The log has no position telemetry; a dry-run log cannot show a flight path")
    valid_clearances = [r["nearest_m"] for r in rows if math.isfinite(r["nearest_m"])]
    minimum = min(valid_clearances) if valid_clearances else math.nan
    sequence = []
    transitions = []
    for row in rows:
        state = row["state"]
        if not sequence or sequence[-1] != state:
            if sequence and (sequence[-1], state) not in transitions:
                transitions.append((sequence[-1], state))
            sequence.append(state)

    elapsed = values("elapsed_s")
    fig, axes = plt.subplots(2, 2, figsize=(15, 10.5))
    map_ax, range_ax, velocity_ax, lateral_ax = axes.flat
    fig.suptitle("Obstacle avoidance: recorded simulation flight", fontsize=18,
                 fontweight="bold", y=0.98)
    map_ax.plot(values("east_m"), values("north_m"), color="#b8bec7", linewidth=1, zorder=2)
    for state in dict.fromkeys(r["state"] for r in positions):
        samples = [r for r in positions if r["state"] == state]
        map_ax.scatter([r["east_m"] for r in samples], [r["north_m"] for r in samples],
                       s=10, color=STATE_COLORS.get(state, "#333333"), label=state, zorder=3)
    # Gazebo world X = east, Y = north. Footprints match obstacle_yolo.sdf.
    map_ax.add_patch(Rectangle((8.5, -4), 1, 8, facecolor="#d96549", edgecolor="#80321f", alpha=0.8))
    map_ax.add_patch(Rectangle((15.5, -6.5), 3, 3, facecolor="#6194d7", edgecolor="#315b90", alpha=0.8))
    map_ax.annotate("Front wall", (9, 4), xytext=(5, 7), textcoords="offset points", fontsize=9)
    map_ax.annotate("Second obstacle", (17, -6.5), xytext=(0, -14),
                    textcoords="offset points", ha="center", fontsize=9)
    for sample, label, marker, offset in ((positions[0], "Start", "o", (6, -14)),
                                         (positions[-1], "Last sample", "X", (6, 6))):
        map_ax.scatter([sample["east_m"]], [sample["north_m"]], marker=marker, s=65,
                       color="#17263a", edgecolor="white", linewidth=1, zorder=5)
        map_ax.annotate(label, (sample["east_m"], sample["north_m"]), xytext=offset,
                        textcoords="offset points", fontsize=9)
    map_ax.set_xlim(min(-2, min(r["east_m"] for r in positions)-2),
                    max(21, max(r["east_m"] for r in positions)+3))
    map_ax.set_ylim(min(-9, min(r["north_m"] for r in positions)-2),
                    max(7, max(r["north_m"] for r in positions)+3))
    map_ax.set_aspect("equal", adjustable="box")
    map_ax.set_title("Measured path and obstacle footprints", fontsize=12)
    map_ax.set_xlabel("East / Gazebo X (m)")
    map_ax.set_ylabel("North / Gazebo Y (m)")
    map_ax.legend(loc="upper right", fontsize=8, framealpha=0.9, markerscale=1.5)

    range_ax.plot(elapsed, values("front_m"), color="#176db8", linewidth=1.5, label="Forward corridor range")
    range_ax.plot(elapsed, values("nearest_m"), color="#ce7720", linewidth=1.5, label="Nearest return (360 deg)")
    if available("trigger_m"):
        range_ax.plot(elapsed, values("trigger_m"), color="#7651b2", linestyle="--", label="Adaptive start distance")
    if available("stop_m"):
        range_ax.plot(elapsed, values("stop_m"), color="#db493f", linestyle="--", label="Measured-speed stopping distance")
    else:
        range_ax.axhline(stop_distance, color="#db493f", linestyle="--", label=f"Legacy brake threshold: {stop_distance:g} m")
    range_ax.axhline(emergency_distance, color="#912747", linestyle=":",
                    label=f"Emergency clearance: {emergency_distance:g} m")
    range_ax.set_title(f"LiDAR and thresholds | minimum nearest: {minimum:.2f} m", fontsize=12)
    range_ax.set_ylabel("Range (m)")
    range_ax.set_ylim(bottom=0)
    range_ax.legend(loc="best", fontsize=8)

    for key, label, color, style in (
            ("forward_m_s", "Forward command", "#176db8", "-"),
            ("actual_forward_m_s", "Forward measured", "#176db8", "--"),
            ("right_m_s", "Right command", "#7651b2", "-"),
            ("actual_right_m_s", "Right measured", "#7651b2", "--")):
        if available(key):
            velocity_ax.plot(elapsed, values(key), color=color, linestyle=style, linewidth=1.5, label=label)
    velocity_ax.axhline(0, color="#777777", linewidth=0.7)
    velocity_ax.set_title("Forward and lateral velocity | right positive", fontsize=12)
    velocity_ax.set_ylabel("Velocity (m/s)")
    if velocity_ax.get_legend_handles_labels()[0]:
        velocity_ax.legend(loc="best", fontsize=8, ncol=2)
    else:
        velocity_ax.text(0.5, 0.5, "Velocity telemetry absent in this log", transform=velocity_ax.transAxes, ha="center")

    if available("lateral_m") and available("target_lateral_m"):
        lateral_ax.plot(elapsed, values("target_lateral_m"), color="#ce7720", linestyle="--", linewidth=1.7, label="Final bypass offset")
        if available("command_lateral_m"):
            lateral_ax.plot(elapsed, values("command_lateral_m"), color="#21855b",
                            linestyle=":", linewidth=1.7, label="Eased setpoint (PID input)")
        lateral_ax.plot(elapsed, values("lateral_m"), color="#7651b2", linewidth=1.7, label="Measured lateral position")
        lateral_ax.legend(loc="best", fontsize=8)
        lateral_ax.set_ylabel("Lateral offset (m; right positive)")
    else:
        lateral_ax.text(0.5, 0.5, "Lateral PID telemetry absent in this legacy log",
                        transform=lateral_ax.transAxes, ha="center", wrap=True)
    yaw_errors = [abs((r.get("yaw_deg", math.nan)-r.get("course_yaw_deg", math.nan)+180) % 360-180)
                  for r in rows if math.isfinite(r.get("yaw_deg", math.nan))
                  and math.isfinite(r.get("course_yaw_deg", math.nan))]
    heading_text = f"Max recorded heading error: {max(yaw_errors):.2f} deg" if yaw_errors else "Heading telemetry unavailable"
    lateral_ax.set_title("Lateral PID tracking | " + heading_text, fontsize=11)

    for ax in axes.flat:
        ax.grid(alpha=0.18)
        if ax is not map_ax:
            ax.set_xlabel("Elapsed flight log time (s)")
    fig.text(0.035, 0.06, f"Log: {Path(log_path).name}  |  Samples: {len(rows)}  |  Last time: {elapsed[-1]:.2f} s", fontsize=10)
    fig.text(0.035, 0.039, "Local NED positions converted to ENU; alignment to the world origin is approximate.", fontsize=9, color="#505762")
    fig.text(0.035, 0.019, "LiDAR samples one horizontal plane. These measurements alone do not prove absence of collisions.", fontsize=9, color="#505762")
    fig.tight_layout(rect=(0.01, 0.085, 0.99, 0.945), h_pad=2.4, w_pad=2.5)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, facecolor="white")
    plt.close(fig)
    print(f"Saved: {output.resolve()}")
    print(f"Samples: {len(rows)}; final time: {elapsed[-1]:.3f} s")
    print(f"Minimum finite nearest LiDAR return: {minimum:.3f} m")
    print(heading_text)
    print("State sequence: " + " -> ".join(sequence))
    print("Distinct transitions: " + (", ".join(f"{a} -> {b}" for a, b in transitions) or "none"))
    print("LiDAR measurements alone do not establish collision-free flight.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", default="logs/avoid-flight-check.csv", help="Completed flight CSV")
    parser.add_argument("--output", default="output/avoidance-check.png")
    parser.add_argument("--stop-distance", type=float, default=2.5, help="Fallback for legacy CSV without stop_m")
    parser.add_argument("--emergency-distance", type=float, default=0.85)
    args = parser.parse_args()
    if not (math.isfinite(args.stop_distance) and math.isfinite(args.emergency_distance)
            and 0 < args.emergency_distance < args.stop_distance):
        parser.error("Thresholds must be finite and 0 < emergency-distance < stop-distance")
    try:
        render(read_log(args.log), args.log, args.output, args.stop_distance, args.emergency_distance)
    except (OSError, ValueError, ImportError) as error:
        parser.exit(1, f"Cannot create plot: {error}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
