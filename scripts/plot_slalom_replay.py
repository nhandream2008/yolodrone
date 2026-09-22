"""Render a read-only replay from avoid_fly CSV evidence.

The script never imports MAVSDK or Gazebo and never sends a flight command.
It visualizes recorded telemetry; LiDAR clearance is not collision proof.
"""
import argparse
import csv
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


STATE_COLORS = {
    "CRUISE": "#21855b", "SLIDE_LEFT": "#7651b2", "SLIDE_RIGHT": "#176db8",
    "CLEARING": "#bd7517", "BRAKE": "#db493f", "BLOCKED": "#912747",
    "WAIT_SCAN": "#777777", "FINAL": "#20242a",
}
REQUIRED = ("elapsed_s", "state", "north_m", "east_m")
NUMERIC = (
    "elapsed_s", "monotonic_elapsed_s", "north_m", "east_m", "down_m",
    "home_north_m", "home_east_m", "home_down_m", "home_distance_m", "altitude_m",
    "forward_m_s", "right_m_s", "yaw_deg_s", "front_m", "nearest_m",
    "target_lateral_m", "command_lateral_m", "lateral_m", "actual_forward_m_s",
    "actual_right_m_s", "requested_north_m_s", "requested_east_m_s",
    "measured_north_m_s", "measured_east_m_s", "vertical_speed_m_s", "yaw_deg",
    "course_yaw_deg", "navigation_yaw_deg", "outbound_forward_m", "lidar_sim_time_s",
    "lidar_age_ms", "pid_error_m", "pid_p_m_s", "pid_i_m_s", "pid_d_m_s",
    "risk_front_m", "risk_closing_speed_m_s", "risk_ttc_s", "risk_uncertainty_margin_m",
    "risk_speed_cap_m_s", "risk_track_confidence",
)
TEXT = (
    "state", "phase", "event_type", "transition_reason", "reason", "result",
    "result_reason", "run_id", "world", "landing_confirmed", "disarmed_confirmed",
    "safety_severity", "safety_code", "safety_reason", "risk_hold_recommended",
)
GATES = ((8.0, -49.0, 1.0, "Gate 1: right", 1),
         (15.0, -1.0, 49.0, "Gate 2: left", -1),
         (22.0, -49.0, 1.0, "Gate 3: right", 1),
         (29.0, -1.0, 49.0, "Gate 4: left", -1))


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(value)


def read_log(path):
    rows = []
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        missing = set(REQUIRED)-fields
        if missing:
            raise ValueError("Missing CSV columns: " + ", ".join(sorted(missing)))
        for line, source in enumerate(reader, start=2):
            try:
                row = {key: float(source[key]) if source.get(key) else math.nan for key in NUMERIC}
                row.update({key: source.get(key, "").strip() for key in TEXT})
            except (TypeError, ValueError) as error:
                raise ValueError(f"Invalid row {line}: {error}") from error
            if not math.isfinite(row["elapsed_s"]) or not row["state"]:
                raise ValueError(f"Missing time or state at row {line}")
            if rows and row["elapsed_s"] < rows[-1]["elapsed_s"]:
                raise ValueError(f"Time moves backwards at row {line}")
            rows.append(row)
    if not rows:
        raise ValueError("The CSV contains no flight samples")
    return rows


def _flight_rows(rows):
    return [row for row in rows if row["event_type"] != "FINAL" and row["state"] != "FINAL"]


def _home_position(rows, positioned):
    for row in rows:
        if _finite(row["home_north_m"]) and _finite(row["home_east_m"]):
            return row["home_north_m"], row["home_east_m"], False
    first = positioned[0]
    return first["north_m"], first["east_m"], True


def course_path(rows):
    """Rotate recorded NED positions around recorded Home into the initial course frame."""
    samples = _flight_rows(rows)
    positioned = [row for row in samples if _finite(row["north_m"]) and _finite(row["east_m"])]
    if not positioned:
        raise ValueError("The log has no position telemetry; dry-run logs cannot be replayed as a path")
    course_yaw = next((row["course_yaw_deg"] for row in positioned
                       if _finite(row["course_yaw_deg"])), math.nan)
    if not _finite(course_yaw):
        raise ValueError("The log has no recorded initial course yaw")
    home_north, home_east, _ = _home_position(rows, positioned)
    yaw = math.radians(course_yaw)
    path = []
    for row in positioned:
        north, east = row["north_m"]-home_north, row["east_m"]-home_east
        path.append((north*math.cos(yaw)+east*math.sin(yaw),
                     -north*math.sin(yaw)+east*math.cos(yaw), row))
    return path, course_yaw


def _crossing_times(path, forward_m):
    outbound = returning = None
    for previous, current in zip(path, path[1:]):
        x0, x1 = previous[0], current[0]
        if x1 == x0 or not (min(x0, x1) <= forward_m <= max(x0, x1)):
            continue
        fraction = (forward_m-x0)/(x1-x0)
        crossing = previous[2]["elapsed_s"] + fraction*(current[2]["elapsed_s"]-previous[2]["elapsed_s"])
        phase = current[2]["phase"] or previous[2]["phase"]
        if x1 > x0 and outbound is None and not phase.startswith("RETURN"):
            outbound = crossing, previous[1]+fraction*(current[1]-previous[1])
        elif x1 < x0 and returning is None and phase in (
                "RETURN_HOME", "HOME_APPROACH", "HOME_SETTLE", "LANDING", "FAILED"):
            returning = crossing, previous[1]+fraction*(current[1]-previous[1])
    return outbound, returning


def _opening_passed(lateral_m, low, high, opening_side, margin_m=0.30):
    if not _finite(lateral_m):
        return False
    return (lateral_m >= high+margin_m if opening_side > 0
            else lateral_m <= low-margin_m)


def _longest_forward_stop(rows, threshold=0.05):
    longest = current_start = None
    samples = _flight_rows(rows)
    for index, row in enumerate(samples):
        velocity = row["actual_forward_m_s"] if _finite(row["actual_forward_m_s"]) else row["forward_m_s"]
        if _finite(velocity) and abs(velocity) < threshold:
            current_start = row["elapsed_s"] if current_start is None else current_start
        elif current_start is not None:
            longest = max(longest or 0.0, row["elapsed_s"]-current_start)
            current_start = None
        if index == len(samples)-1 and current_start is not None:
            longest = max(longest or 0.0, row["elapsed_s"]-current_start)
    return longest or 0.0


def _direction_changes(rows, threshold=0.15):
    last, changes = 0, 0
    for row in _flight_rows(rows):
        velocity = row["actual_right_m_s"] if _finite(row["actual_right_m_s"]) else row["right_m_s"]
        direction = 1 if velocity > threshold else -1 if velocity < -threshold else 0
        if direction and last and direction != last:
            changes += 1
        if direction:
            last = direction
    return changes


def _truth(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def metrics(rows, path):
    samples = _flight_rows(rows)
    nearest = [row["nearest_m"] for row in samples if _finite(row["nearest_m"])]
    yaw_errors = [abs((row["yaw_deg"]-row["course_yaw_deg"]+180.0) % 360.0-180.0)
                  for row in samples if _finite(row["yaw_deg"]) and _finite(row["course_yaw_deg"])]
    yaw_rates = [abs(row["yaw_deg_s"]) for row in samples if _finite(row["yaw_deg_s"])]
    both_crossings = {label: _crossing_times(path, x) for x, _, _, label, _ in GATES}
    gate_geometry = {label: (low, high, side) for _, low, high, label, side in GATES}
    final = next((row for row in reversed(rows) if row["event_type"] == "FINAL"), None)
    positioned = [row for row in samples if _finite(row["north_m"]) and _finite(row["east_m"])]
    _, _, inferred_home = _home_position(rows, positioned)
    final_home_distance = final["home_distance_m"] if final and _finite(final["home_distance_m"]) else math.hypot(path[-1][0], path[-1][1])
    limitations = []
    if inferred_home:
        limitations.append("Home is absent; origin was inferred from the first positioned sample")
    if not any(row["phase"] for row in samples):
        limitations.append("Mission phase is absent; outbound/return separation is unavailable")
    if final is None:
        limitations.append("FINAL lifecycle row is absent; landing/disarm outcome is unknown")
    if not any(_finite(row["yaw_deg"]) for row in samples):
        limitations.append("Measured yaw is absent; nose arrows use local path direction")
    outbound_times = {label: item[0][0] if item[0] else None
                      for label, item in both_crossings.items()}
    return_times = {label: item[1][0] if item[1] else None
                    for label, item in both_crossings.items()}
    outbound_passed = {
        label: bool(item[0] and _opening_passed(item[0][1], *gate_geometry[label]))
        for label, item in both_crossings.items()
    }
    return_passed = {
        label: bool(item[1] and _opening_passed(item[1][1], *gate_geometry[label]))
        for label, item in both_crossings.items()
    }
    world = next((row["world"] for row in rows if row["world"]), "")
    course_yaw = next((row["course_yaw_deg"] for row in samples
                       if _finite(row["course_yaw_deg"])), math.nan)
    course_error = (abs((course_yaw-90.0+180.0) % 360.0-180.0)
                    if world == "slalom_yolo" and _finite(course_yaw) else math.nan)
    course_aligned = (course_error <= 20.0 if _finite(course_error) else None)
    lifecycle_complete = bool(final and final["result"] == "COMPLETE"
                              and _truth(final["landing_confirmed"])
                              and _truth(final["disarmed_confirmed"]))
    acceptance_passed = (lifecycle_complete and course_aligned is not False
                         and all(outbound_passed.values()) and all(return_passed.values()))
    return {
        "samples": len(samples), "duration_s": rows[-1]["elapsed_s"],
        "minimum_nearest_m": min(nearest) if nearest else math.nan,
        "outbound_gate_crossings_s": outbound_times,
        "return_gate_crossings_s": return_times,
        "gate_crossings_s": outbound_times,
        "outbound_gate_passed": outbound_passed,
        "return_gate_passed": return_passed,
        "direction_changes": _direction_changes(rows),
        "longest_forward_stop_s": _longest_forward_stop(rows),
        "max_heading_error_deg": max(yaw_errors) if yaw_errors else math.nan,
        "max_commanded_yaw_rate_deg_s": max(yaw_rates) if yaw_rates else math.nan,
        "final_home_distance_m": final_home_distance,
        "result": final["result"] if final else "UNKNOWN (legacy log)",
        "result_reason": final["result_reason"] if final else "No FINAL lifecycle row",
        "landing_confirmed": _truth(final["landing_confirmed"]) if final else False,
        "disarmed_confirmed": _truth(final["disarmed_confirmed"]) if final else False,
        "home_inferred_from_first_sample": inferred_home,
        "limitations": limitations,
        "course_alignment_error_deg": course_error,
        "course_aligned": course_aligned,
        "acceptance_passed": acceptance_passed,
    }


def _nose_directions(path, course_yaw_deg):
    directions = []
    for index, point in enumerate(path):
        measured_yaw = point[2]["yaw_deg"]
        if _finite(measured_yaw):
            relative = math.radians((measured_yaw-course_yaw_deg+180.0) % 360.0-180.0)
            directions.append((math.cos(relative), math.sin(relative)))
            continue
        before = path[max(0, index-1)]
        after = path[min(len(path)-1, index+1)]
        dx, dy = after[0]-before[0], after[1]-before[1]
        magnitude = math.hypot(dx, dy)
        directions.append((dx/magnitude, dy/magnitude) if magnitude > 1e-6 else (0.0, 0.0))
    return directions


def render(rows, log_path, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    path, course_yaw = course_path(rows)
    samples = _flight_rows(rows)
    report = metrics(rows, path)
    fig, axes = plt.subplots(3, 2, figsize=(16, 14.5))
    map_ax, velocity_ax, lateral_ax, phase_ax, safety_ax, clearance_ax = axes.flat
    fig.suptitle("PX4/Gazebo round-trip replay — recorded telemetry only", fontsize=18, fontweight="bold")

    for forward, low, high, label, _opening_side in GATES:
        map_ax.add_patch(Rectangle((forward-0.3, low), 0.6, high-low, facecolor="#d96549",
                                   edgecolor="#80321f", alpha=0.62, zorder=1))
        map_ax.annotate(label, (forward, high if high < 8 else low), ha="center", va="bottom", fontsize=8)
    map_ax.scatter([0], [0], marker="H", s=150, color="#147a54", edgecolor="white",
                   linewidth=1, label="Recorded Home", zorder=5)
    map_ax.axhline(0.0, color="#525a64", linestyle="--", linewidth=1, label="Initial center line")
    map_ax.axvline(42.0, color="#21855b", linestyle=":", linewidth=1, label="Return marker")
    map_ax.plot([point[0] for point in path], [point[1] for point in path], color="#bcc4cf", linewidth=1.1, zorder=2)
    for phase in dict.fromkeys(point[2]["phase"] or "LEGACY" for point in path):
        points = [point for point in path if (point[2]["phase"] or "LEGACY") == phase]
        color = "#2475b0" if phase in ("OUTBOUND", "LEGACY") else "#dc702d"
        map_ax.scatter([point[0] for point in points], [point[1] for point in points],
                       s=10, color=color, label=phase, zorder=3)
    directions = _nose_directions(path, course_yaw)
    arrow_indices = list(range(0, len(path), max(1, len(path)//18)))
    map_ax.quiver([path[i][0] for i in arrow_indices], [path[i][1] for i in arrow_indices],
                  [directions[i][0] for i in arrow_indices], [directions[i][1] for i in arrow_indices],
                  color="#17263a", angles="xy", scale_units="xy", scale=1.5, width=0.004, zorder=4)
    map_ax.set_title(f"Top-down route; arrows are measured nose yaw | initial yaw {course_yaw:.1f}°")
    map_ax.set_xlabel("Forward on initial heading (m)")
    map_ax.set_ylabel("Lateral offset (m; right positive)")
    map_ax.set_aspect("equal", adjustable="box")
    map_ax.set_xlim(min(-2.0, min(point[0] for point in path)-2), max(44.0, max(point[0] for point in path)+2))
    map_ax.set_ylim(min(-11.0, min(point[1] for point in path)-2), max(11.0, max(point[1] for point in path)+2))
    map_ax.legend(loc="upper right", fontsize=7, ncol=2)

    elapsed = [row["elapsed_s"] for row in samples]
    for key, label, color, style in (("forward_m_s", "Forward requested", "#176db8", "-"),
                                     ("actual_forward_m_s", "Forward measured", "#176db8", "--"),
                                     ("right_m_s", "Right requested", "#7651b2", "-"),
                                     ("actual_right_m_s", "Right measured", "#7651b2", "--")):
        values = [row[key] for row in samples]
        if any(_finite(value) for value in values):
            velocity_ax.plot(elapsed, values, label=label, color=color, linestyle=style, linewidth=1.4)
    velocity_ax.axhline(0.0, color="#777777", linewidth=0.7)
    velocity_ax.set_title("Requested versus measured active-navigation-frame velocity")
    velocity_ax.set_xlabel("Monotonic elapsed time (s)")
    velocity_ax.set_ylabel("Velocity (m/s)")
    velocity_ax.legend(loc="best", fontsize=8, ncol=2)

    for key, label, color, style in (("target_lateral_m", "Bypass target", "#ce7720", "--"),
                                     ("command_lateral_m", "PID setpoint", "#21855b", ":"),
                                     ("lateral_m", "Measured lateral", "#7651b2", "-")):
        values = [row[key] for row in samples]
        if any(_finite(value) for value in values):
            lateral_ax.plot(elapsed, values, label=label, color=color, linestyle=style, linewidth=1.5)
    for key, label, color in (("pid_p_m_s", "PID P", "#d64a3a"),
                              ("pid_i_m_s", "PID I", "#18875c"),
                              ("pid_d_m_s", "PID D", "#315fc2")):
        values = [row[key] for row in samples]
        if any(_finite(value) for value in values):
            lateral_ax.plot(elapsed, values, label=label, color=color, linewidth=0.9, alpha=0.7)
    lateral_ax.axhline(0.0, color="#777777", linewidth=0.7)
    lateral_ax.set_title("Lateral PID evidence")
    lateral_ax.set_xlabel("Monotonic elapsed time (s)")
    lateral_ax.set_ylabel("Position (m) / PID term (m/s)")
    lateral_ax.legend(loc="best", fontsize=8, ncol=2)

    visible_phases = list(dict.fromkeys(row["phase"] or "LEGACY" for row in samples))
    phase_index = {phase: index for index, phase in enumerate(visible_phases)}
    phase_ax.step(elapsed, [phase_index[row["phase"] or "LEGACY"] for row in samples],
                  where="post", color="#17263a", linewidth=1.4)
    phase_ax.set_yticks(list(phase_index.values()), list(phase_index.keys()))
    phase_ax.set_title("Mission lifecycle (separate from planner state)")
    phase_ax.set_xlabel("Monotonic elapsed time (s)")
    phase_ax.set_ylabel("Phase")

    severity_level = {"INFO": 0, "DEGRADE": 1, "HOLD": 2, "ABORT": 3}
    safety_values = [severity_level.get(row["safety_severity"], math.nan) for row in samples]
    if any(_finite(value) for value in safety_values):
        safety_ax.step(elapsed, safety_values, where="post", color="#912747", linewidth=1.5,
                       label="Safety severity")
        safety_ax.set_yticks(list(severity_level.values()), list(severity_level))
    lidar_age = [row["lidar_age_ms"] for row in samples]
    if any(_finite(value) for value in lidar_age):
        safety_ax.plot(elapsed, lidar_age, color="#bd7517", linewidth=1.1, alpha=0.85,
                       label="LiDAR age (ms)")
    safety_ax.set_title("Safety supervisor evidence")
    safety_ax.set_xlabel("Monotonic elapsed time (s)")
    safety_ax.set_ylabel("Severity / scan age (ms)")
    safety_ax.legend(loc="best", fontsize=8)

    for key, label, color, style in (("front_m", "Planner front distance", "#176db8", "-"),
                                     ("risk_front_m", "Temporal corridor distance", "#21855b", "--"),
                                     ("risk_ttc_s", "Time to collision", "#912747", ":"),
                                     ("risk_speed_cap_m_s", "Temporal speed cap", "#7651b2", "-")):
        values = [row[key] for row in samples]
        if any(_finite(value) for value in values):
            clearance_ax.plot(elapsed, values, label=label, color=color, linestyle=style, linewidth=1.35)
    clearance_ax.axhline(0.0, color="#777777", linewidth=0.7)
    clearance_ax.set_title("LiDAR clearance, TTC, and conservative speed cap")
    clearance_ax.set_xlabel("Monotonic elapsed time (s)")
    clearance_ax.set_ylabel("Metres / seconds / m/s")
    clearance_ax.legend(loc="best", fontsize=8)

    for axis in axes.flat:
        axis.grid(alpha=0.18)
    outbound = ", ".join(
        f"{label}: {value:.2f}s {'PASS' if report['outbound_gate_passed'][label] else 'FAIL'}"
        if value is not None else f"{label}: —"
        for label, value in report["outbound_gate_crossings_s"].items())
    returning = ", ".join(
        f"{label}: {value:.2f}s {'PASS' if report['return_gate_passed'][label] else 'FAIL'}"
        if value is not None else f"{label}: —"
        for label, value in report["return_gate_crossings_s"].items())
    minimum = report["minimum_nearest_m"]
    min_text = f"{minimum:.2f} m" if _finite(minimum) else "unavailable"
    home_note = " (legacy: inferred)" if report["home_inferred_from_first_sample"] else ""
    fig.text(0.025, 0.082, f"Lifecycle: {report['result']} | zigzag acceptance: {'PASS' if report['acceptance_passed'] else 'FAIL'} — {report['result_reason']} | Home error: {report['final_home_distance_m']:.2f} m | landing/disarm: {report['landing_confirmed']}/{report['disarmed_confirmed']}{home_note}", fontsize=9.2)
    fig.text(0.025, 0.060, f"Outbound crossings: {outbound}", fontsize=8.5)
    fig.text(0.025, 0.040, f"Return crossings: {returning}", fontsize=8.5)
    fig.text(0.025, 0.020, f"Sensor-reported minimum LiDAR clearance: {min_text}; this is not collision proof. Samples: {report['samples']}; duration: {report['duration_s']:.2f}s", fontsize=8.5)
    fig.tight_layout(rect=(0.01, 0.105, 0.99, 0.95), h_pad=2.3, w_pad=2.3)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, facecolor="white")
    plt.close(fig)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, help="Completed CSV written by avoid_fly.py")
    parser.add_argument("--output", default="output/slalom-replay.png")
    args = parser.parse_args(argv)
    try:
        report = render(read_log(args.log), args.log, args.output)
    except (OSError, ValueError, ImportError) as error:
        parser.exit(1, f"Cannot render replay: {error}\n")
    print(f"Saved: {Path(args.output).resolve()}")
    print(f"Result: {report['result']} ({report['result_reason']})")
    print(f"Measured zigzag acceptance: {'PASS' if report['acceptance_passed'] else 'FAIL'}")
    print(f"Final Home error: {report['final_home_distance_m']:.3f} m; "
          f"landing/disarm confirmed: {report['landing_confirmed']}/{report['disarmed_confirmed']}")
    print("Outbound crossings: " + ", ".join(
        f"{gate}={value:.3f}s/{'PASS' if report['outbound_gate_passed'][gate] else 'FAIL'}"
        if value is not None else f"{gate}=not-crossed"
        for gate, value in report["outbound_gate_crossings_s"].items()))
    print("Return crossings: " + ", ".join(
        f"{gate}={value:.3f}s/{'PASS' if report['return_gate_passed'][gate] else 'FAIL'}"
        if value is not None else f"{gate}=not-crossed"
        for gate, value in report["return_gate_crossings_s"].items()))
    print("Minimum finite nearest LiDAR is sensor-reported clearance, not collision proof: "
          f"{report['minimum_nearest_m']:.3f} m")
    for limitation in report["limitations"]:
        print(f"Legacy-log limitation: {limitation}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
