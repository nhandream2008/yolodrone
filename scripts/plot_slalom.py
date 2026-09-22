"""Render the offline fixed-heading slalom regression course."""

import argparse

from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from offline_slalom import config_for, simulate, simulate_round_trip, slalom


STATE_COLORS = {
    "CRUISE": "#21855b",
    "SLIDE_LEFT": "#7651b2",
    "SLIDE_RIGHT": "#176db8",
    "CLEARING": "#bd7517",
    "WAIT_SCAN": "#777777",
    "BRAKE": "#db493f",
    "BLOCKED": "#912747",
}


def render(output=PROJECT / "output/slalom.png", return_home=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    course = slalom()
    result = (simulate_round_trip(course.boxes, config_for(course), return_at=42.0)
              if return_home else simulate(course.boxes, config_for(course)))
    figure, axis = plt.subplots(figsize=(13, 6.5))
    for xmin, xmax, ymin, ymax in course.boxes:
        axis.add_patch(Rectangle((xmin, ymin), xmax-xmin, ymax-ymin,
                                 facecolor="#d96549", edgecolor="#80321f", alpha=0.85))
    axis.plot(result.xs, result.ys, color="#c3cad3", linewidth=1.2, zorder=2)
    for state in dict.fromkeys(result.states):
        indices = [index for index, value in enumerate(result.states) if value == state]
        axis.scatter([result.xs[index] for index in indices], [result.ys[index] for index in indices],
                     s=9, color=STATE_COLORS.get(state, "#333333"), label=state, zorder=3)
    # A positive-x arrow is deliberately repeated along the path: heading stays fixed.
    step = max(1, len(result.xs)//18)
    for index in range(0, len(result.xs), step):
        axis.arrow(result.xs[index], result.ys[index], 0.55, 0.0, width=0.018,
                   head_width=0.17, head_length=0.22, color="#17263a",
                   length_includes_head=True, zorder=4)
    elapsed = result.final_time_s if return_home else (result.time_to_goal if result.time_to_goal is not None else result.times[-1])
    clearance = result.min_segment_clearance
    axis.set_title(("Four-gate outbound + LiDAR/PID RETURN_HOME" if return_home
                    else "Four-gate fixed-heading slalom with velocity lag") + " | "
                   f"segment_clearance={clearance:.2f} m | "
                   f"yaw rate={result.max_abs_yaw_rate:.2f}°/s | t={elapsed:.2f} s")
    axis.set_xlabel("Forward distance (m)")
    axis.set_ylabel("Lateral offset (m; right positive)")
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlim(-1, course.goal_x+2)
    axis.set_ylim(-11, 11)
    axis.grid(alpha=0.2)
    axis.legend(loc="upper right", fontsize=8, ncol=2)
    figure.tight_layout()
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160, facecolor="white")
    plt.close(figure)
    return output, result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--return-home", action="store_true", help="also render the active LiDAR/PID return leg")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    path, result = render(args.output or (PROJECT / "output" / ("slalom-round-trip.png" if args.return_home else "slalom.png")),
                          return_home=args.return_home)
    print(f"Saved: {path.resolve()}")
    if args.return_home:
        print(f"Reached return marker: {result.reached_return_marker}; returned home: {result.returned_home}; "+
              f"minimum segment clearance: {result.min_segment_clearance:.3f} m")
    else:
        print(f"Reached goal: {result.reached_goal}; minimum clearance: {result.min_clearance:.3f} m")
