"""Render an offline top-down map from an observation CSV."""
import argparse
import csv
import math
from pathlib import Path
import sys


CLASS_COLORS = {
    "person": "#8b5cf6", "bicycle": "#16a34a", "car": "#2563eb",
    "motorcycle": "#0f766e", "bus": "#ea580c", "truck": "#be123c",
}


def read_rows(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    required = {"class_name", "north_m", "east_m", "valid_range"}
    if not rows:
        return []
    if missing := required-set(rows[0]):
        raise ValueError(f"Missing CSV columns: {', '.join(sorted(missing))}")
    valid = []
    for row in rows:
        if row["valid_range"].strip().lower() not in ("true", "1"):
            continue
        try:
            north, east = float(row["north_m"]), float(row["east_m"])
        except (TypeError, ValueError):
            continue
        if math.isfinite(north) and math.isfinite(east):
            row["north_m"], row["east_m"] = north, east
            valid.append(row)
    return valid


def render(rows, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    figure, axis = plt.subplots(figsize=(9, 8))
    # Horizontal = east/Y, vertical = north/X, matching the obstacle world.
    axis.add_patch(Rectangle((-4.0, 8.5), 8.0, 1.0, facecolor="#d96549",
                             edgecolor="#80321f", alpha=0.78, label="Front wall"))
    axis.add_patch(Rectangle((-6.5, 15.5), 3.0, 3.0, facecolor="#6194d7",
                             edgecolor="#315b90", alpha=0.78, label="Second obstacle"))
    for class_name in dict.fromkeys(row["class_name"] for row in rows):
        samples = [row for row in rows if row["class_name"] == class_name]
        axis.scatter([row["east_m"] for row in samples], [row["north_m"] for row in samples],
                     s=38, color=CLASS_COLORS.get(class_name, "#374151"),
                     label=class_name, zorder=3)
    axis.scatter([0], [0], marker="^", color="#111827", s=75, label="Camera origin", zorder=4)
    axis.set_title(f"YOLO + LiDAR observations ({len(rows)} valid ranges)")
    axis.set_xlabel("East / world Y (m)")
    axis.set_ylabel("North / world X (m)")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.2)
    axis.legend(loc="best")
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(output, dpi=160, facecolor="white")
    plt.close(figure)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", help="detections-*.csv written by the observation node")
    parser.add_argument("--output", default="output/detection-map.png")
    args = parser.parse_args(argv)
    try:
        output = render(read_rows(args.csv), args.output)
    except (OSError, ValueError, ImportError) as error:
        parser.exit(1, f"Cannot render detection map: {error}\n")
    print(f"Saved: {output.resolve()}")


if __name__ == "__main__":
    main()
