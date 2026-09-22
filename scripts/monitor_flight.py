"""Read-only live dashboard for an avoid_fly CSV log.

This process imports neither MAVSDK nor Gazebo transport and cannot command a drone.
"""
import argparse
import math
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.plot_slalom_replay import _finite, _flight_rows, course_path, read_log


PHASE_LABELS = {
    "OUTBOUND": "ĐANG ĐI", "TURNAROUND_BRAKE": "GIẢM TỐC ĐỔI CHIỀU",
    "RETURN_HOME": "ĐANG VỀ HOME", "HOME_APPROACH": "TIẾP CẬN HOME",
    "HOME_SETTLE": "ỔN ĐỊNH TẠI HOME", "LANDING": "ĐANG HẠ CÁNH",
    "COMPLETE": "HOÀN TẤT", "FAILED": "THẤT BẠI", "OBSERVE_ONLY": "CHỈ QUAN SÁT",
}


def status_snapshot(rows):
    samples = _flight_rows(rows)
    if not samples:
        raise ValueError("Log chưa có mẫu telemetry")
    row = samples[-1]
    speed = math.hypot(row["measured_north_m_s"], row["measured_east_m_s"])
    if not _finite(speed):
        speed = math.hypot(row["actual_forward_m_s"], row["actual_right_m_s"])
    return {
        "phase": row["phase"] or "LEGACY",
        "phase_label": PHASE_LABELS.get(row["phase"], row["phase"] or "LOG CŨ"),
        "state": row["state"],
        "speed_m_s": speed,
        "altitude_m": row["altitude_m"],
        "home_distance_m": row["home_distance_m"],
        "lidar_age_ms": row["lidar_age_ms"],
        "reason": row["transition_reason"] or row["reason"] or "Không có lý do được ghi",
    }


def _number(value, suffix):
    return f"{value:.2f}{suffix}" if _finite(value) else "không có dữ liệu"


def run_dashboard(log_path, refresh_s):
    import matplotlib.pyplot as plt

    path = Path(log_path)
    plt.ion()
    figure = plt.figure(figsize=(13.5, 8.2))
    grid = figure.add_gridspec(2, 2, height_ratios=(0.30, 0.70))
    status_axis = figure.add_subplot(grid[0, :])
    map_axis = figure.add_subplot(grid[1, 0])
    pid_axis = figure.add_subplot(grid[1, 1])
    last_error = ""
    while plt.fignum_exists(figure.number):
        try:
            rows = read_log(path)
            samples = _flight_rows(rows)
            status = status_snapshot(rows)
            route, _ = course_path(rows)
            last_error = ""
        except (OSError, ValueError) as error:
            last_error = str(error)
            status_axis.clear()
            status_axis.axis("off")
            status_axis.text(0.5, 0.5, f"Đang chờ log {path}\n{last_error}", ha="center", va="center",
                             fontsize=14)
            figure.canvas.draw_idle()
            plt.pause(refresh_s)
            continue

        status_axis.clear()
        status_axis.axis("off")
        status_axis.text(0.01, 0.78, f"{status['phase_label']}  •  PID: {status['state']}",
                         fontsize=18, fontweight="bold", color="#173d5b")
        status_axis.text(
            0.01, 0.43,
            "Tốc độ: " + _number(status["speed_m_s"], " m/s") +
            "     Độ cao: " + _number(status["altitude_m"], " m") +
            "     Home: " + _number(status["home_distance_m"], " m") +
            "     Tuổi LiDAR: " + _number(status["lidar_age_ms"], " ms"),
            fontsize=12,
        )
        status_axis.text(0.01, 0.12, "Vì sao: " + status["reason"], fontsize=11, color="#6e371c")

        map_axis.clear()
        outbound = [point for point in route if point[2]["phase"] in ("", "OUTBOUND", "TURNAROUND_BRAKE")]
        returning = [point for point in route if point[2]["phase"] not in ("", "OUTBOUND", "TURNAROUND_BRAKE")]
        for points, label, color in ((outbound, "Chiều đi", "#2475b0"),
                                     (returning, "Chiều về", "#dc702d")):
            if points:
                map_axis.plot([point[0] for point in points], [point[1] for point in points],
                              color=color, linewidth=2, label=label)
        map_axis.scatter([0], [0], marker="H", s=140, color="#16805b", label="Home", zorder=4)
        map_axis.scatter([route[-1][0]], [route[-1][1]], s=60, color="#20242a", label="Drone", zorder=5)
        map_axis.set_title("Quỹ đạo nhìn từ trên xuống")
        map_axis.set_xlabel("Tiến theo hướng ban đầu (m)")
        map_axis.set_ylabel("Lệch phải (m)")
        map_axis.axis("equal")
        map_axis.grid(alpha=0.2)
        map_axis.legend(loc="best")

        pid_axis.clear()
        elapsed = [row["elapsed_s"] for row in samples]
        for key, label, color, style in (
                ("target_lateral_m", "Đích né", "#ce7720", "--"),
                ("command_lateral_m", "Setpoint PID", "#21855b", ":"),
                ("lateral_m", "Vị trí thực", "#7651b2", "-")):
            values = [row[key] for row in samples]
            if any(_finite(value) for value in values):
                pid_axis.plot(elapsed, values, label=label, color=color,
                              linestyle=style, linewidth=1.7)
        pid_axis.set_title("PID ngang: mục tiêu và vị trí thực")
        pid_axis.set_xlabel("Thời gian đơn điệu (s)")
        pid_axis.set_ylabel("Lệch ngang trong trục điều hướng (m)")
        pid_axis.grid(alpha=0.2)
        pid_axis.legend(loc="best")

        figure.suptitle(f"Theo dõi chỉ đọc — {path.name}", fontsize=13)
        figure.tight_layout()
        figure.canvas.draw_idle()
        plt.pause(refresh_s)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", required=True, help="CSV đang được avoid_fly ghi")
    parser.add_argument("--refresh", type=float, default=0.5, help="Chu kỳ làm mới, giây")
    args = parser.parse_args(argv)
    if not math.isfinite(args.refresh) or not 0.1 <= args.refresh <= 10:
        parser.error("--refresh phải trong khoảng 0.1..10 giây")
    return run_dashboard(args.log, args.refresh)


if __name__ == "__main__":
    sys.exit(main())
