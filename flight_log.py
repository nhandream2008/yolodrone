"""Structured CSV samples plus a JSON run summary for avoidance flights."""

import csv
from datetime import datetime, timezone
import json
from pathlib import Path

from reproducibility import sha256_file


CSV_COLUMNS = (
    # Compatibility fields used by the existing replay.
    "elapsed_s", "state", "front_m", "nearest_m", "forward_m_s", "right_m_s",
    "yaw_deg_s", "north_m", "east_m", "down_m", "reason", "trigger_m", "stop_m",
    "target_lateral_m", "command_lateral_m", "lateral_m", "actual_forward_m_s",
    "actual_right_m_s", "yaw_deg", "course_yaw_deg", "phase", "outbound_forward_m",
    "home_distance_m", "navigation_yaw_deg",
    # Explicit evidence and lifecycle fields.
    "run_id", "world", "event_type", "wall_time_utc", "monotonic_elapsed_s",
    "lidar_sim_time_s", "lidar_age_ms", "transition_reason", "home_north_m",
    "home_east_m", "home_down_m", "altitude_m", "vertical_speed_m_s",
    "requested_north_m_s", "requested_east_m_s", "requested_down_m_s", "measured_north_m_s",
    "measured_east_m_s", "pid_error_m", "pid_p_m_s", "pid_i_m_s", "pid_d_m_s",
    "speed_profile", "profile_simulation_only", "profile_fallback_active", "control_cycle_ms",
    "side_switch_count", "brake_or_stop_state", "safety_severity", "safety_code", "safety_reason",
    "risk_front_m", "risk_closing_speed_m_s", "risk_ttc_s", "risk_uncertainty_margin_m",
    "risk_speed_cap_m_s", "risk_hold_recommended", "risk_track_confidence", "result", "result_reason", "landing_confirmed", "disarmed_confirmed",
    "truth_world_x_m", "truth_world_y_m", "truth_yaw_deg", "roll_deg", "pitch_deg",
)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class FlightRecorder:
    def __init__(self, path, metadata):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path = self.path.with_suffix(".json")
        self.stream = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=CSV_COLUMNS)
        self.writer.writeheader()
        self.metadata = dict(metadata)
        self.metadata["events"] = []
        self.closed = False

    def write(self, values):
        row = {column: "" for column in CSV_COLUMNS}
        row.update({key: value for key, value in values.items() if key in row})
        self.writer.writerow(row)
        self.stream.flush()

    def event(self, elapsed_s, phase, reason):
        self.metadata["events"].append({
            "monotonic_elapsed_s": round(float(elapsed_s), 3),
            "wall_time_utc": utc_now(),
            "phase": str(phase),
            "reason": str(reason),
        })

    def finalize(self, result, reason, home, landing_confirmed, disarmed_confirmed):
        if self.closed:
            return
        self.metadata.update({
            "finished_utc": utc_now(),
            "result": result,
            "result_reason": reason,
            "home": home,
            "landing_confirmed": bool(landing_confirmed),
            "disarmed_confirmed": bool(disarmed_confirmed),
        })
        self.stream.flush()
        self.stream.close()
        # Hash only after the stream has closed, so the sidecar proves the exact
        # append-only CSV evidence that was reviewed offline.
        self.metadata["evidence"] = {
            "csv_path": str(self.path),
            "csv_sha256": sha256_file(self.path),
            "csv_schema_columns": list(CSV_COLUMNS),
        }
        self.summary_path.write_text(json.dumps(self.metadata, ensure_ascii=False, indent=2),
                                     encoding="utf-8")
        self.closed = True
