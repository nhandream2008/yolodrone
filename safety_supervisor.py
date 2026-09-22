"""Pure safety-policy evaluation for the PX4/Gazebo SITL runner.

The supervisor has no MAVSDK, Gazebo, ROS, or YOLO dependency.  It translates
validated runtime facts into deterministic severity/reason codes.  The runner
retains sole authority over hold, abort, and landing actions.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class Severity(str, Enum):
    INFO = "INFO"
    DEGRADE = "DEGRADE"
    HOLD = "HOLD"
    ABORT = "ABORT"


@dataclass(frozen=True)
class SafetyFinding:
    severity: Severity
    code: str
    reason: str

    @property
    def hold_position(self) -> bool:
        return self.severity in (Severity.HOLD, Severity.ABORT)

    @property
    def abort_mission(self) -> bool:
        return self.severity is Severity.ABORT


@dataclass(frozen=True)
class SafetyPolicy:
    lidar_hold_after_s: float = 0.75
    lidar_abort_after_s: float = 3.0
    blocked_abort_after_s: float = 8.0
    max_scan_age_for_degrade_s: float = 0.25
    max_tilt_deg: float = 35.0

    def __post_init__(self):
        values = vars(self).values()
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
            raise ValueError("Safety policy must be finite")
        if not (0 < self.max_scan_age_for_degrade_s < self.lidar_hold_after_s < self.lidar_abort_after_s
                and 0 < self.blocked_abort_after_s and 0 < self.max_tilt_deg <= 45):
            raise ValueError("Invalid safety policy")


class SafetySupervisor:
    """Prioritize safety facts and return one stable, machine-readable finding."""

    def __init__(self, policy: SafetyPolicy | None = None):
        self.policy = policy or SafetyPolicy()

    def evaluate(self, *, lidar_age_s: float | None, planner_state: str,
                 missing_for_s: float | None = None, blocked_for_s: float | None = None,
                 telemetry_valid: bool = True, offboard_active: bool = True,
                 roll_deg: float | None = 0.0, pitch_deg: float | None = 0.0) -> SafetyFinding:
        """Evaluate known facts in descending severity without side effects."""
        c = self.policy
        if not telemetry_valid:
            return SafetyFinding(Severity.ABORT, "TELEMETRY_INVALID", "Telemetry is missing, stale, or non-finite")
        if not offboard_active:
            return SafetyFinding(Severity.ABORT, "OFFBOARD_LOST", "PX4 is no longer in Offboard mode")
        tilt = max(abs(roll_deg or 0.0), abs(pitch_deg or 0.0))
        if not math.isfinite(tilt) or tilt > c.max_tilt_deg:
            return SafetyFinding(Severity.ABORT, "ATTITUDE_LIMIT", "Vehicle attitude exceeds the configured safety limit")
        if missing_for_s is not None and missing_for_s >= c.lidar_abort_after_s:
            return SafetyFinding(Severity.ABORT, "LIDAR_TIMEOUT", "LiDAR has been unavailable beyond the abort timeout")
        if blocked_for_s is not None and blocked_for_s >= c.blocked_abort_after_s:
            return SafetyFinding(Severity.ABORT, "CORRIDOR_TIMEOUT", "No safe LiDAR corridor was available beyond the abort timeout")
        if planner_state in {"BRAKE", "BLOCKED"}:
            return SafetyFinding(Severity.HOLD, f"PLANNER_{planner_state}", "Planner requires an immediate safe hold")
        if planner_state == "WAIT_SCAN" or lidar_age_s is None or not math.isfinite(lidar_age_s):
            return SafetyFinding(Severity.HOLD, "LIDAR_UNAVAILABLE", "LiDAR is unavailable or invalid; holding position")
        if lidar_age_s > c.lidar_hold_after_s:
            return SafetyFinding(Severity.HOLD, "LIDAR_STALE", "LiDAR scan age exceeds the hold threshold")
        if lidar_age_s > c.max_scan_age_for_degrade_s:
            return SafetyFinding(Severity.DEGRADE, "LIDAR_AGING", "LiDAR scan age requires a conservative speed cap")
        return SafetyFinding(Severity.INFO, "NOMINAL", "Safety inputs are within policy")
