"""Deterministic temporal LiDAR risk evidence for the SITL avoidance controller.

This module is deliberately read-only: it receives a validated planar scan and
measured motion, estimates a conservative closing speed/TTC, and returns a speed
cap.  It never creates position, yaw, arm, takeoff, landing, or MAVSDK commands.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from obstacle_avoidance import Motion, Scan, clamp, tinh_van_toc_an_toan


@dataclass(frozen=True)
class RiskConfig:
    corridor_half_width_m: float = 1.3
    clearance_m: float = 1.3
    brake_accel_m_s2: float = 1.5
    reaction_time_s: float = 1.0
    max_track_age_s: float = 0.8
    uncertainty_speed_penalty_m_s_per_s: float = 0.8
    ttc_hold_s: float = 0.65

    def __post_init__(self):
        values = vars(self).values()
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
            raise ValueError("Risk configuration must be finite")
        if not (0.5 <= self.corridor_half_width_m <= 4 and 0.7 <= self.clearance_m <= 2.5
                and 0 < self.brake_accel_m_s2 <= 2 and 0 < self.reaction_time_s <= 3
                and 0.1 <= self.max_track_age_s <= 2 and 0 <= self.uncertainty_speed_penalty_m_s_per_s <= 3
                and 0.1 <= self.ttc_hold_s <= 3):
            raise ValueError("Invalid temporal LiDAR risk configuration")


@dataclass(frozen=True)
class TemporalRisk:
    front_distance_m: float | None
    closing_speed_m_s: float | None
    time_to_collision_s: float | None
    scan_age_s: float | None
    uncertainty_margin_m: float
    speed_cap_m_s: float | None
    hold_recommended: bool
    track_confidence: float
    reason: str


class TemporalLidarRisk:
    """Track the nearest forward-corridor distance across successive scans."""

    def __init__(self, config: RiskConfig | None = None):
        self.config = config or RiskConfig()
        self._previous_distance: float | None = None
        self._previous_at: float | None = None

    def reset(self) -> None:
        self._previous_distance = None
        self._previous_at = None

    def _front_distance(self, scan: Scan, motion: Motion) -> float | None:
        readings = []
        for angle, distance in zip(scan.angles, scan.ranges):
            # A finite range_max is Gazebo's normalized no-hit horizon, not an
            # obstacle. Match the main planner and retain only real returns.
            if not (math.isfinite(angle) and math.isfinite(distance)
                    and 0 < distance < scan.range_max-0.01):
                continue
            forward = distance * math.cos(angle - motion.yaw_delta_rad)
            right = -distance * math.sin(angle - motion.yaw_delta_rad)
            if forward >= 0 and abs(right) <= self.config.corridor_half_width_m:
                readings.append(forward)
        return min(readings) if readings else None

    def update(self, scan: Scan | None, now: float, motion: Motion | None = None) -> TemporalRisk:
        c = self.config
        motion = motion or Motion()
        if scan is None or not math.isfinite(now):
            self.reset()
            return TemporalRisk(None, None, None, None, 0.0, 0.0, True, 0.0, "No valid LiDAR scan")
        scan_age = now - scan.received_at
        if not math.isfinite(scan_age) or scan_age < 0 or scan_age > c.max_track_age_s:
            self.reset()
            return TemporalRisk(None, None, None, scan_age, 0.0, 0.0, True, 0.0, "LiDAR scan is outside temporal tracking window")
        front = self._front_distance(scan, motion)
        if front is None:
            # A valid scan containing only normalized range_max no-hit rays is
            # clear to the sensor horizon. It is not missing data and must not
            # manufacture a hold or speed cap.
            self.reset()
            return TemporalRisk(None, 0.0, math.inf, scan_age, 0.0, math.inf, False, 1.0,
                                "No forward obstacle return within sensor horizon")
        closing_from_range = 0.0
        confidence = 0.5
        if self._previous_distance is not None and self._previous_at is not None:
            dt = scan.received_at - self._previous_at
            if 0 < dt <= c.max_track_age_s:
                closing_from_range = max(0.0, (self._previous_distance-front)/dt)
                confidence = 1.0
        self._previous_distance, self._previous_at = front, scan.received_at
        measured_forward = max(0.0, motion.forward_m_s if math.isfinite(motion.forward_m_s) else 0.0)
        closing_speed = max(measured_forward, closing_from_range)
        uncertainty = max(0.0, scan_age) * c.uncertainty_speed_penalty_m_s_per_s
        usable_distance = max(0.0, front-c.clearance_m-uncertainty)
        ttc = usable_distance/closing_speed if closing_speed > 1e-6 else math.inf
        cap = tinh_van_toc_an_toan(front, c.clearance_m+uncertainty, c.brake_accel_m_s2,
                                   c.reaction_time_s+max(0.0, scan_age))
        hold = front <= c.clearance_m+uncertainty or ttc <= c.ttc_hold_s
        return TemporalRisk(
            front, closing_speed, ttc, scan_age, uncertainty, clamp(cap, 0.0, math.inf), hold,
            confidence, "TTC hold threshold reached" if hold else "Temporal LiDAR risk nominal",
        )
