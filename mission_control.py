"""Pure round-trip mission phases shared by the live and offline simulations."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MissionConfig:
    return_enabled: bool = True
    return_at_m: float = 42.0
    arrival_radius_m: float = 1.5
    approach_radius_m: float = 5.0
    turnaround_speed_m_s: float = 0.15
    turnaround_settle_s: float = 0.5
    turnaround_timeout_s: float = 12.0
    turnaround_approach_deceleration_m_s2: float = 0.7
    home_settle_speed_m_s: float = 0.15
    home_settle_s: float = 1.0
    home_settle_timeout_s: float = 10.0
    return_timeout_s: float = 180.0
    max_radius_m: float = 48.0
    min_altitude_m: float = 1.5
    max_altitude_m: float = 4.5
    approach_speed_m_s: float = 0.75
    approach_deceleration_m_s2: float = 0.8
    away_tolerance_m: float = 0.75
    away_timeout_s: float = 2.0

    def __post_init__(self):
        numeric = [value for key, value in vars(self).items() if key != "return_enabled"]
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("Mission configuration must be finite")
        return_geometry_valid = (5 <= self.return_at_m < self.max_radius_m
                                 and self.approach_radius_m < self.return_at_m)
        if not ((not self.return_enabled or return_geometry_valid)
                and self.max_radius_m <= 50
                and 0.5 <= self.arrival_radius_m < self.approach_radius_m
                and 0 < self.turnaround_speed_m_s <= 0.5
                and 0 <= self.turnaround_settle_s < self.turnaround_timeout_s
                and 0 < self.turnaround_approach_deceleration_m_s2 <= 2
                and 0 < self.home_settle_speed_m_s <= 0.5
                and 0 <= self.home_settle_s < self.home_settle_timeout_s
                and self.return_timeout_s > self.turnaround_timeout_s
                and 0 <= self.min_altitude_m < self.max_altitude_m <= 10
                and 0 < self.approach_speed_m_s <= 1.5
                and 0 < self.approach_deceleration_m_s2 <= 2
                and self.away_tolerance_m > 0 and self.away_timeout_s > 0):
            raise ValueError("Invalid round-trip mission configuration")


@dataclass(frozen=True)
class MissionDirective:
    phase: str
    hold_position: bool = False
    switch_to_return_frame: bool = False
    request_land: bool = False
    failed: bool = False
    speed_limit_m_s: float | None = None
    transition_reason: str = ""


class RoundTripMission:
    """Supervise mission phases without importing Gazebo, MAVSDK, or a flight API."""

    def __init__(self, config):
        self.config = config
        self.phase = "OUTBOUND"
        self.phase_started_at = None
        self.stable_since = None
        self.away_since = None
        self.best_home_distance = math.inf
        self.failure_reason = ""

    def _transition(self, phase, now, reason):
        self.phase = phase
        self.phase_started_at = now
        self.stable_since = None
        self.away_since = None
        if phase in ("RETURN_HOME", "HOME_APPROACH"):
            self.best_home_distance = math.inf
        return reason

    def fail(self, now, reason):
        self.failure_reason = reason
        transition = self._transition("FAILED", now, reason)
        return MissionDirective("FAILED", hold_position=True, request_land=True,
                                failed=True, transition_reason=transition)

    def begin_landing(self, now, reason):
        transition = self._transition("LANDING", now, reason)
        return MissionDirective("LANDING", hold_position=True, request_land=True,
                                transition_reason=transition)

    def complete(self, now):
        transition = self._transition("COMPLETE", now, "Landed and disarmed")
        return MissionDirective("COMPLETE", hold_position=True,
                                transition_reason=transition)

    def update(self, now, outbound_forward_m, home_distance_m, horizontal_speed_m_s,
               altitude_m):
        values = (now, outbound_forward_m, home_distance_m, horizontal_speed_m_s, altitude_m)
        if not all(math.isfinite(value) for value in values):
            return self.fail(now if math.isfinite(now) else 0.0,
                             "Invalid mission position, velocity, or altitude")
        if self.phase_started_at is None:
            self.phase_started_at = now
        c = self.config
        if self.phase in ("FAILED", "LANDING", "COMPLETE"):
            return MissionDirective(self.phase, hold_position=True,
                                    request_land=self.phase in ("FAILED", "LANDING"),
                                    failed=self.phase == "FAILED")
        if home_distance_m > c.max_radius_m:
            return self.fail(now, f"Safety radius exceeded ({home_distance_m:.2f} m)")
        if not c.min_altitude_m <= altitude_m <= c.max_altitude_m:
            return self.fail(now, f"Altitude outside safety band ({altitude_m:.2f} m)")

        transition = ""
        if self.phase == "OUTBOUND":
            if c.return_enabled and outbound_forward_m >= c.return_at_m:
                transition = self._transition(
                    "TURNAROUND_BRAKE", now,
                    "Outbound marker reached; brake before reversing navigation frame")
                return MissionDirective(self.phase, hold_position=True,
                                        transition_reason=transition)
            if c.return_enabled:
                # Approach the return marker at a speed that can stop there.
                # The runner still applies its vector slew limit, so this is a
                # gradual cap rather than a discontinuous velocity command.
                remaining = max(0.0, c.return_at_m-outbound_forward_m)
                braking_limit = math.sqrt(
                    2.0*c.turnaround_approach_deceleration_m_s2*remaining)
                return MissionDirective(self.phase, speed_limit_m_s=braking_limit)
            return MissionDirective(self.phase)

        if self.phase == "TURNAROUND_BRAKE":
            if now-self.phase_started_at > c.turnaround_timeout_s:
                return self.fail(now, "Could not settle before return-frame switch")
            if horizontal_speed_m_s <= c.turnaround_speed_m_s:
                self.stable_since = self.stable_since or now
                if now-self.stable_since >= c.turnaround_settle_s:
                    transition = self._transition(
                        "RETURN_HOME", now,
                        "Vehicle settled; activate reverse LiDAR/PID navigation frame")
                    self.best_home_distance = home_distance_m
                    return MissionDirective(self.phase, hold_position=True,
                                            switch_to_return_frame=True,
                                            transition_reason=transition)
            else:
                self.stable_since = None
            return MissionDirective(self.phase, hold_position=True)

        if self.phase in ("RETURN_HOME", "HOME_APPROACH"):
            if now-self.phase_started_at > c.return_timeout_s:
                return self.fail(now, "Return-to-Home phase timed out")
            self.best_home_distance = min(self.best_home_distance, home_distance_m)
            if outbound_forward_m < -c.arrival_radius_m and home_distance_m > c.arrival_radius_m:
                return self.fail(now, "Passed Home without entering the arrival radius")
            if self.phase == "RETURN_HOME" and home_distance_m <= c.approach_radius_m:
                transition = self._transition(
                    "HOME_APPROACH", now, "Inside Home approach radius; reduce speed")
                self.best_home_distance = home_distance_m
            elif self.phase == "RETURN_HOME":
                # Arrive at the approach boundary near approach_speed instead
                # of waiting until that boundary to brake from cruise speed.
                room = home_distance_m-c.approach_radius_m
                braking_limit = math.sqrt(
                    c.approach_speed_m_s**2
                    + 2.0*c.approach_deceleration_m_s2*room)
                return MissionDirective(self.phase, speed_limit_m_s=braking_limit)
            if self.phase == "HOME_APPROACH":
                if home_distance_m <= c.arrival_radius_m:
                    transition = self._transition(
                        "HOME_SETTLE", now, "Inside Home radius; hold and verify velocity")
                    return MissionDirective(self.phase, hold_position=True,
                                            transition_reason=transition)
                if home_distance_m > self.best_home_distance+c.away_tolerance_m:
                    self.away_since = self.away_since or now
                    if now-self.away_since >= c.away_timeout_s:
                        return self.fail(now, "Home error is increasing during final approach")
                else:
                    self.away_since = None
                room = max(0.0, home_distance_m-c.arrival_radius_m)
                braking_limit = math.sqrt(2.0*c.approach_deceleration_m_s2*room)
                speed_limit = min(c.approach_speed_m_s, max(0.10, braking_limit))
                return MissionDirective(self.phase, speed_limit_m_s=speed_limit,
                                        transition_reason=transition)
            return MissionDirective(self.phase, transition_reason=transition)

        if self.phase == "HOME_SETTLE":
            if now-self.phase_started_at > c.home_settle_timeout_s:
                return self.fail(now, "Could not settle inside Home radius")
            if home_distance_m > c.arrival_radius_m:
                transition = self._transition(
                    "HOME_APPROACH", now, "Drifted outside Home radius; reacquire Home")
                self.best_home_distance = home_distance_m
                return MissionDirective(self.phase, speed_limit_m_s=0.10,
                                        transition_reason=transition)
            if horizontal_speed_m_s <= c.home_settle_speed_m_s:
                self.stable_since = self.stable_since or now
                if now-self.stable_since >= c.home_settle_s:
                    return self.begin_landing(now, "Home position and velocity stable")
            else:
                self.stable_since = None
            return MissionDirective(self.phase, hold_position=True)

        return self.fail(now, f"Unknown mission phase: {self.phase}")
