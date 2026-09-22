"""Deterministic LiDAR/PID slalom simulation used for offline verification only.

This module has no MAVSDK, Gazebo, ROS, or process-control dependency.  It models
the vehicle response between controller updates so a controller command is never
silently treated as the motion that occurred during that same update.
"""
from dataclasses import dataclass, replace
import math
import random

from mission_control import MissionConfig, RoundTripMission
from obstacle_avoidance import AvoidanceConfig, Motion, Planner, Scan


@dataclass(frozen=True)
class SlalomCourse:
    """Axis-aligned walls in the fixed initial course frame (forward, right)."""
    boxes: tuple
    goal_x: float


@dataclass(frozen=True)
class OfflineDynamics:
    """Repeatable first-order velocity tracking and sensor fault model."""
    forward_time_constant_s: float = 0.35
    lateral_time_constant_s: float = 0.55
    forward_accel_m_s2: float = 1.20
    lateral_accel_m_s2: float = 1.00
    vehicle_radius_m: float = 0.30
    lidar_noise_std_m: float = 0.0
    single_scan_drop_probability: float = 0.0
    burst_start_probability: float = 0.0
    burst_length: int = 3
    update_jitter_fraction: float = 0.0

    def __post_init__(self):
        numeric = (
            self.forward_time_constant_s, self.lateral_time_constant_s,
            self.forward_accel_m_s2, self.lateral_accel_m_s2, self.vehicle_radius_m,
            self.lidar_noise_std_m, self.single_scan_drop_probability,
            self.burst_start_probability, self.update_jitter_fraction,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("Offline dynamics values must be finite")
        if not (0 < self.forward_time_constant_s <= 5
                and 0 < self.lateral_time_constant_s <= 5
                and 0 < self.forward_accel_m_s2 <= 5
                and 0 < self.lateral_accel_m_s2 <= 5
                and 0 < self.vehicle_radius_m <= 1
                and self.lidar_noise_std_m >= 0
                and 0 <= self.single_scan_drop_probability < 1
                and 0 <= self.burst_start_probability < 1
                and 1 <= self.burst_length <= 20
                and 0 <= self.update_jitter_fraction < 0.9):
            raise ValueError("Invalid offline dynamics")


@dataclass
class SimulationResult:
    xs: list
    ys: list
    states: list
    times: list
    decisions: list
    actual_forward_m_s: list
    actual_right_m_s: list
    min_clearance: float
    min_segment_clearance: float
    max_abs_yaw_rate: float
    reached_goal: bool
    collided: bool
    forward_stalls: int
    direction_flips: int
    dropped_scans: int
    time_to_goal: float | None


@dataclass
class RoundTripResult:
    xs: list
    ys: list
    phases: list
    states: list
    decisions: list
    min_segment_clearance: float
    max_abs_yaw_rate: float
    reached_return_marker: bool
    returned_home: bool
    collided: bool
    dropped_scans: int
    final_time_s: float
    terminal_phase: str
    failure_reason: str
    home_error_m: float
    final_speed_m_s: float


def slalom(gates=4, spacing=7.0, opening=1.0, first_x=8.0):
    """Alternating one-sided gates matching the Gazebo course's first right bypass."""
    boxes = []
    for index in range(gates):
        x = first_x + index*spacing
        # In the physical ENU world the body-right coordinate is -Gazebo Y.
        # Gate 1 blocks the left side and therefore opens to positive/right.
        if index % 2 == 0:
            boxes.append((x-0.3, x+0.3, -49.0, opening))
        else:
            boxes.append((x-0.3, x+0.3, -opening, 49.0))
    return SlalomCourse(tuple(boxes), first_x + gates*spacing + 6.0)


def config_for(course, speed=1.5):
    return AvoidanceConfig(speed=speed, return_to_course=True, goal_forward_m=course.goal_x)


def _distance_to_box(x, y, box):
    xmin, xmax, ymin, ymax = box
    return math.hypot(max(xmin-x, 0.0, x-xmax), max(ymin-y, 0.0, y-ymax))


def _point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx-ax, by-ay
    denominator = dx*dx + dy*dy
    fraction = 0.0 if denominator <= 1e-15 else max(0.0, min(1.0, ((px-ax)*dx+(py-ay)*dy)/denominator))
    return math.hypot(px-(ax+fraction*dx), py-(ay+fraction*dy))


def _segment_intersects_box(ax, ay, bx, by, box):
    """Liang-Barsky clipping; intersection includes touching an edge."""
    xmin, xmax, ymin, ymax = box
    dx, dy = bx-ax, by-ay
    lower, upper = 0.0, 1.0
    for origin, direction, low, high in ((ax, dx, xmin, xmax), (ay, dy, ymin, ymax)):
        if abs(direction) < 1e-15:
            if origin < low or origin > high:
                return False
            continue
        first, second = (low-origin)/direction, (high-origin)/direction
        lower, upper = max(lower, min(first, second)), min(upper, max(first, second))
        if lower > upper:
            return False
    return True


def segment_box_distance(ax, ay, bx, by, box):
    """Exact minimum distance between a line segment and an axis-aligned box."""
    if _segment_intersects_box(ax, ay, bx, by, box):
        return 0.0
    xmin, xmax, ymin, ymax = box
    corners = ((xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax))
    return min(
        _distance_to_box(ax, ay, box), _distance_to_box(bx, by, box),
        *(_point_segment_distance(x, y, ax, ay, bx, by) for x, y in corners),
    )


def box_scan(now, boxes=(), x=0.0, y=0.0, range_max=20.0):
    """Ray-cast the fixed course frame with the same 360-degree scan shape as SITL."""
    angles = tuple(math.radians(degree) for degree in range(-180, 181))
    ranges = []
    for angle in angles:
        # LiDAR positive angles are vehicle-left; the course coordinate is right-positive.
        dx, dy = math.cos(angle), -math.sin(angle)
        nearest = range_max
        for xmin, xmax, ymin, ymax in boxes:
            enter, leave = -math.inf, math.inf
            for origin, direction, low, high in ((x, dx, xmin, xmax), (y, dy, ymin, ymax)):
                if abs(direction) < 1e-12:
                    if not low <= origin <= high:
                        enter, leave = math.inf, -math.inf
                        break
                else:
                    first, second = (low-origin)/direction, (high-origin)/direction
                    enter, leave = max(enter, min(first, second)), min(leave, max(first, second))
            if leave >= max(enter, 0.0):
                nearest = min(nearest, max(0.0, enter))
        ranges.append(nearest)
    return Scan(angles, tuple(ranges), now, range_max)


def _noisy_scan(scan, noise_std, rng):
    if noise_std <= 0:
        return scan
    ranges = tuple(
        value if value >= scan.range_max-0.01 else max(0.0, rng.gauss(value, noise_std))
        for value in scan.ranges
    )
    return Scan(scan.angles, ranges, scan.received_at, scan.range_max)


def _track_velocity(actual, command, time_constant, max_accel, dt):
    requested_accel = (command-actual)/time_constant
    accel = max(-max_accel, min(max_accel, requested_accel))
    next_velocity = actual + accel*dt
    # A coarse, jittered controller tick must not overshoot its velocity setpoint.
    if (command-actual)*(command-next_velocity) < 0:
        return command
    return next_velocity


def simulate(boxes, config, dt=0.05, max_steps=4000, noise=None, seed=None, dynamics=None):
    """Run an offline replay with physical response, faults, and segment clearance checks.

    ``noise`` remains a compatibility shortcut for the older test API.  It enables
    Gaussian LiDAR noise, 10% isolated drops, short drop bursts, and 25% update jitter.
    All of those behaviours are deterministic when ``seed`` is supplied.
    """
    if not math.isfinite(dt) or dt <= 0 or max_steps <= 0:
        raise ValueError("dt and max_steps must be positive")
    dynamics = dynamics or OfflineDynamics()
    if noise is not None:
        if not math.isfinite(float(noise)) or noise < 0:
            raise ValueError("noise must be finite and non-negative")
        dynamics = replace(dynamics, lidar_noise_std_m=float(noise),
                           single_scan_drop_probability=0.10,
                           burst_start_probability=0.025,
                           burst_length=3, update_jitter_fraction=0.25)
    rng = random.Random(seed)
    planner = Planner(config)
    goal = config.goal_forward_m
    x = y = vx = vy = now = 0.0
    xs, ys, states, times, decisions, actual_forwards, actual_rights = [], [], [], [], [], [], []
    clearance = segment_clearance = math.inf
    max_yaw = 0.0
    last_direction = 0
    flips = stalls = dropped_scans = 0
    burst_remaining = 0
    reached_at = None
    collided = False
    for _ in range(max_steps):
        interval = dt*(1.0+rng.uniform(-dynamics.update_jitter_fraction, dynamics.update_jitter_fraction))
        now += interval
        scan = box_scan(now, boxes, x, y)
        if burst_remaining:
            burst_remaining -= 1
            scan = None
        elif rng.random() < dynamics.burst_start_probability:
            burst_remaining = dynamics.burst_length-1
            scan = None
        elif rng.random() < dynamics.single_scan_drop_probability:
            scan = None
        if scan is None:
            dropped_scans += 1
        else:
            scan = _noisy_scan(scan, dynamics.lidar_noise_std_m, rng)
        decision = planner.update(scan, now, Motion(forward_m_s=vx, right_m_s=vy,
                                                     lateral_m=y, forward_m=x))
        next_vx = _track_velocity(vx, decision.forward_m_s, dynamics.forward_time_constant_s,
                                  dynamics.forward_accel_m_s2, interval)
        next_vy = _track_velocity(vy, decision.right_m_s, dynamics.lateral_time_constant_s,
                                  dynamics.lateral_accel_m_s2, interval)
        next_x = x + (vx+next_vx)*0.5*interval
        next_y = y + (vy+next_vy)*0.5*interval
        step_clearance = min(segment_box_distance(x, y, next_x, next_y, box) for box in boxes) if boxes else math.inf
        segment_clearance = min(segment_clearance, step_clearance)
        clearance = min(clearance, *(_distance_to_box(next_x, next_y, box) for box in boxes)) if boxes else math.inf
        collided = collided or step_clearance <= dynamics.vehicle_radius_m
        x, y, vx, vy = next_x, next_y, next_vx, next_vy
        xs.append(x)
        ys.append(y)
        states.append(decision.state)
        times.append(now)
        decisions.append(decision)
        actual_forwards.append(vx)
        actual_rights.append(vy)
        max_yaw = max(max_yaw, abs(decision.yaw_deg_s))
        stalls += vx < 0.05
        direction = 1 if vy > 0.15 else -1 if vy < -0.15 else 0
        if direction and last_direction and direction != last_direction:
            flips += 1
        if direction:
            last_direction = direction
        if goal and x >= goal:
            reached_at = now
            break
        if collided:
            break
    return SimulationResult(xs, ys, states, times, decisions, actual_forwards, actual_rights,
                            clearance, segment_clearance, max_yaw, reached_at is not None and not collided,
                            collided, stalls, flips, dropped_scans, reached_at)


def simulate_round_trip(boxes, config, return_at, arrival_radius=1.5, dt=0.05,
                        max_steps=8000, noise=None, seed=None, dynamics=None,
                        return_boxes=(), home_y=0.0):
    """Exercise the same body-fixed-yaw return logic as ``avoid_fly.py`` offline.

    The vehicle keeps its physical nose along the original +X course.  On the
    return leg only the planner frame rotates 180 degrees, so the rear LiDAR
    hemisphere becomes the planner's forward safety corridor.
    """
    if not (math.isfinite(return_at) and math.isfinite(arrival_radius) and math.isfinite(home_y)
            and return_at > arrival_radius > 0):
        raise ValueError("return_at must be greater than a positive arrival radius")
    if not math.isfinite(dt) or dt <= 0 or max_steps <= 0:
        raise ValueError("dt and max_steps must be positive")
    dynamics = dynamics or OfflineDynamics()
    if noise is not None:
        if not math.isfinite(float(noise)) or noise < 0:
            raise ValueError("noise must be finite and non-negative")
        dynamics = replace(dynamics, lidar_noise_std_m=float(noise),
                           single_scan_drop_probability=0.10,
                           burst_start_probability=0.025,
                           burst_length=3, update_jitter_fraction=0.25)
    rng = random.Random(seed)
    planner = Planner(config)
    mission = RoundTripMission(MissionConfig(
        return_at_m=return_at,
        arrival_radius_m=arrival_radius,
        max_radius_m=min(50.0, max(return_at+5.0, return_at+arrival_radius+0.5)),
        min_altitude_m=1.0,
        max_altitude_m=5.0,
    ))
    x, y, vx, vy, now = 0.0, home_y, 0.0, 0.0, 0.0
    xs, ys, phases, states, decisions = [], [], [], [], []
    min_segment_clearance = math.inf
    max_yaw = 0.0
    burst_remaining = dropped_scans = 0
    missing_since = blocked_since = None
    reached_marker = returned_home = collided = False
    for _ in range(max_steps):
        interval = dt*(1.0+rng.uniform(-dynamics.update_jitter_fraction, dynamics.update_jitter_fraction))
        now += interval
        directive = mission.update(now, x, math.hypot(x, y-home_y), math.hypot(vx, vy), 3.0)
        if directive.phase == "TURNAROUND_BRAKE":
            reached_marker = True
        if directive.switch_to_return_frame:
            planner = Planner(config)
        return_frame = directive.phase in (
            "RETURN_HOME", "HOME_APPROACH", "HOME_SETTLE", "LANDING", "FAILED")
        active_boxes = tuple(boxes) + (tuple(return_boxes) if return_frame else ())
        scan = box_scan(now, active_boxes, x, y)
        if burst_remaining:
            burst_remaining -= 1
            scan = None
        elif rng.random() < dynamics.burst_start_probability:
            burst_remaining = dynamics.burst_length-1
            scan = None
        elif rng.random() < dynamics.single_scan_drop_probability:
            scan = None
        if scan is None:
            dropped_scans += 1
        else:
            scan = _noisy_scan(scan, dynamics.lidar_noise_std_m, rng)

        if return_frame:
            # Planner sees a virtual forward direction toward Home while the
            # yaw stays fixed.  -pi maps raw rear LiDAR rays into that frame.
            motion = Motion(forward_m_s=-vx, right_m_s=-vy, lateral_m=-(y-home_y),
                            yaw_delta_rad=-math.pi, forward_m=-x)
        else:
            motion = Motion(forward_m_s=vx, right_m_s=vy, lateral_m=y-home_y, forward_m=x)
        decision = planner.update(scan, now, motion)
        missing_since = (missing_since or now) if decision.state == "WAIT_SCAN" else None
        blocked_since = (blocked_since or now) if decision.state in ("BLOCKED", "BRAKE") else None
        if missing_since is not None and now-missing_since >= 3.0:
            directive = mission.fail(now, "LiDAR unavailable for 3 seconds")
        elif blocked_since is not None and now-blocked_since >= 8.0:
            directive = mission.fail(now, "No safe corridor for 8 seconds")
        requested_vx, requested_vy = (decision.forward_m_s, decision.right_m_s)
        if return_frame:
            requested_vx, requested_vy = -requested_vx, -requested_vy
        if directive.hold_position:
            requested_vx = requested_vy = 0.0
        elif directive.speed_limit_m_s is not None:
            command_speed = math.hypot(requested_vx, requested_vy)
            if command_speed > directive.speed_limit_m_s:
                scale = directive.speed_limit_m_s/command_speed
                requested_vx, requested_vy = requested_vx*scale, requested_vy*scale
        next_vx = _track_velocity(vx, requested_vx, dynamics.forward_time_constant_s,
                                  dynamics.forward_accel_m_s2, interval)
        next_vy = _track_velocity(vy, requested_vy, dynamics.lateral_time_constant_s,
                                  dynamics.lateral_accel_m_s2, interval)
        next_x = x + (vx+next_vx)*0.5*interval
        next_y = y + (vy+next_vy)*0.5*interval
        clearance = min(segment_box_distance(x, y, next_x, next_y, box)
                        for box in active_boxes) if active_boxes else math.inf
        min_segment_clearance = min(min_segment_clearance, clearance)
        collided = collided or clearance <= dynamics.vehicle_radius_m
        x, y, vx, vy = next_x, next_y, next_vx, next_vy
        xs.append(x)
        ys.append(y)
        phases.append(directive.phase)
        states.append(decision.state)
        decisions.append(decision)
        max_yaw = max(max_yaw, abs(decision.yaw_deg_s))
        if directive.request_land:
            returned_home = (directive.phase == "LANDING"
                             and math.hypot(x, y-home_y) <= arrival_radius)
            break
        if collided:
            break
    return RoundTripResult(xs, ys, phases, states, decisions, min_segment_clearance, max_yaw,
                           reached_marker, returned_home, collided, dropped_scans, now,
                           mission.phase, mission.failure_reason, math.hypot(x, y-home_y),
                           math.hypot(vx, vy))


def lateral_at_forward(result, forward_m):
    """Interpolate the recorded course-right position at a gate plane."""
    if not result.xs or not math.isfinite(forward_m):
        return None
    previous_x, previous_y = 0.0, 0.0
    for x, y in zip(result.xs, result.ys):
        if previous_x <= forward_m <= x and x > previous_x:
            fraction = (forward_m-previous_x)/(x-previous_x)
            return previous_y+fraction*(y-previous_y)
        previous_x, previous_y = x, y
    return None
