"""Continuous fixed-heading bypass with a lateral PID and braking envelope."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Scan:
    angles: tuple
    ranges: tuple
    received_at: float
    range_max: float = 20.0

    @classmethod
    def from_message(cls, msg, received_at, *, cho_phep_inf_khong_va_cham=False):
        count = int(msg.count)
        values = (msg.angle_min, msg.angle_max, msg.angle_step,
                  msg.range_min, msg.range_max, received_at)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Non-finite scan geometry")
        if count < 36 or msg.vertical_count != 1 or len(msg.ranges) != count:
            raise ValueError("Expected one complete planar scan")
        if not 0 <= msg.range_min < msg.range_max or msg.angle_step <= 0:
            raise ValueError("Invalid scan range/angle limits")
        if abs(msg.angle_min + (count - 1) * msg.angle_step - msg.angle_max) > 0.02:
            raise ValueError("Inconsistent scan angular geometry")
        if abs(msg.angle_min + math.pi) > 0.1 or abs(msg.angle_max - math.pi) > 0.1:
            raise ValueError("A full 360-degree scan is required")
        ranges = []
        for value in msg.ranges:
            # Gazebo gpu_lidar dùng +Inf cho tia không chạm; adapter transport
            # duy nhất mới được chuẩn hóa nó thành chân trời range_max hữu hạn.
            # NaN, -Inf và +Inf từ mọi nguồn khác đều là lỗi fail-closed.
            if math.isinf(value) and value > 0 and cho_phep_inf_khong_va_cham:
                ranges.append(msg.range_max)
                continue
            if not math.isfinite(value) or value < 0:
                raise ValueError("Giá trị LiDAR không hữu hạn hoặc âm")
            # Các return quá gần vẫn là vật cản, tuyệt đối không coi là khoảng trống.
            ranges.append(0.0 if value < msg.range_min else min(value, msg.range_max))
        return cls(tuple(msg.angle_min + i * msg.angle_step for i in range(count)),
                   tuple(ranges), received_at, msg.range_max)

    def sector(self, center, half_width):
        readings = [r for a, r in zip(self.angles, self.ranges)
                    if abs(math.atan2(math.sin(a-center), math.cos(a-center))) <= half_width]
        return min(readings) if readings else 0.0


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class AvoidanceConfig:
    speed: float = 2.5
    detect_distance: float = 10.0
    clearance: float = 1.3
    emergency_distance: float = 0.85
    max_accel: float = 1.0
    brake_accel: float = 1.5
    reaction_time: float = 1.0
    preview_time: float = 2.0
    scan_timeout: float = 0.75
    clear_seconds: float = 0.7
    max_lateral_speed: float = 1.8
    kp: float = 0.8
    ki: float = 0.03
    kd: float = 0.4
    integral_limit: float = 1.5
    offset_margin: float = 0.4
    approach_margin: float = 1.6
    tracking_fraction: float = 0.65
    min_ramp_speed: float = 0.2
    clear_fov_deg: float = 170.0
    # Once the obstacle has left the confirmed-clear forward view, ease back
    # to the original course.  This is lateral-only; it never turns the drone.
    return_to_course: bool = True
    goal_forward_m: float = 0.0

    # Giới hạn cứng độ lệch ngang so với tim trục hành trình (mét).
    # Đây là tham số an toàn tổng quát, không mô tả hay mã hóa bất kỳ world nào.
    corridor_half_width: float = 10.0

    # Hysteresis for lane locking: minimum lateral improvement (m) to justify a side switch.
    side_switch_hysteresis_m: float = 0.5
    # Minimum time between side switches (seconds).
    side_switch_min_interval: float = 1.2

    # Dynamic forward speed reduction: when obstacle within this fraction of detect_distance,
    # begin reducing forward speed linearly to min_forward_speed.
    speed_reduction_start_frac: float = 0.9
    min_forward_speed: float = 1.1

    def __post_init__(self):
        numeric = {key: value for key, value in vars(self).items()
                   if key != "return_to_course"}
        if not all(math.isfinite(v) for v in numeric.values()):
            raise ValueError("Configuration must be finite")
        # Auto-cap min_forward_speed at speed for backward compatibility
        if self.min_forward_speed > self.speed:
            object.__setattr__(self, 'min_forward_speed', self.speed)
        if not (0 < self.speed <= 3 and 5 <= self.detect_distance <= 15
                and 0.7 <= self.emergency_distance < self.clearance <= 2.5
                and 0 < self.max_accel <= 2 and 0 < self.brake_accel <= 2
                and self.reaction_time >= self.scan_timeout + 0.1 and self.preview_time >= 1
                and 0 < self.scan_timeout <= 1 and self.clear_seconds >= 0.3
                and 0 < self.max_lateral_speed <= 2 and self.kp > 0
                and self.ki >= 0 and self.kd >= 0 and self.integral_limit > 0
                and self.offset_margin >= 0.2 and 1 <= self.approach_margin <= 4
                and 0.2 <= self.tracking_fraction <= 1 and 0 < self.min_ramp_speed <= 1
                and 40 <= self.clear_fov_deg <= 180 and self.goal_forward_m >= 0
                and 0.5 <= self.corridor_half_width <= 25.0
                and 0.2 <= self.side_switch_hysteresis_m <= 2.0
                and 0.5 <= self.side_switch_min_interval <= 3.0
                and 0.3 <= self.speed_reduction_start_frac <= 1.0
                and 0.5 <= self.min_forward_speed <= self.speed):
            raise ValueError("Invalid avoidance configuration")


def tinh_van_toc_an_toan(khoang_cach, bien_an_toan, giam_toc, tre):
    """Giải v*trễ + v²/(2a) <= khoảng cách-biên cho vận tốc không âm."""
    khoang_con_lai = max(0.0, khoang_cach-bien_an_toan)
    return max(0.0, math.sqrt((giam_toc*tre)**2 + 2*giam_toc*khoang_con_lai)
               - giam_toc*tre)


def tinh_khoang_cach_phan_ung(van_toc, cau_hinh):
    """Khoảng nhìn tối thiểu gồm phản ứng, phanh và thời gian preview."""
    van_toc = max(0.0, van_toc)
    return max(cau_hinh.detect_distance,
               cau_hinh.clearance + van_toc*cau_hinh.reaction_time
               + van_toc*van_toc/(2*cau_hinh.brake_accel)
               + van_toc*cau_hinh.preview_time)


# Wrapper tương thích cho test/replay/caller cũ; code mới dùng tên tiếng Việt.
def safe_speed(distance, margin, deceleration, delay):
    return tinh_van_toc_an_toan(distance, margin, deceleration, delay)


def reaction_distance(speed, config):
    return tinh_khoang_cach_phan_ung(speed, config)


@dataclass(frozen=True)
class Motion:
    """Actual motion in the fixed initial course, right positive (metres, seconds)."""
    forward_m_s: float = 0.0
    right_m_s: float = 0.0
    lateral_m: float = 0.0
    yaw_delta_rad: float = 0.0
    forward_m: float = 0.0


class LateralPID:
    """Outer position -> velocity loop; PX4 retains its inner flight controllers."""
    def __init__(self, config):
        self.config = config
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def reset_integral_on_side_switch(self, new_side):
        """Reset integral when switching lateral direction to prevent windup.
        Called when the bypass side changes (left <-> right)."""
        self.integral = 0.0

    def update(self, error, measured_velocity, dt, limit):
        c = self.config
        if not all(math.isfinite(value) for value in (error, measured_velocity, dt, limit)):
            self.reset()
            return 0.0
        dt = clamp(dt, 0.0001, 1.0)
        # Anti-windup: when error reverses sign against accumulated integral, reset immediately
        if self.integral * error < 0:
            self.integral = 0.0
        candidate = clamp(self.integral + error*dt, -c.integral_limit, c.integral_limit)
        # Derivative on the measured position (velocity) avoids derivative kick
        # when a newly discovered obstacle extends the lateral target.
        raw = c.kp*error + c.ki*candidate - c.kd*measured_velocity
        if abs(raw) <= limit or raw*error < 0:
            self.integral = candidate
        return clamp(c.kp*error + c.ki*self.integral - c.kd*measured_velocity, -limit, limit)


@dataclass(frozen=True)
class Decision:
    state: str
    forward_m_s: float
    right_m_s: float
    yaw_deg_s: float
    front_m: float
    nearest_m: float
    reason: str
    trigger_m: float = 10.0
    stop_m: float = 1.3
    target_lateral_m: float = 0.0
    command_lateral_m: float = 0.0
    pid_error_m: float = 0.0
    pid_p_m_s: float = 0.0
    pid_i_m_s: float = 0.0
    pid_d_m_s: float = 0.0


class Planner:
    def __init__(self, config=None):
        self.config = config or AvoidanceConfig()
        self.state = "WAIT_SCAN"
        self.target = None
        self.command_target = None  # Eased setpoint the PID actually chases.
        self.side = 0  # right positive; locked for the complete encounter
        self.pid = LateralPID(self.config)
        self.last_time = None
        self.last_command = (0.0, 0.0)
        self.clear_since = None
        self.recovering = False
        # A stable world-frame signature identifies the wall currently being
        # bypassed.  This makes a newly exposed slalom gate a handoff, rather
        # than an extension of the old wall's lateral interval.
        self.active_obstacle = None
        self.last_side_change = None
        self.course_m = 0.0
        self.reported_forward_m = None
        self.last_valid_scan = None
        # Hysteresis/lock-in for lane selection
        self.locked_side = 0  # 0 = unlocked, -1 = locked left, 1 = locked right
        self.locked_until_clear = False  # True when locked until obstacle clears forward view

    def update(self, scan, now, motion=None):
        c = self.config
        motion = motion or Motion()
        valid_now = math.isfinite(now)
        dt = 0.1 if self.last_time is None or not valid_now else clamp(now-self.last_time, 0.001, 0.2)
        if valid_now:
            self.last_time = now
        valid_motion = all(math.isfinite(v) for v in vars(motion).values())
        if self.target is None:
            self.target = motion.lateral_m if valid_motion else 0.0
        if self.command_target is None:
            self.command_target = self.target
        nominal = c.speed
        measured = math.hypot(motion.forward_m_s, motion.right_m_s) if valid_motion else 0.0
        speed = max(nominal, measured)
        trigger = tinh_khoang_cach_phan_ung(speed, c)
        stopping_distance = c.clearance + measured*c.reaction_time + measured**2/(2*c.brake_accel)
        front, nearest = 0.0, 0.0

        def decision(state, forward=0.0, right=0.0, reason=""):
            forward = forward if math.isfinite(forward) else 0.0
            right = right if math.isfinite(right) else 0.0
            front_value = front if math.isfinite(front) else 0.0
            nearest_value = nearest if math.isfinite(nearest) else 0.0
            self.state = state
            self.last_command = forward, right
            if state in ("WAIT_SCAN", "BLOCKED", "BRAKE"):
                self.pid.reset()
                # Restart the ease-in from where the drone actually is, so a
                # resumed bypass does not begin with a stale setpoint jump.
                if valid_motion:
                    self.command_target = motion.lateral_m
            target_value = self.command_target if self.command_target is not None else 0.0
            error = target_value-motion.lateral_m if valid_motion else 0.0
            return Decision(state, max(0.0, forward), right, 0.0, front_value, nearest_value, reason,
                            trigger, stopping_distance, self.target, target_value, error,
                            c.kp*error, c.ki*self.pid.integral,
                            -c.kd*motion.right_m_s if valid_motion else 0.0)

        if not valid_now or not valid_motion:
            self.recovering, self.clear_since = True, None
            return decision("WAIT_SCAN", reason="Invalid time or motion telemetry")
        # A single dropped packet does not invalidate a still-fresh 360-degree
        # observation.  Hold it only until the normal scan timeout; a missing
        # initial or stale scan still commands a safe wait.
        if scan is None:
            scan = self.last_valid_scan
        scan_hop_le = (scan is not None
                        and math.isfinite(getattr(scan, "received_at", math.nan))
                        and math.isfinite(getattr(scan, "range_max", math.nan))
                        and getattr(scan, "range_max", 0.0) > 0.0
                        and len(getattr(scan, "angles", ())) >= 36
                        and len(getattr(scan, "angles", ())) == len(getattr(scan, "ranges", ()))
                        and all(math.isfinite(goc) for goc in scan.angles)
                        and all(math.isfinite(khoang_cach) and khoang_cach >= 0.0
                                for khoang_cach in scan.ranges))
        if (not scan_hop_le or not 0 <= now - scan.received_at <= c.scan_timeout):
            self.recovering, self.clear_since = True, None
            return decision("WAIT_SCAN", reason="LiDAR thiếu, quá cũ hoặc không hữu hạn")
        if scan is not self.last_valid_scan:
            self.last_valid_scan = scan
        valid_ranges = [value for value in scan.ranges if math.isfinite(value) and value >= 0]
        if not valid_ranges:
            valid_ranges = [scan.range_max if math.isfinite(scan.range_max) else 0.0]
        nearest = min(valid_ranges)
        if nearest < c.emergency_distance:
            self.clear_since = None
            return decision("BLOCKED", reason="Obstacle inside emergency clearance")
        if self.recovering:
            if self.clear_since is None:
                self.clear_since = scan.received_at
            if scan.received_at-self.clear_since < c.clear_seconds:
                return decision("WAIT_SCAN", reason="Confirm fresh scans before resuming")
            self.recovering, self.clear_since = False, None

        # Prefer externally measured progress when it changes.  The fallback
        # supports callers that only provide velocity (the original API).
        if self.reported_forward_m is None:
            self.course_m = motion.forward_m
        elif abs(motion.forward_m-self.reported_forward_m) > 1e-5:
            self.course_m = motion.forward_m
        else:
            self.course_m += max(0.0, motion.forward_m_s)*dt
        self.reported_forward_m = motion.forward_m

        # Rotate FLU scan bearings into the fixed course FRD plane. Max-range
        # returns are free space to the sensor horizon, not a ring of obstacles.
        range_max = scan.range_max if math.isfinite(scan.range_max) and scan.range_max > 0 else c.detect_distance
        points = [(r*math.cos(a-motion.yaw_delta_rad), -r*math.sin(a-motion.yaw_delta_rad))
                  for a, r in zip(scan.angles, scan.ranges)
                  if math.isfinite(a) and math.isfinite(r) and 0 < r < range_max-0.01]
        horizon = max(0.0, range_max-c.clearance)
        nominal = min(nominal, tinh_van_toc_an_toan(horizon, c.clearance, c.brake_accel,
                                         c.reaction_time+c.preview_time))
        trigger = min(trigger, horizon)

        def corridor_distance(vx, vy):
            magnitude = math.hypot(vx, vy)
            if magnitude < 1e-6:
                return horizon
            nx, ny = vx/magnitude, vy/magnitude
            return min((x*nx+y*ny for x, y in points
                        if x*nx+y*ny >= 0 and abs(-x*ny+y*nx) < c.clearance), default=horizon)

        front = corridor_distance(1, 0)
        # A vertical face produces many returns with almost identical x.  Group
        # those returns before inflating them: merging all lateral intervals at
        # once incorrectly fuses two staggered gates into one giant wall.
        x_groups = []
        for point in sorted((point for point in points if 0 < point[0] <= trigger),
                            key=lambda point: point[0]):
            if (not x_groups
                    # Compare to the first return in the face, not the last:
                    # otherwise a harmless constant-range sensor horizon forms
                    # a chain of tiny x steps and becomes a fictional wall.
                    or point[0]-x_groups[-1][0][0] > max(0.8, c.clearance*0.6)):
                x_groups.append([point])
            else:
                x_groups[-1].append(point)

        # A constant-range horizon has two disconnected y arcs in each x bin.
        # Split such gaps so its endpoints cannot masquerade as a solid face.
        face_groups = []
        for group in x_groups:
            y_groups = []
            for point in sorted(group, key=lambda point: point[1]):
                if (not y_groups
                        or point[1]-y_groups[-1][-1][1] > max(0.75, c.clearance*0.55)):
                    y_groups.append([point])
                else:
                    y_groups[-1].append(point)
            face_groups.extend(y_groups)

        obstacles = []
        for group in face_groups:
            xs, ys = zip(*group)
            lo, hi = min(ys)-c.clearance, max(ys)+c.clearance
            distance = min(xs)
            obstacles.append({
                "lo": lo, "hi": hi, "distance": distance,
                # World x is invariant while a face approaches; y is included
                # only to make the signature useful for adjacent parallel walls.
                "signature": (self.course_m + sum(xs)/len(xs),
                              motion.lateral_m + (min(ys)+max(ys))/2),
            })
        blocking = [item for item in obstacles if item["lo"] <= 0 <= item["hi"]]
        obstruction = min(blocking, key=lambda item: item["distance"], default=None)

        def same_obstacle(item):
            if item is None or self.active_obstacle is None:
                return False
            old_x, old_y = self.active_obstacle
            new_x, new_y = item["signature"]
            return abs(new_x-old_x) <= 1.4 and abs(new_y-old_y) <= 4.5

        state, reason = "CRUISE", "Hold offset and original heading"
        if obstruction:
            self.clear_since = None
            lo, hi = obstruction["lo"], obstruction["hi"]
            candidates = ((-1, lo-c.offset_margin), (1, hi+c.offset_margin))
            new_face = not same_obstacle(obstruction)
            if self.side == 0:
                feasible = [(side, shift) for side, shift in candidates
                            if abs(shift) < horizon and corridor_distance(0, side) > abs(shift)+c.clearance]
                if not feasible:
                    return decision("BLOCKED", reason="Neither lateral exit is clear")
                self.side, shift = min(feasible, key=lambda item: abs(item[1]))
                # Apply corridor clamp: limit target to arena bounds (±corridor_half_width)
                raw_target = motion.lateral_m + shift
                self.target = clamp(raw_target, -c.corridor_half_width, c.corridor_half_width)
                self.pid.reset()
                self.active_obstacle = obstruction["signature"]
                self.last_side_change = now
                # Lock the chosen side until obstacle clears forward view (hysteresis/lock-in)
                self.locked_side = self.side
                self.locked_until_clear = True
            elif new_face:
                # A queued gate may become visible while the old gate is still
                # physically ahead.  Do not cut back through that old face;
                # remember the handoff opportunity and keep sliding alongside it
                # until its longitudinal plane has been passed.
                active_ahead = any(same_obstacle(item) and item["distance"] > 0.05
                                   for item in obstacles)
                if active_ahead:
                    state = "SLIDE_LEFT" if self.side < 0 else "SLIDE_RIGHT"
                    reason = "Hold current side until its face is passed"
                    # A long or oblique physical wall can split into multiple
                    # point-cloud groups at almost the same world X. Continue
                    # expanding around that surface; do not do this for a true
                    # next gate several metres ahead, which may need reversal.
                    same_longitudinal_surface = (
                        abs(obstruction["signature"][0]-self.active_obstacle[0]) <= 3.0)
                    if same_longitudinal_surface:
                        current_shift = (obstruction["lo"]-c.offset_margin if self.side < 0
                                         else obstruction["hi"]+c.offset_margin)
                        proposed = motion.lateral_m+current_shift
                        proposed = clamp(proposed, -c.corridor_half_width, c.corridor_half_width)
                        self.target = (min(self.target, proposed) if self.side < 0
                                       else max(self.target, proposed))
                    # The remaining command calculation still sees the new
                    # face below and can reduce forward speed for a later reversal.
                else:
                    # Hand off to a genuinely new face.  Keep the previous choice
                    # unless the alternative saves a material lateral displacement;
                    # that hysteresis removes noisy left/right dithering.
                    # Release the lane lock for a new face to allow side switching.
                    self.locked_until_clear = False
                    current_shift = lo-c.offset_margin if self.side < 0 else hi+c.offset_margin
                    alternate_side = -self.side
                    alternate_shift = lo-c.offset_margin if alternate_side < 0 else hi+c.offset_margin
                    improvement = abs(current_shift)-abs(alternate_shift)
                    can_switch = (improvement >= c.side_switch_hysteresis_m
                                  and (self.last_side_change is None
                                       or now-self.last_side_change >= c.side_switch_min_interval
                                       or obstruction["distance"] <= c.emergency_distance+c.clearance))
                    if can_switch:
                        self.side = alternate_side
                        raw_target = motion.lateral_m + alternate_shift
                        self.target = clamp(raw_target, -c.corridor_half_width, c.corridor_half_width)
                        self.pid.reset_integral_on_side_switch(alternate_side)  # Anti-windup on side switch
                        self.last_side_change = now
                        self.locked_side = self.side
                        self.locked_until_clear = True
                    else:
                        proposed = motion.lateral_m + current_shift
                        proposed = clamp(proposed, -c.corridor_half_width, c.corridor_half_width)
                        self.target = proposed
                    self.active_obstacle = obstruction["signature"]
            else:
                shift = lo-c.offset_margin if self.side < 0 else hi+c.offset_margin
                proposed = motion.lateral_m + shift
                proposed = clamp(proposed, -c.corridor_half_width, c.corridor_half_width)
                # Never chase the wall's shrinking edge inward mid-bypass.
                self.target = min(self.target, proposed) if self.side < 0 else max(self.target, proposed)
            state = "SLIDE_LEFT" if self.side < 0 else "SLIDE_RIGHT"
            reason = "Forward + lateral PID; heading fixed"
        elif self.side:
            state, reason = "CLEARING", "Hold offset until the obstacle leaves the forward view"
            # Corridor-clear only means the drone's own lane is free; the wall is
            # still alongside. Keep the bypass locked until nothing remains in the
            # forward view cone, then resume the original course at the new offset.
            half_fov = math.radians(c.clear_fov_deg)/2
            if any(x > 0 and math.hypot(x, y) <= trigger
                   and abs(math.atan2(y, x)) <= half_fov for x, y in points):
                self.clear_since = None
            else:
                if self.clear_since is None:
                    self.clear_since = scan.received_at
                if (scan.received_at-self.clear_since >= c.clear_seconds
                        and abs(self.target-motion.lateral_m) < 0.35 and abs(motion.right_m_s) < 0.3):
                    self.side, self.clear_since = 0, None
                    self.pid.reset()
                    # Release the lane lock when obstacle fully clears
                    self.locked_side = 0
                    self.locked_until_clear = False
                    if c.return_to_course:
                        self.target = 0.0
                    state, reason = "CRUISE", "Obstacle out of view; continue original heading at new offset"

        lateral_limit = min(c.max_lateral_speed, nominal*0.8)
        remaining = self.target-motion.lateral_m
        if obstruction:
            # "Nhich dan": ease the setpoint toward the full offset at the
            # slowest rate that still finishes the bypass before the wall, so the
            # PID works inside its linear range instead of sitting on the limit.
            # Faster flight leaves less time, so the same formula sidesteps
            # harder without retuning any gain.
            travel_time = max(0.0, front-c.clearance)/max(speed, c.min_ramp_speed)
            rate = (lateral_limit if travel_time <= 1e-3 else
                    clamp(abs(remaining)*c.approach_margin/travel_time, 0.0, lateral_limit))
            step = rate*dt
            self.command_target += clamp(self.target-self.command_target, -step, step)
        else:
            # No wall ahead means no time pressure, but the setpoint must still
            # slew. A step here happens whenever the corridor clears mid-bypass,
            # and it would saturate the PID exactly like the original bug.
            step = lateral_limit*dt
            self.command_target += clamp(self.target-self.command_target, -step, step)
        error = self.command_target-motion.lateral_m
        integral_before = self.pid.integral
        vy = self.pid.update(error, motion.right_m_s, dt, lateral_limit)
        pid_requested = vy
        vx = math.sqrt(max(0.0, nominal**2-vy**2))
        
        # Dynamic forward speed reduction (Speed Envelope):
        # When obstacle within < 8.0m (or obstruction detected), reduce forward speed
        # linearly from nominal down to min_forward_speed (1.1 m/s) to ensure safe lateral maneuver.
        speed_threshold = min(8.0, c.detect_distance * c.speed_reduction_start_frac)
        if (obstruction or state in ("SLIDE_LEFT", "SLIDE_RIGHT")) and front < speed_threshold:
            # Linear reduction: at speed_threshold -> nominal, at clearance -> min_forward_speed
            speed_factor = clamp((front - c.clearance) / max(0.1, speed_threshold - c.clearance), 0.0, 1.0)
            reduced_speed = c.min_forward_speed + (nominal - c.min_forward_speed) * speed_factor
            vx = min(vx, reduced_speed)
            nominal = min(nominal, reduced_speed)
        elif state == "CLEARING":
            # Obstacle has left the direct collision path but is still alongside within clear_fov_deg.
            # Hold reduced forward speed until fully transitioned to CRUISE.
            reduced_speed = max(c.min_forward_speed, min(nominal, c.min_forward_speed + 0.3))
            vx = min(vx, reduced_speed)
            nominal = min(nominal, reduced_speed)
        
        delay = c.reaction_time + max(0.0, now-scan.received_at)
        vx = min(vx, tinh_van_toc_an_toan(front, c.clearance, c.brake_accel, delay))
        if obstruction:
            # Budget the whole remaining offset, not the eased setpoint, and
            # assume the tracker lags the ramp, so slowing down stays conservative.
            shift_time = (abs(remaining)/max(c.tracking_fraction*lateral_limit, 0.05)
                          + lateral_limit/c.max_accel)
            vx = min(vx, max(0.0, front-c.clearance)/(shift_time+delay))
            # Reserve time for the first reversal after this face as well.  It
            # uses geometry from the next observed face, so speed scaling falls
            # out of the same kinematics instead of per-speed tuning tables.
            future = min((item for item in obstacles
                          if item["distance"] > obstruction["distance"] + 0.8),
                         key=lambda item: item["distance"], default=None)
            if future is not None:
                future_candidates = ((-1, future["lo"]-c.offset_margin),
                                     (1, future["hi"]+c.offset_margin))
                future_side, future_shift = min(future_candidates,
                                                key=lambda item: abs(item[1]))
                if future_side != self.side:
                    reversal_m = abs((motion.lateral_m+future_shift)-self.target)
                    reversal_s = (reversal_m/max(c.tracking_fraction*lateral_limit, 0.05)
                                  + lateral_limit/c.max_accel)
                    separation = max(0.05, future["distance"]-obstruction["distance"])
                    vx = min(vx, separation/(reversal_s+delay))
        if abs(vy) > 1e-6:
            vy = math.copysign(min(abs(vy), tinh_van_toc_an_toan(corridor_distance(0, vy), c.clearance,
                                                     c.brake_accel, delay)), vy)

        # Slew routine velocity changes; freshness/emergency stops bypass slew.
        old_x, old_y = self.last_command
        delta = math.hypot(vx-old_x, vy-old_y)
        if delta > c.max_accel*dt:
            scale = c.max_accel*dt/delta
            vx, vy = old_x+(vx-old_x)*scale, old_y+(vy-old_y)*scale

        # A diagonal can intersect a corner even when cardinal rays are clear.
        # Project velocity onto braking half-planes of nearby obstacle surfaces.
        # Keeping tangential velocity allows sideways escape instead of deadlock.
        for _ in range(3):
            for x, y in points:
                distance = math.hypot(x, y)
                if distance < 1e-6:
                    continue
                nx, ny = x/distance, y/distance
                allowed = tinh_van_toc_an_toan(distance, c.clearance, c.brake_accel, delay)
                closing = vx*nx+vy*ny
                if closing > allowed:
                    vx, vy = vx-(closing-allowed)*nx, vy-(closing-allowed)*ny
        # Command corridor and measured momentum are independent checks.
        actual_distance = corridor_distance(motion.forward_m_s, motion.right_m_s)
        if measured > tinh_van_toc_an_toan(actual_distance, c.emergency_distance, c.brake_accel, delay)+0.25:
            return decision("BRAKE", reason="Measured momentum exceeds braking envelope")
        # No backwards commands; if projection made one, recheck pure strafe.
        if vx < 0:
            vx = 0.0
            vy = math.copysign(min(abs(vy), tinh_van_toc_an_toan(corridor_distance(0, vy), c.clearance,
                                                     c.brake_accel, delay)), vy)
        vy = clamp(vy, -lateral_limit, lateral_limit)
        # Clipping components changes direction; check the final corridor again.
        command_speed = math.hypot(vx, vy)
        allowed = min(nominal, tinh_van_toc_an_toan(corridor_distance(vx, vy), c.clearance, c.brake_accel, delay))
        if command_speed > allowed and command_speed > 0:
            vx, vy = vx*allowed/command_speed, vy*allowed/command_speed
        if abs(vy-pid_requested) > 1e-5 and (pid_requested-vy)*error > 0:
            self.pid.integral = integral_before  # Don't wind up behind safety limits.
        if math.hypot(vx, vy) < 0.02 and obstruction:
            return decision("BLOCKED", reason="Motion corridor blocked; hold position")
        return decision(state, vx, vy, reason)

