"""Sensor contracts and geometric regressions for continuous lateral avoidance."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from obstacle_avoidance import (
    AvoidanceConfig, LateralPID, Motion, Planner, Scan, reaction_distance, safe_speed,
    tinh_khoang_cach_phan_ung, tinh_van_toc_an_toan,
)


def scan_message(distance=12.0, **overrides):
    """Generate a complete Gazebo scan; positive bearing points left."""
    fields = dict(
        count=361, vertical_count=1, angle_min=-math.pi, angle_max=math.pi,
        angle_step=math.pi / 180, range_min=0.1, range_max=20.0,
        ranges=[distance(deg) if callable(distance) else distance
                for deg in range(-180, 181)],
    )
    fields.update(overrides)
    return SimpleNamespace(**fields)


def scan_at(now, distance=20.0):
    """Tạo scan hợp lệ; tia không va chạm được biểu diễn bằng range_max hữu hạn."""
    return Scan.from_message(scan_message(distance), received_at=now)


def box_scan(now, boxes=(), x=0.0, y=0.0, yaw=0.0):
    """Raycast boxes in a fixed course frame: x forward, y right.

    A box is (xmin, xmax, ymin, ymax). Rays start at the vehicle's measured
    position and rotate with its actual yaw, so translation changes the
    observed wall edges instead of moving a fabricated constant-range ring.
    """
    ranges = []
    for degree in range(-180, 181):
        ray = yaw - math.radians(degree)
        dx, dy = math.cos(ray), math.sin(ray)
        nearest = math.inf
        for xmin, xmax, ymin, ymax in boxes:
            enter, leave = -math.inf, math.inf
            for origin, direction, low, high in (
                (x, dx, xmin, xmax), (y, dy, ymin, ymax),
            ):
                if abs(direction) < 1e-10:
                    if not low <= origin <= high:
                        enter, leave = math.inf, -math.inf
                        break
                else:
                    first, second = (low-origin)/direction, (high-origin)/direction
                    enter = max(enter, min(first, second))
                    leave = min(leave, max(first, second))
            if leave >= max(enter, 0.0):
                nearest = min(nearest, max(0.0, enter))
        ranges.append(min(nearest, 20.0))
    return Scan.from_message(scan_message(ranges=ranges), received_at=now)


def point_face_scan(now, faces):
    """Scan 360° hữu hạn có thêm các điểm mặt: (x, y-thấp, y-cao)."""
    points = [(x, low+index*0.1) for x, low, high in faces
              for index in range(round((high-low)/0.1)+1)]
    goc_nen = tuple(math.radians(degree) for degree in range(-180, 181))
    khoang_cach_nen = (20.0,) * len(goc_nen)
    return Scan(goc_nen + tuple(-math.atan2(y, x) for x, y in points),
                khoang_cach_nen + tuple(math.hypot(x, y) for x, y in points), now, 20.0)


class ScanValidationTests(unittest.TestCase):
    def test_complete_scan_preserves_order_and_receipt_time(self):
        scan = scan_at(42.5, 12.0)
        self.assertEqual(len(scan.angles), 361)
        self.assertEqual(len(scan.ranges), 361)
        self.assertAlmostEqual(scan.angles[0], -math.pi)
        self.assertAlmostEqual(scan.angles[180], 0.0)
        self.assertAlmostEqual(scan.angles[-1], math.pi)
        self.assertEqual(scan.received_at, 42.5)

    def test_range_max_huu_han_means_clear_to_sensor_limit(self):
        scan = scan_at(0.0)
        self.assertTrue(all(value == 20.0 for value in scan.ranges))
        self.assertEqual(scan.range_max, 20.0)

    def test_nan_or_infinity_rejects_entire_scan(self):
        for corrupt in (math.nan, math.inf, -math.inf):
            with self.subTest(value=corrupt):
                with self.assertRaises(ValueError):
                    Scan.from_message(scan_message(corrupt), received_at=0.0)

    def test_gazebo_adapter_only_normalizes_positive_infinite_no_hit(self):
        scan = Scan.from_message(
            scan_message(math.inf), received_at=0.0, cho_phep_inf_khong_va_cham=True)
        self.assertTrue(all(value == 20.0 for value in scan.ranges))

        for corrupt in (math.nan, -math.inf):
            with self.subTest(value=corrupt):
                with self.assertRaises(ValueError):
                    Scan.from_message(
                        scan_message(corrupt), received_at=0.0,
                        cho_phep_inf_khong_va_cham=True)

    def test_below_minimum_range_is_conservative_not_free_space(self):
        for invalid_near_range in (0.0, 0.05):
            with self.subTest(distance=invalid_near_range):
                self.assertTrue(all(value == 0.0
                                    for value in scan_at(0.0, invalid_near_range).ranges))

    def test_corrupt_range_anywhere_rejects_entire_scan(self):
        for corrupt in (math.nan, math.inf, -math.inf, -1.0):
            for index in (0, 180, 270):
                with self.subTest(value=corrupt, index=index):
                    msg = scan_message()
                    msg.ranges[index] = corrupt
                    with self.assertRaises(ValueError):
                        Scan.from_message(msg, received_at=0.0)

    def test_missing_or_extra_rays_rejected(self):
        for count in (0, 180, 360, 362):
            with self.subTest(ranges_length=count), self.assertRaises(ValueError):
                Scan.from_message(scan_message(ranges=[12.0] * count), 0.0)

    def test_partial_view_rejected(self):
        msg = scan_message(count=181, angle_min=-math.pi/2,
                           angle_max=math.pi/2, ranges=[12.0] * 181)
        with self.assertRaises(ValueError):
            Scan.from_message(msg, received_at=0.0)

    def test_invalid_geometry_and_sensor_limits_are_rejected(self):
        cases = (
            {"count": 1, "ranges": [12.0]}, {"vertical_count": 2},
            {"angle_step": 0.0}, {"angle_step": -math.pi/180},
            {"angle_step": math.pi/90}, {"angle_min": math.nan},
            {"angle_max": math.inf}, {"range_min": -1.0},
            {"range_max": 0.0}, {"range_max": math.inf},
        )
        for fields in cases:
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                Scan.from_message(scan_message(**fields), received_at=0.0)


class SpeedEnvelopeTests(unittest.TestCase):
    def test_safe_speed_fits_delay_and_braking_distance(self):
        for distance in (1.3, 2.0, 5.0, 10.0, 20.0):
            with self.subTest(distance=distance):
                speed = tinh_van_toc_an_toan(distance, 1.3, 0.8, 1.2)
                used = 1.3 + speed*1.2 + speed*speed/(2*0.8)
                self.assertGreaterEqual(speed, 0.0)
                self.assertLessEqual(used, distance + 1e-8)

    def test_safe_speed_is_zero_inside_required_margin(self):
        self.assertEqual(tinh_van_toc_an_toan(0.8, 1.3, 1.0, 1.0), 0.0)

    def test_alias_tuong_thich_giu_nguyen_keyword_cu(self):
        self.assertEqual(
            safe_speed(5.0, margin=1.3, deceleration=0.8, delay=1.2),
            tinh_van_toc_an_toan(5.0, 1.3, 0.8, 1.2),
        )

    def test_reaction_distance_increases_with_measured_speed(self):
        config = AvoidanceConfig()
        low = tinh_khoang_cach_phan_ung(1.0, config)
        high = tinh_khoang_cach_phan_ung(4.0, config)
        self.assertGreaterEqual(low, config.detect_distance)
        self.assertGreater(high, low)
        expected = (config.clearance + 4.0*config.reaction_time
                    + 4.0**2/(2*config.brake_accel) + 4.0*config.preview_time)
        self.assertAlmostEqual(high, max(config.detect_distance, expected))


class PlannerSafetyTests(unittest.TestCase):
    def setUp(self):
        self.config = AvoidanceConfig()
        self.planner = Planner(self.config)

    def assert_stopped(self, decision):
        self.assertEqual(decision.forward_m_s, 0.0)
        self.assertEqual(decision.right_m_s, 0.0)
        self.assertEqual(decision.yaw_deg_s, 0.0)
        self.assertTrue(decision.reason)

    def test_missing_expired_and_future_scan_stop(self):
        self.assert_stopped(self.planner.update(None, now=0.0))
        for timestamp in (0.0, 2.0):
            with self.subTest(timestamp=timestamp):
                decision = self.planner.update(scan_at(timestamp), now=1.0)
                self.assertEqual(decision.state, "WAIT_SCAN")
                self.assert_stopped(decision)

    def test_no_hit_at_range_limit_is_not_a_phantom_wall(self):
        decision = self.planner.update(scan_at(0.0), 0.0)
        self.assertEqual(decision.state, "CRUISE")
        self.assertGreater(decision.forward_m_s, 0.0)
        self.assertLessEqual(decision.forward_m_s, self.config.speed)
        self.assertEqual(decision.right_m_s, 0.0)
        self.assertEqual(decision.yaw_deg_s, 0.0)

    def test_normal_wall_encounter_moves_forward_and_sideways_without_yaw(self):
        decision = self.planner.update(box_scan(0.0, [(8.5, 9.0, -4.0, 4.0)]), 0.0)
        self.assertIn(decision.state, ("SLIDE_LEFT", "SLIDE_RIGHT"))
        self.assertGreater(decision.forward_m_s, 0.0)
        self.assertNotEqual(decision.right_m_s, 0.0)
        self.assertEqual(decision.yaw_deg_s, 0.0)
        self.assertGreaterEqual(decision.trigger_m, 10.0)

    def test_asymmetric_wall_selects_shorter_clear_side(self):
        for y_min, y_max, expected_sign in ((-1.0, 6.0, -1), (-6.0, 1.0, 1)):
            with self.subTest(extent=(y_min, y_max)):
                planner = Planner(self.config)
                decision = planner.update(box_scan(0.0, [(8.5, 9.0, y_min, y_max)]), 0.0)
                self.assertGreater(decision.right_m_s * expected_sign, 0.0)
                self.assertGreater(decision.target_lateral_m * expected_sign, 0.0)
                self.assertEqual(decision.yaw_deg_s, 0.0)

    def test_split_oblique_face_expands_target_instead_of_freezing_at_clearance(self):
        first = self.planner.update(point_face_scan(0.0, [(5.0, -2.0, 0.0)]), 0.0)
        self.assertEqual(first.state, "SLIDE_RIGHT")
        old_target = first.target_lateral_m
        # Gazebo can split one oblique wall into a near group plus a still-visible
        # group matching the active face. The newly blocking group must extend
        # the current-side target even before a later gate handoff is allowed.
        second = self.planner.update(
            point_face_scan(0.1, [(2.5, -1.0, 1.0), (5.0, -2.0, 0.0)]),
            0.1, Motion(),
        )
        self.assertEqual(second.state, "SLIDE_RIGHT")
        self.assertGreater(second.target_lateral_m, old_target+0.5)

    def test_far_wall_does_not_trigger_early_at_default_speed(self):
        decision = self.planner.update(box_scan(0.0, [(15.0, 15.5, -4.0, 4.0)]), 0.0)
        self.assertEqual(decision.state, "CRUISE")
        self.assertEqual(decision.right_m_s, 0.0)

    def test_measured_speed_extends_detection_before_requested_speed_catches_up(self):
        scan = box_scan(0.0, [(15.0, 15.5, -4.0, 4.0)])
        decision = self.planner.update(scan, 0.0, Motion(forward_m_s=4.0))
        self.assertGreater(decision.trigger_m, 15.0)
        self.assertIn(decision.state, ("SLIDE_LEFT", "SLIDE_RIGHT", "BRAKE"))

    def test_excess_measured_momentum_requests_emergency_braking(self):
        decision = self.planner.update(box_scan(0.0, [(8.5, 9.0, -4.0, 4.0)]),
                                       0.0, Motion(forward_m_s=5.0))
        self.assertEqual(decision.state, "BRAKE")
        self.assert_stopped(decision)

    def test_actual_yaw_drift_does_not_change_world_bypass_side(self):
        for yaw in (-0.4, 0.4, math.pi):
            with self.subTest(yaw=yaw):
                planner = Planner(self.config)
                scan = box_scan(0.0, [(8.5, 9.0, -1.0, 6.0)], yaw=yaw)
                decision = planner.update(scan, 0.0, Motion(yaw_delta_rad=yaw))
                self.assertEqual(decision.state, "SLIDE_LEFT")
                self.assertGreater(decision.forward_m_s, 0.0)
                self.assertLess(decision.right_m_s, 0.0)
                self.assertEqual(decision.yaw_deg_s, 0.0)

    def test_short_sensor_horizon_caps_cruise_speed(self):
        planner = Planner(AvoidanceConfig(speed=3.0))
        for index in range(80):
            now = index * 0.05
            scan = Scan.from_message(scan_message(5.0, range_max=5.0), now)
            decision = planner.update(scan, now)
        used_distance = (2*self.config.clearance
                         + decision.forward_m_s*(self.config.reaction_time+self.config.preview_time)
                         + decision.forward_m_s**2/(2*self.config.brake_accel))
        self.assertEqual(decision.state, "CRUISE")
        self.assertGreater(decision.forward_m_s, 0.0)
        self.assertLessEqual(used_distance, 5.0+1e-8)

    def test_stale_scan_stops_active_slide_and_preserves_chosen_side(self):
        wall = [(8.5, 9.0, -4.0, 4.0)]
        first = self.planner.update(box_scan(0.0, wall), 0.0)
        stopped = self.planner.update(box_scan(0.0, wall), 1.0)
        self.assertEqual(stopped.state, "WAIT_SCAN")
        self.assert_stopped(stopped)
        recovered = self.planner.update(box_scan(1.1, wall), 1.1)
        self.assertEqual(recovered.state, "WAIT_SCAN")
        self.assert_stopped(recovered)
        now = 1.1 + self.config.clear_seconds + 0.01
        recovered = self.planner.update(box_scan(now, wall), now)
        self.assertGreater(recovered.right_m_s * first.right_m_s, 0.0)

    def test_direct_nonfinite_scan_stops_without_command(self):
        scan = Scan((0.0,) * 36, (math.inf,) * 36, 0.0, 20.0)
        decision = self.planner.update(scan, 0.0)
        self.assertEqual(decision.state, "WAIT_SCAN")
        self.assert_stopped(decision)

    def test_nonfinite_motion_never_produces_nonfinite_commands(self):
        for field in ("forward_m_s", "right_m_s", "lateral_m", "yaw_delta_rad"):
            with self.subTest(field=field):
                planner = Planner(self.config)
                decision = planner.update(scan_at(0.0), 0.0, Motion(**{field: math.nan}))
                self.assert_stopped(decision)

    def test_emergency_return_anywhere_including_rear_stops(self):
        for bearing in (0, 90, -90, 180):
            with self.subTest(bearing=bearing):
                planner = Planner(self.config)
                planner.update(scan_at(0.0), 0.0)
                scan = scan_at(0.1, lambda deg: 0.4 if deg == bearing else 20.0)
                decision = planner.update(scan, 0.1)
                self.assertIn(decision.state, ("BLOCKED", "BRAKE"))
                self.assert_stopped(decision)

    def test_clear_space_acceleration_is_bounded_in_vector_norm(self):
        previous = None
        for index in range(80):
            now = index * 0.05
            decision = self.planner.update(scan_at(now), now)
            self.assertLessEqual(math.hypot(decision.forward_m_s, decision.right_m_s),
                                 self.config.speed + 1e-8)
            if previous:
                delta = math.hypot(decision.forward_m_s-previous.forward_m_s,
                                   decision.right_m_s-previous.right_m_s)
                self.assertLessEqual(delta, self.config.max_accel*0.05 + 1e-8)
            previous = decision

    def test_clearance_confirmation_needs_new_measurements(self):
        encounter = self.planner.update(box_scan(0.0, [(8.5, 9.0, -1.0, 1.0)]), 0.0)
        motion = Motion(lateral_m=encounter.target_lateral_m)
        clear_scan = scan_at(0.1)
        first = self.planner.update(clear_scan, 0.1, motion)
        self.assertEqual(first.state, "CLEARING")
        repeated = self.planner.update(clear_scan, 0.1+self.config.clear_seconds+0.01, motion)
        self.assertNotEqual(repeated.state, "CRUISE")
        now = 0.1+self.config.clear_seconds+0.02
        fresh = self.planner.update(scan_at(now), now, motion)
        self.assertEqual(fresh.state, "CRUISE")

    def test_raycast_wall_rollout_passes_without_turning_then_returns_to_course(self):
        box = (8.5, 9.0, -4.0, 4.0)
        x = y = vx = vy = 0.0
        selected_sign = None
        smallest_gap = math.inf
        passed_at_y = None
        for index in range(600):
            now = index * 0.05
            motion = Motion(forward_m_s=vx, right_m_s=vy, lateral_m=y)
            decision = self.planner.update(box_scan(now, [box], x, y), now, motion)
            self.assertEqual(decision.yaw_deg_s, 0.0)
            if decision.state.startswith("SLIDE_"):
                sign = -1 if decision.state.endswith("LEFT") else 1
                if selected_sign is None:
                    selected_sign = sign
                self.assertEqual(sign, selected_sign, "Direction changed during one wall encounter")
            vx, vy = decision.forward_m_s, decision.right_m_s
            x, y = x + vx*0.05, y + vy*0.05
            gap = math.hypot(max(box[0]-x, 0.0, x-box[1]),
                             max(box[2]-y, 0.0, y-box[3]))
            smallest_gap = min(smallest_gap, gap)
            if x > box[1]+1.0 and passed_at_y is None:
                passed_at_y = y
        self.assertIsNotNone(passed_at_y, "Did not pass wall within 30 seconds")
        self.assertGreater(smallest_gap, self.config.emergency_distance)
        self.assertGreater(x, 12.0)
        # Recovery is enabled by default: only after the wall is passed and the
        # forward view has been confirmed clear may the fixed-heading planner
        # ease laterally back to the original course.
        self.assertLess(abs(y), 0.35, "Did not return to the original course")


class LateralPIDTests(unittest.TestCase):
    def test_measured_lateral_velocity_damps_motion_toward_target(self):
        stationary = LateralPID(AvoidanceConfig()).update(1.0, 0.0, 0.1, 1.2)
        moving = LateralPID(AvoidanceConfig()).update(1.0, 1.0, 0.1, 1.2)
        self.assertGreater(stationary, moving)
        self.assertGreater(moving, 0.0)

    def test_prolonged_saturation_does_not_leave_integral_drift(self):
        pid = LateralPID(AvoidanceConfig())
        for _ in range(1000):
            self.assertEqual(pid.update(100.0, 0.0, 0.1, 1.2), 1.2)
        self.assertEqual(pid.update(0.0, 0.0, 0.1, 1.2), 0.0)


class GradualBypassTests(unittest.TestCase):
    """Fly and ease sideways at the same time; never turn, never step the setpoint."""

    def setUp(self):
        self.config = AvoidanceConfig()
        self.wall = (8.5, 9.0, -4.0, 4.0)

    def rollout(self, config=None, steps=600, dt=0.05):
        planner = Planner(config or self.config)
        x = y = vx = vy = 0.0
        samples = []
        for index in range(steps):
            now = index*dt
            motion = Motion(forward_m_s=vx, right_m_s=vy, lateral_m=y)
            decision = planner.update(box_scan(now, [self.wall], x, y), now, motion)
            samples.append((x, y, decision))
            vx, vy = decision.forward_m_s, decision.right_m_s
            x, y = x + vx*dt, y + vy*dt
        return samples

    def first_slide(self, config=None):
        for _, _, decision in self.rollout(config, steps=40):
            if decision.state.startswith("SLIDE_"):
                return decision
        self.fail("The wall never started a sideways bypass")

    def test_setpoint_eases_in_instead_of_stepping_to_the_full_offset(self):
        first = self.first_slide()
        self.assertGreater(abs(first.target_lateral_m), 4.0)
        self.assertLess(abs(first.command_lateral_m), abs(first.target_lateral_m)/4)

    def test_eased_setpoint_only_ever_advances_toward_the_bypass_side(self):
        eased = [abs(d.command_lateral_m) for _, _, d in self.rollout(steps=60)
                 if d.state.startswith("SLIDE_")]
        self.assertTrue(all(b >= a-1e-9 for a, b in zip(eased, eased[1:])))
        self.assertGreater(eased[-1], eased[0])

    def test_lateral_speed_builds_up_instead_of_jumping_to_the_limit(self):
        limit = min(self.config.max_lateral_speed, self.config.speed*0.8)
        speeds = [abs(d.right_m_s) for _, _, d in self.rollout(steps=20)]
        self.assertGreater(max(speeds), 0.0)
        self.assertLess(max(speeds), limit)

    def test_faster_flight_starts_earlier_and_eases_across_harder(self):
        slow = self.first_slide(AvoidanceConfig(speed=1.0))
        fast = self.first_slide(AvoidanceConfig(speed=2.5))
        self.assertGreater(fast.trigger_m, slow.trigger_m)
        self.assertGreater(abs(fast.command_lateral_m), abs(slow.command_lateral_m))

    def test_bypass_is_held_until_the_obstacle_leaves_the_forward_view(self):
        corridor_clear_at = resumed_at = None
        for x, _, decision in self.rollout():
            if decision.state == "CLEARING" and corridor_clear_at is None:
                corridor_clear_at = x
            if corridor_clear_at is not None and decision.state == "CRUISE":
                resumed_at = x
                break
        self.assertIsNotNone(corridor_clear_at, "The sideways bypass never completed")
        self.assertIsNotNone(resumed_at, "The original course was never resumed")
        # Its own lane clears early, but the wall is still alongside and visible.
        self.assertGreater(resumed_at, self.wall[1])
        self.assertGreater(resumed_at-corridor_clear_at, 1.0)

    def test_no_yaw_is_commanded_anywhere_in_the_encounter(self):
        self.assertTrue(all(d.yaw_deg_s == 0.0 for _, _, d in self.rollout()))


if __name__ == "__main__":
    unittest.main()
