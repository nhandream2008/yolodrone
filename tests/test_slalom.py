"""Offline ray-cast regression tests for fixed-heading slalom avoidance."""

import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from obstacle_avoidance import AvoidanceConfig, LateralPID, Motion, Planner, Scan
from offline_slalom import (
    OfflineDynamics, SlalomCourse, config_for, lateral_at_forward, simulate,
    simulate_round_trip, slalom,
)


class SlalomTests(unittest.TestCase):
    def setUp(self):
        self.course = slalom()

    def test_simulated_vehicle_has_velocity_lag_before_translating(self):
        """A command must not become the measured velocity in the same tick."""
        result = simulate(self.course.boxes, config_for(self.course), max_steps=1)
        self.assertGreater(result.decisions[0].forward_m_s, 0.0)
        self.assertLess(result.xs[0], result.decisions[0].forward_m_s*0.05)

    def test_clears_a_four_gate_slalom_without_touching_anything(self):
        result = simulate(self.course.boxes, config_for(self.course))
        self.assertTrue(result.reached_goal)
        self.assertFalse(result.collided)
        self.assertGreater(result.min_clearance, 0.6)
        self.assertGreater(result.min_segment_clearance, 0.6)
        self.assertEqual(result.max_abs_yaw_rate, 0.0)

    def test_crosses_each_gate_through_its_alternating_opening(self):
        result = simulate(self.course.boxes, config_for(self.course))
        self.assertTrue(result.reached_goal)
        # Passing the final X coordinate alone is insufficient: the path must
        # cross each wall plane from the side its opening permits.
        for index, box in enumerate(self.course.boxes):
            at_gate = lateral_at_forward(result, (box[0]+box[1])/2)
            self.assertIsNotNone(at_gate)
            if index % 2 == 0:
                self.assertGreater(at_gate, box[3]+0.35)
            else:
                self.assertLess(at_gate, box[2]-0.35)

    def test_never_stops_moving_forward_through_the_slalom(self):
        result = simulate(self.course.boxes, config_for(self.course))
        longest = current = 0
        for decision in result.decisions:
            current = current+1 if decision.forward_m_s < 0.05 else 0
            longest = max(longest, current)
        self.assertLessEqual(longest*0.05, 0.5)

    def test_does_not_dither_between_left_and_right(self):
        result = simulate(self.course.boxes, config_for(self.course))
        self.assertTrue(result.reached_goal)
        self.assertLessEqual(result.direction_flips, 6)

    def test_slalom_holds_up_across_the_speed_range(self):
        for speed in (1.0, 1.5, 2.0, 2.5):
            with self.subTest(speed=speed):
                result = simulate(self.course.boxes, config_for(self.course, speed))
                self.assertTrue(result.reached_goal)
                self.assertFalse(result.collided)
                self.assertGreater(result.min_clearance, 0.55)
                self.assertEqual(result.max_abs_yaw_rate, 0.0)

    def test_slalom_holds_up_as_gates_get_tighter(self):
        for spacing in (6.0, 8.0, 10.0):
            for opening in (0.5, 1.0, 1.5):
                with self.subTest(spacing=spacing, opening=opening):
                    course = slalom(spacing=spacing, opening=opening)
                    result = simulate(course.boxes, config_for(course))
                    self.assertFalse(result.collided)
                    self.assertGreater(result.min_clearance, 0.45)
                    if not result.reached_goal:
                        self.assertEqual(result.states[-1], "BLOCKED")

    def test_survives_noisy_and_dropping_lidar(self):
        result = simulate(self.course.boxes, config_for(self.course), noise=0.05, seed=817)
        self.assertTrue(result.reached_goal)
        self.assertFalse(result.collided)
        self.assertGreater(result.min_clearance, 0.5)
        self.assertGreater(result.dropped_scans, 0)
        self.assertEqual(result.max_abs_yaw_rate, 0.0)

    def test_passes_an_off_center_pair_and_a_passable_corner(self):
        for boxes, goal in (
                (((7.7, 8.3, -9.0, 0.5), (13.7, 14.3, -0.5, 9.0)), 20.0),
                (((7.7, 8.3, -9.0, 1.0), (8.3, 12.0, 1.0, 1.6)), 16.0)):
            with self.subTest(boxes=boxes):
                result = simulate(boxes, config_for(SlalomCourse(boxes, goal)))
                self.assertTrue(result.reached_goal)
                self.assertFalse(result.collided)
                self.assertGreater(result.min_segment_clearance, 0.5)

    def test_refuses_a_dead_end_without_collision_or_endless_dithering(self):
        boxes = ((7.7, 8.3, -20.0, 20.0), (0.0, 20.0, -20.0, -1.2),
                 (0.0, 20.0, 1.2, 20.0))
        result = simulate(boxes, config_for(SlalomCourse(boxes, 14.0)), max_steps=1000)
        self.assertFalse(result.reached_goal)
        self.assertFalse(result.collided)
        self.assertEqual(result.states[-1], "BLOCKED")
        self.assertEqual(result.direction_flips, 0)

    def test_segment_clearance_catches_a_wall_crossed_in_one_coarse_step(self):
        from offline_slalom import segment_box_distance
        self.assertEqual(segment_box_distance(0.0, 0.0, 10.0, 0.0, (5.0, 6.0, -1.0, 1.0)), 0.0)

    def test_return_home_re_scans_and_bypasses_the_gates_in_reverse(self):
        result = simulate_round_trip(self.course.boxes, config_for(self.course), return_at=42.0)
        self.assertTrue(result.reached_return_marker)
        self.assertTrue(result.returned_home)
        self.assertFalse(result.collided)
        self.assertGreater(result.min_segment_clearance, 0.6)
        self.assertEqual(result.max_abs_yaw_rate, 0.0)
        self.assertEqual(result.terminal_phase, "LANDING")
        self.assertLessEqual(result.home_error_m, 1.5)
        self.assertLessEqual(result.final_speed_m_s, 0.15)
        self.assertIn("TURNAROUND_BRAKE", result.phases)
        self.assertIn("HOME_APPROACH", result.phases)
        self.assertIn("HOME_SETTLE", result.phases)
        return_states = [state for phase, state in zip(result.phases, result.states)
                         if phase in ("RETURN_HOME", "HOME_APPROACH")]
        # Returning is active avoidance, not a stored inverse path: the planner
        # re-enters slide states while rear LiDAR rays are treated as forward.
        self.assertTrue(any(state in ("SLIDE_LEFT", "SLIDE_RIGHT") for state in return_states))

    def test_return_detects_and_avoids_an_obstacle_that_was_not_present_outbound(self):
        return_only_gate = ((34.7, 35.3, -9.0, 0.5),)
        result = simulate_round_trip(
            self.course.boxes, config_for(self.course), return_at=42.0,
            return_boxes=return_only_gate,
        )
        self.assertTrue(result.returned_home)
        self.assertFalse(result.collided)
        self.assertGreater(result.min_segment_clearance, 0.45)
        return_states = [state for phase, state in zip(result.phases, result.states)
                         if phase in ("RETURN_HOME", "HOME_APPROACH")]
        self.assertTrue(any(state in ("SLIDE_LEFT", "SLIDE_RIGHT") for state in return_states))

    def test_blocked_return_fails_and_holds_without_collision(self):
        return_dead_end = ((34.7, 35.3, -20.0, 20.0),)
        result = simulate_round_trip(
            self.course.boxes, config_for(self.course), return_at=42.0,
            return_boxes=return_dead_end, max_steps=8000,
        )
        self.assertTrue(result.reached_return_marker)
        self.assertFalse(result.returned_home)
        self.assertFalse(result.collided)
        self.assertEqual(result.terminal_phase, "FAILED")
        self.assertIn("safe corridor", result.failure_reason)

    def test_noisy_dropping_lidar_completes_round_trip(self):
        result = simulate_round_trip(
            self.course.boxes, config_for(self.course), return_at=42.0,
            noise=0.035, seed=817,
        )
        self.assertTrue(result.returned_home)
        self.assertFalse(result.collided)
        self.assertGreater(result.dropped_scans, 0)
        self.assertGreater(result.min_segment_clearance, 0.45)

    def test_off_center_home_is_recorded_as_the_return_target(self):
        result = simulate_round_trip(
            self.course.boxes, config_for(self.course), return_at=42.0, home_y=0.55,
        )
        self.assertTrue(result.returned_home)
        self.assertFalse(result.collided)
        self.assertLessEqual(result.home_error_m, 1.5)

    def test_sustained_lidar_loss_on_return_has_explicit_failure_reason(self):
        dynamics = OfflineDynamics(burst_start_probability=0.999, burst_length=20)
        # The very aggressive burst model can fail outbound too; the contract is
        # an explicit sensor failure and safe non-collision, never false success.
        result = simulate_round_trip(
            self.course.boxes, config_for(self.course), return_at=42.0,
            dynamics=dynamics, max_steps=2000, seed=3,
        )
        self.assertFalse(result.returned_home)
        self.assertFalse(result.collided)
        self.assertEqual(result.terminal_phase, "FAILED")
        self.assertIn("LiDAR unavailable", result.failure_reason)

    def test_monte_carlo_courses_never_collide(self):
        rng = random.Random(20260910)
        for number in range(200):
            gates = rng.randint(2, 5)
            spacing = rng.uniform(6.0, 10.0)
            opening = rng.uniform(0.5, 1.5)
            shift = rng.uniform(-0.45, 0.45)
            course = slalom(gates=gates, spacing=spacing, opening=opening)
            boxes = tuple((xmin, xmax, ymin+shift, ymax+shift)
                          for xmin, xmax, ymin, ymax in course.boxes)
            shifted = SlalomCourse(boxes, course.goal_x)
            # Monte Carlo checks 200 courses; 10 Hz is the controller's stated
            # operating rate and keeps the exhaustive offline regression practical.
            result = simulate(shifted.boxes, config_for(shifted), dt=0.1, max_steps=1250)
            with self.subTest(course=number):
                self.assertFalse(result.collided)
                self.assertGreater(result.min_clearance, 0.45)
                self.assertEqual(result.max_abs_yaw_rate, 0.0)
                if not result.reached_goal:
                    self.assertEqual(result.states[-1], "BLOCKED")


class PlannerUpgradeTests(unittest.TestCase):
    def test_hands_off_to_the_next_gate_without_returning_to_cruise(self):
        course = slalom(gates=2)
        result = simulate(course.boxes, config_for(course), max_steps=1200)
        states = result.states
        first_right = states.index("SLIDE_RIGHT")
        first_left = states.index("SLIDE_LEFT")
        self.assertLess(first_right, first_left)
        self.assertNotIn("CRUISE", states[first_right:first_left])

    def test_reversal_resets_lateral_integral(self):
        pid = LateralPID(AvoidanceConfig())
        for _ in range(30):
            pid.update(-0.5, 0.0, 0.1, 1.2)
        self.assertLess(pid.integral, -1.0)
        pid.reset()
        self.assertEqual(pid.integral, 0.0)
        self.assertGreater(pid.update(0.5, 0.0, 0.1, 1.2), 0.0)

    def test_nonfinite_and_irregular_inputs_remain_safe(self):
        angles = tuple(math.radians(degree) for degree in range(-180, 181))
        all_inf = Scan(angles, (math.inf,)*361, 0.0)
        all_zero = Scan(angles, (0.0,)*361, 0.1)
        planner = Planner()
        cases = (
            (all_inf, 0.0, Motion()),
            (all_inf, 0.0, Motion()),  # same timestamp
            (all_inf, -5.0, Motion()),  # time moved backward
            (all_zero, 0.1, Motion()),
            (all_inf, 0.2, Motion(forward_m_s=math.nan)),
        )
        for scan, now, motion in cases:
            with self.subTest(now=now, motion=motion):
                decision = planner.update(scan, now, motion)
                self.assertTrue(all(math.isfinite(value) for value in (
                    decision.forward_m_s, decision.right_m_s, decision.yaw_deg_s,
                    decision.front_m, decision.nearest_m,
                )))
                self.assertGreaterEqual(decision.forward_m_s, 0.0)
                self.assertEqual(decision.yaw_deg_s, 0.0)


if __name__ == "__main__":
    unittest.main()
