import unittest

from mission_control import MissionConfig, RoundTripMission


class RoundTripMissionTests(unittest.TestCase):
    def make_mission(self, **overrides):
        values = dict(return_at_m=42.0, max_radius_m=48.0, min_altitude_m=1.0,
                      max_altitude_m=5.0, turnaround_settle_s=0.5,
                      home_settle_s=0.5)
        values.update(overrides)
        return RoundTripMission(MissionConfig(**values))

    def test_brakes_and_settles_before_switching_return_frame(self):
        mission = self.make_mission()
        directive = mission.update(1.0, 42.0, 42.0, 1.2, 3.0)
        self.assertEqual(directive.phase, "TURNAROUND_BRAKE")
        self.assertTrue(directive.hold_position)
        self.assertFalse(directive.switch_to_return_frame)
        mission.update(2.0, 42.1, 42.1, 0.1, 3.0)
        directive = mission.update(2.6, 42.1, 42.1, 0.1, 3.0)
        self.assertEqual(directive.phase, "RETURN_HOME")
        self.assertTrue(directive.switch_to_return_frame)

    def test_caps_speed_early_enough_to_stop_at_return_marker(self):
        mission = self.make_mission(turnaround_approach_deceleration_m_s2=0.7)
        far = mission.update(1.0, 20.0, 20.0, 2.5, 3.0)
        near = mission.update(2.0, 41.0, 41.0, 2.5, 3.0)
        self.assertGreater(far.speed_limit_m_s, 2.5)
        self.assertAlmostEqual(near.speed_limit_m_s, (2*0.7*1.0)**0.5)
        at_marker = mission.update(3.0, 42.0, 42.0, 0.8, 3.0)
        self.assertEqual(at_marker.phase, "TURNAROUND_BRAKE")
        self.assertTrue(at_marker.hold_position)

    def test_home_requires_both_position_and_stable_velocity(self):
        mission = self.make_mission(turnaround_settle_s=0.0)
        mission.update(0.0, 42.0, 42.0, 0.0, 3.0)
        mission.update(0.1, 42.0, 42.0, 0.0, 3.0)
        directive = mission.update(10.0, 1.0, 1.0, 0.5, 3.0)
        self.assertEqual(directive.phase, "HOME_SETTLE")
        self.assertFalse(directive.request_land)
        mission.update(10.2, 1.0, 1.0, 0.1, 3.0)
        directive = mission.update(10.8, 1.0, 1.0, 0.1, 3.0)
        self.assertEqual(directive.phase, "LANDING")
        self.assertTrue(directive.request_land)

    def test_return_radius_and_altitude_limits_fail_closed(self):
        for distance, altitude, expected in ((48.1, 3.0, "radius"), (20.0, 5.1, "Altitude")):
            mission = self.make_mission()
            directive = mission.update(1.0, 20.0, distance, 1.0, altitude)
            self.assertTrue(directive.failed)
            self.assertIn(expected.lower(), directive.transition_reason.lower())

    def test_passing_home_with_lateral_error_stops_instead_of_flying_away(self):
        mission = self.make_mission(turnaround_settle_s=0.0)
        mission.update(0.0, 42.0, 42.0, 0.0, 3.0)
        mission.update(0.1, 42.0, 42.0, 0.0, 3.0)
        directive = mission.update(10.0, -2.0, 2.5, 0.4, 3.0)
        self.assertEqual(directive.phase, "FAILED")
        self.assertTrue(directive.request_land)

    def test_home_error_increasing_for_two_seconds_fails(self):
        mission = self.make_mission(turnaround_settle_s=0.0, away_timeout_s=1.0)
        mission.update(0.0, 42.0, 42.0, 0.0, 3.0)
        mission.update(0.1, 42.0, 42.0, 0.0, 3.0)
        mission.update(10.0, 4.0, 4.0, 0.3, 3.0)
        mission.update(10.5, 4.8, 4.8, 0.3, 3.0)
        directive = mission.update(11.6, 5.0, 5.0, 0.3, 3.0)
        self.assertTrue(directive.failed)
        self.assertIn("increasing", directive.transition_reason)

    def test_return_slows_before_home_approach_boundary(self):
        mission = self.make_mission(turnaround_settle_s=0.0)
        mission.update(0.0, 42.0, 42.0, 0.0, 3.0)
        mission.update(0.1, 42.0, 42.0, 0.0, 3.0)
        directive = mission.update(1.0, 8.0, 8.0, 2.5, 3.0)
        expected = (0.75**2+2*0.8*(8.0-5.0))**0.5
        self.assertEqual(directive.phase, "RETURN_HOME")
        self.assertAlmostEqual(directive.speed_limit_m_s, expected)

    def test_home_settle_reacquires_on_any_drift_outside_arrival_radius(self):
        mission = self.make_mission(turnaround_settle_s=0.0)
        mission.update(0.0, 42.0, 42.0, 0.0, 3.0)
        mission.update(0.1, 42.0, 42.0, 0.0, 3.0)
        self.assertEqual(mission.update(1.0, 1.4, 1.4, 0.4, 3.0).phase, "HOME_SETTLE")
        directive = mission.update(1.1, 1.55, 1.55, 0.1, 3.0)
        self.assertEqual(directive.phase, "HOME_APPROACH")
        self.assertFalse(directive.request_land)


if __name__ == "__main__":
    unittest.main()
