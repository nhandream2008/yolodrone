"""Pure regression tests for safety policy, temporal LiDAR risk, and run evidence."""

import json
import math
from pathlib import Path
import tempfile
import unittest

from flight_log import FlightRecorder
from lidar_risk import TemporalLidarRisk
from obstacle_avoidance import Motion, Scan
from reproducibility import environment_manifest, sha256_file
from safety_supervisor import SafetySupervisor, Severity


def scan(now, front=20.0):
    angles = tuple(math.radians(angle) for angle in range(-180, 181))
    ranges = tuple(front if angle == 0 else 20.0 for angle in range(-180, 181))
    return Scan(angles, ranges, now, 20.0)


class SafetySupervisorTests(unittest.TestCase):
    def test_nominal_and_aging_lidar_are_distinct_auditable_states(self):
        supervisor = SafetySupervisor()
        nominal = supervisor.evaluate(lidar_age_s=0.1, planner_state="CRUISE")
        aging = supervisor.evaluate(lidar_age_s=0.4, planner_state="CRUISE")
        self.assertEqual(nominal.severity, Severity.INFO)
        self.assertEqual(nominal.code, "NOMINAL")
        self.assertEqual(aging.severity, Severity.DEGRADE)
        self.assertEqual(aging.code, "LIDAR_AGING")

    def test_planner_hold_and_fatal_inputs_fail_closed(self):
        supervisor = SafetySupervisor()
        self.assertTrue(supervisor.evaluate(lidar_age_s=0.0, planner_state="BRAKE").hold_position)
        self.assertTrue(supervisor.evaluate(lidar_age_s=0.0, planner_state="CRUISE", telemetry_valid=False).abort_mission)
        self.assertTrue(supervisor.evaluate(lidar_age_s=0.0, planner_state="CRUISE", offboard_active=False).abort_mission)
        self.assertEqual(
            supervisor.evaluate(lidar_age_s=0.0, planner_state="CRUISE", blocked_for_s=8.0).code,
            "CORRIDOR_TIMEOUT",
        )


class TemporalRiskTests(unittest.TestCase):
    def test_first_measurement_has_conservative_braking_cap(self):
        tracker = TemporalLidarRisk()
        risk = tracker.update(scan(0.0, front=4.0), 0.0, Motion(forward_m_s=2.5))
        self.assertGreater(risk.speed_cap_m_s, 0.0)
        self.assertLess(risk.speed_cap_m_s, 2.5)
        self.assertEqual(risk.track_confidence, 0.5)

    def test_approaching_wall_estimates_closing_speed_and_ttc_hold(self):
        tracker = TemporalLidarRisk()
        tracker.update(scan(0.0, front=4.0), 0.0, Motion(forward_m_s=1.0))
        risk = tracker.update(scan(0.1, front=1.5), 0.1, Motion(forward_m_s=1.0))
        self.assertGreater(risk.closing_speed_m_s, 1.0)
        self.assertTrue(risk.hold_recommended)
        self.assertEqual(risk.track_confidence, 1.0)

    def test_clear_sensor_horizon_is_not_mistaken_for_an_obstacle(self):
        tracker = TemporalLidarRisk()
        clear = Scan(
            tuple(math.radians(angle) for angle in range(-180, 181)),
            (20.0,) * 361, 0.0, 20.0,
        )
        risk = tracker.update(clear, 0.0, Motion(forward_m_s=2.5))
        self.assertFalse(risk.hold_recommended)
        self.assertEqual(risk.speed_cap_m_s, math.inf)

    def test_stale_or_missing_scan_recommends_hold(self):
        tracker = TemporalLidarRisk()
        self.assertTrue(tracker.update(None, 0.0).hold_recommended)
        self.assertTrue(tracker.update(scan(0.0), 1.0).hold_recommended)


class ReproducibilityEvidenceTests(unittest.TestCase):
    def test_final_sidecar_seals_csv_and_preserves_environment_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "flight.csv"
            recorder = FlightRecorder(path, {"reproducibility": environment_manifest()})
            recorder.write({"elapsed_s": 0.0, "state": "CRUISE"})
            recorder.finalize("OBSERVATION_COMPLETE", "done", None, False, False)
            sidecar = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
            digest = sha256_file(path)
        self.assertEqual(sidecar["evidence"]["csv_sha256"], digest)
        self.assertIn("python", sidecar["reproducibility"])
        self.assertIn("safety_code", sidecar["evidence"]["csv_schema_columns"])
        self.assertIn("risk_ttc_s", sidecar["evidence"]["csv_schema_columns"])


if __name__ == "__main__":
    unittest.main()
