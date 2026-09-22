"""Regression coverage for deterministic offline fault-injection evidence."""

import unittest

from scripts.bao_cao_benchmark_sitl import phan_vi
from scripts.run_scenario_matrix import SCENARIOS, run_matrix


class ScenarioMatrixTests(unittest.TestCase):
    def test_one_seed_runs_every_declared_fault_family_with_audit_records(self):
        report = run_matrix(seeds=range(1), speed=1.5, gates=4)
        self.assertEqual(set(report["scenarios"]), set(SCENARIOS))
        self.assertEqual(len(report["runs"]), len(SCENARIOS))
        self.assertTrue(report["acceptance"]["all_passed"])
        for record in report["runs"]:
            self.assertIn("clearance_margin_m", record)
            self.assertGreater(record["min_segment_clearance_m"], record["vehicle_radius_m"])

    def test_percentile_interpolates_and_ignores_empty_values(self):
        self.assertEqual(phan_vi([1.0, 2.0, 3.0], 0.5), 2.0)
        self.assertAlmostEqual(phan_vi([1.0, 3.0], 0.95), 2.9)
        self.assertIsNone(phan_vi([], 0.95))


if __name__ == "__main__":
    unittest.main()
