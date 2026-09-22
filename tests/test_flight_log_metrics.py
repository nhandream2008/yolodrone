"""Regression coverage for additive speed-comparison log metrics."""

import csv
from pathlib import Path
import tempfile
import unittest

from flight_log import CSV_COLUMNS, FlightRecorder


class FlightLogMetricsTests(unittest.TestCase):
    def test_speed_comparison_metrics_are_additive_columns(self):
        required = {
            "speed_profile", "profile_simulation_only", "profile_fallback_active",
            "control_cycle_ms", "side_switch_count", "brake_or_stop_state",
        }
        self.assertTrue(required.issubset(CSV_COLUMNS))
        self.assertIn("elapsed_s", CSV_COLUMNS)
        self.assertIn("landing_confirmed", CSV_COLUMNS)
        self.assertIn("disarmed_confirmed", CSV_COLUMNS)

    def test_recorder_writes_new_metrics_without_dropping_legacy_columns(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "run.csv"
            recorder = FlightRecorder(path, {"world": "test"})
            recorder.write({
                "elapsed_s": 1.0,
                "speed_profile": "fast_sitl",
                "profile_simulation_only": True,
                "profile_fallback_active": True,
                "control_cycle_ms": 101.0,
                "side_switch_count": 2,
                "brake_or_stop_state": False,
                "landing_confirmed": False,
            })
            recorder.finalize("OBSERVATION_COMPLETE", "done", None, False, False)
            with path.open(newline="", encoding="utf-8") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["speed_profile"], "fast_sitl")
            self.assertEqual(row["side_switch_count"], "2")
            self.assertEqual(row["profile_fallback_active"], "True")
            self.assertEqual(row["elapsed_s"], "1.0")
            self.assertEqual(row["landing_confirmed"], "False")


if __name__ == "__main__":
    unittest.main()
