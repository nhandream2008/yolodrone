"""Pure data checks for the read-only flight dashboard."""
import math
from pathlib import Path
import sys
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.monitor_flight import status_snapshot


class MonitorFlightTests(unittest.TestCase):
    def test_status_explains_phase_and_uses_measured_speed(self):
        row = {
            "event_type": "SAMPLE", "state": "SLIDE_RIGHT", "phase": "RETURN_HOME",
            "measured_north_m_s": 0.3, "measured_east_m_s": 0.4,
            "actual_forward_m_s": math.nan, "actual_right_m_s": math.nan,
            "altitude_m": 3.0, "home_distance_m": 12.0, "lidar_age_ms": 24.0,
            "transition_reason": "", "reason": "Forward + lateral PID; heading fixed",
        }
        status = status_snapshot([row])
        self.assertEqual(status["phase_label"], "ĐANG VỀ HOME")
        self.assertAlmostEqual(status["speed_m_s"], 0.5)
        self.assertIn("lateral PID", status["reason"])


if __name__ == "__main__":
    unittest.main()
