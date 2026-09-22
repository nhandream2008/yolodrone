"""Offline contracts for the read-only flight-log replay."""
import csv
from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.plot_slalom_replay import _opening_passed, course_path, metrics, read_log


HEADERS = (
    "elapsed_s", "state", "north_m", "east_m", "forward_m_s", "right_m_s", "yaw_deg_s",
    "front_m", "nearest_m", "target_lateral_m", "command_lateral_m", "lateral_m",
    "actual_forward_m_s", "actual_right_m_s", "yaw_deg", "course_yaw_deg",
)


class SlalomReplayTests(unittest.TestCase):
    def test_gate_pass_requires_the_intended_opening_not_only_crossing_x(self):
        self.assertTrue(_opening_passed(1.31, -20, 1, 1))
        self.assertFalse(_opening_passed(0.0, -20, 1, 1))
        self.assertTrue(_opening_passed(-1.31, -1, 20, -1))
        self.assertFalse(_opening_passed(21.0, -1, 20, -1))

    def test_rotates_ned_telemetry_and_reports_gate_crossings(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "flight.csv"
            samples = (
                (0, "SLIDE_RIGHT", 0, 0, 1, 0.5),
                (8, "SLIDE_RIGHT", -2.5, 8, 1, 0.5),
                (16, "SLIDE_LEFT", 2.5, 16, 1, -0.5),
                (30, "CRUISE", 0, 30, 1, 0),
                (42, "CRUISE", 0, 42, 1, 0),
            )
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=HEADERS)
                writer.writeheader()
                for elapsed, state, north, east, forward, right in samples:
                    writer.writerow({
                        "elapsed_s": elapsed, "state": state, "north_m": north, "east_m": east,
                        "forward_m_s": forward, "right_m_s": right, "yaw_deg_s": 0,
                        "front_m": 8, "nearest_m": 2, "target_lateral_m": right*2,
                        "command_lateral_m": right, "lateral_m": right,
                        "actual_forward_m_s": forward, "actual_right_m_s": right,
                        "yaw_deg": 90, "course_yaw_deg": 90,
                    })
            rows = read_log(path)
            route, yaw = course_path(rows)
            report = metrics(rows, route)
            self.assertEqual(yaw, 90.0)
            self.assertAlmostEqual(route[-1][0], 42.0)
            self.assertAlmostEqual(route[1][1], 2.5)
            self.assertIsNotNone(report["gate_crossings_s"]["Gate 1: right"])
            self.assertEqual(report["max_commanded_yaw_rate_deg_s"], 0.0)
            self.assertTrue(report["home_inferred_from_first_sample"])
            self.assertTrue(any("Home is absent" in item for item in report["limitations"]))

    def test_rich_log_uses_recorded_home_and_reports_both_directions_and_final_result(self):
        headers = HEADERS + (
            "phase", "event_type", "home_north_m", "home_east_m", "home_distance_m",
            "result", "result_reason", "landing_confirmed", "disarmed_confirmed",
        )
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "round-trip.csv"
            samples = (
                (0, "OUTBOUND", "SAMPLE", 100, 200, "", "", ""),
                (10, "OUTBOUND", "SAMPLE", 100, 215, "", "", ""),
                (20, "TURNAROUND_BRAKE", "TRANSITION", 100, 230, "", "", ""),
                (30, "RETURN_HOME", "SAMPLE", 100, 215, "", "", ""),
                (40, "HOME_APPROACH", "SAMPLE", 100, 200.5, "", "", ""),
                (45, "COMPLETE", "FINAL", 100, 200.2, "COMPLETE", "Landed and disarmed", 0.2),
            )
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=headers)
                writer.writeheader()
                for elapsed, phase, event, north, east, result, reason, home_distance in samples:
                    writer.writerow({
                        "elapsed_s": elapsed, "state": "FINAL" if event == "FINAL" else "CRUISE",
                        "phase": phase, "event_type": event, "north_m": north, "east_m": east,
                        "home_north_m": 100, "home_east_m": 200, "home_distance_m": home_distance,
                        "forward_m_s": 1, "right_m_s": 0, "yaw_deg_s": 0,
                        "actual_forward_m_s": 1, "actual_right_m_s": 0,
                        "yaw_deg": 90, "course_yaw_deg": 90, "result": result,
                        "result_reason": reason, "landing_confirmed": event == "FINAL",
                        "disarmed_confirmed": event == "FINAL",
                    })
            rows = read_log(path)
            route, _ = course_path(rows)
            report = metrics(rows, route)
            self.assertAlmostEqual(route[0][0], 0.0)
            self.assertIsNotNone(report["outbound_gate_crossings_s"]["Gate 1: right"])
            self.assertIsNotNone(report["return_gate_crossings_s"]["Gate 1: right"])
            self.assertEqual(report["result"], "COMPLETE")
            self.assertAlmostEqual(report["final_home_distance_m"], 0.2)
            self.assertTrue(report["landing_confirmed"])
            self.assertFalse(report["home_inferred_from_first_sample"])


if __name__ == "__main__":
    unittest.main()
