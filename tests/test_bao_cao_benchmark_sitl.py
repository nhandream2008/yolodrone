"""Tests for the read-only SITL benchmark evidence analyzer."""

import csv
import json
from pathlib import Path
import tempfile
import unittest

from flight_log import CSV_COLUMNS
from scripts.bao_cao_benchmark_sitl import tong_hop_mot_log, trung_binh


class BaoCaoBenchmarkSitlTests(unittest.TestCase):
    def viet_log(self, folder, ten, *, result="COMPLETE", disarmed=True, front=2.3):
        path = Path(folder) / f"{ten}.csv"
        row = {cot: "" for cot in CSV_COLUMNS}
        row.update({
            "event_type": "SAMPLE", "elapsed_s": "80", "front_m": str(front),
            "nearest_m": "2.0", "measured_north_m_s": "2.5",
            "measured_east_m_s": "0", "pid_error_m": "1.0", "lidar_age_ms": "30",
            "control_cycle_ms": "100", "home_distance_m": "1.1", "state": "CRUISE",
            "speed_profile": "fast_sitl", "side_switch_count": "2",
        })
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerow(row)
        path.with_suffix(".json").write_text(json.dumps({
            "result": result,
            "landing_confirmed": disarmed,
            "disarmed_confirmed": disarmed,
        }), encoding="utf-8")
        return path

    def test_summary_reads_additive_metrics(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.viet_log(folder, "ok")
            summary = tong_hop_mot_log(path)
        self.assertEqual(summary["profile"], "fast_sitl")
        self.assertEqual(summary["mean_control_cycle_ms"], 100.0)
        self.assertTrue(summary["landing_confirmed"])

    def test_aggregate_requires_complete_and_disarmed_for_acceptance_inputs(self):
        with tempfile.TemporaryDirectory() as folder:
            good = tong_hop_mot_log(self.viet_log(folder, "good"))
            failed = tong_hop_mot_log(self.viet_log(folder, "failed", result="FAILED", disarmed=False))
            summary = trung_binh([good, failed])
        self.assertFalse(summary["all_complete"])
        self.assertFalse(summary["all_landed_disarmed"])


if __name__ == "__main__":
    unittest.main()
