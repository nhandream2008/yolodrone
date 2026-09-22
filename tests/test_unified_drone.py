"""Regression tests for the unified entry point."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.unified_drone import DroneHUD, UnifiedDroneCLI


class UnifiedDroneTests(unittest.TestCase):
    def setUp(self):
        clear = patch.object(DroneHUD, "_clear_screen")
        clear.start()
        self.addCleanup(clear.stop)
        self.cli = UnifiedDroneCLI()

    def test_sim_passes_px4_directory_before_world(self):
        with patch("scripts.sim_avoid.run", return_value=0) as run_sim:
            result = self.cli._run_sim(SimpleNamespace(world="challenge_yolo"))
        self.assertEqual(result, 0)
        run_sim.assert_called_once_with("~/PX4-Autopilot", "challenge_yolo")

    def test_course_world_uses_declared_axis_and_safe_corridor(self):
        source = SimpleNamespace(
            command="avoid", duration=60.0, altitude=3.0, speed=2.5,
            max_radius=48.0, world="slalom_yolo", return_home=True,
            return_at=42.0, hud=False, dry_run=False, log=None,
            kp=0.8, ki=0.03, kd=0.4, detect_distance=10.0,
        )
        args = self.cli._tao_tham_so_tranh_vat_can(source)
        self.assertEqual(args.expected_course_yaw, 90.0)
        self.assertEqual(args.corridor_half_width, 4.5)
        # Alias tiếng Anh chỉ còn để tương thích caller cũ.
        alias_args = self.cli._build_avoid_args(source)
        self.assertEqual(vars(alias_args), vars(args))


if __name__ == "__main__":
    unittest.main()
