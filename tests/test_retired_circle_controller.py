"""Regression tests for retirement of the unsafe legacy circle controller."""

import asyncio
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import circle_fly
from scripts.unified_drone import DroneHUD, UnifiedDroneCLI


class RetiredCircleControllerTests(unittest.TestCase):
    def test_legacy_module_has_no_px4_parameter_mutation(self):
        source = Path(circle_fly.__file__).read_text(encoding="utf-8")
        self.assertNotIn("set_param", source)
        self.assertNotIn("EKF2_GPS_CHECK", source)
        self.assertNotIn("VelocityBodyYawspeed", source)

    def test_legacy_run_refuses_to_fly(self):
        with self.assertRaisesRegex(RuntimeError, "vô hiệu hóa"):
            asyncio.run(circle_fly.run(None))

    def test_unified_cli_rejects_circle(self):
        with patch.object(DroneHUD, "_clear_screen"):
            cli = UnifiedDroneCLI()
        with patch.object(sys, "argv", ["unified_drone.py", "circle"]):
            with self.assertRaises(SystemExit):
                cli.run()

    def test_launchers_do_not_expose_fly(self):
        root = Path(__file__).resolve().parents[1]
        self.assertNotIn("fly", (root / "Drone.ps1").read_text(encoding="utf-8"))
        self.assertNotIn("fly|avoid", (root / "scripts/run.sh").read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
