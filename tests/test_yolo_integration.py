"""Regression tests: YOLO is observation-only and cannot control flight."""

import ast
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import avoid_fly
from obstacle_avoidance import AvoidanceConfig, Motion, Planner, Scan
from scripts.unified_drone import DroneHUD, UnifiedDroneCLI


class YoloObservationOnlyTests(unittest.TestCase):
    """The flight path must not import, subscribe to, or consume YOLO detections."""

    def test_planner_has_no_yolo_or_apf_control_api(self):
        planner = Planner(AvoidanceConfig())
        for name in ("update_tracking", "check_dynamic_avoidance", "compute_apf_velocity", "update_apf"):
            self.assertFalse(hasattr(planner, name), name)

    def test_controller_source_has_no_yolo_detection_subscription(self):
        source = Path(avoid_fly.__file__).read_text(encoding="utf-8")
        self.assertNotIn("/yolo/detections", source)
        self.assertNotIn("rclpy", source)
        self.assertNotIn("dynamic_avoid", source)
        self.assertNotIn("update_tracking", source)

    def test_controller_imports_no_vision_or_ros_module(self):
        tree = ast.parse(Path(avoid_fly.__file__).read_text(encoding="utf-8"))
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])
        self.assertTrue({"rclpy", "ultralytics", "sensor_fusion", "yolo_gz_node"}.isdisjoint(imported))

    def test_planner_does_not_read_world_geometry_or_gate_coordinates(self):
        source = Path("obstacle_avoidance.py").read_text(encoding="utf-8")
        for forbidden in ("worlds/", "slalom_yolo", "challenge_yolo", "obstacle_yolo", "ElementTree"):
            self.assertNotIn(forbidden, source)

    def test_lidar_decision_is_independent_of_detection_like_data(self):
        scan = Scan(
            angles=(0.0,), ranges=(20.0,), received_at=0.0, range_max=20.0,
        )
        first = Planner(AvoidanceConfig()).update(scan, 0.0, Motion())
        second = Planner(AvoidanceConfig()).update(scan, 0.0, Motion())
        self.assertEqual(first, second)
        self.assertEqual(first.yaw_deg_s, 0.0)

    def test_unified_cli_rejects_removed_yolo_controls(self):
        with patch.object(DroneHUD, "_clear_screen"):
            cli = UnifiedDroneCLI()
        for arguments in (
            ["unified_drone.py", "track"],
            ["unified_drone.py", "avoid", "--track"],
            ["unified_drone.py", "avoid", "--dynamic-avoid"],
        ):
            with self.subTest(arguments=arguments), patch.object(sys, "argv", arguments):
                with self.assertRaises(SystemExit):
                    cli.run()


if __name__ == "__main__":
    unittest.main()
