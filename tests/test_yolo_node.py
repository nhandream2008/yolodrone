"""Offline contracts for the vision branch's parsing and strict isolation."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from yolo_gz_node import (
    DEFAULT_CAMERA_FROM_LIDAR_FORWARD_M, DEFAULT_CAMERA_FROM_LIDAR_RIGHT_M,
    DEFAULT_CAMERA_H_FOV_DEG, _message_stamp, _scan_from_message, _scan_pose_ned,
    extract_boxes, parse_args,
)


class YoloNodeTests(unittest.TestCase):
    def test_no_detection_returns_empty_not_crash(self):
        result = SimpleNamespace(boxes=SimpleNamespace(xyxy=[], cls=[], conf=[], id=None), names={})
        self.assertEqual(extract_boxes(result), [])

    def test_missing_track_ids_are_accepted_for_real_boxes(self):
        boxes = SimpleNamespace(xyxy=[[10, 20, 30, 60]], cls=[0], conf=[0.8], id=None)
        result = SimpleNamespace(boxes=boxes, names={0: "person"})
        parsed = extract_boxes(result)
        self.assertEqual(parsed[0][1:], ("person", 0.8, None))

    def test_default_fov_is_the_px4_camera_model_value_not_a_90_degree_guess(self):
        self.assertAlmostEqual(DEFAULT_CAMERA_H_FOV_DEG, 99.694656, places=5)
        self.assertAlmostEqual(parse_args([]).camera_hfov_deg, DEFAULT_CAMERA_H_FOV_DEG)
        self.assertEqual(DEFAULT_CAMERA_FROM_LIDAR_FORWARD_M, 0.12)
        self.assertEqual(DEFAULT_CAMERA_FROM_LIDAR_RIGHT_M, -0.03)

    def test_gazebo_capture_timestamps_and_lidar_angle_sign_are_preserved(self):
        header = SimpleNamespace(stamp=SimpleNamespace(sec=12, nsec=250_000_000))
        message = SimpleNamespace(header=header, ranges=(1.0, 2.0, 3.0),
                                  angle_min=-1.0, angle_step=1.0, range_max=20.0)
        self.assertAlmostEqual(_message_stamp(message, 99.0), 12.25)
        scan = _scan_from_message(message, _message_stamp(message, 99.0))
        self.assertEqual(scan.angles_deg, tuple(-math.degrees(value) for value in (-1.0, 0.0, 1.0)))

    def test_scan_world_pose_converts_enu_to_ned(self):
        message = SimpleNamespace(world_pose=SimpleNamespace(
            position=SimpleNamespace(x=5.0, y=7.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ))
        north, east, yaw = _scan_pose_ned(message)
        self.assertEqual((north, east), (7.0, 5.0))
        self.assertAlmostEqual(yaw, 90.0)

    def test_vision_branch_never_touches_flight(self):
        root = Path(__file__).resolve().parents[1]
        files = ("yolo_gz_node.py", "sensor_fusion.py", "frame_utils.py",
                 "scripts/plot_detections.py", "scripts/export_engine.py")
        forbidden = ("mavsdk", "offboard", "set_velocity", "avoid_fly", "obstacle_avoidance")
        for relative in files:
            source = (root / relative).read_text(encoding="utf-8").lower()
            with self.subTest(file=relative):
                for term in forbidden:
                    self.assertNotIn(term, source)


if __name__ == "__main__":
    unittest.main()
