"""Offline geometry checks for the observation-only camera/LiDAR association."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sensor_fusion import (
    LidarScan, associated_range, bearing_distance_to_world, fuse_box, lidar_window_deg, median_range,
    pixel_to_bearing_deg,
)


def raycast_scan(boxes, now=0.0, angle_step_deg=1.0, range_max=20.0):
    """Raycast axis-aligned rectangles in an x-forward, y-right sensor plane."""
    angles, ranges = [], []
    for degree in range(-90, 91, int(angle_step_deg)):
        dx, dy = math.cos(math.radians(degree)), math.sin(math.radians(degree))
        nearest = range_max
        for xmin, xmax, ymin, ymax in boxes:
            enter, leave = -math.inf, math.inf
            for direction, low, high in ((dx, xmin, xmax), (dy, ymin, ymax)):
                if abs(direction) < 1e-10:
                    if not low <= 0 <= high:
                        enter, leave = math.inf, -math.inf
                        break
                else:
                    first, second = low/direction, high/direction
                    enter, leave = max(enter, min(first, second)), min(leave, max(first, second))
            if leave >= max(enter, 0.0):
                nearest = min(nearest, max(0.0, enter))
        angles.append(float(degree))
        ranges.append(nearest)
    return LidarScan(tuple(angles), tuple(ranges), now, range_max)


class SensorFusionTests(unittest.TestCase):
    def test_pixel_center_maps_to_zero_bearing(self):
        self.assertAlmostEqual(pixel_to_bearing_deg(640, 1280, 90), 0.0)

    def test_pixel_edges_map_to_half_fov(self):
        self.assertAlmostEqual(pixel_to_bearing_deg(0, 1280, 90), -45.0)
        self.assertAlmostEqual(pixel_to_bearing_deg(1280, 1280, 90), 45.0)

    def test_pinhole_is_not_linear(self):
        pinhole = pixel_to_bearing_deg(320, 1280, 90)
        linear = (320/1280-0.5)*90
        self.assertNotAlmostEqual(pinhole, linear)

    def test_range_ignores_max_range_returns(self):
        scan = LidarScan((-1.0, 0.0, 1.0), (20.0, 20.0, 20.0), 1.0, 20.0)
        distance, _ = median_range(scan, 0, 100, 1280, 90, frame_at=1.0)
        self.assertIsNone(distance)

    def test_range_uses_median_not_mean(self):
        scan = LidarScan((-1.0, 0.0, 1.0), (8.0, 8.1, 99.0), 1.0, 100.0)
        distance, _ = median_range(scan, 0, 100, 1280, 90, frame_at=1.0)
        self.assertAlmostEqual(distance, 8.1)

    def test_wall_at_known_distance(self):
        scan = raycast_scan(((8.5, 9.5, -4.0, 4.0),), now=4.0)
        detection = fuse_box((590, 300, 690, 500), "wall", 0.9, 1280, 90, 4.0, scan)
        self.assertIsNotNone(detection.distance_m)
        self.assertLess(abs(detection.distance_m-8.5), 0.15)

    def test_box_to_world_matches_known_obstacle(self):
        distance = math.hypot(17.0, -5.0)
        bearing = math.degrees(math.atan2(-5.0, 17.0))
        north, east = bearing_distance_to_world(distance, bearing, 0.0, 0.0, 0.0)
        self.assertLess(math.hypot(north-17.0, east+5.0), 0.5)

    def test_stale_scan_is_rejected(self):
        scan = raycast_scan(((8.5, 9.5, -4.0, 4.0),), now=0.0)
        detection = fuse_box((590, 300, 690, 500), "wall", 0.9, 1280, 90, 1.0, scan,
                             max_sync_seconds=0.15)
        self.assertIsNone(detection.distance_m)
        self.assertFalse(detection.valid_range)

    def test_missing_pose_invalidates_even_a_fresh_range(self):
        scan = raycast_scan(((8.5, 9.5, -4.0, 4.0),), now=1.0)
        detection = fuse_box((590, 300, 690, 500), "wall", 0.9, 1280, 90, 1.0, scan,
                             pose_valid=False)
        self.assertIsNone(detection.distance_m)
        self.assertIsNone(detection.north_m)
        self.assertIsNone(detection.east_m)
        self.assertFalse(detection.valid_range)

    def test_window_scales_with_box_width(self):
        narrow = lidar_window_deg(20, 1280, 90)
        wide = lidar_window_deg(300, 1280, 90)
        self.assertGreater(wide, narrow)

    def test_camera_lidar_horizontal_offset_is_used_for_close_association(self):
        # A point straight ahead of LiDAR is about +1.95 degrees from the
        # physically offset camera, outside the narrow uncorrected window.
        scan = LidarScan((0.0,), (1.0,), 2.0, 20.0)
        camera_bearing = math.degrees(math.atan2(0.03, 0.88))
        corrected = associated_range(scan, camera_bearing, 5, 1280, 90, 2.0)
        uncorrected = associated_range(
            scan, camera_bearing, 5, 1280, 90, 2.0,
            camera_forward_from_lidar_m=0.0, camera_right_from_lidar_m=0.0,
        )
        self.assertAlmostEqual(corrected[0], 1.0)
        self.assertEqual(corrected[2], 0.0)
        self.assertIsNone(uncorrected[0])

    def test_fusion_module_imports_nothing_heavy(self):
        source = (Path(__file__).resolve().parents[1] / "sensor_fusion.py").read_text().lower()
        for forbidden in ("import rclpy", "import cv2", "import ultralytics", "import mavsdk", "import gz"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
