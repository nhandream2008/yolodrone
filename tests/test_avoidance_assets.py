import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from scripts.install_avoidance import ASSETS, install


PROJECT = Path(__file__).resolve().parents[1]


class AvoidanceAssetsTests(unittest.TestCase):
    def test_lidar_covers_horizontal_body_frame(self):
        root = ET.parse(PROJECT / "models/x500_mono_lidar/model.sdf").getroot()
        self.assertEqual(root.findtext("model/include/uri"), "model://x500_mono_cam")
        sensor = root.find("model/link/sensor")
        self.assertEqual(sensor.attrib["type"], "gpu_lidar")
        self.assertEqual(sensor.findtext("topic"), "/drone/lidar/scan")
        self.assertGreaterEqual(float(sensor.findtext("update_rate")), 10)
        self.assertEqual(root.findtext("model/link/pose").split()[3:], ["0", "0", "0"])
        self.assertEqual(sensor.findtext("pose").split()[3:], ["0", "0", "0"])
        self.assertAlmostEqual(float(sensor.findtext("lidar/scan/horizontal/min_angle")), -math.pi)
        self.assertAlmostEqual(float(sensor.findtext("lidar/scan/horizontal/max_angle")), math.pi)
        self.assertEqual(sensor.findtext("lidar/scan/vertical/samples"), "1")
        self.assertEqual(sensor.findtext("lidar/range/min"), "0.15")
        self.assertEqual(sensor.findtext("lidar/range/max"), "20")
        # Gazebo gpu_lidar emits +Inf for no-hit rays. The flight transport
        # adapter normalizes only that documented no-hit encoding to range_max.
        self.assertEqual(sensor.attrib["type"], "gpu_lidar")

    def test_world_has_local_obstacles_and_required_systems(self):
        text = (PROJECT / "worlds/obstacle_yolo.sdf").read_text()
        self.assertNotIn("http", text)
        root = ET.fromstring(text)
        world = root.find("world")
        self.assertEqual(world.attrib["name"], "obstacle_yolo")
        wall = world.find("model[@name='front_wall']")
        self.assertEqual(wall.findtext("pose").split()[:3], ["9", "0", "3.5"])
        self.assertEqual(wall.findtext("link/collision/geometry/box/size"), "1 8 7")
        self.assertIsNotNone(world.find("model[@name='second_obstacle']/link/collision"))
        self.assertIsNone(world.find("gui/camera"))
        viewer_plugins = {plugin.attrib["filename"] for plugin in world.findall("gui/plugin")}
        self.assertIn("InteractiveViewControl", viewer_plugins)
        self.assertNotIn("CameraTracking", viewer_plugins)
        plugins = {p.attrib["name"].split("::")[-1] for p in world.findall("plugin")}
        self.assertTrue({"Physics", "Sensors", "Imu", "AirPressure", "NavSat", "Magnetometer", "UserCommands"}.issubset(plugins))

    def make_px4(self, directory):
        px4 = Path(directory)
        gz = px4 / "Tools/simulation/gz"
        (gz / "worlds").mkdir(parents=True)
        for name in ("x500_mono_cam", "x500", "x500_base", "mono_cam"):
            stock = gz / "models" / name / "model.sdf"
            stock.parent.mkdir(parents=True)
            stock.write_text("stock model sentinel")
        return px4, gz

    def test_install_idempotent_and_backs_up_custom_assets(self):
        with tempfile.TemporaryDirectory() as directory:
            px4, gz = self.make_px4(directory)
            installed = install(px4)
            self.assertEqual(installed, [gz / path for path in ASSETS])
            install(px4)
            self.assertEqual(list(gz.rglob("*.backup-*")), [])
            changed = gz / "worlds/obstacle_yolo.sdf"
            changed.write_text("previous custom world")
            install(px4)
            backups = list(gz.rglob("*.backup-*"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_text(), "previous custom world")
            self.assertEqual((gz / "models/x500/model.sdf").read_text(), "stock model sentinel")

    def test_missing_dependency_does_not_partially_install(self):
        with tempfile.TemporaryDirectory() as directory:
            px4, gz = self.make_px4(directory)
            (gz / "models/mono_cam/model.sdf").unlink()
            with self.assertRaises(FileNotFoundError):
                install(px4)
            self.assertFalse((gz / "models/x500_mono_lidar").exists())
            self.assertFalse((gz / "worlds/obstacle_yolo.sdf").exists())


if __name__ == "__main__":
    unittest.main()
