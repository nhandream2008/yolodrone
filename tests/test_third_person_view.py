"""Static checks that Gazebo starts in its normal free-camera mode."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET



PROJECT = Path(__file__).resolve().parents[1]


class FreeViewTests(unittest.TestCase):
    def test_worlds_use_free_interactive_camera_without_follow_plugin(self):
        for name in ("obstacle_yolo.sdf", "slalom_yolo.sdf", "challenge_yolo.sdf"):
            with self.subTest(world=name):
                world = ET.parse(PROJECT / "worlds" / name).find("world")
                plugins = {plugin.attrib["filename"] for plugin in world.findall("gui/plugin")}
                self.assertIn("MinimalScene", plugins)
                self.assertIn("InteractiveViewControl", plugins)
                self.assertNotIn("CameraTracking", plugins)
                self.assertIsNone(world.find("gui/camera"))


if __name__ == "__main__":
    unittest.main()
