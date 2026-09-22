"""Static checks for the Gazebo slalom course; no simulator process is launched."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[1]


class SlalomWorldTests(unittest.TestCase):
    def test_four_gates_have_the_expected_alternating_openings(self):
        source = PROJECT / "worlds/slalom_yolo.sdf"
        text = source.read_text(encoding="utf-8")
        self.assertNotIn("http", text)
        world = ET.fromstring(text).find("world")
        self.assertEqual(world.attrib["name"], "slalom_yolo")
        expected = ((8.0, 24.0), (15.0, -24.0), (22.0, 24.0), (29.0, -24.0))
        for index, (x, y) in enumerate(expected, start=1):
            with self.subTest(gate=index):
                gate = world.find(f"model[@name='slalom_gate_{index}']")
                pose = [float(value) for value in gate.findtext("pose").split()]
                size = [float(value) for value in gate.findtext("link/collision/geometry/box/size").split()]
                self.assertEqual(pose[:2], [x, y])
                self.assertEqual(size, [0.6, 50.0, 7.0])

    def test_finish_pad_is_beyond_all_gates(self):
        world = ET.parse(PROJECT / "worlds/slalom_yolo.sdf").find("world")
        finish_x = float(world.findtext("model[@name='finish_pad']/pose").split()[0])
        self.assertGreater(finish_x, 29.3)

    def test_world_starts_with_a_normal_free_camera(self):
        world = ET.parse(PROJECT / "worlds/slalom_yolo.sdf").find("world")
        self.assertIsNone(world.find("gui/camera"))
        viewer_plugins = {plugin.attrib["filename"] for plugin in world.findall("gui/plugin")}
        self.assertTrue({"MinimalScene", "GzSceneManager", "InteractiveViewControl",
                         "SelectEntities"}.issubset(viewer_plugins))
        self.assertNotIn("CameraTracking", viewer_plugins)


if __name__ == "__main__":
    unittest.main()
