"""Static checks for the long mixed Gazebo challenge; no simulator is launched."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[1]


class ChallengeWorldTests(unittest.TestCase):
    def setUp(self):
        self.text = (PROJECT / "worlds/challenge_yolo.sdf").read_text(encoding="utf-8")
        self.world = ET.fromstring(self.text).find("world")

    def test_five_gates_are_static_and_alternate_sides(self):
        self.assertNotIn("http", self.text)
        self.assertEqual(self.world.attrib["name"], "challenge_yolo")
        expected = ((7.0, 24.0), (13.5, -24.0), (20.0, 24.0), (26.5, -24.0), (33.0, 24.0))
        for index, (x, y) in enumerate(expected, start=1):
            with self.subTest(gate=index):
                gate = self.world.find(f"model[@name='challenge_gate_{index}']")
                self.assertEqual(gate.findtext("static"), "true")
                pose = [float(value) for value in gate.findtext("pose").split()]
                size = [float(value) for value in gate.findtext("link/collision/geometry/box/size").split()]
                self.assertEqual(pose[:2], [x, y])
                self.assertEqual(size, [0.6, 50.0, 7.0])

    def test_offset_barrier_is_before_finish_and_has_collision(self):
        barrier = self.world.find("model[@name='offset_barrier']")
        finish = self.world.find("model[@name='finish_pad']")
        self.assertEqual(barrier.findtext("static"), "true")
        self.assertIsNotNone(barrier.find("link/collision"))
        self.assertEqual(float(barrier.findtext("pose").split()[0]), 38.5)
        self.assertEqual(barrier.findtext("link/collision/geometry/box/size"), "0.8 1.6 4.4")
        self.assertGreater(float(finish.findtext("pose").split()[0]), 40.0)

    def test_world_has_no_camera_follow_or_controller_plugin(self):
        self.assertIsNone(self.world.find("gui/camera"))
        plugins = {plugin.attrib["filename"] for plugin in self.world.findall("gui/plugin")}
        self.assertIn("InteractiveViewControl", plugins)
        self.assertNotIn("CameraTracking", plugins)
        self.assertFalse(any("controller" in plugin.attrib.get("name", "").lower()
                             for plugin in self.world.findall("plugin")))


if __name__ == "__main__":
    unittest.main()
