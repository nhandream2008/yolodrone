import asyncio
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from frame_utils import image_to_bgr
from circle_fly import wait_for
from scripts.install_world import install


class ImageTests(unittest.TestCase):
    def test_rgb_with_row_padding(self):
        msg = SimpleNamespace(width=1, height=2, step=4, pixel_format_type=3,
                              data=bytes([255, 0, 0, 99, 0, 255, 0, 99]))
        self.assertEqual(image_to_bgr(msg).tolist(), [[[0, 0, 255]], [[0, 255, 0]]])

    def test_bgra(self):
        msg = SimpleNamespace(width=1, height=1, step=4, pixel_format_type=5, data=bytes([1, 2, 3, 255]))
        self.assertEqual(image_to_bgr(msg).tolist(), [[[1, 2, 3]]])

    def test_truncated_frame_rejected(self):
        msg = SimpleNamespace(width=2, height=2, step=6, pixel_format_type=3, data=b'bad')
        with self.assertRaises(ValueError):
            image_to_bgr(msg)

    def test_depth_is_not_treated_as_rgb(self):
        msg = SimpleNamespace(width=1, height=1, step=4, pixel_format_type=13, data=b'1234')
        with self.assertRaises(ValueError):
            image_to_bgr(msg)


class WorldTests(unittest.TestCase):
    def test_separate_world_and_backup(self):
        with tempfile.TemporaryDirectory() as folder:
            worlds = Path(folder) / 'Tools/simulation/gz/worlds'
            worlds.mkdir(parents=True)
            original = worlds / 'baylands.sdf'
            original.write_text('original')
            out = install(folder)
            install(folder)
            self.assertFalse(list(worlds.glob('*.backup-*')))
            out.write_text('custom edit')
            install(folder)
            self.assertEqual(original.read_text(), 'original')
            self.assertEqual(next(worlds.glob('*.backup-*')).read_text(), 'custom edit')
            world = ET.parse(out).find('world')
            self.assertEqual(world.get('name'), 'baylands_yolo')
            self.assertTrue(world.findall('actor'))
            for pose in world.findall('include/pose'):
                self.assertEqual(len(pose.text.split()), 6)
                self.assertIsNone(pose.find('relative_to'))


class TelemetryTests(unittest.IsolatedAsyncioTestCase):
    async def test_waits_for_condition_and_closes(self):
        closed = []
        async def source():
            try:
                yield False
                yield True
            finally:
                closed.append(True)
        self.assertTrue(await wait_for(source(), bool, 1))
        self.assertTrue(closed)

    async def test_timeout(self):
        async def source():
            await asyncio.sleep(10)
            yield True
        with self.assertRaises(asyncio.TimeoutError):
            await wait_for(source(), bool, 0.01)


if __name__ == '__main__':
    unittest.main()
