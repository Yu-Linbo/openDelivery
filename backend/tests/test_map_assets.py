import base64
import json
import tempfile
import unittest
from io import BytesIO
from pathlib import Path

from PIL import Image

import sys

BACKEND_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_DIR))

import map_assets  # noqa: E402


class MapAssetsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.map_dir = Path(self.tmp.name)
        self.floor = "floor1"
        folder = self.map_dir / self.floor
        folder.mkdir()
        (folder / "floor1.pgm").write_bytes(b"P5\n2 2\n255\n\xff\x00\xcd\xfe")
        (folder / "floor1.yaml").write_text(
            "image: floor1.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
            encoding="utf-8",
        )

    def tearDown(self):
        self.tmp.cleanup()

    @staticmethod
    def _png_url(color=(255, 0, 0, 255)):
        buf = BytesIO()
        Image.new("RGBA", (2, 2), color).save(buf, format="PNG")
        return "data:image/png;base64," + base64.b64encode(buf.getvalue()).decode()

    def test_points_are_map_scoped_and_normalized(self):
        rows = map_assets.save_points(
            self.map_dir,
            self.floor,
            [
                {"id": "lift_a", "name": "A 电梯", "type": "elevator", "x": 1, "y": 2},
                {"id": "wait", "name": "待机", "type": "standby", "x": 3, "y": 4, "yaw": 1.2},
            ],
        )
        self.assertEqual(rows[0]["yaw"], 0.0)
        self.assertEqual(map_assets.load_points(self.map_dir, self.floor), rows)
        saved = json.loads((self.map_dir / self.floor / "floor1_points.json").read_text())
        self.assertEqual(saved["map"], self.floor)

    def test_rejects_duplicate_or_invalid_points(self):
        with self.assertRaises(ValueError):
            map_assets.save_points(
                self.map_dir,
                self.floor,
                [
                    {"id": "same", "type": "custom", "x": 0, "y": 0},
                    {"id": "same", "type": "custom", "x": 1, "y": 1},
                ],
            )
        with self.assertRaises(ValueError):
            map_assets.save_points(
                self.map_dir,
                self.floor,
                [{"id": "bad/path", "type": "custom", "x": 0, "y": 0}],
            )

    def test_semantic_round_trip(self):
        result = map_assets.save_semantic(self.map_dir, self.floor, self._png_url())
        self.assertEqual((result["width"], result["height"]), (2, 2))
        assets = map_assets.load_assets(self.map_dir, self.floor)
        self.assertTrue(assets["semantic_available"])
        self.assertTrue(assets["semantic_png"].startswith("data:image/png;base64,"))

    def test_rejects_corrupt_semantic_png(self):
        bad = "data:image/png;base64," + base64.b64encode(b"not png").decode()
        with self.assertRaisesRegex(ValueError, "invalid semantic PNG"):
            map_assets.save_semantic(self.map_dir, self.floor, bad)

    def test_raster_save_validates_pgm(self):
        raw = b"P5\n2 2\n255\n\x00\x7f\xff\x20"
        url = "data:image/x-portable-graymap;base64," + base64.b64encode(raw).decode()
        result = map_assets.save_raster(self.map_dir, self.floor, url)
        self.assertEqual((result["width"], result["height"]), (2, 2))
        self.assertEqual((self.map_dir / self.floor / "floor1.pgm").read_bytes(), raw)


if __name__ == "__main__":
    unittest.main()
