import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))
os.environ.setdefault("ROBOT_POSE_MODE", "none")

import bag_replay  # noqa: E402
import server  # noqa: E402


class LogBagReplaySelectionTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.bags = self.root / "log_bag" / "robot1" / "backup" / "bags"
        self.bags.mkdir(parents=True)
        self.good = self.bags / "good_terminal_bag"
        self.bad = self.bags / "bad_terminal_bag"
        self.good.mkdir()
        self.bad.mkdir()
        self.patches = (
            mock.patch.object(server, "ROOT_DIR", self.root),
            mock.patch.object(server, "LOG_BAG_DIR", self.root / "log_bag"),
        )
        for patch in self.patches:
            patch.start()

    def tearDown(self):
        for patch in reversed(self.patches):
            patch.stop()
        self.tmp.cleanup()

    def _replay(self, path, robot_name=""):
        if path == self.bad:
            raise bag_replay.BagReplayError("损坏的 SQLite")
        return {
            "bag": str(path),
            "robot_name": robot_name,
            "start_time_ns": 1,
            "duration": 2.0,
            "message_count": 1,
            "database_count": 1,
            "topics": [],
            "timeline": {"poses": [{"t": 0.0, "x": 1.0, "y": 2.0}]},
            "warnings": [],
        }

    def test_bad_bag_is_skipped_when_another_bag_can_play(self):
        with mock.patch.object(server.bag_replay, "extract_replay", side_effect=self._replay):
            result = server._extract_log_bag_selection([str(self.good), str(self.bad)])

        self.assertEqual(len(result["segments"]), 1)
        self.assertEqual(result["segments"][0]["robot_name"], "robot1")
        self.assertEqual(result["skipped_bags"], [{
            "bag": "log_bag/robot1/backup/bags/bad_terminal_bag",
            "error": "损坏的 SQLite",
        }])
        self.assertTrue(any("bad_terminal_bag" in warning for warning in result["warnings"]))

    def test_missing_bag_is_skipped_but_single_and_all_missing_fail(self):
        missing = self.bags / "removed_terminal_bag"
        with mock.patch.object(server.bag_replay, "extract_replay", side_effect=self._replay):
            result = server._extract_log_bag_selection([str(missing), str(self.good)])
            with self.assertRaises(FileNotFoundError):
                server._extract_log_bag_selection([str(missing)])
            with self.assertRaises(FileNotFoundError):
                server._extract_log_bag_selection([str(missing), str(self.bags / "also_removed")])

        self.assertEqual(len(result["segments"]), 1)
        self.assertEqual(len(result["skipped_bags"]), 1)

    def test_invalid_path_rejects_entire_selection(self):
        outside = self.root / "outside_terminal_bag"
        outside.mkdir()
        with mock.patch.object(server.bag_replay, "extract_replay", side_effect=self._replay):
            with self.assertRaisesRegex(ValueError, "outside log_bag"):
                server._extract_log_bag_selection([str(self.good), str(outside)])


if __name__ == "__main__":
    unittest.main()
