import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))
os.environ.setdefault("ROBOT_POSE_MODE", "none")

import server  # noqa: E402


class LogBagDeleteTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.log_root = self.root / "log_bag"
        self.robot = self.log_root / "robot1"
        self.bags = self.robot / "backup" / "bags"
        self.logs = self.robot / "backup" / "logs"
        self.bags.mkdir(parents=True)
        self.logs.mkdir(parents=True)
        self.first = self.bags / "001_terminal_bag"
        self.second = self.bags / "002_terminal_bag"
        self.first.mkdir()
        self.second.mkdir()
        (self.first / "data.db3").write_bytes(b"first")
        (self.second / "data.db3").write_bytes(b"second")
        self.txt = self.logs / "session_terminal_log.txt"
        self.txt.write_text("shared", encoding="utf-8")
        self.match = self.robot / "backup" / "match.json"
        self.match.write_text(
            json.dumps(
                {
                    "version": 2,
                    "bags": {
                        "log_bag/robot1/backup/bags/001_terminal_bag": {
                            "txt": ["log_bag/robot1/backup/logs/session_terminal_log.txt"]
                        },
                        "log_bag/robot1/backup/bags/002_terminal_bag": {
                            "txt": ["log_bag/robot1/backup/logs/session_terminal_log.txt"]
                        },
                    },
                }
            ),
            encoding="utf-8",
        )
        self.patches = (
            mock.patch.object(server, "ROOT_DIR", self.root),
            mock.patch.object(server, "LOG_BAG_DIR", self.log_root),
        )
        for patch in self.patches:
            patch.start()

    def tearDown(self):
        for patch in reversed(self.patches):
            patch.stop()
        self.tmp.cleanup()

    def test_shared_txt_is_deleted_only_after_last_bag(self):
        first = server._delete_log_bags(
            ["log_bag/robot1/backup/bags/001_terminal_bag"]
        )
        self.assertFalse(self.first.exists())
        self.assertTrue(self.second.exists())
        self.assertTrue(self.txt.exists())
        self.assertEqual(first["deleted_txt"], [])
        index = json.loads(self.match.read_text(encoding="utf-8"))["bags"]
        self.assertNotIn("log_bag/robot1/backup/bags/001_terminal_bag", index)
        self.assertIn("log_bag/robot1/backup/bags/002_terminal_bag", index)

        second = server._delete_log_bags(
            ["log_bag/robot1/backup/bags/002_terminal_bag"]
        )
        self.assertFalse(self.second.exists())
        self.assertFalse(self.txt.exists())
        self.assertEqual(len(second["deleted_txt"]), 1)

    def test_rejects_non_backup_terminal_bag(self):
        active = self.robot / "active_terminal_bag"
        active.mkdir()
        with self.assertRaisesRegex(ValueError, "only indexed"):
            server._delete_log_bags([str(active)])

    def test_live_recorder_txt_symlink_prevents_txt_delete(self):
        live_link = self.robot / "session_terminal_log.txt"
        live_link.symlink_to(Path("backup/logs/session_terminal_log.txt"))

        result = server._delete_log_bags(
            [
                "log_bag/robot1/backup/bags/001_terminal_bag",
                "log_bag/robot1/backup/bags/002_terminal_bag",
            ]
        )

        self.assertTrue(self.txt.exists())
        self.assertEqual(result["deleted_txt"], [])
        self.assertEqual(len(result["kept_txt"]), 1)

    def test_missing_bag_index_does_not_keep_txt_or_reappear(self):
        self.first.rename(self.first.with_name(".deleted_first"))

        references = server._indexed_txt_paths()
        listed = server._list_log_bag_matches("robot1")

        self.assertIn(self.txt, references)
        names = [Path(row["bag"]).name for row in listed["robots"][0]["bags"]]
        self.assertEqual(names, ["002_terminal_bag"])

    def test_live_bag_is_listed_without_match_json_or_rotation(self):
        self.match.unlink()
        live = self.robot / "20260826T102037+0800_terminal_bag"
        live.mkdir()
        (live / "active.db3").write_bytes(b"live sqlite")
        (live / "active.db3-wal").write_bytes(b"wal")
        (live / ".opendelivery_task_tags").write_text(
            '"task-1"\n"task-2"\n', encoding="utf-8"
        )

        listed = server._list_log_bag_matches("robot1")
        discovered = server._list_log_bag_matches()

        robot = listed["robots"][0]
        self.assertFalse(robot["no_log"])
        self.assertEqual(len(robot["bags"]), 1)
        entry = robot["bags"][0]
        self.assertEqual(Path(entry["bag"]).name, live.name)
        self.assertEqual(entry["tags"], ["task-1", "task-2"])
        self.assertEqual(entry["started_at"], "2026-08-26T10:20:37+08:00")
        self.assertEqual(entry["reason"], "recording")
        self.assertTrue(entry["live"])
        self.assertFalse(entry["deletable"])
        self.assertFalse(entry["downloadable"])
        self.assertTrue(entry["files"][0]["exists"])
        self.assertFalse(entry["files"][0]["downloadable"])
        self.assertGreater(entry["bytes"], 0)
        self.assertEqual(discovered["robots"][0]["robot_name"], "robot1")

    def test_indexed_bag_remains_downloadable_and_deletable(self):
        listed = server._list_log_bag_matches("robot1")
        entry = listed["robots"][0]["bags"][0]

        self.assertFalse(entry["live"])
        self.assertTrue(entry["deletable"])
        self.assertTrue(entry["downloadable"])
        self.assertTrue(entry["files"][0]["downloadable"])


if __name__ == "__main__":
    unittest.main()
