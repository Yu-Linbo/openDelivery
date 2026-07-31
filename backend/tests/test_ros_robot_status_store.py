import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).resolve().parents[1] / "ros_robot_status_store.py"


def load_store(db_path: Path):
    spec = importlib.util.spec_from_file_location("status_store_under_test", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(os.environ, {"OPEN_DELIVERY_STATUS_DB_PATH": str(db_path)}):
        assert spec.loader is not None
        spec.loader.exec_module(module)
    return module


class RobotStatusStoreTest(unittest.TestCase):
    def test_persists_status_atomically_and_reloads_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            db_path = Path(tmp) / "nested" / "status.json"
            store = load_store(db_path)
            store.set_last_status(
                "robot2",
                robot_name="robot2",
                current_map="floor_1",
                robot_status="ready",
                task_status="idle",
                task_progress=0.5,
                is_simulation=True,
                topic="/robot2/robot_status",
                stamp_ns=123,
            )

            payload = json.loads(db_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["robot2"]["current_map"], "floor_1")
            self.assertEqual(load_store(db_path).get_last_status("robot2")["stamp_ns"], 123)
            self.assertEqual(list(db_path.parent.glob(f".{db_path.name}.*.tmp")), [])

    def test_corrupt_file_is_treated_as_empty_cache(self):
        with tempfile.TemporaryDirectory() as tmp:
            db_path = Path(tmp) / "status.json"
            db_path.write_text("{partial", encoding="utf-8")
            self.assertEqual(load_store(db_path).list_all_last_status(), [])


if __name__ == "__main__":
    unittest.main()
