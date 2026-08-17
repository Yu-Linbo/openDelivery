import sys
import unittest
from pathlib import Path


BACKEND_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_DIR))

import ros_task_store as store  # noqa: E402


class RosTaskStoreTest(unittest.TestCase):
    def setUp(self):
        store.clear()

    def tearDown(self):
        store.clear()

    def test_status_is_robot_scoped_and_defensively_copied(self):
        payload = {
            "task_id": "web_nav_1",
            "task_status": "Navigating",
            "work_queue": ["navigation"],
            "model_status": ["Navigating"],
        }
        store.set_status("robot2", payload)
        payload["work_queue"].append("changed")
        saved = store.get_status("robot2")
        self.assertEqual(saved["work_queue"], ["navigation"])
        saved["model_status"][0] = "changed"
        self.assertEqual(store.get_status("robot2")["model_status"], ["Navigating"])
        self.assertIsNone(store.get_status("robot3"))
