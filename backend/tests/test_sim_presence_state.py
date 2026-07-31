import os
import sys
import unittest
from pathlib import Path
from unittest import mock


BACKEND_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_DIR))
os.environ.setdefault("ROBOT_POSE_MODE", "mock")

import server  # noqa: E402


class SimPresenceStateTest(unittest.TestCase):
    def setUp(self):
        self.state = server.SimPresenceState()

    def _offline_row(self):
        return [{"id": "robot2", "online": False}]

    def test_starting_survives_process_registration_race(self):
        with mock.patch.object(server.time, "time", return_value=100.0):
            self.state.mark_starting("robot2")
        with mock.patch.object(server.time, "time", return_value=102.0):
            self.state.apply_liveness(self._offline_row(), {})
        self.assertEqual(self.state.snapshot()["phase_by_robot"]["robot2"], "starting")

    def test_starting_clears_after_grace_when_process_never_appears(self):
        with mock.patch.object(server.time, "time", return_value=100.0):
            self.state.mark_starting("robot2")
        with mock.patch.object(server.time, "time", return_value=106.0):
            self.state.apply_liveness(self._offline_row(), {})
        self.assertEqual(self.state.snapshot()["phase_by_robot"]["robot2"], "idle")

    def test_running_process_keeps_starting_until_global_timeout(self):
        managed = {"robot2": {"running": True}}
        with mock.patch.object(server.time, "time", return_value=100.0):
            self.state.mark_starting("robot2")
        with mock.patch.object(server.time, "time", return_value=106.0):
            self.state.apply_liveness(self._offline_row(), managed)
        self.assertEqual(self.state.snapshot()["phase_by_robot"]["robot2"], "starting")
        with mock.patch.object(server.time, "time", return_value=221.0):
            self.state.apply_liveness(self._offline_row(), managed)
        self.assertEqual(self.state.snapshot()["phase_by_robot"]["robot2"], "idle")


if __name__ == "__main__":
    unittest.main()
