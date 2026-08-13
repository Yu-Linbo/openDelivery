import os
import sys
import unittest
from pathlib import Path
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "backend"
sys.path.insert(0, str(BACKEND))
os.environ.setdefault("ROBOT_POSE_MODE", "none")

import robot_motion_api as motion  # noqa: E402


class WebMonitorFeatureTest(unittest.TestCase):
    def tearDown(self):
        motion._TELEOP_PROCESSES.clear()
        motion._TELEOP_SEQUENCE.clear()
        motion._TELEOP_STATE.clear()
        motion._TELEOP_LEASE_DEADLINE.clear()

    def test_teleop_requires_confirmation_and_session_sequence(self):
        with self.assertRaises(ValueError):
            motion.set_teleop_velocity(
                "robot2", 0.2, 0, active=True, confirmed=False,
                session_id="browser", sequence=1,
            )
        with self.assertRaises(ValueError):
            motion.set_teleop_velocity(
                "robot2", 0.2, 0, active=True, confirmed=True,
                session_id="", sequence=0,
            )

    @mock.patch.object(motion, "_start_ros_process")
    def test_stale_start_cannot_override_newer_stop(self, start_process):
        fake = mock.Mock()
        fake.poll.return_value = None
        start_process.return_value = fake
        with mock.patch.object(motion, "_stop_teleop_process"), mock.patch.object(
            motion, "_ros_run", return_value={"ok": True}
        ):
            stopped = motion.set_teleop_velocity(
                "robot2", 0, 0, active=False, confirmed=True,
                session_id="browser", sequence=2,
            )
            stale = motion.set_teleop_velocity(
                "robot2", 0.2, 0, active=True, confirmed=True,
                session_id="browser", sequence=1,
            )
        self.assertFalse(stopped["active"])
        self.assertTrue(stale["stale"])
        start_process.assert_not_called()

    @mock.patch.object(motion, "_ensure_teleop_watchdog")
    @mock.patch.object(motion, "_start_ros_process")
    def test_active_teleop_sets_a_short_lease(self, start_process, ensure_watchdog):
        fake = mock.Mock()
        fake.poll.return_value = None
        start_process.return_value = fake
        result = motion.set_teleop_velocity(
            "robot2", 0.2, 0, active=True, confirmed=True,
            session_id="browser", sequence=1,
        )
        self.assertTrue(result["active"])
        self.assertLessEqual(result["lease_sec"], 1.0)
        self.assertIn("robot2", motion._TELEOP_LEASE_DEADLINE)
        ensure_watchdog.assert_called_once()

    def test_expired_teleop_stops_publisher_and_publishes_zero(self):
        motion._TELEOP_PROCESSES["robot2"] = mock.Mock()
        motion._TELEOP_STATE["robot2"] = ("browser", 0.2, 0.0)
        motion._TELEOP_LEASE_DEADLINE["robot2"] = 10.0
        with mock.patch.object(motion, "_stop_teleop_process") as stop, mock.patch.object(
            motion, "_ros_run", return_value={"ok": True}
        ) as ros_run:
            expired = motion._expire_teleop_leases(now=10.1)
        self.assertEqual(expired, ["robot2"])
        stop.assert_called_once_with("robot2")
        self.assertNotIn("robot2", motion._TELEOP_LEASE_DEADLINE)
        self.assertIn("linear: {x: 0.0", ros_run.call_args.args[0])

    def test_monitor_dom_and_script_contain_requested_features(self):
        html = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
        js = (ROOT / "web" / "app.js").read_text(encoding="utf-8")
        self.assertIn('<select id="reloc-robot-id">', html)
        self.assertIn('id="semantic-map-toggle"', html)
        self.assertIn('id="custom-points-toggle"', html)
        self.assertIn('id="map-editor-layer"', html)
        self.assertIn('data-teleop="forward"', html)
        self.assertIn("syncOnlineRobotSelect", js)
        self.assertIn("/api/robot/motion/teleop", js)
        self.assertIn("visibilitychange", js)
        self.assertIn("teleopHeartbeatTimer", js)
        self.assertIn("teleopHeldRobotId", js)
        self.assertIn("mapEditorDirtyLayers", js)
        self.assertIn("editingPoints", js)
        self.assertIn('tool === "add"', js)
        self.assertIn('["add", "新增点位"]', js)


if __name__ == "__main__":
    unittest.main()
