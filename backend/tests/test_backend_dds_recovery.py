import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


BACKEND_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_DIR))
os.environ.setdefault("ROBOT_POSE_MODE", "mock")

import ros_tf_bridge  # noqa: E402
import server  # noqa: E402


class BackendDdsRecoveryTest(unittest.TestCase):
    def test_restarts_supervised_backend_when_stack_runs_without_heartbeat(self):
        terminate = mock.Mock()
        status = {"managed_nodes": [{"id": "robot1", "running": True}]}
        with mock.patch.dict(
            os.environ, {"OPEN_DELIVERY_BACKEND_SUPERVISED": "1"}
        ), mock.patch.object(
            server, "_robot_live_online", return_value=False
        ), mock.patch.object(
            server.ROS_NODE_MANAGER, "status", return_value=status
        ):
            recovered = server._run_startup_heartbeat_recovery_check(
                "robot1", terminate_fn=terminate
            )
        self.assertTrue(recovered)
        terminate.assert_called_once_with()

    def test_does_not_restart_when_heartbeat_is_online(self):
        terminate = mock.Mock()
        with mock.patch.dict(
            os.environ, {"OPEN_DELIVERY_BACKEND_SUPERVISED": "1"}
        ), mock.patch.object(server, "_robot_live_online", return_value=True):
            recovered = server._run_startup_heartbeat_recovery_check(
                "robot1", terminate_fn=terminate
            )
        self.assertFalse(recovered)
        terminate.assert_not_called()

    def test_does_not_restart_an_unsupervised_backend(self):
        terminate = mock.Mock()
        with mock.patch.dict(os.environ, {}, clear=True), mock.patch.object(
            server.os, "getppid", return_value=12345
        ), mock.patch.object(server.Path, "read_bytes", side_effect=OSError):
            recovered = server._run_startup_heartbeat_recovery_check(
                "robot1", terminate_fn=terminate
            )
        self.assertFalse(recovered)
        terminate.assert_not_called()

    def test_detects_existing_start_script_parent_during_rolling_restart(self):
        with mock.patch.dict(os.environ, {}, clear=True), mock.patch.object(
            server.os, "getppid", return_value=12345
        ), mock.patch.object(
            server.Path,
            "read_bytes",
            return_value=b"/usr/bin/bash\x00/openDelivery/start_web_stack.sh\x00",
        ):
            self.assertTrue(server._backend_is_supervised())


class RobotStatusCallbackLoggingTest(unittest.TestCase):
    def test_malformed_heartbeat_is_logged_instead_of_silently_dropped(self):
        logger = mock.Mock()
        bridge = SimpleNamespace(
            _robot_status_last_ns={},
            _robot_status_topic_by_id={},
            _robot_status_error_last_log_monotonic={},
            get_logger=mock.Mock(return_value=logger),
        )
        callback = ros_tf_bridge.OpenDeliveryTfBridgeNode._make_robot_status_cb(
            bridge, "robot1", "/robot1/robot_status"
        )

        callback(SimpleNamespace())

        logger.error.assert_called_once()
        self.assertIn("/robot1/robot_status", logger.error.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
