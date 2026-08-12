import os
import sys
import unittest
from pathlib import Path
from unittest import mock


BACKEND_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_DIR))
os.environ.setdefault("ROBOT_POSE_MODE", "mock")

import server  # noqa: E402
import robot_motion_api  # noqa: E402


class RobotDetailPayloadTest(unittest.TestCase):
    def test_rejects_invalid_robot_id(self):
        with self.assertRaises(ValueError):
            server._robot_detail_payload("../robot2")

    def test_lifecycle_probe_kills_process_group_on_timeout(self):
        fake = mock.Mock(pid=4321)
        fake.communicate.side_effect = [server.subprocess.TimeoutExpired("probe", 0.1), ("", "")]
        with mock.patch.object(server.subprocess, "Popen", return_value=fake), \
                mock.patch.object(server.os, "killpg") as killpg:
            result = server._lifecycle_get_quick(server.ROOT_DIR, "/robot2/heartbeat", 0.1)
        self.assertEqual(result["state"], "timeout")
        killpg.assert_called_once_with(4321, server.signal.SIGKILL)

    def test_filters_nodes_to_selected_robot(self):
        table = {
            "nodes": [
                {"name": "/robot2/bt_navigator", "running": True},
                {"name": "/robot20/planner_server", "running": True},
                {"name": "/robot3/controller_server", "running": True},
            ]
        }
        presence = {"items": [{"id": "robot2", "online": True}]}
        with mock.patch.object(server, "_build_presence_rows", return_value=presence), \
                mock.patch.object(server.ROS_DEBUG_NODES_TABLE, "snapshot", return_value=table), \
                mock.patch.object(server, "_robot_process_metrics", return_value=[]), \
                mock.patch.object(server, "_list_log_bag_matches", return_value={"robots": []}):
            payload = server._robot_detail_payload("robot2")
        self.assertEqual([n["name"] for n in payload["nodes"]], ["/robot2/bt_navigator"])
        self.assertTrue(payload["status"]["online"])


class MultiRobotNavigationTest(unittest.TestCase):
    def test_each_robot_uses_its_own_lifecycle_and_action_namespace(self):
        with mock.patch.object(
            robot_motion_api, "_ros_run", return_value={"ok": True, "output": "Goal accepted"}
        ) as run:
            robot_motion_api.send_navigate_to_pose("robot2", 1.0, 2.0)
            robot2_command = run.call_args.args[0]
            robot_motion_api.send_navigate_to_pose("robot110", 3.0, 4.0)
            robot110_command = run.call_args.args[0]

        self.assertIn("/robot2/navigation/bt_navigator/get_state", robot2_command)
        self.assertIn("/robot2/navigation/navigate_to_pose", robot2_command)
        self.assertNotIn("/robot110/", robot2_command)
        self.assertIn("/robot110/navigation/bt_navigator/get_state", robot110_command)
        self.assertIn("/robot110/navigation/navigate_to_pose", robot110_command)
        self.assertNotIn("/robot2/", robot110_command)

    def test_action_client_is_destroyed_before_node(self):
        with mock.patch.object(
            robot_motion_api, "_ros_run", return_value={"ok": True, "output": "Goal accepted"}
        ) as run:
            robot_motion_api.send_navigate_to_pose("robot2", 1.0, 2.0)
        command = run.call_args.args[0]
        self.assertLess(command.index("client.destroy()"), command.index("node.destroy_node()"))

    def test_ros_traceback_is_reduced_to_navigation_reason(self):
        traceback = (
            "Traceback (most recent call last):\n"
            "  File \"<string>\", line 25, in <module>\n"
            "RuntimeError: navigation stack for robot110 is not active: unconfigured\n"
            "Exception ignored in: <function ActionClient.__del__>\n"
            "rclpy.handle.InvalidHandle: Tried to use a handle that has been destroyed."
        )
        with mock.patch.object(robot_motion_api, "_ros_run", side_effect=RuntimeError(traceback)):
            with self.assertRaisesRegex(
                RuntimeError,
                r"^navigation stack for robot110 is not active: unconfigured$",
            ):
                robot_motion_api.send_navigate_to_pose("robot110", 1.0, 2.0)


if __name__ == "__main__":
    unittest.main()
