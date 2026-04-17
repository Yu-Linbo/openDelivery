#!/usr/bin/env python3
"""Unit tests for robot lifecycle orchestration (no ROS runtime)."""

import sys
import threading
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

BACKEND = Path(__file__).resolve().parent.parent
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))


class PersistedAutoMappingTest(unittest.TestCase):
    def test_mapping_only(self):
        from robot_lifecycle import _persisted_auto_mapping

        self.assertTrue(_persisted_auto_mapping({"robot_status": "mapping"}))
        self.assertTrue(_persisted_auto_mapping({"robot_status": " Mapping "}))
        self.assertFalse(_persisted_auto_mapping({"robot_status": "navigate"}))
        self.assertFalse(_persisted_auto_mapping({}))
        self.assertFalse(_persisted_auto_mapping(None))


class StatusMergeTest(unittest.TestCase):
    @patch("ros_robot_status_store.get_last_status")
    def test_will_auto_flag_from_store(self, gls):
        from robot_lifecycle import RobotLifecycleOrchestrator

        gls.return_value = {"robot_id": "robot2", "robot_status": "mapping"}
        mgr = MagicMock()
        mgr.status.return_value = {"managed_nodes": []}
        mgr.list_ros_nodes.return_value = []
        mgr._managed_nodes = []
        mgr._lock = threading.Lock()
        orch = RobotLifecycleOrchestrator(Path("/tmp/nonexistent_ws"), mgr)
        with patch.object(
            RobotLifecycleOrchestrator,
            "_lifecycle_get",
            return_value={"available": False, "state": "missing"},
        ):
            out = orch.status("robot2")
        self.assertEqual(len(out["robots"]), 1)
        r0 = out["robots"][0]
        self.assertTrue(r0["will_auto_start_mapping_on_sim_bringup"])
        mapc = next(c for c in r0["components"] if c["id"] == "mapping")
        self.assertTrue(mapc.get("sim_bringup_autostart_mapping"))

    @patch("ros_robot_status_store.get_last_status")
    def test_list_robots_from_store_gets_state(self, gls):
        from robot_lifecycle import RobotLifecycleOrchestrator

        gls.side_effect = lambda rid: (
            {"robot_id": rid, "robot_status": "idle"} if rid == "z1" else None
        )
        mgr = MagicMock()
        mgr.status.return_value = {"managed_nodes": []}
        mgr.list_ros_nodes.return_value = []
        mgr._managed_nodes = []
        mgr._lock = threading.Lock()
        orch = RobotLifecycleOrchestrator(Path("/tmp/nonexistent_ws"), mgr)
        with patch.object(
            RobotLifecycleOrchestrator,
            "_lifecycle_get",
            return_value={"available": False, "state": "missing"},
        ), patch(
            "ros_robot_status_store.list_all_last_status",
            return_value=[{"robot_id": "z1", "robot_status": "idle"}],
        ):
            out = orch.status()
        ids = {r["robot_id"] for r in out["robots"]}
        self.assertIn("z1", ids)


class StartupSimScriptOnlyTest(unittest.TestCase):
    @patch("ros_robot_status_store.get_last_status")
    def test_startup_registers_script_then_aux_managed_autostart_false(self, gls):
        """仿真上线只 start sim_bringup.sh；localization/mapping/navigation 仅注册供状态与 pause。"""
        from robot_lifecycle import RobotLifecycleOrchestrator

        gls.return_value = {"robot_status": "navigate"}
        mgr = MagicMock()
        mgr.status.return_value = {"managed_nodes": []}
        mgr._managed_nodes = []
        mgr._lock = threading.Lock()
        orch = RobotLifecycleOrchestrator(Path("/tmp/nonexistent_ws"), mgr)
        recorded = []

        def cap(node_id, cmd, **kwargs):
            recorded.append((node_id, kwargs.get("autostart", True)))

        fake_script = Path(__file__)
        with patch.object(orch, "_start_if_needed", side_effect=cap), patch.object(
            orch, "_sim_bringup_script_path", return_value=fake_script
        ):
            orch.startup_selected_robot("r9", sim_mode="sim")
        self.assertEqual(
            [x[0] for x in recorded],
            ["r9", "localization_r9", "mapping_r9", "navigation_r9"],
        )
        self.assertEqual([x[1] for x in recorded], [True, False, False, False])


if __name__ == "__main__":
    unittest.main()
