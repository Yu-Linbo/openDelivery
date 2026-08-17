import os
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from contextlib import ExitStack
from unittest import mock


BACKEND_DIR = Path(__file__).resolve().parents[1]
PROJECT_ROOT = BACKEND_DIR.parent
sys.path.insert(0, str(BACKEND_DIR))
os.environ.setdefault("ROBOT_POSE_MODE", "mock")

import robot_lifecycle  # noqa: E402
import server  # noqa: E402


class FakeRosNodeManager:
    def __init__(self):
        self._lock = threading.Lock()
        self._managed_nodes = []
        self.controls = []

    def status(self):
        return {"managed_nodes": []}

    def control(self, node_id, action):
        self.controls.append((node_id, action))
        return {"managed_nodes": []}

    def list_ros_nodes(self):
        return []


class MultiRobotSimulationLifecycleTest(unittest.TestCase):
    def _orchestrator(self, root=None):
        return robot_lifecycle.RobotLifecycleOrchestrator(
            Path(root or PROJECT_ROOT), FakeRosNodeManager()
        )

    def test_spawn_slots_are_persistent_and_do_not_overlap(self):
        with tempfile.TemporaryDirectory() as tmp:
            first = self._orchestrator(tmp)
            robot2_slot, robot2_pose = first._spawn_pose_for_robot("robot2")
            robot110_slot, robot110_pose = first._spawn_pose_for_robot("robot110")

            self.assertEqual(robot2_slot, 0)
            self.assertEqual(robot2_pose[:2], (-13.703, 12.825))
            self.assertNotEqual(robot2_slot, robot110_slot)
            self.assertNotEqual(robot2_pose[:2], robot110_pose[:2])

            restored = self._orchestrator(tmp)
            self.assertEqual(
                restored._spawn_pose_for_robot("robot2"),
                (robot2_slot, robot2_pose),
            )
            self.assertEqual(
                restored._spawn_pose_for_robot("robot110"),
                (robot110_slot, robot110_pose),
            )

    def test_spawn_slot_assigns_unique_lidar_collision_bit(self):
        orchestrator = self._orchestrator()
        first = orchestrator._robot_process_spec(
            "robot2", "sim", orchestrator._SPAWN_POSES[0], slot=0
        )
        second = orchestrator._robot_process_spec(
            "robot110", "sim", orchestrator._SPAWN_POSES[1], slot=1
        )

        self.assertIn("SIM_COLLISION_BIT=4", first["cmd"])
        self.assertIn("SIM_COLLISION_BIT=8", second["cmd"])

    def test_invalid_robot_id_is_rejected_before_shell_commands(self):
        orchestrator = self._orchestrator()
        with self.assertRaisesRegex(ValueError, "robot_id invalid"):
            orchestrator._ensure_robot("robot2; shutdown -h now")

    def test_entity_delete_falls_back_to_gazebo_native_transport(self):
        orchestrator = self._orchestrator()
        responses = [
            subprocess.CompletedProcess(["gz"], 0, stdout="model info", stderr=""),
            subprocess.CompletedProcess(["gz"], 0, stdout="", stderr=""),
            subprocess.CompletedProcess(
                ["gz"], 1, stdout="", stderr="Unable to find model robot2"
            ),
        ]
        with ExitStack() as stack:
            stack.enter_context(
                mock.patch.object(orchestrator, "_run_shell", side_effect=RuntimeError("dds"))
            )
            run = stack.enter_context(
                mock.patch.object(robot_lifecycle.subprocess, "run", side_effect=responses)
            )
            stack.enter_context(mock.patch.object(robot_lifecycle.time, "sleep"))

            self.assertTrue(orchestrator._delete_simulation_entity("robot2"))

        commands = [call.args[0] for call in run.call_args_list]
        self.assertIn(["gz", "model", "-m", "robot2", "-d"], commands)

    def test_native_entity_query_reports_absent_model(self):
        orchestrator = self._orchestrator()
        completed = subprocess.CompletedProcess(
            ["gz"], 1, stdout="", stderr="Unable to find model robot2"
        )
        with mock.patch.object(robot_lifecycle.subprocess, "run", return_value=completed):
            self.assertFalse(orchestrator._native_simulation_entity_present("robot2"))

    def test_gazebo_transport_rejects_success_without_pose(self):
        orchestrator = self._orchestrator()
        completed = subprocess.CompletedProcess(
            ["gz"], 0, stdout="Invalid arguments\n", stderr=""
        )
        with mock.patch.object(robot_lifecycle.subprocess, "run", return_value=completed):
            self.assertFalse(orchestrator._gazebo_transport_ready())

    def test_shared_world_is_started_once_for_concurrent_requests(self):
        orchestrator = self._orchestrator()
        state_lock = threading.Lock()
        world_ready = False
        calls = []

        def ready():
            with state_lock:
                return world_ready

        def start_if_needed(*args, **kwargs):
            nonlocal world_ready
            with state_lock:
                calls.append(kwargs)
                if kwargs["autostart"]:
                    world_ready = True

        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_services_ready", side_effect=ready))
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_transport_ready", side_effect=ready))
            stack.enter_context(mock.patch.object(orchestrator, "_terminate_stale_world_processes"))
            stack.enter_context(mock.patch.object(orchestrator, "_start_if_needed", side_effect=start_if_needed))
            stack.enter_context(mock.patch.object(orchestrator, "_wait_for_gazebo"))
            threads = [
                threading.Thread(target=orchestrator._ensure_simulation_world)
                for _ in range(2)
            ]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join(timeout=2.0)

        self.assertEqual(sum(bool(call["autostart"]) for call in calls), 1)
        self.assertEqual(sum(bool(call["force"]) for call in calls), 1)
        self.assertEqual(len(calls), 2)

    def test_shared_world_restarts_when_ros_services_exist_but_transport_hangs(self):
        orchestrator = self._orchestrator()
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_services_ready", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_transport_ready", return_value=False))
            terminate = stack.enter_context(
                mock.patch.object(orchestrator, "_terminate_stale_world_processes")
            )
            start = stack.enter_context(mock.patch.object(orchestrator, "_start_if_needed"))
            stack.enter_context(mock.patch.object(orchestrator, "_wait_for_gazebo"))

            orchestrator._ensure_simulation_world()

        terminate.assert_called_once_with()
        self.assertTrue(start.call_args.kwargs["autostart"])
        self.assertTrue(start.call_args.kwargs["force"])

    def test_healthy_online_robot_is_idempotent(self):
        orchestrator = self._orchestrator()
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_services_ready", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_transport_ready", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_simulation_entity_present", return_value=True))
            ensure_world = stack.enter_context(mock.patch.object(orchestrator, "_ensure_simulation_world"))
            stack.enter_context(mock.patch.object(orchestrator, "status", return_value={"robots": []}))
            orchestrator.startup_selected_robot(
                "robot2", is_online=lambda _rid: True
            )
        ensure_world.assert_not_called()

    def test_online_robot_with_missing_entity_recovers_only_its_stack(self):
        manager = FakeRosNodeManager()
        orchestrator = robot_lifecycle.RobotLifecycleOrchestrator(
            PROJECT_ROOT, manager
        )
        pose = orchestrator._SPAWN_POSES[0]
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_services_ready", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_transport_ready", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_simulation_entity_present", return_value=False))
            stack.enter_context(mock.patch.object(orchestrator, "_sim_managed_running", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_spawn_pose_for_robot", return_value=(0, pose)))
            ensure_world = stack.enter_context(mock.patch.object(orchestrator, "_ensure_simulation_world"))
            stack.enter_context(mock.patch.object(orchestrator, "_ensure_robot_specs"))
            delete_entity = stack.enter_context(mock.patch.object(orchestrator, "_delete_simulation_entity"))
            start_robot = stack.enter_context(mock.patch.object(orchestrator, "_start_if_needed"))
            stack.enter_context(mock.patch.object(orchestrator, "status", return_value={"robots": []}))
            stack.enter_context(mock.patch.object(robot_lifecycle.time, "sleep"))
            orchestrator.startup_selected_robot(
                "robot2", is_online=lambda _rid: True
            )

        ensure_world.assert_called_once_with()
        delete_entity.assert_called_once_with("robot2")
        self.assertIn(("robot2", "pause"), manager.controls)
        self.assertNotIn(("simulation_world", "pause"), manager.controls)
        self.assertTrue(start_robot.call_args.kwargs["force"])

    def test_startup_recovers_unresponsive_world_when_stale_entity_cannot_delete(self):
        orchestrator = self._orchestrator()
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_services_ready", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_simulation_entity_present", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_sim_managed_running", return_value=False))
            stack.enter_context(mock.patch.object(orchestrator, "_ensure_simulation_world"))
            stack.enter_context(mock.patch.object(orchestrator, "_ensure_robot_specs"))
            stack.enter_context(mock.patch.object(orchestrator, "_delete_simulation_entity", return_value=False))
            recover = stack.enter_context(mock.patch.object(orchestrator, "_restart_simulation_world"))
            start_robot = stack.enter_context(mock.patch.object(orchestrator, "_start_if_needed"))
            stack.enter_context(mock.patch.object(orchestrator, "status", return_value={"robots": []}))
            stack.enter_context(mock.patch.object(robot_lifecycle.time, "sleep"))

            orchestrator.startup_selected_robot("robot2", is_online=lambda _rid: False)

        recover.assert_called_once_with()
        start_robot.assert_called_once()

    def test_shutdown_never_stops_shared_world(self):
        manager = FakeRosNodeManager()
        orchestrator = robot_lifecycle.RobotLifecycleOrchestrator(
            PROJECT_ROOT, manager
        )
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(orchestrator, "_ensure_robot_specs"))
            stack.enter_context(mock.patch.object(orchestrator, "_signal_shutdown_via_heartbeat"))
            stack.enter_context(mock.patch.object(orchestrator, "_lifecycle_try"))
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_services_ready", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_simulation_entity_present", return_value=True))
            delete_entity = stack.enter_context(mock.patch.object(orchestrator, "_delete_simulation_entity", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "status", return_value={"robots": []}))
            stack.enter_context(mock.patch.object(robot_lifecycle.time, "sleep"))
            orchestrator.shutdown_selected_robot("robot2")

        self.assertEqual(
            manager.controls,
            [("navigation_robot2", "pause"), ("robot2", "pause")],
        )
        self.assertNotIn(("simulation_world", "pause"), manager.controls)
        delete_entity.assert_called_once_with("robot2")

    def test_shutdown_reports_entity_delete_failure(self):
        orchestrator = self._orchestrator()
        with ExitStack() as stack:
            stack.enter_context(mock.patch.object(orchestrator, "_ensure_robot_specs"))
            stack.enter_context(mock.patch.object(orchestrator, "_signal_shutdown_via_heartbeat"))
            stack.enter_context(mock.patch.object(orchestrator, "_lifecycle_try"))
            stack.enter_context(mock.patch.object(orchestrator, "_gazebo_services_ready", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_simulation_entity_present", return_value=True))
            stack.enter_context(mock.patch.object(orchestrator, "_delete_simulation_entity", return_value=False))
            stack.enter_context(mock.patch.object(robot_lifecycle.time, "sleep"))
            with self.assertRaisesRegex(RuntimeError, "failed to delete Gazebo entity: robot2"):
                orchestrator.shutdown_selected_robot("robot2")

    def test_process_regex_does_not_confuse_robot2_with_robot20(self):
        manager = server.RosNodeManager(PROJECT_ROOT)
        ps_output = (
            "10001 bash sim_bringup.sh robot2\n"
            "10002 bash sim_bringup.sh robot20\n"
            "10003 bash sim_bringup.sh robot2-extra\n"
        )
        completed = subprocess.CompletedProcess(
            ["ps"], 0, stdout=ps_output, stderr=""
        )
        pattern = r"sim_bringup\.sh\s+robot2(?![A-Za-z0-9_-])"
        with mock.patch.object(server.subprocess, "run", return_value=completed):
            self.assertEqual(
                manager._find_pids(pattern, match_regex=True),
                [10001],
            )

    def test_robot_model_has_peer_visible_shell_and_downward_camera(self):
        model = (
            PROJECT_ROOT
            / "src"
            / "simulate"
            / "simulate"
            / "urdf"
            / "simple_2d_robot.urdf.xacro"
        ).read_text(encoding="utf-8")

        self.assertIn('outer_shell_link', model)
        self.assertIn('name="shell_height" value="0.22"', model)
        self.assertIn('<min>0.10</min>', model)
        self.assertIn('<xacro:arg name="collision_filter_plugin"', model)
        self.assertIn('filename="$(arg collision_filter_plugin)"', model)
        self.assertIn('<own_category_bits>$(arg collision_bit)</own_category_bits>', model)
        self.assertIn('name="front_down_camera_joint"', model)
        self.assertIn('rpy="0 0.436332 0"', model)
        self.assertIn('name="front_down_camera"', model)
        self.assertIn('front_down_camera/image_raw', model)
        # The front-mounted lidar uses a forward field of view so its rays never cross
        # the robot shell, while the shell remains visible to peer lidars.
        self.assertIn('<min_angle>-1.5708</min_angle>', model)
        self.assertIn('<max_angle>1.5708</max_angle>', model)

    def test_per_robot_script_never_starts_gazebo(self):
        text = (
            PROJECT_ROOT
            / "src"
            / "system"
            / "system"
            / "scripts"
            / "sim_bringup.sh"
        ).read_text(encoding="utf-8")
        self.assertIn('START_GZ="${SIM_START_GAZEBO:-false}"', text)
        self.assertIn('"spawn_x:=${SPAWN_X}"', text)
        self.assertNotIn("gazebo_running()", text)


if __name__ == "__main__":
    unittest.main()
