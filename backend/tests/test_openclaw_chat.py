import json
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))
import openclaw_chat  # noqa: E402


class OpenClawChatTest(unittest.TestCase):
    def test_rejects_invalid_input(self):
        with self.assertRaises(ValueError):
            openclaw_chat.run_chat("", "opendelivery-valid", {})
        with self.assertRaises(ValueError):
            openclaw_chat.run_chat("hello", "bad id", {})

    @mock.patch.object(openclaw_chat, "_load_map_point_catalog", return_value=[
        {"floor_id": "test_101", "id": "pickup_front", "name": "前台取货点", "type": "custom"},
    ])
    @mock.patch.object(openclaw_chat.shutil, "which", return_value="/usr/bin/openclaw")
    @mock.patch.object(openclaw_chat.subprocess, "run")
    def test_invokes_cli_without_shell_and_extracts_reply(self, run, _which, _catalog):
        run.return_value = mock.Mock(
            returncode=0,
            stdout=json.dumps({"result": {"payloads": [{"text": "robot1 在线"}]}}),
            stderr="",
        )
        result = openclaw_chat.run_chat("检查 robot1", "opendelivery-12345678", {"view": "monitor"})
        self.assertEqual(result["reply"], "robot1 在线")
        command = run.call_args.args[0]
        self.assertEqual(command[:2], ["/usr/bin/openclaw", "agent"])
        self.assertIn("main", command)
        self.assertNotIn("--session-id", command)
        self.assertNotIn("shell", run.call_args.kwargs)
        self.assertIn("startup_sim", command[-1])
        self.assertIn("开始任务", command[-1])
        self.assertIn("The backend validates and executes actions", command[-1])
        self.assertIn("前台取货点", command[-1])

    @mock.patch.object(openclaw_chat.shutil, "which", return_value="/usr/bin/openclaw")
    @mock.patch.object(openclaw_chat.subprocess, "run")
    def test_surfaces_gateway_failure(self, run, _which):
        run.return_value = mock.Mock(returncode=1, stdout="", stderr="gateway unavailable")
        with self.assertRaisesRegex(RuntimeError, "gateway unavailable"):
            openclaw_chat.run_chat("hello", "opendelivery-12345678", {})

    @mock.patch.object(openclaw_chat, "urlopen")
    def test_executes_confirmed_startup_through_allowlisted_api(self, urlopen):
        startup = mock.MagicMock()
        startup.__enter__.return_value.read.return_value = b'{"ok": true}'
        status = mock.MagicMock()
        status.__enter__.return_value.read.return_value = b'[{"robot_id":"robot1","online":true,"robot_status":"ready"}]'
        urlopen.side_effect = [startup, status]
        result = openclaw_chat._execute_action(
            {"name": "startup_sim", "arguments": {"robot_id": "robot1"}},
            "仿真上线 robot1", 8001,
        )
        self.assertTrue(result["ok"])
        request = urlopen.call_args_list[0].args[0]
        self.assertEqual(request.full_url, "http://127.0.0.1:8001/api/ros/lifecycle/startup")
        self.assertEqual(result["summary"], "robot1 仿真已上线，状态 ready")

    def test_mutation_requires_matching_confirmation(self):
        result = openclaw_chat._execute_action(
            {"name": "shutdown_sim", "arguments": {"robot_id": "robot1"}},
            "robot1 状态如何", 8001,
        )
        self.assertTrue(result["confirmation_required"])
        self.assertNotIn("navigate_to_point", result["summary"])

    def test_plain_stop_never_confirms_sim_shutdown(self):
        result = openclaw_chat._execute_action(
            {"name": "shutdown_sim", "arguments": {"robot_id": "robot1"}},
            "停止", 8001,
        )
        self.assertTrue(result["confirmation_required"])

    @mock.patch.object(openclaw_chat, "urlopen")
    def test_start_execution_confirms_planned_startup(self, urlopen):
        startup = mock.MagicMock()
        startup.__enter__.return_value.read.return_value = b'{"ok":true}'
        status = mock.MagicMock()
        status.__enter__.return_value.read.return_value = b'[{"robot_id":"robot1","online":true,"robot_status":"ready"}]'
        urlopen.side_effect = [startup, status]
        result = openclaw_chat._execute_action(
            {"name": "startup_sim", "arguments": {"robot_id": "robot1"}},
            "开始执行", 8001,
        )
        self.assertTrue(result["ok"])

    def test_continue_and_start_task_confirm_planned_actions(self):
        for message in ("已经上线成功了，继续吧", "开始任务"):
            with self.subTest(message=message):
                self.assertTrue(openclaw_chat._confirmed(message, "navigate_to_point"))

    @mock.patch.object(openclaw_chat.time, "sleep")
    @mock.patch.object(openclaw_chat, "urlopen")
    def test_wait_for_online_accepts_production_items_envelope(self, urlopen, sleep):
        status = mock.MagicMock()
        status.__enter__.return_value.read.return_value = (
            b'{"items":[{"id":"robot1","online":true,"live_robot_status":"ready"}]}'
        )
        urlopen.return_value = status

        result = openclaw_chat._wait_for_robot_online("robot1", 8001, timeout_s=0.1)

        self.assertEqual(result, "ready")
        sleep.assert_not_called()

    @mock.patch.object(openclaw_chat, "urlopen")
    def test_stop_terminates_current_task(self, urlopen):
        detail = mock.MagicMock()
        detail.__enter__.return_value.read.return_value = b'{"task":{"task_id":"task_123"}}'
        stopped = mock.MagicMock()
        stopped.__enter__.return_value.read.return_value = b'{"ok":true}'
        urlopen.side_effect = [detail, stopped]
        result = openclaw_chat._execute_action(
            {"name": "stop_task", "arguments": {"robot_id": "robot1"}},
            "停止", 8001,
        )
        self.assertEqual(result["summary"], "robot1 任务已停止")
        body = json.loads(urlopen.call_args_list[1].args[0].data)
        self.assertEqual(body, {
            "type": "task_command", "robot_id": "robot1",
            "task_id": "task_123", "command": "terminate",
        })

    @mock.patch.object(openclaw_chat, "urlopen")
    def test_model_selected_exact_map_point_is_dispatched(self, urlopen):
        assets = mock.MagicMock()
        assets.__enter__.return_value.read.return_value = json.dumps({
            "points": [{"id": "pickup_front", "name": "前台取货点", "type": "custom", "x": 2, "y": 6, "yaw": 0}],
        }).encode()
        dispatched = mock.MagicMock()
        dispatched.__enter__.return_value.read.return_value = b'{"ok":true,"accepted":true,"task_id":"task_1"}'
        urlopen.side_effect = [assets, dispatched]
        result = openclaw_chat._execute_action(
            {"name": "navigate_to_point", "arguments": {"robot_id": "robot1", "floor_id": "test_101", "point": "前台取货点"}},
            "去1楼取货点", 8001,
        )
        self.assertTrue(result["ok"])
        self.assertEqual(result["task_id"], "task_1")
        body = json.loads(urlopen.call_args_list[1].args[0].data)
        self.assertEqual((body["x"], body["y"], body["floor_id"]), (2, 6, "test_101"))

    @mock.patch.object(openclaw_chat, "_wait_for_navigation_terminal")
    @mock.patch.object(openclaw_chat, "urlopen")
    def test_pickup_and_return_captures_start_pose(self, urlopen, wait):
        def response(payload):
            item = mock.MagicMock()
            item.__enter__.return_value.read.return_value = json.dumps(payload).encode()
            return item
        urlopen.side_effect = [
            response({"points": [{"name": "前台取货点", "type": "custom", "x": 2, "y": 6, "yaw": 0}]}),
            response({"robots": [{"id": "robot1", "active_floor": "test_102", "pose": {"x": 3, "y": 4, "yaw": 1}}]}),
            response({"ok": True, "accepted": True, "task_id": "pickup_1"}),
            response({"ok": True, "accepted": True, "task_id": "return_1"}),
        ]
        result = openclaw_chat._execute_action(
            {"name": "pickup_and_return", "arguments": {"robot_id": "robot1", "floor_id": "test_101", "point": "前台取货点"}},
            "去1楼前台取货后回来", 8001,
        )
        self.assertTrue(result["ok"])
        outbound = json.loads(urlopen.call_args_list[2].args[0].data)
        returning = json.loads(urlopen.call_args_list[3].args[0].data)
        self.assertEqual(outbound["floor_id"], "test_101")
        self.assertEqual(returning, {"robot_id": "robot1", "x": 3.0, "y": 4.0, "yaw": 1.0, "floor_id": "test_102"})
        wait.assert_called_once_with("robot1", "pickup_1", 8001)

    @mock.patch.object(openclaw_chat, "_wait_for_navigation_terminal")
    @mock.patch.object(openclaw_chat, "_execute_action")
    def test_job_waits_for_each_navigation_before_next_action(self, execute, wait):
        execute.side_effect = [
            {"name": "navigate_to_point", "ok": True, "summary": "已下发", "task_id": "task_1"},
            {"name": "navigate_to_point", "ok": True, "summary": "已下发", "task_id": "task_2"},
        ]
        actions = [
            {"name": "navigate_to_point", "arguments": {"robot_id": "robot1"}},
            {"name": "navigate_to_point", "arguments": {"robot_id": "robot1"}},
        ]
        with openclaw_chat._JOBS_LOCK:
            openclaw_chat._JOBS["a" * 32] = {"status": "queued", "results": []}
        openclaw_chat._run_action_job("a" * 32, actions, "开始执行", 8001)
        self.assertEqual([call.args[1] for call in wait.call_args_list], ["task_1", "task_2"])
        self.assertEqual(openclaw_chat.get_action_job("a" * 32)["status"], "completed")

    @mock.patch.object(openclaw_chat, "urlopen")
    def test_short_online_command_is_explicit_confirmation(self, urlopen):
        startup = mock.MagicMock()
        startup.__enter__.return_value.read.return_value = b'{"ok": true}'
        status = mock.MagicMock()
        status.__enter__.return_value.read.return_value = b'[{"robot_id":"robot1","online":true}]'
        urlopen.side_effect = [startup, status]
        result = openclaw_chat._execute_action(
            {"name": "startup_sim", "arguments": {"robot_id": "robot1"}},
            "上线 robot1", 8001,
        )
        self.assertTrue(result["ok"])

    @mock.patch.object(openclaw_chat.Path, "is_file", return_value=True)
    @mock.patch.object(openclaw_chat.shutil, "which", return_value=None)
    @mock.patch.object(openclaw_chat.subprocess, "run")
    def test_falls_back_to_user_npm_bin(self, run, _which, _is_file):
        run.return_value = mock.Mock(returncode=0, stdout='{"reply":"ok"}', stderr="")
        with mock.patch.dict(os.environ, {"OPENCLAW_BIN": ""}, clear=False):
            result = openclaw_chat.run_chat("hello", "opendelivery-12345678", {})
        self.assertEqual(result["reply"], "ok")
        self.assertTrue(run.call_args.args[0][0].endswith("/.npm-global/bin/openclaw"))


if __name__ == "__main__":
    unittest.main()
