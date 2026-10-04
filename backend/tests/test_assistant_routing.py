import json
import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import openclaw_chat as chat
from assistant_language import EN, ZH, response_texts


class AssistantRoutingTest(unittest.TestCase):
    @mock.patch.object(chat, "_load_robot_context", return_value={"available": True, "robots": [
        {"id": "robot1", "online": False, "status": "shutdown", "task_status": "idle"},
        {"id": "robot2", "online": False, "status": "ready", "task_status": "idle"},
    ]})
    @mock.patch.object(chat, "_load_map_point_catalog", return_value=[])
    @mock.patch.object(chat.shutil, "which", return_value="/usr/bin/openclaw")
    @mock.patch.object(chat.subprocess, "run", return_value=mock.Mock(returncode=0, stderr="", stdout='{"reply":"Which existing robot should I use?","decision":"clarify","actions":[]}'))
    def test_robot_reuse_policy_and_all_existing_ids_are_supplied_to_ai(self, run, which, catalog, context):
        chat.run_chat("Take the delivery upstairs", "opendelivery-reuse-policy", {"robot_id": "robot2"})
        prompt = run.call_args.args[0][-1]
        self.assertIn('"id":"robot1","online":false', prompt)
        self.assertIn('"id":"robot2","online":false', prompt)
        self.assertIn('reuse an EXISTING offline robot', prompt)
        self.assertIn('Never invent a new robot ID', prompt)
        self.assertNotIn('choose a NEW valid robot ID', prompt)

    @mock.patch.object(chat, "_read_json", return_value={"items": [
        {"id": "robot2", "online": True, "live_robot_status": "navigating", "live_task_status": "running"},
        {"id": "robot1", "online": True, "live_robot_status": "ready", "live_task_status": "idle"},
        {"id": "robot3", "online": False},
    ]})
    def test_presence_snapshot_contains_facts_without_backend_robot_choice(self, read):
        context = chat._load_robot_context(8001)
        self.assertEqual(context, {"available": True, "robots": [
            {"id": "robot2", "online": True, "status": "navigating", "task_status": "running"},
            {"id": "robot1", "online": True, "status": "ready", "task_status": "idle"},
            {"id": "robot3", "online": False, "status": "", "task_status": ""},
        ]})

    def test_ai_robot_and_startup_plan_are_preserved_exactly(self):
        actions = [
            {"name": "startup_sim", "arguments": {"robot_id": "robot7"}},
            {"name": "navigate_to_point", "arguments": {"robot_id": "robot7", "floor_id": "test_101", "point": "pickup"}},
            {"name": "navigate_to_point", "arguments": {"robot_id": "robot7", "floor_id": "test_103", "point": "destination"}},
        ]
        self.assertEqual(chat._validate_actions(actions, "execute"), actions)
        self.assertEqual(chat._validate_actions(actions[1:], "execute"), actions[1:])

    def test_query_chat_and_clarification_cannot_execute_mutations(self):
        actions = [{"name": "shutdown_sim", "arguments": {"robot_id": "robot1"}}]
        for decision in ("query", "chat", "clarify", None, "anything"):
            with self.subTest(decision=decision), self.assertRaises(ValueError):
                chat._validate_actions(actions, decision)
        self.assertEqual(chat._validate_actions([{"name": "robot_status", "arguments": {}}], "query"),
                         [{"name": "robot_status", "arguments": {}}])

    def test_whole_plan_is_validated_before_execution(self):
        for action in (
            {"name": "unknown", "arguments": {}},
            {"name": "startup_sim", "arguments": {"robot_id": "../robot1"}},
            {"name": "startup_sim", "arguments": {}},
            {"name": "navigate_to_point", "arguments": {"robot_id": "robot1", "floor_id": "../map", "point": "pickup"}},
            {"name": "navigate_to_point", "arguments": {"robot_id": "robot1", "floor_id": "test_101", "point": ""}},
            {"name": "robot_status", "arguments": {}, "include_result": "true"},
        ):
            with self.subTest(action=action), self.assertRaises(ValueError):
                chat._validate_actions([{"name": "startup_sim", "arguments": {"robot_id": "robot1"}}, action], "execute")
        with self.assertRaises(ValueError):
            chat._validate_actions([], "execute")
        with self.assertRaises(ValueError):
            chat._validate_actions([{"name": "robot_status", "arguments": {}}] * (chat.MAX_ACTIONS + 1), "query")

    @mock.patch.object(chat.time, "sleep")
    @mock.patch.object(chat, "urlopen")
    def test_online_initializing_robot_must_be_ready_before_navigation(self, urlopen, sleep):
        responses = []
        for state in ("initializing", "ready"):
            response = mock.MagicMock()
            response.__enter__.return_value.read.return_value = json.dumps({"items": [{"id": "robot1", "online": True, "live_robot_status": state}]}).encode()
            responses.append(response)
        urlopen.side_effect = responses
        self.assertEqual(chat._wait_for_robot_online("robot1", 8001, timeout_s=1), "ready")
        self.assertEqual(urlopen.call_count, 2)

    @mock.patch.object(chat, "_wait_for_navigation_terminal")
    @mock.patch.object(chat, "_execute_action")
    def test_delivery_waits_for_both_ai_destinations_and_uses_english_results(self, execute, wait):
        events = []
        def action(item, *args):
            floor = item["arguments"]["floor_id"]
            events.append("send:" + floor)
            return {"ok": True, "summary": "Navigation task submitted.", "task_id": floor}
        execute.side_effect = action
        wait.side_effect = lambda robot, task, *args, **kwargs: events.append("finish:" + task)
        job_id = "c" * 32
        chat._JOBS[job_id] = {"status": "queued", "results": [], "status_text": EN}
        try:
            chat._run_action_job(job_id, [{"name": "navigate_to_point", "arguments": {"robot_id": "robot1", "floor_id": floor}}
                                          for floor in ("test_101", "test_103")], 8001)
            self.assertEqual(events, ["send:test_101", "finish:test_101", "send:test_103", "finish:test_103"])
            job = chat.get_action_job(job_id)
            self.assertEqual(job["status"], "completed")
            self.assertEqual([item["summary"] for item in job["results"]], ["Arrived at the destination."] * 2)
        finally:
            chat._JOBS.pop(job_id)

    def test_request_language_is_independent_of_dashboard_locale(self):
        self.assertEqual(response_texts("go floor1 take delivery to 3 floor")[1], EN)
        self.assertEqual(response_texts("去一楼取货送到三楼")[1], ZH)
        language, texts = response_texts("Ve al primer piso", {"language": "es", "status_text": {
            "arrived": "Llegó al destino.", "startup": "{robot_id} listo: {status}", "query": "{unexpected}",
        }})
        self.assertEqual(language, "es")
        self.assertEqual(texts["arrived"], "Llegó al destino.")
        self.assertEqual(texts["query"], EN["query"])

    @mock.patch.object(chat, "_load_robot_context", return_value={"available": True, "robots": [{"id": "robot1", "online": True}]})
    @mock.patch.object(chat, "_load_map_point_catalog", return_value=[])
    @mock.patch.object(chat.shutil, "which", return_value="/usr/bin/openclaw")
    @mock.patch.object(chat.subprocess, "run")
    @mock.patch.object(chat.threading, "Thread")
    def test_ai_execution_decision_does_not_require_any_request_keyword(self, thread, run, which, catalog, context):
        for message in ("Please handle the parcel between the two floors.", "帮我处理刚才说的那件事", "Hazlo, por favor."):
            run.return_value = mock.Mock(returncode=0, stderr="", stdout=json.dumps({
                "reply": "Executing the route.", "language": "en", "decision": "execute",
                "actions": [{"name": "navigate_to_point", "arguments": {"robot_id": "robot7", "floor_id": "test_101", "point": "pickup"}}]}))
            result = chat.run_chat(message, "opendelivery-test1234", {}, defer_mutations=True)
            try:
                self.assertEqual(result["decision"], "execute")
                self.assertEqual(result["robot_id"], "robot7")
                self.assertEqual(thread.call_args.kwargs["args"][1][0]["arguments"]["robot_id"], "robot7")
                self.assertIn("you make the decision", run.call_args.args[0][-1])
            finally:
                chat._JOBS.pop(result["job_id"])

    @mock.patch.object(chat, "_load_robot_context", return_value={"available": True, "robots": []})
    @mock.patch.object(chat, "_load_map_point_catalog", return_value=[])
    @mock.patch.object(chat.shutil, "which", return_value="/usr/bin/openclaw")
    @mock.patch.object(chat.subprocess, "run")
    @mock.patch.object(chat, "_execute_action")
    def test_query_with_operation_keywords_does_not_start_a_job(self, execute, run, which, catalog, context):
        run.return_value = mock.Mock(returncode=0, stderr="", stdout=json.dumps({
            "reply": "Here is how navigation works.", "language": "en", "decision": "query", "actions": []}))
        result = chat.run_chat("How do I go to floor 3? Do not start simulation.", "opendelivery-test1234", {}, defer_mutations=True)
        self.assertEqual(result["decision"], "query")
        self.assertNotIn("job_id", result)
        execute.assert_not_called()
        run.return_value.stdout = json.dumps({"reply": "invalid plan", "decision": "query",
                                             "actions": [{"name": "startup_sim", "arguments": {"robot_id": "robot1"}}]})
        with self.assertRaises(ValueError):
            chat.run_chat("How do I go to floor 3?", "opendelivery-test1234", {}, defer_mutations=True)
        execute.assert_not_called()

    @mock.patch.object(chat, "urlopen")
    def test_raw_result_is_requested_by_ai_flag(self, urlopen):
        response = mock.MagicMock()
        response.__enter__.return_value.read.return_value = b'{"floors":["test_101"]}'
        urlopen.return_value = response
        action = {"name": "floors", "arguments": {}, "include_result": True}
        self.assertEqual(chat._execute_action(action, 8001)["result"], {"floors": ["test_101"]})
        self.assertNotIn("result", chat._execute_action({"name": "floors", "arguments": {}}, 8001))


if __name__ == "__main__":
    unittest.main()
