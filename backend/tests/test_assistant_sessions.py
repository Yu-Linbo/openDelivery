import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import assistant_sessions as sessions
import openclaw_chat


class AssistantSessionsTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.store = sessions.SessionStore(Path(self.temp.name) / "sessions.sqlite")
        self.patch = mock.patch.object(sessions, "STORE", self.store)
        self.patch.start()
        self.addCleanup(self.temp.cleanup)
        self.addCleanup(self.patch.stop)

    def request(self, path, method="POST", user="linbo", peer="100.64.0.2", **data):
        handler = mock.Mock()
        handler.headers = {"X-Auth-User": user, "Content-Length": "0", "Host": "localhost:8001"}
        handler.client_address = (peer, 1234)
        handler._read_json_body.return_value = data
        self.assertTrue(sessions.handle_request(handler, path, method))
        args = handler._send_json.call_args.args
        return args[0], args[1] if len(args) > 1 else 200

    def test_reset_archives_history_and_survives_reopening_store(self):
        first = self.store.ensure("linbo", "opendelivery-legacy", [{"role": "user", "text": "原消息"}])
        second = self.store.reset("linbo", first["id"])
        self.assertNotEqual(first["id"], second["id"])
        reopened = sessions.SessionStore(self.store.path)
        self.assertEqual(reopened.get("linbo", first["id"])["messages"][0]["text"], "原消息")
        self.assertFalse(reopened.get("linbo", first["id"])["active"])
        self.assertFalse(second["legacy"])
        self.assertEqual(second["messages"], [])
        self.assertEqual(reopened.ensure("linbo")["id"], second["id"])
        with self.assertRaises(ValueError):
            reopened.reset("linbo", first["id"])
        self.assertEqual(len(reopened.list("linbo")), 2)

    def test_guest_and_forged_identity_cannot_reset_or_read_history(self):
        first = self.store.ensure("linbo")
        for user, peer in [("guest", "100.64.0.2"), ("linbo", "192.168.1.20"), ("linbo", "100.64.0.3")]:
            with self.subTest(user=user, peer=peer):
                for path, method in [("/api/assistant/reset", "POST"), ("/api/assistant/sessions", "GET"),
                                     ("/api/assistant/sessions/" + first["id"], "GET")]:
                    _, status = self.request(path, method, user, peer, session_id=first["id"])
                    self.assertEqual(status, 403)
        self.assertEqual(len(self.store.list("linbo")), 1)

    def test_localhost_has_default_owner_and_ignores_forged_identity(self):
        for peer, host, origin in [("127.0.0.1", "localhost:8001", "http://localhost:8000"),
                                   ("::1", "[::1]:8001", "http://[::1]:8000")]:
            with self.subTest(peer=peer):
                self.assertEqual(sessions.authenticated_user({"Host": host, "Origin": origin,
                                 "X-Auth-User": "another-user"}, peer), "linbo")
        data, status = self.request("/api/assistant/session", peer="127.0.0.1")
        self.assertEqual(status, 200)
        self.assertTrue(data["authenticated"])
        self.assertEqual(data["user"], "linbo")
        _, status = self.request("/api/assistant/reset", peer="127.0.0.1", session_id=data["session"]["id"])
        self.assertEqual(status, 200)

    def test_local_login_denies_external_origin_and_dns_rebinding(self):
        for headers in [{"Host": "attacker.example:8001"}, {"Host": "localhost:8001", "Origin": "https://attacker.example"},
                        {"Host": "localhost:8001", "Origin": "null"}, {"Host": "[invalid"}]:
            with self.subTest(headers=headers):
                self.assertIsNone(sessions.authenticated_user(headers, "127.0.0.1"))
        with mock.patch.dict(os.environ, {"OPEN_DELIVERY_LOCAL_USER": ""}):
            self.assertIsNone(sessions.authenticated_user({"Host": "localhost:8001"}, "127.0.0.1"))
        self.assertIsNone(sessions.authenticated_user({"Host": "localhost:8001"}, "192.168.1.20"))

    def test_guest_bootstrap_does_not_create_a_session(self):
        data, status = self.request("/api/assistant/session", user="guest", session_id="opendelivery-guest")
        self.assertEqual(status, 200)
        self.assertFalse(data["authenticated"])
        self.assertIsNone(data["session"])
        self.assertFalse(self.store.path.exists())

    def test_user_cannot_read_or_send_to_another_users_session(self):
        first = self.store.ensure("other")
        _, status = self.request("/api/assistant/sessions/" + first["id"], "GET")
        self.assertEqual(status, 404)
        with mock.patch.object(openclaw_chat, "run_chat") as chat:
            _, status = self.request("/api/assistant/chat", session_id=first["id"], message="hello")
        self.assertEqual(status, 409)
        chat.assert_not_called()

    @mock.patch.object(openclaw_chat, "run_chat", return_value={"reply": "你好", "actions": []})
    def test_new_chat_uses_isolated_session_and_persists_both_messages(self, chat):
        first = self.store.ensure("linbo")
        second = self.store.reset("linbo", first["id"])
        _, status = self.request("/api/assistant/chat", session_id=second["id"], message="hello")
        self.assertEqual(status, 200)
        self.assertTrue(chat.call_args.kwargs["isolated_session"])
        self.assertEqual(chat.call_args.kwargs["conversation"], ("linbo", second["id"]))
        self.assertEqual([item["text"] for item in self.store.get("linbo", second["id"])["messages"]], ["hello", "你好"])
        self.assertFalse(self.store.get("linbo", first["id"])["messages"])

    @mock.patch.object(openclaw_chat, "run_chat", return_value={"reply": "旧对话", "actions": []})
    def test_guest_cannot_select_new_openclaw_context(self, chat):
        self.request("/api/assistant/chat", user="guest", session_id="opendelivery-forged", message="hello")
        self.assertFalse(chat.call_args.kwargs["isolated_session"])
        self.assertIsNone(chat.call_args.kwargs["conversation"])
        self.assertFalse(self.store.path.exists())

    @mock.patch.object(openclaw_chat, "_load_map_point_catalog", return_value=[])
    @mock.patch.object(openclaw_chat.shutil, "which", return_value="/usr/bin/openclaw")
    @mock.patch.object(openclaw_chat.subprocess, "run")
    def test_isolated_cli_turn_has_explicit_session_key(self, run, which, catalog):
        run.return_value = mock.Mock(returncode=0, stdout='{"reply":"ok"}', stderr="")
        openclaw_chat.run_chat("hello", "opendelivery-new12345", {}, isolated_session=True)
        command = run.call_args.args[0]
        self.assertEqual(command[command.index("--session-key") + 1], "agent:main:opendelivery-new12345")
        self.assertNotIn("shell", run.call_args.kwargs)

    @mock.patch.object(openclaw_chat, "_execute_action", return_value={"ok": True, "summary": "完成"})
    def test_job_result_remains_in_its_original_archived_session(self, execute):
        first = self.store.ensure("linbo")
        second = self.store.reset("linbo", first["id"])
        job_id = "b" * 32
        openclaw_chat._JOBS[job_id] = {"status": "queued", "results": [], "conversation": ("linbo", first["id"])}
        try:
            openclaw_chat._run_action_job(job_id, [{"name": "startup_sim"}], 8001)
            messages = self.store.get("linbo", first["id"])["messages"]
            self.assertIn("完成", [item["text"] for item in messages])
            self.assertEqual(messages[-1]["text"], "任务已完成，计划中的 1 个步骤全部执行完毕。")
            self.assertEqual(self.store.get("linbo", second["id"])["messages"], [])
        finally:
            openclaw_chat._JOBS.pop(job_id)


if __name__ == "__main__":
    unittest.main()
