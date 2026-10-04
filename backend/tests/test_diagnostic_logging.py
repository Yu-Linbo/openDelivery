import io
import json
import logging
import re
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import diagnostic_logging as diagnostics
import openclaw_chat as chat
import ros_task_store
from managed_logging import ManagedLogWriter, collect, MAX_LINE_BYTES


class CapturedLogs:
    def __enter__(self):
        self.output = io.StringIO()
        self.logger = logging.getLogger("opendelivery")
        self.saved = (self.logger.handlers[:], self.logger.level, self.logger.propagate)
        handler = logging.StreamHandler(self.output)
        handler.setFormatter(diagnostics.RosFormatter())
        self.logger.handlers = [handler]
        self.logger.setLevel(logging.INFO)
        self.logger.propagate = False
        return self.output

    def __exit__(self, *args):
        self.logger.handlers, level, self.logger.propagate = self.saved
        self.logger.setLevel(level)


class DiagnosticLoggingTest(unittest.TestCase):
    def test_safe_fields_cannot_forge_lines_or_expose_credentials(self):
        with CapturedLogs() as output, diagnostics.context(request_id="req-test"):
            diagnostics.event(logging.getLogger("opendelivery.test"), logging.ERROR, "failure",
                              error='line1\n[INFO] forged Authorization: Bearer secret123 token=abc123 "api_key": "key123"',
                              long_field="x" * 1000)
        text = output.getvalue()
        self.assertEqual(len(text.splitlines()), 1)
        self.assertIn('request_id="req-test"', text)
        self.assertIn('time=', text)
        self.assertIn(r'\n[INFO]', text)
        for secret in ("secret123", "abc123", "key123"):
            self.assertNotIn(secret, text)
        self.assertNotIn("x" * 601, text)

    def test_http_errors_are_correlated_without_poll_spam_or_private_query(self):
        class Handler(diagnostics.RequestLoggingMixin, BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def respond(self, status, payload):
                self.record_response_error(payload, status)
                self.send_response(status)
                self.end_headers()
                self.wfile.write(json.dumps(payload).encode())

            def do_POST(self):
                diagnostics.update_context(user="linbo", session_id="session-test")
                self.respond(200, {"ok": True})

            def do_GET(self):
                if self.path.startswith("/failure"):
                    try:
                        raise RuntimeError("database unavailable token=hiddenvalue")
                    except RuntimeError as err:
                        self.respond(500, {"error": str(err)})
                elif self.path.startswith("/missing"):
                    self.respond(404, {"error": "not found"})
                else:
                    self.respond(200, {"items": []})

        with CapturedLogs() as output:
            server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
            server.daemon_threads = False
            thread = threading.Thread(target=server.serve_forever)
            thread.start()
            ids = []
            try:
                for method, path in [("GET", "/poll"), ("POST", "/operation"),
                                     ("GET", "/failure?secret=private-query"), ("GET", "/missing"), ("GET", "/poll")]:
                    connection = HTTPConnection(*server.server_address, timeout=3)
                    headers = {"X-Request-ID": "forged"}
                    if path.startswith("/failure"):
                        headers.update({"X-Diagnostic-Parent-Request": ids[1], "X-Diagnostic-Job": "a" * 32})
                    connection.request(method, path, headers=headers)
                    response = connection.getresponse()
                    ids.append(response.getheader("X-Request-ID"))
                    response.read()
                    connection.close()
            finally:
                server.shutdown()
                server.server_close()
                thread.join(3)
        text = output.getvalue()
        self.assertEqual(text.count('event="http.request_completed"'), 3)
        self.assertEqual(len(set(ids)), len(ids))
        self.assertTrue(all(re.fullmatch(r"[a-f0-9]{32}", value) for value in ids))
        operation = next(line for line in text.splitlines() if 'path="/operation"' in line)
        failure = next(line for line in text.splitlines() if 'path="/failure"' in line)
        self.assertIn(f'request_id="{ids[1]}"', operation)
        self.assertIn('user="linbo"', operation)
        self.assertIn('[ERROR]', failure)
        self.assertIn('duration_ms=', failure)
        self.assertNotIn('user=', failure)
        self.assertIn(f'parent_request_id="{ids[1]}"', failure)
        self.assertIn('job_id="' + 'a' * 32 + '"', failure)
        self.assertIn('Traceback (most recent call last)', text)
        self.assertNotIn("hiddenvalue", text)
        self.assertNotIn("private-query", text)

    def test_ai_job_thread_keeps_request_context_and_internal_api_headers(self):
        response = {"reply": "Executing", "language": "en", "decision": "execute", "actions": [
            {"name": "navigate_to_point", "arguments": {"robot_id": "robot7", "floor_id": "test_101", "point": "pickup"}}]}
        done = threading.Event()
        captured = {}
        def execute(*args):
            captured.update(diagnostics.trace_headers())
            return {"ok": True, "task_id": "task7"}
        def wait(*args, **kwargs):
            done.set()
        with CapturedLogs() as output, diagnostics.context(request_id="b" * 32), \
             mock.patch.object(chat, "_load_robot_context", return_value={"available": True, "robots": []}), \
             mock.patch.object(chat, "_load_map_point_catalog", return_value=[]), \
             mock.patch.object(chat.shutil, "which", return_value="/usr/bin/openclaw"), \
             mock.patch.object(chat.subprocess, "run", return_value=mock.Mock(returncode=0, stderr="", stdout=json.dumps(response))), \
             mock.patch.object(chat, "_execute_action", side_effect=execute), \
             mock.patch.object(chat, "_wait_for_navigation_terminal", side_effect=wait):
            result = chat.run_chat("Deliver", "opendelivery-context-test", {}, defer_mutations=True)
            self.assertTrue(done.wait(2))
            job_id = result["job_id"]
            thread = next((t for t in threading.enumerate() if t.name == "openclaw-action-" + job_id[:8]), None)
            if thread:
                thread.join(2)
            try:
                self.assertEqual(chat._JOBS[job_id]["status"], "completed")
                self.assertEqual(captured["X-Diagnostic-Parent-Request"], "b" * 32)
                self.assertEqual(captured["X-Diagnostic-Job"], job_id)
                self.assertEqual(captured["X-Diagnostic-Session"], "opendelivery-context-test")
                self.assertIn('event="assistant.job_finished"', output.getvalue())
            finally:
                chat._JOBS.pop(job_id)

    def test_job_failure_records_step_target_and_reason_and_stops_plan(self):
        job_id = "d" * 32
        actions = [{"name": "navigate_to_point", "arguments": {"robot_id": "robot7", "floor_id": "test_101", "point": "pickup"}},
                   {"name": "navigate_to_point", "arguments": {"robot_id": "robot7", "floor_id": "test_103", "point": "delivery"}}]
        chat._JOBS[job_id] = {"status": "queued", "results": []}
        try:
            with CapturedLogs() as output, diagnostics.context(request_id="req-job", session_id="session-job"), \
                 mock.patch.object(chat, "_execute_action", return_value={"ok": True, "task_id": "task-7"}) as execute, \
                 mock.patch.object(chat, "_wait_for_navigation_terminal", side_effect=RuntimeError("navigation Failed")):
                chat._run_action_job(job_id, actions, 8001)
            self.assertEqual(chat._JOBS[job_id]["status"], "failed")
            self.assertEqual(execute.call_count, 1)
            text = output.getvalue()
            for field in ('request_id="req-job"', 'session_id="session-job"', f'job_id="{job_id}"',
                          'step=1', 'robot_id="robot7"', 'floor_id="test_101"', 'point="pickup"',
                          'task_id="task-7"', 'error="navigation Failed"', 'status="failed"'):
                self.assertIn(field, text)
            self.assertNotIn('event="assistant.action_completed"', text)
            self.assertIn('[ERROR]', text)
        finally:
            chat._JOBS.pop(job_id)

    def test_unexpected_worker_exception_is_failed_and_keeps_traceback(self):
        job_id = "e" * 32
        chat._JOBS[job_id] = {"status": "queued", "results": []}
        try:
            with CapturedLogs() as output, mock.patch.object(chat, "_execute_action", side_effect=KeyError("unexpected")):
                chat._run_action_job(job_id, [{"name": "robot_status", "arguments": {}}], 8001)
            self.assertEqual(chat._JOBS[job_id]["status"], "failed")
            self.assertIn("Traceback", output.getvalue())
            self.assertIn('error_type="KeyError"', output.getvalue())
        finally:
            chat._JOBS.pop(job_id)

    def test_task_progress_updates_do_not_repeat_state_logs(self):
        ros_task_store.clear()
        with CapturedLogs() as output:
            for progress in range(50):
                ros_task_store.set_status("robot1", {"task_id": "task1", "task_status": "Navigating", "progress": progress})
            ros_task_store.set_status("robot1", {"task_id": "task1", "task_status": "Finished"})
        self.assertEqual(output.getvalue().count('event="ros.task_status_changed"'), 2)
        self.assertIn('previous_status="Navigating"', output.getvalue())
        self.assertEqual(ros_task_store.get_status("robot1")["task_status"], "Finished")
        ros_task_store.clear()


class ManagedLoggingTest(unittest.TestCase):
    def test_node_manager_records_real_process_exit_and_collected_log(self):
        import server
        with tempfile.TemporaryDirectory() as directory, CapturedLogs() as output:
            temporary = Path(directory)
            backend = Path(__file__).resolve().parents[1]
            for name in ("managed_logging.py", "diagnostic_logging.py"):
                (temporary / name).symlink_to(backend / name)
            manager = server.RosNodeManager.__new__(server.RosNodeManager)
            manager._root_dir = temporary
            manager._lock = threading.Lock()
            manager._procs = {}
            manager._bash_prefix = lambda: ""
            spec = {"id": "log_test", "start_cmd": "python3 -c 'import sys; print(\"network error\\n\" * 1000, end=\"\"); sys.exit(3)'"}
            with mock.patch.object(server, "_BACKEND_DIR", temporary):
                manager._start_node(spec)
                proc = manager._procs["log_test"]
                self.assertEqual(proc.wait(timeout=3), 3)
                thread = next((t for t in threading.enumerate() if t.name == "managed-log-log_test"), None)
                if thread:
                    thread.join(3)
                text = (temporary / "logs" / "managed_log_test.log").read_text()
            self.assertIn('repeated_lines=999', text)
            self.assertIn('event="managed.output_closed"', text)
            self.assertIn('event="managed.started"', output.getvalue())
            self.assertIn('event="managed.exited"', output.getvalue())
            self.assertIn('return_code=3', output.getvalue())
            self.assertIn('stop_requested=false', output.getvalue())

    def test_collector_survives_parent_exit_and_drains_remaining_child_output(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "managed.log"
            collector = Path(__file__).resolve().parents[1] / "managed_logging.py"
            # Mirrors the API's ownership: API parent exits while a managed child
            # still owns the write end of the independent collector's pipe.
            parent = '''import subprocess, sys
collector = subprocess.Popen([sys.executable, sys.argv[1], "--path", sys.argv[2], "--node", "test"], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, start_new_session=True)
child = subprocess.Popen([sys.executable, "-u", "-c", "import time; print('before'); time.sleep(0.3); print('after parent exit')"], stdout=collector.stdin, start_new_session=True)
collector.stdin.close()
'''
            subprocess.run([sys.executable, "-c", parent, str(collector), str(path)], check=True, timeout=3)
            deadline = time.monotonic() + 3
            text = ""
            while time.monotonic() < deadline:
                text = path.read_text() if path.exists() else ""
                if 'event="managed.output_closed"' in text:
                    break
                time.sleep(0.02)
            self.assertIn("before", text)
            self.assertIn("after parent exit", text)
            self.assertIn('event="managed.output_closed"', text)

    def test_exact_repeats_are_summarized_and_changes_kept_in_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "managed.log"
            writer = ManagedLogWriter(path, "world")
            collect(io.BytesIO(b"network error\n" * 1000 + b"different error\nlast line\n"), writer)
            writer.close()
            text = path.read_text()
            self.assertEqual(text.count("network error"), 2)  # original + summary sample
            self.assertIn('repeated_lines=999', text)
            self.assertLess(text.index('repeated_lines=999'), text.index("different error"))
            self.assertIn("last line", text)

    def test_repeat_summary_is_periodic_even_during_unbroken_spam(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "managed.log"
            writer = ManagedLogWriter(path, "world", interval_s=10)
            writer.write("network error")
            writer.last_report -= 11
            writer.write("network error")
            self.assertIn('repeated_lines=1', path.read_text())
            writer.close()

    def test_rotation_and_oversized_input_are_bounded_and_preserve_old_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "managed.log"
            path.write_text("old evidence\n" * 100)
            writer = ManagedLogWriter(path, "world", max_bytes=400, backups=2)
            writer.write("first new event")
            self.assertIn("old evidence", Path(str(path) + ".1.log").read_text())
            for index in range(20):
                writer.write(str(index) + "x" * 100)
            writer.close()
            self.assertEqual(len(list(Path(directory).glob("*.log"))), 3)
            self.assertTrue(all(f.stat().st_size <= 400 for f in Path(directory).glob("*.log")))
            other = Path(directory) / "long.log"
            writer = ManagedLogWriter(other, "world")
            collect(io.BytesIO(b"x" * (MAX_LINE_BYTES * 4) + b"\nnext valid line\n"), writer)
            writer.close()
            text = other.read_text()
            self.assertIn('event="managed.output_line_truncated"', text)
            self.assertIn("next valid line", text)
            self.assertLess(len(text), MAX_LINE_BYTES + 600)


if __name__ == "__main__":
    unittest.main()
