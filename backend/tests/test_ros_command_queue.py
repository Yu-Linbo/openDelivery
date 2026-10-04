import sys
import threading
import unittest
from unittest import mock
from pathlib import Path


BACKEND_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_DIR))

import ros_command_queue as commands  # noqa: E402
import diagnostic_logging as diagnostics


class RosCommandQueueTest(unittest.TestCase):
    def setUp(self):
        commands.drain_commands()
        commands.set_bridge_ready(True)

    def tearDown(self):
        commands.drain_commands()
        commands.set_bridge_ready(False)

    def test_waiting_command_receives_async_bridge_result(self):
        received = {}

        def caller():
            received.update(commands.enqueue_command_and_wait({"type": "record"}, timeout=1.0))

        thread = threading.Thread(target=caller)
        thread.start()
        queued = []
        for _ in range(100):
            queued = commands.drain_commands()
            if queued:
                break
            thread.join(0.01)
        self.assertEqual(len(queued), 1)
        commands.complete_command(
            queued[0]["_response_id"], result={"ok": True, "map_name": "floor1"}
        )
        thread.join(1.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(received["map_name"], "floor1")

    def test_ros_dispatch_preserves_http_context_and_logs_failure_details(self):
        import ros_tf_bridge
        bridge = mock.Mock()
        bridge._handle_web_command.side_effect = RuntimeError("publisher unavailable")
        with self.assertLogs("opendelivery.commands", level="INFO") as logs:
            with diagnostics.context(request_id="b" * 32, job_id="c" * 32):
                commands.enqueue_command({"type": "navigation_task", "robot_id": "robot7", "task_id": "task7"})
            ros_tf_bridge.OpenDeliveryTfBridgeNode._process_commands(bridge)
        text = "\n".join(logs.output)
        for field in ('event="ros.command_queued"', 'event="ros.command_failed"', 'command_id=',
                      'request_id="' + 'b' * 32 + '"', 'job_id="' + 'c' * 32 + '"',
                      'robot_id="robot7"', 'task_id="task7"', 'error="publisher unavailable"'):
            self.assertIn(field, text)
        self.assertIn("Traceback", text)
        bridge._tick_teleop.assert_called_once()

    def test_successful_teleop_acknowledgement_is_debug_only(self):
        with mock.patch.object(commands._q, "put_nowait") as put:
            def acknowledge(queued):
                commands.complete_command(queued["_response_id"], result={"ok": True})
            put.side_effect = acknowledge
            with self.assertLogs("opendelivery.commands", level="DEBUG") as logs:
                result = commands.enqueue_command_and_wait({"type": "teleop", "robot_id": "robot7"}, timeout=1)
        self.assertTrue(result["ok"])
        self.assertTrue(all(line.startswith("DEBUG:") for line in logs.output))


if __name__ == "__main__":
    unittest.main()
