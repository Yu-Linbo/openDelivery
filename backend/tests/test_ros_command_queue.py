import sys
import threading
import unittest
from pathlib import Path


BACKEND_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_DIR))

import ros_command_queue as commands  # noqa: E402


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


if __name__ == "__main__":
    unittest.main()
