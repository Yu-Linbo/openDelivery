"""Thread-safe ROS graph snapshot shared by the persistent web bridge.

The web backend must not poll the ROS graph by spawning ``ros2`` CLI
processes. Fast DDS in ROS 2 Foxy leaves shared-memory artifacts behind for
those short-lived participants, so a frequent debug poll eventually prevents
new subscribers (including rosbag2) from discovering publishers.
"""

import json
import threading
import time
from typing import Iterable


_LOCK = threading.Lock()
_SNAPSHOT = {"nodes": [], "timestamp": 0.0}


def set_nodes(names: Iterable[str], *, timestamp: float = None) -> None:
    normalized = sorted(
        {str(name or "").strip() for name in names if str(name or "").strip()}
    )
    snapshot = {
        "nodes": [{"name": name, "running": True} for name in normalized],
        "timestamp": float(time.time() if timestamp is None else timestamp),
    }
    with _LOCK:
        global _SNAPSHOT
        _SNAPSHOT = snapshot


def get_snapshot() -> dict:
    with _LOCK:
        return json.loads(json.dumps(_SNAPSHOT))


def clear() -> None:
    with _LOCK:
        global _SNAPSHOT
        _SNAPSHOT = {"nodes": [], "timestamp": 0.0}
