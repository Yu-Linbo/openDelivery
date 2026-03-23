"""Thread-safe queue for web -> ROS commands (map switch, initial pose)."""

from __future__ import annotations

import queue
import threading
from typing import Any, Dict, List

_ready = threading.Event()
_q: queue.Queue = queue.Queue(maxsize=64)


def set_bridge_ready(value: bool) -> None:
    if value:
        _ready.set()
    else:
        _ready.clear()


def is_bridge_ready() -> bool:
    return _ready.is_set()


def enqueue_command(cmd: Dict[str, Any]) -> None:
    if not _ready.is_set():
        raise RuntimeError("ROS2 bridge not running")
    _q.put_nowait(cmd)


def drain_commands() -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    while True:
        try:
            out.append(_q.get_nowait())
        except queue.Empty:
            break
    return out
