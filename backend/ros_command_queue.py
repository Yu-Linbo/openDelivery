"""Thread-safe queue for web -> ROS commands (map switch, initial pose)."""

from __future__ import annotations

import queue
import threading
import uuid
from typing import Any, Dict, List

_ready = threading.Event()
_q: queue.Queue = queue.Queue(maxsize=64)
_pending_lock = threading.Lock()
_pending: Dict[str, Dict[str, Any]] = {}


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


def enqueue_command_and_wait(cmd: Dict[str, Any], timeout: float = 8.0) -> Dict[str, Any]:
    """Enqueue a command and wait for the ROS bridge's asynchronous result."""
    request_id = uuid.uuid4().hex
    event = threading.Event()
    slot: Dict[str, Any] = {"event": event}
    with _pending_lock:
        _pending[request_id] = slot
    queued = dict(cmd)
    queued["_response_id"] = request_id
    try:
        enqueue_command(queued)
        if not event.wait(max(0.1, float(timeout))):
            raise TimeoutError("ROS2 bridge command timed out")
        if slot.get("error"):
            raise RuntimeError(str(slot["error"]))
        result = slot.get("result")
        return dict(result) if isinstance(result, dict) else {"ok": True}
    finally:
        with _pending_lock:
            _pending.pop(request_id, None)


def complete_command(request_id: str, *, result: Dict[str, Any] = None, error: str = "") -> None:
    with _pending_lock:
        slot = _pending.get(str(request_id or ""))
        if slot is None:
            return
        slot["result"] = result or {}
        slot["error"] = str(error or "")
        slot["event"].set()


def drain_commands() -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    while True:
        try:
            out.append(_q.get_nowait())
        except queue.Empty:
            break
    return out
