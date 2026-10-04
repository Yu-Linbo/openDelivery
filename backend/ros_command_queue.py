"""Thread-safe queue for web -> ROS commands (map switch, initial pose)."""

from __future__ import annotations

import queue
import logging
import threading
import time
import uuid
from typing import Any, Dict, List
import diagnostic_logging as diagnostics

LOGGER = logging.getLogger("opendelivery.commands")

_ready = threading.Event()
_q: queue.Queue = queue.Queue(maxsize=64)
_pending_lock = threading.Lock()
_pending: Dict[str, Dict[str, Any]] = {}


def set_bridge_ready(value: bool) -> None:
    changed = _ready.is_set() != bool(value)
    if value:
        _ready.set()
    else:
        _ready.clear()
    if changed:
        diagnostics.event(LOGGER, logging.INFO if value else logging.WARNING, "ros.bridge_ready_changed", ready=bool(value))


def is_bridge_ready() -> bool:
    return _ready.is_set()


def enqueue_command(cmd: Dict[str, Any]) -> None:
    if not _ready.is_set():
        raise RuntimeError("ROS2 bridge not running")
    queued = dict(cmd)
    queued["_log_context"] = diagnostics.current_context()
    queued["_command_id"] = uuid.uuid4().hex
    _q.put_nowait(queued)
    diagnostics.event(LOGGER, logging.DEBUG if cmd.get("type") == "teleop" else logging.INFO, "ros.command_queued",
                      command_id=queued["_command_id"], command_type=cmd.get("type") or cmd.get("mode"),
                      robot_id=cmd.get("robot_id"), task_id=cmd.get("task_id"), queue_depth=_q.qsize())


def enqueue_command_and_wait(cmd: Dict[str, Any], timeout: float = 8.0) -> Dict[str, Any]:
    """Enqueue a command and wait for the ROS bridge's asynchronous result."""
    request_id = uuid.uuid4().hex
    event = threading.Event()
    slot: Dict[str, Any] = {"event": event, "log_context": diagnostics.current_context(), "started": time.monotonic(),
                            "command_type": cmd.get("type") or cmd.get("mode")}
    with _pending_lock:
        _pending[request_id] = slot
    queued = dict(cmd)
    queued["_response_id"] = request_id
    try:
        enqueue_command(queued)
        if not event.wait(max(0.1, float(timeout))):
            diagnostics.event(LOGGER, logging.ERROR, "ros.command_wait_timed_out", response_id=request_id,
                              robot_id=cmd.get("robot_id"), timeout_s=timeout)
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
            diagnostics.event(LOGGER, logging.WARNING, "ros.command_response_expired", response_id=request_id, error=error or None)
            return
        slot["result"] = result or {}
        slot["error"] = str(error or "")
        slot["event"].set()
    with diagnostics.context(**slot["log_context"]):
        level = logging.ERROR if error else logging.DEBUG if slot["command_type"] == "teleop" else logging.INFO
        diagnostics.event(LOGGER, level, "ros.command_completed", response_id=request_id,
                          error=error or None, duration_ms=round((time.monotonic()-slot["started"])*1000))


def drain_commands() -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    while True:
        try:
            out.append(_q.get_nowait())
        except queue.Empty:
            break
    return out
