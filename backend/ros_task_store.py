"""Thread-safe latest root TaskStatus snapshot for each robot."""

from __future__ import annotations

import copy
import logging
import threading
import time
from typing import Any, Dict, Optional
import diagnostic_logging as diagnostics

LOGGER = logging.getLogger("opendelivery.tasks")


_lock = threading.Lock()
_latest: Dict[str, Dict[str, Any]] = {}


def set_status(robot_id: str, status: Dict[str, Any]) -> None:
    rid = str(robot_id or "").strip()
    if not rid or not isinstance(status, dict):
        return
    payload = copy.deepcopy(status)
    payload["robot_id"] = rid
    payload["updated_at"] = time.time()
    with _lock:
        previous = _latest.get(rid) or {}
        _latest[rid] = payload
    if (previous.get("task_id"), previous.get("task_status")) != (payload.get("task_id"), payload.get("task_status")):
        status = str(payload.get("task_status") or "")
        diagnostics.event(LOGGER, logging.ERROR if status.lower() == "failed" else logging.WARNING if status.lower() == "terminated" else logging.INFO,
                          "ros.task_status_changed", robot_id=rid, task_id=payload.get("task_id"), task_status=status,
                          previous_task_id=previous.get("task_id"), previous_status=previous.get("task_status"))


def get_status(robot_id: str) -> Optional[Dict[str, Any]]:
    rid = str(robot_id or "").strip()
    with _lock:
        value = _latest.get(rid)
        return copy.deepcopy(value) if value is not None else None


def clear() -> None:
    with _lock:
        _latest.clear()
