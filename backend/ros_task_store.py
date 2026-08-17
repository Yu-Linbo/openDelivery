"""Thread-safe latest root TaskStatus snapshot for each robot."""

from __future__ import annotations

import copy
import threading
import time
from typing import Any, Dict, Optional


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
        _latest[rid] = payload


def get_status(robot_id: str) -> Optional[Dict[str, Any]]:
    rid = str(robot_id or "").strip()
    with _lock:
        value = _latest.get(rid)
        return copy.deepcopy(value) if value is not None else None


def clear() -> None:
    with _lock:
        _latest.clear()
