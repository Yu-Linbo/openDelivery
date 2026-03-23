"""Thread-safe cache for LaserScan / Path converted for web (map frame)."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional

_lock = threading.Lock()
_scans: Dict[str, Dict[str, Any]] = {}
_paths: Dict[str, Dict[str, Any]] = {}


def set_scan(robot_id: str, payload: Dict[str, Any]) -> None:
    with _lock:
        _scans[str(robot_id)] = {**payload, "_cached_at": time.time()}


def get_scan(robot_id: str) -> Optional[Dict[str, Any]]:
    with _lock:
        data = _scans.get(str(robot_id))
        return json.loads(json.dumps(data)) if data else None


def set_planned_path(robot_id: str, payload: Dict[str, Any]) -> None:
    with _lock:
        _paths[str(robot_id)] = {**payload, "_cached_at": time.time()}


def get_planned_path(robot_id: str) -> Optional[Dict[str, Any]]:
    with _lock:
        data = _paths.get(str(robot_id))
        return json.loads(json.dumps(data)) if data else None


def clear_all() -> None:
    with _lock:
        _scans.clear()
        _paths.clear()
