"""Persistent per-robot last RobotStatus cache for web backend."""

from __future__ import annotations

import json
import os
import tempfile
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_ROOT_DIR = Path(__file__).resolve().parent.parent
_DEFAULT_DB_PATH = _ROOT_DIR / "backend" / "data" / "robot_status_last.json"
_DB_PATH = Path(os.environ.get("OPEN_DELIVERY_STATUS_DB_PATH", _DEFAULT_DB_PATH)).expanduser()
_DB_DIR = _DB_PATH.parent

_lock = threading.Lock()
_cache: Dict[str, Dict[str, Any]] = {}
_loaded = False


def _load_once() -> None:
    global _loaded, _cache
    with _lock:
        if _loaded:
            return
        _loaded = True
        if not _DB_PATH.is_file():
            _cache = {}
            return
        try:
            raw = json.loads(_DB_PATH.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            _cache = {}
            return
        if not isinstance(raw, dict):
            _cache = {}
            return
        out: Dict[str, Dict[str, Any]] = {}
        for rid, item in raw.items():
            if not isinstance(item, dict):
                continue
            rid_s = str(rid).strip()
            if not rid_s:
                continue
            out[rid_s] = {
                "robot_id": rid_s,
                "robot_name": str(item.get("robot_name") or rid_s),
                "robot_model": str(item.get("robot_model") or ""),
                "current_map": str(item.get("current_map") or ""),
                "current_position": str(item.get("current_position") or "unknown;"),
                "robot_status": str(item.get("robot_status") or ""),
                "task_status": str(item.get("task_status") or ""),
                "is_simulation": bool(item.get("is_simulation", False)),
                "task_progress": float(item.get("task_progress", -1.0)),
                "topic": str(item.get("topic") or ""),
                "stamp_ns": int(item.get("stamp_ns") or 0),
                "updated_at": float(item.get("updated_at") or 0.0),
            }
        _cache = out


def _flush_locked() -> None:
    """Atomically persist the cache so readers never observe partial JSON."""
    tmp_path: Optional[Path] = None
    try:
        _DB_DIR.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(_cache, ensure_ascii=False, indent=2, sort_keys=True)
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=_DB_DIR,
            prefix=f".{_DB_PATH.name}.", suffix=".tmp", delete=False,
        ) as tmp:
            tmp.write(payload)
            tmp.flush()
            os.fsync(tmp.fileno())
            tmp_path = Path(tmp.name)
        os.replace(tmp_path, _DB_PATH)
    except Exception:
        # Keep runtime cache available even if disk write fails.
        if tmp_path is not None:
            try:
                tmp_path.unlink(missing_ok=True)
            except Exception:
                pass


def set_last_status(
    robot_id: str,
    *,
    robot_name: str,
    robot_model: str = "",
    current_map: str,
    current_position: str = "unknown;",
    robot_status: str,
    task_status: str = "",
    is_simulation: bool = False,
    task_progress: float = -1.0,
    topic: str,
    stamp_ns: int,
) -> None:
    rid = (robot_id or "").strip()
    if not rid:
        return
    _load_once()
    with _lock:
        _cache[rid] = {
            "robot_id": rid,
            "robot_name": str(robot_name or rid),
            "robot_model": str(robot_model or ""),
            "current_map": str(current_map or ""),
            "current_position": str(current_position or "unknown;"),
            "robot_status": str(robot_status or ""),
            "task_status": str(task_status or ""),
            "is_simulation": bool(is_simulation),
            "task_progress": float(task_progress) if task_progress is not None else -1.0,
            "topic": str(topic or ""),
            "stamp_ns": int(stamp_ns or 0),
            "updated_at": time.time(),
        }
        _flush_locked()


def get_last_status(robot_id: str) -> Optional[Dict[str, Any]]:
    rid = (robot_id or "").strip()
    if not rid:
        return None
    _load_once()
    with _lock:
        data = _cache.get(rid)
        if not data:
            return None
        return json.loads(json.dumps(data))


def list_all_last_status() -> List[Dict[str, Any]]:
    _load_once()
    with _lock:
        return [json.loads(json.dumps(v)) for v in _cache.values()]
