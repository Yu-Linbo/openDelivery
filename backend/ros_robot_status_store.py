"""Persistent per-robot last RobotStatus cache for web backend."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_ROOT_DIR = Path(__file__).resolve().parent.parent
_DB_DIR = _ROOT_DIR / "backend" / "data"
_DB_PATH = _DB_DIR / "robot_status_last.json"

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
                "current_map": str(item.get("current_map") or ""),
                "robot_status": str(item.get("robot_status") or ""),
                "topic": str(item.get("topic") or ""),
                "stamp_ns": int(item.get("stamp_ns") or 0),
                "updated_at": float(item.get("updated_at") or 0.0),
            }
        _cache = out


def _flush_locked() -> None:
    try:
        _DB_DIR.mkdir(parents=True, exist_ok=True)
        _DB_PATH.write_text(
            json.dumps(_cache, ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
    except Exception:
        # Keep runtime cache available even if disk write fails.
        return


def set_last_status(
    robot_id: str,
    *,
    robot_name: str,
    current_map: str,
    robot_status: str,
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
            "current_map": str(current_map or ""),
            "robot_status": str(robot_status or ""),
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
