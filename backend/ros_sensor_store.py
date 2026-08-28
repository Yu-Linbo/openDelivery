"""Thread-safe cache for LaserScan / Path converted for web (map frame)."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Dict, Optional, Tuple

_lock = threading.Lock()
_scans: Dict[str, Dict[str, Any]] = {}
_paths: Dict[str, Dict[str, Any]] = {}
_gazebo_models: Dict[str, Any] = {}
_topdown_image: Dict[str, Any] = {}
_topdown_seq = 0


def set_scan(robot_id: str, payload: Dict[str, Any]) -> None:
    with _lock:
        _scans[str(robot_id)] = {**payload, "_cached_at": time.time()}


def get_scan(robot_id: str) -> Optional[Dict[str, Any]]:
    with _lock:
        data = _scans.get(str(robot_id))
        return json.loads(json.dumps(data)) if data else None


def get_scans() -> Dict[str, Dict[str, Any]]:
    """Return a thread-safe snapshot of every cached LaserScan payload."""
    with _lock:
        return json.loads(json.dumps(_scans))


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
        _gazebo_models.clear()
        _topdown_image.clear()


def clear_gazebo_models() -> None:
    with _lock:
        _gazebo_models.clear()


def set_gazebo_models(payload: Dict[str, Any]) -> None:
    with _lock:
        _gazebo_models.clear()
        _gazebo_models.update({**payload, "_cached_at": time.time()})


def get_gazebo_models() -> Optional[Dict[str, Any]]:
    with _lock:
        if not _gazebo_models:
            return None
        return json.loads(json.dumps(_gazebo_models))


def set_topdown_image(payload: Dict[str, Any]) -> None:
    """Store the latest frame; immutable byte buffers are shared by reference."""
    global _topdown_seq
    with _lock:
        _topdown_seq += 1
        _topdown_image.clear()
        _topdown_image.update(
            {**payload, "frame_seq": _topdown_seq, "_cached_at": time.time()}
        )


def get_topdown_image() -> Optional[Dict[str, Any]]:
    """Return a shallow snapshot; cached frame byte buffers are immutable."""
    with _lock:
        if not _topdown_image:
            return None
        return dict(_topdown_image)


def get_topdown_jpeg() -> Optional[Tuple[bytes, Dict[str, Any]]]:
    with _lock:
        jpeg = _topdown_image.get("jpeg_bytes")
        if not jpeg:
            return None
        meta = {
            k: v for k, v in _topdown_image.items() if not k.endswith("_bytes")
        }
    return bytes(jpeg), augment_topdown_for_api(meta)


def augment_topdown_for_api(frame: Dict[str, Any]) -> Dict[str, Any]:
    """Attach ``received_at`` / ``age_received_sec`` / ``stale_tier``; strip ``_cached_at``."""
    out = dict(frame)
    received = float(out.pop("_cached_at", 0) or 0)
    now = time.time()
    age = max(0.0, now - received) if received else None
    out["received_at"] = received
    out["age_received_sec"] = round(age, 3) if age is not None else None
    out["stale_tier"] = _topdown_stale_tier(age)
    return out


def _topdown_stale_tier(age_sec: Optional[float]) -> int:
    """0: fresh <10s, 1: aging 10–30s, 2: stale >=30s (wall time since last frame)."""
    if age_sec is None:
        return 0
    if age_sec >= 30.0:
        return 2
    if age_sec >= 10.0:
        return 1
    return 0


def get_topdown_image_status() -> Dict[str, Any]:
    """Return metadata without copying the RGB/JPEG frame byte buffers."""
    with _lock:
        if not _topdown_image:
            return {"available": False, "reason": "no topdown camera frame yet"}
        snap = {
            k: v for k, v in _topdown_image.items() if not k.endswith("_bytes")
        }
    received = float(snap.pop("_cached_at", 0) or 0)
    now = time.time()
    age = max(0.0, now - received) if received else None
    tier = _topdown_stale_tier(age)
    return {
        "available": True,
        "width": snap.get("width"),
        "height": snap.get("height"),
        "encoding": snap.get("encoding"),
        "frame_id": snap.get("frame_id"),
        "stamp_sec": snap.get("stamp_sec"),
        "stamp_nanosec": snap.get("stamp_nanosec"),
        "frame_seq": snap.get("frame_seq"),
        "jpeg_size": snap.get("jpeg_size"),
        "received_at": received,
        "age_received_sec": round(age, 3) if age is not None else None,
        "stale_tier": tier,
    }
