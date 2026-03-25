"""Per-robot cache for nav_msgs/OccupancyGrid from /<robot_id>/mapping (web live mapping)."""

from __future__ import annotations

import threading
from typing import Any, Dict, Optional

_lock = threading.Lock()
_snapshots: Dict[str, Dict[str, Any]] = {}


def set_from_occupancy_grid(
    robot_id: str,
    width: int,
    height: int,
    resolution: float,
    origin_xyz: tuple,
    origin_yaw: float,
    stamp_sec: int,
    stamp_nanosec: int,
    frame_id: str,
    pixels: bytes,
) -> None:
    rid = (robot_id or "").strip()
    if not rid:
        return
    with _lock:
        _snapshots[rid] = {
            "robot_id": rid,
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": [origin_xyz[0], origin_xyz[1], origin_xyz[2]],
            "origin_yaw": origin_yaw,
            "stamp_sec": stamp_sec,
            "stamp_nanosec": stamp_nanosec,
            "frame_id": frame_id,
            "pixels": pixels,
        }


def get_snapshot(robot_id: str) -> Optional[Dict[str, Any]]:
    rid = (robot_id or "").strip()
    if not rid:
        return None
    with _lock:
        snap = _snapshots.get(rid)
        if snap is None:
            return None
        return {
            "robot_id": snap["robot_id"],
            "width": snap["width"],
            "height": snap["height"],
            "resolution": snap["resolution"],
            "origin": list(snap["origin"]),
            "origin_yaw": snap["origin_yaw"],
            "stamp_sec": snap["stamp_sec"],
            "stamp_nanosec": snap["stamp_nanosec"],
            "frame_id": snap["frame_id"],
            "pixels": snap["pixels"],
        }
