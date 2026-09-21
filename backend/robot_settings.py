#!/usr/bin/env python3
"""Persistent, per-robot navigation settings used by the Web console."""

import json
import math
import os
import re
import shlex
import tempfile
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Optional


ROBOT_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")
DEFAULT_SETTINGS = {
    "max_linear_speed": 0.18,
    "max_angular_speed": 0.6,
    "inflation_radius": 0.55,
}
SETTING_LIMITS = {
    "max_linear_speed": (0.05, 2.0),
    "max_angular_speed": (0.1, 3.0),
    # The OP1 footprint radius is 0.22 m. Smaller inflation values are unsafe.
    "inflation_radius": (0.22, 3.0),
}


def validate_robot_id(value: Any) -> str:
    robot_id = str(value or "").strip()
    if not ROBOT_ID_RE.fullmatch(robot_id):
        raise ValueError("invalid robot_id")
    return robot_id


def normalize_settings(payload: Any, *, base: Optional[Dict[str, Any]] = None) -> dict:
    if not isinstance(payload, dict):
        raise ValueError("settings must be an object")
    merged = dict(DEFAULT_SETTINGS)
    if isinstance(base, dict):
        for name in DEFAULT_SETTINGS:
            if name in base:
                merged[name] = base[name]
    aliases = {
        "maxSpeed": "max_linear_speed",
        "angularSpeed": "max_angular_speed",
        "safetyDistance": "inflation_radius",
    }
    for raw_name, value in payload.items():
        name = aliases.get(raw_name, raw_name)
        if name in DEFAULT_SETTINGS:
            merged[name] = value
    normalized = {}
    for name, default in DEFAULT_SETTINGS.items():
        raw = merged.get(name, default)
        try:
            value = float(raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be numeric") from exc
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        minimum, maximum = SETTING_LIMITS[name]
        if not minimum <= value <= maximum:
            raise ValueError(f"{name} must be between {minimum:g} and {maximum:g}")
        normalized[name] = round(value, 6)
    return normalized


class RobotSettingsStore:
    """Atomic JSON store whose top-level key is the physical robot ID."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._lock = threading.RLock()

    def _read_unlocked(self) -> dict:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (FileNotFoundError, OSError, ValueError):
            return {"version": 1, "robots": {}}
        robots = raw.get("robots") if isinstance(raw, dict) else None
        return {
            "version": 1,
            "robots": robots if isinstance(robots, dict) else {},
        }

    def _write_unlocked(self, data: dict) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=str(self.path.parent),
                prefix=f".{self.path.name}.",
                suffix=".tmp",
                delete=False,
            ) as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
                temp_path = Path(stream.name)
            os.replace(temp_path, self.path)
        finally:
            if temp_path is not None and temp_path.exists():
                try:
                    temp_path.unlink()
                except OSError:
                    pass

    def get(self, robot_id: str) -> dict:
        rid = validate_robot_id(robot_id)
        with self._lock:
            entry = self._read_unlocked()["robots"].get(rid)
        entry = entry if isinstance(entry, dict) else {}
        saved = isinstance(entry.get("settings"), dict)
        settings = normalize_settings({}, base=entry.get("settings"))
        return {
            "robot_id": rid,
            "settings": settings,
            "source": "saved" if saved else "defaults",
            "updated_at": str(entry.get("updated_at") or ""),
            "last_apply": entry.get("last_apply") if isinstance(entry.get("last_apply"), dict) else None,
        }

    def save(self, robot_id: str, payload: dict) -> dict:
        rid = validate_robot_id(robot_id)
        with self._lock:
            data = self._read_unlocked()
            old_entry = data["robots"].get(rid)
            old_entry = old_entry if isinstance(old_entry, dict) else {}
            settings = normalize_settings(payload, base=old_entry.get("settings"))
            entry = {
                "settings": settings,
                "updated_at": datetime.now(timezone.utc).isoformat(),
            }
            if isinstance(old_entry.get("last_apply"), dict):
                entry["last_apply"] = old_entry["last_apply"]
            data["robots"][rid] = entry
            self._write_unlocked(data)
        return self.get(rid)

    def record_apply(self, robot_id: str, result: dict) -> dict:
        rid = validate_robot_id(robot_id)
        with self._lock:
            data = self._read_unlocked()
            entry = data["robots"].get(rid)
            entry = entry if isinstance(entry, dict) else {"settings": dict(DEFAULT_SETTINGS)}
            entry["last_apply"] = {
                "state": str(result.get("state") or "unknown"),
                "message": str(result.get("message") or ""),
                "applied_at": datetime.now(timezone.utc).isoformat(),
            }
            data["robots"][rid] = entry
            self._write_unlocked(data)
        return self.get(rid)


def ros_parameter_updates(robot_id: str, settings: dict) -> list:
    rid = validate_robot_id(robot_id)
    values = normalize_settings(settings)
    controller = f"/{rid}/navigation/controller_server"
    local_costmap = f"/{rid}/navigation/local_costmap/local_costmap"
    global_costmap = f"/{rid}/navigation/global_costmap/global_costmap"
    recoveries = f"/{rid}/navigation/recoveries_server"
    return [
        (controller, "FollowPath.max_vel_x", values["max_linear_speed"]),
        (controller, "FollowPath.max_speed_xy", values["max_linear_speed"]),
        (controller, "FollowPath.max_vel_theta", values["max_angular_speed"]),
        (recoveries, "max_rotational_vel", values["max_angular_speed"]),
        (local_costmap, "inflation_layer.inflation_radius", values["inflation_radius"]),
        (global_costmap, "inflation_layer.inflation_radius", values["inflation_radius"]),
    ]


def apply_runtime(
    robot_id: str,
    settings: dict,
    run_command: Callable[[str], Any],
) -> dict:
    """Apply settings to live Nav2 nodes and report every target explicitly."""
    applied = []
    errors = []
    for node, parameter, value in ros_parameter_updates(robot_id, settings):
        command = "ros2 param set {} {} {}".format(
            shlex.quote(node), shlex.quote(parameter), shlex.quote(f"{value:g}")
        )
        try:
            completed = run_command(command)
            if getattr(completed, "returncode", 1) != 0:
                detail = (getattr(completed, "stderr", "") or getattr(completed, "stdout", "") or "command failed").strip()
                errors.append({"node": node, "parameter": parameter, "error": detail[-500:]})
                continue
        except Exception as exc:  # noqa: BLE001 - caller needs per-target diagnostics
            errors.append({"node": node, "parameter": parameter, "error": str(exc)})
            continue
        applied.append({"node": node, "parameter": parameter, "value": value})

    if errors and applied:
        state = "partial"
        message = "部分 ROS 参数已应用；失败项将在下次由本平台启动导航栈时从持久配置加载"
    elif errors:
        state = "pending_restart"
        message = "配置已保存；当前 ROS 导航节点未接受参数，将在下次由本平台启动导航栈时加载"
    else:
        state = "applied"
        message = "配置已保存并应用到当前机器人导航栈"
    return {"state": state, "message": message, "applied": applied, "errors": errors}
