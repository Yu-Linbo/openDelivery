"""Pure helpers for Web-to-ROS localization commands."""

from __future__ import annotations

from typing import Any, Dict, Iterable


def resolve_map_name(
    requested: Any,
    robot_id: str,
    live_status: Dict[str, Any],
    persisted_status: Dict[str, Any],
    robot_specs: Iterable[Dict[str, Any]],
) -> str:
    """Resolve an explicit or pose-only localization map deterministically."""
    name = str(requested or "").strip()
    if name:
        return name
    for status in (live_status, persisted_status):
        name = str((status or {}).get("current_map") or "").strip()
        if name:
            return name
    rid = str(robot_id or "").strip()
    for spec in robot_specs:
        if str(spec.get("id") or "").strip() == rid:
            return str(spec.get("current_map") or "").strip()
    return ""
