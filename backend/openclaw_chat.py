"""Small, validated bridge between the web API and the OpenClaw CLI."""

import json
import copy
import contextvars
import logging
import os
import re
import shutil
import subprocess
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen
from assistant_language import EN, ZH, response_texts
import diagnostic_logging as diagnostics


SESSION_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{7,95}")
MAX_MESSAGE_CHARS = 4000
_JOBS = {}
_JOBS_LOCK = threading.Lock()
_JOB_PROGRESS = contextvars.ContextVar("assistant_job_progress", default=None)
LOGGER = logging.getLogger("opendelivery.assistant")
MAX_ACTIONS = 8


SYSTEM_INSTRUCTIONS = """Plan for the embedded OpenDelivery assistant. Return ONLY JSON:
{{"reply":"concise reply","language":"BCP-47","decision":"execute|query|clarify|chat",
"actions":[{{"name":"action","arguments":{{}},"include_result":false}}]}}.
Reply in THIS request's language, independent of history/dashboard. Interpret intent semantically:
execute for requested operations or confirmation of a discussed plan; query for reads; clarify for
essential ambiguity; chat otherwise. Negated operations and explanations do not authorize execution.
Clarify/chat have no actions; query has only reads; execute has a complete ordered plan (max 8).
The backend validates and executes actions, without keyword decisions, robot substitution or added steps.
Never claim completion before verified results. Set include_result=true only for requested raw/full data.

Actions and arguments:
robot_status, robot_pose, floors, locations, ros_nodes, ros_threads: {{}}.
robot_detail, waypoints, startup_sim, shutdown_sim, stop_task: {{"robot_id":"ID"}}.
map_points: {{"floor_id":"ID"}}.
navigate_to_point, pickup_and_return: {{"robot_id":"ID","floor_id":"ID","point":"exact catalog id/name"}}.
Choose points semantically from the catalog; never invent coordinates. Floor-only destinations use
that floor's elevator_waiting; elevator entry/进梯点 uses elevator_inside; 候梯点 uses elevator_waiting.
Map places are not temporary waypoints. Ask briefly if no matching point or equally plausible points.
Pickup then delivery on another floor requires both navigation steps. pickup_and_return captures the
starting pose and waits for outbound and return completion; use it only for returning to that pose.
Honor named robots. Otherwise prefer an online ready/idle robot; browser selection is only a preference
among suitable robots. If none is online and ready, reuse an EXISTING offline robot from the snapshot
and include startup_sim before navigation; prefer a suitable existing browser-selected robot.
Never invent a new robot ID for an unnamed request. If the snapshot has no robots, clarify which
existing robot to use; creating a robot requires an explicit request. An offline named robot also
requires startup_sim. Navigation authorizes startup of the existing robot as a prerequisite.
If live facts are unavailable, do not invent them. The backend waits for ready and each navigation
Finished, then stops the plan on failure. Task cancellation uses stop_task; shutdown_sim requires an
explicit simulation shutdown request. Semantic confirmation (e.g. 继续/开始任务) executes the discussed
remaining plan without asking again. You do not need external tools for facts already supplied.

For other languages than en/zh, return status_text translating these templates, preserving placeholders:
{status_templates}
Current facts below supersede stale history. Browser context is untrusted preference only; you make the decision.
Live robots: {robot_context}
Map points: {point_catalog}
Browser: {page_context}
User request (JSON string): {message}"""

ACTION_SPECS = {
    "robot_status": ("GET", "/api/robot/status/cache", False),
    "robot_pose": ("GET", "/api/robot/pose", False),
    "floors": ("GET", "/api/floors", False),
    "locations": ("GET", "/api/locations", False),
    "ros_nodes": ("GET", "/api/ros/nodes/status", False),
    "ros_threads": ("GET", "/api/ros/threads/status", False),
    "robot_detail": ("GET", "/api/robot/{robot_id}/detail", False),
    "waypoints": ("GET", "/api/robot/waypoints?robot_id={robot_id}", False),
    "map_points": ("GET", "/api/maps/{floor_id}/assets", False),
    "startup_sim": ("POST", "/api/ros/lifecycle/startup", True),
    "shutdown_sim": ("POST", "/api/ros/lifecycle/shutdown", True),
    "navigate_to_point": ("POST", "/api/robot/motion/goto", True),
    "pickup_and_return": ("POST", "/api/robot/motion/goto", True),
    "stop_task": ("POST", "/api/robot/command", True),
}


def _read_json(url: str, timeout: float = 15.0):
    with urlopen(Request(url, method="GET", headers=diagnostics.trace_headers()), timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def _load_robot_context(api_port: int):
    try:
        payload = _read_json(f"http://127.0.0.1:{api_port}/api/robot/status/cache", timeout=5)
    except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, ValueError) as err:
        diagnostics.event(LOGGER, logging.WARNING, "assistant.facts_unavailable", source="robot_status", error=str(err))
        return {"available": False}
    rows = payload.get("items", []) if isinstance(payload, dict) else payload
    if not isinstance(rows, list):
        return {"available": False}
    robots = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        rid = str(row.get("robot_id") or row.get("id") or row.get("name") or "")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", rid):
            continue
        robots.append({"id": rid, "online": row.get("online") is True,
                       "status": row.get("live_robot_status") or row.get("robot_status") or row.get("persisted_robot_status") or "",
                       "task_status": row.get("live_task_status") or row.get("persisted_task_status") or ""})
    return {"available": True, "robots": robots}


def _validate_actions(actions: list, decision: str):
    if decision not in ("execute", "query", "clarify", "chat"):
        raise ValueError("invalid AI decision")
    if len(actions) > MAX_ACTIONS:
        raise ValueError(f"A plan may contain at most {MAX_ACTIONS} actions")
    if actions and decision in ("clarify", "chat"):
        raise ValueError("AI clarification/chat must not include actions")
    if decision == "execute" and not actions:
        raise ValueError("AI execution plan must include actions")
    validated = []
    for action in actions:
        if not isinstance(action, dict) or not isinstance(action.get("arguments"), dict):
            raise ValueError("AI action and arguments must be objects")
        item = {"name": str(action.get("name") or ""),
                "arguments": dict(action["arguments"])}
        spec = ACTION_SPECS.get(item["name"])
        if not spec:
            raise ValueError(f"unsupported action: {item['name']}")
        if spec[2] and decision != "execute":
            raise ValueError("AI query must not include mutating actions")
        if spec[2] or "{robot_id}" in spec[1]:
            if not isinstance(item["arguments"].get("robot_id"), str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", item["arguments"]["robot_id"]):
                raise ValueError("invalid robot_id in AI action")
        if item["name"] in ("map_points", "navigate_to_point", "pickup_and_return"):
            if not isinstance(item["arguments"].get("floor_id"), str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", item["arguments"]["floor_id"]):
                raise ValueError("invalid floor_id in AI action")
        if item["name"] in ("navigate_to_point", "pickup_and_return"):
            point = item["arguments"].get("point")
            if not isinstance(point, str) or not point.strip() or len(point) > 120:
                raise ValueError("invalid point in AI action")
        include_result = action.get("include_result", False)
        if not isinstance(include_result, bool):
            raise ValueError("include_result must be a boolean")
        if include_result:
            item["include_result"] = True
        validated.append(item)
    return validated


def _load_map_point_catalog(api_port: int):
    """Expose real candidates to the model; keep coordinates server-side."""
    try:
        floors_payload = _read_json(f"http://127.0.0.1:{api_port}/api/floors", timeout=5)
    except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, ValueError) as err:
        diagnostics.event(LOGGER, logging.WARNING, "assistant.facts_unavailable", source="floors", error=str(err))
        return []
    catalog = []
    for floor_id in (floors_payload.get("floors") or [])[:32]:
        floor_id = str(floor_id or "").strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", floor_id):
            continue
        try:
            assets = _read_json(
                f"http://127.0.0.1:{api_port}/api/maps/{quote(floor_id, safe='')}/assets",
                timeout=5,
            )
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, ValueError) as err:
            diagnostics.event(LOGGER, logging.WARNING, "assistant.facts_unavailable", source="map_points", floor_id=floor_id, error=str(err))
            continue
        for point in (assets.get("points") or [])[:100]:
            if isinstance(point, dict):
                catalog.append({
                    "floor_id": floor_id,
                    "id": str(point.get("id") or ""),
                    "name": str(point.get("name") or ""),
                    **{key: point[key] for key in ("name_zh", "name_en") if point.get(key)},
                    "type": str(point.get("type") or ""),
                })
    return catalog[:500]


def _resolve_map_point(floor_id: str, query: str, api_port: int, *, texts=None):
    texts = texts or ZH
    assets = _read_json(f"http://127.0.0.1:{api_port}/api/maps/{quote(floor_id, safe='')}/assets")
    points = [item for item in (assets.get("points") or []) if isinstance(item, dict)]
    wanted = str(query or "").strip()
    if not wanted:
        raise ValueError("point is required")
    exact = [p for p in points if wanted in {str(p.get("id") or ""), str(p.get("name") or ""), str(p.get("name_zh") or ""), str(p.get("name_en") or "")}]
    if len(exact) == 1:
        return exact[0]
    typed = [p for p in points if wanted == str(p.get("type") or "")]
    if len(typed) == 1:
        return typed[0]
    if len(typed) > 1:
        names = "、".join(str(point.get("name") or point.get("id")) for point in typed[:5])
        raise ValueError(texts["point_ambiguous"].format(point=query, names=f": {names}"))
    raise ValueError(texts["point_missing"].format(point=f"{floor_id}/{query}"))


def _current_robot_pose(robot_id: str, api_port: int, *, texts=None):
    texts = texts or ZH
    payload = _read_json(f"http://127.0.0.1:{api_port}/api/robot/pose")
    for row in payload.get("robots") or []:
        if str(row.get("id") or row.get("robot_id") or "") == robot_id:
            pose = row.get("pose") or {}
            return {
                "floor_id": str(row.get("active_floor") or ""),
                "x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(pose.get("yaw", 0)),
            }
    raise ValueError(texts["pose_missing"].format(robot_id=robot_id))


def _post_navigation(robot_id: str, destination: dict, api_port: int):
    body = {"robot_id": robot_id, **{key: destination[key] for key in ("x", "y", "yaw", "floor_id")}}
    request = Request(
        f"http://127.0.0.1:{api_port}/api/robot/motion/goto",
        data=json.dumps(body).encode("utf-8"), headers={"Content-Type": "application/json", **diagnostics.trace_headers()}, method="POST",
    )
    with urlopen(request, timeout=125) as response:
        return json.loads(response.read().decode("utf-8"))


def _extract_text(payload: Any) -> str:
    """Accept OpenClaw's current JSON shape and a few older compatible shapes."""
    if isinstance(payload, str):
        return payload.strip()
    if isinstance(payload, list):
        parts = [_extract_text(item) for item in payload]
        return "\n".join(part for part in parts if part).strip()
    if not isinstance(payload, dict):
        return ""
    if isinstance(payload.get("reply"), str) and isinstance(payload.get("actions"), list):
        return json.dumps(payload, ensure_ascii=False)
    for key in ("text", "reply", "message", "content"):
        text = _extract_text(payload.get(key))
        if text:
            return text
    result = payload.get("result")
    if isinstance(result, dict):
        text = _extract_text(result.get("payloads"))
        if text:
            return text
    # Compatibility with wrappers that nest payloads under another key.
    for value in payload.values():
        if value is result:
            continue
        text = _extract_text(value)
        if text:
            return text
    return ""


def _parse_agent_reply(text: str):
    candidate = text.strip()
    if candidate.startswith("```"):
        candidate = re.sub(r"^```(?:json)?\s*|\s*```$", "", candidate, flags=re.I)
    try:
        parsed = json.loads(candidate)
    except json.JSONDecodeError:
        return {"reply": text, "actions": []}
    if not isinstance(parsed, dict):
        return {"reply": text, "actions": []}
    actions = parsed.get("actions")
    if actions is not None and not isinstance(actions, list):
        raise ValueError("AI actions must be an array")
    return {"reply": str(parsed.get("reply") or "").strip(), "actions": actions or [],
            "decision": parsed.get("decision"),
            "language": parsed.get("language"), "status_text": parsed.get("status_text")}


def _execute_action(action: dict, api_port: int):
    texts = action.get("_texts") or ZH
    name = str(action.get("name") or "")
    if name not in ACTION_SPECS:
        raise ValueError(f"unsupported action: {name}")
    method, path_template, mutating = ACTION_SPECS[name]
    arguments = action.get("arguments") if isinstance(action.get("arguments"), dict) else {}
    robot_id = str(arguments.get("robot_id") or "").strip()
    if "{robot_id}" in path_template or mutating:
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", robot_id):
            raise ValueError("invalid robot_id")
    if name == "stop_task":
        detail = _read_json(
            f"http://127.0.0.1:{api_port}/api/robot/{quote(robot_id, safe='')}/detail"
        )
        task = detail.get("task") if isinstance(detail, dict) else None
        task_id = str((task or {}).get("task_id") or "").strip()
        if not task_id:
            return {"name": name, "ok": True, "summary": texts["no_task"].format(robot_id=robot_id)}
        request = Request(
            f"http://127.0.0.1:{api_port}/api/robot/command",
            data=json.dumps({
                "type": "task_command", "robot_id": robot_id,
                "task_id": task_id, "command": "terminate",
            }).encode("utf-8"),
            headers={"Content-Type": "application/json", **diagnostics.trace_headers()}, method="POST",
        )
        with urlopen(request, timeout=15) as response:
            json.loads(response.read().decode("utf-8"))
        return {"name": name, "ok": True, "summary": f"{robot_id} " + texts["stopped"]}
    if name == "map_points":
        floor_id = str(arguments.get("floor_id") or "").strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", floor_id):
            raise ValueError("invalid floor_id")
        path_template = "/api/maps/{floor_id}/assets"
    if name in ("navigate_to_point", "pickup_and_return"):
        floor_id = str(arguments.get("floor_id") or "").strip()
        point_name = str(arguments.get("point") or "").strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", floor_id):
            raise ValueError("invalid floor_id")
        point = _resolve_map_point(floor_id, point_name, api_port, texts=texts)
        arguments = {**arguments, **point, "floor_id": floor_id}
        if name == "pickup_and_return":
            return_pose = _current_robot_pose(robot_id, api_port, texts=texts)
            _emit_progress(texts["pickup_outbound"].format(robot_id=robot_id))
            outbound = _post_navigation(robot_id, arguments, api_port)
            outbound_task_id = str(outbound.get("task_id") or "")
            if not outbound_task_id:
                raise RuntimeError(texts["task_missing"])
            _wait_for_navigation_terminal(robot_id, outbound_task_id, api_port, texts=texts)
            _emit_progress(texts["pickup_return"].format(robot_id=robot_id))
            result = _post_navigation(robot_id, return_pose, api_port)
            return_task_id = str(result.get("task_id") or "")
            if not return_task_id:
                raise RuntimeError(texts["task_missing"])
            _wait_for_navigation_terminal(robot_id, return_task_id, api_port, texts=texts)
            result["point_name"] = point.get("name") or point_name
            return {"name": name, "ok": True, "summary": _summarize_action_result(name, result, texts)}
    path = path_template.format(robot_id=quote(robot_id, safe=""), floor_id=quote(str(arguments.get("floor_id") or ""), safe=""))
    data = None
    headers = diagnostics.trace_headers()
    if method == "POST":
        body = {"robot_id": robot_id}
        if name == "startup_sim":
            body["sim_mode"] = "sim"
        if name == "navigate_to_point":
            body.update({key: arguments[key] for key in ("x", "y", "yaw", "floor_id")})
        data = json.dumps(body).encode("utf-8")
        headers["Content-Type"] = "application/json"
    request = Request(f"http://127.0.0.1:{api_port}{path}", data=data, headers=headers, method=method)
    try:
        with urlopen(request, timeout=125 if mutating else 15) as response:
            result = json.loads(response.read().decode("utf-8"))
    except HTTPError as err:
        detail = err.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"{name} API HTTP {err.code}: {detail[:300]}") from err
    except (URLError, TimeoutError) as err:
        raise RuntimeError(f"{name} API unavailable: {err}") from err
    if name == "startup_sim":
        status = _wait_for_robot_online(robot_id, api_port, texts=texts)
        result = {"robot_id": robot_id, "status": status}
    summary = _summarize_action_result(name, result, texts)
    output = {"name": name, "ok": True, "summary": summary}
    if name == "navigate_to_point":
        if not isinstance(result, dict) or not result.get("task_id"):
            raise RuntimeError(texts["task_missing"])
        output["task_id"] = str(result["task_id"])
    if action.get("include_result"):
        output["result"] = result
    return output


def _wait_for_navigation_terminal(
    robot_id: str, task_id: str, api_port: int, timeout_s: float = 900.0,
    *, texts=None,
):
    texts = texts or ZH
    started = time.monotonic()
    deadline = started + timeout_s
    seen = False
    last_status = ""
    failures = 0
    reported = None
    next_report = started
    diagnostics.event(LOGGER, logging.INFO, "assistant.navigation_wait_started", robot_id=robot_id, task_id=task_id, timeout_s=timeout_s)
    while time.monotonic() < deadline:
        try:
            detail = _read_json(
                f"http://127.0.0.1:{api_port}/api/robot/{quote(robot_id, safe='')}/detail",
                timeout=5,
            )
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, ValueError) as err:
            failures += 1
            now = time.monotonic()
            if failures == 1 or now >= next_report:
                diagnostics.event(LOGGER, logging.WARNING, "assistant.navigation_poll_failed", robot_id=robot_id,
                                  task_id=task_id, poll_failures=failures, error=str(err))
                next_report = now + 30
            time.sleep(2)
            continue
        task = detail.get("task") if isinstance(detail, dict) else None
        state = (str((task or {}).get("task_id") or ""), str((task or {}).get("task_status") or "")) if isinstance(task, dict) else ("", "")
        now = time.monotonic()
        if state != reported or now >= next_report:
            diagnostics.event(LOGGER, logging.INFO, "assistant.navigation_observed", robot_id=robot_id, task_id=task_id,
                              observed_task_id=state[0], task_status=state[1], duration_ms=round((now-started)*1000), poll_failures=failures)
            reported, next_report = state, now + 30
        if isinstance(task, dict) and str(task.get("task_id") or "") == task_id:
            seen = True
            last_status = str(task.get("task_status") or "")
            _navigation_progress(robot_id, task, detail, texts)
            if last_status == "Finished":
                return last_status
            if last_status in ("Failed", "Terminated"):
                raise RuntimeError(texts["navigation_failed"].format(status=last_status))
        time.sleep(2)
    reason = f" ({last_status})" if seen and last_status else ""
    diagnostics.event(LOGGER, logging.ERROR, "assistant.navigation_timed_out", robot_id=robot_id, task_id=task_id,
                      task_seen=seen, task_status=last_status, poll_failures=failures, timeout_s=timeout_s)
    raise RuntimeError(texts["navigation_timeout"].format(status=reason))


def _wait_for_robot_online(robot_id: str, api_port: int, timeout_s: float = 120.0, *, texts=None):
    texts = texts or ZH
    started = time.monotonic()
    deadline = started + timeout_s
    last_status = ""
    failures = 0
    reported = None
    next_report = started
    diagnostics.event(LOGGER, logging.INFO, "assistant.robot_wait_started", robot_id=robot_id, timeout_s=timeout_s)
    while time.monotonic() < deadline:
        request = Request(f"http://127.0.0.1:{api_port}/api/robot/status/cache", method="GET", headers=diagnostics.trace_headers())
        try:
            with urlopen(request, timeout=5) as response:
                rows = json.loads(response.read().decode("utf-8"))
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError) as err:
            failures += 1
            now = time.monotonic()
            if failures == 1 or now >= next_report:
                diagnostics.event(LOGGER, logging.WARNING, "assistant.robot_poll_failed", robot_id=robot_id, poll_failures=failures, error=str(err))
                next_report = now + 30
            rows = []
        # Production wraps presence rows in {"items": [...]}; continue to
        # accept the historical bare-list shape for backward compatibility.
        status_rows = rows.get("items") if isinstance(rows, dict) else rows
        for row in status_rows if isinstance(status_rows, list) else []:
            if not isinstance(row, dict):
                continue
            rid = str(row.get("robot_id") or row.get("id") or row.get("name") or "")
            if rid != robot_id:
                continue
            last_status = str(
                row.get("robot_status") or row.get("live_robot_status")
                or row.get("persisted_robot_status") or row.get("status") or ""
            )
            state = (row.get("online") is True, last_status)
            now = time.monotonic()
            if state != reported or now >= next_report:
                diagnostics.event(LOGGER, logging.INFO, "assistant.robot_observed", robot_id=robot_id,
                                  online=state[0], robot_status=last_status, duration_ms=round((now-started)*1000), poll_failures=failures)
                reported, next_report = state, now + 30
            phase = last_status if row.get("online") is True else "offline"
            _emit_progress(texts["startup_progress"].format(robot_id=robot_id, phase=texts.get("phase_" + phase, phase)))
            if row.get("online") is True and last_status.lower() in ("", "ready", "idle", "online"):
                return last_status or "online"
        time.sleep(2)
    diagnostics.event(LOGGER, logging.ERROR, "assistant.robot_wait_timed_out", robot_id=robot_id,
                      robot_status=last_status, poll_failures=failures, timeout_s=timeout_s)
    raise RuntimeError(texts["startup_timeout"].format(robot_id=robot_id, status=f" ({last_status})" if last_status else ""))


def _summarize_action_result(name: str, result: Any, texts=None) -> str:
    texts = texts or ZH
    if name == "startup_sim":
        rid = result.get("robot_id") or "机器人"
        status = result.get("status")
        return texts["startup"].format(robot_id=rid, status=status or "online")
    if name in ("shutdown_sim", "navigate_to_point", "stop_task"):
        return texts[{"shutdown_sim": "shutdown", "navigate_to_point": "navigation_sent", "stop_task": "stopped"}[name]]
    if name == "pickup_and_return":
        return texts["roundtrip"].format(point=result.get("point_name") or "pickup point")
    if name == "robot_status" and isinstance(result, (list, dict)):
        rows = result.get("items", []) if isinstance(result, dict) else result
        online = [row.get("robot_id") or row.get("id") for row in rows if isinstance(row, dict) and row.get("online")]
        names = ", ".join(str(item) for item in online if item)
        return texts["online"].format(count=len(online), names=f": {names}" if names else "")
    if name == "floors" and isinstance(result, dict):
        return texts["floors"].format(count=len(result.get("floors") or []))
    if name == "waypoints":
        rows = result.get("waypoints", result) if isinstance(result, dict) else result
        return texts["waypoints"].format(count=len(rows) if isinstance(rows, list) else 0)
    if name == "map_points":
        rows = result.get("points") if isinstance(result, dict) else []
        names = "、".join(str(row.get("name") or row.get("id")) for row in rows[:8] if isinstance(row, dict))
        return texts["points"].format(count=len(rows), names=f": {names}" if names else "")
    return texts["query"]


def _emit_progress(message):
    callback = _JOB_PROGRESS.get()
    if callback:
        callback(message)


def _navigation_progress(robot_id, task, detail, texts):
    total = max(1, int(task.get("total_count") or len(task.get("work_queue") or []) or 1))
    current = max(0, int(task.get("current_index") or 0))
    phase = str(task.get("task_status") or "Waiting")
    queue = task.get("work_queue") or []
    if phase == "Navigating" and current < len(queue):
        parts = str(queue[current]).split(":")
        if len(parts) > 1 and parts[0] == "navigation":
            phase = parts[1]
    floor = str((detail.get("status") or {}).get("floor") or "")
    _emit_progress(texts["progress"].format(robot_id=robot_id, floor=" · " + floor if floor else "",
                   index=min(current + 1, total), total=total, phase=texts.get("phase_" + phase, phase)))


def _publish_job_event(job_id, message, kind="progress", *, status=None):
    with _JOBS_LOCK:
        job = _JOBS[job_id]
        events = job.setdefault("events", [])
        if events and events[-1]["message"] == message and events[-1]["kind"] == kind:
            return
        seq = job.get("event_seq", 0) + 1
        job["event_seq"] = seq
        job["updated_at"] = time.time()
        events.append({"seq": seq, "message": message, "kind": kind, "created_at": job["updated_at"]})
        job["events"] = events[-200:]
        if kind == "terminal":
            job["terminal_text"] = message
            if status:
                job["status"] = status
        conversation = job.get("conversation")
    if conversation:
        from assistant_sessions import STORE
        try:
            STORE.append(*conversation, "assistant", message)
        except Exception:
            diagnostics.event(LOGGER, logging.ERROR, "assistant.job_history_write_failed", exc_info=True)


def _run_action_job(job_id: str, actions: list, api_port: int):
    token = _JOB_PROGRESS.set(lambda message: _publish_job_event(job_id, message))
    try:
        with diagnostics.context(job_id=job_id, stage="execution"):
            _run_action_job_logged(job_id, actions, api_port)
    finally:
        _JOB_PROGRESS.reset(token)


def _run_action_job_logged(job_id: str, actions: list, api_port: int):
    started = time.monotonic()
    results = []
    with _JOBS_LOCK:
        _JOBS[job_id]["status"] = "running"
        texts = _JOBS[job_id].get("status_text") or ZH
    diagnostics.event(LOGGER, logging.INFO, "assistant.job_started", action_count=len(actions))
    for index, action in enumerate(actions, 1):
        if not isinstance(action, dict):
            continue
        arguments = action.get("arguments") or {}
        with diagnostics.context(step=index, action=action.get("name"), robot_id=arguments.get("robot_id"),
                                 floor_id=arguments.get("floor_id"), point=arguments.get("point")):
            action_started = time.monotonic()
            _emit_progress(texts["step"].format(index=index, total=len(actions),
                           action=texts.get("action_" + action.get("name", ""), texts["action_query"]),
                           robot_id=arguments.get("robot_id") or ""))
            diagnostics.event(LOGGER, logging.INFO, "assistant.action_started")
            try:
                result = _execute_action({**action, "_texts": texts}, api_port)
            except Exception as err:
                result = {"name": str(action.get("name") or "unknown"), "ok": False, "error": str(err)}
                diagnostics.event(LOGGER, logging.ERROR, "assistant.action_exception", error_type=type(err).__name__,
                                  error=str(err), exc_info=not isinstance(err, (ValueError, RuntimeError, HTTPError, URLError, TimeoutError)))
            diagnostics.event(LOGGER, logging.INFO if result.get("ok") else logging.ERROR, "assistant.action_result",
                              ok=result.get("ok"), task_id=result.get("task_id"), error=result.get("error"),
                              duration_ms=round((time.monotonic()-action_started)*1000))
            results.append(result)
            with _JOBS_LOCK:
                _JOBS[job_id]["results"] = list(results)
            if not result.get("ok"):
                break
            if str(action.get("name") or "") == "navigate_to_point":
                try:
                    _wait_for_navigation_terminal(str(arguments.get("robot_id") or ""),
                                                  str(result.get("task_id") or ""), api_port, texts=texts)
                    result["summary"] = texts["arrived"]
                except Exception as err:
                    result = {**result, "ok": False, "error": str(err)}
                    diagnostics.event(LOGGER, logging.ERROR, "assistant.navigation_failed", task_id=result.get("task_id"),
                                      error_type=type(err).__name__, error=str(err), exc_info=not isinstance(err, RuntimeError))
                results[-1] = result
                with _JOBS_LOCK:
                    _JOBS[job_id]["results"] = list(results)
                if not result.get("ok"):
                    break
            _emit_progress(result.get("summary") or texts["completed"])
            diagnostics.event(LOGGER, logging.INFO, "assistant.action_completed", task_id=result.get("task_id"),
                              duration_ms=round((time.monotonic()-action_started)*1000))
    status = "completed" if results and all(item.get("ok") for item in results) else "failed"
    terminal = texts["plan_completed"].format(count=len(results)) if status == "completed" else (
        texts["failed"] + str(next((item.get("error") for item in results if not item.get("ok")), "unknown error")))
    _publish_job_event(job_id, terminal, "terminal", status=status)
    diagnostics.event(LOGGER, logging.INFO if status == "completed" else logging.ERROR, "assistant.job_finished", status=status,
                      completed_actions=sum(bool(item.get("ok")) for item in results), planned_actions=len(actions),
                      duration_ms=round((time.monotonic()-started)*1000),
                      error=next((item.get("error") for item in results if not item.get("ok")), None))


def get_action_job(job_id: str):
    if not re.fullmatch(r"[a-f0-9]{32}", str(job_id or "")):
        return None
    with _JOBS_LOCK:
        job = _JOBS.get(job_id)
        return copy.deepcopy({key: value for key, value in job.items() if key != "conversation"}) if job else None


def run_chat(
    message: str, session_id: str, page_context: Dict[str, Any], *,
    timeout_s: float = 120.0, defer_mutations: bool = False,
    isolated_session: bool = False, conversation=None,
):
    started = time.monotonic()
    with diagnostics.context(session_id=str(session_id or "")[:96], stage="validation"):
        diagnostics.event(LOGGER, logging.INFO, "assistant.request_started", message_chars=len(str(message or "")),
                          isolated_session=isolated_session)
        try:
            result = _run_chat(message, session_id, page_context, timeout_s=timeout_s, defer_mutations=defer_mutations,
                               isolated_session=isolated_session, conversation=conversation)
        except Exception as err:
            diagnostics.event(LOGGER, logging.WARNING if isinstance(err, ValueError) else logging.ERROR, "assistant.request_failed",
                              error_type=type(err).__name__, error=str(err), duration_ms=round((time.monotonic()-started)*1000),
                              exc_info=not isinstance(err, (ValueError, RuntimeError, TimeoutError)))
            raise
        diagnostics.event(LOGGER, logging.INFO, "assistant.request_completed", decision=result.get("decision"),
                          job_id=result.get("job_id"), duration_ms=round((time.monotonic()-started)*1000))
        return result


def _run_chat(message, session_id, page_context, *, timeout_s, defer_mutations, isolated_session, conversation):
    message = str(message or "").strip()
    session_id = str(session_id or "").strip()
    if not message:
        raise ValueError("message is required")
    if len(message) > MAX_MESSAGE_CHARS:
        raise ValueError(f"message must be at most {MAX_MESSAGE_CHARS} chars")
    if not SESSION_RE.fullmatch(session_id):
        raise ValueError("invalid session_id")
    if not isinstance(page_context, dict):
        raise ValueError("context must be an object")

    configured_bin = os.environ.get("OPENCLAW_BIN")
    user_bin = Path.home() / ".npm-global" / "bin" / "openclaw"
    executable = configured_bin or shutil.which("openclaw") or (str(user_bin) if user_bin.is_file() else None)
    if not executable:
        raise RuntimeError("OpenClaw CLI is not installed")
    # Keep only preferences that help planning. Browser URLs, field dumps and
    # claimed online state are neither authoritative nor useful model context.
    context = {key: str(page_context[key])[:120] for key in ("view", "floor", "robot_id")
               if isinstance(page_context.get(key), str) and page_context[key]}
    context_json = json.dumps(context, ensure_ascii=False, separators=(",", ":"))
    api_port = int(os.environ.get("MAP_API_PORT", "8001"))
    diagnostics.update_context(stage="facts")
    robot_context = _load_robot_context(api_port)
    point_catalog = json.dumps(_load_map_point_catalog(api_port), ensure_ascii=False, separators=(",", ":"))
    prompt = SYSTEM_INSTRUCTIONS.format(
        page_context=context_json,
        point_catalog=point_catalog,
        robot_context=json.dumps(robot_context, ensure_ascii=False, separators=(",", ":")),
        status_templates=json.dumps(EN, ensure_ascii=False, separators=(",", ":")),
        message=json.dumps(message, ensure_ascii=False),
    )
    command = [
        executable, "agent", "--json", "--agent", os.environ.get("OPENCLAW_AGENT_ID", "main"),
        "--thinking", os.environ.get("OPENCLAW_THINKING", "low"),
        "--timeout", str(max(10, min(int(timeout_s), 300))), "--message", prompt,
    ]
    # Use the Gateway's verified default unless a deployment explicitly pins a
    # supported model. Do not leave an unavailable target hardcoded in the bridge.
    if os.environ.get("OPENCLAW_MODEL"):
        command[2:2] = ["--model", os.environ["OPENCLAW_MODEL"]]
    if isolated_session:
        command[2:2] = ["--session-key", f"agent:{os.environ.get('OPENCLAW_AGENT_ID', 'main')}:{session_id}"]
    command_env = os.environ.copy()
    command_env.setdefault("OPENCLAW_ALLOW_INSECURE_PRIVATE_WS", "1")
    diagnostics.update_context(stage="model")
    model_started = time.monotonic()
    diagnostics.event(LOGGER, logging.INFO, "assistant.model_started", agent=os.environ.get("OPENCLAW_AGENT_ID", "main"),
                      thinking=os.environ.get("OPENCLAW_THINKING", "low"), timeout_s=timeout_s, prompt_chars=len(prompt),
                      robots_available=robot_context.get("available"))
    try:
        proc = subprocess.run(command, capture_output=True, text=True, timeout=timeout_s + 5, env=command_env)
    except subprocess.TimeoutExpired as err:
        raise TimeoutError("OpenClaw response timed out") from err
    if proc.returncode != 0:
        detail = ((proc.stderr or proc.stdout or "").strip() or "OpenClaw request failed").splitlines()[-1]
        diagnostics.event(LOGGER, logging.ERROR, "assistant.model_failed", return_code=proc.returncode,
                          error=detail, duration_ms=round((time.monotonic()-model_started)*1000))
        raise RuntimeError(detail[:500])
    diagnostics.update_context(stage="plan_validation")
    try:
        payload = json.loads(proc.stdout)
    except json.JSONDecodeError as err:
        raise RuntimeError("OpenClaw returned invalid JSON") from err
    raw_reply = _extract_text(payload)
    if not raw_reply:
        raise RuntimeError("OpenClaw returned an empty reply")
    parsed = _parse_agent_reply(raw_reply)
    language, texts = response_texts(message, parsed)
    decision = parsed.get("decision") or ("chat" if not parsed["actions"] else None)
    actions = _validate_actions(parsed["actions"], decision)
    robot_id = next((action["arguments"]["robot_id"] for action in actions if action["arguments"].get("robot_id")), None)
    metadata = {"language": language, "status_text": texts, "robot_id": robot_id, "decision": decision}
    envelope = payload.get("result", payload) if isinstance(payload, dict) else {}
    meta = envelope.get("meta", {}) if isinstance(envelope, dict) else {}
    agent_meta = meta.get("agentMeta", {}) if isinstance(meta, dict) else {}
    usage = agent_meta.get("usage", {})
    diagnostics.event(LOGGER, logging.INFO, "assistant.plan_validated", decision=decision, language=language,
                      robot_id=robot_id, actions=[action["name"] for action in actions], model=agent_meta.get("model"),
                      prompt_tokens=agent_meta.get("promptTokens"), input_tokens=usage.get("input"), cache_read=usage.get("cacheRead"),
                      output_tokens=usage.get("output"), duration_ms=round((time.monotonic()-model_started)*1000))
    if defer_mutations and any(ACTION_SPECS.get(str(action.get("name") or ""), ("", "", False))[2] for action in actions):
        job_id = uuid.uuid4().hex
        with _JOBS_LOCK:
            _JOBS[job_id] = {
                "job_id": job_id, "status": "queued", "results": [], "events": [], "event_seq": 0,
                "created_at": time.time(), "updated_at": time.time(),
                "conversation": conversation,
                **metadata,
            }
        if conversation:
            from assistant_sessions import STORE
            STORE.append(*conversation, "assistant", parsed["reply"] or texts["accepted"])
        diagnostics.event(LOGGER, logging.INFO, "assistant.job_queued", job_id=job_id, action_count=len(actions))
        log_context = contextvars.copy_context()
        threading.Thread(
            target=lambda *args: log_context.run(_run_action_job, *args), args=(job_id, actions, api_port),
            daemon=True, name=f"openclaw-action-{job_id[:8]}",
        ).start()
        return {
            "reply": parsed["reply"] or texts["accepted"],
            "actions": [], "job_id": job_id, "job_status": "queued", "session_id": session_id,
            **metadata,
        }
    results = []
    diagnostics.update_context(stage="execution")
    for index, action in enumerate(actions, 1):
        with diagnostics.context(step=index, action=action["name"]):
            try:
                result = _execute_action({**action, "_texts": texts}, api_port)
            except (ValueError, RuntimeError, HTTPError, URLError, TimeoutError, json.JSONDecodeError) as err:
                result = {"name": str(action.get("name") or "unknown"), "ok": False, "error": str(err)}
            results.append(result)
            diagnostics.event(LOGGER, logging.INFO if result.get("ok") else logging.ERROR, "assistant.action_result",
                              ok=result.get("ok"), error=result.get("error"))
    return {"reply": parsed["reply"] or raw_reply, "actions": results, "session_id": session_id, **metadata}
