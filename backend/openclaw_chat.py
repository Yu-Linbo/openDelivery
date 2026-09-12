"""Small, validated bridge between the web API and the OpenClaw CLI."""

import json
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


SESSION_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{7,95}")
MAX_MESSAGE_CHARS = 4000
MAX_CONTEXT_CHARS = 6000
_JOBS = {}
_JOBS_LOCK = threading.Lock()


SYSTEM_INSTRUCTIONS = """You are the assistant embedded in the OpenDelivery robot management page.
Reply in concise Chinese and lead with the conclusion. Unless the user explicitly asks for raw data, JSON,
or a complete response, never include raw API payloads or long field listings. You do not have HTTP tools.
Return ONLY JSON:
{{"reply":"short response","actions":[{{"name":"action_name","arguments":{{}}}}]}}.
The backend validates and executes actions. Available actions: robot_status {{}}, robot_pose {{}}, floors {{}},
locations {{}}, robot_detail {{"robot_id":"robot1"}}, waypoints {{"robot_id":"robot1"}}, ros_nodes {{}},
ros_threads {{}}, startup_sim {{"robot_id":"robot1"}}, shutdown_sim {{"robot_id":"robot1"}},
stop_task {{"robot_id":"robot1"}}.
For navigation use navigate_to_point {{"robot_id":"robot1","floor_id":"test_102","point":"elevator_inside"}}.
To list map places use map_points {{"floor_id":"test_101"}}. For a pickup round trip use
pickup_and_return {{"robot_id":"robot1","floor_id":"test_101","point":"前台取货点"}}.
“进梯点” means elevator_inside, “候梯点” means elevator_waiting, and “2楼” means test_102. When the user
requests a floor but gives no destination on that floor, choose that floor's elevator_waiting point.
Use no action for ordinary conversation. Never claim success before the backend returns a result. A command
like “上线 robot1” or “仿真上线 robot1” explicitly confirms startup_sim. Treat “继续/继续吧/开始任务/开始执行”
as explicit confirmation of the remaining actions already discussed; return those actions again instead of asking.
Interpret a plain “停止/取消/终止”
as stop_task, never as shutdown_sim. Choose shutdown_sim only for explicit “关闭仿真”, “仿真下线”,
“停止仿真”, or “下线机器人”.
For named places such as “前台取货点”, use map_points or navigate_to_point; robot waypoints are a separate
temporary list and must not be used to decide that a map place does not exist. Point names support natural
language interpretation by you. Use pickup_and_return {{"robot_id":"robot1","floor_id":"test_101",
"point":"前台取货点"}} for requests like “去1楼取货后回来”; it captures the robot's current pose as the
return destination. If there is exactly one online/context robot, use it without asking. Only ask the user
when point matching is ambiguous or no candidate exists. Choose only from the real point catalog below; use
the selected point's exact id or exact name in actions. Decide semantic similarity yourself; the backend does
not perform fuzzy matching. For a multi-step request, return every ordered action in one response (up to three);
the backend runs them sequentially and startup_sim waits for online before the next action. Current map point catalog: {point_catalog}
Current browser context: {page_context}\nUser request: {message}"""

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
    with urlopen(Request(url, method="GET"), timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def _load_map_point_catalog(api_port: int):
    """Expose real candidates to the model; keep coordinates server-side."""
    try:
        floors_payload = _read_json(f"http://127.0.0.1:{api_port}/api/floors", timeout=5)
    except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, ValueError):
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
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, ValueError):
            continue
        for point in (assets.get("points") or [])[:100]:
            if isinstance(point, dict):
                catalog.append({
                    "floor_id": floor_id,
                    "id": str(point.get("id") or ""),
                    "name": str(point.get("name") or ""),
                    "type": str(point.get("type") or ""),
                })
    return catalog[:500]


def _resolve_map_point(floor_id: str, query: str, api_port: int):
    assets = _read_json(f"http://127.0.0.1:{api_port}/api/maps/{quote(floor_id, safe='')}/assets")
    points = [item for item in (assets.get("points") or []) if isinstance(item, dict)]
    wanted = str(query or "").strip()
    if not wanted:
        raise ValueError("point is required")
    exact = [p for p in points if wanted in {str(p.get("id") or ""), str(p.get("name") or "")}]
    if len(exact) == 1:
        return exact[0]
    typed = [p for p in points if wanted == str(p.get("type") or "")]
    if len(typed) == 1:
        return typed[0]
    if len(typed) > 1:
        names = "、".join(str(point.get("name") or point.get("id")) for point in typed[:5])
        raise ValueError(f"点位类型“{query}”不唯一：{names}")
    raise ValueError(f"模型选择的点位不存在：{floor_id}/{query}")


def _current_robot_pose(robot_id: str, api_port: int):
    payload = _read_json(f"http://127.0.0.1:{api_port}/api/robot/pose")
    for row in payload.get("robots") or []:
        if str(row.get("id") or row.get("robot_id") or "") == robot_id:
            pose = row.get("pose") or {}
            return {
                "floor_id": str(row.get("active_floor") or ""),
                "x": float(pose["x"]), "y": float(pose["y"]), "yaw": float(pose.get("yaw", 0)),
            }
    raise ValueError(f"未获取到 {robot_id} 的当前位置")


def _post_navigation(robot_id: str, destination: dict, api_port: int):
    body = {"robot_id": robot_id, **{key: destination[key] for key in ("x", "y", "yaw", "floor_id")}}
    request = Request(
        f"http://127.0.0.1:{api_port}/api/robot/motion/goto",
        data=json.dumps(body).encode("utf-8"), headers={"Content-Type": "application/json"}, method="POST",
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
    return {"reply": str(parsed.get("reply") or "").strip(), "actions": actions if isinstance(actions, list) else []}


def _confirmed(message: str, name: str) -> bool:
    value = message.lower()
    if any(term in value for term in (
        "开始执行", "开始任务", "确认执行", "按计划执行", "继续执行", "继续吧",
        "执行吧", "现在执行", "proceed", "continue", "execute now",
    )):
        return True
    terms = {
        "startup_sim": ("仿真上线", "上线", "启动仿真", "启动机器人", "start simulation", "bringup"),
        "shutdown_sim": ("仿真下线", "关闭仿真", "停止仿真", "下线机器人", "shutdown simulation"),
        "navigate_to_point": ("前往", "导航", "发任务", "到", "去", "goto"),
        "pickup_and_return": ("取货后回来", "取货再回来", "取货并返回", "取货后返回", "取货"),
        "stop_task": ("停止", "取消任务", "终止任务", "停下", "stop task", "cancel task"),
    }
    return any(term in value for term in terms.get(name, ()))


def _execute_action(action: dict, message: str, api_port: int):
    name = str(action.get("name") or "")
    if name not in ACTION_SPECS:
        raise ValueError(f"unsupported action: {name}")
    method, path_template, mutating = ACTION_SPECS[name]
    arguments = action.get("arguments") if isinstance(action.get("arguments"), dict) else {}
    robot_id = str(arguments.get("robot_id") or "").strip()
    if "{robot_id}" in path_template or mutating:
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", robot_id):
            raise ValueError("invalid robot_id")
    if mutating and not _confirmed(message, name):
        return {"name": name, "ok": False, "confirmation_required": True, "summary": "请明确说明要执行的操作"}
    if name == "stop_task":
        detail = _read_json(
            f"http://127.0.0.1:{api_port}/api/robot/{quote(robot_id, safe='')}/detail"
        )
        task = detail.get("task") if isinstance(detail, dict) else None
        task_id = str((task or {}).get("task_id") or "").strip()
        if not task_id:
            return {"name": name, "ok": True, "summary": f"{robot_id} 当前没有运行中的任务"}
        request = Request(
            f"http://127.0.0.1:{api_port}/api/robot/command",
            data=json.dumps({
                "type": "task_command", "robot_id": robot_id,
                "task_id": task_id, "command": "terminate",
            }).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST",
        )
        with urlopen(request, timeout=15) as response:
            json.loads(response.read().decode("utf-8"))
        return {"name": name, "ok": True, "summary": f"{robot_id} 任务已停止"}
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
        point = _resolve_map_point(floor_id, point_name, api_port)
        arguments = {**arguments, **point, "floor_id": floor_id}
        if name == "pickup_and_return":
            return_pose = _current_robot_pose(robot_id, api_port)
            outbound = _post_navigation(robot_id, arguments, api_port)
            outbound_task_id = str(outbound.get("task_id") or "")
            if not outbound_task_id:
                raise RuntimeError("取货导航未返回任务编号")
            _wait_for_navigation_terminal(robot_id, outbound_task_id, api_port)
            result = _post_navigation(robot_id, return_pose, api_port)
            result["point_name"] = point.get("name") or point_name
            return {"name": name, "ok": True, "summary": _summarize_action_result(name, result)}
    path = path_template.format(robot_id=quote(robot_id, safe=""), floor_id=quote(str(arguments.get("floor_id") or ""), safe=""))
    data = None
    headers = {}
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
        status = _wait_for_robot_online(robot_id, api_port)
        result = {"robot_id": robot_id, "status": status}
    summary = _summarize_action_result(name, result)
    wants_raw = any(term in message.lower() for term in ("原始数据", "原始响应", "完整响应", "raw data", "json"))
    output = {"name": name, "ok": True, "summary": summary}
    if name == "navigate_to_point" and isinstance(result, dict) and result.get("task_id"):
        output["task_id"] = str(result["task_id"])
    if wants_raw:
        output["result"] = result
    return output


def _wait_for_navigation_terminal(
    robot_id: str, task_id: str, api_port: int, timeout_s: float = 900.0,
):
    deadline = time.monotonic() + timeout_s
    seen = False
    last_status = ""
    while time.monotonic() < deadline:
        try:
            detail = _read_json(
                f"http://127.0.0.1:{api_port}/api/robot/{quote(robot_id, safe='')}/detail",
                timeout=5,
            )
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, ValueError):
            time.sleep(2)
            continue
        task = detail.get("task") if isinstance(detail, dict) else None
        if isinstance(task, dict) and str(task.get("task_id") or "") == task_id:
            seen = True
            last_status = str(task.get("task_status") or "")
            if last_status == "Finished":
                return last_status
            if last_status in ("Failed", "Terminated"):
                raise RuntimeError(f"导航任务{last_status}")
        time.sleep(2)
    reason = f"，最后状态 {last_status}" if seen and last_status else ""
    raise RuntimeError(f"等待导航完成超时{reason}")


def _wait_for_robot_online(robot_id: str, api_port: int, timeout_s: float = 120.0):
    deadline = time.monotonic() + timeout_s
    last_status = ""
    while time.monotonic() < deadline:
        request = Request(f"http://127.0.0.1:{api_port}/api/robot/status/cache", method="GET")
        try:
            with urlopen(request, timeout=5) as response:
                rows = json.loads(response.read().decode("utf-8"))
        except (HTTPError, URLError, TimeoutError, json.JSONDecodeError):
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
            if row.get("online") is True:
                return last_status or "online"
        time.sleep(2)
    raise RuntimeError(f"{robot_id} 上线超时" + (f"，最后状态 {last_status}" if last_status else ""))


def _summarize_action_result(name: str, result: Any) -> str:
    if name == "startup_sim":
        rid = result.get("robot_id") or "机器人"
        status = result.get("status")
        return f"{rid} 仿真已上线" + (f"，状态 {status}" if status else "")
    if name == "shutdown_sim":
        return "机器人仿真已下线"
    if name == "navigate_to_point":
        return "导航任务已下发"
    if name == "pickup_and_return":
        return f"已下发前往{result.get('point_name') or '取货点'}并返回的任务"
    if name == "stop_task":
        return "任务已停止"
    if name == "robot_status" and isinstance(result, list):
        online = [row.get("robot_id") or row.get("id") for row in result if isinstance(row, dict) and row.get("online")]
        names = ", ".join(str(item) for item in online if item)
        return f"在线 {len(online)} 台" + (f"：{names}" if names else "")
    if name == "floors" and isinstance(result, dict):
        return f"共 {len(result.get('floors') or [])} 个地图"
    if name == "waypoints":
        rows = result.get("waypoints", result) if isinstance(result, dict) else result
        return f"共 {len(rows) if isinstance(rows, list) else 0} 个点位"
    if name == "map_points":
        rows = result.get("points") if isinstance(result, dict) else []
        names = "、".join(str(row.get("name") or row.get("id")) for row in rows[:8] if isinstance(row, dict))
        return f"共 {len(rows)} 个地图点位" + (f"：{names}" if names else "")
    return "查询成功"


def _run_action_job(job_id: str, actions: list, message: str, api_port: int):
    results = []
    with _JOBS_LOCK:
        _JOBS[job_id]["status"] = "running"
    for action in actions:
        if not isinstance(action, dict):
            continue
        try:
            result = _execute_action(action, message, api_port)
        except (ValueError, RuntimeError) as err:
            result = {"name": str(action.get("name") or "unknown"), "ok": False, "error": str(err)}
        results.append(result)
        with _JOBS_LOCK:
            _JOBS[job_id]["results"] = list(results)
        if not result.get("ok"):
            break
        if str(action.get("name") or "") == "navigate_to_point":
            try:
                _wait_for_navigation_terminal(
                    str((action.get("arguments") or {}).get("robot_id") or ""),
                    str(result.get("task_id") or ""), api_port,
                )
                result["summary"] = "已到达目标点"
            except RuntimeError as err:
                result = {**result, "ok": False, "error": str(err)}
            results[-1] = result
            with _JOBS_LOCK:
                _JOBS[job_id]["results"] = list(results)
            if not result.get("ok"):
                break
    with _JOBS_LOCK:
        job = _JOBS[job_id]
        job["status"] = "completed" if results and all(item.get("ok") for item in results) else "failed"
        job["updated_at"] = time.time()


def get_action_job(job_id: str):
    if not re.fullmatch(r"[a-f0-9]{32}", str(job_id or "")):
        return None
    with _JOBS_LOCK:
        job = _JOBS.get(job_id)
        return dict(job) if job else None


def run_chat(
    message: str, session_id: str, page_context: Dict[str, Any], *,
    timeout_s: float = 120.0, defer_mutations: bool = False,
):
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
    context_json = json.dumps(page_context, ensure_ascii=False, separators=(",", ":"))
    api_port = int(os.environ.get("MAP_API_PORT", "8001"))
    point_catalog = json.dumps(_load_map_point_catalog(api_port), ensure_ascii=False, separators=(",", ":"))
    prompt = SYSTEM_INSTRUCTIONS.format(
        page_context=context_json[:MAX_CONTEXT_CHARS],
        point_catalog=point_catalog[:MAX_CONTEXT_CHARS],
        message=message,
    )
    command = [
        executable, "agent", "--json", "--agent", os.environ.get("OPENCLAW_AGENT_ID", "main"),
        "--timeout", str(max(10, min(int(timeout_s), 300))), "--message", prompt,
    ]
    command_env = os.environ.copy()
    command_env.setdefault("OPENCLAW_ALLOW_INSECURE_PRIVATE_WS", "1")
    try:
        proc = subprocess.run(command, capture_output=True, text=True, timeout=timeout_s + 5, env=command_env)
    except subprocess.TimeoutExpired as err:
        raise TimeoutError("OpenClaw response timed out") from err
    if proc.returncode != 0:
        detail = (proc.stderr or proc.stdout or "OpenClaw request failed").strip().splitlines()[-1]
        raise RuntimeError(detail[:500])
    try:
        payload = json.loads(proc.stdout)
    except json.JSONDecodeError as err:
        raise RuntimeError("OpenClaw returned invalid JSON") from err
    raw_reply = _extract_text(payload)
    if not raw_reply:
        raise RuntimeError("OpenClaw returned an empty reply")
    parsed = _parse_agent_reply(raw_reply)
    actions = [action for action in parsed["actions"][:3] if isinstance(action, dict)]
    if defer_mutations and any(ACTION_SPECS.get(str(action.get("name") or ""), ("", "", False))[2] for action in actions):
        job_id = uuid.uuid4().hex
        with _JOBS_LOCK:
            _JOBS[job_id] = {
                "job_id": job_id, "status": "queued", "results": [],
                "created_at": time.time(), "updated_at": time.time(),
            }
        threading.Thread(
            target=_run_action_job, args=(job_id, actions, message, api_port),
            daemon=True, name=f"openclaw-action-{job_id[:8]}",
        ).start()
        return {
            "reply": parsed["reply"] or "收到，正在执行中。",
            "actions": [], "job_id": job_id, "job_status": "queued", "session_id": session_id,
        }
    results = []
    for action in actions:
        try:
            results.append(_execute_action(action, message, api_port))
        except (ValueError, RuntimeError) as err:
            results.append({"name": str(action.get("name") or "unknown"), "ok": False, "error": str(err)})
    return {"reply": parsed["reply"] or raw_reply, "actions": results, "session_id": session_id}
