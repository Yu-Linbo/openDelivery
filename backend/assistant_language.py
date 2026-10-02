"""Per-turn assistant text, independent of the dashboard's display language."""

import re


EN = {
    "accepted": "Received. Executing the requested steps.",
    "confirmation": "Please explicitly request the operation you want to execute.",
    "failed": "Execution failed: ",
    "completed": "Operation completed.",
    "startup": "{robot_id} simulation is online; status: {status}",
    "shutdown": "Robot simulation is offline.",
    "navigation_sent": "Navigation task submitted.",
    "arrived": "Arrived at the destination.",
    "roundtrip": "Pickup and return completed: {point}",
    "no_task": "{robot_id} has no running task.",
    "stopped": "Task stopped.",
    "query": "Query completed.",
    "online": "{count} robots online{names}",
    "floors": "{count} maps available.",
    "waypoints": "{count} waypoints available.",
    "points": "{count} map points{names}",
    "navigation_timeout": "Timed out waiting for navigation to finish{status}",
    "navigation_failed": "Navigation ended with status: {status}",
    "startup_timeout": "Timed out waiting for {robot_id} to come online{status}",
    "job_timeout": "Task status polling timed out; the task may still be running.",
    "empty_reply": "OpenClaw returned no content.",
    "request_failed": "Request failed: ",
    "point_missing": "The selected map point does not exist: {point}",
    "point_ambiguous": "The map point type is ambiguous: {point}{names}",
    "pose_missing": "The current pose of {robot_id} is unavailable.",
    "task_missing": "Navigation did not provide a task id.",
    "truncated": "…result truncated",
}
ZH = {
    "accepted": "收到，正在执行中。",
    "confirmation": "请明确说明要执行的操作",
    "failed": "执行失败：",
    "completed": "操作已完成",
    "startup": "{robot_id} 仿真已上线，状态 {status}",
    "shutdown": "机器人仿真已下线",
    "navigation_sent": "导航任务已下发",
    "arrived": "已到达目标点",
    "roundtrip": "取货并返回已完成：{point}",
    "no_task": "{robot_id} 当前没有运行中的任务",
    "stopped": "任务已停止",
    "query": "查询成功",
    "online": "在线 {count} 台{names}",
    "floors": "共 {count} 个地图",
    "waypoints": "共 {count} 个点位",
    "points": "共 {count} 个地图点位{names}",
    "navigation_timeout": "等待导航完成超时{status}",
    "navigation_failed": "导航任务状态：{status}",
    "startup_timeout": "{robot_id} 上线超时{status}",
    "job_timeout": "任务状态查询超时，任务可能仍在运行。",
    "empty_reply": "OpenClaw 未返回内容。",
    "request_failed": "请求失败：",
    "point_missing": "模型选择的点位不存在：{point}",
    "point_ambiguous": "点位类型不唯一：{point}{names}",
    "pose_missing": "未获取到 {robot_id} 的当前位置",
    "task_missing": "导航未返回任务编号",
    "truncated": "…结果已截断",
}


def response_texts(message, parsed=None):
    parsed = parsed or {}
    language = str(parsed.get("language") or "").lower()
    if not language:
        language = "zh" if re.search(r"[\u3400-\u9fff]", str(message)) else "en"
    texts = dict(ZH if language.startswith("zh") else EN)
    # The model supplies translations for languages beyond the built-in English
    # and Chinese templates. Only known keys and valid placeholders are accepted.
    overrides = parsed.get("status_text")
    if isinstance(overrides, dict) and not language.startswith(("zh", "en")):
        for key, value in overrides.items():
            if key not in EN or not isinstance(value, str) or not value.strip() or len(value) > 300:
                continue
            try:
                value.format(robot_id="robot1", status="ready", point="point", count=1, names="")
            except (KeyError, ValueError, IndexError, AttributeError):
                continue
            texts[key] = value
    return language, texts
