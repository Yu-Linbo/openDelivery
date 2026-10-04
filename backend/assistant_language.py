"""Per-turn assistant text, independent of the dashboard's display language."""

import re


EN = {
    "step": "Step {index}/{total}: {action} — {robot_id}",
    "action_startup_sim": "Starting simulation",
    "action_navigate_to_point": "Navigating to the destination",
    "action_pickup_and_return": "Picking up and returning",
    "action_shutdown_sim": "Stopping simulation",
    "action_stop_task": "Stopping the task",
    "action_query": "Reading status",
    "progress": "{robot_id}{floor} · Subtask {index}/{total}: {phase}",
    "startup_progress": "{robot_id}: {phase}",
    "phase_initializing": "Initializing",
    "phase_localizing": "Localizing",
    "phase_localization_lost": "Waiting for localization",
    "phase_ready": "Ready",
    "phase_offline": "Waiting to come online",
    "phase_Waiting": "Waiting",
    "phase_Navigating": "Navigating",
    "phase_CallingElevator": "Calling the elevator",
    "phase_RidingElevator": "Riding the elevator",
    "phase_SwitchingMap": "Arriving on the target floor",
    "phase_MovingModel": "Transferring to the target floor",
    "phase_Relocalizing": "Localizing on the target floor",
    "phase_Finished": "Completed",
    "phase_Failed": "Failed",
    "phase_Terminated": "Stopped",
    "phase_elevator_waiting": "Going to the elevator waiting point",
    "phase_elevator_inside": "Entering the elevator",
    "phase_elevator_exit": "Leaving the elevator",
    "phase_goal": "Going to the destination",
    "pickup_outbound": "{robot_id}: Going to the pickup point.",
    "pickup_return": "{robot_id}: Returning to the starting point.",
    "plan_completed": "Task completed. All {count} planned steps finished.",
    "job_reconnecting": "Reconnecting to task progress…",
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
    "step": "步骤 {index}/{total}：{action} — {robot_id}",
    "action_startup_sim": "正在启动仿真",
    "action_navigate_to_point": "正在前往目标点",
    "action_pickup_and_return": "正在取货并返回",
    "action_shutdown_sim": "正在关闭仿真",
    "action_stop_task": "正在停止任务",
    "action_query": "正在读取状态",
    "progress": "{robot_id}{floor} · 子任务 {index}/{total}：{phase}",
    "startup_progress": "{robot_id}：{phase}",
    "phase_initializing": "正在初始化",
    "phase_localizing": "正在定位",
    "phase_localization_lost": "等待恢复定位",
    "phase_ready": "已就绪",
    "phase_offline": "等待上线",
    "phase_Waiting": "等待执行",
    "phase_Navigating": "正在导航",
    "phase_CallingElevator": "正在呼叫电梯",
    "phase_RidingElevator": "正在乘梯",
    "phase_SwitchingMap": "正在到达目标楼层",
    "phase_MovingModel": "正在切换到目标楼层",
    "phase_Relocalizing": "正在目标楼层定位",
    "phase_Finished": "已完成",
    "phase_Failed": "执行失败",
    "phase_Terminated": "已停止",
    "phase_elevator_waiting": "正在前往电梯候梯点",
    "phase_elevator_inside": "正在进入电梯",
    "phase_elevator_exit": "正在离开电梯",
    "phase_goal": "正在前往目标点",
    "pickup_outbound": "{robot_id}：正在前往取货点。",
    "pickup_return": "{robot_id}：正在返回起点。",
    "plan_completed": "任务已完成，计划中的 {count} 个步骤全部执行完毕。",
    "job_reconnecting": "正在重新连接任务进度…",
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
                value.format(robot_id="robot1", status="ready", point="point", count=1, names="", index=1, total=2, action="navigate", floor="floor1", phase="navigating")
            except (KeyError, ValueError, IndexError, AttributeError):
                continue
            texts[key] = value
    return language, texts
