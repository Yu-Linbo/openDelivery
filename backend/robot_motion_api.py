"""HTTP-backed robot motion helpers (no direct ros2 from agents)."""

from __future__ import annotations

import json
import math
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_LOCK = threading.Lock()
_WAYPOINTS_PATH: Optional[Path] = None
_TELEOP_PROCESSES: Dict[str, subprocess.Popen] = {}
_TELEOP_LOCK = threading.Lock()
_TELEOP_SEQUENCE: Dict[str, Tuple[str, int]] = {}
_TELEOP_STATE: Dict[str, Tuple[str, float, float]] = {}
_TELEOP_LEASE_DEADLINE: Dict[str, float] = {}
_TELEOP_WATCHDOG: Optional[threading.Thread] = None
_TELEOP_LEASE_SEC = 0.8


class RosCommandTimeoutError(RuntimeError):
    """Raised when a ROS helper process cannot finish within its deadline."""


def _root_dir() -> Path:
    return Path(__file__).resolve().parent.parent


def _waypoints_path() -> Path:
    global _WAYPOINTS_PATH
    if _WAYPOINTS_PATH is None:
        _WAYPOINTS_PATH = Path(__file__).resolve().parent / "data" / "waypoints.json"
    return _WAYPOINTS_PATH


def _ros_run(cmd: str, timeout: float = 120.0) -> Dict[str, Any]:
    root = _root_dir()
    install = root / "install" / "setup.bash"
    distro = (os.environ.get("ROS_DISTRO") or "foxy").strip()
    install_src = f'source "{install}"' if install.is_file() else "true"
    full = (
        f'set -eo pipefail; source "/opt/ros/{distro}/setup.bash"; '
        f'cd "{root}" && {install_src}; {cmd}'
    )
    proc = subprocess.Popen(
        ["bash", "-lc", full],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired as exc:
        # The shell launches ROS/Python children. Killing only the shell leaves
        # those children holding stdout/stderr open, which can wedge the HTTP
        # request indefinitely. Terminate the complete process group instead.
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            stdout, stderr = proc.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            stdout, stderr = proc.communicate()
        detail = (stderr or stdout or "").strip()
        suffix = f": {detail}" if detail else ""
        raise RosCommandTimeoutError(
            f"ROS command timed out after {timeout:.1f}s{suffix}"
        ) from exc

    out = (stdout or "").strip()
    err = (stderr or "").strip()
    if proc.returncode != 0:
        raise RuntimeError(err or out or "ros command failed")
    return {"ok": True, "output": out}


def _yaw_quat(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def publish_cmd_vel_timed(
    robot_id: str,
    linear: float,
    angular: float,
    seconds: float,
    *,
    confirmed: bool,
) -> Dict[str, Any]:
    if not confirmed:
        raise ValueError("velocity command requires user confirmation (confirmed=true)")
    rid = str(robot_id or "").strip()
    if not rid or not re.match(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$", rid):
        raise ValueError("invalid robot_id")
    sec = max(0.1, min(float(seconds), 30.0))
    lin = float(linear)
    ang = float(angular)
    if abs(lin) > 2.0 or abs(ang) > 3.0:
        raise ValueError("linear/angular out of safe range")
    topic = f"/{rid}/cmd_vel"
    msg = (
        f"{{linear: {{x: {lin}, y: 0.0, z: 0.0}}, "
        f"angular: {{x: 0.0, y: 0.0, z: {ang}}}}}"
    )
    rate_hz = 10
    n = max(1, int(sec * rate_hz))
    period = 1.0 / rate_hz
    cmd = (
        f"for i in $(seq 1 {n}); do "
        f"ros2 topic pub -1 {shlex.quote(topic)} geometry_msgs/msg/Twist "
        f"{shlex.quote(msg)}; sleep {period}; done"
    )
    return _ros_run(cmd, timeout=sec + 25.0)


def _validate_velocity(robot_id: str, linear: float, angular: float) -> Tuple[str, float, float]:
    rid = str(robot_id or "").strip()
    if not rid or not re.match(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$", rid):
        raise ValueError("invalid robot_id")
    lin = float(linear)
    ang = float(angular)
    if abs(lin) > 1.2 or abs(ang) > 1.5:
        raise ValueError("teleop linear/angular out of safe range")
    return rid, lin, ang


def _velocity_pub_command(rid: str, linear: float, angular: float, *, once: bool = False) -> str:
    topic = f"/{rid}/cmd_vel"
    msg = (
        f"{{linear: {{x: {linear}, y: 0.0, z: 0.0}}, "
        f"angular: {{x: 0.0, y: 0.0, z: {angular}}}}}"
    )
    mode = "-1" if once else "-r 10"
    return f"ros2 topic pub {mode} {shlex.quote(topic)} geometry_msgs/msg/Twist {shlex.quote(msg)}"


def _start_ros_process(cmd: str) -> subprocess.Popen:
    root = _root_dir()
    install = root / "install" / "setup.bash"
    distro = (os.environ.get("ROS_DISTRO") or "foxy").strip()
    install_src = f'source "{install}"' if install.is_file() else "true"
    full = (
        f'set -eo pipefail; source "/opt/ros/{distro}/setup.bash"; '
        f'cd "{root}" && {install_src}; exec {cmd}'
    )
    return subprocess.Popen(
        ["bash", "-lc", full], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=os.environ.copy(), start_new_session=True,
    )


def _stop_teleop_process(rid: str) -> None:
    proc = _TELEOP_PROCESSES.pop(rid, None)
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=1.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def _teleop_watchdog_loop() -> None:
    while True:
        time.sleep(0.1)
        expired = []
        now = time.monotonic()
        with _TELEOP_LOCK:
            for rid, deadline in list(_TELEOP_LEASE_DEADLINE.items()):
                if now >= deadline:
                    _stop_teleop_process(rid)
                    _TELEOP_LEASE_DEADLINE.pop(rid, None)
                    _TELEOP_STATE.pop(rid, None)
                    expired.append(rid)
        for rid in expired:
            try:
                _ros_run(_velocity_pub_command(rid, 0.0, 0.0, once=True), timeout=8.0)
            except Exception:
                pass


def _ensure_teleop_watchdog() -> None:
    global _TELEOP_WATCHDOG
    if _TELEOP_WATCHDOG is not None and _TELEOP_WATCHDOG.is_alive():
        return
    _TELEOP_WATCHDOG = threading.Thread(
        target=_teleop_watchdog_loop, daemon=True, name="web-teleop-watchdog"
    )
    _TELEOP_WATCHDOG.start()


def set_teleop_velocity(
    robot_id: str, linear: float, angular: float, *, active: bool, confirmed: bool,
    session_id: str = "", sequence: int = 0,
) -> Dict[str, Any]:
    """Maintain exactly one browser teleop publisher per robot; stop always sends zero."""
    if not confirmed:
        raise ValueError("velocity command requires user confirmation (confirmed=true)")
    rid, lin, ang = _validate_velocity(robot_id, linear, angular)
    session = str(session_id or "").strip()
    if not re.match(r"^[A-Za-z0-9_-]{1,80}$", session) or int(sequence) < 1:
        raise ValueError("teleop session_id and positive sequence are required")
    seq = int(sequence)
    with _TELEOP_LOCK:
        previous = _TELEOP_SEQUENCE.get(rid)
        if previous and previous[0] == session and seq <= previous[1]:
            return {"ok": True, "robot_id": rid, "active": bool(active), "stale": True}
        _TELEOP_SEQUENCE[rid] = (session, seq)
        if active and (lin != 0.0 or ang != 0.0):
            state = (session, lin, ang)
            proc = _TELEOP_PROCESSES.get(rid)
            if _TELEOP_STATE.get(rid) != state or proc is None or proc.poll() is not None:
                _stop_teleop_process(rid)
                _TELEOP_PROCESSES[rid] = _start_ros_process(_velocity_pub_command(rid, lin, ang))
                _TELEOP_STATE[rid] = state
            _TELEOP_LEASE_DEADLINE[rid] = time.monotonic() + _TELEOP_LEASE_SEC
            _ensure_teleop_watchdog()
            return {"ok": True, "robot_id": rid, "active": True, "linear": lin, "angular": ang, "lease_sec": _TELEOP_LEASE_SEC}
        _stop_teleop_process(rid)
        _TELEOP_STATE.pop(rid, None)
        _TELEOP_LEASE_DEADLINE.pop(rid, None)
    _ros_run(_velocity_pub_command(rid, 0.0, 0.0, once=True), timeout=8.0)
    return {"ok": True, "robot_id": rid, "active": False, "linear": 0.0, "angular": 0.0}


def stop_all_teleop() -> None:
    """Stop managed teleop publishers during backend shutdown."""
    with _TELEOP_LOCK:
        for rid in list(_TELEOP_PROCESSES):
            _stop_teleop_process(rid)
        _TELEOP_STATE.clear()
        _TELEOP_LEASE_DEADLINE.clear()


def send_navigate_to_pose(
    robot_id: str,
    x: float,
    y: float,
    yaw: float = 0.0,
) -> Dict[str, Any]:
    rid = str(robot_id or "").strip()
    if not rid or not re.match(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$", rid):
        raise ValueError("invalid robot_id")
    z, w = _yaw_quat(float(yaw))
    # Use rclpy action client directly to avoid ros2 CLI yaml parsing bugs
    script = f"""
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from builtins import float as _float

rclpy.init()
node = rclpy.create_node("_goal_sender_{rid}")
client = None
lifecycle = None
try:
    lifecycle = node.create_client(GetState, "/{rid}/navigation/bt_navigator/get_state")
    if not lifecycle.wait_for_service(timeout_sec=3.0):
        raise RuntimeError("navigation lifecycle service not available for {rid}")
    state_future = lifecycle.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, state_future, timeout_sec=3.0)
    if not state_future.done() or state_future.result() is None:
        raise RuntimeError("navigation lifecycle state unavailable for {rid}")
    state = state_future.result().current_state.label
    if state != "active":
        raise RuntimeError("navigation stack for {rid} is not active: " + state)
    node.destroy_client(lifecycle)
    lifecycle = None
    client = ActionClient(node, NavigateToPose, "/{rid}/navigation/navigate_to_pose")
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError("action server not available")
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.pose.position.x = _float({x})
    goal_msg.pose.pose.position.y = _float({y})
    goal_msg.pose.pose.orientation.z = _float({z})
    goal_msg.pose.pose.orientation.w = _float({w})
    send_goal_future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    if not send_goal_future.done():
        raise RuntimeError("action server did not answer the goal request")
    result = send_goal_future.result()
    if not result or not result.accepted:
        raise RuntimeError("navigation goal rejected")
    print("Goal accepted")
finally:
    if client is not None:
        client.destroy()
    if lifecycle is not None:
        node.destroy_client(lifecycle)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
"""
    try:
        return _ros_run(f'python3 -c {shlex.quote(script)}', timeout=25.0)
    except RuntimeError as exc:
        # rclpy writes a Python traceback to stderr; expose only the actual
        # reason to the HTTP/Web caller.
        lines = [line.strip() for line in str(exc).splitlines() if line.strip()]
        detail = next(
            (line.split("RuntimeError:", 1)[1].strip() for line in reversed(lines)
             if "RuntimeError:" in line),
            lines[-1] if lines else "navigation request failed",
        )
        raise RuntimeError(detail) from None


def _load_waypoints() -> Dict[str, Dict[str, Dict[str, float]]]:
    path = _waypoints_path()
    if not path.is_file():
        return {}
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    return data if isinstance(data, dict) else {}


def _save_waypoints(data: Dict[str, Any]) -> None:
    path = _waypoints_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")


def list_waypoints(robot_id: str) -> Dict[str, Any]:
    rid = str(robot_id or "").strip()
    with _LOCK:
        all_wp = _load_waypoints()
    return {"robot_id": rid, "waypoints": dict(all_wp.get(rid) or {})}


def record_waypoint(
    robot_id: str,
    name: str,
    x: float,
    y: float,
    yaw: float = 0.0,
) -> Dict[str, Any]:
    rid = str(robot_id or "").strip()
    nm = str(name or "").strip()
    if not rid or not nm:
        raise ValueError("robot_id and name are required")
    if not re.match(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$", nm):
        raise ValueError("invalid waypoint name")
    with _LOCK:
        all_wp = _load_waypoints()
        bucket = all_wp.setdefault(rid, {})
        bucket[nm] = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        _save_waypoints(all_wp)
    return {"ok": True, "robot_id": rid, "name": nm, "pose": bucket[nm]}


def goto_waypoint(robot_id: str, name: str) -> Dict[str, Any]:
    rid = str(robot_id or "").strip()
    nm = str(name or "").strip()
    with _LOCK:
        all_wp = _load_waypoints()
    wp = (all_wp.get(rid) or {}).get(nm)
    if not wp:
        raise ValueError(f"waypoint not found: {rid}/{nm}")
    return send_navigate_to_pose(rid, wp["x"], wp["y"], wp.get("yaw", 0.0))


def ros2_read_only(cmd: str, timeout: float = 15.0) -> Dict[str, Any]:
    """Run read-only ros2 introspection (topic echo -n 1, node list, etc.)."""
    allowed_prefixes = (
        "ros2 topic echo ",
        "ros2 topic hz ",
        "ros2 topic info ",
        "ros2 topic list",
        "ros2 node list",
        "ros2 service list",
        "ros2 action list",
        "ros2 lifecycle get ",
    )
    stripped = cmd.strip()
    if not any(stripped.startswith(p) for p in allowed_prefixes):
        raise ValueError("read-only ros2 command not allowed")
    forbidden = (" pub ", " service call ", " action send_goal ", " launch ", " run ")
    low = f" {stripped} "
    if any(x in low for x in forbidden):
        raise ValueError("command must not publish, call services, launch, or run nodes")
    return _ros_run(stripped, timeout=timeout)
