#!/usr/bin/env python3
import base64
import json
import math
import os
import queue
import re
import shlex
import subprocess
import sys
import threading
import time
import signal
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional, Tuple
from urllib.parse import parse_qs, unquote, urlparse

from ros_sensor_store import (
    get_gazebo_models,
    get_planned_path,
    get_scan,
    get_topdown_image,
)

_BACKEND_DIR = Path(__file__).resolve().parent
if str(_BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(_BACKEND_DIR))

ROOT_DIR = Path(__file__).resolve().parent.parent
MAP_DIR = ROOT_DIR / "map"


def _rpy_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Roll, pitch, yaw (rad) for ``gz model -R -P -Y`` fallback; same convention as common ROS utilities."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def call_gazebo_set_model_state(
    root_dir: Path,
    model_name: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    reference_frame: str,
    orientation: Optional[Tuple[float, float, float, float]] = None,
):
    """Call /gazebo/set_model_state via ros2 CLI with sourced workspace.

    Positions are in ``reference_frame`` (typically Gazebo ``world``).
    If ``orientation`` is ``(qx, qy, qz, qw)``, it is applied; otherwise yaw-only about Z (legacy).
    """
    fast_disabled = os.environ.get("GAZEBO_SET_STATE_DISABLE_FAST", "").strip().lower() in (
        "1",
        "true",
        "yes",
    )
    if not fast_disabled:
        try:
            from gazebo_set_state_client import try_set_model_state_fast

            fast = try_set_model_state_fast(
                model_name,
                x,
                y,
                z,
                yaw,
                reference_frame,
                orientation=orientation,
            )
            if fast is not None:
                return fast
        except RuntimeError:
            raise
        except Exception:  # noqa: BLE001
            pass

    ros_distro = (os.environ.get("ROS_DISTRO") or "foxy").strip()
    install_setup = Path(root_dir) / "install" / "setup.bash"
    install_src = f'source "{install_setup}"' if install_setup.is_file() else "true"

    # Preflight: in some setups service list can show stale entries, while the service
    # is actually unreachable from this process (common with cross-container DDS).
    probe_cmd = (
        f'set -eo pipefail; source "/opt/ros/{ros_distro}/setup.bash"; '
        f'cd "{root_dir}" && {install_src}; '
        "python3 - <<'PY'\n"
        "import rclpy\n"
        "from gazebo_msgs.srv import SetModelState\n"
        "rclpy.init()\n"
        "node = rclpy.create_node('set_model_state_probe_web')\n"
        "cli = node.create_client(SetModelState, '/gazebo/set_model_state')\n"
        "ok = cli.wait_for_service(timeout_sec=1.5)\n"
        "print('OK' if ok else 'NO')\n"
        "node.destroy_node()\n"
        "rclpy.shutdown()\n"
        "PY"
    )
    probe = subprocess.run(
        ["bash", "-lc", probe_cmd],
        capture_output=True,
        text=True,
        timeout=6.0,
        env=os.environ.copy(),
    )
    if probe.returncode != 0 or "OK" not in (probe.stdout or ""):
        # Fallback: use Gazebo native transport command, which is often more reliable
        # than ROS service calls in mixed host/container DDS setups.
        q_model = shlex.quote(str(model_name))
        if orientation is not None:
            ox, oy, oz, ow = orientation
            r, p, y_ang = _rpy_from_quaternion(ox, oy, oz, ow)
            rpy = f"-R {float(r)} -P {float(p)} -Y {float(y_ang)}"
        else:
            rpy = f"-R 0.0 -P 0.0 -Y {float(yaw)}"
        gz_cmd = (
            f'cd "{root_dir}" && '
            f"gz model -m {q_model} "
            f"-x {float(x)} -y {float(y)} -z {float(z)} "
            f"{rpy}"
        )
        gz_proc = subprocess.run(
            ["bash", "-lc", gz_cmd],
            capture_output=True,
            text=True,
            timeout=8.0,
            env=os.environ.copy(),
        )
        if gz_proc.returncode != 0:
            msg = (gz_proc.stderr or gz_proc.stdout or "").strip() or "gz model failed"
            raise RuntimeError(
                "set_model_state service unreachable and gz fallback failed: " + msg
            )
        return {
            "ok": True,
            "output": (gz_proc.stdout or "").strip(),
            "fallback": "gz model",
        }

    q_model = shlex.quote(str(model_name))
    q_ref = shlex.quote(str(reference_frame))
    if orientation is not None:
        ox, oy, oz, ow = orientation
        ori = f"orientation: {{x: {float(ox)}, y: {float(oy)}, z: {float(oz)}, w: {float(ow)}}}"
    else:
        ori = (
            f"orientation: {{x: 0.0, y: 0.0, "
            f"z: {math.sin(float(yaw) / 2.0)}, w: {math.cos(float(yaw) / 2.0)}}}"
        )
    payload = (
        "{model_state: {"
        f"model_name: {q_model}, "
        "pose: {"
        f"position: {{x: {float(x)}, y: {float(y)}, z: {float(z)}}}, "
        f"{ori}"
        "}, "
        "twist: {"
        "linear: {x: 0.0, y: 0.0, z: 0.0}, "
        "angular: {x: 0.0, y: 0.0, z: 0.0}"
        "}, "
        f"reference_frame: {q_ref}"
        "}}"
    )
    cmd = (
        f'set -eo pipefail; source "/opt/ros/{ros_distro}/setup.bash"; '
        f'cd "{root_dir}" && {install_src}; '
        "ros2 service call /gazebo/set_model_state gazebo_msgs/srv/SetModelState "
        f"{shlex.quote(payload)}"
    )
    proc = subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        timeout=12.0,
        env=os.environ.copy(),
    )
    if proc.returncode != 0:
        msg = (proc.stderr or proc.stdout or "").strip() or "ros2 service call failed"
        raise RuntimeError(msg)
    out = (proc.stdout or "").strip()
    ok = ("success: True" in out) or ("success=true" in out)
    if not ok:
        raise RuntimeError(out or "set_model_state returned non-success")
    return {"ok": True, "output": out}


class RosNodeManager:
    """Manage configurable ROS node processes for web control page."""

    def __init__(self, root_dir: Path):
        self._root_dir = Path(root_dir)
        self._lock = threading.Lock()
        self._procs = {}
        self._last_error = ""
        self._managed_nodes = self._load_managed_nodes()

    def _load_managed_nodes(self):
        raw = (os.environ.get("ROS_MANAGED_NODES_JSON") or "").strip()
        if raw:
            try:
                data = json.loads(raw)
                if isinstance(data, list):
                    out = []
                    for item in data:
                        if not isinstance(item, dict):
                            continue
                        node_id = str(item.get("id") or "").strip()
                        if not node_id:
                            continue
                        out.append(
                            {
                                "id": node_id,
                                "name": str(item.get("name") or node_id),
                                "start_cmd": str(item.get("start_cmd") or "").strip(),
                                "stop_cmd": str(item.get("stop_cmd") or "").strip(),
                                "match": str(item.get("match") or node_id).strip(),
                                "note": str(item.get("note") or "").strip(),
                            }
                        )
                    if out:
                        return out
            except Exception as exc:  # noqa: BLE001
                self._last_error = f"invalid ROS_MANAGED_NODES_JSON: {exc}"

        return [
            {
                "id": "simulate",
                "name": "simulate",
                "persistent": True,
                "start_cmd": "ros2 launch simulate simulate.launch.py",
                "stop_cmd": "pkill -f 'ros2 launch simulate simulate.launch.py'",
                "match": "simulate.launch.py",
                "note": "仿真场景（示例）",
            },
        ]

    _NODE_ID_VALID = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$")

    @classmethod
    def _sanitize_node_fragment(cls, v: str, field_name: str) -> str:
        s = (v or "").strip()
        if not cls._NODE_ID_VALID.match(s):
            raise ValueError(f"{field_name} invalid: use [a-zA-Z0-9_-], 1-64 chars, start with letter/digit")
        return s

    def _make_fake_pub_spec(self, robot_name: str, current_map: str) -> dict:
        rn = self._sanitize_node_fragment(robot_name, "robot_name")
        cm = self._sanitize_node_fragment(current_map, "current_map")
        node_id = f"fake_pub_{rn}"
        return {
            "id": node_id,
            "name": node_id,
            "persistent": False,
            "start_cmd": f"cd src/fake/scripts && python3 fake_pub.py --ros-args -p robot_name:={rn} -p current_map:={cm}",
            "stop_cmd": f"pkill -f 'python3 fake_pub.py.*robot_name:={rn}' || true",
            "match": f"robot_name:={rn}",
            "note": f"假数据发布 (robot_name={rn}, current_map={cm})",
        }

    def create_fake_pub_node_and_start(self, robot_name: str, current_map: str):
        spec = self._make_fake_pub_spec(robot_name, current_map)
        with self._lock:
            exists = any(s.get("id") == spec["id"] for s in self._managed_nodes)
            if not exists:
                self._managed_nodes.append(spec)
        # 默认创建后立即启动；若已存在则按重启语义处理，避免重复拉起
        status = self.control(spec["id"], "restart")
        return {"node_id": spec["id"], "status": status}

    def _make_slam_mapping_spec(self) -> dict:
        return {
            "id": "slam_bringup_mapping",
            "name": "slam_bringup_mapping",
            "persistent": False,
            "start_cmd": "ros2 launch slam_bringup mapping.launch.py",
            "stop_cmd": "pkill -f 'ros2 launch slam_bringup mapping.launch.py'",
            "match": "mapping.launch.py",
            "note": "SLAM 建图",
        }

    def create_slam_mapping_node_and_start(self):
        spec = self._make_slam_mapping_spec()
        with self._lock:
            exists = any(s.get("id") == spec["id"] for s in self._managed_nodes)
            if not exists:
                self._managed_nodes.append(spec)
        status = self.control(spec["id"], "restart")
        return {"node_id": spec["id"], "status": status}

    @property
    def managed_nodes(self):
        return list(self._managed_nodes)

    def _bash_prefix(self):
        ros_distro = (os.environ.get("ROS_DISTRO") or "foxy").strip()
        install_setup = self._root_dir / "install" / "setup.bash"
        install_src = f'source "{install_setup}"' if install_setup.is_file() else "true"
        return (
            # Don't use `set -u` because ROS setup.bash may reference
            # optional env vars (e.g. AMENT_TRACE_SETUP_FILES) that aren't set.
            f'set -eo pipefail; source "/opt/ros/{ros_distro}/setup.bash"; '
            f'cd "{self._root_dir}" && {install_src}; '
        )

    def _find_pids(self, match_text: str):
        if not match_text:
            return []
        try:
            proc = subprocess.run(
                ["ps", "-eo", "pid=,args="],
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except Exception:
            return []
        pids = []
        for line in (proc.stdout or "").splitlines():
            txt = line.strip()
            if not txt or match_text not in txt:
                continue
            parts = txt.split(None, 1)
            if not parts:
                continue
            try:
                pid = int(parts[0])
            except ValueError:
                continue
            if pid == os.getpid():
                continue
            pids.append(pid)
        return pids

    def _node_status(self, spec):
        node_id = spec["id"]
        with self._lock:
            proc = self._procs.get(node_id)
        proc_running = bool(proc and proc.poll() is None)
        pids = self._find_pids(spec.get("match") or "")
        running = proc_running or bool(pids)
        return {
            "id": node_id,
            "name": spec.get("name") or node_id,
            "running": running,
            "managed": True,
            "persistent": bool(spec.get("persistent")),
            "pid": proc.pid if proc_running else None,
            "process_count": len(pids),
            "note": spec.get("note") or "",
        }

    def _run_shell(self, cmd: str):
        if not cmd:
            raise ValueError("empty command")
        full_cmd = self._bash_prefix() + cmd
        return subprocess.run(
            ["bash", "-lc", full_cmd],
            capture_output=True,
            text=True,
            timeout=15.0,
            env=os.environ.copy(),
        )

    def _start_node(self, spec):
        node_id = spec["id"]
        start_cmd = (spec.get("start_cmd") or "").strip()
        if not start_cmd:
            raise RuntimeError(f"node {node_id} has no start_cmd")
        full_cmd = self._bash_prefix() + start_cmd
        proc = subprocess.Popen(
            ["bash", "-lc", full_cmd],
            cwd=str(self._root_dir),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=os.environ.copy(),
            preexec_fn=os.setsid,
        )
        with self._lock:
            self._procs[node_id] = proc

    def _stop_node(self, spec):
        node_id = spec["id"]
        with self._lock:
            proc = self._procs.get(node_id)
        if proc and proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=3.0)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass
        stop_cmd = (spec.get("stop_cmd") or "").strip()
        if stop_cmd:
            self._run_shell(stop_cmd)

    def _spec_by_id(self, node_id: str):
        for spec in self._managed_nodes:
            if spec["id"] == node_id:
                return spec
        return None

    def list_ros_nodes(self):
        cmd = self._bash_prefix() + "ros2 node list"
        try:
            proc = subprocess.run(
                ["bash", "-lc", cmd],
                capture_output=True,
                text=True,
                timeout=0.8,
                env=os.environ.copy(),
            )
            if proc.returncode != 0:
                return []
            out = []
            for line in (proc.stdout or "").splitlines():
                name = line.strip()
                if name:
                    out.append({"name": name, "running": True})
            return out
        except Exception:
            return []

    def status(self):
        managed = [self._node_status(spec) for spec in self._managed_nodes]
        with self._lock:
            last_error = self._last_error
        try:
            discovered = self.list_ros_nodes()
        except Exception:
            discovered = []
        return {
            "managed_nodes": managed,
            # ros2 node list can block due to DDS discovery; keep it best-effort.
            "discovered_nodes": discovered,
            "last_error": last_error,
            "timestamp": time.time(),
        }

    def kill_discovered_node(self, node_name: str):
        name = (node_name or "").strip()
        if not name.startswith("/"):
            raise ValueError("node_name must start with /")
        token = name.lstrip("/")
        if not token:
            raise ValueError("invalid node_name")

        # Best-effort: match by common ROS2 CLI node remap args or node name token.
        patterns = [f"__node:={token}", f"__node:=/{token}", f"/{token}", token]
        pids = []
        seen = set()
        for pat in patterns:
            for pid in self._find_pids(pat):
                if pid in seen:
                    continue
                seen.add(pid)
                pids.append(pid)
        killed = 0
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
                killed += 1
            except Exception:
                continue
        return {"node_name": name, "matched_pids": pids, "killed_count": killed}

    def control(self, node_id: str, action: str):
        spec = self._spec_by_id(node_id)
        if not spec:
            raise ValueError(f"unknown node_id: {node_id}")
        action = (action or "").strip().lower()
        if action == "stop":
            action = "pause"
        if action not in ("start", "pause", "restart"):
            raise ValueError("action must be start|pause|restart")

        persistent = bool(spec.get("persistent"))
        try:
            if action == "start":
                self._start_node(spec)
            elif action == "pause":
                self._stop_node(spec)
                # 非长期节点：暂停后直接下线（kill 成功后从列表消失）
                if not persistent:
                    with self._lock:
                        self._managed_nodes = [
                            s for s in self._managed_nodes if s.get("id") != node_id
                        ]
                        self._procs.pop(node_id, None)
            else:
                self._stop_node(spec)
                self._start_node(spec)
            with self._lock:
                self._last_error = ""
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                self._last_error = str(exc)
            raise
        return self.status()


def list_floors():
    if not MAP_DIR.exists():
        return []
    floors = []
    for entry in MAP_DIR.iterdir():
        if not entry.is_dir():
            continue
        floor = entry.name
        if (entry / f"{floor}.pgm").exists() and (entry / f"{floor}.yaml").exists():
            floors.append(floor)
    floors.sort()
    return floors


def _list_mapping_floor_labels() -> list:
    try:
        from ros_tf_bridge import _load_robot_specs, list_mapping_robot_ids_from_status_store

        ids = [str(s["id"]) for s in _load_robot_specs()]
        # Mapping floors: robot_status store says "mapping", entry is fresh, and (when
        # pose snapshot lists robots) the id is still present on the ROS bridge output.
        mapping_ids = set(list_mapping_robot_ids_from_status_store())
        try:
            snap = POSE_PROVIDER.get_pose()
            if snap.get("source") == "ros2_tf":
                live_ids = set()
                for r in snap.get("robots") or []:
                    if isinstance(r, dict) and r.get("id"):
                        live_ids.add(str(r["id"]).strip())
                if live_ids:
                    mapping_ids &= live_ids
        except Exception:
            pass
        if mapping_ids:
            ids = [rid for rid in ids if rid in mapping_ids] + [
                rid for rid in sorted(mapping_ids) if rid not in ids
            ]
        else:
            ids = []
        return [f"{rid}_mapping" for rid in ids]
    except Exception:
        return []


def list_floors_for_api() -> list:
    saved = list_floors()
    mapping_labels = _list_mapping_floor_labels()
    seen = set()
    out = []
    for x in mapping_labels + saved:
        if x in seen:
            continue
        seen.add(x)
        out.append(x)
    return out


def _read_pgm_tokens(raw):
    tokens = []
    i = 0
    n = len(raw)
    while i < n and len(tokens) < 4:
        while i < n and raw[i] in b" \t\r\n":
            i += 1
        if i >= n:
            break
        if raw[i] == 35:  # '#'
            while i < n and raw[i] not in b"\r\n":
                i += 1
            continue
        start = i
        while i < n and raw[i] not in b" \t\r\n":
            i += 1
        tokens.append(raw[start:i].decode("ascii"))
    return tokens, i


def parse_pgm(raw):
    tokens, idx = _read_pgm_tokens(raw)
    if len(tokens) != 4:
        raise ValueError("invalid PGM header")

    magic, width_s, height_s, max_val_s = tokens
    width = int(width_s)
    height = int(height_s)
    max_val = int(max_val_s)
    expected_len = width * height

    while idx < len(raw) and raw[idx] in b" \t\r\n":
        idx += 1
    body = raw[idx:]

    if magic == "P2":
        data_tokens = body.decode("ascii").split()
        data = [int(x) for x in data_tokens]
    elif magic == "P5":
        if max_val < 256:
            data = list(body[:expected_len])
        else:
            expected_bytes = expected_len * 2
            raw_body = body[:expected_bytes]
            if len(raw_body) < expected_bytes:
                raise ValueError("PGM binary data incomplete")
            data = [int.from_bytes(raw_body[i : i + 2], "big") for i in range(0, expected_bytes, 2)]
    else:
        raise ValueError("unsupported PGM format")

    if len(data) < expected_len:
        raise ValueError("PGM data incomplete")

    return {"width": width, "height": height, "maxVal": max_val, "data": data[:expected_len]}


_MAP_NAME_VALID = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{0,63}$")


def _sanitize_map_name(name: str) -> str:
    s = (name or "").strip()
    if not _MAP_NAME_VALID.match(s):
        raise ValueError(
            "map_name must be 1–64 chars: letter/digit start, then [a-zA-Z0-9_-]"
        )
    return s


def read_floor_map(floor):
    safe_floor = Path(floor).name
    if safe_floor != floor:
        raise FileNotFoundError("invalid floor name")
    floor_dir = MAP_DIR / safe_floor
    pgm_path = floor_dir / f"{safe_floor}.pgm"
    yaml_path = floor_dir / f"{safe_floor}.yaml"
    if not pgm_path.exists() or not yaml_path.exists():
        raise FileNotFoundError("map file missing")
    pgm = parse_pgm(pgm_path.read_bytes())
    yaml_text = yaml_path.read_text(encoding="utf-8")
    return pgm, yaml_text


class RobotPoseProvider:
    """Multi-robot snapshot for web display. Each robot has id, name, active_floor, pose."""

    def __init__(self):
        self._lock = threading.Lock()
        self._running = True
        self._thread = None
        self._ros_thread = None
        self._ros_stop_event = None
        self._ros_last_error = ""

        raw = (os.environ.get("ROBOT_POSE_MODE") or "ros2_tf").strip().lower()
        if raw in ("", "default"):
            raw = "ros2_tf"

        # 默认：真数据来自 ROS2 TF。仅当显式 ROBOT_POSE_MODE=mock 时使用假轨迹演示。
        if raw == "mock":
            self._mode = "mock"
            self._pose = {
                "timestamp": time.time(),
                "source": "mock",
                "robots": self._mock_robots_seed(),
            }
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            return

        if raw in ("none", "empty"):
            self._mode = raw
            self._pose = self._empty_snapshot("empty")
            return

        if raw != "ros2_tf":
            print(f"[pose] unknown ROBOT_POSE_MODE={raw!r}, using empty robots (no fake data)")
            self._mode = "empty"
            self._pose = self._empty_snapshot("empty")
            return

        self._mode = "ros2_tf"
        self._pose = self._empty_snapshot("ros2_tf")
        self.start_ros_threads()

    @staticmethod
    def _empty_snapshot(source):
        return {
            "timestamp": time.time(),
            "source": source,
            "robots": [],
        }

    @staticmethod
    def _mock_robots_seed():
        return [
            {
                "id": "robot-1",
                "name": "送餐-01",
                "frame_id": "map",
                "active_floor": "nh_102",
                "pose": {"x": 3.0, "y": 3.0, "yaw": 0.0},
                "velocity": {"linear": 0.0, "angular": 0.0},
                "localization": "ok",
                "task_status": "Monitoring",
            },
            {
                "id": "robot-2",
                "name": "送餐-02",
                "frame_id": "map",
                "active_floor": "nh_103",
                "pose": {"x": 4.0, "y": 4.0, "yaw": 0.0},
                "velocity": {"linear": 0.0, "angular": 0.0},
                "localization": "ok",
                "task_status": "Idle",
            },
        ]

    def _apply_ros_update(self, robots, stamp):
        with self._lock:
            self._pose = {
                "timestamp": float(stamp),
                "source": "ros2_tf",
                "robots": json.loads(json.dumps(robots)),
            }

    def _run(self):
        t = 0.0
        while self._running:
            if self._mode == "mock":
                x1 = 3.0 + 1.8 * math.cos(t / 7.0)
                y1 = 3.0 + 1.4 * math.sin(t / 5.0)
                yaw1 = math.atan2(math.cos(t / 5.0), -math.sin(t / 7.0))
                x2 = 5.0 + 1.2 * math.sin(t / 6.0)
                y2 = 5.0 + 1.0 * math.cos(t / 4.0)
                yaw2 = t * 0.15
                with self._lock:
                    self._pose["timestamp"] = time.time()
                    self._pose["source"] = "mock"
                    robots = self._pose["robots"]
                    if len(robots) >= 1:
                        robots[0]["pose"] = {
                            "x": round(x1, 3),
                            "y": round(y1, 3),
                            "yaw": round(yaw1, 3),
                        }
                        robots[0]["velocity"] = {"linear": 0.25, "angular": 0.08}
                    if len(robots) >= 2:
                        robots[1]["pose"] = {
                            "x": round(x2, 3),
                            "y": round(y2, 3),
                            "yaw": round(yaw2, 3),
                        }
                        robots[1]["velocity"] = {"linear": 0.2, "angular": 0.05}
                t += 0.2
            time.sleep(0.2)

    def get_pose(self):
        with self._lock:
            return json.loads(json.dumps(self._pose))

    def get_primary_robot(self):
        """First robot, for legacy /api/robot/status."""
        with self._lock:
            robots = list(self._pose.get("robots") or [])
        if not robots:
            return None
        return json.loads(json.dumps(robots[0]))

    def _set_error(self, msg: str) -> None:
        with self._lock:
            self._ros_last_error = msg

    def _clear_error(self) -> None:
        with self._lock:
            self._ros_last_error = ""

    def _start_ros_bridge_once(self) -> None:
        from ros_tf_bridge import start_ros_tf_thread

        self._ros_stop_event = threading.Event()
        self._ros_thread = start_ros_tf_thread(
            self._apply_ros_update,
            stop_event=self._ros_stop_event,
        )

    def start_ros_threads(self) -> dict:
        if self._mode != "ros2_tf":
            return self.get_ros_threads_status()
        if self._ros_thread and self._ros_thread.is_alive():
            return self.get_ros_threads_status()
        try:
            self._start_ros_bridge_once()
            self._clear_error()
            print("[pose] ROS2 TF bridge started")
        except Exception as exc:  # noqa: BLE001
            msg = (
                f"ROS2 TF unavailable ({exc}). "
                "Install/sourced ROS2 + rclpy/tf2, or set ROBOT_POSE_MODE=mock for local demo."
            )
            print(f"[pose] {msg}")
            self._set_error(msg)
            with self._lock:
                self._pose = self._empty_snapshot("ros2_unavailable")
        return self.get_ros_threads_status()

    def stop_ros_threads(self) -> dict:
        if self._mode != "ros2_tf":
            return self.get_ros_threads_status()
        try:
            from ros_tf_bridge import request_ros_shutdown
        except Exception:
            request_ros_shutdown = None
        if self._ros_stop_event:
            self._ros_stop_event.set()
        if request_ros_shutdown:
            request_ros_shutdown()
        t = self._ros_thread
        if t and t.is_alive():
            t.join(timeout=3.0)
        self._ros_thread = None
        self._ros_stop_event = None
        return self.get_ros_threads_status()

    def restart_ros_threads(self) -> dict:
        self.stop_ros_threads()
        return self.start_ros_threads()

    def get_ros_threads_status(self) -> dict:
        t = self._ros_thread
        running = bool(t and t.is_alive())
        with self._lock:
            err = self._ros_last_error
        return {
            "mode": self._mode,
            "threads": [
                {
                    "name": "ros_tf_bridge",
                    "running": running,
                    "managed": self._mode == "ros2_tf",
                    "note": "TF + /map subscriber thread",
                }
            ],
            "last_error": err,
            "timestamp": time.time(),
        }


POSE_PROVIDER = RobotPoseProvider()
ROS_NODE_MANAGER = RosNodeManager(ROOT_DIR)


class ApiHandler(BaseHTTPRequestHandler):
    def _send_json(self, payload, status=200):
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        try:
            self.wfile.write(raw)
        except BrokenPipeError:
            # Client disconnected (browser aborted request). Ignore to avoid log spam.
            return

    def _not_found(self, message):
        self._send_json({"error": message}, status=404)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/api/ros/nodes/create":
            length_s = self.headers.get("Content-Length")
            try:
                length = int(length_s) if length_s else 0
            except ValueError:
                self._send_json({"error": "invalid Content-Length"}, 400)
                return
            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json({"error": "invalid JSON body"}, 400)
                return
            if not isinstance(data, dict):
                self._send_json({"error": "body must be a JSON object"}, 400)
                return

            node_type = str(data.get("type") or "").strip().lower()

            try:
                if node_type == "fake_pub":
                    robot_name = str(data.get("robot_name") or "").strip()
                    initial_floor = str(
                        data.get("initial_floor") or data.get("current_map") or ""
                    ).strip()
                    if not robot_name or not initial_floor:
                        self._send_json(
                            {"error": "robot_name and initial_floor are required"},
                            400,
                        )
                        return
                    payload = ROS_NODE_MANAGER.create_fake_pub_node_and_start(
                        robot_name, initial_floor
                    )
                elif node_type == "slam_bringup_mapping":
                    payload = ROS_NODE_MANAGER.create_slam_mapping_node_and_start()
                else:
                    self._send_json(
                        {
                            "error": "unsupported node type (supported: fake_pub, slam_bringup_mapping)"
                        },
                        400,
                    )
                    return
            except ValueError as err:
                self._send_json({"error": str(err)}, 400)
                return
            except Exception as err:  # noqa: BLE001
                self._send_json({"error": str(err)}, 500)
                return

            self._send_json({"ok": True, **payload})
            return

        if path == "/api/ros/nodes/control":
            length_s = self.headers.get("Content-Length")
            try:
                length = int(length_s) if length_s else 0
            except ValueError:
                self._send_json({"error": "invalid Content-Length"}, 400)
                return
            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json({"error": "invalid JSON body"}, 400)
                return
            if not isinstance(data, dict):
                self._send_json({"error": "body must be a JSON object"}, 400)
                return
            node_id = str(data.get("node_id") or "").strip()
            action = str(data.get("action") or "").strip().lower()
            if not node_id:
                self._send_json({"error": "node_id is required"}, 400)
                return
            if action not in ("start", "pause", "restart", "stop"):
                self._send_json({"error": "action must be start|pause|restart"}, 400)
                return
            try:
                status = ROS_NODE_MANAGER.control(node_id, action)
            except ValueError as err:
                self._send_json({"error": str(err)}, 400)
                return
            except Exception as err:  # noqa: BLE001
                self._send_json({"error": str(err)}, 500)
                return
            self._send_json({"ok": True, "node_id": node_id, "action": action, "status": status})
            return

        if path == "/api/ros/nodes/discovered/kill":
            length_s = self.headers.get("Content-Length")
            try:
                length = int(length_s) if length_s else 0
            except ValueError:
                self._send_json({"error": "invalid Content-Length"}, 400)
                return
            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json({"error": "invalid JSON body"}, 400)
                return
            if not isinstance(data, dict):
                self._send_json({"error": "body must be a JSON object"}, 400)
                return
            node_name = str(data.get("node_name") or "").strip()
            if not node_name:
                self._send_json({"error": "node_name is required"}, 400)
                return
            try:
                # Known node in backend process: stop ROS TF thread instead of killing backend.
                if node_name == "/open_delivery_web_tf_bridge":
                    thread_status = POSE_PROVIDER.stop_ros_threads()
                    out = {
                        "node_name": node_name,
                        "handled_by": "ros_threads_control",
                        "threads_status": thread_status,
                    }
                # Common fake publisher node in this workspace.
                elif node_name == "/fake_robots_tf":
                    proc = ROS_NODE_MANAGER._run_shell("pkill -f 'python3 fake_pub.py' || true")
                    out = {
                        "node_name": node_name,
                        "handled_by": "pkill_fake_pub",
                        "returncode": proc.returncode,
                    }
                else:
                    out = ROS_NODE_MANAGER.kill_discovered_node(node_name)
            except ValueError as err:
                self._send_json({"error": str(err)}, 400)
                return
            except Exception as err:  # noqa: BLE001
                self._send_json({"error": str(err)}, 500)
                return
            self._send_json({"ok": True, **out, "status": ROS_NODE_MANAGER.status()})
            return

        if path == "/api/ros/threads/control":
            length_s = self.headers.get("Content-Length")
            try:
                length = int(length_s) if length_s else 0
            except ValueError:
                self._send_json({"error": "invalid Content-Length"}, 400)
                return
            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json({"error": "invalid JSON body"}, 400)
                return
            if not isinstance(data, dict):
                self._send_json({"error": "body must be a JSON object"}, 400)
                return
            action = str(data.get("action") or "").strip().lower()
            if action not in ("start", "stop", "restart"):
                self._send_json({"error": "action must be start|stop|restart"}, 400)
                return
            if action == "start":
                status = POSE_PROVIDER.start_ros_threads()
            elif action == "stop":
                status = POSE_PROVIDER.stop_ros_threads()
            else:
                status = POSE_PROVIDER.restart_ros_threads()
            self._send_json({"ok": True, "action": action, "status": status})
            return

        if path == "/api/mapping/save":
            length_s = self.headers.get("Content-Length")
            try:
                length = int(length_s) if length_s else 0
            except ValueError:
                self._send_json({"error": "invalid Content-Length"}, 400)
                return
            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json({"error": "invalid JSON body"}, 400)
                return
            if not isinstance(data, dict):
                self._send_json({"error": "body must be a JSON object"}, 400)
                return
            raw_name = data.get("map_name")
            if raw_name is None or not str(raw_name).strip():
                self._send_json({"error": "map_name is required"}, 400)
                return
            try:
                safe_name = _sanitize_map_name(str(raw_name))
            except ValueError as err:
                self._send_json({"error": str(err)}, 400)
                return
            out_dir = MAP_DIR / safe_name
            try:
                out_dir.mkdir(parents=True, exist_ok=True)
            except OSError as err:
                self._send_json({"error": f"cannot create map dir: {err}"}, 500)
                return
            prefix = out_dir / safe_name
            root = str(ROOT_DIR)
            ros_distro = (os.environ.get("ROS_DISTRO") or "foxy").strip()
            install_setup = Path(root) / "install" / "setup.bash"
            install_src = f'source "{install_setup}"' if install_setup.is_file() else "true"
            bash_cmd = (
                f'set -eo pipefail; '
                f'source "/opt/ros/{ros_distro}/setup.bash"; '
                f'cd "{root}" && {install_src}; '
                f'cd "{out_dir}" && '
                f'ros2 run nav2_map_server map_saver_cli -f "{prefix}"'
            )
            try:
                proc = subprocess.run(
                    ["bash", "-lc", bash_cmd],
                    capture_output=True,
                    text=True,
                    timeout=120,
                    env=os.environ.copy(),
                )
            except subprocess.TimeoutExpired:
                self._send_json({"error": "map_saver timed out (120s)"}, 504)
                return
            if proc.returncode != 0:
                err = (proc.stderr or proc.stdout or "").strip() or "map_saver failed"
                self._send_json({"error": err, "code": proc.returncode}, 500)
                return
            yaml_path = Path(f"{prefix}.yaml")
            pgm_path = Path(f"{prefix}.pgm")
            if not yaml_path.is_file() or not pgm_path.is_file():
                self._send_json(
                    {
                        "error": "map_saver finished but .yaml/.pgm not found",
                        "stderr": (proc.stderr or "")[:2000],
                    },
                    500,
                )
                return
            self._send_json(
                {
                    "ok": True,
                    "map_name": safe_name,
                    "yaml": str(yaml_path),
                    "pgm": str(pgm_path),
                }
            )
            return

        if path == "/api/gazebo/set_model_state":
            length_s = self.headers.get("Content-Length")
            try:
                length = int(length_s) if length_s else 0
            except ValueError:
                self._send_json({"error": "invalid Content-Length"}, 400)
                return
            body = self.rfile.read(length) if length > 0 else b"{}"
            try:
                data = json.loads(body.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._send_json({"error": "invalid JSON body"}, 400)
                return
            if not isinstance(data, dict):
                self._send_json({"error": "body must be a JSON object"}, 400)
                return
            model_name = str(data.get("model_name") or "").strip()
            reference_frame = str(data.get("reference_frame") or "world").strip() or "world"
            if not model_name:
                self._send_json({"error": "model_name is required"}, 400)
                return
            try:
                x = float(data.get("x"))
                y = float(data.get("y"))
                yaw = float(data.get("yaw", 0.0))
                z = float(data.get("z", 0.05))
            except (TypeError, ValueError):
                self._send_json({"error": "x, y, yaw, z must be numeric"}, 400)
                return
            orientation = None
            ori = data.get("orientation")
            if isinstance(ori, dict):
                try:
                    orientation = (
                        float(ori["x"]),
                        float(ori["y"]),
                        float(ori["z"]),
                        float(ori["w"]),
                    )
                except (KeyError, TypeError, ValueError):
                    self._send_json(
                        {"error": "orientation, if set, must be object with numeric x,y,z,w"},
                        400,
                    )
                    return
            try:
                out = call_gazebo_set_model_state(
                    ROOT_DIR,
                    model_name,
                    x,
                    y,
                    z,
                    yaw,
                    reference_frame,
                    orientation=orientation,
                )
            except subprocess.TimeoutExpired:
                self._send_json({"error": "set_model_state timeout"}, 504)
                return
            except RuntimeError as err:
                self._send_json({"error": str(err)}, 500)
                return
            except Exception as err:  # noqa: BLE001
                self._send_json({"error": str(err)}, 500)
                return
            body_out = {
                "ok": True,
                "model_name": model_name,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "reference_frame": reference_frame,
                "raw": out.get("output", ""),
            }
            if orientation is not None:
                body_out["orientation"] = {
                    "x": orientation[0],
                    "y": orientation[1],
                    "z": orientation[2],
                    "w": orientation[3],
                }
            self._send_json(body_out)
            return

        if path != "/api/robot/command":
            self._not_found("endpoint not found")
            return

        length_s = self.headers.get("Content-Length")
        try:
            length = int(length_s) if length_s else 0
        except ValueError:
            self._send_json({"error": "invalid Content-Length"}, 400)
            return
        body = self.rfile.read(length) if length > 0 else b"{}"
        try:
            data = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._send_json({"error": "invalid JSON body"}, 400)
            return

        if not isinstance(data, dict):
            self._send_json({"error": "body must be a JSON object"}, 400)
            return

        mode = str(data.get("mode") or "").strip()
        if mode not in ("map_only", "pose_only", "both"):
            self._send_json(
                {"error": "mode must be map_only, pose_only, or both"},
                400,
            )
            return

        robot_id = str(data.get("robot_id") or "").strip()
        if not robot_id:
            self._send_json({"error": "robot_id is required"}, 400)
            return

        map_name = data.get("map_name")
        if map_name is not None:
            map_name = str(map_name).strip()
        x = data.get("x")
        y = data.get("y")
        yaw = data.get("yaw")

        if mode in ("map_only", "both"):
            if not map_name:
                self._send_json({"error": "map_name is required for this mode"}, 400)
                return
        if mode in ("pose_only", "both"):
            try:
                xf = float(x)
                yf = float(y)
                yawf = float(yaw if yaw is not None else 0.0)
            except (TypeError, ValueError):
                self._send_json(
                    {"error": "x, y and yaw (number, rad) are required for pose"},
                    400,
                )
                return
        else:
            xf = yf = yawf = 0.0

        try:
            import ros_command_queue as rcq
        except ImportError:
            self._send_json({"error": "ros_command_queue not available"}, 500)
            return

        cmd = {"mode": mode, "robot_id": robot_id}
        if mode in ("map_only", "both"):
            cmd["map_name"] = map_name
        if mode in ("pose_only", "both"):
            cmd["x"] = xf
            cmd["y"] = yf
            cmd["yaw"] = yawf

        try:
            rcq.enqueue_command(cmd)
        except RuntimeError as exc:
            self._send_json({"error": str(exc)}, 503)
        except queue.Full:
            self._send_json({"error": "command queue full"}, 503)
        else:
            self._send_json({"ok": True})

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/ros/nodes/status":
            self._send_json(ROS_NODE_MANAGER.status())
            return

        if path == "/api/ros/threads/status":
            self._send_json(POSE_PROVIDER.get_ros_threads_status())
            return

        if path == "/api/floors":
            self._send_json({"floors": list_floors_for_api()})
            return

        if path == "/api/mapping/live":
            try:
                from ros_map_store import get_snapshot
            except ImportError:
                self._send_json({"available": False, "reason": "ros_map_store missing"}, 200)
                return
            q = parse_qs(urlparse(self.path).query)
            robot_id = (q.get("robot_id") or [None])[0]
            robot_id = (robot_id or "").strip()
            if not robot_id:
                self._send_json(
                    {"available": False, "reason": "robot_id query required"},
                    200,
                )
                return
            snap = get_snapshot(robot_id)
            if not snap:
                self._send_json(
                    {
                        "available": False,
                        "reason": (
                            f"no data on /{robot_id}/mapping yet "
                            "(publish OccupancyGrid there; ROS TF bridge must run)"
                        ),
                    },
                    200,
                )
                return
            pix = snap["pixels"]
            self._send_json(
                {
                    "available": True,
                    "robot_id": snap.get("robot_id") or robot_id,
                    "width": snap["width"],
                    "height": snap["height"],
                    "resolution": snap["resolution"],
                    "origin": snap["origin"],
                    "origin_yaw": snap["origin_yaw"],
                    "stamp_sec": snap["stamp_sec"],
                    "stamp_nanosec": snap["stamp_nanosec"],
                    "frame_id": snap["frame_id"],
                    "encoding": "base64",
                    "data_b64": base64.b64encode(pix).decode("ascii"),
                }
            )
            return

        if path.startswith("/api/maps/"):
            floor = unquote(path[len("/api/maps/") :]).strip()
            if not floor:
                self._not_found("floor is required")
                return
            try:
                pgm, yaml_text = read_floor_map(floor)
            except FileNotFoundError:
                self._not_found(f"floor not found: {floor}")
                return
            except ValueError as err:
                self._send_json({"error": f"invalid map format: {err}"}, status=500)
                return
            self._send_json({"floor": floor, "pgm": pgm, "yaml": yaml_text})
            return

        if path == "/api/robot/pose":
            self._send_json(POSE_PROVIDER.get_pose())
            return

        if path == "/api/gazebo/topdown/state":
            snap = get_gazebo_models()
            if not snap:
                self._send_json(
                    {"available": False, "reason": "no /gazebo/model_states data yet"},
                    200,
                )
                return
            self._send_json(snap, 200)
            return

        if path == "/api/gazebo/top_camera":
            frame = get_topdown_image()
            if not frame:
                self._send_json(
                    {"available": False, "reason": "no topdown camera frame yet"},
                    200,
                )
                return
            self._send_json(frame, 200)
            return

        m_scan = re.match(r"^/api/robot/([^/]+)/scan_2d$", path)
        if m_scan:
            rid = unquote(m_scan.group(1))
            data = get_scan(rid)
            if not data:
                self._send_json({"error": "no scan data", "robot_id": rid}, 404)
            else:
                clean = {k: v for k, v in data.items() if not str(k).startswith("_")}
                self._send_json(clean)
            return

        m_path = re.match(r"^/api/robot/([^/]+)/planned_path$", path)
        if m_path:
            rid = unquote(m_path.group(1))
            data = get_planned_path(rid)
            if not data:
                self._send_json({"error": "no path data", "robot_id": rid}, 404)
            else:
                clean = {k: v for k, v in data.items() if not str(k).startswith("_")}
                self._send_json(clean)
            return

        if path == "/api/robot/status":
            snap = POSE_PROVIDER.get_pose()
            primary = POSE_PROVIDER.get_primary_robot()
            if primary:
                self._send_json(
                    {
                        "online": True,
                        "timestamp": snap["timestamp"],
                        "localization": primary.get("localization"),
                        "task_status": primary.get("task_status"),
                        "active_floor": primary.get("active_floor"),
                        "robot_id": primary.get("id"),
                        "robot_name": primary.get("name"),
                        "source": snap.get("source"),
                    }
                )
            else:
                self._send_json(
                    {
                        "online": False,
                        "timestamp": snap["timestamp"],
                        "source": snap.get("source"),
                    }
                )
            return

        if path == "/api/robot/status/cache":
            try:
                import ros_robot_status_store
            except ImportError:
                self._send_json({"items": []})
                return
            self._send_json(
                {
                    "items": ros_robot_status_store.list_all_last_status(),
                    "timestamp": time.time(),
                }
            )
            return

        if path == "/api/robot/pose/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(POSE_PROVIDER.get_pose(), ensure_ascii=False)
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(0.25)
            except (BrokenPipeError, ConnectionResetError):
                return

        self._not_found("endpoint not found")

    def log_message(self, format, *args):
        # keep terminal output concise
        return


def main():
    host = os.environ.get("MAP_API_HOST", "0.0.0.0")
    port = int(os.environ.get("MAP_API_PORT", "8001"))
    server = ThreadingHTTPServer((host, port), ApiHandler)
    print(f"map api listening on http://{host}:{port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
