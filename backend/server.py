#!/usr/bin/env python3
import json
import math
import os
import queue
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

from ros_sensor_store import get_planned_path, get_scan

_BACKEND_DIR = Path(__file__).resolve().parent
if str(_BACKEND_DIR) not in sys.path:
    sys.path.insert(0, str(_BACKEND_DIR))

ROOT_DIR = Path(__file__).resolve().parent.parent
MAP_DIR = ROOT_DIR / "map"


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
    """Multi-robot snapshot for web display. Each robot has id, name, current_map, pose."""

    def __init__(self):
        self._lock = threading.Lock()
        self._running = True
        self._thread = None

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
        try:
            from ros_tf_bridge import start_ros_tf_thread

            start_ros_tf_thread(self._apply_ros_update)
            print("[pose] ROS2 TF bridge started (default; set ROBOT_POSE_MODE=mock for demo only)")
        except Exception as exc:  # noqa: BLE001
            print(
                f"[pose] ROS2 TF unavailable ({exc}). "
                "Robots list stays empty (no fake data). "
                "Install/sourced ROS2 + rclpy/tf2, or set ROBOT_POSE_MODE=mock for local demo."
            )
            self._mode = "ros2_unavailable"
            self._pose = self._empty_snapshot("ros2_unavailable")

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
                "current_map": "nh_102",
                "pose": {"x": 3.0, "y": 3.0, "yaw": 0.0},
                "velocity": {"linear": 0.0, "angular": 0.0},
                "localization": "ok",
                "task_status": "Monitoring",
            },
            {
                "id": "robot-2",
                "name": "送餐-02",
                "frame_id": "map",
                "current_map": "nh_103",
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


POSE_PROVIDER = RobotPoseProvider()


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
        self.wfile.write(raw)

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
        if path == "/api/floors":
            self._send_json({"floors": list_floors()})
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
                        "current_map": primary.get("current_map"),
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
