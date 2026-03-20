#!/usr/bin/env python3
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse


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
    def __init__(self):
        self._lock = threading.Lock()
        self._mode = os.environ.get("ROBOT_POSE_MODE", "mock")
        self._pose = {
            "timestamp": time.time(),
            "frame_id": "map",
            "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "velocity": {"linear": 0.0, "angular": 0.0},
            "localization": "ok",
            "task_status": "Idle",
            "current_map": "nh_102",
            "source": self._mode,
        }
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        t = 0.0
        while self._running:
            if self._mode == "mock":
                x = 3.0 + 1.8 * math.cos(t / 7.0)
                y = 3.0 + 1.4 * math.sin(t / 5.0)
                yaw = math.atan2(math.cos(t / 5.0), -math.sin(t / 7.0))
                linear = 0.25
                angular = 0.08
                with self._lock:
                    self._pose.update(
                        {
                            "timestamp": time.time(),
                            "pose": {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)},
                            "velocity": {"linear": linear, "angular": angular},
                            "localization": "ok",
                            "task_status": "Monitoring",
                            "source": "mock",
                        }
                    )
                t += 0.2
            time.sleep(0.2)

    def get_pose(self):
        with self._lock:
            return json.loads(json.dumps(self._pose))


POSE_PROVIDER = RobotPoseProvider()


class ApiHandler(BaseHTTPRequestHandler):
    def _send_json(self, payload, status=200):
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(raw)

    def _not_found(self, message):
        self._send_json({"error": message}, status=404)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

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

        if path == "/api/robot/status":
            pose = POSE_PROVIDER.get_pose()
            self._send_json(
                {
                    "online": True,
                    "timestamp": pose["timestamp"],
                    "localization": pose["localization"],
                    "task_status": pose["task_status"],
                    "current_map": pose["current_map"],
                    "source": pose["source"],
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
