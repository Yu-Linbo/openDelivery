"""Read-only rosbag2 SQLite playback extraction without importing or starting ROS.

Only the small subset of CDR messages needed by the Web log player is decoded.
The module never publishes topics and opens every database with SQLite ``mode=ro``.
"""

import json
import base64
import io
import math
import sqlite3
import struct
from collections import deque
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple
from urllib.parse import quote

from PIL import Image


class BagReplayError(ValueError):
    """A bag cannot be opened or does not contain supported rosbag2 data."""


class _CdrReader:
    def __init__(self, payload: bytes):
        if not isinstance(payload, (bytes, bytearray, memoryview)) or len(payload) < 4:
            raise BagReplayError("invalid CDR payload")
        self._data = memoryview(payload)
        # CDR encapsulation identifiers 0x0001/0x0003 are little-endian.
        self._endian = "<" if payload[1] & 1 else ">"
        self._offset = 4
        self._origin = 4

    def _align(self, size: int) -> None:
        self._offset += (-(self._offset - self._origin)) % size

    def _unpack(self, fmt: str, size: int, alignment: Optional[int] = None):
        self._align(alignment or size)
        end = self._offset + size
        if end > len(self._data):
            raise BagReplayError("truncated CDR payload")
        value = struct.unpack_from(self._endian + fmt, self._data, self._offset)[0]
        self._offset = end
        return value

    def uint32(self) -> int:
        return int(self._unpack("I", 4))

    def int32(self) -> int:
        return int(self._unpack("i", 4))

    def float32(self) -> float:
        return float(self._unpack("f", 4))

    def float64(self) -> float:
        return float(self._unpack("d", 8))

    def boolean(self) -> bool:
        return bool(self._unpack("B", 1))

    def uint8(self) -> int:
        return int(self._unpack("B", 1))

    def octets(self, size: int) -> bytes:
        if size < 0 or size > 128 * 1024 * 1024:
            raise BagReplayError("CDR byte sequence is too large")
        end = self._offset + size
        if end > len(self._data):
            raise BagReplayError("truncated CDR byte sequence")
        value = bytes(self._data[self._offset:end])
        self._offset = end
        return value

    def string(self) -> str:
        size = self.uint32()
        if size > 16 * 1024 * 1024:
            raise BagReplayError("CDR string is too large")
        end = self._offset + size
        if end > len(self._data):
            raise BagReplayError("truncated CDR string")
        raw = bytes(self._data[self._offset:end])
        self._offset = end
        if raw.endswith(b"\x00"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")


def _header(reader: _CdrReader) -> Dict[str, Any]:
    sec = reader.int32()
    nanosec = reader.uint32()
    return {"stamp": sec + nanosec / 1_000_000_000.0, "frame_id": reader.string()}


def _pose(reader: _CdrReader) -> Dict[str, float]:
    x, y, z = reader.float64(), reader.float64(), reader.float64()
    qx, qy, qz, qw = (
        reader.float64(), reader.float64(), reader.float64(), reader.float64()
    )
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return {"x": x, "y": y, "z": z, "yaw": math.atan2(siny, cosy)}


def _skip_float64(reader: _CdrReader, count: int) -> None:
    for _ in range(count):
        reader.float64()


def _decode_odometry(payload: bytes) -> Dict[str, Any]:
    reader = _CdrReader(payload)
    head = _header(reader)
    child = reader.string()
    pose = _pose(reader)
    _skip_float64(reader, 36)
    linear = {"x": reader.float64(), "y": reader.float64(), "z": reader.float64()}
    angular = {"x": reader.float64(), "y": reader.float64(), "z": reader.float64()}
    return {
        "frame_id": head["frame_id"],
        "child_frame_id": child,
        "pose": pose,
        "linear": linear,
        "angular": angular,
    }


def _decode_pose_stamped(payload: bytes, with_covariance: bool = False) -> Dict[str, Any]:
    reader = _CdrReader(payload)
    head = _header(reader)
    pose = _pose(reader)
    if with_covariance:
        _skip_float64(reader, 36)
    return {"frame_id": head["frame_id"], "pose": pose}


def _decode_twist(payload: bytes) -> Dict[str, Any]:
    reader = _CdrReader(payload)
    linear = {"x": reader.float64(), "y": reader.float64(), "z": reader.float64()}
    angular = {"x": reader.float64(), "y": reader.float64(), "z": reader.float64()}
    return {"linear": linear, "angular": angular}


def _decode_laser_scan(payload: bytes, max_hits: int = 240) -> Dict[str, Any]:
    reader = _CdrReader(payload)
    head = _header(reader)
    angle_min = reader.float32()
    reader.float32()  # angle_max
    angle_increment = reader.float32()
    reader.float32()  # time_increment
    reader.float32()  # scan_time
    range_min = reader.float32()
    range_max = reader.float32()
    count = reader.uint32()
    if count > 2_000_000:
        raise BagReplayError("LaserScan sequence is too large")
    stride = max(1, int(math.ceil(count / max_hits)))
    points: List[float] = []
    valid_count = 0
    for index in range(count):
        distance = reader.float32()
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        valid_count += 1
        if index % stride:
            continue
        angle = angle_min + angle_increment * index
        points.extend((round(distance * math.cos(angle), 4), round(distance * math.sin(angle), 4)))
    intensity_count = reader.uint32()
    for _ in range(intensity_count):
        reader.float32()
    return {
        "frame_id": head["frame_id"],
        "coordinates": "sensor_frame",
        "points": points,
        "valid_count": valid_count,
    }


def _decode_path(payload: bytes, max_points: int = 600) -> Dict[str, Any]:
    reader = _CdrReader(payload)
    head = _header(reader)
    count = reader.uint32()
    if count > 1_000_000:
        raise BagReplayError("Path sequence is too large")
    stride = max(1, int(math.ceil(count / max_points)))
    points: List[float] = []
    for index in range(count):
        _header(reader)
        pose = _pose(reader)
        if index % stride == 0 or index == count - 1:
            points.extend((round(pose["x"], 4), round(pose["y"], 4)))
    return {"frame_id": head["frame_id"], "points": points, "point_count": count}


def _decode_robot_status(payload: bytes) -> Dict[str, Any]:
    reader = _CdrReader(payload)
    _header(reader)
    result = {
        "robot_name": reader.string(),
        "robot_model": reader.string(),
        "current_map": reader.string(),
        "current_position": reader.string(),
        "robot_status": reader.string(),
        "task_status": reader.string(),
        "control_status": reader.string(),
        "localization_method": reader.string(),
        "is_simulation": reader.boolean(),
        "task_progress": reader.float32(),
    }
    return result



def _decode_task_status(payload: bytes) -> Dict[str, Any]:
    reader = _CdrReader(payload)
    _header(reader)
    task_id = reader.string()

    def string_sequence() -> List[str]:
        count = reader.uint32()
        if count > 100_000:
            raise BagReplayError("TaskStatus string sequence is too large")
        return [reader.string() for _ in range(count)]

    work_queue = string_sequence()
    model_status = string_sequence()
    task_status = reader.string()
    message = reader.string()
    current_index = reader.uint32()
    total_count = reader.uint32()
    return {
        "task_id": task_id,
        "work_queue": work_queue,
        "model_status": model_status,
        "task_status": task_status,
        "message": message,
        "current_index": current_index,
        "total_count": total_count,
    }

def _decode_image(payload: bytes) -> Dict[str, Any]:
    reader = _CdrReader(payload)
    head = _header(reader)
    height = reader.uint32()
    width = reader.uint32()
    encoding = reader.string().strip().lower()
    is_bigendian = reader.uint8()
    step = reader.uint32()
    data_size = reader.uint32()
    data = reader.octets(data_size)
    if not width or not height or width > 16384 or height > 16384:
        raise BagReplayError("Image dimensions are invalid")

    formats = {
        "rgb8": ("RGB", "RGB", 3),
        "bgr8": ("RGB", "BGR", 3),
        "rgba8": ("RGBA", "RGBA", 4),
        "bgra8": ("RGBA", "BGRA", 4),
        "mono8": ("L", "L", 1),
        "8uc1": ("L", "L", 1),
    }
    image_format = formats.get(encoding)
    if not image_format:
        raise BagReplayError(f"unsupported Image encoding: {encoding or 'unknown'}")
    mode, raw_mode, channels = image_format
    if step < width * channels or data_size < step * height:
        raise BagReplayError("Image data is shorter than width/height/step")

    image = Image.frombytes(mode, (width, height), data, "raw", raw_mode, step, 1)
    resampling = getattr(Image, "Resampling", Image)
    image.thumbnail((480, 270), resampling.LANCZOS)
    if image.mode != "RGB":
        image = image.convert("RGB")
    output = io.BytesIO()
    image.save(output, format="JPEG", quality=68, optimize=True)
    return {
        "frame_id": head["frame_id"],
        "width": width,
        "height": height,
        "preview_width": image.width,
        "preview_height": image.height,
        "encoding": encoding,
        "is_bigendian": is_bigendian,
        "data_url": "data:image/jpeg;base64," + base64.b64encode(output.getvalue()).decode("ascii"),
    }


def _decode_tf(payload: bytes) -> List[Dict[str, Any]]:
    reader = _CdrReader(payload)
    count = reader.uint32()
    if count > 100_000:
        raise BagReplayError("TF sequence is too large")
    rows = []
    for _ in range(count):
        head = _header(reader)
        child = reader.string()
        x, y, z = reader.float64(), reader.float64(), reader.float64()
        qx, qy, qz, qw = (
            reader.float64(), reader.float64(), reader.float64(), reader.float64()
        )
        siny = 2.0 * (qw * qz + qx * qy)
        cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
        rows.append(
            {
                "parent": _clean_frame(head["frame_id"]),
                "child": _clean_frame(child),
                "x": x,
                "y": y,
                "z": z,
                "yaw": math.atan2(siny, cosy),
            }
        )
    return rows


def _clean_frame(value: str) -> str:
    return str(value or "").strip().strip("/")


def _compose_2d(first: Tuple[float, float, float], second: Tuple[float, float, float]):
    ax, ay, ayaw = first
    bx, by, byaw = second
    cosine, sine = math.cos(ayaw), math.sin(ayaw)
    return (
        ax + cosine * bx - sine * by,
        ay + sine * bx + cosine * by,
        math.atan2(math.sin(ayaw + byaw), math.cos(ayaw + byaw)),
    )


def _inverse_2d(value: Tuple[float, float, float]):
    x, y, yaw = value
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (-cosine * x - sine * y, sine * x - cosine * y, -yaw)


def _tf_frames(
    graph: Dict[Tuple[str, str], Tuple[float, float, float]]
) -> Set[str]:
    return {
        frame
        for parent_child in graph
        for frame in parent_child
        if frame
    }


def _ranked_tf_frames(
    frames: Iterable[str], suffixes: Tuple[str, ...], robot_name: str
) -> List[str]:
    robot_prefix = _clean_frame(robot_name)
    return sorted(
        (
            frame
            for frame in frames
            if any(frame == suffix or frame.endswith("/" + suffix) for suffix in suffixes)
        ),
        key=lambda frame: (
            0 if robot_prefix and frame.startswith(robot_prefix + "/") else 1,
            len(frame),
        ),
    )


def _lookup_tf_2d(
    graph: Dict[Tuple[str, str], Tuple[float, float, float]],
    sources: Iterable[str],
    targets: Iterable[str],
) -> Optional[Dict[str, Any]]:
    adjacency: Dict[str, List[Tuple[str, Tuple[float, float, float]]]] = {}
    for (parent, child), transform in graph.items():
        if not parent or not child:
            continue
        adjacency.setdefault(parent, []).append((child, transform))
        adjacency.setdefault(child, []).append((parent, _inverse_2d(transform)))

    target_set = set(targets)
    for source in sources:
        if source in target_set:
            return {"source": source, "target": source, "transform": (0.0, 0.0, 0.0)}
        queue = deque([(source, (0.0, 0.0, 0.0))])
        visited = {source}
        while queue:
            frame, accumulated = queue.popleft()
            if frame in target_set:
                return {"source": source, "target": frame, "transform": accumulated}
            for next_frame, edge in adjacency.get(frame, []):
                if next_frame in visited:
                    continue
                visited.add(next_frame)
                queue.append((next_frame, _compose_2d(accumulated, edge)))
    return None


def _transform_scan_points(
    scan: Dict[str, Any], transform: Tuple[float, float, float]
) -> None:
    origin_x, origin_y, yaw = transform
    cosine, sine = math.cos(yaw), math.sin(yaw)
    source_points = scan.get("points") or []
    transformed: List[float] = []
    for index in range(0, len(source_points) - 1, 2):
        local_x = float(source_points[index])
        local_y = float(source_points[index + 1])
        transformed.extend(
            (
                round(origin_x + cosine * local_x - sine * local_y, 4),
                round(origin_y + sine * local_x + cosine * local_y, 4),
            )
        )
    scan["points"] = transformed


def _place_scan_in_tf_graph(
    scan: Dict[str, Any],
    graph: Dict[Tuple[str, str], Tuple[float, float, float]],
    robot_name: str,
) -> None:
    scan_frame = _clean_frame(scan.get("frame_id", ""))
    scan["frame_id"] = scan_frame
    if not scan_frame:
        return

    frames = _tf_frames(graph)
    map_frames = _ranked_tf_frames(frames, ("map",), robot_name)
    located = _lookup_tf_2d(graph, map_frames, (scan_frame,))
    coordinates = "map"
    if not located:
        base_frames = _ranked_tf_frames(
            frames, ("base_footprint", "base_link"), robot_name
        )
        located = _lookup_tf_2d(graph, base_frames, (scan_frame,))
        coordinates = "robot_base"
    if not located:
        return

    _transform_scan_points(scan, located["transform"])
    scan.update(
        {
            "source_frame_id": scan_frame,
            "frame_id": located["source"],
            "coordinates": coordinates,
            "tf_applied": True,
        }
    )


def _find_tf_pose(
    graph: Dict[Tuple[str, str], Tuple[float, float, float]], robot_name: str
) -> Optional[Dict[str, Any]]:
    frames = _tf_frames(graph)
    sources = _ranked_tf_frames(frames, ("map",), robot_name) or _ranked_tf_frames(
        frames, ("odom",), robot_name
    )
    targets = _ranked_tf_frames(frames, ("base_footprint", "base_link"), robot_name)
    located = _lookup_tf_2d(graph, sources, targets)
    if not located:
        return None
    x, y, yaw = located["transform"]
    return {
        "x": x,
        "y": y,
        "yaw": yaw,
        "frame_id": located["source"],
        "child_frame_id": located["target"],
    }


_TYPE_CAPS = {
    "nav_msgs/msg/Odometry": 5000,
    "geometry_msgs/msg/PoseStamped": 5000,
    "geometry_msgs/msg/PoseWithCovarianceStamped": 5000,
    "sensor_msgs/msg/LaserScan": 600,
    "nav_msgs/msg/Path": 300,
    "custom_msgs_srvs/msg/RobotStatus": 2000,
    "custom_msgs_srvs/msg/TaskStatus": 2000,
    "geometry_msgs/msg/Twist": 2000,
    "tf2_msgs/msg/TFMessage": 6000,
    "sensor_msgs/msg/Image": 90,
}

def _is_supported_topic(topic_name: str, type_name: str) -> bool:
    if type_name not in _TYPE_CAPS:
        return False
    if type_name != "sensor_msgs/msg/Image":
        return True
    return topic_name.endswith(
        ("/front_camera/image_raw", "/front_down_camera/image_raw")
    )


def _database_files(path: Path) -> List[Path]:
    if path.is_dir():
        files = sorted(path.glob("*.db3"))
    elif path.is_file() and path.suffix.lower() == ".db3":
        files = [path]
    else:
        files = []
    if not files:
        raise BagReplayError("仅支持包含 .db3 的 rosbag2 目录")
    return files


def _open_database(path: Path) -> sqlite3.Connection:
    uri = "file:" + quote(str(path.resolve())) + "?mode=ro"
    connection = sqlite3.connect(uri, uri=True, timeout=3.0)
    connection.execute("PRAGMA query_only=ON")
    return connection

def _read_robot_status_sidecar(
    path: Path,
    start_ns: int,
    end_ns: int,
    robot_name: str,
) -> List[Dict[str, Any]]:
    bag_dir = path if path.is_dir() else path.parent
    sidecar = bag_dir / ".opendelivery_robot_status.json"
    if not sidecar.is_file():
        return []
    document = json.loads(sidecar.read_text(encoding="utf-8"))
    raw_rows = document.get("statuses")
    if not isinstance(raw_rows, list):
        raise BagReplayError("RobotStatus sidecar statuses must be an array")
    duration = max(0.0, (end_ns - start_ns) / 1_000_000_000.0)
    rows: List[Dict[str, Any]] = []
    for raw in raw_rows[:10000]:
        if not isinstance(raw, dict):
            continue
        timestamp_ns = int(raw.get("timestamp_ns"))
        relative = min(duration, max(0.0, (timestamp_ns - start_ns) / 1_000_000_000.0))
        rows.append(
            {
                "robot_name": str(raw.get("robot_name") or robot_name),
                "robot_model": str(raw.get("robot_model") or ""),
                "current_map": str(raw.get("current_map") or ""),
                "current_position": str(raw.get("current_position") or ""),
                "robot_status": str(raw.get("robot_status") or ""),
                "task_status": str(raw.get("task_status") or ""),
                "control_status": str(raw.get("control_status") or ""),
                "localization_method": str(raw.get("localization_method") or ""),
                "is_simulation": bool(raw.get("is_simulation", False)),
                "task_progress": float(raw.get("task_progress", -1.0)),
                "t": round(relative, 6),
                "topic": f"/{robot_name}/robot_status" if robot_name else "robot_status",
                "source": "recorder_sidecar",
            }
        )
    return rows


def _round_pose(pose: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "x": round(float(pose.get("x", 0.0)), 5),
        "y": round(float(pose.get("y", 0.0)), 5),
        "yaw": round(float(pose.get("yaw", 0.0)), 6),
    }


def extract_replay(path: Path, robot_name: str = "") -> Dict[str, Any]:
    """Extract a bounded browser-friendly playback timeline from a rosbag2 directory."""
    databases = _database_files(path)
    topic_rows: Dict[Tuple[str, str], Dict[str, Any]] = {}
    events: List[Tuple[int, str, str, bytes]] = []
    image_sources: Dict[Path, List[Tuple[int, str, str, int]]] = {}
    start_ns: Optional[int] = None
    end_ns: Optional[int] = None
    message_count = 0

    for database in databases:
        try:
            connection = _open_database(database)
        except sqlite3.Error as exc:
            raise BagReplayError(f"无法只读打开 rosbag2 数据库: {exc}") from exc
        try:
            schema = connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table' AND name IN ('topics','messages')"
            ).fetchall()
            if {row[0] for row in schema} != {"topics", "messages"}:
                raise BagReplayError("文件不是有效的 rosbag2 SQLite 数据库")
            bounds = connection.execute("SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages").fetchone()
            count = int(bounds[0] or 0)
            message_count += count
            if bounds[1] is not None:
                start_ns = int(bounds[1]) if start_ns is None else min(start_ns, int(bounds[1]))
                end_ns = int(bounds[2]) if end_ns is None else max(end_ns, int(bounds[2]))

            topics = connection.execute(
                "SELECT t.id, t.name, t.type, t.serialization_format, COUNT(m.id), MIN(m.id) "
                "FROM topics t LEFT JOIN messages m ON m.topic_id=t.id GROUP BY t.id ORDER BY t.name"
            ).fetchall()
            for topic_id, name, type_name, serialization, topic_count, first_id in topics:
                key = (str(name), str(type_name))
                summary = topic_rows.setdefault(
                    key,
                    {"name": str(name), "type": str(type_name), "count": 0, "supported": str(type_name) in _TYPE_CAPS},
                )
                summary["count"] += int(topic_count or 0)
                if not topic_count or type_name not in _TYPE_CAPS or serialization != "cdr":
                    continue
                if type_name == "sensor_msgs/msg/Image":
                    image_sources.setdefault(database, []).append(
                        (int(topic_id), str(type_name), str(name), int(topic_count))
                    )
                    continue
                cap = _TYPE_CAPS[str(type_name)]
                stride = max(1, int(math.ceil(int(topic_count) / cap)))
                query = (
                    "SELECT timestamp, data FROM messages WHERE topic_id=? "
                    "AND ((id - ?) % ?)=0 ORDER BY timestamp"
                )
                for timestamp, payload in connection.execute(query, (topic_id, int(first_id), stride)):
                    events.append((int(timestamp), str(type_name), str(name), bytes(payload)))
        except sqlite3.Error as exc:
            raise BagReplayError(f"读取 rosbag2 SQLite 失败: {exc}") from exc
        finally:
            connection.close()

    image_offsets: Dict[Tuple[str, str], int] = {}
    for database, sources in image_sources.items():
        try:
            connection = _open_database(database)
        except sqlite3.Error as exc:
            raise BagReplayError(f"无法只读打开 rosbag2 图像数据库: {exc}") from exc
        try:
            for topic_id, type_name, topic_name, topic_count in sources:
                key = (topic_name, type_name)
                total_count = int(topic_rows[key]["count"])
                stride = max(1, int(math.ceil(total_count / _TYPE_CAPS[type_name])))
                offset = image_offsets.get(key, 0)
                selected = [
                    (int(message_id), int(timestamp))
                    for local_index, (message_id, timestamp) in enumerate(
                        connection.execute(
                            "SELECT id, timestamp FROM messages WHERE topic_id=? ORDER BY timestamp",
                            (topic_id,),
                        )
                    )
                    if (offset + local_index) % stride == 0
                ]
                image_offsets[key] = offset + topic_count
                for chunk_start in range(0, len(selected), 300):
                    chunk = selected[chunk_start:chunk_start + 300]
                    placeholders = ",".join("?" for _ in chunk)
                    payloads = dict(
                        connection.execute(
                            f"SELECT id, data FROM messages WHERE id IN ({placeholders})",
                            tuple(message_id for message_id, _ in chunk),
                        )
                    )
                    for message_id, timestamp in chunk:
                        payload = payloads.get(message_id)
                        if payload is not None:
                            events.append((timestamp, type_name, topic_name, bytes(payload)))
        except sqlite3.Error as exc:
            raise BagReplayError(f"读取 rosbag2 图像失败: {exc}") from exc
        finally:
            connection.close()

    if not message_count or start_ns is None or end_ns is None:
        raise BagReplayError("bag 中没有可回放消息")

    events.sort(key=lambda row: row[0])
    poses_from_messages: List[Dict[str, Any]] = []
    poses_from_map_tf: List[Dict[str, Any]] = []
    poses_from_odom_tf: List[Dict[str, Any]] = []
    scans: List[Dict[str, Any]] = []
    paths: List[Dict[str, Any]] = []
    statuses: List[Dict[str, Any]] = []
    tasks: List[Dict[str, Any]] = []
    twists: List[Dict[str, Any]] = []
    images: List[Dict[str, Any]] = []
    warnings: List[str] = []
    tf_graph: Dict[Tuple[str, str], Tuple[float, float, float]] = {}

    # Static transforms are valid for the whole bag. Seed them first so a scan
    # can be placed correctly even when rosbag reception order puts /tf_static
    # after the first LaserScan sample.
    for _, type_name, topic_name, payload in events:
        if type_name != "tf2_msgs/msg/TFMessage" or not topic_name.rstrip("/").endswith("tf_static"):
            continue
        try:
            for transform in _decode_tf(payload):
                if transform["parent"] and transform["child"]:
                    tf_graph[(transform["parent"], transform["child"])] = (
                        transform["x"], transform["y"], transform["yaw"]
                    )
        except (BagReplayError, UnicodeError, ValueError, struct.error):
            pass

    def relative_time(timestamp: int) -> float:
        return round((timestamp - start_ns) / 1_000_000_000.0, 6)

    for timestamp, type_name, topic_name, payload in events:
        try:
            if type_name == "nav_msgs/msg/Odometry":
                decoded = _decode_odometry(payload)
                pose = _round_pose(decoded["pose"])
                pose.update(
                    {
                        "t": relative_time(timestamp),
                        "source": "odometry",
                        "frame_id": decoded["frame_id"],
                        "linear_x": round(decoded["linear"]["x"], 5),
                        "angular_z": round(decoded["angular"]["z"], 5),
                    }
                )
                poses_from_messages.append(pose)
            elif type_name in (
                "geometry_msgs/msg/PoseStamped",
                "geometry_msgs/msg/PoseWithCovarianceStamped",
            ):
                decoded = _decode_pose_stamped(
                    payload, type_name == "geometry_msgs/msg/PoseWithCovarianceStamped"
                )
                pose = _round_pose(decoded["pose"])
                pose.update({"t": relative_time(timestamp), "source": "pose", "frame_id": decoded["frame_id"]})
                poses_from_messages.append(pose)
            elif type_name == "sensor_msgs/msg/LaserScan":
                decoded = _decode_laser_scan(payload)
                _place_scan_in_tf_graph(decoded, tf_graph, robot_name)
                decoded.update({"t": relative_time(timestamp), "topic": topic_name})
                scans.append(decoded)
            elif type_name == "nav_msgs/msg/Path":
                decoded = _decode_path(payload)
                decoded.update({"t": relative_time(timestamp), "topic": topic_name})
                paths.append(decoded)
            elif type_name == "custom_msgs_srvs/msg/RobotStatus":
                decoded = _decode_robot_status(payload)
                decoded.update({"t": relative_time(timestamp), "topic": topic_name})
                statuses.append(decoded)
            elif type_name == "custom_msgs_srvs/msg/TaskStatus":
                decoded = _decode_task_status(payload)
                decoded.update({"t": relative_time(timestamp), "topic": topic_name})
                tasks.append(decoded)
            elif type_name == "geometry_msgs/msg/Twist":
                decoded = _decode_twist(payload)
                twists.append(
                    {
                        "t": relative_time(timestamp),
                        "topic": topic_name,
                        "linear_x": round(decoded["linear"]["x"], 5),
                        "angular_z": round(decoded["angular"]["z"], 5),
                    }
                )
            elif type_name == "sensor_msgs/msg/Image":
                decoded = _decode_image(payload)
                decoded.update(
                    {
                        "t": relative_time(timestamp),
                        "topic": topic_name,
                        "camera": "front_down"
                        if topic_name.endswith("/front_down_camera/image_raw")
                        else "front",
                    }
                )
                images.append(decoded)
            elif type_name == "tf2_msgs/msg/TFMessage":
                transforms = _decode_tf(payload)
                for transform in transforms:
                    if transform["parent"] and transform["child"]:
                        tf_graph[(transform["parent"], transform["child"])] = (
                            transform["x"], transform["y"], transform["yaw"]
                        )
                tf_pose = _find_tf_pose(tf_graph, robot_name)
                if tf_pose:
                    pose = _round_pose(tf_pose)
                    pose.update(
                        {
                            "t": relative_time(timestamp),
                            "source": "tf",
                            "frame_id": tf_pose["frame_id"],
                        }
                    )
                    if tf_pose["frame_id"] == "map" or tf_pose["frame_id"].endswith("/map"):
                        poses_from_map_tf.append(pose)
                    else:
                        poses_from_odom_tf.append(pose)
        except (BagReplayError, UnicodeError, ValueError, struct.error) as exc:
            warning = f"{topic_name}: {exc}"
            if warning not in warnings and len(warnings) < 12:
                warnings.append(warning)

    poses = poses_from_map_tf or poses_from_messages or poses_from_odom_tf
    poses.sort(key=lambda row: row["t"])
    scans.sort(key=lambda row: row["t"])
    paths.sort(key=lambda row: row["t"])
    try:
        statuses.extend(
            _read_robot_status_sidecar(path, start_ns, end_ns, robot_name)
        )
    except (BagReplayError, OSError, TypeError, ValueError) as exc:
        warning = f"RobotStatus sidecar: {exc}"
        if warning not in warnings and len(warnings) < 12:
            warnings.append(warning)

    statuses.sort(key=lambda row: row["t"])
    tasks.sort(key=lambda row: row["t"])
    twists.sort(key=lambda row: row["t"])
    images.sort(key=lambda row: row["t"])
    recorded_robot = next((row.get("robot_name") for row in statuses if row.get("robot_name")), "")
    map_changes: List[Dict[str, Any]] = []
    previous_map = ""
    for status in statuses:
        current_map = str(status.get("current_map") or "").strip()
        if not current_map or current_map == previous_map:
            continue
        map_changes.append(
            {
                "t": 0.0 if not map_changes else float(status["t"]),
                "current_map": current_map,
            }
        )
        previous_map = current_map
    initial_map_name = map_changes[0]["current_map"] if map_changes else ""
    map_name = map_changes[-1]["current_map"] if map_changes else ""

    return {
        "bag": str(path),
        "robot_name": recorded_robot or robot_name,
        "initial_map_name": initial_map_name,
        "map_name": map_name,
        "start_time_ns": start_ns,
        "duration": round(max(0, end_ns - start_ns) / 1_000_000_000.0, 6),
        "message_count": message_count,
        "database_count": len(databases),
        "topics": sorted(topic_rows.values(), key=lambda row: (-row["count"], row["name"])),
        "timeline": {
            "poses": poses,
            "scans": scans,
            "paths": paths,
            "statuses": statuses,
            "tasks": tasks,
            "twists": twists,
            "maps": map_changes,
            "images": images,
        },
        "warnings": warnings,
        "offline": True,
    }


def merge_replays(replays: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Sort and concatenate independently decoded bags into one bounded timeline."""
    if not replays:
        raise BagReplayError("至少需要一个可回放 bag")

    ordered = sorted(
        enumerate(replays),
        key=lambda item: (int(item[1].get("start_time_ns") or 0), item[0]),
    )
    timeline_names = ("poses", "scans", "paths", "statuses", "tasks", "twists", "images", "maps")
    merged_timeline: Dict[str, List[Dict[str, Any]]] = {name: [] for name in timeline_names}
    topic_rows: Dict[Tuple[str, str], Dict[str, Any]] = {}
    segments: List[Dict[str, Any]] = []
    warnings: List[str] = []
    cursor = 0.0

    for segment_index, (_, replay) in enumerate(ordered):
        duration = max(0.0, float(replay.get("duration") or 0.0))
        bag_name = str(replay.get("bag") or "")
        segment = {
            "index": segment_index,
            "bag": bag_name,
            "start": round(cursor, 6),
            "end": round(cursor + duration, 6),
            "duration": round(duration, 6),
            "source_start_time_ns": int(replay.get("start_time_ns") or 0),
            "robot_name": str(replay.get("robot_name") or ""),
            "initial_map_name": str(replay.get("initial_map_name") or ""),
            "map_name": str(replay.get("map_name") or ""),
            "message_count": int(replay.get("message_count") or 0),
        }
        segments.append(segment)

        source_timeline = replay.get("timeline") or {}
        for name in timeline_names:
            for sample in source_timeline.get(name) or []:
                row = dict(sample)
                row["t"] = round(cursor + max(0.0, float(sample.get("t") or 0.0)), 6)
                row["segment_index"] = segment_index
                row["bag"] = bag_name
                merged_timeline[name].append(row)

        for topic in replay.get("topics") or []:
            key = (str(topic.get("name") or ""), str(topic.get("type") or ""))
            summary = topic_rows.setdefault(
                key,
                {"name": key[0], "type": key[1], "count": 0, "supported": False},
            )
            summary["count"] += int(topic.get("count") or 0)
            summary["supported"] = bool(summary["supported"] or topic.get("supported"))

        label = Path(bag_name).name or f"bag {segment_index + 1}"
        warnings.extend(f"{label}: {warning}" for warning in replay.get("warnings") or [])
        cursor += duration

    first = ordered[0][1]
    robot_names = [str(item[1].get("robot_name") or "") for item in ordered]
    map_names = list(
        dict.fromkeys(
            str(row.get("current_map") or "")
            for row in merged_timeline["maps"]
            if row.get("current_map")
        )
    )
    initial_map_names = [
        str(item[1].get("initial_map_name") or "") for item in ordered
    ]
    initial_map_name = next((name for name in initial_map_names if name), "")
    return {
        "bag": str(first.get("bag") or ""),
        "bags": [segment["bag"] for segment in segments],
        "segments": segments,
        "robot_name": next((name for name in robot_names if name), ""),
        "initial_map_name": initial_map_name,
        "map_name": initial_map_name,
        "map_names": list(dict.fromkeys(name for name in map_names if name)),
        "start_time_ns": min(int(item[1].get("start_time_ns") or 0) for item in ordered),
        "duration": round(cursor, 6),
        "message_count": sum(int(item[1].get("message_count") or 0) for item in ordered),
        "database_count": sum(int(item[1].get("database_count") or 0) for item in ordered),
        "topics": sorted(topic_rows.values(), key=lambda row: (-row["count"], row["name"])),
        "timeline": merged_timeline,
        "warnings": warnings[:48],
        "offline": True,
    }
