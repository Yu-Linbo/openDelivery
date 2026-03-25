#!/usr/bin/env python3
"""
Subscribe to standard TF topics (tf2_msgs/TFMessage), lookup map -> robot base_link.

Typical tree (shared /tf + often /tf_static):
  map
   ├── robot1/odom → robot1/base_link
   └── robot2/odom → robot2/base_link

Environment (single robot defaults):
  ROS_TF_TOPIC=/tf
  ROS_TF_STATIC_TOPIC=/tf_static   # set empty to disable (not recommended)
  ROS_FRAME_MAP=map
  ROS_FRAME_BASE=robot1/base_link
  ROS_ROBOT_ID=robot1
  ROS_ROBOT_NAME=robot1
  ROS_CURRENT_MAP=nh_102

Current map string for web floor filter: subscribe ``std_msgs/String`` on ``/{robot_id}/current_map``
(e.g. ``/robot1/current_map``). Until the first message, ``ROS_CURRENT_MAP`` / JSON ``current_map`` is used.
Set ``ROS_SUBSCRIBE_CURRENT_MAP_TOPIC=0`` to disable.

Multi-robot JSON (overrides default env-based list when set).
  tf_topic optional per entry — defaults to ROS_TF_TOPIC or /tf.
  ROS_ROBOTS_TF_JSON='[
    {"id":"robot1","name":"送餐-01","map_frame":"map","base_frame":"robot1/base_link","current_map":"nh_102"},
    {"id":"robot2","name":"送餐-02","map_frame":"map","base_frame":"robot2/base_link","current_map":"nh_103",
     "current_map_topic":"/robot2/current_map"}
  ]'

When JSON is **not** set: bridge loads ``ROS_ROBOT_ID`` (default robot1) **and** a second id
``ROS_SECOND_ROBOT_ID`` (default **robot2**) so Web / API match dual ``fake_pub`` topics.
Set ``ROS_TF_DISABLE_SECOND_ROBOT=1`` for a single-robot stack only.

Optional per-robot ``current_map_topic``; default ``/{id}/current_map``.

Web commands (HTTP ``POST /api/robot/command`` → queue → this node):

- Publish ``geometry_msgs/PoseWithCovarianceStamped`` on ``/{robot_id}/initial`` (template:
  ``ROS_INITIAL_POSE_TOPIC_TEMPLATE`` default ``/{id}/initial``).
- Publish ``std_msgs/String`` map id on the same ``current_map`` topic as subscribed (switch floor).

Lost TF behavior:
  ROS_TF_LOST_POSE=hold   # default: keep last good x,y,yaw
  ROS_TF_LOST_POSE=zero  # use 0,0,0 when lookup fails
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

import ros_command_queue
import ros_map_store
import ros_sensor_store


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _load_robot_specs() -> List[Dict[str, Any]]:
    raw = os.environ.get("ROS_ROBOTS_TF_JSON", "").strip()
    if raw:
        data = json.loads(raw)
        if not isinstance(data, list):
            raise ValueError("ROS_ROBOTS_TF_JSON must be a JSON array")
        default_tf = os.environ.get("ROS_TF_TOPIC", "/tf")
        required = ("id", "map_frame", "base_frame")
        normalized: List[Dict[str, Any]] = []
        for i, spec in enumerate(data):
            if not isinstance(spec, dict):
                raise ValueError(f"ROS_ROBOTS_TF_JSON[{i}] must be an object")
            for key in required:
                if key not in spec:
                    raise ValueError(f"ROS_ROBOTS_TF_JSON[{i}] missing {key!r}")
            topic = spec.get("tf_topic") or default_tf
            normalized.append({**spec, "tf_topic": str(topic)})
        return normalized

    first_id = os.environ.get("ROS_ROBOT_ID", "robot1")
    tf_topic = os.environ.get("ROS_TF_TOPIC", "/tf")
    map_frame = os.environ.get("ROS_FRAME_MAP", "map")
    base_default = os.environ.get("ROS_FRAME_BASE", f"{first_id}/base_link")
    current_map = os.environ.get("ROS_CURRENT_MAP", "nh_102")

    first: Dict[str, Any] = {
        "id": first_id,
        "name": os.environ.get("ROS_ROBOT_NAME", first_id),
        "tf_topic": tf_topic,
        "map_frame": map_frame,
        "base_frame": base_default,
        "current_map": current_map,
    }
    out: List[Dict[str, Any]] = [first]

    disable_second = os.environ.get("ROS_TF_DISABLE_SECOND_ROBOT", "").strip().lower() in (
        "1",
        "true",
        "yes",
    )
    second_raw = os.environ.get("ROS_SECOND_ROBOT_ID")
    if second_raw is None:
        second_id = "robot2"
    else:
        second_id = second_raw.strip()
    if not disable_second and second_id and second_id != first_id:
        cm2 = os.environ.get("ROS_CURRENT_MAP_2", current_map)
        out.append(
            {
                "id": second_id,
                "name": os.environ.get("ROS_ROBOT_NAME_2", second_id),
                "tf_topic": tf_topic,
                "map_frame": map_frame,
                "base_frame": f"{second_id}/base_link",
                "current_map": cm2,
            }
        )
    return out


class OpenDeliveryTfBridgeNode(Node):
    def __init__(
        self,
        robot_specs: List[Dict[str, Any]],
        publish_fn: Callable[[List[Dict[str, Any]], float], None],
    ) -> None:
        super().__init__("open_delivery_web_tf_bridge")
        self._specs = robot_specs
        self._publish_fn = publish_fn
        self._lost_mode = os.environ.get("ROS_TF_LOST_POSE", "hold").lower()
        self._last_good: Dict[str, Dict[str, Any]] = {}
        self._current_map_from_topic: Dict[str, str] = {}
        self._initial_pubs: Dict[str, Any] = {}
        self._map_cmd_pubs: Dict[str, Any] = {}

        from tf2_ros import Buffer

        cache_sec = float(os.environ.get("ROS_TF_CACHE_SEC", "30"))
        self._buffer = Buffer(cache_time=Duration(seconds=cache_sec))

        topics = sorted({str(s["tf_topic"]) for s in robot_specs})
        for tp in topics:
            self.create_subscription(TFMessage, tp, self._on_tf_message, 10)
            self.get_logger().info(f"subscribing TFMessage on {tp}")

        static_topic = os.environ.get("ROS_TF_STATIC_TOPIC", "/tf_static").strip()
        if static_topic and static_topic not in topics:
            self.create_subscription(TFMessage, static_topic, self._on_tf_message, 10)
            self.get_logger().info(f"subscribing TFMessage on {static_topic} (static)")

        if os.environ.get("ROS_SUBSCRIBE_CURRENT_MAP_TOPIC", "1").strip() not in (
            "0",
            "false",
            "no",
        ):
            for spec in robot_specs:
                rid = str(spec["id"])
                map_topic = str(spec.get("current_map_topic") or f"/{rid}/current_map")
                self.create_subscription(
                    String,
                    map_topic,
                    self._make_current_map_cb(rid),
                    10,
                )
                self.get_logger().info(f"subscribing String (current_map) on {map_topic}")

        scan_suffix = os.environ.get("ROS_SCAN_2D_TOPIC_SUFFIX", "/scan_2d")
        path_suffix = os.environ.get("ROS_PLANNED_PATH_TOPIC_SUFFIX", "/planned_path")
        if os.environ.get("ROS_SUBSCRIBE_SCAN_2D", "1").strip() not in ("0", "false", "no"):
            for spec in robot_specs:
                rid = str(spec["id"])
                mf = str(spec["map_frame"])
                scan_topic = str(spec.get("scan_2d_topic") or f"/{rid}{scan_suffix}")
                self.create_subscription(
                    LaserScan,
                    scan_topic,
                    self._make_scan_cb(rid, mf),
                    10,
                )
                self.get_logger().info(f"subscribing LaserScan on {scan_topic}")
        if os.environ.get("ROS_SUBSCRIBE_PLANNED_PATH", "1").strip() not in (
            "0",
            "false",
            "no",
        ):
            for spec in robot_specs:
                rid = str(spec["id"])
                mf = str(spec["map_frame"])
                path_topic = str(spec.get("planned_path_topic") or f"/{rid}{path_suffix}")
                self.create_subscription(
                    Path,
                    path_topic,
                    self._make_path_cb(rid, mf),
                    10,
                )
                self.get_logger().info(f"subscribing Path on {path_topic}")

        # Live mapping for web: one OccupancyGrid per robot at /<id>/mapping (override via
        # per-spec "mapping_topic" or env ROS_MAPPING_TOPIC_TEMPLATE default "/{id}/mapping").
        if os.environ.get("ROS_SUBSCRIBE_ROBOT_MAPPING", "1").strip() not in (
            "0",
            "false",
            "no",
        ):
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

            qos_map = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            tpl = os.environ.get("ROS_MAPPING_TOPIC_TEMPLATE", "/{id}/mapping").strip()
            for spec in robot_specs:
                rid = str(spec["id"])
                topic = str(spec.get("mapping_topic") or tpl.replace("{id}", rid))
                self.create_subscription(
                    OccupancyGrid,
                    topic,
                    self._make_mapping_grid_cb(rid),
                    qos_map,
                )
                self.get_logger().info(f"subscribing OccupancyGrid (live mapping) on {topic}")

        map_topic = os.environ.get("ROS_OCCUPANCY_MAP_TOPIC", "").strip()
        if map_topic and os.environ.get("ROS_SUBSCRIBE_GLOBAL_MAP", "0").strip() in (
            "1",
            "true",
            "yes",
        ):
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

            qos_map = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            rid_fallback = str(robot_specs[0]["id"]) if robot_specs else "robot1"
            self.create_subscription(
                OccupancyGrid,
                map_topic,
                self._make_mapping_grid_cb(rid_fallback),
                qos_map,
            )
            self.get_logger().info(f"subscribing OccupancyGrid (global) on {map_topic}")

        rate = float(os.environ.get("ROS_TF_LOOKUP_HZ", "20"))
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._tick)

        for spec in robot_specs:
            self._ensure_cmd_pubs(str(spec["id"]))
        cmd_hz = float(os.environ.get("ROS_COMMAND_TIMER_HZ", "20"))
        self._cmd_timer = self.create_timer(1.0 / max(cmd_hz, 1.0), self._process_commands)

        ros_command_queue.set_bridge_ready(True)
        self.get_logger().info("web command queue ready (initial pose + current_map publishers)")

    def _current_map_pub_topic(self, rid: str) -> str:
        for s in self._specs:
            if str(s["id"]) == rid:
                return str(s.get("current_map_topic") or f"/{rid}/current_map")
        return f"/{rid}/current_map"

    def _map_frame_for_robot(self, rid: str) -> str:
        for s in self._specs:
            if str(s["id"]) == rid:
                return str(s["map_frame"])
        return "map"

    def _ensure_cmd_pubs(self, rid: str) -> None:
        if rid in self._initial_pubs:
            return
        tpl_init = os.environ.get("ROS_INITIAL_POSE_TOPIC_TEMPLATE", "/{id}/initial")
        topic_i = tpl_init.replace("{id}", rid)
        self._initial_pubs[rid] = self.create_publisher(PoseWithCovarianceStamped, topic_i, 10)
        mt = self._current_map_pub_topic(rid)
        self._map_cmd_pubs[rid] = self.create_publisher(String, mt, 10)
        self.get_logger().info(f"cmd publishers: {topic_i} + String {mt}")

    def _process_commands(self) -> None:
        for cmd in ros_command_queue.drain_commands():
            try:
                self._handle_web_command(cmd)
            except Exception as ex:  # noqa: BLE001
                self.get_logger().error(f"web command failed: {ex}")

    def _handle_web_command(self, cmd: Dict[str, Any]) -> None:
        mode = str(cmd.get("mode") or "").strip()
        if mode not in ("map_only", "pose_only", "both"):
            self.get_logger().warning(f"ignore command: bad mode {mode!r}")
            return
        rid = str(cmd.get("robot_id") or "").strip()
        if not rid:
            self.get_logger().warning("ignore command: empty robot_id")
            return
        self._ensure_cmd_pubs(rid)
        mf = self._map_frame_for_robot(rid)

        if mode in ("map_only", "both"):
            mn = cmd.get("map_name")
            if mn is not None and str(mn).strip():
                m = String()
                m.data = str(mn).strip()
                self._map_cmd_pubs[rid].publish(m)
                self.get_logger().info(f"publish current_map {rid} -> {m.data!r}")

        if mode in ("pose_only", "both"):
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = mf
            msg.pose.pose.position.x = float(cmd["x"])
            msg.pose.pose.position.y = float(cmd["y"])
            msg.pose.pose.position.z = float(cmd.get("z") or 0.0)
            yaw = float(cmd.get("yaw") or 0.0)
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            cov = [0.0] * 36
            cov[0] = 0.25
            cov[7] = 0.25
            cov[35] = 0.068
            msg.pose.covariance = cov
            self._initial_pubs[rid].publish(msg)
            self.get_logger().info(
                f"publish initial pose {rid} map={mf} x={msg.pose.pose.position.x} "
                f"y={msg.pose.pose.position.y} yaw={yaw}"
            )

    def _make_mapping_grid_cb(self, robot_id: str):
        def _cb(msg: OccupancyGrid) -> None:
            if not msg.info:
                return
            w = int(msg.info.width)
            h = int(msg.info.height)
            if w <= 0 or h <= 0:
                return
            need = w * h
            if len(msg.data) < need:
                return
            raw = bytearray(need)
            for i in range(need):
                v = int(msg.data[i])
                if v < 0:
                    raw[i] = 205
                else:
                    raw[i] = min(255, max(0, int(255 - v * 255 // 100)))
            ox = float(msg.info.origin.position.x)
            oy = float(msg.info.origin.position.y)
            oz = float(msg.info.origin.position.z)
            qx = float(msg.info.origin.orientation.x)
            qy = float(msg.info.origin.orientation.y)
            qz = float(msg.info.origin.orientation.z)
            qw = float(msg.info.origin.orientation.w)
            yaw = _yaw_from_quat(qx, qy, qz, qw)
            ros_map_store.set_from_occupancy_grid(
                robot_id,
                w,
                h,
                float(msg.info.resolution),
                (ox, oy, oz),
                yaw,
                int(msg.header.stamp.sec),
                int(msg.header.stamp.nanosec),
                str(msg.header.frame_id or "map"),
                bytes(raw),
            )

        return _cb

    def _on_tf_message(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            try:
                self._buffer.set_transform(t, "open_delivery_web")
            except Exception as ex:  # noqa: BLE001
                self.get_logger().debug(f"set_transform skip: {ex}")

    def _make_current_map_cb(self, rid: str):
        def _cb(msg: String) -> None:
            text = (msg.data or "").strip()
            if text:
                self._current_map_from_topic[rid] = text

        return _cb

    def _make_scan_cb(self, rid: str, map_frame: str):
        stride = max(1, int(os.environ.get("ROS_SCAN_STRIDE", "3")))
        max_hits = max(50, int(os.environ.get("ROS_SCAN_MAX_HITS", "400")))

        def _cb(msg: LaserScan) -> None:
            try:
                tr = self._buffer.lookup_transform(
                    map_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except Exception:  # noqa: BLE001
                return
            qx = float(tr.transform.rotation.x)
            qy = float(tr.transform.rotation.y)
            qz = float(tr.transform.rotation.z)
            qw = float(tr.transform.rotation.w)
            yaw = _yaw_from_quat(qx, qy, qz, qw)
            tx = float(tr.transform.translation.x)
            ty = float(tr.transform.translation.y)
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            hits: List[List[float]] = []
            ranges = msg.ranges
            n = len(ranges)
            if n == 0:
                return
            step = max(stride, int(math.ceil(n / max_hits)))
            for i in range(0, n, step):
                if len(hits) >= max_hits:
                    break
                rng = float(ranges[i])
                if math.isnan(rng) or math.isinf(rng) or rng < msg.range_min or rng > msg.range_max:
                    continue
                a = float(msg.angle_min + i * msg.angle_increment)
                lx = rng * math.cos(a)
                ly = rng * math.sin(a)
                wx = cos_y * lx - sin_y * ly + tx
                wy = sin_y * lx + cos_y * ly + ty
                hits.append([round(wx, 3), round(wy, 3)])
            ros_sensor_store.set_scan(
                rid,
                {
                    "frame_id": map_frame,
                    "origin": [round(tx, 3), round(ty, 3)],
                    "hits": hits,
                },
            )

        return _cb

    def _make_path_cb(self, rid: str, map_frame: str):
        def _cb(msg: Path) -> None:
            pts: List[List[float]] = []
            for ps in msg.poses:
                if ps.header.frame_id != map_frame:
                    continue
                pts.append(
                    [
                        round(float(ps.pose.position.x), 3),
                        round(float(ps.pose.position.y), 3),
                    ]
                )
            ros_sensor_store.set_planned_path(
                rid, {"frame_id": map_frame, "points": pts}
            )

        return _cb

    def _tick(self) -> None:
        stamp = time.time()
        out: List[Dict[str, Any]] = []
        for spec in self._specs:
            rid = str(spec["id"])
            name = str(spec.get("name") or rid)
            mf = str(spec["map_frame"])
            bf = str(spec["base_frame"])
            current_map = self._current_map_from_topic.get(rid) or str(
                spec.get("current_map") or "nh_102"
            )
            try:
                tr = self._buffer.lookup_transform(mf, bf, Time(), timeout=Duration(seconds=0.05))
                x = float(tr.transform.translation.x)
                y = float(tr.transform.translation.y)
                q = tr.transform.rotation
                yaw = _yaw_from_quat(
                    float(q.x), float(q.y), float(q.z), float(q.w)
                )
                pose = {"x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)}
                self._last_good[rid] = {"pose": pose.copy()}
                out.append(
                    {
                        "id": rid,
                        "name": name,
                        "frame_id": mf,
                        "current_map": current_map,
                        "pose": pose,
                        "velocity": {"linear": 0.0, "angular": 0.0},
                        "localization": "ok",
                        "task_status": str(spec.get("task_status") or "Running"),
                    }
                )
            except Exception as ex:  # noqa: BLE001
                self.get_logger().debug(f"lookup {mf} <- {bf} failed: {ex}")
                if self._lost_mode == "zero":
                    pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                else:
                    prev = self._last_good.get(rid, {}).get("pose")
                    if prev:
                        pose = prev.copy()
                    else:
                        pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                out.append(
                    {
                        "id": rid,
                        "name": name,
                        "frame_id": mf,
                        "current_map": current_map,
                        "pose": pose,
                        "velocity": {"linear": 0.0, "angular": 0.0},
                        "localization": "lost",
                        "task_status": str(spec.get("task_status") or "Running"),
                    }
                )
        try:
            self._publish_fn(out, stamp)
        except Exception as ex:  # noqa: BLE001
            self.get_logger().error(f"publish_fn failed: {ex}")


def run_ros_tf_bridge(
    publish_fn: Callable[[List[Dict[str, Any]], float], None],
    stop_event: Optional[threading.Event] = None,
) -> None:
    specs = _load_robot_specs()
    rclpy.init(args=None)
    node = OpenDeliveryTfBridgeNode(robot_specs=specs, publish_fn=publish_fn)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not (stop_event and stop_event.is_set()):
            executor.spin_once(timeout_sec=0.05)
    finally:
        ros_command_queue.set_bridge_ready(False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def start_ros_tf_thread(
    publish_fn: Callable[[List[Dict[str, Any]], float], None],
    stop_event: Optional[threading.Event] = None,
) -> threading.Thread:
    def _target() -> None:
        try:
            run_ros_tf_bridge(publish_fn, stop_event=stop_event)
        except Exception as ex:  # noqa: BLE001
            print(f"[ros_tf_bridge] fatal: {ex}")

    t = threading.Thread(target=_target, name="ros_tf_bridge", daemon=True)
    t.start()
    return t


def request_ros_shutdown() -> None:
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
