#!/usr/bin/env python3
"""
Subscribe to standard TF topics (tf2_msgs/TFMessage), lookup map -> robot base_link.

Typical tree (shared /tf + often /tf_static):
  map
   ├── robot1/odom → robot1/base_link   # URDF may insert robot1/base_footprint between odom and base_link
   └── robot2/odom → robot2/base_link

Environment (single robot defaults):
  ROS_TF_TOPIC=/tf
  ROS_TF_STATIC_TOPIC=/tf_static   # set empty to disable (not recommended)
  ROS_FRAME_MAP=map
  ROS_FRAME_BASE=robot1/base_link
  ROS_ROBOT_ID=robot1
  ROS_ROBOT_NAME=robot1
  ROS_CURRENT_MAP=nh_102

Floor / map id for web filtering comes from ``custom_msgs_srvs/RobotStatus`` on
``/{robot_id}/robot_status`` (field ``current_map``). Until the first status message,
``ROS_CURRENT_MAP`` / JSON ``current_map`` on the spec is used. The web pose snapshot
exposes this as JSON field ``active_floor`` (not a ROS topic).

Multi-robot JSON (overrides default env-based list when set).
  tf_topic optional per entry — defaults to ROS_TF_TOPIC or /tf.
  ROS_ROBOTS_TF_JSON='[
    {"id":"robot1","name":"送餐-01","map_frame":"map","base_frame":"robot1/base_link","current_map":"nh_102"},
    {"id":"robot2","name":"送餐-02","map_frame":"map","base_frame":"robot2/base_link","current_map":"nh_103"}
  ]'

When JSON is **not** set: the bridge discovers robots only from ``/*/robot_status`` topics at runtime
(no hardcoded robot1/robot2 list).

Web commands (HTTP ``POST /api/robot/command`` → queue → this node):

- Publish ``geometry_msgs/PoseWithCovarianceStamped`` on ``/{robot_id}/initial`` (template:
  ``ROS_INITIAL_POSE_TOPIC_TEMPLATE`` default ``/{id}/initial``).
- Publish ``custom_msgs_srvs/RobotStatus`` on ``/{robot_id}/robot_status`` with updated
  ``current_map`` (switch floor; preserves ``robot_name`` / ``robot_status`` from last status when known).

Lost TF behavior:
  ROS_TF_LOST_POSE=hold   # default: keep last good x,y,yaw
  ROS_TF_LOST_POSE=zero  # use 0,0,0 when lookup fails
"""
from __future__ import annotations

import json
import math
import os
import re
import threading
import time
import base64
from typing import Any, Callable, Dict, List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image
from tf2_msgs.msg import TFMessage

try:
    # custom_msgs_srvs/RobotStatus: std_msgs/Header header + string robot_name
    from custom_msgs_srvs.msg import RobotStatus
except Exception:  # noqa: BLE001
    RobotStatus = None

import ros_command_queue
import ros_map_store
import ros_robot_status_store
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

    # Default mode: no hardcoded robots. Robot identities are discovered from
    # /<id>/robot_status at runtime, then robot-specific subscriptions are created.
    return []


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
        self._initial_pubs: Dict[str, Any] = {}
        self._robot_status_cmd_pubs: Dict[str, Any] = {}

        # Heartbeat-based liveness detection via /<robot_name>/robot_status.
        # When enabled, web-side identity is derived from discovered robot_status topics.
        self._robot_status_enabled = RobotStatus is not None
        self._require_robot_status = os.environ.get("ROS_REQUIRE_ROBOT_STATUS", "1").strip().lower() in (
            "1",
            "true",
            "yes",
        )
        self._robot_status_timeout_sec = float(os.environ.get("ROS_ROBOT_STATUS_TIMEOUT_SEC", "2.5"))
        self._robot_status_timeout_ns = int(self._robot_status_timeout_sec * 1_000_000_000)
        self._robot_status_topic_rx = re.compile(r"^/(.+)/robot_status$")
        self._robot_status_last_ns: Dict[str, int] = {}
        self._robot_status_topic_by_id: Dict[str, str] = {}
        self._robot_status_subs: Dict[str, Any] = {}
        self._robot_status_payload_by_id: Dict[str, Dict[str, Any]] = {}
        self._robot_specific_subs_enabled = False
        self._wait_enable_robot_specific_timer = None
        self._sub_scan: Dict[str, Any] = {}
        self._sub_path: Dict[str, Any] = {}
        self._sub_mapping: Dict[str, Any] = {}

        from tf2_ros import Buffer

        cache_sec = float(os.environ.get("ROS_TF_CACHE_SEC", "30"))
        self._buffer = Buffer(cache_time=Duration(seconds=cache_sec))

        topics = sorted({str(s["tf_topic"]) for s in robot_specs})
        if not topics:
            topics = [os.environ.get("ROS_TF_TOPIC", "/tf")]
        for tp in topics:
            self.create_subscription(TFMessage, tp, self._on_tf_message, 10)
            self.get_logger().info(f"subscribing TFMessage on {tp}")

        static_topic = os.environ.get("ROS_TF_STATIC_TOPIC", "/tf_static").strip()
        if static_topic and static_topic not in topics:
            self.create_subscription(TFMessage, static_topic, self._on_tf_message, 10)
            self.get_logger().info(f"subscribing TFMessage on {static_topic} (static)")

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # Gazebo global model states for lightweight top-down view.
        self.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self._on_gazebo_model_states,
            sensor_qos,
        )
        self.get_logger().info("subscribing ModelStates on /gazebo/model_states")

        # Optional top-down RGB camera stream.
        self._topdown_camera_topic = os.environ.get(
            "ROS_TOPDOWN_CAMERA_TOPIC", "/drawn_model/topdown_camera/image_raw"
        ).strip()
        if self._topdown_camera_topic:
            self.create_subscription(
                Image,
                self._topdown_camera_topic,
                self._on_topdown_image,
                sensor_qos,
            )
            self.get_logger().info(f"subscribing topdown image on {self._topdown_camera_topic}")

        scan_suffix = os.environ.get("ROS_SCAN_2D_TOPIC_SUFFIX", "/scan_2d")
        path_suffix = os.environ.get("ROS_PLANNED_PATH_TOPIC_SUFFIX", "/planned_path")

        rate = float(os.environ.get("ROS_TF_LOOKUP_HZ", "20"))
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._tick)

        cmd_hz = float(os.environ.get("ROS_COMMAND_TIMER_HZ", "20"))
        self._cmd_timer = self.create_timer(1.0 / max(cmd_hz, 1.0), self._process_commands)

        ros_command_queue.set_bridge_ready(True)
        self.get_logger().info("web command queue ready (initial pose + robot_status publishers)")

        # Discover and subscribe to /<robot_name>/robot_status topics dynamically when
        # the custom message type is available. But for the "no robot_status topics,
        # don't subscribe to robot-specific topics" requirement, we only need to check
        # topic names, not the message type.
        if self._robot_status_enabled:
            self.create_timer(1.0, self._discover_robot_status_topics)
            self.get_logger().info("enabled heartbeat topic discovery (/robot_status)")

        # If there is *no* robot_status topic at all, do not create robot-specific
        # subscriptions (scan_2d/planned_path/mapping). Enable them once
        # we observe at least one /*/robot_status topic.
        if not self._has_any_robot_status_topics():
            self.get_logger().info(
                "no /*/robot_status topics found yet; delaying robot-specific subscriptions"
            )
            self._wait_enable_robot_specific_timer = self.create_timer(
                1.0, self._wait_and_enable_robot_specific_subs
            )
        else:
            self._enable_robot_specific_subs(robot_specs, scan_suffix, path_suffix)

    def _has_any_robot_status_topics(self) -> bool:
        try:
            names_and_types = self.get_topic_names_and_types()
        except Exception:  # noqa: BLE001
            return False
        for name, _types in names_and_types:
            if self._robot_status_topic_rx.match(str(name)):
                return True
        return False

    def _wait_and_enable_robot_specific_subs(self) -> None:
        if self._robot_specific_subs_enabled:
            return
        if self._has_any_robot_status_topics():
            try:
                # Cancel the wait timer by destroying it.
                if self._wait_enable_robot_specific_timer is not None:
                    self.destroy_timer(self._wait_enable_robot_specific_timer)
            except Exception:  # noqa: BLE001
                pass
            self._wait_enable_robot_specific_timer = None
            self._enable_robot_specific_subs(self._specs, os.environ.get("ROS_SCAN_2D_TOPIC_SUFFIX", "/scan_2d"), os.environ.get("ROS_PLANNED_PATH_TOPIC_SUFFIX", "/planned_path"))

    def _enable_robot_specific_subs(
        self,
        robot_specs: List[Dict[str, Any]],
        scan_suffix: str,
        path_suffix: str,
    ) -> None:
        if self._robot_specific_subs_enabled:
            return
        self._robot_specific_subs_enabled = True

        for spec in robot_specs:
            self._ensure_robot_specific_subs_for_spec(spec, scan_suffix, path_suffix)

        # Global mapping for web.
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

    def _ensure_robot_specific_subs_for_spec(
        self,
        spec: Dict[str, Any],
        scan_suffix: str,
        path_suffix: str,
    ) -> None:
        rid = str(spec["id"])
        mf = str(spec["map_frame"])

        if (
            rid not in self._sub_scan
            and os.environ.get("ROS_SUBSCRIBE_SCAN_2D", "1").strip()
            not in ("0", "false", "no")
        ):
            scan_topic = str(spec.get("scan_2d_topic") or f"/{rid}{scan_suffix}")
            self._sub_scan[rid] = self.create_subscription(
                LaserScan,
                scan_topic,
                self._make_scan_cb(rid, mf),
                10,
            )
            self.get_logger().info(f"subscribing LaserScan on {scan_topic}")

        if (
            rid not in self._sub_path
            and os.environ.get("ROS_SUBSCRIBE_PLANNED_PATH", "1").strip()
            not in ("0", "false", "no")
        ):
            path_topic = str(spec.get("planned_path_topic") or f"/{rid}{path_suffix}")
            self._sub_path[rid] = self.create_subscription(
                Path,
                path_topic,
                self._make_path_cb(rid, mf),
                10,
            )
            self.get_logger().info(f"subscribing Path on {path_topic}")

        if (
            rid not in self._sub_mapping
            and os.environ.get("ROS_SUBSCRIBE_ROBOT_MAPPING", "1").strip()
            not in ("0", "false", "no")
        ):
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

            qos_map = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            )
            tpl = os.environ.get("ROS_MAPPING_TOPIC_TEMPLATE", "/{id}/mapping").strip()
            topic = str(spec.get("mapping_topic") or tpl.replace("{id}", rid))
            self._sub_mapping[rid] = self.create_subscription(
                OccupancyGrid,
                topic,
                self._make_mapping_grid_cb(rid),
                qos_map,
            )
            self.get_logger().info(f"subscribing OccupancyGrid (live mapping) on {topic}")

    def _make_robot_status_cb(self, rid: str, topic_name: str):
        def _cb(msg: RobotStatus) -> None:
            try:
                sec = int(getattr(msg.header.stamp, "sec", 0))
                nanosec = int(getattr(msg.header.stamp, "nanosec", 0))
                last_ns = sec * 1_000_000_000 + nanosec
                self._robot_status_last_ns[rid] = last_ns
                self._robot_status_topic_by_id[rid] = topic_name
                robot_name = str(getattr(msg, "robot_name", "") or rid).strip() or rid
                current_map = str(getattr(msg, "current_map", "") or "").strip()
                robot_status = str(getattr(msg, "robot_status", "") or "").strip()
                payload = {
                    "robot_name": robot_name,
                    "current_map": current_map,
                    "robot_status": robot_status,
                }
                self._robot_status_payload_by_id[rid] = payload
                ros_robot_status_store.set_last_status(
                    rid,
                    robot_name=robot_name,
                    current_map=current_map,
                    robot_status=robot_status,
                    topic=topic_name,
                    stamp_ns=last_ns,
                )
            except Exception:  # noqa: BLE001
                return

        return _cb

    def _ensure_robot_from_status_topic(self, rid: str, topic_name: str) -> None:
        # Create subscriptions for the discovered robot identity.
        if topic_name not in self._robot_status_subs:
            self._robot_status_subs[topic_name] = self.create_subscription(
                RobotStatus,
                topic_name,
                self._make_robot_status_cb(rid, topic_name),
                10,
            )
        self._robot_status_topic_by_id[rid] = topic_name

        # If this robot is not in self._specs yet, add a minimal spec so that /api/robot/pose
        # can include it (TF lookup; current_map from RobotStatus).
        if not any(str(s.get("id")) == rid for s in self._specs):
            map_frame = os.environ.get("ROS_FRAME_MAP", "map")
            current_map_default = os.environ.get("ROS_CURRENT_MAP", "nh_102")
            spec = {
                "id": rid,
                "name": rid,
                "tf_topic": os.environ.get("ROS_TF_TOPIC", "/tf"),
                "map_frame": map_frame,
                "base_frame": f"{rid}/base_link",
                "current_map": current_map_default,
            }
            self._specs.append(spec)

            self.get_logger().info(f"discovered robot {rid} from {topic_name}")
            self._ensure_cmd_pubs(rid)
            if self._robot_specific_subs_enabled:
                self._ensure_robot_specific_subs_for_spec(
                    spec,
                    os.environ.get("ROS_SCAN_2D_TOPIC_SUFFIX", "/scan_2d"),
                    os.environ.get("ROS_PLANNED_PATH_TOPIC_SUFFIX", "/planned_path"),
                )

            # Preload last-known status if this robot existed before restart.
            last = ros_robot_status_store.get_last_status(rid)
            if last:
                self._robot_status_payload_by_id[rid] = {
                    "robot_name": str(last.get("robot_name") or rid),
                    "current_map": str(last.get("current_map") or ""),
                    "robot_status": str(last.get("robot_status") or ""),
                }

    def _discover_robot_status_topics(self) -> None:
        if not self._robot_status_enabled:
            return
        try:
            topic_names_and_types = self.get_topic_names_and_types()
        except Exception:  # noqa: BLE001
            return

        # Expected type string for interface discovery.
        # (Exact type depends on build; keep the check permissive.)
        expected_suffix = "/robot_status"
        for name, types in topic_names_and_types:
            if not name.endswith(expected_suffix):
                continue
            m = self._robot_status_topic_rx.match(name)
            if not m:
                continue
            rid = str(m.group(1))
            # If types is available, prefer verifying it is our RobotStatus.
            if types:
                if not any(
                    (t == "custom_msgs_srvs/msg/RobotStatus") or str(t).endswith("/RobotStatus")
                    for t in types
                ):
                    continue
            self._ensure_robot_from_status_topic(rid, name)

    def _robot_status_cmd_topic(self, rid: str) -> str:
        return f"/{rid}/robot_status"

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
        if self._robot_status_enabled:
            mt = self._robot_status_cmd_topic(rid)
            self._robot_status_cmd_pubs[rid] = self.create_publisher(RobotStatus, mt, 10)
            self.get_logger().info(f"cmd publishers: {topic_i} + RobotStatus {mt}")
        else:
            self.get_logger().info(f"cmd publishers: {topic_i} (RobotStatus unavailable; map switch disabled)")

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
                if not self._robot_status_enabled or rid not in self._robot_status_cmd_pubs:
                    self.get_logger().warning(
                        f"skip map switch for {rid}: RobotStatus publisher not available"
                    )
                else:
                    payload = self._robot_status_payload_by_id.get(rid) or {}
                    st = RobotStatus()
                    st.header.stamp = self.get_clock().now().to_msg()
                    st.header.frame_id = mf
                    st.robot_name = str(payload.get("robot_name") or rid).strip() or rid
                    st.current_map = str(mn).strip()
                    st.robot_status = str(payload.get("robot_status") or "").strip()
                    self._robot_status_cmd_pubs[rid].publish(st)
                    self.get_logger().info(f"publish robot_status {rid} current_map -> {st.current_map!r}")

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

    def _on_gazebo_model_states(self, msg: ModelStates) -> None:
        try:
            models = []
            n = min(len(msg.name), len(msg.pose))
            for i in range(n):
                nm = str(msg.name[i] or "").strip()
                p = msg.pose[i]
                x = float(p.position.x)
                y = float(p.position.y)
                z = float(p.position.z)
                qx = float(p.orientation.x)
                qy = float(p.orientation.y)
                qz = float(p.orientation.z)
                qw = float(p.orientation.w)
                yaw = _yaw_from_quat(qx, qy, qz, qw)
                models.append(
                    {
                        "name": nm,
                        "pose": {
                            "x": x,
                            "y": y,
                            "z": z,
                            "yaw": yaw,
                            "qx": qx,
                            "qy": qy,
                            "qz": qz,
                            "qw": qw,
                        },
                    }
                )
            ros_sensor_store.set_gazebo_models(
                {
                    "available": True,
                    "models": models,
                    "source": "model_states_topic",
                    "stamp": time.time(),
                }
            )
        except Exception:  # noqa: BLE001
            return

    def _on_topdown_image(self, msg: Image) -> None:
        try:
            width = int(msg.width)
            height = int(msg.height)
            if width <= 0 or height <= 0:
                return
            enc = str(msg.encoding or "").lower()
            raw = bytes(msg.data or b"")
            if not raw:
                return
            if enc == "rgb8":
                rgb = raw
            elif enc == "bgr8":
                rgb = bytearray(len(raw))
                for i in range(0, len(raw), 3):
                    if i + 2 >= len(raw):
                        break
                    rgb[i] = raw[i + 2]
                    rgb[i + 1] = raw[i + 1]
                    rgb[i + 2] = raw[i]
                rgb = bytes(rgb)
            else:
                return
            ros_sensor_store.set_topdown_image(
                {
                    "available": True,
                    "width": width,
                    "height": height,
                    "encoding": "rgb8",
                    "data_b64": base64.b64encode(rgb).decode("ascii"),
                    "frame_id": str(msg.header.frame_id or ""),
                    "stamp_sec": int(msg.header.stamp.sec),
                    "stamp_nanosec": int(msg.header.stamp.nanosec),
                }
            )
        except Exception:  # noqa: BLE001
            return

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
        now_ns = self.get_clock().now().nanoseconds
        for spec in list(self._specs):
            rid = str(spec["id"])
            name = str(spec.get("name") or rid)
            mf = str(spec["map_frame"])
            bf = str(spec["base_frame"])

            # Heartbeat liveness (if enabled).
            hb_known = (
                rid in self._robot_status_topic_by_id
                or rid in self._robot_status_last_ns
            )
            if self._require_robot_status and self._robot_status_enabled and not hb_known:
                continue
            hb_ns = self._robot_status_last_ns.get(rid)
            hb_online = hb_ns is not None and (now_ns - hb_ns) <= self._robot_status_timeout_ns
            robot_status_topic = self._robot_status_topic_by_id.get(rid)

            # If we've identified this robot via heartbeat topic discovery and its heartbeat
            # has gone stale, do not include it in web output at all.
            if self._robot_status_enabled and hb_known and not hb_online:
                continue

            current_map = str(spec.get("current_map") or os.environ.get("ROS_CURRENT_MAP", "nh_102"))
            status_payload = self._robot_status_payload_by_id.get(rid) or {}
            status_current_map = str(status_payload.get("current_map") or "").strip()
            if status_current_map:
                current_map = status_current_map
            robot_name = str(status_payload.get("robot_name") or name or rid)
            robot_status = str(status_payload.get("robot_status") or "").strip()
            task_status = robot_status if robot_status else str(spec.get("task_status") or "Running")
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
                        "name": robot_name,
                        "frame_id": mf,
                        "active_floor": current_map,
                        "pose": pose,
                        "velocity": {"linear": 0.0, "angular": 0.0},
                        "localization": "ok",
                        "task_status": task_status,
                        "robot_status": robot_status,
                        "robot_status_topic": robot_status_topic,
                        "heartbeat_online": hb_online,
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
                        "name": robot_name,
                        "frame_id": mf,
                        "active_floor": current_map,
                        "pose": pose,
                        "velocity": {"linear": 0.0, "angular": 0.0},
                        "localization": "lost",
                        "task_status": task_status,
                        "robot_status": robot_status,
                        "robot_status_topic": robot_status_topic,
                        "heartbeat_online": hb_online,
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


def list_mapping_robot_ids_from_status_store(
    max_status_age_sec: Optional[float] = None,
) -> List[str]:
    """Robots whose persisted RobotStatus indicates mapping mode.

    Entries older than ``max_status_age_sec`` (default: env
    ``ROS_MAPPING_STATUS_MAX_AGE_SEC`` or 25s, or ``inf`` if env is ``inf``/``none``)
    by ``updated_at`` are ignored so removed robots do not keep ``{id}_mapping`` floors.
    """
    if max_status_age_sec is None:
        raw = os.environ.get("ROS_MAPPING_STATUS_MAX_AGE_SEC", "25").strip().lower()
        if raw in ("inf", "infinity", "none", "off"):
            max_status_age_sec = float("inf")
        else:
            max_status_age_sec = float(raw) if raw else 25.0
    now = time.time()
    out: List[str] = []
    for item in ros_robot_status_store.list_all_last_status():
        rid = str(item.get("robot_id") or "").strip()
        st = str(item.get("robot_status") or "").strip().lower()
        if not rid:
            continue
        if st != "mapping":
            continue
        if max_status_age_sec < float("inf"):
            updated_at = float(item.get("updated_at") or 0.0)
            if not updated_at or (now - updated_at) > max_status_age_sec:
                continue
        out.append(rid)
    return out
