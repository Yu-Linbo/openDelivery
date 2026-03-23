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

Multi-robot JSON (overrides single-robot env when set).
  tf_topic optional per entry — defaults to ROS_TF_TOPIC or /tf.
  ROS_ROBOTS_TF_JSON='[
    {"id":"robot1","name":"送餐-01","map_frame":"map","base_frame":"robot1/base_link","current_map":"nh_102"},
    {"id":"robot2","name":"送餐-02","map_frame":"map","base_frame":"robot2/base_link","current_map":"nh_103",
     "current_map_topic":"/robot2/current_map"}
  ]'

Optional per-robot ``current_map_topic``; default ``/{id}/current_map``.

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
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


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
    return [
        {
            "id": os.environ.get("ROS_ROBOT_ID", "robot1"),
            "name": os.environ.get("ROS_ROBOT_NAME", "robot1"),
            "tf_topic": os.environ.get("ROS_TF_TOPIC", "/tf"),
            "map_frame": os.environ.get("ROS_FRAME_MAP", "map"),
            "base_frame": os.environ.get("ROS_FRAME_BASE", "robot1/base_link"),
            "current_map": os.environ.get("ROS_CURRENT_MAP", "nh_102"),
        }
    ]


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

        rate = float(os.environ.get("ROS_TF_LOOKUP_HZ", "20"))
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._tick)

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


def run_ros_tf_bridge(publish_fn: Callable[[List[Dict[str, Any]], float], None]) -> None:
    specs = _load_robot_specs()
    rclpy.init(args=None)
    node = OpenDeliveryTfBridgeNode(robot_specs=specs, publish_fn=publish_fn)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def start_ros_tf_thread(publish_fn: Callable[[List[Dict[str, Any]], float], None]) -> threading.Thread:
    def _target() -> None:
        try:
            run_ros_tf_bridge(publish_fn)
        except Exception as ex:  # noqa: BLE001
            print(f"[ros_tf_bridge] fatal: {ex}")

    t = threading.Thread(target=_target, name="ros_tf_bridge", daemon=True)
    t.start()
    return t
