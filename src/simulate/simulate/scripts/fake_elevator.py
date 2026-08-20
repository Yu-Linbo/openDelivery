#!/usr/bin/env python3
"""Temporary elevator/map-switch executor for simulation task orchestration."""

import os
import math
import json
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET
from urllib import error as urlerror
from urllib import request as urlrequest

from custom_msgs_srvs.msg import ElevatorCommand, ElevatorInfo, ElevatorStatus, RobotStatus
from custom_msgs_srvs.srv import Relocalize, SetHeartbeatParams
from geometry_msgs.msg import Pose
from nav2_msgs.srv import LoadMap
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


TERMINAL = {
    ElevatorStatus.STATUS_FINISHED,
    ElevatorStatus.STATUS_FAILED,
    ElevatorStatus.STATUS_TERMINATED,
}


class FakeElevator(Node):
    """Simulates elevator call/ride, moves Gazebo, switches map, and relocalizes."""

    def __init__(self):
        super().__init__("fake_elevator")
        self.declare_parameter("robot_name", "robot2")
        self.declare_parameter("map_root", os.environ.get("OPEN_DELIVERY_MAP_ROOT", ""))
        self.declare_parameter("call_delay_sec", 3.0)
        self.declare_parameter("ride_delay_sec", 1.0)
        self.declare_parameter("model_z", 0.05)
        self.declare_parameter(
            "world_path", os.environ.get("OPEN_DELIVERY_GAZEBO_WORLD", "")
        )
        self.declare_parameter("relocalize_retry_count", 10)
        self.declare_parameter("relocalize_retry_delay_sec", 0.3)
        self.declare_parameter("map_status_propagation_delay_sec", 0.5)
        self.declare_parameter("service_wait_sec", 10.0)
        self.declare_parameter(
            "web_api_base", os.environ.get("OPEN_DELIVERY_API", "http://127.0.0.1:8001")
        )
        self.declare_parameter("web_request_timeout_sec", 30.0)
        self.declare_parameter("map_frame", "map")
        self._robot = str(self.get_parameter("robot_name").value).strip().strip("/") or "robot2"
        self._map_root = Path(str(self.get_parameter("map_root").value).strip())
        self._call_delay = max(0.0, float(self.get_parameter("call_delay_sec").value))
        self._delay = max(0.0, float(self.get_parameter("ride_delay_sec").value))
        self._model_z = float(self.get_parameter("model_z").value)
        self._world_path = Path(str(self.get_parameter("world_path").value).strip())
        self._retry_count = max(1, int(self.get_parameter("relocalize_retry_count").value))
        self._retry_delay = max(
            0.05, float(self.get_parameter("relocalize_retry_delay_sec").value)
        )
        self._map_status_delay = max(
            0.05, float(self.get_parameter("map_status_propagation_delay_sec").value)
        )
        self._service_wait = max(0.1, float(self.get_parameter("service_wait_sec").value))
        self._web_api_base = str(
            self.get_parameter("web_api_base").value
        ).strip().rstrip("/") or "http://127.0.0.1:8001"
        self._web_timeout = max(
            1.0, float(self.get_parameter("web_request_timeout_sec").value)
        )
        self._map_frame = str(self.get_parameter("map_frame").value).strip() or "map"
        status_qos = QoSProfile(depth=10)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(ElevatorStatus, "fake_elevator/status", status_qos)
        self.create_subscription(ElevatorInfo, "fake_elevator/info", self._on_info, 10)
        self.create_subscription(ElevatorCommand, "fake_elevator/command", self._on_command, 10)
        self.create_subscription(RobotStatus, "robot_status", self._on_robot_status, 10)
        self._load_map = self.create_client(LoadMap, f"/{self._robot}/map_server/load_map")
        self._heartbeat = self.create_client(
            SetHeartbeatParams, f"/{self._robot}/set_heartbeat_params"
        )
        self._relocalize = self.create_client(Relocalize, f"/{self._robot}/relocalize")
        self._lock = threading.RLock()
        self._info = None
        self._status = ElevatorStatus.STATUS_WAITING
        self._message = "waiting for elevator task"
        self._generation = 0
        self._timer = None
        self._observed_floor = ""
        self._publish()

    def _on_robot_status(self, msg):
        with self._lock:
            self._observed_floor = str(msg.current_map).strip()

    @staticmethod
    def _yaw(pose):
        q = pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _pgm_size(path):
        """Read a PGM header without adding image-library runtime dependencies."""
        tokens = []
        with path.open("rb") as stream:
            while len(tokens) < 4:
                line = stream.readline()
                if not line:
                    break
                tokens.extend(line.split(b"#", 1)[0].split())
        if len(tokens) < 4 or tokens[0] not in (b"P2", b"P5"):
            raise RuntimeError(f"invalid PGM map: {path}")
        width, height = int(tokens[1]), int(tokens[2])
        if width <= 0 or height <= 0:
            raise RuntimeError(f"invalid PGM dimensions: {path}")
        return width, height

    @staticmethod
    def _map_metadata(yaml_path):
        values = {}
        for raw in yaml_path.read_text(encoding="utf-8").splitlines():
            line = raw.split("#", 1)[0].strip()
            if ":" in line:
                key, value = line.split(":", 1)
                values[key.strip()] = value.strip()
        try:
            resolution = float(values["resolution"])
            origin = [float(v.strip()) for v in values["origin"].strip("[]").split(",")]
            image = values["image"].strip("'\"")
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"invalid map metadata: {yaml_path}") from exc
        if resolution <= 0.0 or len(origin) < 3:
            raise RuntimeError(f"invalid map metadata: {yaml_path}")
        image_path = Path(image)
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        width, height = FakeElevator._pgm_size(image_path)
        return resolution, origin[:3], width, height

    def _floor_world_pose(self, floor):
        if not self._world_path.is_file():
            raise RuntimeError(f"Gazebo world file missing: {self._world_path}")
        try:
            root = ET.parse(str(self._world_path)).getroot()
        except (OSError, ET.ParseError) as exc:
            raise RuntimeError(f"invalid Gazebo world file {self._world_path}: {exc}") from exc
        matches = []
        for include in root.findall(".//include"):
            name = (include.findtext("name") or "").strip()
            uri = (include.findtext("uri") or "").strip()
            if name == floor or uri == f"model://{floor}":
                raw = (include.findtext("pose") or "0 0 0 0 0 0").split()
                if len(raw) != 6:
                    raise RuntimeError(f"invalid Gazebo pose for floor {floor}")
                try:
                    matches.append(tuple(float(value) for value in raw))
                except ValueError as exc:
                    raise RuntimeError(f"invalid Gazebo pose for floor {floor}") from exc
        if len(matches) != 1:
            raise RuntimeError(
                f"Gazebo world requires exactly one {floor} model, found {len(matches)}"
            )
        return matches[0]

    def _target_world_pose(self, floor, target):
        """Convert map pose using the centered-STL floor generation contract."""
        yaml_path = self._map_root / floor / f"{floor}.yaml"
        try:
            resolution, origin, width, height = self._map_metadata(yaml_path)
            floor_x, floor_y, _z, _roll, _pitch, floor_yaw = self._floor_world_pose(floor)
        except (OSError, RuntimeError) as exc:
            self._pose_conversion_error = str(exc)
            return None
        map_yaw = origin[2]
        center_x = origin[0] + math.cos(map_yaw) * width * resolution / 2.0 \
            - math.sin(map_yaw) * height * resolution / 2.0
        center_y = origin[1] + math.sin(map_yaw) * width * resolution / 2.0 \
            + math.cos(map_yaw) * height * resolution / 2.0
        dx = target.position.x - center_x
        dy = target.position.y - center_y
        relative_yaw = floor_yaw - map_yaw
        co = math.cos(relative_yaw)
        so = math.sin(relative_yaw)
        result = Pose()
        result.position.x = floor_x + co * dx - so * dy
        result.position.y = floor_y + so * dx + co * dy
        result.position.z = self._model_z
        yaw = self._yaw(target) + relative_yaw
        result.orientation.z = math.sin(yaw / 2.0)
        result.orientation.w = math.cos(yaw / 2.0)
        self._pose_conversion_error = ""
        return result

    def _web_json(self, path, payload=None):
        body = None
        headers = {}
        method = "GET"
        if payload is not None:
            body = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
            method = "POST"
        req = urlrequest.Request(
            self._web_api_base + path, data=body, headers=headers, method=method
        )
        try:
            with urlrequest.urlopen(req, timeout=self._web_timeout) as response:
                raw = response.read().decode("utf-8")
                data = json.loads(raw or "{}")
        except urlerror.HTTPError as exc:
            try:
                detail = json.loads(exc.read().decode("utf-8")).get("error", "")
            except Exception:  # noqa: BLE001
                detail = ""
            raise RuntimeError(
                f"Web API {path} failed ({exc.code}): {detail or exc.reason}"
            ) from exc
        except (urlerror.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"Web API {path} unavailable: {exc}") from exc
        if not isinstance(data, dict):
            raise RuntimeError(f"Web API {path} returned a non-object response")
        return data

    def _target_inside_pose(self, floor):
        points_path = self._map_root / floor / f"{floor}_points.json"
        try:
            data = json.loads(points_path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise RuntimeError(f"elevator points file missing: {points_path}") from exc
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"invalid elevator points file {points_path}: {exc}") from exc
        rows = data.get("points") if isinstance(data, dict) else None
        if not isinstance(rows, list):
            raise RuntimeError(f"elevator points file has no points array: {points_path}")
        matches = [
            row for row in rows
            if isinstance(row, dict) and
            str(row.get("type") or "").strip().lower() in ("elevator", "elevator_inside")
        ]
        if len(matches) != 1:
            raise RuntimeError(
                f"{floor} requires exactly one elevator_inside point, found {len(matches)}"
            )
        row = matches[0]
        try:
            x = float(row["x"])
            y = float(row["y"])
            yaw = float(row.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(f"{floor} elevator_inside point is invalid") from exc
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise RuntimeError(f"{floor} elevator_inside point must be finite")
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _move_model_through_web(self, pose):
        yaw = self._yaw(pose)
        response = self._web_json(
            "/api/gazebo/set_model_state",
            {
                "model_name": self._robot,
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "z": float(self._model_z),
                "yaw": float(yaw),
                "reference_frame": "world",
            },
        )
        if not response.get("ok"):
            raise RuntimeError(
                "Web set_model_state returned failure: " +
                str(response.get("error") or response)
            )
        return response

    def _on_info(self, msg):
        operation = str(msg.operation).strip().lower()
        if not str(msg.task_id).strip() or operation not in (
                ElevatorInfo.OPERATION_CALL, ElevatorInfo.OPERATION_RIDE):
            with self._lock:
                self._info = msg
                self._generation += 1
                self._status = ElevatorStatus.STATUS_FAILED
                self._message = "task_id and operation=call|ride are required"
                self._publish()
            return
        if operation == ElevatorInfo.OPERATION_RIDE and not str(msg.target_floor).strip():
            with self._lock:
                self._info = msg
                self._generation += 1
                self._status = ElevatorStatus.STATUS_FAILED
                self._message = "ride operation requires target_floor"
                self._publish()
            return
        with self._lock:
            self._cancel_timer()
            self._info = msg
            self._generation += 1
            generation = self._generation
            self._schedule_operation(generation)

    def _on_command(self, msg):
        command = str(msg.command).strip().lower()
        with self._lock:
            if not self._info or str(msg.task_id).strip() != self._info.task_id:
                return
            if command == ElevatorCommand.COMMAND_TERMINATE.lower():
                if self._status in TERMINAL:
                    return
                self._generation += 1
                self._cancel_timer()
                self._status = ElevatorStatus.STATUS_TERMINATED
                self._message = "elevator task terminated"
                self._publish()
                return
            if command == ElevatorCommand.COMMAND_PAUSE.lower():
                if self._status in TERMINAL or self._status == ElevatorStatus.STATUS_PAUSED:
                    return
                self._generation += 1
                self._cancel_timer()
                self._status = ElevatorStatus.STATUS_PAUSED
                self._message = "elevator task paused"
                self._publish()
                return
            if command != ElevatorCommand.COMMAND_RESUME.lower() or \
                    self._status != ElevatorStatus.STATUS_PAUSED:
                return
            self._generation += 1
            generation = self._generation
            self._schedule_operation(generation, resumed=True)

    def _schedule_operation(self, generation, resumed=False):
        operation = str(self._info.operation).strip().lower()
        if operation == ElevatorInfo.OPERATION_CALL:
            self._status = ElevatorStatus.STATUS_CALLING
            self._message = "calling fake elevator" if not resumed else "elevator call resumed"
            delay = self._call_delay
        else:
            self._status = ElevatorStatus.STATUS_RIDING
            self._message = "simulating elevator ride" if not resumed else "elevator ride resumed"
            delay = self._delay
        self._publish()
        self._timer = self.create_timer(
            max(0.01, delay), lambda: self._after_delay(generation)
        )

    def _cancel_timer(self):
        if self._timer is not None:
            try:
                self.destroy_timer(self._timer)
            except Exception:  # noqa: BLE001
                pass
            self._timer = None

    def _after_delay(self, generation):
        with self._lock:
            self._cancel_timer()
            if generation != self._generation or not self._info:
                return
            if str(self._info.operation).strip().lower() == ElevatorInfo.OPERATION_CALL:
                self._finish(
                    generation, ElevatorStatus.STATUS_FINISHED,
                    f"fake elevator arrived at {self._info.from_floor}",
                )
                return
            target = str(self._info.target_floor).strip()
            yaml_path = self._map_root / target / f"{target}.yaml"
            if not yaml_path.is_file():
                self._finish(
                    generation, ElevatorStatus.STATUS_FAILED,
                    f"map yaml missing: {yaml_path}",
                )
                return
            self._status = ElevatorStatus.STATUS_MOVING_MODEL
            self._message = f"moving Gazebo model to {self._info.target_floor} elevator interior"
            self._publish()
            try:
                target_inside_pose = self._target_inside_pose(target)
            except RuntimeError as exc:
                self._finish(generation, ElevatorStatus.STATUS_FAILED, str(exc))
                return
            target_pose = self._target_world_pose(target, target_inside_pose)
        if target_pose is None:
            self._finish(
                generation,
                ElevatorStatus.STATUS_FAILED,
                f"map-to-world conversion failed: {self._pose_conversion_error}",
            )
            return
        required_services = (
            (self._heartbeat, "heartbeat"),
            (self._load_map, "map_server load_map"),
            (self._relocalize, "relocalize"),
        )
        unavailable = next(
            (name for client, name in required_services
             if not client.wait_for_service(timeout_sec=self._service_wait)),
            "",
        )
        if unavailable:
            self._finish(
                generation, ElevatorStatus.STATUS_FAILED,
                f"{unavailable} service unavailable",
            )
            return
        try:
            response = self._move_model_through_web(target_pose)
        except RuntimeError as exc:
            self._finish(
                generation, ElevatorStatus.STATUS_FAILED,
                f"Web Gazebo move failed: {exc}",
            )
            return
        self.get_logger().info(
            f"Web Gazebo move succeeded: {self._robot} -> "
            f"x={float(response.get('x', target_pose.position.x)):.3f} "
            f"y={float(response.get('y', target_pose.position.y)):.3f}"
        )
        self._switch_map(generation)

    def _switch_map(self, generation):
        with self._lock:
            if generation != self._generation or not self._info:
                return
            target = str(self._info.target_floor).strip()
            yaml_path = self._map_root / target / f"{target}.yaml"
            if not yaml_path.is_file():
                self._finish(generation, ElevatorStatus.STATUS_FAILED, f"map yaml missing: {yaml_path}")
                return
            self._status = ElevatorStatus.STATUS_SWITCHING_MAP
            self._message = f"loading map {target}"
            self._publish()
        if not self._heartbeat.wait_for_service(timeout_sec=self._service_wait):
            self._finish(generation, ElevatorStatus.STATUS_FAILED, "heartbeat service unavailable")
            return
        request = SetHeartbeatParams.Request()
        request.current_map = target
        request.robot_status = "localization_lost"
        request.rate_hz = 0.0
        request.task_progress = -1.0
        self._heartbeat.call_async(request).add_done_callback(
            lambda future: self._map_status_updated(future, generation, str(yaml_path))
        )

    def _map_loaded(self, future, generation):
        if not self._active(generation):
            return
        try:
            response = future.result()
            if response is None or int(response.result) != int(LoadMap.Response.RESULT_SUCCESS):
                result = getattr(response, "result", "empty")
                self._rollback_floor(generation, f"load_map failed: {result}")
                return
        except Exception as exc:  # noqa: BLE001
            self._rollback_floor(generation, f"load_map exception: {exc}")
            return
        # LoadMap response can arrive before RobotStatus/current_map and the new
        # OccupancyGrid callbacks reach the relocalization node.  Defer the first
        # request so it cannot accidentally match against the previous floor.
        self._schedule_relocalize_retry(
            generation, 1, "waiting for target map propagation"
        )

    def _map_status_updated(self, future, generation, yaml_path):
        if not self._active(generation):
            return
        try:
            response = future.result()
            if response is None or not response.success:
                detail = response.message if response is not None else "empty response"
                self._finish(generation, ElevatorStatus.STATUS_FAILED, f"heartbeat update failed: {detail}")
                return
        except Exception as exc:  # noqa: BLE001
            self._finish(generation, ElevatorStatus.STATUS_FAILED, f"heartbeat exception: {exc}")
            return
        # Service success only means the update was accepted.  Wait until this
        # node observes the target RobotStatus, then allow other subscribers the
        # normal propagation delay before publishing the new OccupancyGrid.
        with self._lock:
            if generation != self._generation:
                return
            self._message = "waiting to observe target floor before loading map"
            self._publish()
            self._cancel_timer()
            self._timer = self.create_timer(
                0.1,
                lambda: self._wait_for_target_floor(
                    generation, yaml_path, time.monotonic()
                ),
            )

    def _wait_for_target_floor(self, generation, yaml_path, started_at):
        with self._lock:
            self._cancel_timer()
            if generation != self._generation or not self._info:
                return
            target = str(self._info.target_floor).strip()
            observed = self._observed_floor
            elapsed = time.monotonic() - started_at
            if observed == target:
                self._message = "target floor observed; waiting for subscriber propagation"
                self._publish()
                self._timer = self.create_timer(
                    self._map_status_delay,
                    lambda: self._load_target_map(generation, yaml_path),
                )
                return
            if elapsed >= self._service_wait:
                self._finish(
                    generation, ElevatorStatus.STATUS_FAILED,
                    f"target floor status not observed: expected {target}, "
                    f"got {observed or 'empty'}",
                )
                return
            self._timer = self.create_timer(
                0.1,
                lambda: self._wait_for_target_floor(generation, yaml_path, started_at),
            )

    def _load_target_map(self, generation, yaml_path):
        with self._lock:
            self._cancel_timer()
            if generation != self._generation:
                return
        if not self._load_map.wait_for_service(timeout_sec=self._service_wait):
            self._rollback_floor(
                generation, "map_server load_map unavailable after heartbeat floor update"
            )
            return
        request = LoadMap.Request()
        request.map_url = yaml_path
        self._load_map.call_async(request).add_done_callback(
            lambda completed: self._map_loaded(completed, generation)
        )

    def _begin_relocalize(self, generation, attempt):
        with self._lock:
            if generation != self._generation or not self._info:
                return
            self._status = ElevatorStatus.STATUS_RELOCALIZING
            self._message = "running scan relocalization"
            self._publish()
            info = self._info
        if not self._relocalize.wait_for_service(timeout_sec=self._service_wait):
            self._finish(generation, ElevatorStatus.STATUS_FAILED, "relocalize service unavailable")
            return
        request = Relocalize.Request()
        request.mode = (
            Relocalize.Request.MODE_POSE_FIRST
            if info.use_pose_first else Relocalize.Request.MODE_HISTORY
        )
        request.pose = info.relocalization_pose
        request.pose.header.frame_id = self._map_frame
        self._relocalize.call_async(request).add_done_callback(
            lambda future: self._relocalized(future, generation, attempt)
        )

    def _relocalized(self, future, generation, attempt):
        if not self._active(generation):
            return
        try:
            response = future.result()
            if response is None or not response.success:
                detail = response.message if response is not None else "empty response"
                if attempt < self._retry_count and self._retryable_relocalize_error(detail):
                    self._schedule_relocalize_retry(generation, attempt + 1, detail)
                    return
                self._finish(
                    generation, ElevatorStatus.STATUS_FAILED,
                    f"relocalize failed after {attempt} attempt(s): {detail}",
                )
                return
            self._finish(
                generation, ElevatorStatus.STATUS_FINISHED,
                f"map switched and relocalized score={response.score:.3f}",
            )
        except Exception as exc:  # noqa: BLE001
            self._finish(generation, ElevatorStatus.STATUS_FAILED, f"relocalize exception: {exc}")

    @staticmethod
    def _retryable_relocalize_error(message):
        text = str(message).lower()
        return any(token in text for token in (
            "occupancygrid is not ready", "no fresh scan_2d frame",
        ))

    def _schedule_relocalize_retry(self, generation, attempt, detail):
        with self._lock:
            if generation != self._generation:
                return
            self._message = (
                f"waiting for target map/scan before relocalization "
                f"({attempt}/{self._retry_count}): {detail}"
            )
            self._publish()
            self._cancel_timer()
            self._timer = self.create_timer(
                self._retry_delay, lambda: self._retry_relocalize(generation, attempt)
            )

    def _retry_relocalize(self, generation, attempt):
        with self._lock:
            self._cancel_timer()
        self._begin_relocalize(generation, attempt)

    def _rollback_floor(self, generation, reason):
        if not self._active(generation):
            return
        with self._lock:
            from_floor = str(self._info.from_floor).strip()
        if not from_floor or not self._heartbeat.wait_for_service(timeout_sec=self._service_wait):
            self._finish(generation, ElevatorStatus.STATUS_FAILED, reason)
            return
        request = SetHeartbeatParams.Request()
        request.current_map = from_floor
        request.rate_hz = 0.0
        request.task_progress = -1.0
        self._heartbeat.call_async(request).add_done_callback(
            lambda future: self._rollback_finished(future, generation, reason)
        )

    def _rollback_finished(self, future, generation, reason):
        suffix = ""
        try:
            response = future.result()
            if response is None or not response.success:
                suffix = "; heartbeat floor rollback failed"
        except Exception as exc:  # noqa: BLE001
            suffix = f"; heartbeat floor rollback exception: {exc}"
        self._finish(generation, ElevatorStatus.STATUS_FAILED, reason + suffix)

    def _active(self, generation):
        with self._lock:
            return generation == self._generation and self._info is not None

    def _finish(self, generation, status, message):
        with self._lock:
            if generation != self._generation:
                return
            self._status = status
            self._message = message
            self._publish()

    def _publish(self):
        msg = ElevatorStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.task_id = self._info.task_id if self._info else ""
        msg.elevator_status = self._status
        msg.message = self._message
        # These diagnostic fields were added after the initial ElevatorStatus
        # schema.  Keep the executor alive if an older generated message is
        # still sourced during a rolling/incremental workspace rebuild.
        if hasattr(msg, "operation"):
            msg.operation = self._info.operation if self._info else ""
        if hasattr(msg, "from_floor"):
            msg.from_floor = self._info.from_floor if self._info else ""
        if hasattr(msg, "target_floor"):
            msg.target_floor = self._info.target_floor if self._info else ""
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeElevator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
