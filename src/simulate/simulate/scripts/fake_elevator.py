#!/usr/bin/env python3
"""Temporary elevator/map-switch executor for simulation task orchestration."""

import os
from pathlib import Path
import threading

from custom_msgs_srvs.msg import ElevatorCommand, ElevatorInfo, ElevatorStatus
from custom_msgs_srvs.srv import Relocalize, SetHeartbeatParams
from gazebo_msgs.srv import SetModelState
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
        self.declare_parameter("relocalize_retry_count", 10)
        self.declare_parameter("relocalize_retry_delay_sec", 0.3)
        self.declare_parameter("service_wait_sec", 10.0)
        self.declare_parameter("map_frame", "map")
        self._robot = str(self.get_parameter("robot_name").value).strip().strip("/") or "robot2"
        self._map_root = Path(str(self.get_parameter("map_root").value).strip())
        self._call_delay = max(0.0, float(self.get_parameter("call_delay_sec").value))
        self._delay = max(0.0, float(self.get_parameter("ride_delay_sec").value))
        self._model_z = float(self.get_parameter("model_z").value)
        self._retry_count = max(1, int(self.get_parameter("relocalize_retry_count").value))
        self._retry_delay = max(
            0.05, float(self.get_parameter("relocalize_retry_delay_sec").value)
        )
        self._service_wait = max(0.1, float(self.get_parameter("service_wait_sec").value))
        self._map_frame = str(self.get_parameter("map_frame").value).strip() or "map"
        status_qos = QoSProfile(depth=10)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(ElevatorStatus, "fake_elevator/status", status_qos)
        self.create_subscription(ElevatorInfo, "fake_elevator/info", self._on_info, 10)
        self.create_subscription(ElevatorCommand, "fake_elevator/command", self._on_command, 10)
        self._load_map = self.create_client(LoadMap, f"/{self._robot}/map_server/load_map")
        self._set_model_state = self.create_client(SetModelState, "/gazebo/set_model_state")
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
        self._publish()

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
            target_pose = self._info.target_inside_pose
        required_services = (
            (self._set_model_state, "gazebo set_model_state"),
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
        request = SetModelState.Request()
        request.model_state.model_name = self._robot
        request.model_state.pose = target_pose
        request.model_state.pose.position.z = self._model_z
        request.model_state.reference_frame = "world"
        self._set_model_state.call_async(request).add_done_callback(
            lambda future: self._model_moved(future, generation)
        )

    def _model_moved(self, future, generation):
        if not self._active(generation):
            return
        try:
            response = future.result()
            if response is None or not response.success:
                detail = response.status_message if response is not None else "empty response"
                self._finish(
                    generation, ElevatorStatus.STATUS_FAILED,
                    f"set_model_state failed: {detail}",
                )
                return
        except Exception as exc:  # noqa: BLE001
            self._finish(
                generation, ElevatorStatus.STATUS_FAILED,
                f"set_model_state exception: {exc}",
            )
            return
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
        self._begin_relocalize(generation, 1)

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
