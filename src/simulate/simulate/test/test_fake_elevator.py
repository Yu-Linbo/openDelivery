"""Focused unit tests for fake elevator retry and stale-callback behavior."""

import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "fake_elevator.py"
SPEC = importlib.util.spec_from_file_location("fake_elevator", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
FakeElevator = MODULE.FakeElevator


class RecordingClient:
    def __init__(self):
        self.request = None
        self.wait_timeout = None

    def wait_for_service(self, timeout_sec):
        self.wait_timeout = timeout_sec
        return True

    def call_async(self, request):
        self.request = request
        return SimpleNamespace(add_done_callback=lambda _callback: None)


class UnavailableClient(RecordingClient):
    def wait_for_service(self, timeout_sec):
        self.wait_timeout = timeout_sec
        return False


def bare_elevator():
    node = object.__new__(FakeElevator)
    node._lock = threading.RLock()
    node._generation = 7
    node._info = SimpleNamespace(from_floor="floor1", operation="call")
    node._timer = None
    node._service_wait = 10.0
    node._map_pose = None
    node._world_pose = None
    return node


def test_call_operation_finishes_without_switching_map():
    node = bare_elevator()
    calls = []
    node._cancel_timer = lambda: None
    node._finish = lambda generation, status, message: calls.append((generation, status, message))

    node._after_delay(generation=7)

    assert calls == [(7, MODULE.ElevatorStatus.STATUS_FINISHED,
                      "fake elevator arrived at floor1")]


def test_call_operation_uses_three_second_default_delay():
    node = bare_elevator()
    periods = []
    node._call_delay = 3.0
    node._publish = lambda: None
    node.create_timer = lambda period, _callback: periods.append(period) or object()

    node._schedule_operation(generation=7)

    assert node._status == MODULE.ElevatorStatus.STATUS_CALLING
    assert periods == [3.0]


def test_publish_supports_older_elevator_status_schema(monkeypatch):
    class LegacyElevatorStatus:
        STATUS_WAITING = "Waiting"

        def __init__(self):
            self.header = SimpleNamespace(
                stamp=None,
                frame_id="",
            )
            self.task_id = ""
            self.elevator_status = ""
            self.message = ""

    node = bare_elevator()
    node._map_frame = "map"
    node._status = "Calling"
    node._message = "calling fake elevator"
    node._info = SimpleNamespace(
        task_id="task-1", operation="call", from_floor="floor1", target_floor="floor1"
    )
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
    )
    published = []
    node._status_pub = SimpleNamespace(publish=published.append)
    monkeypatch.setattr(MODULE, "ElevatorStatus", LegacyElevatorStatus)

    node._publish()

    assert published[0].task_id == "task-1"
    assert published[0].elevator_status == "Calling"


def test_ride_moves_model_to_target_inside_pose_before_map_switch(monkeypatch):
    node = bare_elevator()
    info = MODULE.ElevatorInfo()
    info.operation = MODULE.ElevatorInfo.OPERATION_RIDE
    info.from_floor = "floor1"
    info.target_floor = "floor2"
    info.target_inside_pose.position.x = 4.2
    info.target_inside_pose.position.y = -1.3
    info.target_inside_pose.orientation.w = 1.0
    node._info = info
    node._robot = "robot2"
    node._map_root = Path("/tmp")
    node._model_z = 0.05
    node._set_model_state = RecordingClient()
    node._heartbeat = RecordingClient()
    node._load_map = RecordingClient()
    node._relocalize = RecordingClient()
    node._cancel_timer = lambda: None
    node._publish = lambda: None
    node._target_world_pose = lambda pose: pose

    monkeypatch.setattr(Path, "is_file", lambda _path: True)
    node._after_delay(generation=7)

    request = node._set_model_state.request
    assert node._set_model_state.wait_timeout == 10.0
    assert request.model_state.model_name == "robot2"
    assert request.model_state.reference_frame == "world"
    assert request.model_state.pose.position.x == 4.2
    assert request.model_state.pose.position.y == -1.3
    assert request.model_state.pose.position.z == 0.05


def test_map_target_is_transformed_into_gazebo_world_coordinates():
    current_map = MODULE.Pose()
    current_map.position.x = 2.0
    current_map.position.y = 3.0
    current_map.orientation.w = 1.0
    current_world = MODULE.Pose()
    current_world.position.x = -10.0
    current_world.position.y = 8.0
    current_world.orientation.z = 2 ** -0.5
    current_world.orientation.w = 2 ** -0.5
    target = MODULE.Pose()
    target.position.x = 3.0
    target.position.y = 5.0
    target.orientation.w = 1.0

    result = FakeElevator._map_pose_to_world(target, current_map, current_world)

    assert result.position.x == pytest.approx(-12.0)
    assert result.position.y == pytest.approx(9.0)
    assert FakeElevator._yaw(result) == pytest.approx(MODULE.math.pi / 2.0)


def test_native_gazebo_pose_is_used_when_model_states_is_undiscovered(monkeypatch):
    node = bare_elevator()
    node._robot = "robot4"
    node._map_pose = MODULE.Pose()
    node._map_pose.position.x = 2.0
    node._map_pose.position.y = 3.0
    node._map_pose.orientation.w = 1.0
    node._world_pose = None
    target = MODULE.Pose()
    target.position.x = 3.0
    target.position.y = 5.0
    target.orientation.w = 1.0
    monkeypatch.setattr(
        MODULE.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            returncode=0, stdout="-10 8 0.05 0 0 1.5707963267948966", stderr=""
        ),
    )

    result = node._target_world_pose(target)

    assert result.position.x == pytest.approx(-12.0)
    assert result.position.y == pytest.approx(9.0)


def test_successful_model_move_continues_with_map_switch():
    node = bare_elevator()
    calls = []
    node._switch_map = lambda generation: calls.append(generation)
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(success=True, status_message="done")
    )

    node._model_moved(future, generation=7)

    assert calls == [7]


def test_service_discovery_failure_uses_gazebo_native_transport(monkeypatch):
    node = bare_elevator()
    info = MODULE.ElevatorInfo()
    info.operation = MODULE.ElevatorInfo.OPERATION_RIDE
    info.target_floor = "floor2"
    info.target_inside_pose.position.x = 7.8
    info.target_inside_pose.position.y = 7.5
    info.target_inside_pose.orientation.w = 1.0
    node._info = info
    node._robot = "robot4"
    node._map_root = Path("/tmp")
    node._model_z = 0.05
    node._set_model_state = UnavailableClient()
    node._heartbeat = RecordingClient()
    node._load_map = RecordingClient()
    node._relocalize = RecordingClient()
    node._cancel_timer = lambda: None
    node._publish = lambda: None
    node._target_world_pose = lambda pose: pose
    node.get_logger = lambda: SimpleNamespace(warning=lambda *_args: None)
    switched = []
    node._switch_map = switched.append
    commands = []
    monkeypatch.setattr(Path, "is_file", lambda _path: True)
    monkeypatch.setattr(
        MODULE.subprocess,
        "run",
        lambda command, **_kwargs: commands.append(command) or SimpleNamespace(
            returncode=0, stdout="", stderr=""
        ),
    )

    node._after_delay(generation=7)

    assert commands[0][:4] == ["gz", "model", "-m", "robot4"]
    assert switched == [7]


def test_ride_relocalization_uses_mode_one_and_previous_floor_pose():
    node = bare_elevator()
    info = MODULE.ElevatorInfo()
    info.operation = MODULE.ElevatorInfo.OPERATION_RIDE
    info.use_pose_first = True
    info.relocalization_pose.pose.pose.position.x = 1.25
    node._info = info
    node._map_frame = "map"
    node._relocalize = RecordingClient()
    node._publish = lambda: None

    node._begin_relocalize(generation=7, attempt=1)

    request = node._relocalize.request
    assert request.mode == MODULE.Relocalize.Request.MODE_POSE_FIRST
    assert request.pose.pose.pose.position.x == 1.25


def test_only_transient_map_or_scan_readiness_errors_are_retried():
    retryable = FakeElevator._retryable_relocalize_error
    assert retryable("current map OccupancyGrid is not ready")
    assert retryable("no fresh scan_2d frame")
    assert not retryable("no saved scan records for current map")
    assert not retryable("scan matching produced no candidate")


def test_stale_map_callback_does_not_continue_switch():
    node = bare_elevator()
    calls = []
    node._rollback_floor = lambda *_args: calls.append("rollback")
    node._begin_relocalize = lambda *_args: calls.append("relocalize")

    node._map_loaded(SimpleNamespace(result=lambda: None), generation=6)

    assert calls == []


def test_load_map_failure_rolls_heartbeat_floor_back():
    node = bare_elevator()
    calls = []
    node._rollback_floor = lambda generation, reason: calls.append((generation, reason))
    response = SimpleNamespace(result=1)
    future = SimpleNamespace(result=lambda: response)

    node._map_loaded(future, generation=7)

    assert calls == [(7, "load_map failed: 1")]


def test_successful_load_map_defers_relocalization_for_map_propagation():
    node = bare_elevator()
    calls = []
    node._schedule_relocalize_retry = lambda generation, attempt, detail: calls.append(
        (generation, attempt, detail)
    )
    response = SimpleNamespace(result=MODULE.LoadMap.Response.RESULT_SUCCESS)

    node._map_loaded(SimpleNamespace(result=lambda: response), generation=7)

    assert calls == [(7, 1, "waiting for target map propagation")]
