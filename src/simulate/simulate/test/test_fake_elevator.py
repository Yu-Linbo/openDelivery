"""Focused unit tests for fake elevator retry and stale-callback behavior."""

import importlib.util
import json
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


def bare_elevator():
    node = object.__new__(FakeElevator)
    node._lock = threading.RLock()
    node._generation = 7
    node._info = SimpleNamespace(
        from_floor="floor1", target_floor="floor2", operation="call"
    )
    node._timer = None
    node._service_wait = 10.0
    node._observed_floor = ""
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


def test_ride_loads_target_floor_inside_point_and_moves_through_web(tmp_path):
    node = bare_elevator()
    info = MODULE.ElevatorInfo()
    info.operation = MODULE.ElevatorInfo.OPERATION_RIDE
    info.from_floor = "floor1"
    info.target_floor = "floor2"
    node._info = info
    node._robot = "robot2"
    node._map_root = tmp_path
    node._model_z = 0.05
    node._heartbeat = RecordingClient()
    node._load_map = RecordingClient()
    node._relocalize = RecordingClient()
    node._cancel_timer = lambda: None
    node._publish = lambda: None
    node._target_world_pose = lambda floor, pose: pose
    info_logs = []
    node.get_logger = lambda: SimpleNamespace(info=info_logs.append)
    moved = []
    node._move_model_through_web = lambda pose: moved.append(pose) or {
        "ok": True, "x": pose.position.x, "y": pose.position.y
    }
    switched = []
    node._switch_map = switched.append
    floor = tmp_path / "floor2"
    floor.mkdir()
    (floor / "floor2.yaml").write_text("image: floor2.pgm\n", encoding="utf-8")
    (floor / "floor2_points.json").write_text(
        json.dumps({"points": [{
            "id": "inside", "type": "elevator_inside",
            "x": 4.2, "y": -1.3, "yaw": 0.4,
        }]}),
        encoding="utf-8",
    )
    node._after_delay(generation=7)

    assert moved[0].position.x == 4.2
    assert moved[0].position.y == -1.3
    assert FakeElevator._yaw(moved[0]) == pytest.approx(0.4)
    assert switched == [7]
    assert info_logs == ["Web Gazebo move succeeded: robot2 -> x=4.200 y=-1.300"]


def test_map_target_uses_centered_floor_generation_rule(tmp_path):
    floor = tmp_path / "test_102"
    floor.mkdir()
    (floor / "test_102.pgm").write_bytes(b"P5\n240 200\n255\n")
    (floor / "test_102.yaml").write_text(
        "image: test_102.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8",
    )
    world = tmp_path / "world.sdf"
    world.write_text(
        "<sdf><world><include><uri>model://test_102</uri>"
        "<name>test_102</name><pose>10 10 0 0 0 0</pose>"
        "</include></world></sdf>",
        encoding="utf-8",
    )
    node = bare_elevator()
    node._map_root = tmp_path
    node._world_path = world
    node._model_z = 0.05
    target = MODULE.Pose()
    target.position.x = 3.6751758218341175
    target.position.y = 1.9965636010119399
    target.orientation.z = MODULE.math.sin(MODULE.math.pi / 4)
    target.orientation.w = MODULE.math.cos(MODULE.math.pi / 4)

    result = node._target_world_pose("test_102", target)

    assert result.position.x == pytest.approx(7.6751758218341175)
    assert result.position.y == pytest.approx(6.99656360101194)
    assert FakeElevator._yaw(result) == pytest.approx(MODULE.math.pi / 2.0)


def test_floor_world_pose_requires_exactly_one_named_model(tmp_path):
    node = bare_elevator()
    node._world_path = tmp_path / "world.sdf"
    node._world_path.write_text("<sdf><world/></sdf>", encoding="utf-8")
    with pytest.raises(RuntimeError, match="exactly one"):
        node._floor_world_pose("test_102")


def test_model_move_posts_same_payload_as_web_manual_gazebo_action():
    node = bare_elevator()
    node._robot = "robot4"
    node._model_z = 0.05
    calls = []
    node._web_json = lambda path, payload: calls.append((path, payload)) or {
        "ok": True, "x": payload["x"], "y": payload["y"]
    }
    pose = MODULE.Pose()
    pose.position.x = 7.8
    pose.position.y = 7.5
    pose.orientation.z = MODULE.math.sin(0.6 / 2.0)
    pose.orientation.w = MODULE.math.cos(0.6 / 2.0)

    response = node._move_model_through_web(pose)

    path, payload = calls[0]
    assert path == "/api/gazebo/set_model_state"
    assert payload == {
        "model_name": "robot4",
        "x": 7.8,
        "y": 7.5,
        "z": 0.05,
        "yaw": pytest.approx(0.6),
        "reference_frame": "world",
    }
    assert response["ok"] is True


def test_target_inside_point_requires_exactly_one_match(tmp_path):
    node = bare_elevator()
    node._map_root = tmp_path
    floor = tmp_path / "floor2"
    floor.mkdir()
    points = floor / "floor2_points.json"
    points.write_text(json.dumps({"points": []}), encoding="utf-8")

    with pytest.raises(RuntimeError, match="exactly one"):
        node._target_inside_pose("floor2")


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


def test_floor_status_must_be_observed_before_load_map_request(monkeypatch):
    node = bare_elevator()
    node._map_status_delay = 0.5
    node._load_map = RecordingClient()
    node._publish = lambda: None
    node._cancel_timer = lambda: None
    scheduled = []
    now = [1.0]
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: now[0])
    node.create_timer = lambda delay, callback: scheduled.append((delay, callback)) or object()
    response = SimpleNamespace(success=True, message="updated")

    node._map_status_updated(
        SimpleNamespace(result=lambda: response), generation=7, yaml_path="/maps/floor2.yaml"
    )

    assert node._load_map.request is None
    assert scheduled[0][0] == pytest.approx(0.1)
    scheduled[0][1]()
    assert node._load_map.request is None
    node._observed_floor = "floor2"
    scheduled[-1][1]()
    assert scheduled[-1][0] == pytest.approx(0.5)
    scheduled[-1][1]()
    assert node._load_map.request.map_url == "/maps/floor2.yaml"


def test_floor_status_wait_times_out_without_loading_map(monkeypatch):
    node = bare_elevator()
    node._observed_floor = "floor1"
    node._load_map = RecordingClient()
    node._publish = lambda: None
    node._cancel_timer = lambda: None
    node._service_wait = 1.0
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: 2.0)
    failures = []
    node._finish = lambda *args: failures.append(args)

    node._wait_for_target_floor(7, "/maps/floor2.yaml", 0.0)

    assert node._load_map.request is None
    assert failures[0][2] == (
        "target floor status not observed: expected floor2, got floor1"
    )
