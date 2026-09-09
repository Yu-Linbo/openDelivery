import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ground_truth_math import (
    apply_pose_error,
    bounded_time_step,
    compose,
    decay_pose_error,
    inverse,
    map_to_world_from_files,
    map_yaml_for_name,
    normalize_angle,
    pose_error,
    world_velocity_to_body,
    yaw_from_quaternion,
    yaw_quaternion,
)


def test_inverse_cancels_pose():
    pose = (2.5, -1.2, 0.7)
    identity = compose(pose, inverse(pose))
    assert all(abs(value) < 1.0e-9 for value in identity)


def test_map_to_odom_recovers_ground_truth_base_pose():
    map_to_base = (-13.703, 12.825, 0.4)
    odom_to_base = (1.2, -0.8, -0.3)
    recovered = compose(compose(map_to_base, inverse(odom_to_base)), odom_to_base)
    for expected, actual in zip(map_to_base, recovered):
        assert abs(expected - actual) < 1.0e-9


def test_initial_pose_error_reports_requested_pose_exactly():
    true_map_to_base = (-7.3, 12.8, -0.4)
    requested = (2.363, 7.015, 0.2)
    error = pose_error(true_map_to_base, requested)
    actual = apply_pose_error(true_map_to_base, error)
    for expected, value in zip(requested, actual):
        assert math.isclose(expected, value, abs_tol=1.0e-9)


def test_pose_error_tracks_robot_motion_without_changing_bias():
    error = pose_error((1.0, 2.0, 0.3), (1.5, 1.7, 0.5))
    before = apply_pose_error((1.0, 2.0, 0.3), error)
    after = apply_pose_error((1.4, 2.2, 0.4), error)
    assert math.isclose(after[0] - before[0], 0.4, abs_tol=1.0e-12)
    assert math.isclose(after[1] - before[1], 0.2, abs_tol=1.0e-12)
    assert math.isclose(
        normalize_angle(after[2] - before[2]), 0.1, abs_tol=1.0e-12
    )


def test_decay_pose_error_limits_translation_and_rotation_together():
    actual = decay_pose_error((3.0, 4.0, 0.5), 2.0, 0.2, 1.0)
    assert math.isclose(actual[0], 1.8, abs_tol=1.0e-12)
    assert math.isclose(actual[1], 2.4, abs_tol=1.0e-12)
    assert math.isclose(actual[2], 0.3, abs_tol=1.0e-12)


def test_decay_pose_error_snaps_exactly_to_truth():
    assert decay_pose_error((0.03, -0.04, -0.02), 0.1, 0.1, 1.0) == (
        0.0,
        0.0,
        0.0,
    )


def test_decay_pose_error_does_not_advance_for_stopped_time():
    error = (0.3, -0.4, 0.2)
    assert decay_pose_error(error, 0.1, 0.1, 0.0) == error
    assert decay_pose_error(error, 0.1, 0.1, -1.0) == error


def test_initial_yaw_error_uses_shortest_path_across_pi():
    true_pose = (0.0, 0.0, math.radians(179.0))
    requested = (0.0, 0.0, math.radians(-179.0))
    error = pose_error(true_pose, requested)
    assert math.isclose(error[2], math.radians(2.0), abs_tol=1.0e-12)
    corrected = decay_pose_error(error, 0.1, math.radians(1.0), 1.0)
    assert math.isclose(corrected[2], math.radians(1.0), abs_tol=1.0e-12)


def test_new_initial_pose_replaces_existing_correction():
    true_pose = (2.0, 3.0, 0.4)
    old_error = pose_error(true_pose, (5.0, 3.0, 0.4))
    partly_corrected = decay_pose_error(old_error, 1.0, 1.0, 1.0)
    assert partly_corrected != (0.0, 0.0, 0.0)
    replacement = pose_error(true_pose, (1.5, 3.25, 0.2))
    assert apply_pose_error(true_pose, replacement) == (1.5, 3.25, 0.2)


def test_bounded_time_step_handles_pause_reset_and_large_gap():
    assert bounded_time_step(None, 10_000_000_000, 0.25) == 0.0
    assert bounded_time_step(10_000_000_000, 10_000_000_000, 0.25) == 0.0
    assert bounded_time_step(10_000_000_000, 9_000_000_000, 0.25) == 0.0
    assert math.isclose(
        bounded_time_step(10_000_000_000, 10_100_000_000, 0.25),
        0.1,
        abs_tol=1.0e-12,
    )
    assert bounded_time_step(10_000_000_000, 12_000_000_000, 0.25) == 0.25


def test_project_floor_map_world_conversion(tmp_path):
    pgm = tmp_path / "test_101.pgm"
    pgm.write_bytes(b"P5\n240 200\n255\n")
    yaml = tmp_path / "test_101.yaml"
    yaml.write_text("image: test_101.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
    world = tmp_path / "drawn_model.world"
    world.write_text(
        "<sdf><world><include><name>test_101</name><uri>model://test_101</uri>"
        "<pose>-10 10 0 0 0 0</pose></include></world></sdf>"
    )
    world_to_map = map_to_world_from_files(yaml, world)
    assert world_to_map == (-16.0, 5.0, 0.0)
    assert compose(world_to_map, (6.0, 5.0, 0.0)) == (-10.0, 10.0, 0.0)


def test_map_yaml_for_name_resolves_sibling_map(tmp_path):
    map_root = tmp_path / "map"
    initial = map_root / "test_101" / "test_101.yaml"
    target = map_root / "test_102" / "test_102.yaml"
    initial.parent.mkdir(parents=True)
    target.parent.mkdir(parents=True)
    initial.write_text("initial")
    target.write_text("target")
    assert map_yaml_for_name(initial, "test_102") == target


def test_map_yaml_for_name_rejects_invalid_or_missing_map(tmp_path):
    initial = tmp_path / "map" / "test_101" / "test_101.yaml"
    initial.parent.mkdir(parents=True)
    initial.write_text("initial")
    for map_name in ("", "../test_102", "missing"):
        try:
            map_yaml_for_name(initial, map_name)
        except RuntimeError:
            pass
        else:
            raise AssertionError("expected invalid map name to fail: %r" % map_name)


def test_switching_truth_map_removes_floor_sized_offset(tmp_path):
    pgm_101 = tmp_path / "test_101.pgm"
    pgm_102 = tmp_path / "test_102.pgm"
    pgm_101.write_bytes(b"P5\n240 200\n255\n")
    pgm_102.write_bytes(b"P5\n240 200\n255\n")
    yaml_101 = tmp_path / "test_101.yaml"
    yaml_102 = tmp_path / "test_102.yaml"
    yaml_101.write_text(
        "image: test_101.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"
    )
    yaml_102.write_text(
        "image: test_102.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"
    )
    world = tmp_path / "drawn_model.world"
    world.write_text(
        "<sdf><world>"
        "<include><name>test_101</name><uri>model://test_101</uri>"
        "<pose>-10 10 0 0 0 0</pose></include>"
        "<include><name>test_102</name><uri>model://test_102</uri>"
        "<pose>10 10 0 0 0 0</pose></include>"
        "</world></sdf>"
    )
    world_to_base = (7.667, 7.025, 0.0)
    pose_101 = compose(
        inverse(map_to_world_from_files(yaml_101, world)), world_to_base
    )
    pose_102 = compose(
        inverse(map_to_world_from_files(yaml_102, world)), world_to_base
    )
    for actual, expected in zip(pose_101, (23.667, 2.025, 0.0)):
        assert math.isclose(actual, expected, abs_tol=1.0e-12)
    for actual, expected in zip(pose_102, (3.667, 2.025, 0.0)):
        assert math.isclose(actual, expected, abs_tol=1.0e-12)


def test_yaw_quaternion_round_trip():
    yaw = -2.4
    assert math.isclose(yaw_from_quaternion(*yaw_quaternion(yaw)), yaw, abs_tol=1.0e-12)


def test_world_velocity_is_rotated_into_robot_axes():
    vx, vy = world_velocity_to_body(0.0, 2.0, math.pi / 2.0)
    assert math.isclose(vx, 2.0, abs_tol=1.0e-12)
    assert math.isclose(vy, 0.0, abs_tol=1.0e-12)
