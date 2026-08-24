import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from ground_truth_math import (
    alignment_for_initial,
    compose,
    inverse,
    map_to_world_from_files,
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


def test_initial_pose_realigns_current_world_pose_exactly():
    world_to_base = (-7.3, 12.8, -0.4)
    requested = (2.363, 7.015, 0.2)
    alignment = alignment_for_initial(world_to_base, requested)
    actual = compose(alignment, world_to_base)
    for expected, value in zip(requested, actual):
        assert math.isclose(expected, value, abs_tol=1.0e-9)


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


def test_yaw_quaternion_round_trip():
    yaw = -2.4
    assert math.isclose(yaw_from_quaternion(*yaw_quaternion(yaw)), yaw, abs_tol=1.0e-12)
