"""Regression: a teleport must not turn an old cached pose into a floor-sized bias."""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from gazebo_ground_truth_localizer import GazeboGroundTruthLocalizer


def test_initial_pose_uses_next_model_sample_instead_of_pre_teleport_cache():
    node = object.__new__(GazeboGroundTruthLocalizer)
    node.map_frame = 'map'
    node.robot_model = 'robot1'
    node._last_world_to_base = (-12.325, 6.997, 0.0)
    node._true_map_to_world = (-4.0, -5.0, 0.0)
    node._pending_initial = None
    node._correction_error = (0.0, 0.0, 0.0)
    node.get_logger = lambda: SimpleNamespace(info=lambda *_: None)
    requested = PoseWithCovarianceStamped()
    requested.header.frame_id = 'map'
    requested.pose.pose.position.x = 3.675
    requested.pose.pose.position.y = 1.997
    requested.pose.pose.orientation.w = 1.0

    node._on_initial_pose(requested)
    assert node._correction_error == (0.0, 0.0, 0.0)
    assert node._pending_initial == pytest.approx((3.675, 1.997, 0.0))

    # Stop after correction calculation, before unrelated odometry publication.
    class SampleApplied(Exception):
        pass

    def apply(truth, pose):
        GazeboGroundTruthLocalizer._set_initial_error(node, truth, pose)
        raise SampleApplied()

    node._set_initial_error = apply
    sample = ModelStates()
    sample.name = ['robot1']
    moved = Pose()
    moved.position.x = 7.675
    moved.position.y = 6.997
    moved.orientation.w = 1.0
    sample.pose = [moved]
    with pytest.raises(SampleApplied):
        node._on_model_states(sample)
    assert node._correction_error == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
