"""Regression cases for endless feedback and success at the wrong pose."""
import math
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf2_ros import LookupException

from navigation_tasks.node import NavigationTaskNode
from test_task_validation import WatchdogHarness, task


def pose(x=0.0, y=0.0, yaw=0.0):
    p = Pose()
    p.position.x, p.position.y = x, y
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class CompletionHarness(WatchdogHarness):
    _pose_error = staticmethod(NavigationTaskNode._pose_error)
    _validate = staticmethod(NavigationTaskNode._validate)
    _arrival_error = NavigationTaskNode._arrival_error
    _complete_goal = NavigationTaskNode._complete_goal

    def __init__(self, current=None, goal=None, age=0):
        super().__init__(elapsed=0)
        self._task = task()
        self._index = 0
        self._status = 'Navigating'
        self._map_frame = 'map'
        self._base_frame = 'robot1/base_footprint'
        self._arrival_xy_tolerance = 0.10
        self._arrival_yaw_tolerance = 0.15
        self._arrival_tf_max_age_sec = 1.0
        self._arrival_verification_timeout_sec = 2.0
        self._active_goal_pose = goal or pose(1.0)
        self._best_remaining = math.inf
        self._best_yaw_error = math.inf
        self.transform = TransformStamped()
        self.transform.header.stamp.sec = 100
        p = current or pose(1.0)
        self.transform.transform.translation.x = p.position.x
        self.transform.transform.translation.y = p.position.y
        self.transform.transform.rotation = p.orientation
        self._tf_buffer = SimpleNamespace(lookup_transform=lambda *args: self.transform)
        self.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int((100 + age) * 1e9)))

    def feedback(self, distance, current=None, generation=4, dispatch_id=9):
        stamped = PoseStamped()
        stamped.header.frame_id = 'map'
        stamped.pose = current or pose()
        NavigationTaskNode._goal_feedback(self, SimpleNamespace(feedback=SimpleNamespace(
            distance_remaining=distance, current_pose=stamped)), generation, dispatch_id)

    def success(self):
        future = SimpleNamespace(result=lambda: SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
        NavigationTaskNode._goal_result(self, future, 4, 9)


def test_live_feedback_without_progress_is_bounded():
    node = CompletionHarness()
    node.feedback(5.0)
    node._goal_progress_at -= 61.0
    node.feedback(5.0)  # Messages alone must not reset the progress deadline.
    NavigationTaskNode._check_goal_watchdog(node, 4, 9)
    assert node.retries == [(4, 'Nav2 goal made no progress for 60.0s')]


def test_real_distance_progress_and_final_rotation_reset_deadline():
    node = CompletionHarness(goal=pose(1.0, yaw=1.0))
    node.feedback(1.0)
    node._goal_progress_at -= 61.0
    node.feedback(0.8)
    NavigationTaskNode._check_goal_watchdog(node, 4, 9)
    assert node.retries == []
    node.feedback(0.0, pose(1.0, yaw=0.0))
    node._goal_progress_at -= 61.0
    node.feedback(0.0, pose(1.0, yaw=0.3))
    NavigationTaskNode._check_goal_watchdog(node, 4, 9)
    assert node.retries == []


def test_continuous_movement_cannot_exceed_goal_deadline():
    node = CompletionHarness()
    node.feedback(1.0)
    node._goal_started_at -= 601
    NavigationTaskNode._check_goal_watchdog(node, 4, 9)
    assert node.retries == [(4, 'Nav2 goal exceeded 600.0s')]


@pytest.mark.parametrize('current', [pose(0.7), pose(1.0, yaw=0.3), pose(0.5, yaw=0)])
def test_success_outside_original_goal_does_not_finish(current):
    node = CompletionHarness(current=current)
    node.success()
    assert node.finished == []
    assert node._goal_watchdog_phase == 'verify'
    node._goal_activity_at -= 3
    NavigationTaskNode._check_goal_watchdog(node, 4, 9)
    assert len(node.retries) == 1
    assert 'arrival was not verified' in node.retries[0][1]


def test_success_at_goal_finishes():
    node = CompletionHarness(current=pose(0.95, yaw=0.1))
    node.success()
    assert node.finished[0][1] == 'Finished'


@pytest.mark.parametrize('age', [2.0, -2.0])
def test_stale_or_future_pose_cannot_finish(age):
    node = CompletionHarness(age=age)
    node.success()
    assert node.finished == []
    assert 'stale' in node._message


def test_missing_tf_can_catch_up_during_verification():
    node = CompletionHarness()
    def unavailable(*args):
        raise LookupException('not ready')
    node._tf_buffer.lookup_transform = unavailable
    node.success()
    assert node.finished == []
    node._tf_buffer.lookup_transform = lambda *args: node.transform
    NavigationTaskNode._check_goal_watchdog(node, 4, 9)
    assert node.finished[0][1] == 'Finished'


def test_next_patrol_pose_only_dispatches_after_verified_arrival():
    node = CompletionHarness(current=pose(0.5))
    node._task = task('patrol', 2)
    node.success()
    assert node._index == 0
    assert node.dispatched == []
    node.transform.transform.translation.x = 1.0
    NavigationTaskNode._check_goal_watchdog(node, 4, 9)
    assert node._index == 1
    assert node.dispatched == [4]


def test_stale_feedback_and_result_cannot_change_replaced_task():
    node = CompletionHarness()
    activity = node._goal_activity_at
    node.feedback(0.0, generation=3)
    assert node._goal_activity_at == activity
    node._generation = 5
    node.success()
    assert node.finished == []
    assert node.published == []


def test_yaw_wrap_and_invalid_pose_are_handled():
    xy, yaw = NavigationTaskNode._pose_error(pose(yaw=-math.pi + 0.02), pose(yaw=math.pi - 0.02))
    assert xy == 0
    assert yaw == pytest.approx(0.04)
    invalid = Pose()
    invalid.orientation.w = 0.0
    assert NavigationTaskNode._pose_error(invalid, pose())[1] == math.inf
    assert NavigationTaskNode._pose_error(pose(math.nan), pose())[0] == math.inf


def test_initial_zero_path_distance_does_not_poison_progress_tracking():
    node = CompletionHarness()
    node.feedback(0.0, pose(0.0))
    assert node._best_remaining == math.inf
    node.feedback(1.0)
    node._goal_progress_at -= 61
    node.feedback(0.8)
    NavigationTaskNode._check_goal_watchdog(node, 4, 9)
    assert node.retries == []


def test_duplicate_active_task_does_not_crash_foxy_logger():
    node = CompletionHarness()
    messages = []
    # Foxy logger.warning accepts a single message, not Python logging args.
    node.get_logger = lambda: SimpleNamespace(warning=lambda message: messages.append(message))
    NavigationTaskNode._on_task(node, task())
    assert messages == ['duplicate active task ignored: task-1']
    assert node.finished == []
    assert node.dispatched == []
