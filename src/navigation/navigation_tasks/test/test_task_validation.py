import threading

from action_msgs.msg import GoalStatus
from custom_msgs_srvs.msg import TaskInfo, TaskStatus
from geometry_msgs.msg import Pose

from navigation_tasks.node import NavigationTaskNode


def task(task_type="navigation", count=1, end_action="waiting"):
    msg = TaskInfo()
    msg.task_id = "task-1"
    msg.task_type = task_type
    msg.end_action = end_action
    msg.poses = [Pose() for _ in range(count)]
    return msg


def test_supported_task_shapes():
    assert NavigationTaskNode._validate(task()) == ""
    assert NavigationTaskNode._validate(task("navigation", 2, "back")) == ""
    assert NavigationTaskNode._validate(task("patrol", 3)) == ""
    assert NavigationTaskNode._validate(task("following", 20)) == ""


def test_rejects_ambiguous_or_invalid_tasks():
    assert "return pose" in NavigationTaskNode._validate(task("patrol", 1, "back"))
    assert "optional return" in NavigationTaskNode._validate(task("navigation", 3))
    msg = task()
    msg.floor_ids = ["floor1", "floor1"]
    assert "match poses" in NavigationTaskNode._validate(msg)


def test_only_aborted_nav2_goals_use_bounded_retry_policy():
    assert NavigationTaskNode._retryable_goal_status(GoalStatus.STATUS_ABORTED)
    assert not NavigationTaskNode._retryable_goal_status(GoalStatus.STATUS_SUCCEEDED)
    assert not NavigationTaskNode._retryable_goal_status(GoalStatus.STATUS_CANCELED)
    assert not NavigationTaskNode._retryable_goal_status(GoalStatus.STATUS_UNKNOWN)


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class RetryHarness:
    def __init__(self, attempts=0):
        self._lock = threading.RLock()
        self._generation = 4
        self._task = object()
        self._retry_attempt = attempts
        self._nav2_goal_retry_count = 2
        self._nav2_goal_retry_delay_sec = 1.0
        self._retry_timer = None
        self._goal_handle = object()
        self.published = []
        self.dispatched = []
        self.finished = []
        self.destroyed = []

    def _publish_status(self):
        self.published.append((self._status, self._message))

    def create_timer(self, delay, callback):
        assert delay == 1.0
        return FakeTimer(callback)

    def destroy_timer(self, timer):
        self.destroyed.append(timer)

    def _dispatch(self, generation):
        self.dispatched.append(generation)

    def _finish(self, generation, status, message):
        self.finished.append((generation, status, message))


def test_aborted_goal_retry_is_delayed_once_and_then_dispatched():
    node = RetryHarness()
    NavigationTaskNode._retry_goal(node, 4, "Nav2 goal status=6")

    assert node._retry_attempt == 1
    assert node._goal_handle is None
    assert node.published == [
        (TaskStatus.STATUS_WAITING, "Nav2 goal status=6; retrying Nav2 goal 1/2")
    ]
    timer = node._retry_timer
    timer.callback()
    assert timer.cancelled
    assert node.destroyed == [timer]
    assert node.dispatched == [4]


def test_aborted_goal_retry_stops_at_configured_limit():
    node = RetryHarness(attempts=2)
    NavigationTaskNode._retry_goal(node, 4, "Nav2 goal status=6")

    assert node.finished == [
        (4, TaskStatus.STATUS_FAILED, "Nav2 goal status=6")
    ]
    assert node._retry_timer is None
