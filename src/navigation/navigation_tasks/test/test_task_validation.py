from custom_msgs_srvs.msg import TaskInfo
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
