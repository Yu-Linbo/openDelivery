import copy
import threading

from action_msgs.msg import GoalStatus
from custom_msgs_srvs.msg import TaskCommand, TaskInfo, TaskStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


TERMINAL = {TaskStatus.STATUS_FINISHED, TaskStatus.STATUS_FAILED, TaskStatus.STATUS_TERMINATED}
SUPPORTED_TYPES = {
    TaskInfo.TASK_TYPE_NAVIGATION,
    TaskInfo.TASK_TYPE_PATROL,
    TaskInfo.TASK_TYPE_FOLLOWING,
}


class NavigationTaskNode(Node):
    """Executes one task at a time through Nav2 and exposes a message-only control API."""

    def __init__(self):
        super().__init__("navigation_task")
        self.declare_parameter("robot_name", "robot2")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("action_server_wait_sec", 15.0)
        robot = str(self.get_parameter("robot_name").value).strip().strip("/") or "robot2"
        self._map_frame = str(self.get_parameter("map_frame").value).strip() or "map"
        self._action_server_wait_sec = max(
            0.1, float(self.get_parameter("action_server_wait_sec").value)
        )
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(TaskStatus, "task_status", qos)
        self.create_subscription(TaskInfo, "task_info", self._on_task, 10)
        self.create_subscription(TaskCommand, "task_command", self._on_command, 10)
        # Normal task traffic stays inside /<robot>/navigation.  The public
        # /<robot>/navigate_to_pose action is retained only as a compatibility API.
        self._navigate = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._follow = ActionClient(self, FollowPath, "follow_path")
        self._navigate_action_name = f"/{robot}/navigation/navigate_to_pose"
        self._follow_action_name = f"/{robot}/navigation/follow_path"
        self._lock = threading.RLock()
        self._task = None
        self._status = TaskStatus.STATUS_WAITING
        self._message = "waiting for task"
        self._index = 0
        self._goal_handle = None
        self._generation = 0
        self._publish_status()

    @staticmethod
    def _validate(msg):
        task_id = str(msg.task_id).strip()
        task_type = str(msg.task_type).strip().lower()
        end_action = str(msg.end_action).strip().lower() or TaskInfo.END_ACTION_WAITING
        if not task_id:
            return "task_id is required"
        if task_type not in SUPPORTED_TYPES:
            return "task_type must be navigation, patrol, or following"
        if end_action not in (TaskInfo.END_ACTION_WAITING, TaskInfo.END_ACTION_BACK):
            return "end_action must be waiting or back"
        if not msg.poses:
            return "poses must not be empty"
        if msg.floor_ids and len(msg.floor_ids) != len(msg.poses):
            return "floor_ids must be empty or match poses length"
        if end_action == TaskInfo.END_ACTION_BACK and len(msg.poses) < 2:
            return "end_action=back requires the final pose to be the return pose"
        if task_type == TaskInfo.TASK_TYPE_NAVIGATION and len(msg.poses) > 2:
            return "navigation task accepts one goal and one optional return pose"
        return ""

    def _on_task(self, msg):
        error = self._validate(msg)
        with self._lock:
            if error:
                self._task = copy.deepcopy(msg)
                self._index = 0
                self._status = TaskStatus.STATUS_FAILED
                self._message = error
                self._generation += 1
                self._cancel_goal()
                self._publish_status()
                return
            if self._task and self._status not in TERMINAL and msg.task_id == self._task.task_id:
                self.get_logger().warning("duplicate active task ignored: %s", msg.task_id)
                return
            self._generation += 1
            self._cancel_goal()
            self._task = copy.deepcopy(msg)
            self._task.task_type = str(msg.task_type).strip().lower()
            self._task.end_action = str(msg.end_action).strip().lower() or TaskInfo.END_ACTION_WAITING
            self._index = 0
            self._status = TaskStatus.STATUS_WAITING
            self._message = "task accepted"
            generation = self._generation
            self._publish_status()
        self._dispatch(generation)

    def _on_command(self, msg):
        command = str(msg.command).strip().lower()
        with self._lock:
            if not self._task or str(msg.task_id).strip() != self._task.task_id:
                self.get_logger().warning("command ignored for non-current task: %s", msg.task_id)
                return
            if command == TaskCommand.COMMAND_PAUSE.lower():
                if self._status in TERMINAL or self._status == TaskStatus.STATUS_PAUSED:
                    return
                self._generation += 1
                self._status = TaskStatus.STATUS_PAUSED
                self._message = "task paused"
                self._cancel_goal()
                self._publish_status()
                return
            if command == TaskCommand.COMMAND_TERMINATE.lower():
                if self._status in TERMINAL:
                    return
                self._generation += 1
                self._status = TaskStatus.STATUS_TERMINATED
                self._message = "task terminated"
                self._cancel_goal()
                self._publish_status()
                return
            if command == TaskCommand.COMMAND_RESUME.lower():
                if self._status != TaskStatus.STATUS_PAUSED:
                    return
                self._generation += 1
                generation = self._generation
                self._status = TaskStatus.STATUS_WAITING
                self._message = "task resumed"
                self._publish_status()
            else:
                self.get_logger().warning("unknown task command: %s", msg.command)
                return
        self._dispatch(generation)

    def _cancel_goal(self):
        handle = self._goal_handle
        self._goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()

    def _pose_stamped(self, pose):
        stamped = PoseStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._map_frame
        stamped.pose = pose
        return stamped

    def _dispatch(self, generation):
        with self._lock:
            if generation != self._generation or not self._task:
                return
            following = self._task.task_type == TaskInfo.TASK_TYPE_FOLLOWING
            client = self._follow if following else self._navigate
            action_name = self._follow_action_name if following else self._navigate_action_name
        if not client.wait_for_server(timeout_sec=self._action_server_wait_sec):
            self._finish(
                generation,
                TaskStatus.STATUS_FAILED,
                f"Nav2 action server unavailable: {action_name}",
            )
            return
        with self._lock:
            if generation != self._generation:
                return
            if following:
                path = Path()
                path.header.stamp = self.get_clock().now().to_msg()
                path.header.frame_id = self._map_frame
                path.poses = [self._pose_stamped(pose) for pose in self._task.poses]
                goal = FollowPath.Goal()
                goal.path = path
                self._status = TaskStatus.STATUS_FOLLOWING
            else:
                goal = NavigateToPose.Goal()
                goal.pose = self._pose_stamped(self._task.poses[self._index])
                self._status = TaskStatus.STATUS_NAVIGATING
            self._message = "goal dispatched"
            self._publish_status()
            future = client.send_goal_async(goal)
            future.add_done_callback(lambda done: self._goal_response(done, generation))

    def _goal_response(self, future, generation):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(generation, TaskStatus.STATUS_FAILED, f"goal request failed: {exc}")
            return
        with self._lock:
            if generation != self._generation:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._finish(generation, TaskStatus.STATUS_FAILED, "Nav2 rejected goal")
                return
            self._goal_handle = handle
            handle.get_result_async().add_done_callback(
                lambda done: self._goal_result(done, generation)
            )

    def _goal_result(self, future, generation):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # noqa: BLE001
            self._finish(generation, TaskStatus.STATUS_FAILED, f"goal result failed: {exc}")
            return
        with self._lock:
            if generation != self._generation:
                return
            self._goal_handle = None
            if status != GoalStatus.STATUS_SUCCEEDED:
                self._finish(generation, TaskStatus.STATUS_FAILED, f"Nav2 goal status={status}")
                return
            if self._task.task_type != TaskInfo.TASK_TYPE_FOLLOWING and self._index + 1 < len(self._task.poses):
                self._index += 1
                self._message = "dispatching next pose"
                self._publish_status()
            else:
                self._finish(generation, TaskStatus.STATUS_FINISHED, "task finished")
                return
        self._dispatch(generation)

    def _finish(self, generation, status, message):
        with self._lock:
            if generation != self._generation:
                return
            self._goal_handle = None
            self._status = status
            self._message = message
            self._publish_status()

    def _publish_status(self):
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.task_id = self._task.task_id if self._task else ""
        msg.task_status = self._status
        msg.message = self._message
        msg.current_index = self._index
        msg.total_count = len(self._task.poses) if self._task else 0
        if self._task:
            label = self._task.task_type
            msg.work_queue = [label]
            msg.model_status = [self._status]
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationTaskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
