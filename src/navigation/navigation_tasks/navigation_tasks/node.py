import copy
import math
import threading
import time

from action_msgs.msg import GoalStatus
from custom_msgs_srvs.msg import TaskCommand, TaskInfo, TaskStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, NavigateToPose
from nav_msgs.msg import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


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
        self.declare_parameter("nav2_goal_retry_count", 2)
        self.declare_parameter("nav2_goal_retry_delay_sec", 1.0)
        self.declare_parameter("nav2_goal_response_timeout_sec", 10.0)
        self.declare_parameter("nav2_feedback_timeout_sec", 20.0)
        self.declare_parameter("nav2_progress_timeout_sec", 60.0)
        self.declare_parameter("nav2_goal_timeout_sec", 600.0)
        self.declare_parameter("arrival_xy_tolerance", 0.10)
        self.declare_parameter("arrival_yaw_tolerance", 0.15)
        self.declare_parameter("arrival_tf_max_age_sec", 1.0)
        self.declare_parameter("arrival_verification_timeout_sec", 2.0)
        robot = str(self.get_parameter("robot_name").value).strip().strip("/") or "robot2"
        self._map_frame = str(self.get_parameter("map_frame").value).strip() or "map"
        self._base_frame = f"{robot}/base_footprint"
        for name in ("nav2_progress_timeout_sec", "nav2_goal_timeout_sec",
                     "arrival_xy_tolerance", "arrival_yaw_tolerance",
                     "arrival_tf_max_age_sec", "arrival_verification_timeout_sec"):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
            setattr(self, "_" + name, value)
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_server_wait_sec = max(
            0.1, float(self.get_parameter("action_server_wait_sec").value)
        )
        self._nav2_goal_retry_count = max(
            0, int(self.get_parameter("nav2_goal_retry_count").value)
        )
        self._nav2_goal_retry_delay_sec = max(
            0.1, float(self.get_parameter("nav2_goal_retry_delay_sec").value)
        )
        self._nav2_goal_response_timeout_sec = max(
            0.5, float(self.get_parameter("nav2_goal_response_timeout_sec").value)
        )
        self._nav2_feedback_timeout_sec = max(
            1.0, float(self.get_parameter("nav2_feedback_timeout_sec").value)
        )
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(TaskStatus, "task_status", qos)
        self.create_subscription(TaskInfo, "task_info", self._on_task, 10)
        self.create_subscription(TaskCommand, "task_command", self._on_command, 10)
        # Task traffic and the Web goal sender use the same namespaced action.
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
        self._retry_attempt = 0
        self._retry_timer = None
        self._goal_watchdog_timer = None
        self._goal_watchdog_phase = ""
        self._goal_activity_at = 0.0
        self._goal_started_at = 0.0
        self._goal_progress_at = 0.0
        self._best_remaining = math.inf
        self._best_yaw_error = math.inf
        self._active_goal_pose = None
        self._dispatch_id = 0
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
                self.get_logger().warning(f"duplicate active task ignored: {msg.task_id}")
                return
            self._generation += 1
            self._cancel_goal()
            self._task = copy.deepcopy(msg)
            self._task.task_type = str(msg.task_type).strip().lower()
            self._task.end_action = str(msg.end_action).strip().lower() or TaskInfo.END_ACTION_WAITING
            self._index = 0
            self._retry_attempt = 0
            self._status = TaskStatus.STATUS_WAITING
            self._message = "task accepted"
            generation = self._generation
            self._publish_status()
        self._dispatch(generation)

    def _on_command(self, msg):
        command = str(msg.command).strip().lower()
        with self._lock:
            if not self._task or str(msg.task_id).strip() != self._task.task_id:
                self.get_logger().warning(f"command ignored for non-current task: {msg.task_id}")
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
                self.get_logger().warning(f"unknown task command: {msg.command}")
                return
        self._dispatch(generation)

    def _cancel_goal(self):
        self._cancel_retry()
        self._cancel_goal_watchdog()
        self._dispatch_id += 1
        handle = self._goal_handle
        self._goal_handle = None
        if handle is not None:
            handle.cancel_goal_async()

    def _cancel_retry(self):
        timer = self._retry_timer
        self._retry_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

    def _cancel_goal_watchdog(self):
        timer = self._goal_watchdog_timer
        self._goal_watchdog_timer = None
        self._goal_watchdog_phase = ""
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

    def _start_goal_watchdog(self, generation, dispatch_id):
        self._cancel_goal_watchdog()
        self._goal_watchdog_phase = "response"
        self._goal_activity_at = time.monotonic()
        self._goal_started_at = self._goal_activity_at
        self._goal_progress_at = self._goal_activity_at
        self._best_remaining = math.inf
        self._best_yaw_error = math.inf
        self._goal_watchdog_timer = self.create_timer(
            1.0,
            lambda: self._check_goal_watchdog(generation, dispatch_id),
            clock=self._watchdog_clock,
        )

    def _check_goal_watchdog(self, generation, dispatch_id):
        with self._lock:
            if generation != self._generation or dispatch_id != self._dispatch_id:
                return
            phase = self._goal_watchdog_phase
            now = time.monotonic()
            if phase == "verify":
                error = self._arrival_error()
                if not error:
                    self._complete_goal(generation, dispatch_id)
                    return
                if now - self._goal_activity_at < self._arrival_verification_timeout_sec:
                    return
                message = f"Nav2 reported success but arrival was not verified: {error}"
            elif phase == "feedback" and now - self._goal_started_at >= self._nav2_goal_timeout_sec:
                message = f"Nav2 goal exceeded {self._nav2_goal_timeout_sec:.1f}s"
            elif phase == "feedback" and now - self._goal_progress_at >= self._nav2_progress_timeout_sec:
                message = f"Nav2 goal made no progress for {self._nav2_progress_timeout_sec:.1f}s"
            else:
                timeout = (
                    self._nav2_goal_response_timeout_sec
                    if phase == "response"
                    else self._nav2_feedback_timeout_sec
                )
                if not phase or now - self._goal_activity_at < timeout:
                    return
                message = (
                    f"Nav2 goal response timed out after {timeout:.1f}s"
                    if phase == "response"
                    else f"Nav2 goal produced no feedback for {timeout:.1f}s"
                )
            self._cancel_goal_watchdog()
            self._dispatch_id += 1
            handle = self._goal_handle
            self._goal_handle = None
            if handle is not None:
                handle.cancel_goal_async()
        self._retry_goal(generation, message)

    def _goal_feedback(self, feedback, generation, dispatch_id):
        with self._lock:
            if generation != self._generation or dispatch_id != self._dispatch_id:
                return
            if self._goal_watchdog_phase == "verify":
                return
            self._goal_watchdog_phase = "feedback"
            self._goal_activity_at = time.monotonic()
            payload = feedback.feedback
            remaining = getattr(payload, "distance_remaining", getattr(payload, "distance_to_goal", math.nan))
            current = getattr(payload, "current_pose", None)
            xy, yaw = (self._pose_error(current.pose, self._active_goal_pose)
                       if current and current.header.frame_id == self._map_frame
                       else (math.inf, math.inf))
            # NavigateToPose can emit zero before its first path is available.
            # Do not let that default prevent later real progress from counting.
            usable = remaining > 0 or xy <= self._arrival_xy_tolerance
            if usable and math.isfinite(remaining) and remaining >= 0 and remaining < self._best_remaining - 0.05:
                self._best_remaining = remaining
                self._goal_progress_at = self._goal_activity_at
            # Final rotation is useful progress even after distance reaches zero.
            if xy <= self._arrival_xy_tolerance and yaw < self._best_yaw_error - 0.05:
                self._best_yaw_error = yaw
                self._goal_progress_at = self._goal_activity_at

    @staticmethod
    def _retryable_goal_status(status):
        # Nav2 Foxy may abort NavigateToPose with "send_goal failed" after the
        # controller has already accepted the path. A bounded retry preempts
        # that orphaned controller goal and restores a coherent action chain.
        return status == GoalStatus.STATUS_ABORTED

    def _retry_goal(self, generation, failure_message):
        with self._lock:
            if generation != self._generation or not self._task:
                return
            if self._retry_attempt >= self._nav2_goal_retry_count:
                self._finish(generation, TaskStatus.STATUS_FAILED, failure_message)
                return
            self._cancel_goal_watchdog()
            self._dispatch_id += 1
            handle = self._goal_handle
            self._goal_handle = None
            if handle is not None:
                handle.cancel_goal_async()
            self._retry_attempt += 1
            attempt = self._retry_attempt
            self._status = TaskStatus.STATUS_WAITING
            self._message = (
                f"{failure_message}; retrying Nav2 goal "
                f"{attempt}/{self._nav2_goal_retry_count}"
            )
            self._publish_status()

            def retry_once():
                with self._lock:
                    if self._retry_timer is not timer:
                        return
                    self._retry_timer = None
                    timer.cancel()
                    active = generation == self._generation and self._task is not None
                self.destroy_timer(timer)
                if active:
                    self._dispatch(generation)

            timer = self.create_timer(self._nav2_goal_retry_delay_sec, retry_once, clock=self._watchdog_clock)
            self._retry_timer = timer

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
                self._active_goal_pose = copy.deepcopy(self._task.poses[-1])
            else:
                goal = NavigateToPose.Goal()
                goal.pose = self._pose_stamped(self._task.poses[self._index])
                self._status = TaskStatus.STATUS_NAVIGATING
                self._active_goal_pose = copy.deepcopy(self._task.poses[self._index])
            self._message = "goal dispatched"
            self._publish_status()
            self._dispatch_id += 1
            dispatch_id = self._dispatch_id
            self._start_goal_watchdog(generation, dispatch_id)
            try:
                future = client.send_goal_async(
                    goal,
                    feedback_callback=lambda feedback: self._goal_feedback(
                        feedback, generation, dispatch_id
                    ),
                )
            except Exception as exc:  # noqa: BLE001
                self._retry_goal(generation, f"goal request failed: {exc}")
                return
            future.add_done_callback(
                lambda done: self._goal_response(done, generation, dispatch_id)
            )

    def _goal_response(self, future, generation, dispatch_id):
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                current = generation == self._generation and dispatch_id == self._dispatch_id
            if current:
                self._retry_goal(generation, f"goal request failed: {exc}")
            return
        with self._lock:
            if generation != self._generation or dispatch_id != self._dispatch_id:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._retry_goal(generation, "Nav2 rejected goal")
                return
            self._goal_handle = handle
            self._goal_watchdog_phase = "feedback"
            self._goal_activity_at = time.monotonic()
            handle.get_result_async().add_done_callback(
                lambda done: self._goal_result(done, generation, dispatch_id)
            )

    def _goal_result(self, future, generation, dispatch_id):
        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                current = generation == self._generation and dispatch_id == self._dispatch_id
            if current:
                self._retry_goal(generation, f"goal result failed: {exc}")
            return
        with self._lock:
            if generation != self._generation or dispatch_id != self._dispatch_id:
                return
            self._goal_handle = None
            if status != GoalStatus.STATUS_SUCCEEDED:
                message = f"Nav2 goal status={status}"
                if self._retryable_goal_status(status):
                    self._retry_goal(generation, message)
                else:
                    self._finish(generation, TaskStatus.STATUS_FAILED, message)
                return
            error = self._arrival_error()
            if error:
                self._goal_watchdog_phase = "verify"
                self._goal_activity_at = time.monotonic()
                self._message = f"verifying arrival: {error}"
                self._publish_status()
                return
        self._complete_goal(generation, dispatch_id)

    @staticmethod
    def _pose_error(current, goal):
        if current is None or goal is None:
            return math.inf, math.inf
        def yaw(q):
            norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
            if not math.isfinite(norm) or norm < 1e-12:
                return math.nan
            return math.atan2(2 * (q.w*q.z + q.x*q.y) / norm,
                              1 - 2 * (q.y*q.y + q.z*q.z) / norm)
        xy = math.hypot(current.position.x - goal.position.x, current.position.y - goal.position.y)
        delta = yaw(current.orientation) - yaw(goal.orientation)
        angle = abs(math.atan2(math.sin(delta), math.cos(delta))) if math.isfinite(delta) else math.inf
        return (xy if math.isfinite(xy) else math.inf), angle

    def _arrival_error(self):
        try:
            transform = self._tf_buffer.lookup_transform(self._map_frame, self._base_frame, Time())
        except TransformException as exc:
            return f"robot pose unavailable: {exc}"
        stamp = transform.header.stamp
        age = self.get_clock().now().nanoseconds / 1e9 - (stamp.sec + stamp.nanosec / 1e9)
        if age < -0.1 or age > self._arrival_tf_max_age_sec:
            return f"robot pose is stale (age={age:.2f}s)"
        pose = PoseStamped().pose
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.orientation = transform.transform.rotation
        xy, yaw = self._pose_error(pose, self._active_goal_pose)
        if xy > self._arrival_xy_tolerance or yaw > self._arrival_yaw_tolerance:
            return f"goal error {xy:.3f}m / {yaw:.3f}rad"
        return ""

    def _complete_goal(self, generation, dispatch_id):
        with self._lock:
            if generation != self._generation or dispatch_id != self._dispatch_id:
                return
            self._cancel_goal_watchdog()
            if self._task.task_type != TaskInfo.TASK_TYPE_FOLLOWING and self._index + 1 < len(self._task.poses):
                self._index += 1
                self._retry_attempt = 0
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
            self._cancel_retry()
            self._cancel_goal_watchdog()
            self._dispatch_id += 1
            handle = self._goal_handle
            self._goal_handle = None
            if handle is not None:
                handle.cancel_goal_async()
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
