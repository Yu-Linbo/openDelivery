#!/usr/bin/env python3
"""Publish Gazebo truth localization with gradual initial-pose correction."""

from pathlib import Path
import time

import rclpy
from custom_msgs_srvs.msg import RobotStatus
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from ground_truth_math import (
    apply_pose_error,
    bounded_time_step,
    compose,
    decay_pose_error,
    inverse,
    map_to_world_from_files,
    map_yaml_for_name,
    pose_error,
    world_velocity_to_body,
    yaw_from_quaternion,
    yaw_quaternion,
)


class GazeboGroundTruthLocalizer(Node):
    _MAX_CORRECTION_DT = 0.25
    _MAP_SWITCH_INITIAL_WINDOW = 5.0

    def __init__(self):
        super().__init__("gazebo_ground_truth_localizer")
        self.robot_model = str(self.declare_parameter("robot_model", "").value).strip()
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self.base_frame = str(
            self.declare_parameter("base_frame", "base_footprint").value
        )
        self.pose_topic = str(self.declare_parameter("pose_topic", "amcl_pose").value)
        self.map_file = str(self.declare_parameter("map_file", "").value).strip()
        self.world_file = str(self.declare_parameter("world_file", "").value).strip()
        self.covariance = max(
            0.0, float(self.declare_parameter("covariance", 1.0e-9).value)
        )
        self.linear_correction_speed = max(
            0.0,
            float(self.declare_parameter("linear_correction_speed", 0.10).value),
        )
        self.angular_correction_speed = max(
            0.0,
            float(
                self.declare_parameter(
                    "angular_correction_speed", 0.0872665
                ).value
            ),
        )
        self.external_odom_timeout = max(
            0.1, float(self.declare_parameter("external_odom_timeout", 1.0).value)
        )
        if not self.robot_model:
            namespace = self.get_namespace().strip("/")
            self.robot_model = namespace.split("/")[0] if namespace else ""
        if not self.robot_model:
            raise RuntimeError("robot_model parameter is required")
        if not self.map_file or not self.world_file:
            raise RuntimeError("map_file and world_file parameters are required")
        configured_odom_topic = str(
            self.declare_parameter("odom_topic", "").value
        ).strip()
        self.odom_topic = configured_odom_topic or "/%s/odom" % self.robot_model

        self._initial_map_file = self.map_file
        self._active_map_name = Path(self.map_file).stem
        world_to_map = map_to_world_from_files(self.map_file, self.world_file)
        self._true_map_to_world = inverse(world_to_map)
        self._last_world_to_base = None
        self._pending_initial = None
        self._last_initial_request = None
        self._last_initial_wall_time = None
        self._correction_error = (0.0, 0.0, 0.0)
        self._last_correction_time_ns = None
        self._odom_missing_since = None
        self._owns_fallback_odom = False
        self._fallback_world_to_odom = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.pose_topic, 10
        )
        self.odom_publisher = self.create_publisher(Odometry, self.odom_topic, 10)
        self.create_subscription(ModelStates, "model_states", self._on_model_states, 1)
        self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self._on_initial_pose, 10
        )
        self.create_subscription(
            RobotStatus,
            "/%s/robot_status" % self.robot_model,
            self._on_robot_status,
            10,
        )
        self._missing_model_logged = False
        self.get_logger().info(
            "Gazebo truth localization ready: model=%s map=%s world=%s "
            "correction=%.3f m/s, %.3f rad/s"
            % (
                self.robot_model,
                self.map_file,
                self.world_file,
                self.linear_correction_speed,
                self.angular_correction_speed,
            )
        )

    @staticmethod
    def _stamp_nanoseconds(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _activate_fallback_odom(self, world_to_base, seed_odom_to_base=None):
        if seed_odom_to_base is None:
            seed_odom_to_base = (0.0, 0.0, 0.0)
        self._fallback_world_to_odom = compose(
            world_to_base, inverse(seed_odom_to_base)
        )
        self._owns_fallback_odom = True
        self.get_logger().warning(
            "external odom is missing or stale; publishing Gazebo truth odom on %s"
            % self.odom_topic
        )

    def _publish_fallback_odom(self, message, index, world_to_base, stamp):
        odom_to_base = compose(inverse(self._fallback_world_to_odom), world_to_base)
        qx, qy, qz, qw = yaw_quaternion(odom_to_base[2])

        tf_message = TransformStamped()
        tf_message.header.stamp = stamp
        tf_message.header.frame_id = self.odom_frame
        tf_message.child_frame_id = self.base_frame
        tf_message.transform.translation.x = odom_to_base[0]
        tf_message.transform.translation.y = odom_to_base[1]
        tf_message.transform.rotation.x = qx
        tf_message.transform.rotation.y = qy
        tf_message.transform.rotation.z = qz
        tf_message.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_message)

        odom_message = Odometry()
        odom_message.header.stamp = stamp
        odom_message.header.frame_id = self.odom_frame
        odom_message.child_frame_id = self.base_frame
        odom_message.pose.pose.position.x = odom_to_base[0]
        odom_message.pose.pose.position.y = odom_to_base[1]
        odom_message.pose.pose.orientation.x = qx
        odom_message.pose.pose.orientation.y = qy
        odom_message.pose.pose.orientation.z = qz
        odom_message.pose.pose.orientation.w = qw
        if index < len(message.twist):
            twist = message.twist[index]
            body_vx, body_vy = world_velocity_to_body(
                twist.linear.x, twist.linear.y, world_to_base[2]
            )
            odom_message.twist.twist.linear.x = body_vx
            odom_message.twist.twist.linear.y = body_vy
            odom_message.twist.twist.angular.z = twist.angular.z
        self.odom_publisher.publish(odom_message)
        return odom_to_base

    @staticmethod
    def _pose2d(pose):
        q = pose.orientation
        return (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    @staticmethod
    def _transform2d(transform):
        q = transform.rotation
        return (
            float(transform.translation.x),
            float(transform.translation.y),
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def _set_initial_error(self, true_map_to_base, requested):
        self._correction_error = pose_error(true_map_to_base, requested)
        # Keep the first publish at the requested pose. Correction begins with
        # the following model-state sample.
        self._last_correction_time_ns = None
        self.get_logger().info(
            "applied initial pose x=%.3f y=%.3f yaw=%.3f; "
            "correcting bias dx=%.3f dy=%.3f dyaw=%.3f"
            % (requested + self._correction_error)
        )

    def _on_robot_status(self, message):
        map_name = str(message.current_map or "").strip()
        if not map_name or map_name == self._active_map_name:
            return

        try:
            map_file = map_yaml_for_name(self._initial_map_file, map_name)
            world_to_map = map_to_world_from_files(map_file, self.world_file)
        except RuntimeError as exc:
            self.get_logger().warning(
                "ignore truth map switch %r: %s" % (map_name, exc),
                throttle_duration_sec=5.0,
            )
            return

        recent_initial = None
        if self._pending_initial is not None:
            recent_initial = self._pending_initial
        elif (
            self._last_initial_request is not None
            and self._last_initial_wall_time is not None
            and time.monotonic() - self._last_initial_wall_time
            <= self._MAP_SWITCH_INITIAL_WINDOW
        ):
            # task_manager intentionally publishes initialpose before it writes
            # RobotStatus.current_map. Re-apply that request against the new
            # map truth instead of carrying a floor-sized error forward.
            recent_initial = self._last_initial_request

        old_map_name = self._active_map_name
        self.map_file = str(map_file)
        self._active_map_name = map_name
        self._true_map_to_world = inverse(world_to_map)
        self._correction_error = (0.0, 0.0, 0.0)
        self._last_correction_time_ns = None
        self._pending_initial = None
        if recent_initial is not None:
            # Apply against a newly received model sample, never a cached pose
            # from before a map switch or Gazebo teleport.
            self._pending_initial = recent_initial
            self._last_initial_request = None
            self._last_initial_wall_time = None

        self.get_logger().info(
            "switched Gazebo truth map %s -> %s yaml=%s"
            % (old_map_name, map_name, self.map_file)
        )

    def _on_initial_pose(self, message):
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                "ignore initial pose in frame %r; expected %r"
                % (message.header.frame_id, self.map_frame)
            )
            return
        requested = self._pose2d(message.pose.pose)
        self._last_initial_request = requested
        self._last_initial_wall_time = time.monotonic()
        # ModelStates has no header. Use a depth-one queue and defer the bias
        # calculation until its next callback rather than using the cached pose.
        self._pending_initial = requested

    def _on_model_states(self, message):
        try:
            index = message.name.index(self.robot_model)
        except ValueError:
            if not self._missing_model_logged:
                self.get_logger().warning(
                    "Gazebo model %r not present on model_states" % self.robot_model
                )
                self._missing_model_logged = True
            return
        self._missing_model_logged = False
        world_to_base = self._pose2d(message.pose[index])
        self._last_world_to_base = world_to_base
        true_map_to_base = compose(self._true_map_to_world, world_to_base)
        if self._pending_initial is not None:
            self._set_initial_error(true_map_to_base, self._pending_initial)
            self._pending_initial = None

        now = self.get_clock().now()
        now_ns = now.nanoseconds
        stamp = now.to_msg()
        if self._owns_fallback_odom:
            odom_to_base = self._publish_fallback_odom(
                message, index, world_to_base, stamp
            )
        else:
            odom_to_base_msg = None
            odom_error = None
            try:
                odom_to_base_msg = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, Time()
                )
                odom_stamp_ns = self._stamp_nanoseconds(
                    odom_to_base_msg.header.stamp
                )
                age_seconds = (now_ns - odom_stamp_ns) / 1.0e9
                if odom_stamp_ns <= 0 or age_seconds > self.external_odom_timeout:
                    odom_error = "latest transform is %.3f seconds old" % age_seconds
            except Exception as exc:
                odom_error = str(exc)

            if odom_error is not None:
                missing_now = time.monotonic()
                if self._odom_missing_since is None:
                    self._odom_missing_since = missing_now
                if missing_now - self._odom_missing_since < self.external_odom_timeout:
                    self.get_logger().warning(
                        "waiting for fresh odom->base before publishing truth: %s"
                        % odom_error,
                        throttle_duration_sec=2.0,
                    )
                    return
                seed = None
                if odom_to_base_msg is not None:
                    seed = self._transform2d(odom_to_base_msg.transform)
                self._activate_fallback_odom(world_to_base, seed)
                odom_to_base = self._publish_fallback_odom(
                    message, index, world_to_base, stamp
                )
            else:
                self._odom_missing_since = None
                odom_to_base = self._transform2d(odom_to_base_msg.transform)

        dt = bounded_time_step(
            self._last_correction_time_ns, now_ns, self._MAX_CORRECTION_DT
        )
        self._last_correction_time_ns = now_ns
        self._correction_error = decay_pose_error(
            self._correction_error,
            self.linear_correction_speed,
            self.angular_correction_speed,
            dt,
        )
        map_to_base = apply_pose_error(true_map_to_base, self._correction_error)
        map_to_odom = compose(map_to_base, inverse(odom_to_base))

        tf_message = TransformStamped()
        tf_message.header.stamp = stamp
        tf_message.header.frame_id = self.map_frame
        tf_message.child_frame_id = self.odom_frame
        tf_message.transform.translation.x = map_to_odom[0]
        tf_message.transform.translation.y = map_to_odom[1]
        qx, qy, qz, qw = yaw_quaternion(map_to_odom[2])
        tf_message.transform.rotation.x = qx
        tf_message.transform.rotation.y = qy
        tf_message.transform.rotation.z = qz
        tf_message.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_message)

        pose_message = PoseWithCovarianceStamped()
        pose_message.header.stamp = stamp
        pose_message.header.frame_id = self.map_frame
        pose_message.pose.pose.position.x = map_to_base[0]
        pose_message.pose.pose.position.y = map_to_base[1]
        qx, qy, qz, qw = yaw_quaternion(map_to_base[2])
        pose_message.pose.pose.orientation.x = qx
        pose_message.pose.pose.orientation.y = qy
        pose_message.pose.pose.orientation.z = qz
        pose_message.pose.pose.orientation.w = qw
        pose_message.pose.covariance[0] = self.covariance
        pose_message.pose.covariance[7] = self.covariance
        pose_message.pose.covariance[35] = self.covariance
        self.pose_publisher.publish(pose_message)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboGroundTruthLocalizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
