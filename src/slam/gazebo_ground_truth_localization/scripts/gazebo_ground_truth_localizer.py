#!/usr/bin/env python3
"""Publish exact Gazebo localization while honoring normal initial-pose commands."""

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from ground_truth_math import (
    alignment_for_initial,
    compose,
    inverse,
    map_to_world_from_files,
    yaw_from_quaternion,
    yaw_quaternion,
)


class GazeboGroundTruthLocalizer(Node):
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
        if not self.robot_model:
            namespace = self.get_namespace().strip("/")
            self.robot_model = namespace.split("/")[0] if namespace else ""
        if not self.robot_model:
            raise RuntimeError("robot_model parameter is required")
        if not self.map_file or not self.world_file:
            raise RuntimeError("map_file and world_file parameters are required")

        world_to_map = map_to_world_from_files(self.map_file, self.world_file)
        self._map_to_world = inverse(world_to_map)
        self._last_world_to_base = None
        self._pending_initial = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, self.pose_topic, 10
        )
        self.create_subscription(ModelStates, "model_states", self._on_model_states, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self._on_initial_pose, 10
        )
        self._missing_model_logged = False
        self.get_logger().info(
            "Gazebo truth localization ready: model=%s map=%s world=%s"
            % (self.robot_model, self.map_file, self.world_file)
        )

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

    def _on_initial_pose(self, message):
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                "ignore initial pose in frame %r; expected %r"
                % (message.header.frame_id, self.map_frame)
            )
            return
        requested = self._pose2d(message.pose.pose)
        if self._last_world_to_base is None:
            self._pending_initial = requested
            self.get_logger().info("queued initial pose until first Gazebo model state")
            return
        self._map_to_world = alignment_for_initial(self._last_world_to_base, requested)
        self._pending_initial = None
        self.get_logger().info(
            "applied exact initial pose x=%.3f y=%.3f yaw=%.3f" % requested
        )

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
        if self._pending_initial is not None:
            self._map_to_world = alignment_for_initial(
                world_to_base, self._pending_initial
            )
            self._pending_initial = None

        try:
            odom_to_base_msg = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time()
            )
        except Exception as exc:
            self.get_logger().warning(
                "waiting for odom->base before publishing truth: %s" % exc,
                throttle_duration_sec=2.0,
            )
            return

        map_to_base = compose(self._map_to_world, world_to_base)
        odom_to_base = self._transform2d(odom_to_base_msg.transform)
        map_to_odom = compose(map_to_base, inverse(odom_to_base))
        stamp = self.get_clock().now().to_msg()

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
