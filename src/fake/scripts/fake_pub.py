"""
Fake TF + scan + /{robot_name}/current_map for web stack 联调.

当前地图话题带机器人前缀，例如 robot1 → ``/robot1/current_map``。

示例（与后端 ``ROS_FRAME_BASE=robot1/base_link``、桥接订阅 ``/robot1/current_map`` 一致）::

    python3 fake_pub.py --ros-args -p robot_name:=robot1 -p current_map:=nh_102

第二台车默认会一起发布；只要一台时::

    python3 fake_pub.py --ros-args -p robot_name:=robot1 -p publish_second_robot:=false
"""

from __future__ import annotations

import math
import random
import re
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _sanitize_robot_prefix(name: str, fallback: str) -> str:
    """ROS frame 前缀：去空白、禁止路径符。"""
    s = (name or "").strip()
    s = re.sub(r"[^\w\-]+", "_", s).strip("_")
    return s if s else fallback


class FakeRobotNode(Node):
    """
    Publishes TF on /tf::

      map → {robot_name}/odom → {robot_name}/base_link
      [optional] map → {second}/odom → {second}/base_link
    """

    def __init__(self):
        super().__init__("fake_robots_tf")

        self.declare_parameter("robot_name", "robot1")
        self.declare_parameter("second_robot_name", "robot2")
        self.declare_parameter("publish_second_robot", True)
        self.declare_parameter("current_map", "nh_102")
        self.declare_parameter("second_current_map", "")

        self.robot_name = _sanitize_robot_prefix(
            str(self.get_parameter("robot_name").value), "robot1"
        )
        second_raw = self.get_parameter("second_robot_name").value
        second_str = str(second_raw).strip() if second_raw is not None else "robot2"
        self.second_robot_name = (
            _sanitize_robot_prefix(second_str, "robot2") if second_str else ""
        )
        self.publish_second_robot = bool(
            self.get_parameter("publish_second_robot").value
        )
        self.current_map = str(self.get_parameter("current_map").value)
        second_map_raw = self.get_parameter("second_current_map").value
        second_map_str = str(second_map_raw).strip() if second_map_raw else ""
        self.second_current_map = second_map_str if second_map_str else self.current_map

        self.br = TransformBroadcaster(self)
        scan_topic = f"/{self.robot_name}/scan"
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.current_map_topic = f"/{self.robot_name}/current_map"
        self.map_pub = self.create_publisher(String, self.current_map_topic, 10)
        self.map_pub_second = None
        self.second_map_topic = None
        if (
            self.publish_second_robot
            and self.second_robot_name
            and self.second_robot_name != self.robot_name
        ):
            self.second_map_topic = f"/{self.second_robot_name}/current_map"
            self.map_pub_second = self.create_publisher(String, self.second_map_topic, 10)

        self.get_logger().info(
            f"robot_name={self.robot_name!r}, topics: scan=/{self.robot_name}/scan, "
            f"current_map={self.current_map_topic!r}"
            + (
                f", second_current_map={self.second_map_topic!r}"
                if self.second_map_topic
                else ""
            )
            + f", map_data={self.current_map!r}, "
            f"second={self.second_robot_name!r} (publish_second={self.publish_second_robot})"
        )

        self.t = 0.0
        self.timer = self.create_timer(0.1, self.update)

    def _chain(
        self,
        prefix: str,
        phase: float,
        offset_xy: Optional[Tuple[float, float]] = None,
    ):
        """Returns (map->prefix/odom, prefix/odom->prefix/base_link, base_frame)."""
        now = self.get_clock().now().to_msg()
        ox, oy = offset_xy if offset_xy else (0.0, 0.0)
        odom_frame = f"{prefix}/odom"
        base_frame = f"{prefix}/base_link"

        m = TransformStamped()
        m.header.stamp = now
        m.header.frame_id = "map"
        m.child_frame_id = odom_frame
        m.transform.translation.x = float(ox)
        m.transform.translation.y = float(oy)
        m.transform.rotation.w = 1.0

        b = TransformStamped()
        b.header.stamp = now
        b.header.frame_id = odom_frame
        b.child_frame_id = base_frame
        b.transform.translation.x = math.cos(phase)
        b.transform.translation.y = math.sin(phase)
        b.transform.rotation.z = math.sin(phase / 2)
        b.transform.rotation.w = math.cos(phase / 2)
        return m, b, base_frame

    def update(self):
        now = self.get_clock().now().to_msg()

        msgs: List[TransformStamped] = []

        m1, b1, base1 = self._chain(self.robot_name, self.t)
        msgs.extend([m1, b1])
        self.t += 0.05

        if (
            self.publish_second_robot
            and self.second_robot_name
            and self.second_robot_name != self.robot_name
        ):
            t2 = self.t + 1.0
            m2, b2, _ = self._chain(
                self.second_robot_name, t2, offset_xy=(2.0, 1.0)
            )
            msgs.extend([m2, b2])

        self.br.sendTransform(msgs)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = f"{self.robot_name}/base_link"
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 10.0
        num_points = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [
            5.0 + 0.5 * math.sin(self.t + i * 0.1) + random.uniform(-0.1, 0.1)
            for i in range(num_points)
        ]
        self.scan_pub.publish(scan)

        map_msg = String()
        map_msg.data = self.current_map
        self.map_pub.publish(map_msg)

        if self.map_pub_second is not None:
            map2 = String()
            map2.data = self.second_current_map
            self.map_pub_second.publish(map2)


def main():
    rclpy.init()
    node = FakeRobotNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
