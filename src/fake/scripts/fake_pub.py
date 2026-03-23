"""
Fake TF + /{robot}/scan_2d + /{robot}/planned_path + /{robot}/current_map.

TF 树（单机示例 robot1）::

    map → robot1/odom → robot1/base_link → robot1/scan_2d
                                      └── robot1/imu

虚拟规划路径为 map 系下闭合圆轨迹，机体沿路径行驶。

与 Web 切图/重定位联动（与 ``openDelivery`` 后端发布的话题一致）：

- 订阅 ``std_msgs/String`` ``/{robot_name}/current_map``：更新内部楼层 id（与 Web「仅切图」一致）。
- 订阅 ``geometry_msgs/PoseWithCovarianceStamped`` ``/{robot_name}/initial``：按该位姿 **接回** 圆周演示——
  将圆心与弧长对齐到 ``(x,y,yaw)`` 后 **继续沿轨迹运动**（不再长期停住）。

``planned_path`` 与上述联动：**切图**时按地图 id 重算圆心并重建路径；**重定位**时重建与当前圆周一致的闭合路径，机体随后沿该圆行驶。

示例::

    python3 fake_pub.py --ros-args -p robot_name:=robot1 -p current_map:=nh_114
"""

from __future__ import annotations

import math
import random
import re
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
)
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _sanitize_robot_prefix(name: str, fallback: str) -> str:
    s = (name or "").strip()
    s = re.sub(r"[^\w\-]+", "_", s).strip("_")
    return s if s else fallback


def _yaw_from_quaternion(q: Quaternion) -> float:
    """Yaw (rad) from unit quaternion, same convention as map/base_link."""
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _build_loop_path(frame_id: str, cx: float, cy: float, radius: float, n: int = 72) -> Path:
    p = Path()
    p.header.frame_id = frame_id
    for i in range(n + 1):
        th = 2.0 * math.pi * i / n
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = cx + radius * math.cos(th)
        ps.pose.position.y = cy + radius * math.sin(th)
        ps.pose.position.z = 0.0
        ps.pose.orientation = _quat_yaw(th + math.pi / 2.0)
        p.poses.append(ps)
    return p


def _stable_map_hash(map_id: str) -> int:
    """Deterministic hash (per string), for demo path center offsets."""
    acc = 5381
    for b in map_id.encode("utf-8", errors="ignore"):
        acc = ((acc << 5) + acc + b) & 0xFFFFFFFF
    return acc


def _center_from_map_id(
    map_id: str, base_cx: float, base_cy: float, radius: float
) -> Tuple[float, float]:
    """Perturb circle center from map name so 切图后规划路径可见变化。"""
    s = (map_id or "").strip()
    if not s:
        return base_cx, base_cy
    h = _stable_map_hash(s)
    span = max(radius * 3.0, 4.0)
    fx = (h & 0xFFFF) / 65535.0
    fy = ((h >> 16) & 0xFFFF) / 65535.0
    dx = (fx - 0.5) * span
    dy = (fy - 0.5) * span
    return base_cx + dx, base_cy + dy


def _pose_on_circle(cx: float, cy: float, radius: float, arc_len: float) -> Tuple[float, float, float]:
    """弧长沿圆周前进，返回 (x, y, yaw) map 系。"""
    if radius <= 1e-6:
        return cx, cy, 0.0
    theta = (arc_len / radius) % (2.0 * math.pi)
    x = cx + radius * math.cos(theta)
    y = cy + radius * math.sin(theta)
    yaw = theta + math.pi / 2.0
    return x, y, yaw


def _circle_motion_from_seed(
    ix: float, iy: float, yaw: float, radius: float
) -> Tuple[float, float, float]:
    """给定轨迹上一点 (ix,iy) 与朝向 yaw，求圆心 (cx,cy) 与弧长起点，使 _pose_on_circle 从该点开始连续运动。"""
    if radius <= 1e-9:
        return ix, iy, 0.0
    theta = (yaw - math.pi / 2.0) % (2.0 * math.pi)
    cx = ix - radius * math.cos(theta)
    cy = iy - radius * math.sin(theta)
    arc_len = theta * radius
    return cx, cy, arc_len


class FakeRobotNode(Node):
    def __init__(self):
        super().__init__("fake_robots_tf")

        self.declare_parameter("robot_name", "robot1")
        self.declare_parameter("second_robot_name", "robot2")
        self.declare_parameter("publish_second_robot", True)
        self.declare_parameter("current_map", "nh_102")
        self.declare_parameter("second_current_map", "")

        self.declare_parameter("path_radius", 2.0)
        self.declare_parameter("path_center_x", 3.0)
        self.declare_parameter("path_center_y", 3.0)
        self.declare_parameter("path_speed", 0.35)

        self.declare_parameter("path_radius_2", 1.2)
        self.declare_parameter("path_center_x_2", 5.5)
        self.declare_parameter("path_center_y_2", 3.5)
        self.declare_parameter("path_speed_2", 0.28)

        self.declare_parameter("scan_x_offset", 0.2)
        self.declare_parameter("scan_y_offset", 0.0)
        self.declare_parameter("imu_z_offset", 0.02)

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

        self._r1 = float(self.get_parameter("path_radius").value)
        self._default_cx1 = float(self.get_parameter("path_center_x").value)
        self._default_cy1 = float(self.get_parameter("path_center_y").value)
        self._sp1 = float(self.get_parameter("path_speed").value)
        self._r2 = float(self.get_parameter("path_radius_2").value)
        self._default_cx2 = float(self.get_parameter("path_center_x_2").value)
        self._default_cy2 = float(self.get_parameter("path_center_y_2").value)
        self._sp2 = float(self.get_parameter("path_speed_2").value)
        self._cx1, self._cy1 = _center_from_map_id(
            self.current_map, self._default_cx1, self._default_cy1, self._r1
        )
        self._cx2, self._cy2 = _center_from_map_id(
            self.second_current_map, self._default_cx2, self._default_cy2, self._r2
        )
        self._scan_x = float(self.get_parameter("scan_x_offset").value)
        self._scan_y = float(self.get_parameter("scan_y_offset").value)
        self._imu_z = float(self.get_parameter("imu_z_offset").value)

        self._arc1 = 0.0
        self._arc2 = 0.0

        self.br = TransformBroadcaster(self)
        scan_topic = f"/{self.robot_name}/scan_2d"
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.path_pub = self.create_publisher(Path, f"/{self.robot_name}/planned_path", 10)

        self.current_map_topic = f"/{self.robot_name}/current_map"
        self.map_pub = self.create_publisher(String, self.current_map_topic, 10)
        self.map_pub_second = None
        self.second_map_topic = None
        self.path_pub_second = None

        if (
            self.publish_second_robot
            and self.second_robot_name
            and self.second_robot_name != self.robot_name
        ):
            self.second_map_topic = f"/{self.second_robot_name}/current_map"
            self.map_pub_second = self.create_publisher(String, self.second_map_topic, 10)
            self.scan_pub_2 = self.create_publisher(
                LaserScan, f"/{self.second_robot_name}/scan_2d", 10
            )
            self.path_pub_second = self.create_publisher(
                Path, f"/{self.second_robot_name}/planned_path", 10
            )
        else:
            self.scan_pub_2 = None

        self._rebuild_path_robot1()
        self._path_msg_2 = None
        if self.path_pub_second:
            self._rebuild_path_robot2()

        self.get_logger().info(
            f"robot_name={self.robot_name!r}, scan=/{self.robot_name}/scan_2d, "
            f"path=/{self.robot_name}/planned_path, current_map={self.current_map_topic!r}"
            + (
                f" | second={self.second_robot_name!r}"
                if self.path_pub_second
                else ""
            )
        )

        self.create_subscription(
            String,
            self.current_map_topic,
            self._on_current_map_robot1,
            10,
        )
        initial_topic_1 = f"/{self.robot_name}/initial"
        self.create_subscription(
            PoseWithCovarianceStamped,
            initial_topic_1,
            self._on_initial_pose_robot1,
            10,
        )
        self.get_logger().info(f"subscribing String {self.current_map_topic!r} + initial {initial_topic_1!r}")

        if self.second_map_topic:
            self.create_subscription(
                String,
                self.second_map_topic,
                self._on_current_map_robot2,
                10,
            )
            initial_topic_2 = f"/{self.second_robot_name}/initial"
            self.create_subscription(
                PoseWithCovarianceStamped,
                initial_topic_2,
                self._on_initial_pose_robot2,
                10,
            )
            self.get_logger().info(
                f"subscribing String {self.second_map_topic!r} + initial {initial_topic_2!r}"
            )

        self.t = 0.0
        self.timer = self.create_timer(0.1, self.update)

    def _rebuild_path_robot1(self) -> None:
        self._path_msg_1 = _build_loop_path("map", self._cx1, self._cy1, self._r1)

    def _rebuild_path_robot2(self) -> None:
        if self.path_pub_second is None:
            return
        self._path_msg_2 = _build_loop_path("map", self._cx2, self._cy2, self._r2)

    def _apply_map_change_robot1(self, map_id: str) -> None:
        self.current_map = map_id
        self._cx1, self._cy1 = _center_from_map_id(
            map_id, self._default_cx1, self._default_cy1, self._r1
        )
        self._arc1 = 0.0
        self._rebuild_path_robot1()

    def _apply_map_change_robot2(self, map_id: str) -> None:
        self.second_current_map = map_id
        self._cx2, self._cy2 = _center_from_map_id(
            map_id, self._default_cx2, self._default_cy2, self._r2
        )
        self._arc2 = 0.0
        self._rebuild_path_robot2()

    def _on_current_map_robot1(self, msg: String) -> None:
        d = (msg.data or "").strip()
        if not d or d == self.current_map:
            return
        self._apply_map_change_robot1(d)
        self.get_logger().info(f"robot1 current_map (sub) -> {d!r}")

    def _on_current_map_robot2(self, msg: String) -> None:
        d = (msg.data or "").strip()
        if not d or d == self.second_current_map:
            return
        self._apply_map_change_robot2(d)
        self.get_logger().info(f"robot2 current_map (sub) -> {d!r}")

    def _on_initial_pose_robot1(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(q)
        self._cx1, self._cy1, self._arc1 = _circle_motion_from_seed(
            float(p.x), float(p.y), yaw, self._r1
        )
        self._rebuild_path_robot1()
        self.get_logger().info(
            f"robot1 initial -> seed motion at x={p.x:.3f} y={p.y:.3f} yaw={yaw:.3f}, "
            f"center=({self._cx1:.3f},{self._cy1:.3f})"
        )

    def _on_initial_pose_robot2(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quaternion(q)
        self._cx2, self._cy2, self._arc2 = _circle_motion_from_seed(
            float(p.x), float(p.y), yaw, self._r2
        )
        self._rebuild_path_robot2()
        self.get_logger().info(
            f"robot2 initial -> seed motion at x={p.x:.3f} y={p.y:.3f} yaw={yaw:.3f}, "
            f"center=({self._cx2:.3f},{self._cy2:.3f})"
        )

    def _static_tf_base_to_sensor(
        self, stamp, prefix: str, child_suffix: str, tx: float, ty: float, tz: float
    ) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = f"{prefix}/base_link"
        t.child_frame_id = f"{prefix}/{child_suffix}"
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.w = 1.0
        return t

    def _robot_chain(
        self,
        stamp,
        prefix: str,
        x: float,
        y: float,
        yaw: float,
        map_offset: Tuple[float, float],
    ) -> List[TransformStamped]:
        ox, oy = map_offset
        odom_frame = f"{prefix}/odom"
        base_frame = f"{prefix}/base_link"

        m = TransformStamped()
        m.header.stamp = stamp
        m.header.frame_id = "map"
        m.child_frame_id = odom_frame
        m.transform.translation.x = float(ox)
        m.transform.translation.y = float(oy)
        m.transform.rotation.w = 1.0

        b = TransformStamped()
        b.header.stamp = stamp
        b.header.frame_id = odom_frame
        b.child_frame_id = base_frame
        b.transform.translation.x = float(x - ox)
        b.transform.translation.y = float(y - oy)
        b.transform.rotation = _quat_yaw(yaw)

        scan_tf = self._static_tf_base_to_sensor(
            stamp, prefix, "scan_2d", self._scan_x, self._scan_y, 0.0
        )
        imu_tf = self._static_tf_base_to_sensor(
            stamp, prefix, "imu", 0.0, 0.0, self._imu_z
        )
        return [m, b, scan_tf, imu_tf]

    def update(self):
        now = self.get_clock().now().to_msg()
        dt = 0.1

        self._arc1 += self._sp1 * dt
        x1, y1, yaw1 = _pose_on_circle(self._cx1, self._cy1, self._r1, self._arc1)

        msgs: List[TransformStamped] = self._robot_chain(
            now, self.robot_name, x1, y1, yaw1, (0.0, 0.0)
        )

        if self.path_pub_second and self.second_robot_name:
            self._arc2 += self._sp2 * dt
            x2, y2, yaw2 = _pose_on_circle(
                self._cx2, self._cy2, self._r2, self._arc2 + 1.0
            )
            msgs.extend(
                self._robot_chain(
                    now, self.second_robot_name, x2, y2, yaw2, (0.0, 0.0)
                )
            )

        self.br.sendTransform(msgs)

        self._path_msg_1.header.stamp = now
        self.path_pub.publish(self._path_msg_1)
        if self._path_msg_2 is not None and self.path_pub_second is not None:
            self._path_msg_2.header.stamp = now
            self.path_pub_second.publish(self._path_msg_2)

        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = f"{self.robot_name}/scan_2d"
        scan.angle_min = -1.57
        scan.angle_max = 1.57
        scan.angle_increment = 0.02
        scan.range_min = 0.1
        scan.range_max = 12.0
        num_points = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        self.t += 0.05
        scan.ranges = [
            min(
                scan.range_max,
                4.0
                + 1.5 * math.sin(self.t + i * 0.05)
                + 0.3 * math.cos(2 * self.t + i * 0.02)
                + random.uniform(-0.08, 0.08),
            )
            for i in range(num_points)
        ]
        self.scan_pub.publish(scan)

        if self.scan_pub_2 is not None:
            scan2 = LaserScan()
            scan2.header.stamp = now
            scan2.header.frame_id = f"{self.second_robot_name}/scan_2d"
            scan2.angle_min = scan.angle_min
            scan2.angle_max = scan.angle_max
            scan2.angle_increment = scan.angle_increment
            scan2.range_min = scan.range_min
            scan2.range_max = scan.range_max
            scan2.ranges = [
                min(
                    scan.range_max,
                    3.5
                    + math.sin(self.t * 1.3 + i * 0.04)
                    + random.uniform(-0.1, 0.1),
                )
                for i in range(num_points)
            ]
            self.scan_pub_2.publish(scan2)

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
