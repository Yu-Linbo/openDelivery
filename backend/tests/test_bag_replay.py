import json
import math
import sqlite3
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


BACKEND = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND))

from bag_replay import BagReplayError, extract_replay, merge_replays  # noqa: E402


class CdrWriter:
    def __init__(self):
        self.data = bytearray(b"\x00\x01\x00\x00")

    def align(self, size):
        relative = len(self.data) - 4
        self.data.extend(b"\x00" * ((-relative) % size))

    def pack(self, fmt, value, size):
        self.align(size)
        self.data.extend(struct.pack("<" + fmt, value))

    def u32(self, value):
        self.pack("I", value, 4)

    def i32(self, value):
        self.pack("i", value, 4)

    def f32(self, value):
        self.pack("f", value, 4)

    def f64(self, value):
        self.pack("d", value, 8)

    def boolean(self, value):
        self.pack("B", int(bool(value)), 1)

    def string(self, value):
        raw = str(value).encode("utf-8") + b"\x00"
        self.u32(len(raw))
        self.data.extend(raw)

    def header(self, frame_id="map", stamp_sec=1, stamp_nanosec=2):
        self.i32(stamp_sec)
        self.u32(stamp_nanosec)
        self.string(frame_id)
    def octets(self, value):
        raw = bytes(value)
        self.u32(len(raw))
        self.data.extend(raw)


    def bytes(self):
        return bytes(self.data)


def odometry_payload():
    out = CdrWriter()
    out.header("robot2/odom")
    out.string("robot2/base_footprint")
    for value in (1.25, -2.5, 0.0):
        out.f64(value)
    yaw = 0.5
    for value in (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)):
        out.f64(value)
    for _ in range(36):
        out.f64(0.0)
    for value in (0.4, 0.0, 0.0, 0.0, 0.0, -0.2):
        out.f64(value)
    for _ in range(36):
        out.f64(0.0)
    return out.bytes()


def robot_status_payload(current_map="test_101"):
    out = CdrWriter()
    out.header("map")
    for value in (
        "robot2",
        "OP1",
        current_map,
        "lobby;lift:2.0;",
        "ready",
        "delivery",
        "AUTO",
        "amcl",
    ):
        out.string(value)
    out.boolean(True)
    out.f32(0.75)
    return out.bytes()



def task_status_payload():
    out = CdrWriter()
    out.header("map")
    out.string("delivery-42")
    out.u32(2)
    out.string("navigation:pickup")
    out.string("navigation:dropoff")
    out.u32(2)
    out.string("Finished")
    out.string("Navigating")
    out.string("Navigating")
    out.string("moving to dropoff")
    out.u32(1)
    out.u32(2)
    return out.bytes()

def image_payload(encoding, pixels, step, width=2, height=2):
    out = CdrWriter()
    out.header("robot2/camera")
    out.u32(height)
    out.u32(width)
    out.string(encoding)
    out.boolean(False)
    out.u32(step)
    out.octets(pixels)
    return out.bytes()


def tf_payload(transforms, stamp_sec=1):
    out = CdrWriter()
    out.u32(len(transforms))
    for parent, child, x, y, yaw in transforms:
        out.header(parent, stamp_sec=stamp_sec,
                   stamp_nanosec=0 if stamp_sec == 0 else 2)
        out.string(child)
        for value in (x, y, 0.0):
            out.f64(value)
        for value in (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)):
            out.f64(value)
    return out.bytes()


def laser_scan_payload(frame_id="robot2/laser_link", stamp_sec=1):
    out = CdrWriter()
    out.header(frame_id, stamp_sec=stamp_sec,
               stamp_nanosec=0 if stamp_sec == 0 else 2)
    for value in (0.0, 0.0, 1.0, 0.0, 0.1, 0.1, 10.0):
        out.f32(value)
    out.u32(1)
    out.f32(1.0)
    out.u32(0)
    return out.bytes()


def map_pose_payload(x, y, yaw=0.0, stamp_sec=1, frame_id="map"):
    out = CdrWriter()
    out.header(frame_id, stamp_sec=stamp_sec)
    for value in (x, y, 0.0):
        out.f64(value)
    for value in (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)):
        out.f64(value)
    for _ in range(36):
        out.f64(0.0)
    return out.bytes()


class BagReplayTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.bag = Path(self.tmp.name) / "sample_terminal_bag"
        self.bag.mkdir()
        database = self.bag / "sample_0.db3"
        with sqlite3.connect(str(database)) as connection:
            connection.executescript(
                """
                CREATE TABLE topics(
                  id INTEGER PRIMARY KEY,
                  name TEXT NOT NULL,
                  type TEXT NOT NULL,
                  serialization_format TEXT NOT NULL
                );
                CREATE TABLE messages(
                  id INTEGER PRIMARY KEY,
                  topic_id INTEGER NOT NULL,
                  timestamp INTEGER NOT NULL,
                  data BLOB NOT NULL
                );
                """
            )
            connection.execute(
                "INSERT INTO topics VALUES(1, ?, ?, 'cdr')",
                ("/robot2/odom", "nav_msgs/msg/Odometry"),
            )
            connection.execute(
                "INSERT INTO topics VALUES(2, ?, ?, 'cdr')",
                ("/robot2/robot_status", "custom_msgs_srvs/msg/RobotStatus"),
            )
            connection.execute(
                "INSERT INTO topics VALUES(3, ?, ?, 'cdr')",
                ("/robot2/task_status", "custom_msgs_srvs/msg/TaskStatus"),
            )
            connection.execute(
                "INSERT INTO topics VALUES(4, ?, ?, 'cdr')",
                ("/robot2/front_camera/image_raw", "sensor_msgs/msg/Image"),
            )
            connection.execute(
                "INSERT INTO topics VALUES(5, ?, ?, 'cdr')",
                ("/robot2/front_down_camera/image_raw", "sensor_msgs/msg/Image"),
            )
            connection.execute(
                "INSERT INTO messages VALUES(1, 1, 1000000000, ?)",
                (odometry_payload(),),
            )
            connection.execute(
                "INSERT INTO messages VALUES(3, 3, 1750000000, ?)",
                (task_status_payload(),),
            )
            connection.execute(
                "INSERT INTO messages VALUES(2, 2, 1500000000, ?)",
                (robot_status_payload(),),
            )
            connection.execute(
                "INSERT INTO messages VALUES(6, 2, 1700000000, ?)",
                (robot_status_payload("test_102"),),
            )
            connection.execute(
                "INSERT INTO messages VALUES(4, 4, 1250000000, ?)",
                (
                    image_payload(
                        "rgb8", bytes(range(256)) * 2400, 1920, 640, 320
                    ),
                ),
            )
            connection.execute(
                "INSERT INTO messages VALUES(5, 5, 1600000000, ?)",
                (image_payload("mono8", bytes((0, 64, 128, 255)), 2),),
            )

    def tearDown(self):
        self.tmp.cleanup()

    def test_extracts_browser_timeline_without_ros(self):
        result = extract_replay(self.bag, robot_name="robot2")
        self.assertTrue(result["offline"])
        self.assertEqual(result["robot_name"], "robot2")
        self.assertEqual(result["initial_map_name"], "test_101")
        self.assertEqual(result["map_name"], "test_102")
        self.assertEqual(result["message_count"], 6)
        self.assertAlmostEqual(result["duration"], 0.75)
        pose = result["timeline"]["poses"][0]
        self.assertAlmostEqual(pose["x"], 1.25)
        self.assertAlmostEqual(pose["y"], -2.5)
        self.assertAlmostEqual(pose["yaw"], 0.5, places=5)
        self.assertAlmostEqual(pose["linear_x"], 0.4)
        status = result["timeline"]["statuses"][0]
        self.assertEqual(status["task_status"], "delivery")
        self.assertAlmostEqual(status["task_progress"], 0.75)
        self.assertEqual(
            result["timeline"]["maps"],
            [{"t": 0.0, "current_map": "test_101"},
             {"t": 0.7, "current_map": "test_102"}],
        )
        task = result["timeline"]["tasks"][0]
        self.assertEqual(task["task_id"], "delivery-42")
        self.assertEqual(task["current_index"], 1)
        images = result["timeline"]["images"]
        self.assertEqual([row["camera"] for row in images], ["front", "front_down"])
        self.assertTrue(all(row["data_url"].startswith("data:image/jpeg;base64,") for row in images))
        self.assertEqual((images[0]["width"], images[0]["height"]), (640, 320))
        self.assertEqual((images[0]["preview_width"], images[0]["preview_height"]), (480, 240))
        self.assertEqual(
            images[0]["width"] * images[0]["preview_height"],
            images[0]["height"] * images[0]["preview_width"],
        )
        self.assertEqual(images[1]["encoding"], "mono8")

    def test_live_wal_reader_keeps_a_stable_snapshot(self):
        database = self.bag / "sample_0.db3"
        from bag_replay import _open_database

        with sqlite3.connect(str(database)) as writer:
            self.assertEqual(writer.execute("PRAGMA journal_mode=WAL").fetchone()[0], "wal")
        reader = _open_database(database)
        try:
            original_count = reader.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
            with sqlite3.connect(str(database)) as writer:
                writer.execute(
                    "INSERT INTO messages VALUES(8, 1, 1900000000, ?)",
                    (odometry_payload(),),
                )
            self.assertEqual(reader.execute("SELECT COUNT(*) FROM messages").fetchone()[0], original_count)
        finally:
            reader.close()
        with sqlite3.connect(str(database)) as connection:
            self.assertEqual(connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0], original_count + 1)

    def test_rosout_decode_and_multi_bag_time_alignment(self):
        payload = CdrWriter()
        payload.i32(12)
        payload.u32(500000000)
        payload.pack("B", 40, 1)
        for value in ("amcl", "failed <scan>", "amcl.cpp", "tick"):
            payload.string(value)
        payload.u32(42)
        with sqlite3.connect(str(self.bag / "sample_0.db3")) as connection:
            connection.execute("INSERT INTO topics VALUES(6, '/rosout', 'rcl_interfaces/msg/Log', 'cdr')")
            connection.execute("INSERT INTO messages VALUES(7, 6, 1600000000, ?)", (payload.bytes(),))
        result = extract_replay(self.bag, robot_name="robot2")
        row = result["timeline"]["logs"][0]
        self.assertEqual(row["level"], "ERROR")
        self.assertEqual(row["node"], "amcl")
        self.assertEqual(row["epoch"], 12.5)
        self.assertEqual(row["t"], 0.6)
        self.assertIn("source=amcl.cpp:42 tick", row["message"])
        result["bag"] = "first"
        second = dict(result, bag="second", start_time_ns=20000000000)
        merged = merge_replays([second, result])
        logs = merged["timeline"]["logs"]
        self.assertEqual(logs[0]["bag"], "first")
        self.assertEqual(logs[1]["segment_index"], 1)
        self.assertAlmostEqual(logs[1]["t"], result["duration"] + 0.600001)

    def test_database_removed_after_open_still_replays_images(self):
        database = self.bag / "sample_0.db3"
        from bag_replay import _open_database

        def open_then_remove(path):
            connection = _open_database(path)
            path.unlink()
            return connection

        with mock.patch("bag_replay._open_database", side_effect=open_then_remove):
            result = extract_replay(self.bag, robot_name="robot2")

        self.assertFalse(database.exists())
        self.assertEqual(len(result["timeline"]["images"]), 2)
        self.assertEqual(len(result["timeline"]["poses"]), 1)

    def test_corrupt_image_does_not_fail_other_replay_data(self):
        database = self.bag / "sample_0.db3"
        with sqlite3.connect(str(database)) as connection:
            connection.execute(
                "INSERT INTO messages VALUES(7, 4, 1800000000, ?)",
                (b"bad image",),
            )

        result = extract_replay(self.bag, robot_name="robot2")

        self.assertEqual(len(result["timeline"]["images"]), 2)
        self.assertTrue(result["timeline"]["poses"])
        self.assertTrue(any("/robot2/front_camera/image_raw" in warning for warning in result["warnings"]))

    def test_uses_recorder_status_sidecar_when_rosbag_misses_status_topic(self):
        database = self.bag / "sample_0.db3"
        with sqlite3.connect(str(database)) as connection:
            connection.execute("DELETE FROM messages WHERE topic_id=2")
        sidecar = {
            "version": 1,
            "statuses": [
                {
                    "timestamp_ns": 1200000000,
                    "robot_name": "robot2",
                    "robot_model": "OP1",
                    "current_map": "test_103",
                    "current_position": "lobby;",
                    "robot_status": "ready",
                    "task_status": "idle",
                    "control_status": "AUTO",
                    "localization_method": "amcl",
                    "is_simulation": True,
                    "task_progress": -1.0,
                }
            ],
        }
        (self.bag / ".opendelivery_robot_status.json").write_text(
            json.dumps(sidecar), encoding="utf-8"
        )

        result = extract_replay(self.bag, robot_name="robot2")

        self.assertEqual(result["initial_map_name"], "test_103")
        self.assertEqual(result["map_name"], "test_103")
        self.assertEqual(result["timeline"]["maps"], [{"t": 0.0, "current_map": "test_103"}])
        self.assertEqual(len(result["timeline"]["statuses"]), 1)
        self.assertEqual(result["timeline"]["statuses"][0]["source"], "recorder_sidecar")

    def test_does_not_duplicate_bag_statuses_with_sidecar(self):
        sidecar = {
            "version": 1,
            "statuses": [{
                "timestamp_ns": 1200000000,
                "robot_name": "robot2",
                "robot_status": "ready",
            }],
        }
        (self.bag / ".opendelivery_robot_status.json").write_text(
            json.dumps(sidecar), encoding="utf-8"
        )

        result = extract_replay(self.bag, robot_name="robot2")

        self.assertEqual(len(result["timeline"]["statuses"]), 2)
        self.assertTrue(
            all(row.get("source") != "recorder_sidecar" for row in result["timeline"]["statuses"])
        )

    def test_transforms_laser_scan_from_sensor_frame_into_map(self):
        database = self.bag / "sample_0.db3"
        with sqlite3.connect(str(database)) as connection:
            connection.execute(
                "INSERT INTO topics VALUES(6, ?, ?, 'cdr')",
                ("/robot2/tf_static", "tf2_msgs/msg/TFMessage"),
            )
            connection.execute(
                "INSERT INTO topics VALUES(7, ?, ?, 'cdr')",
                ("/robot2/tf", "tf2_msgs/msg/TFMessage"),
            )
            connection.execute(
                "INSERT INTO topics VALUES(8, ?, ?, 'cdr')",
                ("/robot2/scan_2d", "sensor_msgs/msg/LaserScan"),
            )
            connection.execute(
                "INSERT INTO messages VALUES(7, 6, 1300000000, ?)",
                (tf_payload([("robot2/base_footprint", "robot2/laser_link", 0.2, 0.0, 0.0)]),),
            )
            connection.execute(
                "INSERT INTO messages VALUES(8, 7, 1100000000, ?)",
                (tf_payload([("map", "robot2/base_footprint", 1.0, 2.0, math.pi / 2)]),),
            )
            connection.execute(
                "INSERT INTO messages VALUES(9, 8, 1200000000, ?)",
                (laser_scan_payload(),),
            )

        result = extract_replay(self.bag, robot_name="robot2")

        scan = result["timeline"]["scans"][0]
        self.assertEqual(scan["source_frame_id"], "robot2/laser_link")
        self.assertEqual(scan["frame_id"], "map")
        self.assertEqual(scan["coordinates"], "map")
        self.assertTrue(scan["tf_applied"])
        self.assertAlmostEqual(scan["points"][0], 1.0, places=4)
        self.assertAlmostEqual(scan["points"][1], 3.2, places=4)

    def test_uses_recorded_map_localization_pose_at_scan_time(self):
        database = self.bag / "sample_0.db3"
        with sqlite3.connect(str(database)) as connection:
            connection.executemany(
                "INSERT INTO topics VALUES(?, ?, ?, 'cdr')",
                [
                    (6, "/robot2/tf_static", "tf2_msgs/msg/TFMessage"),
                    (7, "/robot2/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
                    (8, "/robot2/scan_2d", "sensor_msgs/msg/LaserScan"),
                    (9, "/robot2/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
                ],
            )
            connection.executemany(
                "INSERT INTO messages VALUES(?, ?, ?, ?)",
                [
                    (7, 6, 1900000000, tf_payload([
                        ("robot2/base_footprint", "robot2/laser_link", 0.2, 0.0, 0.0)
                    ])),
                    (8, 7, 1950000000, map_pose_payload(1.0, 2.0, math.pi / 2, stamp_sec=10)),
                    (9, 8, 2000000000, laser_scan_payload(stamp_sec=10)),
                    # A future pose must not pull the scan ahead in playback.
                    (10, 7, 2100000000, map_pose_payload(9.0, 9.0, stamp_sec=11)),
                    (11, 9, 1920000000, map_pose_payload(99.0, 99.0, stamp_sec=10)),
                ],
            )

        result = extract_replay(self.bag, robot_name="robot2")

        scan = result["timeline"]["scans"][0]
        self.assertEqual(scan["coordinates"], "map")
        self.assertEqual(scan["frame_id"], "map")
        self.assertAlmostEqual(scan["points"][0], 1.0, places=4)
        self.assertAlmostEqual(scan["points"][1], 3.2, places=4)
        poses = result["timeline"]["poses"]
        self.assertEqual([pose["x"] for pose in poses], [1.0, 9.0])
        self.assertLess(poses[0]["t"], scan["t"])
        self.assertGreater(poses[1]["t"], scan["t"])

    def test_matches_localization_received_just_after_same_stamped_scan(self):
        database = self.bag / "sample_0.db3"
        with sqlite3.connect(str(database)) as connection:
            connection.executemany(
                "INSERT INTO topics VALUES(?, ?, ?, 'cdr')",
                [
                    (6, "/robot2/tf_static", "tf2_msgs/msg/TFMessage"),
                    (7, "/robot2/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"),
                    (8, "/robot2/scan_2d", "sensor_msgs/msg/LaserScan"),
                ],
            )
            connection.executemany(
                "INSERT INTO messages VALUES(?, ?, ?, ?)",
                [
                    (7, 6, 1900000000, tf_payload([
                        ("robot2/base_footprint", "robot2/laser_link", 0.2, 0.0, 0.0)
                    ])),
                    (8, 7, 1950000000, map_pose_payload(0.0, 0.0, stamp_sec=9)),
                    (9, 8, 2000000000, laser_scan_payload(stamp_sec=10)),
                    (10, 7, 2050000000, map_pose_payload(1.0, 2.0, stamp_sec=10)),
                ],
            )

        result = extract_replay(self.bag, robot_name="robot2")

        scan = result["timeline"]["scans"][0]
        self.assertEqual(scan["coordinates"], "map")
        self.assertAlmostEqual(scan["points"][0], 2.2, places=4)
        self.assertAlmostEqual(scan["points"][1], 2.0, places=4)
        self.assertLess(result["timeline"]["poses"][0]["t"], scan["t"])
        self.assertGreater(result["timeline"]["poses"][1]["t"], scan["t"])

    def test_uses_late_tf_with_matching_scan_ros_stamp_without_amcl(self):
        database = self.bag / "sample_0.db3"
        with sqlite3.connect(str(database)) as connection:
            connection.executemany(
                "INSERT INTO topics VALUES(?, ?, ?, 'cdr')",
                [
                    (6, "/robot2/tf_static", "tf2_msgs/msg/TFMessage"),
                    (7, "/robot2/tf", "tf2_msgs/msg/TFMessage"),
                    (8, "/robot2/scan_2d", "sensor_msgs/msg/LaserScan"),
                ],
            )
            connection.executemany(
                "INSERT INTO messages VALUES(?, ?, ?, ?)",
                [
                    (7, 6, 1900000000, tf_payload([
                        ("robot2/base_footprint", "robot2/laser_link", 0.2, 0.0, 0.0)
                    ])),
                    (8, 8, 2000000000, laser_scan_payload(stamp_sec=10)),
                    (9, 7, 3000000000, tf_payload([
                        ("map", "robot2/odom", 1.0, 2.0, math.pi / 2),
                        ("robot2/odom", "robot2/base_footprint", 0.0, 0.0, 0.0),
                    ], stamp_sec=10)),
                    # The same ROS time after a clock reset must not win.
                    (10, 7, 100000000000, tf_payload([
                        ("map", "robot2/odom", 99.0, 99.0, 0.0),
                        ("robot2/odom", "robot2/base_footprint", 0.0, 0.0, 0.0),
                    ], stamp_sec=10)),
                ],
            )

        result = extract_replay(self.bag, robot_name="robot2")

        scan = result["timeline"]["scans"][0]
        self.assertEqual(scan["coordinates"], "map")
        self.assertAlmostEqual(scan["points"][0], 1.0, places=4)
        self.assertAlmostEqual(scan["points"][1], 3.2, places=4)

    def test_falls_back_to_receive_order_when_scan_has_zero_ros_stamp(self):
        database = self.bag / "sample_0.db3"
        with sqlite3.connect(str(database)) as connection:
            connection.executemany(
                "INSERT INTO topics VALUES(?, ?, ?, 'cdr')",
                [
                    (6, "/robot2/tf_static", "tf2_msgs/msg/TFMessage"),
                    (7, "/robot2/tf", "tf2_msgs/msg/TFMessage"),
                    (8, "/robot2/scan_2d", "sensor_msgs/msg/LaserScan"),
                ],
            )
            connection.executemany(
                "INSERT INTO messages VALUES(?, ?, ?, ?)",
                [
                    (7, 6, 1900000000, tf_payload([
                        ("robot2/base_footprint", "robot2/laser_link", 0.2, 0.0, 0.0)
                    ])),
                    (8, 7, 1950000000, tf_payload([
                        ("map", "robot2/base_footprint", 1.0, 2.0, math.pi / 2)
                    ], stamp_sec=0)),
                    (9, 8, 2000000000, laser_scan_payload(stamp_sec=0)),
                ],
            )

        scan = extract_replay(self.bag, robot_name="robot2")["timeline"]["scans"][0]
        self.assertEqual(scan["coordinates"], "map")
        self.assertAlmostEqual(scan["points"][0], 1.0, places=4)
        self.assertAlmostEqual(scan["points"][1], 3.2, places=4)

    def test_merges_multiple_bags_in_recording_order(self):
        first = extract_replay(self.bag, robot_name="robot2")
        second = extract_replay(self.bag, robot_name="robot2")
        first["bag"] = "log_bag/robot2/backup/bags/first_terminal_bag"
        second["bag"] = "log_bag/robot2/backup/bags/second_terminal_bag"
        second["start_time_ns"] = first["start_time_ns"] + 10_000_000_000

        merged = merge_replays([second, first])

        self.assertEqual(merged["bags"], [first["bag"], second["bag"]])
        self.assertEqual(len(merged["segments"]), 2)
        self.assertAlmostEqual(merged["segments"][0]["start"], 0.0)
        next_start = round(first["duration"] + 0.000001, 6)
        self.assertEqual(merged["segments"][1]["start"], next_start)
        self.assertEqual(merged["duration"], round(next_start + second["duration"], 6))
        self.assertEqual(merged["message_count"], 12)
        self.assertEqual(merged["database_count"], 2)
        poses = merged["timeline"]["poses"]
        self.assertEqual([row["segment_index"] for row in poses], [0, 1])
        self.assertEqual([row["bag"] for row in poses], [first["bag"], second["bag"]])
        self.assertEqual(poses[1]["t"], next_start)
        odom_topic = next(topic for topic in merged["topics"] if topic["name"] == "/robot2/odom")
        self.assertEqual(odom_topic["count"], 2)
        images = merged["timeline"]["images"]
        self.assertEqual(len(images), 4)
        self.assertEqual([row["segment_index"] for row in images], [0, 0, 1, 1])
        self.assertTrue(all(row["bag"] in merged["bags"] for row in images))

        maps = merged["timeline"]["maps"]
        self.assertEqual(
            [row["current_map"] for row in maps],
            ["test_101", "test_102", "test_101", "test_102"],
        )
        self.assertEqual(maps[2]["t"], next_start)
        self.assertEqual(merged["map_names"], ["test_101", "test_102"])
        self.assertEqual(merged["initial_map_name"], "test_101")
        self.assertEqual(merged["map_name"], "test_101")

    def test_last_image_of_one_bag_precedes_first_image_of_next_bag(self):
        def replay(name, robot, start_ns, image_time):
            return {
                "bag": name,
                "robot_name": robot,
                "start_time_ns": start_ns,
                "duration": 1.0,
                "message_count": 1,
                "database_count": 1,
                "topics": [],
                "timeline": {
                    "images": [{"t": image_time, "camera": "front", "data_url": name}],
                },
                "warnings": [],
                "initial_map_name": robot,
                "map_name": robot,
            }

        first = replay("first", "robot1", 1, 1.0)
        second = replay("second", "robot2", 2, 0.0)
        merged = merge_replays([second, first])
        segments = merged["segments"]
        images = merged["timeline"]["images"]

        self.assertEqual([row["bag"] for row in images], ["first", "second"])
        self.assertEqual(images[0]["t"], segments[0]["end"])
        self.assertEqual(images[1]["t"], segments[1]["start"])
        self.assertAlmostEqual(images[1]["t"] - images[0]["t"], 0.000001, places=10)
        self.assertEqual(segments[1]["start"], 1.000001)
        self.assertEqual(merged["duration"], segments[1]["end"])

    def test_rejects_empty_multi_bag_merge(self):
        with self.assertRaisesRegex(BagReplayError, "至少需要"):
            merge_replays([])

    def test_source_has_no_ros_or_process_runtime_dependency(self):
        source = (BACKEND / "bag_replay.py").read_text(encoding="utf-8")
        self.assertIn("sqlite3.connect", source)
        self.assertIn("?mode=ro", source)
        self.assertNotIn("import rclpy", source)
        self.assertNotIn("subprocess", source)
        self.assertNotIn("ros2 ", source)

    def test_rejects_non_rosbag2_directory(self):
        empty = Path(self.tmp.name) / "empty"
        empty.mkdir()
        with self.assertRaisesRegex(BagReplayError, "rosbag2"):
            extract_replay(empty)


if __name__ == "__main__":
    unittest.main()
