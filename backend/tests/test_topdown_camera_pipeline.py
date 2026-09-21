import os
import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
BACKEND = ROOT / "backend"
sys.path.insert(0, str(BACKEND))
os.environ.setdefault("ROBOT_POSE_MODE", "none")

import gazebo_set_state_client as gazebo_client  # noqa: E402
import ros_command_queue  # noqa: E402
import ros_sensor_store as sensor_store  # noqa: E402
import server  # noqa: E402


class TopdownCameraStoreTest(unittest.TestCase):
    def setUp(self):
        sensor_store.clear_all()

    def tearDown(self):
        sensor_store.clear_all()

    def test_status_is_metadata_only_and_jpeg_is_returned_without_rgb_copy(self):
        rgb = bytes(range(24))
        jpeg = b"\xff\xd8small-jpeg\xff\xd9"
        sensor_store.set_topdown_image(
            {
                "available": True,
                "width": 4,
                "height": 2,
                "encoding": "jpeg",
                "rgb_bytes": rgb,
                "jpeg_bytes": jpeg,
                "jpeg_size": len(jpeg),
                "frame_id": "topdown_camera_link",
                "stamp_sec": 12,
                "stamp_nanosec": 34,
            }
        )

        status = sensor_store.get_topdown_image_status()
        self.assertTrue(status["available"])
        frame_seq = status["frame_seq"]
        self.assertGreater(frame_seq, 0)
        self.assertEqual(status["jpeg_size"], len(jpeg))
        self.assertNotIn("rgb_bytes", status)
        self.assertNotIn("jpeg_bytes", status)

        cached_jpeg, metadata = sensor_store.get_topdown_jpeg()
        self.assertIs(cached_jpeg, jpeg)
        self.assertEqual(metadata["frame_seq"], frame_seq)
        self.assertNotIn("rgb_bytes", metadata)
        self.assertNotIn("jpeg_bytes", metadata)

        legacy = sensor_store.get_topdown_image()
        self.assertIs(legacy["rgb_bytes"], rgb)
        legacy["width"] = 99
        self.assertEqual(sensor_store.get_topdown_image_status()["width"], 4)

    def test_frame_sequence_advances_for_each_received_frame(self):
        sensor_store.set_topdown_image({"jpeg_bytes": b"one", "jpeg_size": 3})
        first = sensor_store.get_topdown_image_status()["frame_seq"]
        sensor_store.set_topdown_image({"jpeg_bytes": b"two", "jpeg_size": 3})
        second = sensor_store.get_topdown_image_status()["frame_seq"]
        self.assertEqual(second, first + 1)


class GazeboFloorTextureTest(unittest.TestCase):
    def test_each_zone_uses_its_own_dense_text_texture(self):
        model_root = ROOT / "src" / "simulate" / "simulate" / "model"
        for zone_id in range(101, 105):
            label = f"test{zone_id}"
            material_dir = model_root / f"test_{zone_id}" / "materials"
            material = (
                material_dir / "scripts" / "floor_marker.material"
            ).read_text(encoding="utf-8")
            texture = material_dir / "textures" / f"floor_marker_{label}.png"

            self.assertIn(f"material OpenDelivery/Floor/test_{zone_id}", material)
            self.assertIn(f"texture floor_marker_{label}.png", material)
            self.assertIn("scale 0.2 0.2", material)
            self.assertNotIn("texture floor_marker.png", material)
            self.assertTrue(texture.is_file())
            self.assertFalse(
                (material_dir / "textures" / "floor_marker.png").exists()
            )


class GazeboLatestWinsQueueTest(unittest.TestCase):
    @staticmethod
    def _request(model_name):
        return SimpleNamespace(model_state=SimpleNamespace(model_name=model_name))

    def test_new_pose_supersedes_queued_pose_for_same_model(self):
        worker = gazebo_client._GazeboSetStateWorker()
        old_result = {}
        old_event = threading.Event()
        new_result = {}
        new_event = threading.Event()
        old_request = self._request("topdown_camera")
        new_request = self._request("topdown_camera")

        worker._enqueue_latest(old_request, old_result, old_event)
        worker._enqueue_latest(new_request, new_result, new_event)

        self.assertTrue(old_event.is_set())
        self.assertTrue(old_result["ok"])
        self.assertIn("superseded", old_result["err"])
        self.assertFalse(new_event.is_set())
        self.assertEqual(len(worker._pending), 1)
        self.assertIs(worker._pending["topdown_camera"][0], new_request)

    def test_different_models_keep_independent_latest_requests(self):
        worker = gazebo_client._GazeboSetStateWorker()
        first_event = threading.Event()
        second_event = threading.Event()
        worker._enqueue_latest(self._request("topdown_camera"), {}, first_event)
        worker._enqueue_latest(self._request("robot1"), {}, second_event)

        self.assertEqual(list(worker._pending), ["topdown_camera", "robot1"])
        self.assertFalse(first_event.is_set())
        self.assertFalse(second_event.is_set())

        worker.stop()
        self.assertTrue(first_event.is_set())
        self.assertTrue(second_event.is_set())
        self.assertFalse(worker._pending)


class TopdownCameraFastControlTest(unittest.TestCase):
    def test_world_plugin_is_built_installed_and_wired_to_the_bridge_topic(self):
        simulate_root = ROOT / "src" / "simulate" / "simulate"
        plugin = (simulate_root / "src" / "topdown_camera_control_plugin.cpp").read_text(
            encoding="utf-8"
        )
        cmake = (simulate_root / "CMakeLists.txt").read_text(encoding="utf-8")
        world = (simulate_root / "worlds" / "drawn_model.world").read_text(
            encoding="utf-8"
        )
        bridge = (ROOT / "backend" / "ros_tf_bridge.py").read_text(encoding="utf-8")

        self.assertIn("add_library(topdown_camera_control_plugin SHARED", cmake)
        self.assertIn("install(TARGETS ray_collision_filter_plugin topdown_camera_control_plugin", cmake)
        self.assertIn('filename="libtopdown_camera_control_plugin.so"', world)
        self.assertIn("/open_delivery/topdown_camera/pose", world)
        self.assertIn("/open_delivery/topdown_camera/pose", bridge)
        self.assertIn("world_->ModelByName(model_name_)", plugin)
        self.assertIn("model->SetWorldPose(world_pose", plugin)

    def test_topdown_pose_uses_bridge_command_topic(self):
        orientation = (-0.5, 0.5, 0.5, 0.5)
        with mock.patch.object(
            ros_command_queue,
            "enqueue_command_and_wait",
            return_value={"ok": True},
        ) as enqueue:
            result = server.try_publish_topdown_camera_pose(
                "topdown_camera", 1.5, -2.0, 24.0, "world", orientation
            )

        self.assertEqual(result["output"], "topdown_camera_pose_topic")
        command = enqueue.call_args.args[0]
        self.assertEqual(command["type"], "topdown_camera_pose")
        self.assertEqual(command["x"], 1.5)
        self.assertEqual(command["orientation"]["w"], 0.5)
        self.assertEqual(enqueue.call_args.kwargs["timeout"], 0.5)

    def test_fast_control_falls_back_when_plugin_is_not_ready(self):
        with mock.patch.object(
            ros_command_queue,
            "enqueue_command_and_wait",
            side_effect=RuntimeError("plugin not ready"),
        ):
            result = server.try_publish_topdown_camera_pose(
                "topdown_camera",
                0.0,
                0.0,
                32.0,
                "world",
                (-0.5, 0.5, 0.5, 0.5),
            )
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main()
