import os
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


BACKEND_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_DIR))
os.environ.setdefault("ROBOT_POSE_MODE", "mock")

import ros_tf_bridge  # noqa: E402


class RosTfClockResetTest(unittest.TestCase):
    @staticmethod
    def _clock(sec: int, nanosec: int = 0):
        return SimpleNamespace(clock=SimpleNamespace(sec=sec, nanosec=nanosec))

    @staticmethod
    def _bridge(previous_ns=None, tolerance_ns=500_000_000):
        logger = mock.Mock()
        return SimpleNamespace(
            _last_sim_clock_ns=previous_ns,
            _clock_reset_tolerance_ns=tolerance_ns,
            _buffer=mock.Mock(),
            get_logger=mock.Mock(return_value=logger),
        )

    def test_forward_clock_does_not_clear_tf(self):
        bridge = self._bridge(previous_ns=4_000_000_000)
        ros_tf_bridge.OpenDeliveryTfBridgeNode._on_clock(bridge, self._clock(5))
        bridge._buffer.clear.assert_not_called()
        self.assertEqual(bridge._last_sim_clock_ns, 5_000_000_000)

    def test_small_out_of_order_clock_does_not_clear_tf(self):
        bridge = self._bridge(previous_ns=5_000_000_000)
        ros_tf_bridge.OpenDeliveryTfBridgeNode._on_clock(
            bridge, self._clock(4, 750_000_000)
        )
        bridge._buffer.clear.assert_not_called()

    def test_simulation_clock_reset_clears_tf_and_gazebo_cache(self):
        bridge = self._bridge(previous_ns=30_000_000_000)
        with mock.patch("ros_sensor_store.clear_gazebo_models") as clear_models:
            ros_tf_bridge.OpenDeliveryTfBridgeNode._on_clock(
                bridge, self._clock(1)
            )
        bridge._buffer.clear.assert_called_once_with()
        clear_models.assert_called_once_with()
        bridge.get_logger.return_value.warning.assert_called_once()
        self.assertEqual(bridge._last_sim_clock_ns, 1_000_000_000)

    def test_static_tf_uses_static_buffer_api(self):
        bridge = self._bridge()
        transforms = [object(), object()]
        message = SimpleNamespace(transforms=transforms)
        ros_tf_bridge.OpenDeliveryTfBridgeNode._on_tf_static_message(bridge, message)
        self.assertEqual(bridge._buffer.set_transform_static.call_count, 2)
        bridge._buffer.set_transform.assert_not_called()


if __name__ == "__main__":
    unittest.main()
