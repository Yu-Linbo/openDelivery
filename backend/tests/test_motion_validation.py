import sys
import os
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_motion_api as motion
os.environ.setdefault('ROBOT_POSE_MODE', 'none')
import server


class MotionValidationTest(unittest.TestCase):
    def test_accepted_navigation_helper_finishes_without_missing_imports(self):
        import shlex
        with mock.patch.object(motion, '_ros_run', return_value={'ok': True}) as run:
            motion.send_navigate_to_pose('robot1', 1, 2, 0)
        script = shlex.split(run.call_args[0][0])[2]
        rclpy = mock.Mock()
        node = rclpy.create_node.return_value
        node.create_client.return_value.call_async.return_value.result.return_value.current_state.label = 'active'
        rclpy.ok.return_value = False
        action = mock.Mock()
        action.ActionClient.return_value.send_goal_async.return_value.result.return_value.accepted = True
        modules = {'rclpy': rclpy, 'rclpy.action': action,
                   'nav2_msgs.action': mock.Mock(), 'lifecycle_msgs.srv': mock.Mock()}
        with mock.patch.dict(sys.modules, modules):
            exec(script, {})
        action.ActionClient.return_value.destroy.assert_called_once()
        node.destroy_node.assert_called_once()

    def test_invalid_motion_bodies_return_http_400(self):
        handler = object.__new__(server.ApiHandler)
        payload = {'robot_id': 'robot1', 'name': 'home', 'confirmed': True,
                   'session_id': 'audit', 'sequence': 1, 'active': True,
                   'linear': None, 'x': None, 'y': 0}
        handler._read_json_body = lambda: payload
        for route in ('teleop', 'velocity'):
            handler.path = '/api/robot/motion/' + route
            handler._send_json = mock.Mock()
            handler.do_POST()
            self.assertEqual(handler._send_json.call_args[0][1], 400)
        handler.path = '/api/robot/waypoints/record'
        handler._send_json = mock.Mock()
        handler.do_POST()
        self.assertEqual(handler._send_json.call_args[0][1], 400)

    def test_non_finite_velocities_never_reach_ros(self):
        with mock.patch.object(motion, '_ros_run') as run, mock.patch.object(motion, '_send_teleop_command') as send:
            for value in (float('nan'), float('inf'), -float('inf'), None):
                for linear, angular, seconds in ((value, 0, 1), (0, value, 1), (0, 0, value)):
                    with self.subTest(linear=linear, angular=angular, seconds=seconds), self.assertRaises(ValueError):
                        motion.publish_cmd_vel_timed('robot1', linear, angular, seconds, confirmed=True)
                with self.assertRaises(ValueError):
                    motion.set_teleop_velocity('robot1', value, 0, active=True, confirmed=True, session_id='audit', sequence=1)
            run.assert_not_called()
            send.assert_not_called()

    def test_invalid_navigation_pose_never_reaches_ros(self):
        with mock.patch.object(motion, '_ros_run') as run:
            for pose in ((float('nan'), 0, 0), (0, float('inf'), 0), (0, 0, None)):
                with self.subTest(pose=pose), self.assertRaises(ValueError):
                    motion.send_navigate_to_pose('robot1', *pose)
            run.assert_not_called()

    def test_invalid_waypoint_is_not_persisted(self):
        with mock.patch.object(motion, '_save_waypoints') as save:
            with self.assertRaises(ValueError):
                motion.record_waypoint('../robot', 'home', 0, 0)
            with self.assertRaises(ValueError):
                motion.record_waypoint('robot1', 'home', float('nan'), 0)
            save.assert_not_called()

    def test_read_only_shell_operators_are_passed_as_literal_arguments(self):
        import shlex
        command = 'ros2 node list ; touch /tmp/should-not-exist $(id)'
        with mock.patch.object(motion, '_ros_run', return_value={'ok': True}) as run:
            motion.ros2_read_only(command)
        safe = run.call_args[0][0]
        self.assertEqual(shlex.split(safe), shlex.split(command))
        self.assertIn("';'", safe)
        self.assertIn("'$(id)'", safe)
        self.assertNotIn(' ; ', safe)


if __name__ == '__main__':
    unittest.main()
