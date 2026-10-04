import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import navigation_processes
import server
import robot_lifecycle


class NavigationProcessesTest(unittest.TestCase):
    def test_robot_startup_cleanup_preserves_other_domain_and_other_robot(self):
        with tempfile.TemporaryDirectory() as directory:
            entries = []
            for pid, robot, domain in ((10001, 'robot2', '0'), (10002, 'robot2', '94'), (10003, 'robot20', '0')):
                path = Path(directory) / str(pid)
                path.mkdir()
                (path / 'cmdline').write_bytes(('python3\0navigation_task_node\0__ns:=/' + robot + '/navigation').encode())
                (path / 'environ').write_bytes(('ROS_DOMAIN_ID=' + domain).encode())
                entries.append(path)
            orchestrator = robot_lifecycle.RobotLifecycleOrchestrator.__new__(robot_lifecycle.RobotLifecycleOrchestrator)
            orchestrator._ensure_robot = mock.Mock(return_value='robot2')
            with mock.patch.object(Path, 'iterdir', return_value=iter(entries)), \
                    mock.patch.dict(os.environ, {'ROS_DOMAIN_ID': '0'}), \
                    mock.patch.object(os, 'kill') as kill, mock.patch.object(time, 'sleep'):
                orchestrator._terminate_stale_robot_processes('robot2')
            self.assertEqual([call.args[0] for call in kill.call_args_list], [10001, 10001])

    def test_matching_requires_actual_navigation_argv_robot_and_ros_domain(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cases = [
                (101, ['/opt/ros/foxy/lib/nav2_controller/controller_server', '--ros-args', '-r', '__ns:=/robot2/navigation'], '0'),
                (102, ['/opt/ros/foxy/lib/nav2_controller/controller_server', '--ros-args', '-r', '__ns:=/robot20/navigation'], '0'),
                (103, ['/opt/ros/foxy/lib/nav2_controller/controller_server', '--ros-args', '-r', '__ns:=/robot2/navigation'], '94'),
                (104, ['python3', '/workspace/install/navigation_tasks/lib/navigation_tasks/navigation_task_node', '--ros-args', '-r', '__ns:=/robot2/navigation'], '0'),
                (105, ['python3', '/opt/ros/foxy/bin/ros2', 'launch', 'nav_bringup', 'stack.launch.py', 'robot_name:=robot2'], '0'),
                (106, ['bash', '-c', 'ros2 launch nav_bringup stack.launch.py robot_name:=robot2'], '0'),
                (107, ['gzserver', '__ns:=/robot2/navigation'], '0'),
            ]
            for pid, argv, domain in cases:
                path = root / str(pid)
                path.mkdir()
                (path / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv))
                (path / 'environ').write_bytes(('ROS_DOMAIN_ID=' + domain).encode())
                (path / 'stat').write_text(str(pid) + ' (test) S ' + ' '.join(['0'] * 18 + ['123']))
            self.assertEqual(set(navigation_processes.navigation_processes('robot2', '0', root)), {101, 104, 105})
            self.assertEqual(set(navigation_processes.navigation_processes('robot2', '94', root)), {103})

    def test_navigation_stop_uses_scoped_cleanup_instead_of_shell_pkill(self):
        manager = server.RosNodeManager.__new__(server.RosNodeManager)
        manager._lock = threading.Lock()
        manager._procs = {}
        with mock.patch.object(navigation_processes, 'stop_navigation', return_value=[101]) as stop, \
                mock.patch.object(manager, '_run_shell') as shell:
            manager._stop_node({'id': 'navigation_robot2', 'stop_cmd': 'pkill -f unsafe-pattern'})
        stop.assert_called_once_with('robot2')
        shell.assert_not_called()

    def test_stuck_orphan_exits_and_other_ros_domain_survives(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / 'navigation_tasks'
            root.mkdir()
            script = root / 'navigation_task_node'
            script.write_text('import signal,sys,time\n'
                              'signal.signal(signal.SIGTERM,signal.SIG_IGN)\n'
                              'open(sys.argv[1],"w").close()\n'
                              'while True: time.sleep(.1)\n')
            processes = []
            try:
                for domain in ('187', '188'):
                    ready = root / domain
                    processes.append(subprocess.Popen(
                        [sys.executable, str(script), str(ready), '--ros-args', '-r', '__ns:=/robot2/navigation'],
                        env={**os.environ, 'ROS_DOMAIN_ID': domain}))
                    deadline = time.monotonic() + 3
                    while not ready.exists() and time.monotonic() < deadline:
                        time.sleep(.02)
                    self.assertTrue(ready.exists())
                matched = navigation_processes.stop_navigation('robot2', '187')
                self.assertIn(processes[0].pid, matched)
                processes[0].wait(timeout=2)
                self.assertIsNone(processes[1].poll())
            finally:
                for process in processes:
                    if process.poll() is None:
                        process.kill()
                    process.wait(timeout=2)
