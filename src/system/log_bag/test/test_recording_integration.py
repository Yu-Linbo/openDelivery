"""Run with ROS sourced: python3 test_recording_integration.py <recorder binary>.

Uses an isolated ROS domain and temporary storage, never the live robot.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '187'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/opendelivery-recorder-test-ros'
import json
import pathlib
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from custom_msgs_srvs.msg import RobotStatus, TaskStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


def run(binary):
    os.environ['OPEN_DELIVERY_ROOT'] = str(pathlib.Path(__file__).resolve().parents[4])
    rclpy.init()
    node = rclpy.create_node('recorder_integration_publisher')
    status_pub = node.create_publisher(RobotStatus, '/test_robot/robot_status', 10)
    odom_pub = node.create_publisher(Odometry, '/test_robot/odom', 10)
    latch = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
    task_pub = node.create_publisher(TaskStatus, '/test_robot/task_status', latch)
    camera_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    front_pub = node.create_publisher(Image, '/test_robot/front_camera/image_raw', camera_qos)
    down_pub = node.create_publisher(Image, '/test_robot/front_down_camera/image_raw', camera_qos)
    frame = Image(height=2, width=2, encoding='rgb8', step=6,
                  data=[255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255])
    frame.header.frame_id = 'test_robot/camera'
    camera_enabled = True
    tf_pub = node.create_publisher(TFMessage, '/tf_static', latch)
    transform = TransformStamped(child_frame_id='test_robot/base_link')
    transform.header.frame_id = 'map'
    transform.transform.rotation.w = 1.0
    tf_pub.publish(TFMessage(transforms=[transform]))
    status = RobotStatus(robot_name='test_robot', robot_status='initializing', current_map='test_map')
    odom = Odometry(child_frame_id='test_robot/base_link')
    odom.header.frame_id = 'map'
    odom.pose.pose.orientation.w = 1.0
    scan_pub = None
    scan = LaserScan(angle_min=0.0, angle_max=1.0, angle_increment=0.5, range_min=0.1, range_max=10.0, ranges=[1.0, 2.0, 3.0])
    scan.header.frame_id = 'test_robot/base_link'

    with tempfile.TemporaryDirectory(prefix='recorder-integration-') as tmp:
        root = pathlib.Path(tmp)
        with (root/'process.log').open('w') as output:
            process = subprocess.Popen([str(binary), '--robot-name', 'test_robot', '--root', tmp,
                                        '--max-bag-bytes', '65536'], stdout=output, stderr=output)
            def pump(seconds, state):
                status.robot_status = state
                until = time.monotonic() + seconds
                while time.monotonic() < until:
                    assert process.poll() is None, (root/'process.log').read_text()
                    status_pub.publish(status)
                    odom.pose.pose.position.x += 0.01
                    odom_pub.publish(odom)
                    if camera_enabled:
                        front_pub.publish(frame)
                        down_pub.publish(frame)
                    if scan_pub is not None:
                        scan_pub.publish(scan)
                    rclpy.spin_once(node, timeout_sec=0.01)
                    time.sleep(0.02)
            def wait_for_next_bag():
                active_dir = root/'test_robot'
                current = list(active_dir.glob('*_terminal_bag'))
                assert len(current) == 1, current
                previous = current[0]
                deadline = time.monotonic() + 12
                while time.monotonic() < deadline:
                    pump(0.5, 'ready')
                    current = list(active_dir.glob('*_terminal_bag'))
                    if len(current) == 1 and current[0] != previous:
                        return
                raise AssertionError(f'Bag did not rotate after {previous}')

            try:
                pump(3, 'initializing')
                pump(2, 'localizing')
                assert not list(root.rglob('*.db3')), 'Recorded before startup completed'
                pump(10, 'localization_lost')
                assert list(root.rglob('*.db3')), 'No recording in localization_lost'
                # An orphan terminal must not tag this bag or capture a frame.
                task_pub.publish(TaskStatus(task_id='orphan-old', task_status='Finished'))
                pump(0.4, 'ready')
                # A live camera stream alone must not write any image messages.
                task_pub.publish(TaskStatus(task_id='task-first', task_status='Waiting'))
                pump(0.6, 'ready')
                task_pub.publish(TaskStatus(task_id='task-first', task_status='Waiting'))
                pump(0.3, 'ready')
                # A new ID implicitly ends the first task. One snapshot serves
                # that end and the new task's start.
                task_pub.publish(TaskStatus(task_id='task-second', task_status='Waiting'))
                pump(4, 'ready')
                task_pub.publish(TaskStatus(task_id='task-second', task_status='Finished'))
                pump(0.5, 'ready')
                task_pub.publish(TaskStatus(task_id='task-second', task_status='Finished'))
                pump(0.5, 'ready')
                # An old cached frame must not be used when cameras stop.
                camera_enabled = False
                pump(2.5, 'ready')
                task_pub.publish(TaskStatus(task_id='task-third', task_status='Waiting'))
                pump(0.5, 'ready')
                camera_enabled = True
                scan_pub = node.create_publisher(LaserScan, '/test_robot/scan_2d', camera_qos)
                pump(4, 'ready')
                # An idle state from a restarted manager ends the old task.
                task_pub.publish(TaskStatus(task_id='', task_status='Waiting'))
                pump(0.5, 'ready')
                wait_for_next_bag()
                task_pub.publish(TaskStatus(task_id='after-idle', task_status='Waiting'))
                pump(0.5, 'ready')
                # The manager can replace an active task with a rejected TaskInfo;
                # it then publishes only Failed for the new ID.
                task_pub.publish(TaskStatus(task_id='rejected-task', task_status='Failed'))
                pump(0.5, 'ready')
                wait_for_next_bag()
                task_pub.publish(TaskStatus(task_id='after-failed', task_status='Waiting'))
                pump(0.5, 'ready')
                # A new manager has a different publisher GID. Subsequent bags
                # must not replay the previous publisher's latched task status.
                node.destroy_publisher(task_pub)
                task_pub = node.create_publisher(TaskStatus, '/test_robot/task_status', latch)
                task_pub.publish(TaskStatus(task_id='', task_status='Waiting'))
                pump(0.5, 'ready')
                wait_for_next_bag()
                task_pub.publish(TaskStatus(task_id='after-manager-restart', task_status='Waiting'))
                pump(0.5, 'ready')
                # Previously completed task IDs may be accepted again. Waiting
                # starts the new generation even while another ID is active.
                task_pub.publish(TaskStatus(task_id='task-second', task_status='Waiting'))
                pump(0.5, 'ready')
                task_pub.publish(TaskStatus(task_id='task-second', task_status='Finished'))
                pump(0.5, 'ready')
                # A later localization transition must not stop ongoing capture.
                pump(2, 'localizing')
                pump(1, 'shutdown')
                bags_before = len(list(root.rglob('*.db3')))
                pump(1, 'shutdown')
                assert len(list(root.rglob('*.db3'))) == bags_before, 'Restarted after shutdown'
                pump(2, 'ready')  # Recorder restarted while already ready is allowed.
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            assert process.returncode == 0, (root/'process.log').read_text()
        bags = list(root.rglob('*.db3'))
        assert len(bags) >= 3, f'Expected rotations, got {len(bags)} bags'
        sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4]/'backend'))
        import bag_replay
        scans = 0
        image_counts = {'/test_robot/front_camera/image_raw': 0,
                        '/test_robot/front_down_camera/image_raw': 0}
        for db in bags:
            with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
                assert conn.execute('pragma quick_check').fetchone()[0] == 'ok'
                counts = dict(conn.execute('select t.name,count(m.id) from topics t left join messages m on t.id=m.topic_id group by t.id'))
            for topic in ('/test_robot/robot_status', '/test_robot/odom', '/tf_static'):
                assert counts.get(topic, 0) > 0, (db, counts)
            for topic in image_counts:
                image_counts[topic] += counts.get(topic, 0)
            replay = bag_replay.extract_replay(db.parent, 'test_robot')
            assert replay['timeline']['statuses'] and replay['timeline']['poses'], db
            assert not replay['warnings'], replay['warnings']
            scans += len(replay['timeline']['scans'])
        assert scans > 0, 'Late best-effort scan topic was not recorded'
        assert image_counts == dict.fromkeys(image_counts, 11), image_counts
        index = json.loads((root/'test_robot/backup/match.json').read_text())
        indexed = index['bags']
        assert any(meta.get('tags') == ['task-second'] for meta in indexed.values()), index
        assert any(meta.get('tags') == ['after-idle'] for meta in indexed.values()), index
        assert any(meta.get('tags') == ['after-failed'] for meta in indexed.values()), index
        assert all('orphan-old' not in meta.get('tags', []) and
                   'rejected-task' not in meta.get('tags', []) for meta in indexed.values()), index
        restarted = [(path, meta) for path, meta in indexed.items()
                     if 'after-manager-restart' in meta.get('tags', [])]
        assert restarted, index
        for path, meta in restarted:
            assert 'after-failed' not in meta['tags'], meta
            replay = bag_replay.extract_replay(pathlib.Path(path), 'test_robot')
            task_ids = {task.get('task_id') for task in replay['timeline']['tasks']}
            assert 'after-failed' not in task_ids, (path, task_ids)
        print(f'PASS: startup gate, {len(bags)} bags, boundary frames, task generations, GID replay')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run(pathlib.Path(sys.argv[1]).resolve())
