"""Run with ROS sourced: python3 test_recording_integration.py <recorder binary>.

Uses an isolated ROS domain and temporary storage, never the live robot.
"""
import os
os.environ['ROS_DOMAIN_ID'] = '187'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['ROS_LOG_DIR'] = '/tmp/opendelivery-recorder-test-ros'
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
from sensor_msgs.msg import LaserScan
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
    tf_pub = node.create_publisher(TFMessage, '/tf_static', latch)
    task = TaskStatus(task_id='integration-task', task_status='running')
    task_pub.publish(task)
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
                    if scan_pub is not None:
                        scan_pub.publish(scan)
                    rclpy.spin_once(node, timeout_sec=0.01)
                    time.sleep(0.02)
            try:
                pump(3, 'initializing')
                pump(2, 'localizing')
                assert not list(root.rglob('*.db3')), 'Recorded before startup completed'
                pump(10, 'localization_lost')
                assert list(root.rglob('*.db3')), 'No recording in localization_lost'
                scan_pub = node.create_publisher(LaserScan, '/test_robot/scan_2d', QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
                pump(4, 'ready')
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
        for db in bags:
            with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as conn:
                assert conn.execute('pragma quick_check').fetchone()[0] == 'ok'
                counts = dict(conn.execute('select t.name,count(m.id) from topics t left join messages m on t.id=m.topic_id group by t.id'))
            for topic in ('/test_robot/robot_status', '/test_robot/odom', '/tf_static'):
                assert counts.get(topic, 0) > 0, (db, counts)
            replay = bag_replay.extract_replay(db.parent, 'test_robot')
            assert replay['timeline']['statuses'] and replay['timeline']['poses'], db
            assert not replay['warnings'], replay['warnings']
            scans += len(replay['timeline']['scans'])
        assert scans > 0, 'Late best-effort scan topic was not recorded'
        print(f'PASS: startup gate, {len(bags)} bags, persistent subscriptions, retained TF, shutdown, replay')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    run(pathlib.Path(sys.argv[1]).resolve())
