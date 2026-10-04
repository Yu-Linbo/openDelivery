"""Real Nav2 closed-loop elevator check on project maps, isolated ROS domain 94.

Uses the existing robot2 identity in an isolated kinematic simulation; no robot
is registered, no hardware is connected, and no production commands are sent.
Checks the padded full footprint against occupied map cells on every tick.
"""
import json
import math
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
from PIL import Image
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from custom_msgs_srvs.msg import TaskInfo, TaskStatus
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import yaml

ROOT = Path(__file__).resolve().parents[4]
OUT = Path('/tmp/od-elevator-check')
ROBOT = 'robot2'


class ElevatorProbe(Node):
    def __init__(self):
        super().__init__('elevator_closed_loop_probe')
        self.v = self.w = 0.0
        self.last = time.monotonic()
        self.statuses = []
        self.trajectory = []
        self.last_command = self.last
        self.probe_executor = SingleThreadedExecutor(context=self.context)
        self.probe_executor.add_node(self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, f'/{ROBOT}/navagtion/cmd_vel', self.command, 10)
        self.create_subscription(TaskStatus, f'/{ROBOT}/navigation/task_status', self.status, qos)
        self.tasks = self.create_publisher(TaskInfo, f'/{ROBOT}/navigation/task_info', 10)
        self.maps = self.create_publisher(OccupancyGrid, f'/{ROBOT}/map', qos)
        self.odom = self.create_publisher(Odometry, f'/{ROBOT}/odom', 10)
        self.scan = self.create_publisher(LaserScan, f'/{ROBOT}/scan_2d', 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        static = TransformStamped()
        static.header.frame_id = 'map'
        static.child_frame_id = f'{ROBOT}/odom'
        static.transform.rotation.w = 1.0
        self.static_tf.sendTransform(static)
        config = (ROOT / 'src/navigation/nav_bringup/config/nav2_params.yaml').read_text()
        # Read the production footprint without interpreting launch placeholders.
        line = next(line.strip() for line in config.splitlines() if line.strip().startswith('footprint:'))
        self.footprint = np.array(json.loads(yaml.safe_load(line)['footprint']))
        padding_line = next(line.strip() for line in config.splitlines() if line.strip().startswith('footprint_padding:'))
        self.padding = yaml.safe_load(padding_line)['footprint_padding']
        minimum = self.footprint.min(axis=0) - self.padding
        maximum = self.footprint.max(axis=0) + self.padding
        self.center = (minimum + maximum) / 2
        self.half = (maximum - minimum) / 2
        self.load_floor('test_101')
        self.create_timer(0.033, self.tick)
        self.create_timer(0.10, self.scan_tick)
        self.create_timer(1.0, self.map_tick)

    def load_floor(self, floor):
        self.floor = floor
        root = ROOT / 'map' / floor
        meta = yaml.safe_load((root / f'{floor}.yaml').read_text())
        image = np.flipud(np.asarray(Image.open(root / meta['image'])))
        self.resolution = float(meta['resolution'])
        self.origin = np.array(meta['origin'][:2])
        self.occupied = image < (1 - float(meta['occupied_thresh'])) * 255
        self.height, self.width = image.shape
        yy, xx = np.where(self.occupied)
        self.cells = np.column_stack(((xx + .5) * self.resolution + self.origin[0],
                                     (yy + .5) * self.resolution + self.origin[1]))
        points = json.loads((root / f'{floor}_points.json').read_text())['points']
        self.inside = next(point for point in points if point['type'] == 'elevator_inside')
        self.waiting = next(point for point in points if point['type'] == 'elevator_waiting')
        self.x, self.y, self.yaw = (self.waiting[key] for key in ('x', 'y', 'yaw'))
        self.v = self.w = 0.0
        self.last = time.monotonic()
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width, grid.info.height = self.width, self.height
        grid.info.resolution = self.resolution
        grid.info.origin.position.x, grid.info.origin.position.y = self.origin
        grid.info.origin.orientation.w = 1.0
        grid.data = np.where(self.occupied, 100, np.where(image > 205, 0, -1)).astype(np.int8).ravel().tolist()
        self.grid = grid
        self.map_tick()

    def command(self, message):
        self.v, self.w = message.linear.x, message.angular.z
        self.last_command = time.monotonic()

    def status(self, message):
        self.statuses.append(message)
        if message.task_status in ('Finished', 'Failed', 'Terminated'):
            print('STATUS', message.task_id, message.task_status, message.message, flush=True)

    def collision(self, x, y, yaw):
        # Separating-axis test: full oriented padded rectangle versus every
        # nearby solid map cell. Checks overlap of interiors, not only edges.
        cosine, sine = math.cos(yaw), math.sin(yaw)
        center = np.array([x, y]) + np.array([cosine*self.center[0]-sine*self.center[1],
                                             sine*self.center[0]+cosine*self.center[1]])
        nearby = self.cells[np.max(np.abs(self.cells-center), axis=1) < .6]
        if not len(nearby):
            return False
        delta = nearby-center
        half_cell = self.resolution/2
        hx, hy = self.half
        projections = np.column_stack((np.abs(delta[:,0]), np.abs(delta[:,1]),
                                       np.abs(delta[:,0]*cosine+delta[:,1]*sine),
                                       np.abs(-delta[:,0]*sine+delta[:,1]*cosine)))
        c, s = abs(cosine), abs(sine)
        bounds = np.array([hx*c+hy*s+half_cell, hx*s+hy*c+half_cell,
                           hx+half_cell*(c+s), hy+half_cell*(c+s)])
        return bool(np.any(np.all(projections <= bounds, axis=1)))

    def tick(self):
        now = time.monotonic()
        dt = min(.1, now-self.last)
        self.last = now
        if now-self.last_command > .5:
            self.v = self.w = 0.0
        x = self.x+self.v*math.cos(self.yaw)*dt
        y = self.y+self.v*math.sin(self.yaw)*dt
        yaw = self.yaw+self.w*dt
        if self.collision(x, y, yaw):
            raise RuntimeError(f'Full footprint collision: {self.floor} pose={(x,y,yaw)}')
        self.x, self.y, self.yaw = x, y, yaw
        self.trajectory.append([self.floor, x, y, yaw])
        stamp = self.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = f'{ROBOT}/odom'
        transform.child_frame_id = f'{ROBOT}/base_footprint'
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.tf.sendTransform(transform)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = f'{ROBOT}/odom'
        odom.child_frame_id = f'{ROBOT}/base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.v, self.w
        self.odom.publish(odom)

    def map_tick(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.maps.publish(self.grid)
        (OUT/'state.json').write_text(json.dumps({'floor': self.floor, 'pose': [self.x,self.y,self.yaw],
                                                'velocity': [self.v,self.w]}))

    def scan_tick(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f'{ROBOT}/base_footprint'
        scan.angle_min, scan.angle_max = -math.pi, math.pi
        scan.angle_increment = 2*math.pi/180
        scan.range_min, scan.range_max = .05, 5.0
        angles = self.yaw + np.linspace(-math.pi, math.pi, 181)
        distances = np.arange(.025, 5.001, .025)
        x = self.x + np.cos(angles)[:,None]*distances
        y = self.y + np.sin(angles)[:,None]*distances
        ix = np.floor((x-self.origin[0])/self.resolution).astype(int)
        iy = np.floor((y-self.origin[1])/self.resolution).astype(int)
        valid = (ix>=0)&(ix<self.width)&(iy>=0)&(iy<self.height)
        hits = ~valid | self.occupied[np.clip(iy,0,self.height-1), np.clip(ix,0,self.width-1)]
        scan.ranges = np.min(np.where(hits, distances, 5.0), axis=1).astype(np.float32).tolist()
        self.scan.publish(scan)

    def wait(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.probe_executor.spin_once(timeout_sec=.03)

    def run_task(self, direction, point, timeout=80):
        task = TaskInfo()
        task.task_id = f'{self.floor}-{direction}'
        task.task_type, task.end_action = 'navigation', 'waiting'
        pose = Pose()
        pose.position.x, pose.position.y = point['x'], point['y']
        pose.orientation.z, pose.orientation.w = math.sin(point['yaw']/2), math.cos(point['yaw']/2)
        task.poses = [pose]
        self.tasks.publish(task)
        started = time.monotonic()
        while time.monotonic()-started < timeout:
            self.probe_executor.spin_once(timeout_sec=.03)
            terminal = [s for s in self.statuses if s.task_id==task.task_id and s.task_status in ('Finished','Failed','Terminated')]
            if terminal:
                xy = math.hypot(self.x-point['x'], self.y-point['y'])
                yaw = abs(math.atan2(math.sin(self.yaw-point['yaw']), math.cos(self.yaw-point['yaw'])))
                result = {'floor': self.floor, 'direction': direction, 'status': terminal[-1].task_status,
                          'xy_error': xy, 'yaw_error': yaw, 'elapsed': time.monotonic()-started,
                          'message': terminal[-1].message}
                print('RESULT', json.dumps(result), flush=True)
                assert result['status']=='Finished' and xy<=.101 and yaw<=.151, result
                return result
        raise RuntimeError(f'Elevator task deadline: {task.task_id} pose={(self.x,self.y,self.yaw)}')


def main():
    OUT.mkdir(exist_ok=True)
    (OUT/'report.json').unlink(missing_ok=True)
    os.environ.update(ROS_DOMAIN_ID='94', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(OUT/'ros'))
    os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    rclpy.init()
    simulation = ElevatorProbe()
    with (OUT/'nav2.log').open('w') as log:
        process = subprocess.Popen(['ros2','launch','nav_bringup','stack.launch.py',f'robot_name:={ROBOT}',
                                    'use_sim_time:=false'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        results = []
        try:
            simulation.wait(18)
            for floor in ('test_101','test_102','test_103','test_104'):
                if floor != simulation.floor:
                    simulation.load_floor(floor)
                    simulation.wait(3)
                results.append(simulation.run_task('entry', simulation.inside))
                results.append(simulation.run_task('exit', simulation.waiting))
                (OUT/'report.json').write_text(json.dumps(results, indent=2))
            print('ALL PASSED: eight elevator tasks, zero padded footprint collisions', flush=True)
        finally:
            if process.poll() is None:
                # Let launch send each child SIGINT once instead of delivering
                # both a process-group signal and launch's forwarded signal.
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            (OUT/'trajectory.json').write_text(json.dumps(simulation.trajectory))
            simulation.probe_executor.shutdown()
            simulation.probe_executor.remove_node(simulation)
            simulation.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
