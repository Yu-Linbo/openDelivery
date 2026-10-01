"""Opt-in real Nav2 check with a kinematic robot in isolated ROS domain 93.

Requires a sourced ROS/colcon workspace. No hardware or Gazebo is involved.
Results and stack logs are written under /tmp/od-navigation-check.
"""
import json, math, os, signal, subprocess, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Pose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from custom_msgs_srvs.msg import TaskInfo, TaskStatus
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

OUT=Path('/tmp/od-navigation-check'); OUT.mkdir(exist_ok=True)
class Simulation(Node):
    def __init__(self):
        super().__init__('closed_loop_probe')
        self.x=self.y=self.yaw=0.0; self.v=self.w=0.0; self.frozen=False
        self.last=time.monotonic(); self.statuses=[]; self.trajectory=[]; self.plans={}
        self.create_subscription(Twist,'/navcheck/navagtion/cmd_vel',self.command,10)
        for topic in ('/navcheck/navigation/plan','/navcheck/navigation/transformed_global_plan'):
            self.create_subscription(NavPath,topic,lambda m,topic=topic:self.plans.update({topic:[[p.pose.position.x,p.pose.position.y] for p in m.poses]}),10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TaskStatus,'/navcheck/navigation/task_status',self.status,qos)
        self.tasks=self.create_publisher(TaskInfo,'/navcheck/navigation/task_info',10)
        self.maps=self.create_publisher(OccupancyGrid,'/navcheck/map',qos)
        self.odom=self.create_publisher(Odometry,'/navcheck/odom',10)
        self.scan=self.create_publisher(LaserScan,'/navcheck/scan_2d',10)
        self.tf=TransformBroadcaster(self); self.static_tf=StaticTransformBroadcaster(self)
        static=TransformStamped(); static.header.frame_id='map'; static.child_frame_id='navcheck/odom'; static.transform.rotation.w=1.0
        self.static_tf.sendTransform(static)
        grid=OccupancyGrid(); grid.header.frame_id='map'; grid.info.resolution=0.05; grid.info.width=160; grid.info.height=160
        grid.info.origin.position.x=grid.info.origin.position.y=-3.0; grid.info.origin.orientation.w=1.0
        grid.data=[0]*(160*160)
        for iy in range(160):
            for ix in range(160):
                x=-3+(ix+.5)*.05; y=-3+(iy+.5)*.05
                if 1.0 <= x <= 1.2 and -.35 <= y <= .35: grid.data[iy*160+ix]=100
        self.grid=grid; self.create_timer(1.,self.map_tick); self.create_timer(.033,self.tick); self.create_timer(.10,self.scan_tick)
        self.map_tick()
    def status(self,msg):
        self.statuses.append(msg)
        print('STATUS',msg.task_id,msg.task_status,msg.message,flush=True)
    def command(self,msg): self.v=msg.linear.x; self.w=msg.angular.z
    def map_tick(self):
        self.grid.header.stamp=self.get_clock().now().to_msg(); self.maps.publish(self.grid)
        (OUT/'state.json').write_text(json.dumps({'x':self.x,'y':self.y,'yaw':self.yaw,'v':self.v,'w':self.w,'plans':self.plans}))
    def collision(self,x,y): return math.hypot(max(1.0-x,0.,x-1.2),max(-.35-y,0.,y-.35)) < .28
    def tick(self):
        now=time.monotonic(); dt=min(.1,now-self.last); self.last=now
        if not self.frozen:
            x=self.x+self.v*math.cos(self.yaw)*dt; y=self.y+self.v*math.sin(self.yaw)*dt
            if self.collision(x,y): raise RuntimeError('Robot collision in closed-loop check')
            self.x,self.y=x,y; self.yaw+=self.w*dt
        self.trajectory.append([self.x,self.y,self.yaw])
        stamp=self.get_clock().now().to_msg()
        tf=TransformStamped(); tf.header.stamp=stamp; tf.header.frame_id='navcheck/odom'; tf.child_frame_id='navcheck/base_footprint'
        tf.transform.translation.x=self.x; tf.transform.translation.y=self.y; tf.transform.rotation.z=math.sin(self.yaw/2); tf.transform.rotation.w=math.cos(self.yaw/2)
        self.tf.sendTransform(tf)
        odom=Odometry(); odom.header.stamp=stamp; odom.header.frame_id='navcheck/odom'; odom.child_frame_id='navcheck/base_footprint'
        odom.pose.pose.position.x=self.x; odom.pose.pose.position.y=self.y; odom.pose.pose.orientation=tf.transform.rotation
        odom.twist.twist.linear.x=self.v if not self.frozen else 0.; odom.twist.twist.angular.z=self.w if not self.frozen else 0.
        self.odom.publish(odom)
    def scan_tick(self):
        msg=LaserScan(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id='navcheck/base_footprint'
        msg.angle_min=-math.pi; msg.angle_max=math.pi; msg.angle_increment=2*math.pi/180; msg.range_min=.05; msg.range_max=5.0
        ranges=[]
        for i in range(181):
            angle=self.yaw+msg.angle_min+i*msg.angle_increment; hit=5.0
            for j in range(1,101):
                d=j*.05; x=self.x+d*math.cos(angle); y=self.y+d*math.sin(angle)
                if 1.0<=x<=1.2 and -.35<=y<=.35: hit=d; break
            ranges.append(hit)
        msg.ranges=ranges; self.scan.publish(msg)
    def run_task(self,name,x,y,yaw,expected,timeout=100):
        msg=TaskInfo(); msg.task_id=name; msg.task_type='navigation'; msg.end_action='waiting'
        p=Pose(); p.position.x=x; p.position.y=y; p.orientation.z=math.sin(yaw/2); p.orientation.w=math.cos(yaw/2); msg.poses=[p]
        self.tasks.publish(msg); start=time.monotonic()
        while time.monotonic()-start < timeout:
            rclpy.spin_once(self,timeout_sec=.03)
            matching=[s for s in self.statuses if s.task_id==name and s.task_status in ('Finished','Failed','Terminated')]
            if matching:
                status=matching[-1]; xy=math.hypot(self.x-x,self.y-y); yaw_error=abs(math.atan2(math.sin(self.yaw-yaw),math.cos(self.yaw-yaw)))
                record={'name':name,'status':status.task_status,'message':status.message,'xy_error':xy,'yaw_error':yaw_error,'elapsed':time.monotonic()-start,'pose':[self.x,self.y,self.yaw]}
                assert status.task_status==expected,record
                if expected=='Finished': assert xy<=.101 and yaw_error<=.151,record
                print('RESULT',json.dumps(record),flush=True); return record
        raise RuntimeError('task deadline exceeded: '+name+' pose='+str((self.x,self.y,self.yaw,self.v,self.w)))

def main():
    os.environ.update(ROS_DOMAIN_ID='93', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR='/tmp/od-navigation-check/ros')
    (OUT/'report.json').unlink(missing_ok=True)
    os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    rclpy.init(); sim=Simulation(); log=(OUT/'nav2.log').open('w')
    proc=subprocess.Popen(['ros2','launch','nav_bringup','stack.launch.py','robot_name:=navcheck','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    report=[]
    try:
        deadline=time.monotonic()+18
        while time.monotonic()<deadline: rclpy.spin_once(sim,timeout_sec=.03)
        report.append(sim.run_task('near-wall',.55,0.,0.,'Finished'))
        report.append(sim.run_task('turn',.55,0.,1.57,'Finished'))
        report.append(sim.run_task('around-obstacle',1.9,0.,0.,'Finished',timeout=150))
        report.append(sim.run_task('occupied-goal',1.1,0.,0.,'Failed',timeout=220))
        (OUT/'report.json').write_text(json.dumps(report,indent=2)); (OUT/'trajectory.json').write_text(json.dumps(sim.trajectory))
        print('ALL PASSED',flush=True)
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try: proc.wait(timeout=15)
        except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL); proc.wait()
        (OUT/'trajectory.json').write_text(json.dumps(sim.trajectory))
        sim.destroy_node(); rclpy.shutdown(); log.close()


if __name__ == "__main__":
    main()
