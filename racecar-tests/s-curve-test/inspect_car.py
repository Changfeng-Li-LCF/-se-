#!/usr/bin/env python3
"""Read-only ROS 2 snapshot and short observation; never publishes commands."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer, TransformListener
from lifecycle_msgs.srv import GetState

parser = argparse.ArgumentParser()
parser.add_argument('--output', required=True)
args = parser.parse_args()
out = Path(args.output)
out.mkdir(parents=True, exist_ok=False)
rclpy.init()
node = rclpy.create_node('s_curve_readonly_inspection')
buffer = Buffer()
listener = TransformListener(buffer, node)
counts = Counter()
poses, commands, scans, statuses, maps = [], [], [], {}, []
best = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
start = time.monotonic()

def cmd(message, topic):
    counts[topic] += 1
    commands.append([time.monotonic()-start, topic, message.linear.x, message.angular.z])

def odom(message):
    counts['/odom'] += 1
    q = message.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    p = message.pose.pose.position
    poses.append([time.monotonic()-start, p.x, p.y, yaw,
                  message.twist.twist.linear.x, message.twist.twist.angular.z])

def scan(message):
    counts['/scan'] += 1
    scans[:] = [message]

def map_callback(message):
    counts['/map'] += 1
    maps[:] = [message]

subscriptions = [
    node.create_subscription(Odometry, '/odom', odom, best),
    node.create_subscription(LaserScan, '/scan', scan, best),
    node.create_subscription(OccupancyGrid, '/map', map_callback, transient),
]
for topic in ('/car_cmd_vel', '/cmd_vel_nav', '/teleop_cmd_vel'):
    subscriptions.append(node.create_subscription(Twist, topic, lambda m,t=topic: cmd(m,t), best))
for topic in ('/follow_path/_action/status', '/navigate_to_pose/_action/status'):
    subscriptions.append(node.create_subscription(GoalStatusArray, topic,
        lambda m,t=topic: statuses.update({t:[s.status for s in m.status_list]}), transient))

requests = {
    '/racecar_driver': ['servo_center_pwm','servo_left_pwm','servo_right_pwm','wheelbase_m',
        'left_angle_deg','right_angle_deg','motor_neutral_pwm','motor_min_pwm','motor_max_pwm',
        'forward_pwm_per_mps','max_speed_mps','speed_epsilon_mps','dry_run'],
    '/controller_server': ['FollowPath.desired_linear_vel','FollowPath.lookahead_dist',
        'FollowPath.regulated_linear_scaling_min_speed','FollowPath.regulated_linear_scaling_min_radius',
        'FollowPath.use_collision_detection','FollowPath.use_rotate_to_heading'],
    '/velocity_smoother': ['max_velocity','min_velocity','scale_velocities','max_accel','max_decel'],
}
clients, pending, parameters, lifecycle = [], [], {}, {}
deadline = time.monotonic() + 18
sent = set()
while time.monotonic() < deadline:
    for target, names in requests.items():
        if target in sent:
            continue
        client = node.create_client(GetParameters, target+'/get_parameters')
        if client.service_is_ready():
            request = GetParameters.Request(names=names)
            pending.append((target,names,client.call_async(request)))
            clients.append(client)
            sent.add(target)
        else:
            node.destroy_client(client)
    rclpy.spin_once(node, timeout_sec=0.05)
    if len(sent) == len(requests) and all(f.done() for _,_,f in pending) and time.monotonic()-start > 10:
        break
for target,names,future in pending:
    if future.done() and future.result():
        values = []
        for value in future.result().values:
            field = {1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',
                     6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}.get(value.type)
            item = getattr(value,field) if field else None
            values.append(list(item) if value.type >= 6 and field else item)
        parameters[target] = dict(zip(names,values))
for target in ('/controller_server','/velocity_smoother'):
    client = node.create_client(GetState, target+'/get_state')
    if client.wait_for_service(timeout_sec=1):
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=2)
        if future.done() and future.result():
            lifecycle[target] = future.result().current_state.label
transforms = {}
for target,source in [('odom','base_footprint'),('odom','laser_link'),('map','odom')]:
    try:
        tf = buffer.lookup_transform(target,source,rclpy.time.Time())
        q=tf.transform.rotation; p=tf.transform.translation
        transforms[target+'<-'+source] = {'x':p.x,'y':p.y,
            'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
            'age_s':(node.get_clock().now().nanoseconds-(tf.header.stamp.sec*10**9+tf.header.stamp.nanosec))/1e9}
    except Exception as exc:
        transforms[target+'<-'+source] = {'error':str(exc)}
if scans:
    m=scans[0]
    (out/'scan.json').write_text(json.dumps({'frame':m.header.frame_id,'angle_min':m.angle_min,
        'angle_increment':m.angle_increment,'ranges':[r if math.isfinite(r) else None for r in m.ranges],
        'range_min':m.range_min,'range_max':m.range_max}))
if maps:
    m=maps[0]
    p=m.info.origin.position; q=m.info.origin.orientation
    (out/'map.json').write_text(json.dumps({'frame':m.header.frame_id,'resolution':m.info.resolution,
        'width':m.info.width,'height':m.info.height,'origin':[p.x,p.y,
        math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],'data':list(m.data)}))
summary = {'parameters':parameters,'lifecycle':lifecycle,'counts':dict(counts),
           'statuses':statuses,'transforms':transforms,'duration_s':time.monotonic()-start,
           'nodes':node.get_node_names(),'publishers':{t:[e.node_name for e in node.get_publishers_info_by_topic(t)]
               for t in ('/car_cmd_vel','/cmd_vel_nav','/teleop_cmd_vel')}}
if poses:
    summary['observed_pose_change_m'] = math.hypot(poses[-1][1]-poses[0][1],poses[-1][2]-poses[0][2])
    summary['pose_extent_m'] = [max(p[i] for p in poses)-min(p[i] for p in poses) for i in (1,2)]
summary['nonzero_commands'] = sum(abs(v)>0.001 or abs(w)>0.001 for _,_,v,w in commands)
(out/'inspection.json').write_text(json.dumps(summary,indent=2))
(out/'observations.json').write_text(json.dumps({'poses':poses,'commands':commands}))
print(json.dumps(summary,indent=2),flush=True)
node.destroy_node()
rclpy.shutdown()
