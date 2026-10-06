#!/usr/bin/env python3
"""Passive ROS 2 recorder; optional observed Path publication for RViz.

Never starts vehicle nodes, publishes motion commands, sends actions or opens serial.
Reference is anchored ONCE in odom; later runs can reuse --reference-world.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rcl_interfaces.msg import Log, ParameterEvent
from rcl_interfaces.srv import GetParameters
from action_msgs.msg import GoalStatusArray
from async_raw_writer import AsyncRawWriter
from tracking_core import anchor
from pose_sampling import PoseSampler


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {k:clean(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--reference', default=str(Path(__file__).with_name('reference_local.json')))
    ap.add_argument('--reference-world', help='Reuse an already anchored reference; never realign later')
    ap.add_argument('--output', required=True)
    ap.add_argument('--duration', type=float, default=120)
    ap.add_argument('--startup-timeout', type=float, default=15)
    ap.add_argument('--frame', default='odom')
    ap.add_argument('--base-frame', default='base_footprint')
    ap.add_argument('--pose-topic', default='/odom',help='Source-stamped localization Odometry; must match --frame/--base-frame')
    ap.add_argument('--record-map-tf', action='store_true',
                    help='Save map/odom TF separately for source-time trajectory reconstruction')
    ap.add_argument('--publish-actual-path',action='store_true',help='Publish observed RViz trajectory on this recorder')
    ap.add_argument('--actual-frame',choices=('map','odom'),default='odom')
    args = ap.parse_args()
    if args.duration <= 0 or args.startup_timeout <= 0:
        ap.error('Durations must be positive')
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    template = json.loads(Path(args.reference).read_text(encoding='utf-8'))
    world = json.loads(Path(args.reference_world).read_text(encoding='utf-8')) if args.reference_world else None
    if template['frame'] != 'start_local' or (world and world['frame'] != args.frame):
        ap.error('Reference frame mismatch')
    (out/'reference_local.json').write_text(json.dumps(template,indent=2), encoding='utf-8')
    if world:
        (out/'reference_world.json').write_text(json.dumps(world,indent=2), encoding='utf-8')
    config_root = Path.home()/'racecar'
    for label, relative in [('source_driver','src/racecar/config/driver_calibration.yaml'),
                            ('installed_driver','install/racecar/share/racecar/config/driver_calibration.yaml'),
                            ('source_nav_carto','src/racecar/config/nav_carto.yaml'),
                            ('installed_nav_carto','install/racecar/share/racecar/config/nav_carto.yaml')]:
        if (config_root/relative).is_file():
            shutil.copy2(config_root/relative, out/(label+'.yaml'))
    rclpy.init()
    node = rclpy.create_node('s_curve_passive_recorder')
    # Localization uses the same TF-derived pose already published at 20 Hz.
    # Do not create a second, deep TF listener queue on this recording node.
    latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    tf_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT) if args.record_map_tf or (args.publish_actual_path and args.actual_frame != args.frame) else latest
    transient = QoSProfile(depth=30, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
    start = time.monotonic(); counts = Counter(); reject = Counter(); parameters = {}
    last_pose_flush = start
    sampler=PoseSampler(rate_hz=20.0)
    raw_counts=Counter();raw_skipped=Counter();last_raw_time={};ready_written=False
    raw_rates={'/tf':20.0,'/scan':5.0,'/imu/data':20.0,'/imu':20.0,'/IMU_data':50.0,
               '/FollowPath/lookahead_samples':5.0}
    metadata = {'started_utc':datetime.now(timezone.utc).isoformat(), 'mode':'passive_only',
                'frame':args.frame, 'base_frame':args.base_frame, 'parameters':parameters,
                'pose_source':'Odometry.pose from '+args.pose_topic+'; Cartographer TF-derived localization, not ground truth',
                'pose_topic':args.pose_topic,'pose_rate_limit_hz':20.0,
                'tf_age_s_field':'Age of the source Odometry pose stamp; CSV column name retained for compatibility',
                'raw_rate_limits_hz':raw_rates,
                'reference_alignment':'fixed at first fresh pose' if world is None else 'reuse supplied world reference',
                'raw_log':'raw.jsonl (separate-process JSON writer, bounded queue; drops reported, not rosbag)',
                'full_plan_recording':'reference_world.json stores the complete reference; large plan topics intentionally unsubscribed',
                'preview_topic':'/FollowPath/lookahead_samples'}
    live_trajectory = None
    if args.publish_actual_path:
        from live_trajectory import LiveTrajectory
        live_trajectory = LiveTrajectory(node,args.frame,args.actual_frame)
    raw = AsyncRawWriter(out/'raw.jsonl')
    posefile = (out/'poses.csv').open('w',newline='',encoding='utf-8')
    cmdfile = (out/'commands.csv').open('w',newline='',encoding='utf-8')
    map_tf_file = (out/'map_transforms.csv').open('w',newline='',encoding='utf-8') if args.record_map_tf else None
    map_tf_count = 0
    if map_tf_file:
        map_tf_writer = csv.writer(map_tf_file)
        map_tf_writer.writerow(['stamp','x','y','z','qx','qy','qz','qw'])
        metadata['map_transform_log'] = 'map_transforms.csv; callback capture before bulk TF rate limiting'
    posewriter = csv.DictWriter(posefile,fieldnames=['t','stamp','x','y','yaw','tf_age_s','quality'])
    posewriter.writeheader()
    cmdwriter = csv.DictWriter(cmdfile,fieldnames=['t','topic','v','w']);cmdwriter.writeheader()
    def capture_pose(msg,t):
        nonlocal world,ready_written
        if msg.header.frame_id!=args.frame or msg.child_frame_id!=args.base_frame:
            reject['pose_frame_mismatch']+=1;return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        angle=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        row=sampler.offer(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            x=p.x,y=p.y,yaw=angle,now=node.get_clock().now().nanoseconds/1e9,receipt=t)
        if row is None:return
        if world is None:
            world=anchor(template,p.x,p.y,angle,args.frame);world['anchor_stamp']=row['stamp']
            (out/'reference_world.json').write_text(json.dumps(world,indent=2),encoding='utf-8')
        posewriter.writerow(row)
        if live_trajectory:live_trajectory.offer(row)
        if sampler.ready and not ready_written:
            posefile.flush();cmdfile.flush()
            temporary=out/'ready.json.tmp'
            temporary.write_text(json.dumps({'pid':__import__('os').getpid(),
                'stamp':row['stamp'],'pose_samples':sampler.sample_count,'pose_topic':args.pose_topic,
                'ready':True}),encoding='utf-8')
            temporary.replace(out/'ready.json');ready_written=True
    def capture_map_tf(msg):
        nonlocal map_tf_count
        # Reuse the existing subscription. Small buffered records bypass bulk
        # JSON throttling/drops; coordinate reconstruction runs on the PC.
        for tf in msg.transforms:
            if tf.header.frame_id.lstrip('/') == 'map' and tf.child_frame_id.lstrip('/') == args.frame.lstrip('/'):
                p,q = tf.transform.translation,tf.transform.rotation
                map_tf_writer.writerow([tf.header.stamp.sec+tf.header.stamp.nanosec/1e9,
                                        p.x,p.y,p.z,q.x,q.y,q.z,q.w])
                map_tf_count += 1

    def capture(msg, topic):
        counts[topic] += 1
        t = time.monotonic()-start
        # Save the small, fresh pose first; bulk JSON must not delay readiness.
        if topic==args.pose_topic:capture_pose(msg,t)
        if topic=='/tf':
            if map_tf_file:capture_map_tf(msg)
            if live_trajectory:live_trajectory.capture_tf(msg)
        rate=raw_rates.get(topic)
        if rate and t-last_raw_time.get(topic,-math.inf)<1.0/rate:
            raw_skipped[topic]+=1;return
        last_raw_time[topic]=t
        if raw.enqueue(t,topic,msg):raw_counts[topic]+=1
        else:raw_skipped[topic+'_queue_full']+=1
        if isinstance(msg,Twist):
            cmdwriter.writerow(dict(t=t,topic=topic,v=msg.linear.x,w=msg.angular.z))
    specs = [('/racecar_driver/closed_loop_state',String,best),
             (args.pose_topic,Odometry,latest),('/scan',LaserScan,latest),('/imu/data',Imu,latest),
             ('/imu',Imu,latest),('/IMU_data',Imu,latest),('/tf',TFMessage,tf_qos),('/tf_static',TFMessage,transient),
             ('/rosout',Log,best),('/parameter_events',ParameterEvent,best),
             ('/FollowPath/lookahead_samples',RosPath,latest),
             ('/follow_path/_action/status',GoalStatusArray,transient),
             ('/navigate_to_pose/_action/status',GoalStatusArray,transient)]
    specs += [(t,Twist,best) for t in ('/cmd_vel_nav','/car_cmd_vel','/cmd_vel','/teleop_cmd_vel')]
    subscriptions = [node.create_subscription(typ,topic,lambda m,t=topic:capture(m,t),qos)
                     for topic,typ,qos in specs]
    requests = {
        '/racecar_driver':['wheelbase_m','left_angle_deg','right_angle_deg','servo_center_pwm',
                          'servo_left_pwm','servo_right_pwm','motor_neutral_pwm','motor_min_pwm',
                          'motor_max_pwm','forward_pwm_per_mps','reverse_pwm_per_mps',
                          'max_speed_mps','speed_epsilon_mps','dry_run','command_timeout_s',
                          'speed_feedback_enabled','yaw_rate_feedback_enabled',
                          'motor_kp','motor_ki','yaw_kp_deg_per_radps','yaw_ki_deg_per_rad',
                          'feedback_timeout_s','feedback_filter_tau_s',
                          'imu_yaw_feedback_enabled','imu_feedback_topic','imu_feedback_frame',
                          'imu_yaw_axis','imu_yaw_bias_radps','imu_feedback_timeout_s','imu_filter_tau_s'],
        '/velocity_smoother':['max_velocity','min_velocity','max_accel','max_decel','scale_velocities'],
        '/controller_server':['FollowPath.desired_linear_vel','FollowPath.lookahead_dist',
                              'FollowPath.regulated_linear_scaling_min_radius',
                              'FollowPath.regulated_linear_scaling_min_speed',
                              'FollowPath.min_approach_linear_velocity',
                              'FollowPath.use_regulated_linear_velocity_scaling',
                              'FollowPath.use_cost_regulated_linear_velocity_scaling']}
    clients = {name:node.create_client(GetParameters,name+'/get_parameters') for name in requests}
    pending = {};stop_reason = 'duration'
    try:
        while rclpy.ok() and time.monotonic()-start < args.duration:
            raw.check()
            rclpy.spin_once(node,timeout_sec=.02)
            if time.monotonic()-last_pose_flush >= .2:
                posefile.flush()
                if map_tf_file:map_tf_file.flush()
                last_pose_flush = time.monotonic()
            t = time.monotonic()-start
            for name,client in clients.items():
                if name not in pending and client.service_is_ready():
                    pending[name] = client.call_async(GetParameters.Request(names=requests[name]))
                if name in pending and pending[name].done() and name not in parameters:
                    response = pending[name].result()
                    if response:
                        fields = {1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',
                                  6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
                        values = []
                        for p in response.values:
                            v = getattr(p,fields[p.type]) if p.type in fields else None
                            values.append(list(v) if p.type >= 6 and v is not None else v)
                        parameters[name] = dict(zip(requests[name],values))
            if not ready_written and t > args.startup_timeout:
                stop_reason = 'no_fresh_pose'; break
            if ready_written and t-sampler.last_fresh_receipt>1.0:
                metadata['pose_stream_failure']={'receipt_age_s':t-sampler.last_fresh_receipt,
                    'last_source_stamp':sampler.last_received_stamp,'rejections':dict(sampler.rejected),
                    'raw_enqueued':raw.enqueued,'raw_dropped':raw.dropped}
                stop_reason='pose_stream_stale';break
    except KeyboardInterrupt:
        stop_reason = 'user_interrupt'
    finally:
        # SIGINT may invalidate the ROS context before this block. Always save files.
        try:
            node_names=node.get_node_names() if rclpy.ok() else []
            publishers={t:[e.node_name for e in node.get_publishers_info_by_topic(t)]
                        for t in ('/cmd_vel_nav','/car_cmd_vel','/cmd_vel','/teleop_cmd_vel')} if rclpy.ok() else {}
        except Exception as exc:
            node_names=[];publishers={};metadata['shutdown_graph_error']=str(exc)
        metadata['raw_writer']=raw.close()
        reject.update(sampler.rejected)
        metadata.update(duration_s=time.monotonic()-start, stop_reason=stop_reason,
                        pose_samples=sampler.sample_count, counts=dict(counts),quality_events=dict(reject),
                        raw_enqueued_counts=dict(raw_counts),raw_rate_limited_counts=dict(raw_skipped),
                        last_pose_stamp=sampler.last_written['stamp'] if sampler.last_written else None,
                        nodes=node_names,command_publishers=publishers,
                        unavailable_parameters=[n for n in requests if n not in parameters])
        metadata['pose_timing'] = dict(max_source_gap_s=sampler.max_source_gap_s,
            max_receipt_gap_s=sampler.max_receipt_gap_s,max_recorded_gap_s=sampler.max_recorded_gap_s)
        if live_trajectory:metadata['live_trajectory'] = live_trajectory.snapshot()
        if map_tf_file:metadata['map_transform_samples'] = map_tf_count
        (out/'metadata.json').write_text(json.dumps(clean(metadata),indent=2,ensure_ascii=False),encoding='utf-8')
        if map_tf_file:
            map_tf_file.close()
        posefile.close();cmdfile.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print(json.dumps({'output':str(out),'poses':sampler.sample_count,'stop_reason':stop_reason}))
    return 0 if ready_written and stop_reason in ('duration','user_interrupt') else 2

if __name__ == '__main__':
    raise SystemExit(main())
