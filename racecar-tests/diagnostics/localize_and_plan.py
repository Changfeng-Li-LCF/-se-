#!/usr/bin/env python3
"""Explicit user trial: global AMCL localization then planning only; no motion goals."""
import json,math,pathlib,time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from nav2_msgs.action import ComputePathToPose
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer,TransformListener
rclpy.init();n=rclpy.create_node('global_localization_plan_only_trial')
run=pathlib.Path.home()/'maps/plans'/time.strftime('localized-plan-%Y%m%d-%H%M%S');run.mkdir(parents=True)
state={};history=[];report={'run':str(run),'motion_goal_sent':False,'goal':[.2,.85,math.pi]}
buf=Buffer();listener=TransformListener(buf,n)
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def posecb(m):
 state['pose']=m
 history.append({'received':time.time(),'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,'x':m.pose.pose.position.x,'y':m.pose.pose.position.y,'yaw':yaw(m.pose.pose.orientation),'covariance':list(m.pose.covariance)})
subs=[n.create_subscription(OccupancyGrid,'/map',lambda m:state.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),n.create_subscription(LaserScan,'/scan',lambda m:state.update(scan=m),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)),n.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',posecb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
def wait(f,seconds):
 end=time.monotonic()+seconds
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 if not f.done():raise TimeoutError('Request exceeded '+str(seconds)+' seconds')
 return f.result()
def empty(name):
 c=n.create_client(Empty,name)
 if not c.wait_for_service(timeout_sec=5):raise RuntimeError(name+' unavailable')
 wait(c.call_async(Empty.Request()),8);n.destroy_client(c)
def output(stage,**data):print(json.dumps({'stage':stage,**data}),flush=True)
try:
 spin(4)
 if 'map' not in state or 'scan' not in state:raise RuntimeError('Map or laser scan unavailable')
 c=n.create_client(GetParameters,'/map_server/get_parameters');c.wait_for_service(timeout_sec=3)
 q=GetParameters.Request();q.names=['yaml_filename'];report['map']=wait(c.call_async(q),5).values[0].string_value
 report['before']=history[-1] if history else None
 output('global_localization_started',run=str(run),map=report['map'])
 empty('/reinitialize_global_localization');history.clear()
 for i in range(30):
  empty('/request_nomotion_update');spin(.45)
  if i%5==4 and history:output('matching',updates=i+1,pose={k:history[-1][k] for k in ['x','y','yaw']})
 if len(history)<3:raise RuntimeError('AMCL did not publish enough new pose estimates')
 p=history[-1];report['localization']=p
 report['std_xy_m']=[math.sqrt(max(0,p['covariance'][j])) for j in [0,7]]
 report['std_yaw_deg']=math.degrees(math.sqrt(max(0,p['covariance'][35])))
 m=state['map'];s=state['scan'];a=np.array(m.data).reshape(m.info.height,m.info.width)
 if abs(yaw(m.info.origin.orientation))>1e-5:raise RuntimeError('Rotated map origin requires explicit support')
 yy,xx=np.where(a>=65);obs=np.column_stack((m.info.origin.position.x+(xx+.5)*m.info.resolution,m.info.origin.position.y+(yy+.5)*m.info.resolution))
 tf=buf.lookup_transform('map',s.header.frame_id,rclpy.time.Time());t=tf.transform;heading=yaw(t.rotation)
 rays=[]
 for i,r in enumerate(s.ranges):
  if math.isfinite(r) and max(.15,s.range_min)<r<min(10.,s.range_max):
   angle=heading+s.angle_min+i*s.angle_increment;rays.append([t.translation.x+r*math.cos(angle),t.translation.y+r*math.sin(angle)])
 rays=np.array(rays)[::max(1,len(rays)//160)]
 if len(rays)<20 or len(obs)==0:raise RuntimeError('Insufficient usable laser returns or occupied cells')
 distances=np.array([np.sqrt(np.min(np.sum((obs-point)**2,axis=1))) for point in rays])
 report['scan_match']={'valid_rays':len(rays),'fraction_within_015m':float(np.mean(distances<.15)),'median_endpoint_error_m':float(np.median(distances))}
 stable=max(math.hypot(h['x']-p['x'],h['y']-p['y']) for h in history[-5:])<.15
 report['candidate_gate_passed']=max(report['std_xy_m'])<.3 and report['std_yaw_deg']<20 and report['scan_match']['fraction_within_015m']>.7 and stable
 output('localization_candidate',**{k:report[k] for k in ['std_xy_m','std_yaw_deg','scan_match','candidate_gate_passed']})
 report['map_snapshot']={'width':m.info.width,'height':m.info.height,'resolution':m.info.resolution,'origin':[m.info.origin.position.x,m.info.origin.position.y],'data':list(m.data)}
 report['laser_endpoints_map']=rays.tolist()
 if not report['candidate_gate_passed']:raise RuntimeError('Global localization not sufficiently converged/aligned; no planning request sent')
 action=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
 if not action.wait_for_server(timeout_sec=8):raise RuntimeError('Planner unavailable')
 req=ComputePathToPose.Goal();req.use_start=False;req.planner_id='GridBased';req.goal.header.frame_id='map';req.goal.header.stamp=n.get_clock().now().to_msg();req.goal.pose.position.x=.2;req.goal.pose.position.y=.85;req.goal.pose.orientation.z=1.
 handle=wait(action.send_goal_async(req),8)
 if not handle.accepted:raise RuntimeError('Planning goal rejected')
 result=wait(handle.get_result_async(),35);report['action_status']=result.status
 path=result.result.path;report['path']=[{'x':v.pose.position.x,'y':v.pose.position.y,'qx':v.pose.orientation.x,'qy':v.pose.orientation.y,'qz':v.pose.orientation.z,'qw':v.pose.orientation.w} for v in path.poses]
 report['length_m']=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(report['path'],report['path'][1:]))
 output('planning_complete',status=result.status,poses=len(path.poses),length_m=report['length_m'])
except Exception as e:report['error']=str(e);output('stopped',reason=str(e))
finally:
 report['pose_history']=history;(run/'result.json').write_text(json.dumps(report,indent=2));output('saved',run=str(run));n.destroy_node();rclpy.shutdown()
