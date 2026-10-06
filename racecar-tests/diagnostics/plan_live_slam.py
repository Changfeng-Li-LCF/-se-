#!/usr/bin/env python3
"""Plan only from current SLAM pose into observed free space. Never sends a motion goal."""
import json,math,pathlib,time
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid,Path
from nav2_msgs.action import ComputePathToPose
from tf2_ros import Buffer,TransformListener
from std_msgs.msg import Bool
rclpy.init();n=rclpy.create_node('live_slam_plan_only');state={};buf=Buffer();listener=TransformListener(buf,n)
run=pathlib.Path.home()/'maps/plans'/time.strftime('live-slam-%Y%m%d-%H%M%S');run.mkdir(parents=True)
report={'run':str(run),'mode':'fresh live SLAM map; current TF start; planning only','motion_goal_sent':False,'attempts':[]}
subs=[n.create_subscription(OccupancyGrid,'/map',lambda m:state.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),n.create_subscription(Bool,'/racecar_driver/emergency_stop_active',lambda m:state.update(stop=m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
def output(stage,**kw):print(json.dumps({'stage':stage,**kw}),flush=True)
def wait(f,timeout):
 end=time.monotonic()+timeout
 while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
 if not f.done():raise TimeoutError('ROS request timeout')
 return f.result()
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
try:
 end=time.monotonic()+40;tf=None
 while time.monotonic()<end:
  rclpy.spin_once(n,timeout_sec=.1)
  if 'map' in state:
   try:tf=buf.lookup_transform('map','base_footprint',rclpy.time.Time());break
   except Exception:pass
 if tf is None:raise RuntimeError('Live map/robot TF not ready')
 report['map_publishers']=[x.node_name for x in n.get_publishers_info_by_topic('/map')]
 if 'map_server' in report['map_publishers']:raise RuntimeError('Unexpected saved map publisher still present')
 report['emergency_stopped']=state.get('stop')
 m=state['map'];grid=np.array(m.data).reshape(m.info.height,m.info.width);origin=m.info.origin.position;res=m.info.resolution
 if abs(yaw(m.info.origin.orientation))>1e-5:raise RuntimeError('Rotated map origin unsupported')
 t=tf.transform;theta=yaw(t.rotation);start=[t.translation.x,t.translation.y,theta];report['start']=start
 report['map_snapshot']={'width':m.info.width,'height':m.info.height,'resolution':res,'origin':[origin.x,origin.y],'data':list(m.data)}
 config=yaml.safe_load((pathlib.Path(get_package_share_directory("racecar"))/"config/nav_carto.yaml").read_text())
 footprint=yaml.safe_load(config["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"])
 xs=[v[0] for v in footprint];ys=[v[1] for v in footprint]
 # Use the configured footprint bounding box plus 0.05 m goal clearance.
 def free_footprint(x,y):
  for dx in np.arange(min(xs)-.05,max(xs)+.05+res/4,res/2):
   for dy in np.arange(min(ys)-.05,max(ys)+.05+res/4,res/2):
    wx=x+dx*math.cos(theta)-dy*math.sin(theta);wy=y+dx*math.sin(theta)+dy*math.cos(theta)
    ix=math.floor((wx-origin.x)/res);iy=math.floor((wy-origin.y)/res)
    if not(0<=ix<m.info.width and 0<=iy<m.info.height) or not(0<=grid[iy,ix]<65):return False
  return True
 candidates=[]
 for dist in [2.,1.5,1.,.75]:
  gx=start[0]+dist*math.cos(theta);gy=start[1]+dist*math.sin(theta)
  if free_footprint(gx,gy):candidates.append((dist,gx,gy))
 report['candidates']=candidates
 output('map_ready',start=start,map_publishers=report['map_publishers'],candidates=candidates)
 if not candidates:raise RuntimeError('No observed free forward goal large enough for footprint')
 action=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
 if not action.wait_for_server(timeout_sec=20):raise RuntimeError('Planner action unavailable')
 for dist,gx,gy in candidates:
  request=ComputePathToPose.Goal();request.use_start=False;request.planner_id='GridBased';request.goal.header.frame_id='map';request.goal.header.stamp=n.get_clock().now().to_msg();request.goal.pose.position.x=gx;request.goal.pose.position.y=gy;request.goal.pose.orientation.z=math.sin(theta/2);request.goal.pose.orientation.w=math.cos(theta/2)
  handle=wait(action.send_goal_async(request),8)
  if not handle.accepted:report['attempts'].append({'distance':dist,'accepted':False});continue
  result=wait(handle.get_result_async(),30);report['attempts'].append({'distance':dist,'status':result.status})
  if result.status!=4 or not result.result.path.poses:continue
  path=result.result.path;report['goal']=[gx,gy,theta];report['action_status']=result.status;report['planning_time_s']=result.result.planning_time.sec+result.result.planning_time.nanosec*1e-9
  report['path']=[{'x':v.pose.position.x,'y':v.pose.position.y,'qx':v.pose.orientation.x,'qy':v.pose.orientation.y,'qz':v.pose.orientation.z,'qw':v.pose.orientation.w} for v in path.poses]
  report['length_m']=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(report['path'],report['path'][1:]))
  # Keep a dedicated preview topic as well as the planner's /plan publication.
  pub=n.create_publisher(Path,'/live_slam_preview_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));pub.publish(path)
  output('planned',goal=report['goal'],length_m=report['length_m'],poses=len(path.poses),planning_time_s=report['planning_time_s'])
  end=time.monotonic()+3
  while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
  break
 if 'path' not in report:raise RuntimeError('No forward candidate yielded a path with the current vehicle constraints')
except Exception as e:report['error']=str(e);output('failed',reason=str(e))
finally:
 (run/'result.json').write_text(json.dumps(report,indent=2));output('saved',run=str(run));n.destroy_node();rclpy.shutdown()
