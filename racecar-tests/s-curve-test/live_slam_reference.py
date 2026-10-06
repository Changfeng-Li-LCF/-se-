"""One path planned from a fresh live map; no motion publications in this module."""
import math,json,time
from pathlib import Path
import numpy as np
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def wrap(a):return math.atan2(math.sin(a),math.cos(a))
def forward_candidates(start,grid=None):
 x,y,a=start
 return [(d,x+d*math.cos(a),y+d*math.sin(a),a) for d in [2.,1.5,1.,.75]]

def template_from_odom(poses,start):
 x,y,a=start;c,s=math.cos(a),math.sin(a);points=[];length=0.
 for px,py,heading in poses:
  dx,dy=px-x,py-y;lx=c*dx+s*dy;ly=-s*dx+c*dy
  if points:
   dist=math.hypot(lx-points[-1]['x'],ly-points[-1]['y'])
   if dist<1e-8:continue
   length+=dist
  points.append(dict(x=lx,y=ly,yaw=wrap(heading-a),s=length,curvature=0.))
 if len(points)<2 or length<.3:raise RuntimeError('Planned path is too short or degenerate')
 for u,v in zip(points,points[1:]):u['curvature']=wrap(v['yaw']-u['yaw'])/(v['s']-u['s'])
 points[-1]['curvature']=points[-2]['curvature']
 return dict(schema=1,name='fresh_live_slam_forward_plan',frame='start_local',length_m=length,points=points,
             bounds_m={axis:[min(p[axis] for p in points),max(p[axis] for p in points)] for axis in ['x','y']})
def plan_reference(node,buffer,observed,await_future,run,nav_source,start,goal_candidates=None,candidate_frame=None):
 from rclpy.action import ActionClient
 from rclpy.qos import QoSProfile,DurabilityPolicy
 from nav2_msgs.action import ComputePathToPose
 from nav_msgs.msg import Path as RosPath
 from planner_costmap_audit import snapshot_global_costmap,current_pose,transform_poses,runtime_rules
 publishers=[p.node_name for p in node.get_publishers_info_by_topic('/map')]
 if not publishers or any('map_server' in n for n in publishers) or not any('occupancy_grid' in n for n in publishers):
  raise RuntimeError('Fresh Cartographer /map required; saved-map publisher refused: '+str(publishers))
 if time.monotonic()-observed['map'][0]>3:raise RuntimeError('Live map is stale before planning')
 directory=Path(run)/'planning_audit';directory.mkdir(parents=True,exist_ok=True)
 result={'mode':'fresh_live_slam','map_publishers':publishers,'attempts':[],
         'collision_source':'/global_costmap/get_costmap','collision_policy':'runtime Smac uint8 costs and footprint'}
 action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
 try:
  rules=runtime_rules(node,await_future)
  initial,info=snapshot_global_costmap(node,await_future,directory,'initial',rules)
  frame=info['frame'];current=current_pose(node,buffer,frame,rules['base_frame'])
  if goal_candidates is not None and candidate_frame!=frame:raise RuntimeError('Exploration candidate/costmap frame mismatch')
  candidates=goal_candidates if goal_candidates is not None else forward_candidates(current)
  from goal_heading_preferences import heading_preferences
  # A selected boundary-normal approach may fail curvature/footprint checks;
  # try the direct bearing for this same point before abandoning the candidate.
  candidates=[(dist,gx,gy,heading) for dist,gx,gy,ga in candidates
              for heading in heading_preferences(current,(gx,gy,ga))]
  result['goal_heading_policy']='selected_approach_then_direct_bearing'
  result['goal_selection']='open_space_exploration' if goal_candidates is not None else 'forward_demo'
  result.update(start_map=current,candidates=candidates,rules=rules)
  if not action.wait_for_server(timeout_sec=10):raise RuntimeError('Planner action unavailable')
  for index,(dist,gx,gy,ga) in enumerate(candidates):
   label='candidate-%02d'%index
   attempt={'distance':dist,'goal':[gx,gy,ga],'path':[],'first_rejection':None,
            'costmap_before':str(directory/(label+'.before.costmap.json'))}
   result['attempts'].append(attempt)
   handle=None
   try:
    grid,before=snapshot_global_costmap(node,await_future,directory,label+'.before',rules)
    if before['frame']!=frame:raise RuntimeError('Global costmap frame changed')
    current=current_pose(node,buffer,frame,rules['base_frame']);attempt['start_pose']=current
    # Same check at candidate selection and immediately before executing the path.
    for phase,poses in [('current_footprint',[current]),('goal_footprint',[[gx,gy,ga]])]:
     checked=grid.audit(poses);attempt[phase]=checked
     if not checked['clear']:
      attempt.update(rejected=phase,first_rejection=checked['first_rejection']);break
    if 'rejected' in attempt:continue
    request=ComputePathToPose.Goal();request.use_start=False;request.planner_id='GridBased'
    request.goal.header.frame_id=frame;request.goal.header.stamp=node.get_clock().now().to_msg()
    request.goal.pose.position.x=gx;request.goal.pose.position.y=gy
    request.goal.pose.orientation.z=math.sin(ga/2);request.goal.pose.orientation.w=math.cos(ga/2)
    handle=await_future(action.send_goal_async(request),15)
    attempt['accepted']=handle.accepted
    if not handle.accepted:
     attempt.update(rejected='planner_goal_rejected',first_rejection={'classification':'planner_goal_rejected','cell':None});continue
    answer=await_future(handle.get_result_async(),25);attempt['status']=answer.status
    path=answer.result.path
    map_poses=[(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in path.poses]
    attempt['path']=map_poses;attempt['path_frame']=path.header.frame_id
    if answer.status!=4 or len(path.poses)<2:
     # Planner failure does not identify a collision cell; never invent one.
     attempt.update(rejected='planner_no_path',first_rejection={'classification':'planner_no_path','cell':None});continue
    if path.header.frame_id!=frame:raise RuntimeError('Planner returned unexpected frame')
    latest,after=snapshot_global_costmap(node,await_future,directory,label+'.after',rules)
    attempt['costmap_after']=str(directory/(label+'.after.costmap.json'))
    if after['frame']!=frame:raise RuntimeError('Global costmap frame changed')
    current=current_pose(node,buffer,frame,rules['base_frame'])
    sequence=[current]+map_poses
    check=latest.audit(sequence);attempt['path_audit']=check
    if not check['clear']:
     attempt.update(rejected='latest_costmap_path_or_connector',first_rejection=check['first_rejection']);continue
    poses=transform_poses(buffer,'odom',frame,map_poses)
    template=template_from_odom(poses,start)
    result.update(goal_map=[gx,gy,ga],path_map=map_poses,path_odom=poses,map_frame=frame,
                  length_m=template['length_m'],selected_distance_m=dist,selected_attempt=index)
    attempt['selected']=True
    pub=node.create_publisher(RosPath,'/live_slam_initial_plan',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub.publish(path);observed['live_plan_publisher']=pub
    observed['live_plan_timer']=node.create_timer(.5,lambda:pub.publish(path))
    return template,result
   except Exception as error:
    attempt.update(error=str(error),first_rejection=attempt.get('first_rejection') or {'classification':'audit_or_planner_unavailable','cell':None})
    if handle is not None and handle.accepted:
     try:await_future(handle.cancel_goal_async(),3)
     except Exception:pass
    # Timeouts/TF failure are not evidence a shorter goal is reachable.
    raise
   finally:
    (directory/(label+'.json')).write_text(json.dumps(attempt,ensure_ascii=False,indent=2),encoding='utf-8')
  raise RuntimeError('No usable forward path on the current planner global costmap; see planning_audit candidate reports')
 finally:
  (Path(run)/'live_plan.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
  action.destroy()
