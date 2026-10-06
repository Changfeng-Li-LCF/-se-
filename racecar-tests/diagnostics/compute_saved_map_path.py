#!/usr/bin/env python3
"""Run Nav2's planner on a saved map in a separate, localhost-only ROS domain.
Only map_server, planner_server and a synthetic static TF are started. No controller.
"""
import os
os.environ['ROS_DOMAIN_ID']='83'
os.environ['ROS_LOCALHOST_ONLY']='1'
import argparse, json, math, pathlib, signal, subprocess, time
import yaml
import rclpy
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import GetCostmap
from rclpy.action import ActionClient

parser=argparse.ArgumentParser()
parser.add_argument('--map',required=True)
parser.add_argument('--budget',type=float,default=7.)
args=parser.parse_args()
home=pathlib.Path.home()
run=home/'maps/plans'/time.strftime('saved-map-%Y%m%d-%H%M%S')
run.mkdir(parents=True)
source=home/'racecar/install/racecar/share/racecar/config/nav_carto.yaml'
base=yaml.safe_load(source.read_text())
config={k:base[k] for k in ['planner_server','global_costmap']}
config['planner_server']['ros__parameters']['GridBased']['max_planning_time']=args.budget
cost=config['global_costmap']['global_costmap']['ros__parameters']
# Planning against the saved snapshot only, with the actual footprint/inflation.
cost['plugins']=['static_layer','inflation_layer']
cost.pop('obstacle_layer',None)
cost['static_layer']['map_subscribe_transient_local']=True
cost['static_layer']['subscribe_to_updates']=False
config['map_server']={'ros__parameters':{'yaml_filename':str(pathlib.Path(args.map).resolve()),'use_sim_time':False}}
params=run/'planner_params.yaml';params.write_text(yaml.safe_dump(config))
report={'map':args.map,'start':[.2,-.1,0.],'goal':[.2,.85,math.pi],
        'mode':'native Nav2 ComputePathToPose; saved-map static+inflation only',
        'ros_domain_id':83,'localhost_only':True,'motion_nodes_started':False,
        'planning_budget_s':args.budget,'footprint':cost['footprint'],
        'minimum_turning_radius':config['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']}
children=[]; handles=[]
def start(package,exe,extra):
    log=(run/(exe+'.log')).open('w');handles.append(log)
    p=subprocess.Popen(['/opt/ros/humble/lib/'+package+'/'+exe]+extra,
                       stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    children.append(p)

def await_future(future,timeout):
    end=time.monotonic()+timeout
    while not future.done() and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
    if not future.done():raise TimeoutError('ROS future timeout')
    return future.result()

def transition(name,ident):
    client=node.create_client(ChangeState,'/'+name+'/change_state')
    if not client.wait_for_service(timeout_sec=25.):raise TimeoutError(name+' lifecycle unavailable')
    request=ChangeState.Request();request.transition.id=ident
    result=await_future(client.call_async(request),40.)
    if not result.success:raise RuntimeError(name+' transition failed: '+str(ident))
    node.destroy_client(client)
    print(json.dumps({'stage':'lifecycle','node':name,'transition':ident}),flush=True)

def pose(x,y,yaw):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
    p.pose.position.x=x;p.pose.position.y=y
    p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
    return p

rclpy.init();node=rclpy.create_node('saved_map_planning_request')
try:
    start('tf2_ros','static_transform_publisher',['0.2','-0.1','0','0','0','0','map','base_footprint'])
    start('nav2_map_server','map_server',['--ros-args','--params-file',str(params)])
    start('nav2_planner','planner_server',['--ros-args','--params-file',str(params)])
    for name in ['map_server','planner_server']:
        transition(name,1);transition(name,3)
    action=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    if not action.wait_for_server(timeout_sec=15.):raise TimeoutError('Planner action unavailable')
    request=ComputePathToPose.Goal();request.start=pose(*report['start']);request.goal=pose(*report['goal'])
    request.use_start=True;request.planner_id='GridBased'
    handle=await_future(action.send_goal_async(request),10.)
    if not handle.accepted:raise RuntimeError('Planning request rejected')
    print(json.dumps({'stage':'planning','run':str(run)}),flush=True)
    result=await_future(handle.get_result_async(),args.budget+25.)
    report['action_status']=result.status
    path=result.result.path
    report['path']=[{'x':p.pose.position.x,'y':p.pose.position.y,
                     'qx':p.pose.orientation.x,'qy':p.pose.orientation.y,
                     'qz':p.pose.orientation.z,'qw':p.pose.orientation.w} for p in path.poses]
    report['length_m']=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(report['path'],report['path'][1:]))
    report['planning_time_s']=result.result.planning_time.sec+result.result.planning_time.nanosec*1e-9
    client=node.create_client(GetCostmap,'/global_costmap/get_costmap')
    if client.wait_for_service(timeout_sec=3.):
        cm=await_future(client.call_async(GetCostmap.Request()),5.).map
        data={'size_x':cm.metadata.size_x,'size_y':cm.metadata.size_y,'resolution':cm.metadata.resolution,
              'origin':[cm.metadata.origin.position.x,cm.metadata.origin.position.y], 'data':list(cm.data)}
        (run/'costmap.json').write_text(json.dumps(data))
except Exception as e:
    report['error']=str(e)
finally:
    (run/'result.json').write_text(json.dumps(report,indent=2))
    for p in children:
        if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
    for p in children:
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    for h in handles:h.close()
    node.destroy_node();rclpy.shutdown()
print(json.dumps({'run':str(run),**{k:v for k,v in report.items() if k!='path'},'poses':len(report.get('path',[]))}),flush=True)
