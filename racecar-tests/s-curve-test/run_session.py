#!/usr/bin/env python3
"""Recorded path trial or live SLAM navigation with continuous replanning."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback




def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def stop_process(p):
    if p and p.poll() is None:
        try:os.killpg(p.pid,signal.SIGINT)
        except ProcessLookupError:p.wait();return
        try: p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try: p.wait(timeout=4)
            except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=2)


def check_map(reference, grid, tf):
    """Check a 0.40 m swept-radius corridor, treating unknown cells as blocked."""
    q=tf.transform.rotation;p=tf.transform.translation;a=yaw(q)
    c,s=math.cos(a),math.sin(a);origin=grid.info.origin; oa=yaw(origin.orientation)
    oc,os_=math.cos(oa),math.sin(oa);res=grid.info.resolution
    width,height=grid.info.width,grid.info.height
    samples=[]
    # Disk encloses the larger configured 0.32 x 0.18 m half-footprint.
    n=math.ceil(.40/res)
    offsets=[(i,j) for i in range(-n,n+1) for j in range(-n,n+1) if math.hypot(i,j)*res <= .40+res*.5]
    checked=unknown=occupied=0;violations=[]
    for pt in reference['points'][::3]+reference['points'][-1:]:
        x=p.x+c*pt['x']-s*pt['y'];y=p.y+s*pt['x']+c*pt['y']
        dx,dy=x-origin.position.x,y-origin.position.y
        ix=math.floor((oc*dx+os_*dy)/res);iy=math.floor((-os_*dx+oc*dy)/res)
        for ox,oy in offsets:
            gx,gy=ix+ox,iy+oy;checked+=1
            value=-1 if gx<0 or gy<0 or gx>=width or gy>=height else grid.data[gy*width+gx]
            if value < 0 or value >= 65:
                unknown+=int(value<0);occupied+=int(value>=65)
                if len(violations)<25:violations.append(dict(s_m=pt['s'],x=x,y=y,value=int(value)))
    return dict(clear=not (unknown or occupied),checked_cells=checked,unknown_cells=unknown,
                occupied_cells=occupied,corridor_radius_m=.40,first_violations=violations)


def expected_completion_stop(reason, action_status):
    return reason == 'Driver stop was activated during preparation or driving' and action_status == 4


def request_stop(halt, reason):
    # A later driver latch must not replace a user's stop/signal: only the driver
    # reason is eligible for the goal-completion exception below.
    if halt['reason'] is None or halt['reason']=='Driver stop was activated during preparation or driving':
        halt['reason']=reason


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--preflight-only',action='store_true')
    ap.add_argument('--live-slam',action='store_true',help='Start a fresh SLAM map and navigation')
    ap.add_argument('--saved-route',help='Ordered marked-route JSON on a saved map; shared exploration planner/controller')
    ap.add_argument('--relocalize',action='store_true',help='Explicitly reset saved-map localization to the route prior; otherwise retain healthy localization')
    ap.add_argument('--explore',action='store_true',help='Keep selecting known-free open-space exploration waypoints as the map grows; implies --live-slam')
    ap.add_argument('--allow-unknown-map',action='store_true',
                    help='Legacy recorded-path compatibility flag; live planning follows GridBased.allow_unknown. Does not override obstacle checks.')
    ap.add_argument('--timeout',type=float,default=120)
    ap.add_argument('--countdown',type=float,default=1.0)
    ap.add_argument('--stop-stack',action='store_true',help='Stop the background stack owned by this tool')
    ap.add_argument('--restart-stack',action='store_true',help='Rebuild the owned stack before this run')
    from startup_policy import add_shutdown_options, restart_reason
    add_shutdown_options(ap)
    args=ap.parse_args()
    # Keep spawned selector workers free of ROS imports and node state.
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import OccupancyGrid, Path as RosPath
    from nav2_msgs.action import FollowPath, NavigateToPose, NavigateThroughPoses
    from std_msgs.msg import Bool, String
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    from session_localization import SessionLocalization, StationaryGate, PoseJumpGuard
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from std_srvs.srv import Trigger
    import yaml
    from tracking_core import anchor, project
    from record_tracking import clean
    from stack_manager import load_stack, start_stack, configuration_fingerprint
    from session_stop_reports import export_stop_reports
    from frontier_exploration import EXPLORATION_CHECK_PERIOD_S, EXPLORATION_MIN_GOAL_INTERVAL_S, select_frontiers_job
    from rolling_goal_policy import choose_rolling_goal, rolling_distances
    from rolling_goal_tracker import RollingGoalTracker
    from session_scan_monitor import SessionScanMonitor
    from async_session_log import AsyncSessionLog
    route=None
    if args.saved_route:
        if args.explore:ap.error('--saved-route and --explore are mutually exclusive')
        from saved_route_reference import load_route
        route=load_route(args.saved_route)
        args.live_slam=True
    if args.explore:args.live_slam=True
    if args.relocalize and route is None:ap.error('--relocalize requires --saved-route')
    fresh_mapping=args.live_slam and route is None
    if not 10 <= args.timeout <= 180:ap.error('timeout must be 10..180 s')
    if not 0 <= args.countdown <= 30:ap.error('countdown must be 0..30 s')
    root=Path(__file__).resolve().parent
    brake_config=yaml.safe_load((Path.home()/'racecar/src/racecar/config/driver_calibration.yaml').read_text())['racecar_driver']['ros__parameters']
    if not args.stop_stack and not args.preflight_only and not brake_config.get('reverse_brake_enabled',False):
        raise SystemExit('Automatic run blocked: reverse-brake PWM and duration are not yet calibrated/enabled. No nodes started.')
    lock=(root/'.run.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise SystemExit('An S-curve session is already running; finish it or press Ctrl+C before starting another')
    session_started=time.monotonic()
    run=root/'runs'/datetime.now().strftime(('marked-route-' if route else 'live-slam-' if args.live_slam else 's-curve-')+'%Y%m%d-%H%M%S-%f')
    run.mkdir(parents=True);(root/'latest_run.txt').write_text(str(run))
    if not args.stop_stack and not args.live_slam:
        from start_live_view import start as start_live_view
        start_live_view(root)
    state={'run':str(run),'stage':'starting','goal_sent':False,'parameters_changed':False,
           'allow_unknown_map':args.allow_unknown_map}
    session_log=None;scan_monitor=None
    event_sequence=0;terminal_event=None
    loop_timing={'last_interval_ms':None,'max_interval_ms':0.,'iterations':0,
                 'last_phase_ms':{},'max_phase_ms':{}}
    last_iteration=None
    def phase_elapsed(name,begin):
        elapsed_ms=(time.monotonic()-begin)*1000
        loop_timing['last_phase_ms'][name]=elapsed_ms
        loop_timing['max_phase_ms'][name]=max(elapsed_ms,loop_timing['max_phase_ms'].get(name,0.))
    def event(stage,**extra):
        nonlocal event_sequence,terminal_event
        elapsed=time.monotonic()-session_started
        state.update(stage=stage,session_elapsed_s=elapsed,**extra)
        event_sequence+=1
        row=clean({'stage':stage,'session_elapsed_s':elapsed,'event_sequence':event_sequence,**extra})
        if stage in ('stopped','failed_preflight','succeeded','rolling_horizon_exhausted',
                     'preflight_passed','stack_stopped') or stage.startswith('action_ended_'):
            terminal_event=row
        # Copy/enqueue only. JSON encoding, disk and console I/O run in a
        # bounded helper process, never in a sensor/navigation callback.
        if session_log is not None:
            try:session_log.event(row,clean(state))
            except Exception as exc:state['session_log_enqueue_error']=str(exc)
    def ensure_scan_fresh():
        diagnostic=scan_monitor.snapshot()
        if diagnostic.get('monitor_error') or diagnostic.get('thread_alive') is False:
            state['scan_health_at_failure']=scan_monitor.snapshot(include_recent=True)
            raise RuntimeError('Laser monitor failed: '+str(diagnostic.get('monitor_error') or 'receiver thread exited'))
        if diagnostic['latched_timeout'] or not diagnostic['healthy']:
            state['scan_health_at_failure']=scan_monitor.snapshot(include_recent=True)
            raise RuntimeError('Laser data lost')
        return diagnostic
    rclpy.init()
    node=rclpy.create_node('s_curve_session')
    from session_executor import SessionExecutor
    rolling_status_node=rclpy.create_node('s_curve_rolling_feedback',
        start_parameter_services=False,enable_rosout=False) if args.explore else None
    session_executor=SessionExecutor(node,rolling_status_node)
    localization=SessionLocalization(brake_config,saved_map=route is not None)
    buffer=localization.buffer
    best=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    transient=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    observed={};stack=None;recorder=None;handle=None;goal_future=None;started=None
    saved_costmap_sync=None
    halt={'reason':None}
    def interrupt(sig,frame):request_stop(halt,'signal_'+str(sig))
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupt)
    def capture(key,msg):observed[key]=(time.monotonic(),msg)
    stop_monitor={'watch':False}
    stop_events=[];stop_ids=set();stop_phase={'value':'preparation'}
    session_ros_started=node.get_clock().now().nanoseconds*1e-9
    def capture_stop_event(msg):
        try:
            entry=json.loads(msg.data)
            if entry['id'] in stop_ids or entry['stamp']<session_ros_started:return
            stop_ids.add(entry['id']);stop_events.append({'event':entry,'phase':stop_phase['value']})
        except (ValueError,KeyError,TypeError):pass

    def capture_stop(msg):
        capture('estop',msg)
        if msg.data and stop_monitor['watch']:
            request_stop(halt,'Driver stop was activated during preparation or driving')
    def external_stop(msg):
        if msg.data:request_stop(halt,'External emergency stop received during this session')
    subscriptions=[node.create_subscription(OccupancyGrid,'/map',lambda m:capture('map',m),transient),
                   node.create_subscription(Twist,'/teleop_cmd_vel',lambda m:capture('teleop',m),best),
                   node.create_subscription(Twist,'/car_cmd_vel',lambda m:capture('car_command',m),best),
                   node.create_subscription(Twist,'/cmd_vel_nav',lambda m:capture('nav_command',m),best),
                   node.create_subscription(GoalStatusArray,'/follow_path/_action/status',lambda m:capture('goals',m),transient),
                   node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',lambda m:capture('navigation_goals',m),transient),
                   node.create_subscription(GoalStatusArray,'/navigate_through_poses/_action/status',lambda m:capture('through_goals',m),transient),
                   node.create_subscription(Bool,'/emergency_stop',external_stop,10),
                   node.create_subscription(Bool,'/racecar_driver/emergency_stop_active',capture_stop,transient)]
    subscriptions.append(node.create_subscription(String,'/racecar_driver/closed_loop_state',
        lambda m:capture('driver_state',m),best))
    subscriptions.append(node.create_subscription(String,'/racecar_driver/stop_event',capture_stop_event,
        QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    emergency_pub=node.create_publisher(Bool,'/emergency_stop',10)
    stop_pub=node.create_publisher(Twist,'/car_cmd_vel',1)
    client=(ActionClient(node,NavigateThroughPoses,'/navigate_through_poses') if route else
            ActionClient(node,NavigateToPose,'/navigate_to_pose') if args.live_slam else ActionClient(node,FollowPath,'/follow_path'))
    from async_plan_log import AsyncPlanLog
    plan_log=AsyncPlanLog(run/'navigation_plans.jsonl') if args.live_slam else None
    def capture_plan(msg):
        if not args.live_slam or not state['goal_sent']:return
        begin=time.monotonic()
        try:
            received=node.get_clock().now().nanoseconds*1e-9
            plan_log.capture_serialized(msg,received)
        except (ValueError,OSError) as exc:
            state['plan_recording_error']=str(exc)
        finally:phase_elapsed('plan_callback',begin)
    if args.live_slam:
        subscriptions.append(node.create_subscription(RosPath,'/plan',capture_plan,best,raw=True))
    def spin_for(duration,cleanup=False):
        end=time.monotonic()+duration
        while time.monotonic()<end and (cleanup or not halt['reason']):session_executor.spin_once(timeout_sec=.02)
    def await_future(future,timeout=5,cleanup=False):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end and (cleanup or not halt['reason']):session_executor.spin_once(timeout_sec=.02)
        if not future.done():raise RuntimeError('ROS request timed out or interrupted')
        return future.result()
    def wait_until(predicate,timeout,description,cleanup=False):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if halt['reason'] and not cleanup:raise RuntimeError(halt['reason'])
            if stack and stack.poll() is not None:raise RuntimeError('Owned stack exited: '+description)
            if predicate():return
            spin_for(.04,cleanup=cleanup)
        raise RuntimeError('Timed out waiting for '+description)
    def trigger(service,cleanup=False):
        c=node.create_client(Trigger,service)
        try:
            if not c.wait_for_service(timeout_sec=4):raise RuntimeError('Service unavailable: '+service)
            answer=await_future(c.call_async(Trigger.Request()),cleanup=cleanup)
            if not answer or not answer.success:
                raise RuntimeError(service+': '+(answer.message if answer else 'empty response'))
            return answer
        finally:node.destroy_client(c)
    def verify_command_sources():
        allowed={'/teleop_cmd_vel':set(),'/cmd_vel_nav':{'controller_server'},
                 '/car_cmd_vel':{'velocity_smoother','s_curve_session'}}
        for topic,names in allowed.items():
            unexpected={p.node_name for p in node.get_publishers_info_by_topic(topic)}-names
            if unexpected:raise RuntimeError('Other command publisher on '+topic+': '+', '.join(sorted(unexpected)))
    def quiet():
        now=time.monotonic()
        return all(now-observed.get(key,(0,None))[0]>.65
                   for key in ('teleop','car_command','nav_command'))
    def active_goals():
        return any(s.status in (1,2,3) for key in ('goals','navigation_goals','through_goals')
                   for s in observed.get(key,(0,GoalStatusArray()))[1].status_list)
    def normal_stop_started():
        received,msg=observed.get('driver_state',(0,None))
        if msg is None or time.monotonic()-received>.3:return False
        try:driver=json.loads(msg.data)
        except (ValueError,TypeError):return False
        return (not driver.get('emergency_stopped',True) and
                (driver.get('normal_braking',False) or normal_stop_complete()))
    def normal_stop_complete():
        received,msg=observed.get('driver_state',(0,None))
        if msg is None or time.monotonic()-received>.3:return False
        try:driver=json.loads(msg.data)
        except (ValueError,TypeError):return False
        return (not driver.get('active',True) and not driver.get('normal_braking',True)
                and driver.get('stop_phase') not in ('braking','decelerating')
                and driver.get('last_written_motor_pwm')==brake_config['motor_neutral_pwm'])
    def quiesce(cleanup=False,normal_stop=False):
        stop_monitor['watch']=False
        # Preparation still uses a latched stop. Routine completion merges with
        # normal braking and waits for its pulse; it must not escalate to estop.
        trigger('/racecar_driver/normal_stop' if normal_stop else '/racecar_driver/emergency_stop',cleanup=cleanup)
        # Cancel the BT parent first, so it cannot dispatch another child path.
        from startup_policy import cancellation_actions
        for action_name in cancellation_actions(stack.record):
            c=node.create_client(CancelGoal,action_name+'/_action/cancel_goal')
            try:
                if not c.wait_for_service(timeout_sec=1 if action_name=='/navigate_to_pose' else 3):
                    if action_name=='/navigate_to_pose':continue # Older/FollowPath-only stack.
                    raise RuntimeError(action_name+' cancellation service unavailable')
                answer=await_future(c.call_async(CancelGoal.Request()),cleanup=cleanup)
                if answer.return_code != 0:raise RuntimeError(action_name+' cancellation was refused')
            finally:node.destroy_client(c)
        # Wait for the smoother's trailing zero commands to expire, including
        # at least a complete driver command-timeout interval after cancellation.
        quiet_started=time.monotonic()
        wait_until(lambda: time.monotonic()-quiet_started>.65 and quiet() and not active_goals()
                   and (normal_stop_complete() if normal_stop else observed.get('estop',(0,Bool(data=False)))[1].data),
                   6,'cancelled goals, quiet commands and '+('completed normal stop' if normal_stop else 'latched stop'),cleanup=cleanup)
    def pose():
        return localization.pose()
    def stop_owned_stack():
        # A dead child may no longer answer cancellation. Finish terminating
        # every verified owned process before permitting a replacement stack.
        try:quiesce(cleanup=True)
        except Exception as exc:
            event('stack_quiesce_incomplete',reason=str(exc))
            emergency_pub.publish(Bool(data=True));spin_for(.15,cleanup=True)
        stack.stop()
    def params(target,names,timeout=5):
        c=node.create_client(GetParameters,target+'/get_parameters')
        if not c.wait_for_service(timeout_sec=3):raise RuntimeError('Parameter service unavailable: '+target)
        answer=await_future(c.call_async(GetParameters.Request(names=names)),timeout=timeout)
        fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',
                6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
        result={}
        for name,v in zip(names,answer.values):
            item=getattr(v,fields[v.type]) if v.type in fields else None
            result[name]=list(item) if v.type in (6,7,8,9) else item
        node.destroy_client(c);return result
    def check_startup_abort():
        if halt['reason']:raise RuntimeError(halt['reason'])
        if stack and stack.poll() is not None:
            raise RuntimeError('Vehicle launch exited during startup')

    def wait_saved_map_stable(description):
        # The independent monitor keeps map TF / AMCL / scan evidence alive
        # throughout service calls and the potentially long planner search.
        begin=time.monotonic();last_notice=-math.inf
        while True:
            check_startup_abort()
            local_ready,stationary_diagnostic=localization.stationary_readiness_snapshot()
            global_state=localization.saved_map_snapshot()
            state['saved_map_localization']=global_state
            if global_state['ready'] and local_ready:return global_state
            now=time.monotonic()
            if now-last_notice>=2.:
                event('waiting_saved_map_stability',description=description,
                      elapsed_s=now-begin,global_localization=global_state,
                      stationary_check=stationary_diagnostic)
                last_notice=now
            if now-begin>=75.:
                raise RuntimeError('Saved-map localization did not establish a stable scan-matched pose: '
                    +str(global_state.get('reason'))+'; local='+str(gate.diagnostic.get('stationary_reason')))
            spin_for(.04)

    def prepare_marked_plan(reason):
        from saved_route_reference import plan_route_reference
        from saved_startup_consistency import epoch_change
        from saved_costmap_sync import CostmapLocalizationChanged
        attempt=0;begin=time.monotonic()
        while True:
            before=wait_saved_map_stable('before saved-map route preparation')
            attempt+=1
            try:
                rebuilt=saved_costmap_sync.refresh(reason=reason)
                rebuilt_epoch=dict(before,map_from_odom=rebuilt['map_from_odom'])
                after_rebuild=wait_saved_map_stable('after obstacle-layer rebuild')
                if epoch_change(rebuilt_epoch,after_rebuild)['changed']:
                    raise CostmapLocalizationChanged('Global localization changed after rebuilding the obstacle layer')
                before=after_rebuild
                start=pose()
                template,plan=plan_route_reference(node,buffer,observed,await_future,run,
                    nav_source,start,route,pose_wait=spin_for,check_abort=check_startup_abort,
                    on_pose_wait=lambda diagnostic:event('waiting_marked_route_pose',pose_wait=diagnostic,
                        note='Waiting at latched stop for fresh TF; no driving goal sent'))
            except Exception as exc:
                current=localization.saved_map_snapshot()
                changed=epoch_change(before,current)
                if not isinstance(exc,CostmapLocalizationChanged) and not changed['changed']:
                    raise
                event('saved_map_plan_invalidated',attempt=attempt,reason=str(exc),epoch_change=changed,
                      note='Driver remains stopped; rebuild after global localization stabilizes')
            else:
                after=wait_saved_map_stable('after saved-map route planning')
                changed=epoch_change(before,after)
                if not changed['changed']:
                    plan['localization_epoch']=after
                    (run/'live_plan.json').write_text(json.dumps(plan,indent=2))
                    return template,plan,start
                event('saved_map_plan_invalidated',attempt=attempt,epoch_change=changed,
                      note='Discard odom template from the old map transform; map goals stay fixed')
            if time.monotonic()-begin>=180.:
                raise RuntimeError('Global localization kept changing during route preparation; '
                                   'no consistent map start could be established')
            reason='global localization changed during preparation'

    def start_lifecycle_manager(target):
        from nav2_msgs.srv import ManageLifecycleNodes
        from startup_wait import wait_for_response
        service=target+'/manage_nodes'
        c=node.create_client(ManageLifecycleNodes,service)
        begin=time.monotonic()
        def request():
            return ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
        try:
            answer=wait_for_response(c,request,
                lambda dt:session_executor.spin_once(timeout_sec=dt),check_startup_abort,
                lambda status,elapsed:event('startup_stage',node=target,status=status,elapsed_s=round(elapsed,2)),
                timeout=60.,description=service)
            if not answer or not answer.success:
                raise RuntimeError(target+': lifecycle initialization failed; inspect stack.log')
            event('startup_stage_ready',node=target,elapsed_s=time.monotonic()-begin)
        finally:node.destroy_client(c)

    def lifecycle(target):
        from startup_wait import wait_for_active
        c=node.create_client(GetState,target+'/get_state')
        def check_abort():
            if halt['reason']:raise RuntimeError(halt['reason'])
            if stack and stack.poll() is not None:
                raise RuntimeError('Vehicle launch exited during activation: '+target)
        try:
            wait_for_active(c,GetState.Request,
                lambda dt:session_executor.spin_once(timeout_sec=dt),check_abort,
                lambda last,elapsed:event('waiting_lifecycle',node=target,
                    last_state=last,elapsed_s=round(elapsed,1)),timeout=60.)
        except RuntimeError as exc:
            raise RuntimeError(target+': '+str(exc)) from exc
        finally:node.destroy_client(c)
    reference=None;outcome='failed_preflight';stack_offset=0;runtime_ready=False
    exploration_goal=None;exploration_generation=0;exploration_changed=0.;exploration_poll=0.
    exploration_last_map=None;exploration_diagnostic={};exploration_visited=[]
    frontier_worker=ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn')) if args.explore else None
    frontier_future=None;frontier_submitted=0.;frontier_compute_ms=0.;frontier_roundtrip_ms=0.
    rolling_message=None
    rolling_tracker=RollingGoalTracker() if args.explore else None
    rolling_feedback_enabled=False;pending_generation=0
    def capture_rolling_status(msg):
        nonlocal exploration_goal, rolling_message
        if not rolling_feedback_enabled:return
        try:report=json.loads(msg.data)
        except (ValueError,TypeError):return
        received_ros_s=node.get_clock().now().nanoseconds*1e-9
        pending=rolling_tracker.pending
        request_to_ack_ms=1000*(time.monotonic()-pending.submitted_at) if pending else None
        resolution=rolling_tracker.handle_ack(report)
        if not resolution['matched']:return
        if resolution['status']=='accepted':
            exploration_goal=list(rolling_tracker.accepted_goal)
        rolling_message=None
        event('exploration_goal_'+resolution['status'],generation=pending_generation,
              planner_report=report,accepted_goal=rolling_tracker.accepted_goal,
              ack_received_ros_s=received_ros_s,request_to_ack_ms=request_to_ack_ms,
              report_transport_ms=(received_ros_s-report['reported_ros_s'])*1000
                  if isinstance(report.get('reported_ros_s'),(int,float)) else None,
              note='BT committed the planned path and goal' if resolution['status']=='accepted'
                   else 'Candidate planning failed; the accepted goal remains unchanged')
    if args.explore:
        subscriptions.append(rolling_status_node.create_subscription(String,'/live_slam/rolling_goal_status',
            capture_rolling_status,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    rolling_pub=node.create_publisher(PoseStamped,'/live_slam/rolling_goal',1) if args.explore else None
    # System-default GoalUpdater subscription is volatile. Repeat only the latest
    # target with its original stamp, so a late-created subscriber receives it.
    rolling_timer=node.create_timer(EXPLORATION_CHECK_PERIOD_S,
        lambda:rolling_pub.publish(rolling_message) if rolling_message is not None
        and rolling_tracker.pending is not None else None) if args.explore else None
    def frontier_request(exclude=(),current_map=None):
        from frontier_exploration import grid_from_message
        from planner_costmap_audit import current_pose,padded_footprint
        if 'map' not in observed or time.monotonic()-observed['map'][0]>3:
            return None,{'reason':'map_unavailable_or_stale'}
        msg=observed['map'][1]
        if current_map is None:
            current_map=current_pose(node,buffer,msg.header.frame_id,source['feedback_base_frame'])
        cfg=nav_source['global_costmap']['global_costmap']['ros__parameters']
        footprint=cfg['footprint']
        if isinstance(footprint,str):footprint=yaml.safe_load(footprint)
        clearance=max(math.hypot(*pt) for pt in padded_footprint(footprint,float(cfg.get('footprint_padding',.01))))+.03
        minimum,preferred=rolling_distances(source['speed_setpoint_mps'])
        return (grid_from_message(msg),current_map,dict(clearance_m=clearance,
            min_goal_m=minimum,step_m=preferred,exclude=list(exclude),separation_m=.35,
            continuation_goal=rolling_tracker.anchor_goal if rolling_tracker else None)),{'pose_map':current_map}
    def frontier_options(exclude=()):
        request,diagnostic=frontier_request(exclude)
        if request is None:return [],diagnostic
        # Spawn/import/warm the worker while the vehicle is still stopped.
        future=frontier_worker.submit(select_frontiers_job,request)
        wait_until(future.done,15,'initial open-space exploration selection')
        return future.result()
    def navigation_goal(goal_map,frame):
        goal=NavigateToPose.Goal();goal.pose.header.frame_id=frame
        if args.explore:
            from ament_index_python.packages import get_package_share_directory
            goal.behavior_tree=str(Path(get_package_share_directory('racecar'))/'config/navigate_live_rolling.xml')
        goal.pose.header.stamp=node.get_clock().now().to_msg()
        gx,gy,ga=goal_map;goal.pose.pose.position.x=gx;goal.pose.pose.position.y=gy
        goal.pose.pose.orientation.z=math.sin(ga/2);goal.pose.pose.orientation.w=math.cos(ga/2)
        return goal
    recorderlog=(run/'recorder.log').open('w')
    try:
        session_log=AsyncSessionLog(run,event_capacity=64,bulk_capacity=32)
        scan_monitor=SessionScanMonitor(timeout_s=1.,
            on_timeout=lambda diagnostic:request_stop(halt,'Laser data lost'))
        stack=load_stack(root)
        if stack:stack_offset=stack.log_path.stat().st_size
        if args.stop_stack:
            if stack:
                stop_owned_stack();stack=None
            outcome='stack_stopped';event(outcome);return 0
        fingerprint=configuration_fingerprint(Path.home()/'racecar',saved_map=route['map_yaml'] if route else None)
        restart=(restart_reason(stack.record,fingerprint,follow_path_only=not args.live_slam,
                    saved_map=route['map_yaml'] if route else None,fresh_mapping=fresh_mapping,
                    explicit=args.restart_stack) if stack else None)
        if stack and not restart:restart=stack.health_reason()
        if restart:
            event('restarting_stack',reason=restart)
            stop_owned_stack();stack=None
        reused=stack is not None
        check_startup_abort()
        if stack is None:
            event('starting_stack',note='Saved map localization plus shared planner; no goal sent' if route else 'Fresh SLAM plus planner; no goal sent' if args.live_slam else 'No navigation goal has been sent; FollowPath-only stack')
            stack=start_stack(root,fingerprint,follow_path_only=not args.live_slam,saved_map=route['map_yaml'] if route else None);stack_offset=0
        else:
            event('reusing_stack',note='Reuse sensors, localization and active controller; no goal sent')
            if route:
                from stack_manager import reopen_rviz
                try:reopen_rviz(root,root/'marked_route_run.rviz')
                except OSError as exc:event('rviz_open_warning',reason=str(exc))
        stack.mark_idle(False)
        state.update(stack_reused=reused,stack_log=str(stack.log_path))
        (run/'stack_source.json').write_text(json.dumps({'path':str(stack.log_path),'offset':stack_offset}))
        if route:
            from saved_route_reference import initialize_localization
            from startup_policy import prepare_saved_map
            def sensors_ready():
                event('waiting_local_odom',note='Prepare continuous odom and laser before AMCL initialization')
                wait_until(lambda:localization.ready() and scan_monitor.ready(),60,
                           'continuous local odometry and laser before AMCL')
            def saved_localize():
                result=initialize_localization(node,buffer,await_future,wait_until,spin_for,trigger,event,
                    run,route,lifecycle,reused=reused,force_reinitialize=args.relocalize,monitor=localization)
                state['saved_map_initialization']=result
            prepare_saved_map(reused=reused,stop_previous=quiesce,wait_sensors=sensors_ready,
                start_localization=lambda:start_lifecycle_manager('/lifecycle_manager_localization'),
                localize=saved_localize,
                start_navigation=lambda:start_lifecycle_manager('/lifecycle_manager_navigation'))
            (run/'marked_route.json').write_text(json.dumps(route,indent=2))
        elif not reused:
            event('waiting_slam_ready',note='Initialize navigation after fresh odometry, laser, map and map-to-base TF')
            wait_until(lambda:localization.ready() and scan_monitor.ready() and 'map' in observed
                       and buffer.can_transform(observed['map'][1].header.frame_id,
                                                brake_config.get('feedback_base_frame','base_footprint'),rclpy.time.Time()),
                       60,'SLAM map, continuous odometry, laser and TF before navigation')
            start_lifecycle_manager('/lifecycle_manager_navigation')
        end=time.monotonic()+55;ready=False
        while time.monotonic()<end and not halt['reason']:
            spin_for(.2)
            if stack.poll() is not None:raise RuntimeError('Vehicle launch exited; inspect stack.log')
            ready=False
            try:
                pose()
                ready=(localization.ready() and client.server_is_ready() and 'map' in observed
                       and scan_monitor.ready())
                if ready:break
            except Exception:pass
        if not ready:raise RuntimeError('Localization/map/laser/navigation action did not become ready')
        event('sensors_localization_ready')
        lifecycle('/controller_server');lifecycle('/velocity_smoother')
        if args.live_slam:
            lifecycle('/planner_server');lifecycle('/bt_navigator')
        event('navigation_nodes_ready')
        stack.remember_ready_children()
        runtime_ready=True
        # A new invocation explicitly starts a new run. Clear abandoned goals now;
        # release the driver stop only once, immediately before sending this goal.
        quiesce()
        event('previous_commands_cleared')
        if args.live_slam:
            from saved_route_reference import configure_terminal_reference
            reference=configure_terminal_reference(node,await_future,route)
            event('terminal_approach_preference',reference=reference,hard_waypoint=False)
        if route:
            from saved_costmap_sync import SavedCostmapSync
            saved_costmap_sync=SavedCostmapSync(node,scan_monitor,localization,buffer,event,
                                               await_future,wait_until,run)
        verify_command_sources()
        stop_monitor['watch']=True
        source=yaml.safe_load((Path.home()/'racecar/src/racecar/config/driver_calibration.yaml').read_text())['racecar_driver']['ros__parameters']
        names=list(source)
        effective=params('/racecar_driver',names)
        for key,value in source.items():
            if effective.get(key)!=value:raise RuntimeError('Driver effective/source parameter mismatch: '+key)
        smoother=params('/velocity_smoother',['max_velocity','min_velocity','max_accel','max_decel','smoothing_frequency'])
        if smoother['max_velocity'][2]!=math.inf or smoother['min_velocity'][2]!=-math.inf:
            raise RuntimeError('Angular-speed limit removal has not taken effect')
        controller_names=['FollowPath.'+key for key in ('plugin','multipoint_enabled',
            'preview_spacing_m','progress_search_ahead_m','tracking_mode','lookahead_dist',
            'min_lookahead_dist','max_lookahead_dist','use_velocity_scaled_lookahead_dist','desired_linear_vel',
            'use_collision_detection','min_approach_linear_velocity',
            'use_regulated_linear_velocity_scaling','use_cost_regulated_linear_velocity_scaling')]
        controller_names.extend(['controller_frequency','goal_checker.plugin'])
        ctrl=params('/controller_server',controller_names)
        nav_source=yaml.safe_load((Path.home()/'racecar/src/racecar/config/nav_carto.yaml').read_text())
        for key,value in smoother.items():
            if value!=nav_source['velocity_smoother']['ros__parameters'][key]:
                raise RuntimeError('Smoother effective/source parameter mismatch: '+key)
        for key,value in ctrl.items():
            expected_ctrl=nav_source['controller_server']['ros__parameters']
            expected_value=(expected_ctrl[key.split('.',1)[0]][key.split('.',1)[1]]
                            if '.' in key else expected_ctrl[key])
            if value!=expected_value:
                raise RuntimeError('Controller effective/source parameter mismatch: '+key)
        event('runtime_verified',effective_driver=effective,effective_smoother=smoother,effective_controller=ctrl)
        event('protection_policy',map_boundary_guard=effective['map_boundary_guard_enabled'],
              runtime_collision_detection=ctrl['FollowPath.use_collision_detection'],
              planner_allow_unknown=nav_source['planner_server']['ros__parameters']['GridBased']['allow_unknown'] if args.live_slam else None,
              note='Boundary latch and collision checking are independent; live start uses the planner costmap and footprint')
        if args.live_slam:
            from ament_index_python.packages import get_package_share_directory
            from racecar.planning_geometry import startup_radius
            required_radius=startup_radius(get_package_share_directory('racecar'))
            radius_readback={}
            def execution_radius_ready():
                radius_readback.update(params('/planner_server',['GridBased.minimum_turning_radius'],timeout=20))
                actual=radius_readback.get('GridBased.minimum_turning_radius')
                return isinstance(actual,(int,float)) and math.isfinite(actual) and abs(actual-required_radius)<=1e-6
            wait_until(execution_radius_ready,25,'configured geometric minimum turning radius')
            event('planning_radius_verified',required_radius_m=required_radius,actual_radius_m=radius_readback['GridBased.minimum_turning_radius'],
                  execution_speed_mps=source['speed_setpoint_mps'])
        # Replace fixed settling sleeps with a short observed stable-pose window.
        stable=StationaryGate()
        last_wait_log=[-math.inf]
        def stationary():
            current,diagnostic=localization.readiness_snapshot()
            now=time.monotonic()
            ready=stable.update(current,diagnostic,now)
            state['stationary_check']=stable.diagnostic
            if not ready and now-last_wait_log[0]>=1.:
                event('waiting_localization',stationary_check=stable.diagnostic)
                last_wait_log[0]=now
            return ready
        if route:wait_saved_map_stable('saved-map route start')
        else:wait_until(stationary,8,'stationary fresh localization')
        event('localization_stable')
        x,y,a=pose()
        if args.live_slam:
            from live_slam_reference import plan_reference
            if route:
                template,planning,(x,y,a)=prepare_marked_plan('new saved-route session after localization')
            elif args.explore:
                choices,exploration_diagnostic=frontier_options()
                event('exploration_candidates',diagnostic=exploration_diagnostic,candidates=choices)
                if not choices:raise RuntimeError('No eligible known-free open-space waypoint: '+exploration_diagnostic['reason'])
                initial_pose=exploration_diagnostic['pose_map']
                first=choose_rolling_goal(choices,initial_pose,None,source['speed_setpoint_mps'])
                if first is None:raise RuntimeError('No forward exploration goal with sufficient distance')
                choices=[first]+[p for p in choices if p['goal']!=first['goal'] and choose_rolling_goal([p],initial_pose,None,source['speed_setpoint_mps']) is not None]
                candidates=[(p['distance_m'],*p['goal']) for p in choices]
                template,planning=plan_reference(node,buffer,observed,await_future,run,nav_source,(x,y,a),
                    goal_candidates=candidates,candidate_frame=observed['map'][1].header.frame_id)
                exploration_goal=planning['goal_map'];exploration_generation=1
            else:
                template,planning=plan_reference(node,buffer,observed,await_future,run,nav_source,(x,y,a))
            event('live_path_planned',goal_map=planning['goal_map'],length_m=planning['length_m'],
                  mode='saved-map ordered marked targets' if route else 'continuous open-space exploration' if args.explore else 'current live map and planner; no prerecorded path')
        else:template=json.loads((root/'reference_local.json').read_text())
        reference=anchor(template,x,y,a,'odom')
        (run/'reference_world.json').write_text(json.dumps(reference,indent=2))
        (run/'reference_local.json').write_text(json.dumps(template,indent=2))
        map_tf=buffer.lookup_transform(observed['map'][1].header.frame_id,'odom',rclpy.time.Time())
        if args.live_slam:
            # plan_reference audited the planner's costmap, configured footprint
            # and unknown policy. Recheck that same policy immediately before rearm.
            event('path_checked',method='planner costmap and footprint',clear=True)
        else:
            check=check_map(reference,observed['map'][1],map_tf)
            event('reference_map_diagnostic',path_check=check,
                  note='Legacy raw map corridor is diagnostic only; runtime collision checking remains independent')
        # Save a compact occupancy grid for later analysis.
        g=observed['map'][1]
        (run/'map_snapshot.json').write_text(json.dumps({'frame':g.header.frame_id,'width':g.info.width,
            'height':g.info.height,'resolution':g.info.resolution,'origin':[g.info.origin.position.x,g.info.origin.position.y,yaw(g.info.origin.orientation)],
            'data':list(g.data),'map_from_odom':[map_tf.transform.translation.x,map_tf.transform.translation.y,yaw(map_tf.transform.rotation)]}))
        # Do not turn a raw-map visualization audit into a separate boundary
        # latch when runtime collision detection is enabled. Live planning and
        # startup share a costmap audit; recorded templates use the controller's
        # live obstacle costmap during execution.
        recorder=subprocess.Popen([sys.executable,str(root/'record_tracking.py'),'--output',str(run/'tracking'),
            '--reference-world',str(run/'reference_world.json'),'--duration',str(args.timeout+25)]
            + (['--record-map-tf'] if route else [])
            + (['--publish-actual-path','--actual-frame','map' if route else 'odom'] if args.live_slam else []),
            stdout=recorderlog,stderr=subprocess.STDOUT,start_new_session=True)
        def recorder_ready():
            if recorder.poll() is not None:raise RuntimeError('Recorder exited before run')
            ready_file=run/'tracking/ready.json'
            return ready_file.exists() and json.loads(ready_file.read_text()).get('ready',False)
        wait_until(recorder_ready,15,'recorder subscriptions and fresh recorded poses')
        event('recorder_ready')
        if args.live_slam:
            (run/'tracking/reference_scope.json').write_text(json.dumps({
                'reference_world_role':'initial plan only; rerouting is expected',
                'current_plan_history':'../navigation_plans.jsonl',
                'current_tracking_samples':'../current_plan_tracking.jsonl',
                'static_analysis_warning':'Static reference error includes deliberate obstacle detours; use current_plan_tracking for rerouted tracking error'},indent=2))
        if args.preflight_only:
            outcome='preflight_passed';event(outcome);return 0
        if halt['reason']:raise RuntimeError(halt['reason'])
        print(f"Path ({reference['length_m']:.2f} m) starts in {args.countdown:g} seconds. Ctrl+C stops this run.",flush=True)
        spin_for(args.countdown)
        if halt['reason']:raise RuntimeError(halt['reason'])
        x,y,a=pose()
        if math.hypot(x-reference['anchor']['x'],y-reference['anchor']['y'])>.12:
            raise RuntimeError('Vehicle moved after reference was fixed')
        verify_command_sources()
        wait_until(lambda:quiet() and not active_goals(),3,'quiet command inputs before explicit rearm')
        if recorder.poll() is not None:raise RuntimeError('Recorder exited before rearm')
        if not args.live_slam:
            pathmsg=RosPath();pathmsg.header.frame_id='odom';pathmsg.header.stamp=node.get_clock().now().to_msg()
            for p in reference['points']:
                m=PoseStamped();m.header=pathmsg.header;m.pose.position.x=p['x'];m.pose.position.y=p['y']
                m.pose.orientation.z=math.sin(p['yaw']/2);m.pose.orientation.w=math.cos(p['yaw']/2);pathmsg.poses.append(m)
        if args.live_slam:
            if route:
                from saved_route_reference import navigation_goal as marked_navigation_goal
                goal=marked_navigation_goal(node,route)
                event('marked_route_mode',action='NavigateThroughPoses',ordered_goals=route['goals'],
                      map_yaml=route['map_yaml'],shared_parameters=True,replan_hz=10.,
                      note='Pass through intermediate poses; stop only at final goal. No Spin/BackUp recovery actions.')
            else:goal=navigation_goal(planning['goal_map'],planning['map_frame'])
            event('navigation_mode',action='NavigateThroughPoses' if route else 'NavigateToPose',replan_hz=10.0,
                  exploration_check_hz=1.0/EXPLORATION_CHECK_PERIOD_S if args.explore else None,
                  exploration_min_goal_interval_s=EXPLORATION_MIN_GOAL_INTERVAL_S if args.explore else None,
                  rolling_goal_distance_m=rolling_distances(source['speed_setpoint_mps']) if args.explore else None,
                  goal_update_method='ordered poses with passed-goal removal' if route else 'GoalUpdater while moving; new action after normal endpoint waiting' if args.explore else 'fixed goal',
                  rolling_goal_topic='/live_slam/rolling_goal' if args.explore else None,
                  rolling_goal_status_topic='/live_slam/rolling_goal_status' if args.explore else None,
                  goal_map=planning['goal_map'],reference_role='initial plan; current plans are recorded separately',
                  obstacle_policy='Replan while following; brake if collision imminent or no feasible plan')
        else:
            goal=FollowPath.Goal();goal.path=pathmsg;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
        # Reception runs on its own node while the large path message is built.
        # Wait at latched stop for continuously advancing, fresh localization.
        wait_until(lambda:localization.ready(),8,'continuous fresh /odom before rearm')
        current=pose()
        if math.hypot(current[0]-reference['anchor']['x'],current[1]-reference['anchor']['y'])>.12:
            raise RuntimeError('Vehicle moved before explicit rearm')
        event('localization_ready',localization=localization.snapshot()[1])
        if args.live_slam and not route:
            from planner_costmap_audit import recheck_start
            def check_start_audit_abort():
                if halt['reason']:raise RuntimeError(halt['reason'])
                if stack.poll() is not None:raise RuntimeError('Vehicle launch exited during startup path audit')
                if recorder.poll() is not None:raise RuntimeError('Recorder exited during startup path audit')
            start_check=recheck_start(node,buffer,await_future,run,planning,
                reference_odom=[(p['x'],p['y'],p['yaw']) for p in reference['points']],
                pose_wait=spin_for,check_abort=check_start_audit_abort,pose_timeout_s=2.,
                on_pose_wait=lambda diagnostic:event('waiting_start_pose',pose_wait=diagnostic,
                    note='Waiting at latched stop for fresh TF; no navigation goal sent'))
            event('start_path_checked',path_check={k:start_check.get(k) for k in
                ('clear','reason','error','first_rejection','costmap_artifact','pose_wait','costmap_age_after_pose_wait_s')})
            if not start_check['clear']:
                raise RuntimeError('Live start blocked by planner-consistent costmap audit: '+str(start_check.get('error') or start_check.get('first_rejection') or start_check.get('reason')))
            check_start_audit_abort()
            if not localization.ready():raise RuntimeError('Localization lost readiness during startup path audit')
            current=pose()
            if math.hypot(current[0]-reference['anchor']['x'],current[1]-reference['anchor']['y'])>.12:
                raise RuntimeError('Vehicle moved during startup path audit before explicit rearm')
        if route:
            from saved_startup_consistency import epoch_change
            from planner_costmap_audit import recheck_start
            # The recorder/countdown may outlast an AMCL correction. Rebuild the
            # plan at stopped state instead of auditing its old odom projection.
            final_begin=time.monotonic();final_attempt=0
            while True:
                final_attempt+=1
                before=wait_saved_map_stable('immediately before marked-route start')
                changed=epoch_change(planning['localization_epoch'],before)
                if changed['changed']:
                    event('saved_map_plan_invalidated',phase='before_rearm',epoch_change=changed,
                          note='No goal sent; refresh the plan and recorder reference at stopped state')
                    template,planning,(x,y,a)=prepare_marked_plan('global localization changed before rearm')
                    reference=anchor(template,x,y,a,'odom')
                    (run/'reference_world.json').write_text(json.dumps(reference,indent=2))
                    (run/'reference_local.json').write_text(json.dumps(template,indent=2))
                    map_tf=buffer.lookup_transform('map','odom',rclpy.time.Time())
                    snapshot=json.loads((run/'map_snapshot.json').read_text())
                    snapshot['map_from_odom']=[map_tf.transform.translation.x,map_tf.transform.translation.y,yaw(map_tf.transform.rotation)]
                    (run/'map_snapshot.json').write_text(json.dumps(snapshot))
                    stop_process(recorder)
                    (run/'tracking').rename(run/('tracking-before-replan-'+str(time.time_ns())))
                    recorder=subprocess.Popen([sys.executable,str(root/'record_tracking.py'),'--output',str(run/'tracking'),
                        '--reference-world',str(run/'reference_world.json'),'--duration',str(args.timeout+25)]
                        + (['--record-map-tf'] if route else [])
                        + ['--publish-actual-path','--actual-frame','map'],
                        stdout=recorderlog,stderr=subprocess.STDOUT,start_new_session=True)
                    wait_until(recorder_ready,15,'recorder for the corrected saved-map reference')
                    (run/'tracking/reference_scope.json').write_text(json.dumps({
                        'reference_world_role':'corrected initial map plan; earlier stopped references are archived',
                        'current_plan_history':'../navigation_plans.jsonl',
                        'current_tracking_samples':'../current_plan_tracking.jsonl'},indent=2))
                    before=wait_saved_map_stable('after corrected reference recorder is ready')
                    if time.monotonic()-final_begin>=180. and epoch_change(planning['localization_epoch'],before)['changed']:
                        raise RuntimeError('Global localization kept changing before rearm; vehicle remains stopped')
                    continue
                def check_saved_audit_abort():
                    check_startup_abort()
                    if recorder.poll() is not None:raise RuntimeError('Recorder exited during saved-map start audit')
                start_check=recheck_start(node,buffer,await_future,run,planning,
                    reference_map=planning['path_map'],pose_wait=spin_for,
                    check_abort=check_saved_audit_abort,pose_timeout_s=None,
                    on_pose_wait=lambda diagnostic:event('waiting_start_pose',pose_wait=diagnostic,
                        note='Waiting at stopped state for saved-map TF'))
                after=wait_saved_map_stable('after marked-route start audit')
                changed=epoch_change(planning['localization_epoch'],after)
                if changed['changed']:
                    event('saved_map_plan_invalidated',phase='start_audit',epoch_change=changed)
                else:
                    event('start_path_checked',path_check={k:start_check.get(k) for k in
                        ('clear','reason','error','first_rejection','costmap_artifact','pose_wait','reference_frame')})
                    if not start_check['clear']:
                        raise RuntimeError('Saved-map start blocked by current obstacle/footprint audit: '
                            +str(start_check.get('error') or start_check.get('first_rejection') or start_check.get('reason')))
                    break
                if time.monotonic()-final_begin>=180.:
                    raise RuntimeError('Global localization kept changing before rearm; vehicle remains stopped')
        # Never delete the latch file or disable the driver stop. Use its existing
        # neutral-write, brake-pulse and command-timeout checks for this new run.
        if not scan_monitor.arm():raise RuntimeError('Laser data not fresh before rearm')
        trigger('/racecar_driver/reset_emergency_stop')
        wait_until(lambda:'estop' in observed and not observed['estop'][1].data,
                   2,'driver stop reset acknowledgement')
        if halt['reason']:raise RuntimeError(halt['reason'])
        event('rearmed',note='Initial rearm for this invocation; normal endpoint continuation never resets emergency stops')
        def feedback(msg):
            details=({'distance_remaining':msg.feedback.distance_remaining,
                      'number_of_recoveries':msg.feedback.number_of_recoveries} if args.live_slam
                     else {'distance_to_goal':msg.feedback.distance_to_goal,'speed':msg.feedback.speed})
            if route:details['number_of_poses_remaining']=msg.feedback.number_of_poses_remaining
            observed['feedback']=(time.monotonic(),details)
        if not localization.ready():raise RuntimeError('Localization lost readiness before goal submission')
        ensure_scan_fresh()
        stop_phase['value']='driving'
        if args.explore:
            stamp=goal.pose.header.stamp
            rolling_tracker=RollingGoalTracker(initial_goal=exploration_goal,
                session_start_stamp=(stamp.sec,stamp.nanosec))
            if not rolling_tracker.register_request(exploration_goal,(stamp.sec,stamp.nanosec),initial=True):
                raise RuntimeError('Could not register initial exploration goal')
            pending_generation=1;rolling_feedback_enabled=True
        goal_future=client.send_goal_async(goal,feedback_callback=feedback)
        state['goal_sent']=True
        handle=await_future(goal_future)
        if not handle.accepted:raise RuntimeError('Navigation goal rejected' if args.live_slam else 'FollowPath goal rejected')
        result_future=handle.get_result_async();started=time.monotonic();last_print=0
        pose_time_tolerance=nav_source['controller_server']['ros__parameters']['FollowPath']['transform_tolerance']
        pose_tf_diagnostics={'skipped_queries':0,'last_error':None,'last_pose_stamp':None}
        state['pose_tf_diagnostics']=pose_tf_diagnostics
        last_pose_tf_notice=-math.inf
        def pose_in_frame(frame):
            nonlocal last_pose_tf_notice
            # Reuse one transform per frame within this loop iteration: goal
            # selection, displayed trajectory and tracking see the same sample.
            if frame not in pose_transforms:
                from planner_costmap_audit import transform_poses
                diagnostic={}
                try:
                    value=transform_poses(buffer,frame,'odom',[current],source_stamp=sample['stamp'],
                        transform_tolerance=pose_time_tolerance,
                        now_ros=node.get_clock().now().nanoseconds*1e-9,diagnostic=diagnostic)[0]
                except (LookupException,ConnectivityException,ExtrapolationException) as exc:
                    # This observer must not stop navigation for a missing TF.
                    # Never fabricate a pose; defer observations/new exploration
                    # targets for this sample. Controller safety remains independent.
                    value=None
                    diagnostic.update(mode='unavailable',pose_stamp=sample['stamp'],
                                      target_frame=frame,error=str(exc))
                    pose_tf_diagnostics.update(skipped_queries=pose_tf_diagnostics['skipped_queries']+1,
                                              last_error=str(exc),last_pose_stamp=sample['stamp'])
                    notice=time.monotonic()
                    if notice-last_pose_tf_notice>=2.:
                        event('pose_tf_diagnostic_skipped',pose_tf_skip=dict(diagnostic,
                              skipped_queries=pose_tf_diagnostics['skipped_queries']))
                        last_pose_tf_notice=notice
                pose_transforms[frame]=(value,diagnostic)
            return pose_transforms[frame]
        waiting_for_goal=False;waiting_since=None;completed_segments=0
        exploration_changed=started
        jump_guard=PoseJumpGuard(effective['max_speed_mps'])
        jump_guard.update(localization.snapshot()[0])
        event('driving',goal_sent=True,startup_seconds=time.monotonic()-session_started,
              jump_guard_max_speed_mps=jump_guard.max_speed_mps)
        while True:
            iteration=time.monotonic()
            if last_iteration is not None:
                interval=(iteration-last_iteration)*1000
                loop_timing['last_interval_ms']=interval
                loop_timing['max_interval_ms']=max(loop_timing['max_interval_ms'],interval)
            last_iteration=iteration;loop_timing['iterations']+=1
            phase_begin=time.monotonic()
            spin_for(.01 if args.explore else .05);now=time.monotonic()
            phase_elapsed('spin_callbacks',phase_begin)
            phase_begin=time.monotonic()
            if plan_log:plan_log.refresh()
            phase_elapsed('plan_snapshot_refresh',phase_begin)
            # A normal zero-speed brake is resumable and does not latch an estop.
            # Exploration must never mistake an actual latch for endpoint waiting.
            if not args.explore and halt['reason']=='Driver stop was activated during preparation or driving':
                if not result_future.done():spin_for(.35,cleanup=True)
                delivered=result_future.result().status if result_future.done() else None
                if expected_completion_stop(halt['reason'],delivered):halt['reason']=None
            if halt['reason']:raise RuntimeError(halt['reason'])
            if result_future.done():
                status=result_future.result().status
                if not args.explore or status!=4:break
                if not waiting_for_goal:
                    waiting_for_goal=True;waiting_since=now;completed_segments+=1
                    abandoned=rolling_tracker.finish_navigation()
                    rolling_message=None
                    stop_phase['value']='waiting_for_goal'
                    event('exploration_waiting_for_goal',waiting_for_goal=True,completed_segments=completed_segments,
                          last_goal=exploration_goal,action_status=status,
                          retired_request_stamp=list(abandoned.stamp) if abandoned else None,
                          rolling_goal_tracker=rolling_tracker.snapshot(now=now),
                          note='Current route completed; stopped while mapping and selecting continue. No emergency reset.')
            if now-started>args.timeout:raise RuntimeError('Run time limit reached')
            if stack.poll() is not None or recorder.poll() is not None:raise RuntimeError('Stack or recorder exited')
            ensure_scan_fresh()
            if 'teleop' in observed:raise RuntimeError('Teleop command received during automatic run')
            if observed.get('estop',(None,Bool(data=False)))[1].data:
                raise RuntimeError('Driver brake/emergency stop activated')
            # Freshness is checked even when the latest source sample repeats.
            sample,_=localization.snapshot()
            jump_guard.update(sample)
            current=(sample['x'],sample['y'],sample['yaw'])
            pose_transforms={}
            phase_begin=time.monotonic()
            current_map=None
            if args.explore:
                current_map,_=pose_in_frame(planning['map_frame'])
            if args.explore and current_map is not None:
                if not exploration_visited or math.dist(current_map[:2],exploration_visited[-1])>.4:
                    exploration_visited.append(list(current_map[:2]));exploration_visited=exploration_visited[-500:]
                if frontier_future is not None and frontier_future.done():
                    choices,diagnostic=frontier_future.result();frontier_future=None
                    frontier_roundtrip_ms=1000*(time.monotonic()-frontier_submitted)
                    frontier_compute_ms=diagnostic['selector_compute_ms']
                    # Re-evaluate at today's pose: the car moved during selection.
                    replacement=choose_rolling_goal(choices,current_map,rolling_tracker.anchor_goal,source['speed_setpoint_mps'])
                    decision=('awaiting_planner_ack' if not rolling_tracker.can_request else
                              'no_eligible_extension' if replacement is None else
                              'minimum_update_interval' if now-exploration_changed<EXPLORATION_MIN_GOAL_INTERVAL_S else 'publish')
                    session_log.append('exploration_decisions.jsonl',clean(dict(session_elapsed_s=now-session_started,
                            pose_map=current_map,decision=decision,tracker=rolling_tracker.snapshot(now=now),
                            candidates=choices,selected_candidate=replacement,
                            selector_compute_ms=frontier_compute_ms,selector_roundtrip_ms=frontier_roundtrip_ms)))
                    if decision=='publish':
                        previous=rolling_tracker.accepted_goal
                        exploration_generation+=1
                        exploration_changed=now
                        next_navigation_goal=navigation_goal(replacement['goal'],planning['map_frame'])
                        proposed_message=next_navigation_goal.pose
                        stamp=proposed_message.header.stamp
                        if not rolling_tracker.register_request(replacement['goal'],(stamp.sec,stamp.nanosec),now=now):
                            raise RuntimeError('Exploration goal registration failed')
                        pending_generation=exploration_generation
                        if waiting_for_goal:
                            # The old NavigateToPose action is terminal. Start a
                            # fresh action from the current pose; never retransmit
                            # into the finished BT or reset any physical stop.
                            if halt['reason']:raise RuntimeError(halt['reason'])
                            if not localization.ready():raise RuntimeError('Localization not ready for exploration continuation')
                            ensure_scan_fresh()
                            goal_future=client.send_goal_async(next_navigation_goal,feedback_callback=feedback)
                            handle=await_future(goal_future)
                            if halt['reason']:raise RuntimeError(halt['reason'])
                            if not handle.accepted:raise RuntimeError('Exploration continuation goal rejected')
                            result_future=handle.get_result_async()
                            waiting_for_goal=False
                            stop_phase['value']='driving'
                            event('exploration_resuming',waiting_for_goal=False,generation=exploration_generation,
                                  goal_map=replacement['goal'],waited_s=time.monotonic()-waiting_since,
                                  completed_segments=completed_segments,
                                  note='New navigation action accepted; motion requires a valid new path. No emergency reset.')
                            waiting_since=None
                        # A fast acknowledgement may already have arrived while
                        # awaiting the new action handle. Do not revive its timer.
                        rolling_message=proposed_message if rolling_tracker.pending is not None else None
                        if rolling_message is not None:rolling_pub.publish(rolling_message)
                        event('exploration_goal_updated',generation=exploration_generation,previous_goal=previous,
                              goal_map=replacement['goal'],frontier=replacement['frontier'],diagnostic=diagnostic,
                              selected_candidate=replacement,
                              request_stamp=[stamp.sec,stamp.nanosec],accepted_goal=rolling_tracker.accepted_goal,
                              distance_to_goal_m=math.dist(current_map[:2],replacement['goal'][:2]),
                              selector_compute_ms=frontier_compute_ms,selector_roundtrip_ms=frontier_roundtrip_ms,
                              note='Proposed goal; waiting for BT path commitment before changing the selection anchor')
                    exploration_diagnostic=diagnostic
                if frontier_future is None and now-exploration_poll>=EXPLORATION_CHECK_PERIOD_S:
                    exploration_poll=now
                    request,diagnostic=frontier_request(exclude=exploration_visited,current_map=current_map)
                    if request is not None:
                        frontier_submitted=time.monotonic()
                        # One worker, one in-flight selection, no backlog of old goals.
                        frontier_future=frontier_worker.submit(select_frontiers_job,request)
                    else:exploration_diagnostic=diagnostic
            phase_elapsed('exploration_results_and_submit',phase_begin)
            phase_begin=time.monotonic()
            # Record deviation without stopping solely for distance from the reference.
            if now-last_print>2:
                # This projection is diagnostic only; compute it when logged,
                # rather than scanning the full reference every iteration.
                deviation=project(reference,*current)
                current_deviation=None
                if args.live_slam and plan_log.latest:
                    try:
                        plan_pose,pose_tf=pose_in_frame(plan_log.latest['frame'])
                        if plan_pose is None:
                            current_deviation={'unavailable':pose_tf['error']}
                        else:
                            current_deviation=project(plan_log.latest,*plan_pose)
                            row={'stamp':sample['stamp'],
                                 'recorded_ros_s':node.get_clock().now().nanoseconds*1e-9,'pose_tf':pose_tf,
                                 'plan_sequence':plan_log.sequence,'pose_in_plan_frame':list(plan_pose),
                                 **current_deviation}
                            session_log.append('current_plan_tracking.jsonl',row)
                    except Exception as exc:current_deviation={'unavailable':str(exc)}
                event('exploration_waiting' if waiting_for_goal else 'driving',
                      elapsed_s=now-started,s_m=deviation['s_m'],cross_track_m=deviation['cross_track_m'],
                      heading_error_deg=deviation['heading_error_deg'],
                      reference_role='initial_plan' if args.live_slam else 'fixed_path',
                      current_plan_sequence=plan_log.sequence if args.live_slam else None,
                      current_plan_tracking=current_deviation,feedback=observed.get('feedback',(None,None))[1],
                      rolling_goal_map=exploration_goal if args.explore else None,
                      rolling_goal_tracker=rolling_tracker.snapshot(now=now) if args.explore else None,
                      waiting_for_goal=waiting_for_goal,completed_segments=completed_segments,
                      rolling_goal_distance_m=math.dist(current_map[:2],exploration_goal[:2]) if args.explore and current_map is not None else None,
                      selector_compute_ms=frontier_compute_ms if args.explore else None,
                      selector_roundtrip_ms=frontier_roundtrip_ms if args.explore else None,
                      exploration_diagnostic=exploration_diagnostic if args.explore else None,
                      scan_health=scan_monitor.snapshot(),main_loop_timing=loop_timing,
                      session_log_health=session_log.snapshot())
                last_print=now
            phase_elapsed('tracking_and_diagnostics',phase_begin)
        rolling_feedback_enabled=False
        status=result_future.result().status
        if halt['reason'] and not expected_completion_stop(halt['reason'],status):
            raise RuntimeError(halt['reason'])
        if expected_completion_stop(halt['reason'],status):halt['reason']=None
        outcome=('rolling_horizon_exhausted' if args.explore else 'succeeded') if status==4 else 'action_ended_'+str(status)
        if args.explore:
            event('exploration_finished',goals_selected=exploration_generation,
                  last_goal=exploration_goal,action_status=status,
                  rolling_goal_tracker=rolling_tracker.snapshot(),
                  note='Last usable route ended before another route was installed; this does not mean exploration is complete' if status==4 else 'Navigation ended; no automatic safety-stop reset')
        event(outcome,action_status=status,elapsed_s=time.monotonic()-started)
        return 0 if status==4 and not args.explore else 2
    except Exception as exc:
        outcome='stopped' if state['goal_sent'] else 'failed_preflight'
        if scan_monitor is not None:state['scan_health_at_failure']=scan_monitor.snapshot(include_recent=True)
        state['main_loop_timing_at_failure']=loop_timing
        if session_log is not None:state['session_log_at_failure']=session_log.snapshot()
        state['localization_readiness_at_failure']=localization.readiness_snapshot()[1]
        if route:state['saved_map_localization_at_failure']=localization.saved_map_snapshot()
        try:state['localization_at_failure']=localization.snapshot()[1]
        except RuntimeError as detail:state['localization_at_failure']={'error':str(detail)}
        event(outcome,reason=str(exc),traceback=traceback.format_exc(),
              localization=state['localization_at_failure'])
        return 2
    finally:
        rolling_feedback_enabled=False
        if scan_monitor is not None:scan_monitor.disarm()
        if rolling_timer is not None:rolling_timer.cancel()
        if frontier_worker is not None:frontier_worker.shutdown(wait=False,cancel_futures=True)
        stop_phase['value']='cleanup' 
        idle_verified=False
        if stack:
            try:
                ordinary_cleanup=(state['goal_sent'] and not halt['reason'] and
                    (outcome in ('succeeded','rolling_horizon_exhausted') or normal_stop_started()))
                quiesce(cleanup=True,normal_stop=ordinary_cleanup)
                stack.mark_idle(True)
                idle_verified=True
            except Exception as exc:
                state['cleanup_error']=str(exc)
                # Failed confirmation must not leave an uncontrolled reusable stack.
                emergency_pub.publish(Bool(data=True));spin_for(.15,cleanup=True)
            keep=idle_verified and runtime_ready and not fresh_mapping and not args.shutdown_after_run and not args.stop_stack
            if not keep:
                try:stack.stop()
                except Exception as exc:state['stack_stop_error']=str(exc)
            state['stack_kept_ready']=keep
            try:
                with stack.log_path.open('rb') as stream:
                    stream.seek(stack_offset);(run/'stack.log').write_bytes(stream.read())
            except OSError as exc:state['stack_log_error']=str(exc)
        # Vehicle stop/cancellation takes priority over saving maps or draining
        # diagnostics. Slow storage must not postpone the emergency service.
        if saved_costmap_sync is not None:
            try:saved_costmap_sync.close()
            except Exception as exc:state['saved_costmap_sync_close_error']=str(exc)
        if route:
            try:
                state['saved_map_localization_final']=localization.saved_map_snapshot()
                (run/'saved_map_localization_history.json').write_text(json.dumps(clean(localization.saved_map_history()),indent=2))
            except Exception as exc:state['saved_map_history_save_error']=str(exc)
        if args.explore and 'map' in observed:
            try:
                from frontier_exploration import grid_from_message
                final_map=grid_from_message(observed['map'][1]);final_map['data']=list(final_map['data'])
                (run/'map_final.json').write_text(json.dumps(final_map))
            except Exception as exc:state['final_map_save_error']=str(exc)
        if scan_monitor is not None:
            try:state['scan_health_final']=scan_monitor.snapshot(include_recent=True)
            except Exception as exc:state['scan_snapshot_error']=str(exc)
            try:scan_monitor.close()
            except Exception as exc:state['scan_monitor_close_error']=str(exc)
            try:
                (run/'scan_health.json').write_text(json.dumps(clean(dict(
                    at_failure=state.get('scan_health_at_failure'),final=state.get('scan_health_final'))),indent=2))
            except Exception as exc:state['scan_health_save_error']=str(exc)
        # Recorder includes the brake/cancellation interval; no Nav2 lifecycle
        # teardown is needed for the next user-issued command.
        try:stop_process(recorder)
        except Exception as exc:state['recorder_close_error']=str(exc)
        try:recorderlog.close()
        except Exception as exc:state['recorder_log_close_error']=str(exc)
        if plan_log is not None:
            try:state['plan_log_final']=plan_log.close(timeout_s=3.)
            except Exception as exc:state['plan_log_close_error']=str(exc)
        writer_confirmed_dead=session_log is None
        if session_log is not None:
            try:
                state['session_log_final']=session_log.close(timeout_s=3.)
                writer_confirmed_dead=state['session_log_final'].get('writer_alive') is False
            except Exception as exc:
                state['session_log_close_error']=str(exc)
                try:
                    state['session_log_final']=session_log.snapshot()
                    writer_confirmed_dead=state['session_log_final'].get('writer_alive') is False
                except Exception as detail:state['session_log_snapshot_error']=str(detail)
        # Preserve the terminal reason even if a bounded queue overflowed or
        # its writer failed. This fallback is reached only after quiescence.
        if terminal_event is not None:
            try:
                (run/'terminal_event.json').write_text(json.dumps(terminal_event,indent=2))
                log_health=state.get('session_log_final',{})
                if writer_confirmed_dead and (
                        log_health.get('incomplete',True) or state.get('session_log_enqueue_error')):
                    path=run/'events.jsonl';found=False
                    if path.exists():
                        for line in path.read_text().splitlines():
                            try:found=found or json.loads(line).get('event_sequence')==terminal_event['event_sequence']
                            except ValueError:pass
                    if not found:
                        separator=''
                        if path.exists() and path.stat().st_size:
                            with path.open('rb') as stream:
                                stream.seek(-1,2)
                                if stream.read(1)!=b'\n':separator='\n'
                        with path.open('a') as stream:
                            stream.write(separator+json.dumps(terminal_event)+'\n')
                        state['terminal_event_recovered']=True
            except Exception as exc:state['terminal_event_save_error']=str(exc)
        state.update(stage='finished',outcome=outcome,finished=datetime.now().isoformat())
        state['final_summary_file']='session_final.json'
        if not writer_confirmed_dead:
            state['session_log_shutdown_warning']='Writer exit unconfirmed; session_final.json is authoritative'
        def save_final_state():
            payload=json.dumps(clean(state),indent=2)
            # A stuck helper can only replace session.json, never this final
            # summary. Do not race it when its exit could not be confirmed.
            (run/'session_final.json').write_text(payload)
            if writer_confirmed_dead:(run/'session.json').write_text(payload)
        save_final_state()
        tracking=run/'tracking'
        if (tracking/'metadata.json').exists():
            with (run/'analysis.log').open('w') as stream:
                analysis=subprocess.Popen(['nice','-n','10',sys.executable,str(root/'analyze_tracking.py'),str(tracking),'--no-plot'],
                    stdin=subprocess.DEVNULL,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,close_fds=True)
            state['analysis_pid']=analysis.pid
            save_final_state()
        try:
            export_stop_reports(run,stop_events,outcome,state.get('reason'))
            if route:
                with (run/'stop_report.md').open('a') as stream:
                    stream.write('\n## 保存地图定位与障碍层同步\n\n'
                        '初始化/复用判据：[localization_initialization_status.json](localization_initialization_status.json)。\n\n'
                        '连续地图姿态、AMCL与扫描匹配记录：[saved_map_localization_history.json](saved_map_localization_history.json)。\n\n'
                        '障碍层重建证据见 costmap_sync*.json；地图目标与起步复查始终使用 map 坐标。\n')
            scan=state.get('scan_health_at_failure') or state.get('scan_health_final')
            if scan is not None:
                with (run/'stop_report.md').open('a') as stream:
                    stream.write('\n## 独立雷达监测\n\n'
                        '阈值：接收间隔 1 秒；消息时间戳年龄仅作诊断。\n\n'
                        '监测数据（含超时现场和最近接收记录）：[scan_health.json](scan_health.json)\n\n'
                        '状态：'+str(scan.get('status'))+'；接收年龄：'+str(scan.get('receipt_age_s'))+' s。\n\n'
                        '主循环耗时与异步写入统计见 [session_final.json](session_final.json)。\n')
            print('Stop report: '+str(run/'stop_report.md'),flush=True)
        except Exception as exc:
            print('Stop report export failed: '+str(exc),flush=True)
        print('Result directory: '+str(run),flush=True)
        if state.get('stack_kept_ready'):
            print('Stack remains ready at latched stop. Run the same command for another path; --stop-stack shuts it down.',flush=True)
        localization.close()
        session_executor.close()
        if rolling_status_node is not None:rolling_status_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        lock.close()


if __name__=='__main__':raise SystemExit(main())
