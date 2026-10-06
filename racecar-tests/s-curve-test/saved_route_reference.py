"""Saved-map ordered goals using the trial's existing planner/costmap policy."""
import json,math,time
from pathlib import Path

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def load_route(filename):
    data=json.loads(Path(filename).expanduser().read_text())
    if data.get('frame')!='map':raise ValueError('Marked route must use map coordinates')
    data['map_yaml']=str(Path(data['map_yaml']).expanduser().resolve())
    if not Path(data['map_yaml']).is_file():raise ValueError('Saved map YAML is missing')
    if len(data.get('goals',[]))<2:raise ValueError('Marked route needs ordered goals')
    for point in data['goals']+[data['initial_prior']]:
        if not all(math.isfinite(point[k]) for k in ('x','y','yaw')):raise ValueError('Non-finite route pose')
    if data.get('repeat',False):raise ValueError('This runner executes one route only')
    distance=float(data.get('terminal_approach_distance_m',0.0))
    if not math.isfinite(distance) or distance<0:raise ValueError('Invalid terminal approach distance')
    # The final approach is a planner preference, never an action waypoint.
    # Also remove guides present in route files saved by the previous version.
    data['goals']=[p for p in data['goals'] if p.get('label')!='terminal-alignment-guide']
    if len(data['goals'])<2:raise ValueError('Marked route needs ordered goals')
    return data

def configure_terminal_reference(node,await_future,route):
    """Set or clear the final-only soft preference before any planning request."""
    from rcl_interfaces.msg import Parameter,ParameterType,ParameterValue
    from rcl_interfaces.srv import SetParametersAtomically
    reference=[]
    if route and float(route.get('terminal_approach_distance_m',0.0))>0:
        end=route['goals'][-1]
        reference=[float(end[k]) for k in ('x','y','yaw')]+[float(route['terminal_approach_distance_m'])]
    client=node.create_client(SetParametersAtomically,'/planner_server/set_parameters_atomically')
    try:
        if not client.wait_for_service(timeout_sec=4):raise RuntimeError('Planner parameter service unavailable')
        request=SetParametersAtomically.Request()
        request.parameters=[Parameter(name='GridBased.terminal_alignment_reference',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,double_array_value=reference))]
        response=await_future(client.call_async(request))
        if response is None or not response.result.successful:
            raise RuntimeError('Planner terminal preference rejected: '+
                (response.result.reason if response is not None else 'no response'))
        return reference
    finally:
        node.destroy_client(client)

def make_pose(node,p):
    from geometry_msgs.msg import PoseStamped
    pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
    pose.pose.position.x=float(p['x']);pose.pose.position.y=float(p['y'])
    pose.pose.orientation.z=math.sin(p['yaw']/2);pose.pose.orientation.w=math.cos(p['yaw']/2)
    return pose

def navigation_goal(node,route):
    from nav2_msgs.action import NavigateThroughPoses
    from ament_index_python.packages import get_package_share_directory
    goal=NavigateThroughPoses.Goal();goal.poses=[make_pose(node,p) for p in route['goals']]
    goal.behavior_tree=str(Path(get_package_share_directory('racecar'))/'config/navigate_marked_route.xml')
    return goal

def initialize_localization(node,buffer,await_future,wait_until,spin_for,trigger,event,run,route,wait_lifecycle,
                            *,reused=False,force_reinitialize=False,monitor=None):
    """Reuse a verified map estimate; reset only a new/unhealthy/explicit session.

    `reused` is true only for a stack whose configuration fingerprint includes
    this saved map. The monitor remains alive through planning and driving.
    """
    from collections import deque
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_srvs.srv import Empty
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from saved_localization_policy import reuse_decision,verified_snapshot,reuse_observation,force_consumed
    if monitor is None:
        raise RuntimeError('Saved-map initialization requires the continuous localization monitor')
    history=deque(maxlen=2048)
    invalid_samples=0
    assessment={}
    diagnostic=dict(reused=bool(reused),force_reinitialize=bool(force_reinitialize),
                    reinitialized=False,reason=None,pose=None,assessment={})
    def capture(msg):
        nonlocal invalid_samples
        p=msg.pose.pose.position; q=msg.pose.pose.orientation; cv=msg.pose.covariance
        values=(p.x,p.y,q.x,q.y,q.z,q.w,cv[0],cv[7],cv[35])
        if not all(math.isfinite(value) for value in values) or any(cv[i]<0 for i in (0,7,35)):
            invalid_samples+=1;return
        norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if norm<1e-6:invalid_samples+=1;return
        qx,qy,qz,qw=(value/norm for value in (q.x,q.y,q.z,q.w))
        a=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        history.append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                        'x':p.x,'y':p.y,'yaw':a,'std':[math.sqrt(cv[i]) for i in (0,7,35)],
                        'received':time.monotonic()})
    sub=node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',capture,
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pub=None;force=None;pending=None
    try:
        wait_lifecycle('/amcl')
        # Hold stop throughout startup, including the short warm-reuse assessment.
        trigger('/racecar_driver/emergency_stop')
        decision=reuse_decision(None,reused=reused,force_reinitialize=force_reinitialize)
        if reused and not force_reinitialize:
            begun=time.monotonic()
            def inspect_reuse():
                nonlocal assessment,decision
                assessment=monitor.saved_map_snapshot()
                observation=reuse_observation(assessment)
                decision=dict(reuse=observation['healthy'],reason=observation['reason'])
                # Wait for actual map/AMCL/scan evidence and a complete global
                # window. DDS discovery or an empty window must never reset a
                # healthy retained pose simply because three seconds elapsed.
                return observation['complete']
            try:
                wait_until(inspect_reuse,75.,'saved-map reuse evidence')
            except RuntimeError as exc:
                if str(exc)=='Timed out waiting for saved-map reuse evidence':
                    diagnostic['reason']='retained localization evidence unavailable; no initial-pose reset sent'
                    raise RuntimeError('Retained saved-map localization lacks fresh assessment evidence: '+
                        str(assessment.get('reason','no observation received'))+
                        '; existing estimate was preserved; assessment='+str(assessment)) from exc
                raise
            event('saved_map_reuse_assessed',reuse=decision['reuse'],reason=decision['reason'],
                  assessment=assessment,elapsed_s=time.monotonic()-begun)
        diagnostic['reason']=decision['reason']
        if decision['reuse']:
            diagnostic.update(pose=assessment['pose'],assessment=assessment)
            event('saved_map_localization_reused',**diagnostic,
                  note='Retained verified same-map localization; no initial-pose reset or forced scan update')
            return diagnostic
        pub=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        force=node.create_client(Empty,'/request_nomotion_update')
        wait_until(lambda:pub.get_subscription_count()>0,15,'AMCL initial-pose subscription')
        pose=PoseWithCovarianceStamped();pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
        p=dict(route['initial_prior']);seed=monitor.refine_saved_map_prior(p)
        diagnostic['scan_seed']=seed
        event('saved_map_scan_seed',scan_seed=seed)
        if seed.get('used',False):p=seed['pose']
        pose.pose.pose.position.x=p['x'];pose.pose.pose.position.y=p['y']
        pose.pose.pose.orientation.z=math.sin(p['yaw']/2);pose.pose.pose.orientation.w=math.cos(p['yaw']/2)
        std=seed['covariance_std'] if seed.get('used',False) else [.35,.35,math.radians(20)]
        pose.pose.covariance[0]=std[0]**2;pose.pose.covariance[7]=std[1]**2;pose.pose.covariance[35]=std[2]**2
        pose.header.stamp=node.get_clock().now().to_msg()
        # The old stability and covariance must not approve the new initialization.
        monitor.reset_saved_map_stability(invalidate_amcl=True)
        history.clear();pub.publish(pose);diagnostic['reinitialized']=True
        event('saved_map_initial_prior',prior=p,reinitialization_reason=decision['reason'],
              note='Approximate prior; hold at latched stop until independent scan/map matching and global pose stabilize')
        begun=time.monotonic();last_force=begun-.5;last_event=begun-2.
        phase='converging';last_request=None;request_acknowledged=False
        def converged():
            nonlocal last_force,last_event,pending,assessment,phase,last_request,request_acknowledged
            now=time.monotonic()
            assessment=monitor.saved_map_snapshot()
            ready,reason=verified_snapshot(assessment)
            if pending is not None and pending.done():
                answer=pending.result();pending=None
                if answer is None:raise RuntimeError('AMCL no-motion update returned an empty response')
                request_acknowledged=True
            consumed=force_consumed(assessment,last_request,request_acknowledged)
            if phase=='converging':
                # Once covariance, scan alignment and freshness pass, stop
                # forcing AMCL so its final stationary window can settle.
                if ready or assessment.get('quality_ready',False):
                    phase='draining'
                elif now-last_force>=.5 and force.service_is_ready() and pending is None and consumed:
                    alignment=assessment.get('alignment') or {}
                    last_request=dict(previous_amcl_stamp=assessment.get('amcl_stamp'),
                        scan_source_boundary=alignment.get('scan_stamp'),requested_monotonic=now)
                    request_acknowledged=False
                    pending=force.call_async(Empty.Request());last_force=now
                    consumed=False
            if phase=='draining' and pending is None and consumed:
                # A completed service only sets AMCL's flag. Observe a subsequent
                # AMCL source update plus advancing scans, then start a fresh
                # full map stability window without further forced requests.
                monitor.reset_saved_map_stability(invalidate_amcl=False)
                phase='settling';assessment=monitor.saved_map_snapshot();ready=False
                reason='waiting for the post-force global stability window'
            elif phase=='settling':
                if ready and pending is None and consumed:
                    diagnostic['settled_after_last_force']=True
                    diagnostic['last_force_request']=last_request
                    return True
                observation=reuse_observation(assessment)
                # If the last update changed quality, another convergence round
                # is allowed only after the quiet verification window completed.
                # Never clear the window on every no-motion request.
                if (observation['complete'] and not observation['healthy']
                        and not assessment.get('quality_ready',False)):
                    phase='converging'
            if now-last_event>=2:
                event('saved_map_localizing',elapsed_s=now-begun,assessment=assessment,
                      ready=ready,reason=reason,convergence_phase=phase,
                      force_acknowledged=request_acknowledged,force_consumed=consumed,
                      last_force_request=last_request)
                last_event=now
            return False
        try:
            wait_until(converged,75,'saved-map scan matching convergence')
        except RuntimeError as exc:
            # Retain the exact unresolved sensor/map condition rather than only
            # reporting a generic startup timeout.
            if str(exc)=='Timed out waiting for saved-map scan matching convergence':
                raise RuntimeError('Saved-map localization did not converge: '+
                    str(assessment.get('reason','no fresh map pose assessment'))+
                    '; phase='+phase+'; last_force_request='+str(last_request)+
                    '; force_acknowledged='+str(request_acknowledged)+'; assessment='+str(assessment)) from exc
            raise
        diagnostic.update(pose=assessment['pose'],assessment=assessment)
        event('saved_map_localization_ready',**diagnostic)
        return diagnostic
    finally:
        diagnostic['assessment']=assessment
        diagnostic['invalid_amcl_samples']=invalid_samples
        (Path(run)/'localization_initialization.json').write_text(json.dumps(list(history),indent=2))
        (Path(run)/'localization_initialization_status.json').write_text(json.dumps(diagnostic,indent=2))
        if pending is not None and not pending.done():
            pending.cancel();force.remove_pending_request(pending)
        node.destroy_subscription(sub)
        if pub is not None:node.destroy_publisher(pub)
        if force is not None:node.destroy_client(force)

def plan_route_reference(node,buffer,observed,await_future,run,nav_source,start,route,
                         *,pose_wait=None,check_abort=None,on_pose_wait=None):
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from nav2_msgs.action import ComputePathThroughPoses
    from nav_msgs.msg import Path as RosPath
    from visualization_msgs.msg import Marker,MarkerArray
    from planner_costmap_audit import snapshot_global_costmap,current_pose,transform_poses,runtime_rules,wait_for_current_pose
    from live_slam_reference import template_from_odom
    directory=Path(run)/'planning_audit';directory.mkdir(exist_ok=True)
    publishers=[p.node_name for p in node.get_publishers_info_by_topic('/map')]
    if publishers!=['map_server']:raise RuntimeError('Saved route requires one map_server publisher: '+str(publishers))
    action=ActionClient(node,ComputePathThroughPoses,'/compute_path_through_poses')
    result={'mode':'saved_map_marked_route','map_publishers':publishers,'goals':route['goals']}
    handle=None
    def snapshot_with_pose(label):
        diagnostic=result.setdefault('pose_wait',{}).setdefault(label,{'phase':label})
        while True:
            if check_abort is not None:check_abort()
            grid,info=snapshot_global_costmap(node,await_future,directory,label,rules)
            if info['frame']!='map':raise RuntimeError('Marked route costmap must use map coordinates')
            read_pose=lambda:current_pose(node,buffer,info['frame'],rules['base_frame'],diagnostic=diagnostic)
            if pose_wait is None:return grid,info,read_pose()
            current=wait_for_current_pose(read_pose,wait=pose_wait,diagnostic=diagnostic,
                timeout_s=None,check_abort=check_abort,on_wait=on_pose_wait)
            age=node.get_clock().now().nanoseconds*1e-9-info['update_stamp']
            diagnostic['costmap_age_after_pose_wait_s']=age
            if -.05<=age<=3.:return grid,info,current
            if age<-.05:raise RuntimeError('Planner global costmap has future update time')
            # A prolonged TF wait requires a new snapshot before auditing.
    try:
        rules=runtime_rules(node,await_future)
        grid,info,current=snapshot_with_pose('marked-route.before')
        frame=info['frame']
        checks={}
        result.update(start_map=current,rules=rules,footprint_checks=checks,map_frame=frame)
        for label,poses in [('start',[current])]+[(p['label'],[[p['x'],p['y'],p['yaw']]]) for p in route['goals']]:
            checks[label]=grid.audit(poses)
            if not checks[label]['clear']:
                result.update(rejected_target=label,first_rejection=checks[label]['first_rejection'])
                raise RuntimeError('Marked route footprint rejected: '+label+' '+str(checks[label]['first_rejection']))
        result.update(start_map=current,rules=rules,footprint_checks=checks)
        if not action.wait_for_server(timeout_sec=10):raise RuntimeError('ComputePathThroughPoses unavailable')
        goal=ComputePathThroughPoses.Goal();goal.use_start=False;goal.planner_id='GridBased'
        goal.goals=[make_pose(node,p) for p in route['goals']]
        handle=await_future(action.send_goal_async(goal),15)
        if not handle.accepted:raise RuntimeError('Marked route planning request rejected')
        answer=await_future(handle.get_result_async(),max(30,8*len(goal.goals)))
        result['status']=answer.status
        path=answer.result.path
        if answer.status!=4 or len(path.poses)<2:raise RuntimeError('Planner could not connect marked targets; see stack/planning logs')
        if path.header.frame_id!=frame:raise RuntimeError('Unexpected planned path frame')
        map_poses=[(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in path.poses]
        result.update(path_map=map_poses,map_frame=frame)
        latest,after,current=snapshot_with_pose('marked-route.after')
        checked=latest.audit([current]+map_poses);result['path_audit']=checked
        if not checked['clear']:raise RuntimeError('Marked route path audit rejected '+str(checked['first_rejection']))
        odom=transform_poses(buffer,'odom',frame,map_poses);template=template_from_odom(odom,start)
        end=route['goals'][-1]
        result.update(goal_map=[end['x'],end['y'],end['yaw']],path_map=map_poses,path_odom=odom,map_frame=frame,length_m=template['length_m'])
        # A corrected stopped-start plan replaces the old visualization rather
        # than leaving old timers to keep publishing superseded routes.
        for key in ('live_plan_timer','target_marker_timer'):
            timer=observed.pop(key,None)
            if timer is not None:node.destroy_timer(timer)
        for key in ('live_plan_publisher','target_marker_publisher'):
            old=observed.pop(key,None)
            if old is not None:node.destroy_publisher(old)
        pub=node.create_publisher(RosPath,'/live_slam_initial_plan',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        pub.publish(path);observed['live_plan_publisher']=pub;observed['live_plan_timer']=node.create_timer(.5,lambda:pub.publish(path))
        marks=MarkerArray()
        for i,p in enumerate(route['display_targets']):
            pose=make_pose(node,p)
            marker=Marker();marker.header=pose.header;marker.ns='marked_target';marker.id=i
            marker.type=Marker.ARROW;marker.action=Marker.ADD;marker.pose=pose.pose
            marker.scale.x=.4;marker.scale.y=.06;marker.scale.z=.08
            marker.color.r=.65;marker.color.g=.15;marker.color.b=.85;marker.color.a=1.;marks.markers.append(marker)
            label=Marker();label.header=pose.header;label.ns='marked_label';label.id=i;label.type=Marker.TEXT_VIEW_FACING
            label.pose=make_pose(node,p).pose;label.pose.position.z=.35;label.scale.z=.23;label.color.a=1.;label.color.r=1.;label.color.g=1.;label.color.b=1.
            label.text=p['label'];marks.markers.append(label)
        marker_pub=node.create_publisher(MarkerArray,'/marked_route/targets',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        marker_pub.publish(marks);observed['target_marker_publisher']=marker_pub;observed['target_marker_timer']=node.create_timer(1.,lambda:marker_pub.publish(marks))
        return template,result
    except Exception:
        if handle is not None and handle.accepted:
            try:await_future(handle.cancel_goal_async(),3)
            except Exception:pass
        raise
    finally:
        (Path(run)/'live_plan.json').write_text(json.dumps(result,indent=2))
        action.destroy()
