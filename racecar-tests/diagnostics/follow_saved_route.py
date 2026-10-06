#!/usr/bin/env python3
"""User-invoked execution of an existing Nav2 path; never assumes an initial pose."""
import argparse
import json
import math
from pathlib import Path
import signal
import time


def load_route(filename):
    result=json.loads(Path(filename).read_text())
    if result.get('action_status')!=4 or len(result.get('path',[]))<2:
        raise ValueError('File does not contain a successfully planned path')
    for p in result['path']:
        if not all(math.isfinite(p[k]) for k in ['x','y','qx','qy','qz','qw']):
            raise ValueError('Non-finite path coordinate')
        if abs(sum(p[k]**2 for k in ['qx','qy','qz','qw'])-1)>1e-3:
            raise ValueError('Invalid path orientation')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',default=str(Path.home()/'maps/plans/saved-map-20260929-170422/result.json'))
    parser.add_argument('--describe',action='store_true',help='Print route metadata without contacting ROS or moving')
    args=parser.parse_args();route=load_route(args.plan)
    if args.describe:
        print(json.dumps({k:v for k,v in route.items() if k!='path'},indent=2));return

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from nav_msgs.msg import Path as PathMessage
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import FollowPath
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import GetParameters

    rclpy.init();node=rclpy.create_node('follow_saved_route_request')
    interrupted=False;submitted=False;handle=None;last_feedback=0.
    def interrupt(sig,frame):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    def wait(future,timeout,allow_interrupt=True):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:
            if allow_interrupt and interrupted:raise KeyboardInterrupt()
            rclpy.spin_once(node,timeout_sec=.1)
        if not future.done():raise TimeoutError('ROS request timed out')
        return future.result()
    def emergency_stop():
        client=node.create_client(Trigger,'/racecar_driver/emergency_stop')
        if not client.wait_for_service(timeout_sec=2.):
            print('STOP SERVICE UNAVAILABLE: use the vehicle power cutoff.',flush=True);return
        response=wait(client.call_async(Trigger.Request()),3.,False)
        print(json.dumps({'stop_success':response.success,'message':response.message}),flush=True)
    try:
        # Ensure the saved-map mode loaded exactly the map used for this path.
        client=node.create_client(GetParameters,'/map_server/get_parameters')
        if not client.wait_for_service(timeout_sec=10.):
            raise RuntimeError('Start saved-map navigation first; map_server is unavailable')
        request=GetParameters.Request();request.names=['yaml_filename']
        values=wait(client.call_async(request),5.).values
        if not values or Path(values[0].string_value).resolve()!=Path(route['map']).resolve():
            raise RuntimeError('map_server is not using the map recorded with this route')
        latch=[]
        sub=node.create_subscription(Bool,'/racecar_driver/emergency_stop_active',
            lambda msg:latch.append(msg.data),QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        end=time.monotonic()+5.
        while not latch and time.monotonic()<end:
            if interrupted:raise KeyboardInterrupt()
            rclpy.spin_once(node,timeout_sec=.1)
        if not latch:raise RuntimeError('Driver stop state unavailable; no path sent')
        if latch[-1]:raise RuntimeError('Emergency stop remains latched; run reset_emergency_stop.sh first')
        action=ActionClient(node,FollowPath,'/follow_path')
        if not action.wait_for_server(timeout_sec=10.):raise RuntimeError('FollowPath server unavailable')
        path=PathMessage();path.header.frame_id='map';path.header.stamp=node.get_clock().now().to_msg()
        for item in route['path']:
            pose=PoseStamped();pose.header=path.header
            pose.pose.position.x=item['x'];pose.pose.position.y=item['y']
            for field in ['x','y','z','w']:setattr(pose.pose.orientation,field,item['q'+field])
            path.poses.append(pose)
        # Existing saved-map RViz displays /plan. This publisher is visualization only.
        publisher=node.create_publisher(PathMessage,'/plan',QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        publisher.publish(path);timer=node.create_timer(1.,lambda:publisher.publish(path))
        goal=FollowPath.Goal();goal.path=path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker'
        def feedback(msg):
            nonlocal last_feedback
            now=time.monotonic()
            if now-last_feedback>=1.:
                last_feedback=now
                print(json.dumps({'distance_to_goal_m':msg.feedback.distance_to_goal,
                                  'speed_mps':msg.feedback.speed}),flush=True)
        print('Sending saved path. Ctrl+C requests emergency stop and cancels this goal.',flush=True)
        submitted=True
        handle=wait(action.send_goal_async(goal,feedback_callback=feedback),10.)
        if not handle.accepted:
            submitted=False;raise RuntimeError('FollowPath goal was rejected')
        result_future=handle.get_result_async()
        while not result_future.done():
            if interrupted:raise KeyboardInterrupt()
            rclpy.spin_once(node,timeout_sec=.1)
        result=result_future.result()
        if result.status!=4:emergency_stop()
        submitted=False
        print(json.dumps({'action_status':result.status,'succeeded':result.status==4}),flush=True)
    except (KeyboardInterrupt,Exception) as error:
        if submitted:
            try:emergency_stop()
            except Exception as stop_error:print('Stop request failed: '+str(stop_error),flush=True)
            if handle is not None and handle.accepted:
                try:wait(handle.cancel_goal_async(),3.,False)
                except Exception:pass
        print('Interrupted' if isinstance(error,KeyboardInterrupt) else str(error),flush=True)
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
