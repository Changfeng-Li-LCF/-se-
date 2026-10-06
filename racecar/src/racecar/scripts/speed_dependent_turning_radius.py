#!/usr/bin/env python3
"""Speed-dependent Smac radius. This node sets planning parameters only."""
import math
import time


def radius_for_speed(speed, wheelbase, left_deg, right_deg, lateral_accel):
    values=(speed,wheelbase,left_deg,right_deg,lateral_accel)
    if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite radius input')
    if wheelbase<=0 or lateral_accel<=0 or not(0<left_deg<89 and 0<right_deg<89):
        raise ValueError('Invalid geometry/acceleration')
    geometric=wheelbase/math.tan(math.radians(min(left_deg,right_deg)))
    return max(geometric,abs(speed)**2/lateral_accel)


class RadiusPolicy:
    def __init__(self,wheelbase,left,right,accel,fallback_speed,timeout=.3,step=.05,tau=.5):
        self.geometry=(wheelbase,left,right,accel)
        self.execution_speed=abs(fallback_speed)
        self.fallback=radius_for_speed(fallback_speed,*self.geometry)
        self.timeout=timeout;self.step=step;self.tau=tau
        self.speed=None;self.received=None;self.stamp=None;self.applied=None;self.last_ros=None
    def accept(self,speed,stamp,ros_now,steady_now,max_speed):
        if not all(math.isfinite(v) for v in (speed,stamp,ros_now,steady_now)):return False
        if stamp<=0 or not(-.05<=ros_now-stamp<=self.timeout) or abs(speed)>max_speed:return False
        if self.last_ros is not None and ros_now<self.last_ros-.05:
            self.speed=None;self.received=None;self.stamp=None
        self.last_ros=ros_now
        if self.stamp is not None and stamp<=self.stamp:return False
        value=abs(speed)
        # Fast attack, slow release: rising speed never gets hidden by smoothing.
        if self.speed is None or self.received is None or steady_now-self.received>self.timeout or value>=self.speed:self.speed=value
        else:
            dt=max(0,steady_now-self.received);alpha=1-math.exp(-dt/self.tau)
            self.speed+=alpha*(value-self.speed)
        self.received=steady_now;self.stamp=stamp;return True
    def desired(self,ros_now,steady_now):
        fresh=self.received is not None and 0<=steady_now-self.received<=self.timeout and -.05<=ros_now-self.stamp<=self.timeout
        # A stationary car will accelerate to the shared execution setpoint after
        # planning. Do not plan tighter turns than that execution speed permits.
        # Faster measured motion can still increase the radius immediately.
        raw=radius_for_speed(max(self.speed,self.execution_speed),*self.geometry) if fresh else max(self.fallback,self.applied or 0.)
        return math.ceil((raw-1e-10)/self.step)*self.step,fresh


def needs_radius_update(target, applied, hysteresis):
    return applied is not None and (target>applied+1e-6 or target<applied-hysteresis+1e-6)


def main():
    import json,pathlib,yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rcl_interfaces.srv import SetParametersAtomically, GetParameters
    from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_share_directory
    class RadiusNode(Node):
        def __init__(self):
            super().__init__('speed_dependent_turning_radius')
            default=str(pathlib.Path(get_package_share_directory('racecar'))/'config/driver_calibration.yaml')
            values={'driver_config':default,'lateral_accel_limit_mps2':.8,'update_period_s':.5,'radius_step_m':.05,'decrease_hysteresis_m':.1,'speed_release_tau_s':.5,'planner_node':'planner_server','planner_id':'GridBased','owner_pid':0}
            for k,v in values.items():self.declare_parameter(k,v)
            cfg=yaml.safe_load(pathlib.Path(self.get_parameter('driver_config').value).read_text())['racecar_driver']['ros__parameters']
            get=lambda k:self.get_parameter(k).value
            self.policy=RadiusPolicy(cfg['wheelbase_m'],cfg['left_angle_deg'],cfg['right_angle_deg'],get('lateral_accel_limit_mps2'),max(cfg['speed_setpoint_mps'],cfg['max_speed_mps']),cfg['feedback_timeout_s'],get('radius_step_m'),get('speed_release_tau_s'))
            if get('update_period_s')<=0 or get('radius_step_m')<=0 or get('speed_release_tau_s')<=0:raise ValueError('Invalid update timing/quantization')
            self.frame=cfg['feedback_base_frame'];self.max_speed=cfg['feedback_max_speed_mps'];self.hysteresis=get('decrease_hysteresis_m')
            self.parameter=get('planner_id')+'.minimum_turning_radius';planner=get('planner_node').rstrip('/')
            self.client=self.create_client(SetParametersAtomically,planner+'/set_parameters_atomically')
            self.read_client=self.create_client(GetParameters,planner+'/get_parameters')
            self.state_client=self.create_client(GetState,planner+'/get_state')
            self.state_future=None;self.state_when=0.;self.active=False;self.pending=None;self.pending_when=0.;self.last_log=0.;self.last_check=0.
            self.pending_kind=None;self.next_request=0.;self.last_apply_ms=None
            self.owner=int(get('owner_pid'));self.owner_identity=None
            if self.owner:self.owner_identity=pathlib.Path('/proc',str(self.owner),'stat').read_text().split()[21]
            # Use the primary odometry source only, avoiding mixed timestamps/frames.
            self.sub=self.create_subscription(Odometry,cfg['feedback_odom_topics'][0],self.odom,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.pub=self.create_publisher(String,'~/state',1)
            self.timer=self.create_timer(get('update_period_s'),self.tick)
            self.get_logger().info('Dynamic planning radius: max(L/tan(delta), max(abs(v_measured), v_execution)^2/a); a='+str(get('lateral_accel_limit_mps2'))+'; primary odom='+cfg['feedback_odom_topics'][0])
        def odom(self,msg):
            if msg.child_frame_id.lstrip('/')!=self.frame.lstrip('/'):return
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            self.policy.accept(msg.twist.twist.linear.x,stamp,self.get_clock().now().nanoseconds*1e-9,time.monotonic(),self.max_speed)
        def finish_request(self,future):
            if future is not self.pending:return
            now=time.monotonic()
            try:
                answer=self.pending.result()
                if self.pending_kind=='read':
                    value=answer.values[0]
                    if value.type!=ParameterType.PARAMETER_DOUBLE or not math.isfinite(value.double_value) or value.double_value<=0:
                        raise ValueError('Invalid planner radius readback')
                    self.policy.applied=value.double_value;self.last_check=now
                elif answer.result.successful:
                    self.policy.applied=self.sent;self.last_check=now
                    self.last_apply_ms=(now-self.pending_when)*1000
                    self.get_logger().info('Planner turning radius applied: %.2f m (%.3f ms)'%(self.sent,self.last_apply_ms))
                else:raise RuntimeError('Planner rejected radius: '+answer.result.reason)
            except Exception as e:self.get_logger().error('Radius update failed: '+str(e))
            self.pending=None;self.pending_kind=None;self.next_request=now+.5
        def tick(self):
            now=time.monotonic()
            if self.owner:
                try:
                    if pathlib.Path('/proc',str(self.owner),'stat').read_text().split()[21]!=self.owner_identity:raise OSError()
                except OSError:raise KeyboardInterrupt()
            if self.state_future is not None and self.state_future.done():
                try:
                    active=self.state_future.result().current_state.id==3
                    if not active:self.policy.applied=None
                    self.active=active
                except Exception as e:
                    self.active=False;self.get_logger().warn('Planner state unavailable: '+str(e))
                self.state_future=None
            if self.state_future is not None and now-self.state_when>8:
                self.state_future.cancel();self.state_client.remove_pending_request(self.state_future)
                self.state_future=None;self.active=False
            if self.pending is not None:
                if self.pending.done():self.finish_request(self.pending)
                elif now-self.pending_when>15 and now-self.last_log>5:
                    self.get_logger().warn('Planner parameter response pending; no overlapping requests');self.last_log=now
                if self.pending is not None and self.pending_kind=='read' and now-self.pending_when>8:
                    self.pending.cancel();self.read_client.remove_pending_request(self.pending)
                    self.pending=None;self.pending_kind=None;self.active=False;self.next_request=now+2.
            target,fresh=self.policy.desired(self.get_clock().now().nanoseconds*1e-9,now)
            # Do not queue lifecycle requests behind a table rebuild.
            if self.pending is None and self.state_future is None and now-self.state_when>=2 and self.state_client.service_is_ready():
                self.state_future=self.state_client.call_async(GetState.Request());self.state_when=now
            can_request=self.active and self.pending is None and self.state_future is None and now>=self.next_request
            # Read back to detect a restart; identical writes rebuild Smac too.
            if can_request and (self.policy.applied is None or now-self.last_check>=30):
                if self.read_client.service_is_ready():
                    self.pending=self.read_client.call_async(GetParameters.Request(names=[self.parameter]))
                    self.pending_kind='read';self.pending_when=now
                    self.pending.add_done_callback(self.finish_request)
            elif can_request and needs_radius_update(target,self.policy.applied,self.hysteresis) and self.client.service_is_ready():
                req=SetParametersAtomically.Request();req.parameters=[Parameter(name=self.parameter,value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=float(target)))]
                self.sent=target;self.pending_when=now;self.pending=self.client.call_async(req)
                self.pending_kind='write'
                self.pending.add_done_callback(self.finish_request)
            msg=String();msg.data=json.dumps({'feedback_fresh':fresh,'speed_mps':self.policy.speed,'execution_speed_floor_mps':self.policy.execution_speed,'desired_radius_m':target,'applied_radius_m':self.policy.applied,'planner_active':self.active,'request_pending':self.pending_kind,'request_elapsed_s':now-self.pending_when if self.pending else 0.,'last_apply_ms':self.last_apply_ms});self.pub.publish(msg)
    rclpy.init();node=RadiusNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
