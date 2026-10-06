#!/usr/bin/env python3
"""Unknown-map boundary heartbeat; does not publish velocity or navigation goals."""
import math,time
import numpy as np

class BoundaryGrid:
    def __init__(self,data,width,height,resolution,origin,footprint,padding=.05):
        if width<=0 or height<=0 or resolution<=0 or len(data)!=width*height:raise ValueError('Invalid occupancy grid')
        self.grid=np.asarray(data,dtype=np.int8).reshape(height,width);self.w=width;self.h=height;self.res=resolution;self.origin=origin
        xs=[p[0] for p in footprint];ys=[p[1] for p in footprint]
        xx=np.linspace(min(xs)-padding,max(xs)+padding,math.ceil((max(xs)-min(xs)+2*padding)/(resolution/2))+1)
        yy=np.linspace(min(ys)-padding,max(ys)+padding,math.ceil((max(ys)-min(ys)+2*padding)/(resolution/2))+1)
        a,b=np.meshgrid(xx,yy);self.body=np.column_stack([a.ravel(),b.ravel()])
    def check(self,x,y,yaw,distance,curvature=0.):
        if not all(math.isfinite(v) for v in [x,y,yaw,distance,curvature]) or distance<0:return 'invalid_prediction'
        count=max(1,math.ceil(distance/(self.res/2)))
        ss=np.linspace(0,distance,count+1)
        if abs(curvature)<1e-8:dx=ss;dy=np.zeros_like(ss)
        else:dx=np.sin(curvature*ss)/curvature;dy=(1-np.cos(curvature*ss))/curvature
        angle=yaw+curvature*ss;cx=x+math.cos(yaw)*dx-math.sin(yaw)*dy;cy=y+math.sin(yaw)*dx+math.cos(yaw)*dy
        px=cx[:,None]+np.cos(angle)[:,None]*self.body[:,0]-np.sin(angle)[:,None]*self.body[:,1]
        py=cy[:,None]+np.sin(angle)[:,None]*self.body[:,0]+np.cos(angle)[:,None]*self.body[:,1]
        ox,oy,oa=self.origin;vx=px-ox;vy=py-oy
        ix=np.floor((math.cos(oa)*vx+math.sin(oa)*vy)/self.res).astype(int);iy=np.floor((-math.sin(oa)*vx+math.cos(oa)*vy)/self.res).astype(int)
        if np.any((ix<0)|(iy<0)|(ix>=self.w)|(iy>=self.h)):return 'map_outer_boundary'
        if np.any(self.grid[iy,ix]<0):return 'unknown_region'
        return 'clear'

def main():
    import json,pathlib,yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Bool,String
    from tf2_ros import Buffer,TransformListener
    from ament_index_python.packages import get_package_share_directory
    def angle(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    class Guard(Node):
        def __init__(self):
            super().__init__('map_boundary_guard')
            share=pathlib.Path(get_package_share_directory('racecar'))
            defaults={'require_map':False,'require_map_updates':False,'map_timeout_s':3.,'tf_timeout_s':.3,'state_timeout_s':.5,'reaction_time_s':.6,'forward_margin_m':.15,'footprint_padding_m':.05,'check_frequency_hz':20.,'driver_config':str(share/'config/driver_calibration.yaml'),'nav_config':str(share/'config/nav_carto.yaml')}
            for k,v in defaults.items():self.declare_parameter(k,v)
            self.p={k:self.get_parameter(k).value for k in defaults}
            self.driver=yaml.safe_load(pathlib.Path(self.p['driver_config']).read_text())['racecar_driver']['ros__parameters']
            nav=yaml.safe_load(pathlib.Path(self.p['nav_config']).read_text());self.foot=yaml.safe_load(nav['global_costmap']['global_costmap']['ros__parameters']['footprint'])
            self.grid=None;self.map_frame='map';self.map_received=None;self.seen_map=False;self.state=None;self.state_received=None;self.last_reason=None;self.last_report=0.
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.map_sub=self.create_subscription(OccupancyGrid,'/map',self.map_cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.state_sub=self.create_subscription(String,'/racecar_driver/closed_loop_state',self.state_cb,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.clear_pub=self.create_publisher(Bool,'/racecar/map_boundary_clear',1);self.report=self.create_publisher(String,'~/state',1)
            self.timer=self.create_timer(1/self.p['check_frequency_hz'],self.tick)
        def map_cb(self,m):
            self.seen_map=True
            try:self.grid=BoundaryGrid(m.data,m.info.width,m.info.height,m.info.resolution,(m.info.origin.position.x,m.info.origin.position.y,angle(m.info.origin.orientation)),self.foot,self.p['footprint_padding_m']);self.map_frame=m.header.frame_id;self.map_received=time.monotonic()
            except Exception:self.grid=None
        def state_cb(self,m):
            try:self.state=json.loads(m.data);self.state_received=time.monotonic()
            except Exception:pass
        def evaluate(self):
            now=time.monotonic()
            if not self.seen_map and not self.p['require_map']:return 'no_map_mode',0.
            if self.grid is None:return 'map_unavailable',0.
            if self.p['require_map_updates'] and now-self.map_received>self.p['map_timeout_s']:return 'map_updates_stale',0.
            if self.state is None or now-self.state_received>self.p['state_timeout_s']:return 'driver_state_stale',0.
            s=self.state
            try:
                t=self.buffer.lookup_transform(self.map_frame,self.driver['feedback_base_frame'],rclpy.time.Time())
                age=self.get_clock().now().nanoseconds*1e-9-t.header.stamp.sec-t.header.stamp.nanosec*1e-9
                if not(-.05<=age<=self.p['tf_timeout_s']):return 'localization_stale',0.
                speed=max(abs(float(s.get('measured_v',0))),abs(float(s.get('target_v',0))))
                if not math.isfinite(speed) or speed>self.driver['feedback_max_speed_mps']:return 'speed_invalid',0.
                # Full-speed travel during pulse + response margin: provisional, not a measured stopping distance.
                distance=speed*(self.driver['brake_duration_s']+self.p['reaction_time_s'])+self.p['forward_margin_m']
                steer=float(s.get('base_steering_deg',0))+float(s.get('yaw_correction_deg',0))
                if not math.isfinite(steer):return 'steering_invalid',0.
                steer=max(-self.driver['right_angle_deg'],min(self.driver['left_angle_deg'],steer))
                curvature=math.tan(math.radians(steer))/self.driver['wheelbase_m']
                tr=t.transform;return self.grid.check(tr.translation.x,tr.translation.y,angle(tr.rotation),distance,curvature),distance
            except Exception:return 'localization_unavailable',0.
        def tick(self):
            started=time.monotonic();reason,horizon=self.evaluate();clear=reason in ['clear','no_map_mode']
            message=Bool();message.data=clear;self.clear_pub.publish(message)
            if reason!=self.last_reason or started-self.last_report>=1:
                report=String();report.data=json.dumps({'clear':clear,'reason':reason,'forward_horizon_m':horizon,'map_seen':self.seen_map,'compute_ms':1000*(time.monotonic()-started)});self.report.publish(report)
                if reason!=self.last_reason:self.get_logger().info(report.data)
                self.last_reason=reason;self.last_report=started
    rclpy.init();node=Guard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
