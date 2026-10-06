"""Fresh localization intake for the session monitor, independent of its action loop."""
from collections import Counter, deque
import math
import threading
import time


class PoseInbox:
    def __init__(self, base_frame='base_footprint', max_age=.5, max_speed=2., max_yaw_rate=6.,
                 ros_clock=time.time, monotonic_clock=time.monotonic):
        self.base_frame=base_frame
        self.max_age=max_age
        self.max_speed=max_speed
        self.max_yaw_rate=max_yaw_rate
        self.lock=threading.Lock()
        self.sample=None
        self.window=deque(maxlen=80)
        self.status='no odometry received'
        self.ros_clock=ros_clock
        self.monotonic_clock=monotonic_clock
        self.rejected=Counter()
        self.gap_resets=0

    def ingest(self, *, stamp, receipt, now_ros, frame, child, x, y, quaternion, vx, vy, wz):
        with self.lock:
            values=(stamp,receipt,now_ros,x,y,vx,vy,wz,*quaternion)
            reason=None
            if not all(math.isfinite(v) for v in values):reason='nonfinite odometry'
            elif frame.lstrip('/')!='odom' or child.lstrip('/')!=self.base_frame.lstrip('/'):
                reason='unexpected odometry frame: '+frame+' -> '+child
            elif not -.05<=now_ros-stamp<=self.max_age:
                reason=f'odometry stamp age {now_ros-stamp:.6f}s outside [-0.05,{self.max_age}]'
            elif math.hypot(vx,vy)>self.max_speed or abs(wz)>self.max_yaw_rate:
                reason='implausible odometry velocity'
            norm=math.sqrt(sum(v*v for v in quaternion))
            if not math.isfinite(norm) or norm<1e-6:reason='invalid odometry quaternion'
            if reason:
                self.rejected['odometry stamp age' if reason.startswith('odometry stamp age') else reason]+=1
                self.status=reason;self.window.clear();return False
            if self.sample and stamp<=self.sample['stamp']:
                self.rejected['duplicate or out-of-order odometry']+=1
                self.status='duplicate or out-of-order odometry';return False
            qx,qy,qz,qw=(v/norm for v in quaternion)
            yaw=math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
            if self.window and (receipt-self.window[-1][1]>.3 or stamp-self.window[-1][0]>.3):
                self.gap_resets+=1
                self.window.clear()
            self.sample=dict(stamp=stamp,receipt=receipt,x=x,y=y,yaw=yaw)
            self.window.append((stamp,receipt));self.status='ok';return True

    def _read_locked(self):
        # Sample both clocks only after acquiring the same lock as ingest().
        # An older caller-supplied time must never be compared with a newer sample.
        now_ros=self.ros_clock()
        now_mono=self.monotonic_clock()
        if self.sample is None:raise RuntimeError('Localization unavailable: '+self.status)
        p=dict(self.sample)
        age=now_ros-p['stamp'];receipt_age=now_mono-p['receipt']
        if not -.05<=age<=self.max_age or not 0<=receipt_age<=self.max_age:
            raise RuntimeError(f'Localization odometry stale: stamp_age_s={age:.6f}, '
                f'receipt_age_s={receipt_age:.6f}, stamp={p["stamp"]:.9f}, now={now_ros:.9f}, '
                f'last_status={self.status}')
        return p,dict(source='/odom',stamp=p['stamp'],stamp_age_s=age,
                      receipt_age_s=receipt_age,last_status=self.status)

    def read(self):
        with self.lock:
            return self._read_locked()

    def readiness_snapshot(self):
        """Read pose, freshness and continuity from one consistent locked snapshot."""
        with self.lock:
            details=dict(last_status=self.status,window_samples=len(self.window),
                         gap_resets=self.gap_resets,rejected=dict(self.rejected))
            try:p,diagnostic=self._read_locked()
            except RuntimeError as exc:
                return None,dict(details,ready=False,readiness_reason=str(exc))
            details.update(diagnostic)
            receipt_span=self.window[-1][1]-self.window[0][1] if self.window else 0.
            stamp_span=self.window[-1][0]-self.window[0][0] if self.window else 0.
            reasons=[]
            if self.status!='ok':reasons.append(self.status)
            if diagnostic['stamp_age_s']>.3:reasons.append('source stamp older than 0.3s')
            if diagnostic['receipt_age_s']>.3:reasons.append('no accepted message within 0.3s')
            if len(self.window)<3:reasons.append('fewer than 3 samples')
            if receipt_span<.4:reasons.append('receipt window shorter than 0.4s')
            if stamp_span<.35:reasons.append('source window shorter than 0.35s')
            return p,dict(details,ready=not reasons,readiness_reason='; '.join(reasons) or 'ok',
                          receipt_span_s=receipt_span,source_span_s=stamp_span)

    def ready(self):
        return self.readiness_snapshot()[1]['ready']


class PoseJumpGuard:
    """Compare fresh poses using their source timestamps, never polling intervals.

    The caller must still check freshness on every poll, including duplicates.
    Keep the existing 0.2 m margin; use the shared configured speed ceiling.
    """
    def __init__(self, max_speed_mps):
        self.max_speed_mps=float(max_speed_mps)
        if not math.isfinite(self.max_speed_mps) or self.max_speed_mps<=0:
            raise ValueError('Invalid shared speed ceiling for localization jump check')
        self.previous=None

    def update(self, sample):
        current={key:sample[key] for key in ('stamp','x','y')}
        if not all(math.isfinite(value) for value in current.values()):
            raise RuntimeError('Localization jump check received nonfinite pose')
        if self.previous is None:
            self.previous=current
            return True
        previous=self.previous
        dt=current['stamp']-previous['stamp']
        if dt<0:
            raise RuntimeError('Localization source time moved backwards: '
                f'previous_stamp={previous["stamp"]:.9f}, stamp={current["stamp"]:.9f}')
        if dt==0:
            return False
        distance=math.hypot(current['x']-previous['x'],current['y']-previous['y'])
        limit=.2+self.max_speed_mps*dt
        if distance>limit:
            raise RuntimeError('Localization jumped: '
                f'distance_m={distance:.6f}, limit_m={limit:.6f}, source_dt_s={dt:.6f}, '
                f'max_speed_mps={self.max_speed_mps:.6f}, '
                f'previous_stamp={previous["stamp"]:.9f}, stamp={current["stamp"]:.9f}, '
                f'previous_xy=({previous["x"]:.6f},{previous["y"]:.6f}), '
                f'xy=({current["x"]:.6f},{current["y"]:.6f})')
        self.previous=current
        return True


class StationaryGate:
    """Keep the existing stationary thresholds and expose each timer reset."""
    def __init__(self):
        self.since=None
        self.pose=None
        self.resets=0
        self.diagnostic={}

    def update(self, p, diagnostic, now):
        self.diagnostic=dict(diagnostic)
        if not diagnostic['ready']:
            if self.since is not None:self.resets+=1
            self.since=None;self.pose=None
            self.diagnostic.update(stationary_reason='localization not ready',stationary_resets=self.resets)
            return False
        current=(p['x'],p['y'],p['yaw'])
        distance=angle=0.
        reason='waiting for 0.4s stationary window'
        if self.pose is None:self.since=now;self.pose=current
        else:
            distance=math.hypot(current[0]-self.pose[0],current[1]-self.pose[1])
            angle=math.atan2(math.sin(current[2]-self.pose[2]),math.cos(current[2]-self.pose[2]))
            if distance>.03 or abs(angle)>.08:
                self.since=now;self.pose=current;self.resets+=1
                reason='pose moved beyond 0.03m or 0.08rad'
        elapsed=now-self.since
        self.diagnostic.update(stationary_reason='ok' if elapsed>=.4 else reason,
                               stationary_elapsed_s=elapsed,position_change_m=distance,
                               heading_change_rad=angle,stationary_resets=self.resets)
        return elapsed>=.4


class SessionLocalization:
    def __init__(self, settings, saved_map=False):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from session_odom_process import ProcessPoseInbox
        from tf2_ros import Buffer,TransformListener
        self.node=rclpy.create_node('s_curve_session_localization')
        self.inbox=ProcessPoseInbox(base_frame=settings.get('feedback_base_frame','base_footprint'),
            max_speed=settings.get('feedback_max_speed_mps',2.),
            max_yaw_rate=settings.get('feedback_max_yaw_rate_radps',6.),
            ros_clock=lambda:self.node.get_clock().now().nanoseconds/1e9,
            stationary=saved_map,
            use_sim_time=bool(self.node.get_parameter('use_sim_time').value))
        self.buffer=Buffer()
        # Bound transport backlog after startup bursts. The TF buffer still
        # keeps historical transforms; static transforms retain their own QoS.
        tf_qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.listener=TransformListener(self.buffer,self.node,qos=tf_qos)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.saved_monitor=None
        self.saved_alignment=None
        self.saved_subscriptions=[]
        self.saved_timers=[]
        self.latest_saved_scan=None
        self.pending_saved_scan=None
        self.last_saved_alignment_monotonic=-math.inf
        self.saved_scan_result=None
        self.saved_data_lock=threading.Lock()
        self.last_saved_scan_key=None
        self.base_frame=settings.get('feedback_base_frame','base_footprint')
        if saved_map:
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav_msgs.msg import OccupancyGrid
            from sensor_msgs.msg import LaserScan
            from saved_map_monitor import SavedMapPoseMonitor,ScanToMapAlignment
            self.saved_monitor=SavedMapPoseMonitor(
                ros_clock=lambda:self.node.get_clock().now().nanoseconds/1e9)
            self.saved_alignment=ScanToMapAlignment()
            retained=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.saved_subscriptions=[
                self.node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.capture_amcl,retained),
                self.node.create_subscription(OccupancyGrid,'/map',self.capture_saved_map,retained),
                self.node.create_subscription(LaserScan,'/scan',self.capture_saved_scan,qos)]
            # Normal matching is 5 Hz. Only a pending source-time TF uses
            # a 20 ms retry, so idle checks do not wake the executor at 50 Hz.
            self.saved_alignment_timer=self.node.create_timer(.2,self.capture_saved_alignment)
            self.saved_timers=[self.node.create_timer(.1,self.capture_global_tf),
                               self.saved_alignment_timer]
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,name='session-localization',daemon=True)
        self.thread.start()

    def snapshot(self):
        return self.inbox.read()

    def pose(self):
        p,_=self.snapshot();return p['x'],p['y'],p['yaw']

    def ready(self):
        return self.inbox.ready()

    def readiness_snapshot(self):
        return self.inbox.readiness_snapshot()

    @staticmethod
    def _tf_pose(transform):
        from saved_map_monitor import quaternion_yaw
        p=transform.transform.translation;q=transform.transform.rotation
        return [p.x,p.y,quaternion_yaw((q.x,q.y,q.z,q.w))]

    def capture_amcl(self,msg):
        from saved_map_monitor import quaternion_yaw
        p=msg.pose.pose.position;q=msg.pose.pose.orientation;cv=msg.pose.covariance
        if msg.header.frame_id.lstrip('/')!='map':
            self.saved_monitor.ingest_amcl(stamp=0.,pose=(math.nan,math.nan,math.nan),
                                          std=(math.nan,math.nan,math.nan))
            return
        try:
            self.saved_monitor.ingest_amcl(
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                pose=(p.x,p.y,quaternion_yaw((q.x,q.y,q.z,q.w))),
                std=[math.sqrt(cv[i]) if math.isfinite(cv[i]) and cv[i]>=0 else math.nan
                     for i in (0,7,35)])
        except ValueError as exc:
            self.saved_monitor.ingest_amcl(stamp=0.,pose=(math.nan,math.nan,math.nan),
                                          std=(math.nan,math.nan,math.nan))

    def capture_saved_map(self,msg):
        from saved_map_monitor import quaternion_yaw
        p=msg.info.origin.position;q=msg.info.origin.orientation
        with self.saved_data_lock:
            previous_version=(self.saved_alignment.grid or {}).get('version')
            try:
                if msg.header.frame_id.lstrip('/')!='map':
                    raise ValueError('unexpected static map frame: '+msg.header.frame_id)
                self.saved_alignment.update_map(width=msg.info.width,height=msg.info.height,
                    resolution=msg.info.resolution,origin=(p.x,p.y,quaternion_yaw((q.x,q.y,q.z,q.w))),
                    data=msg.data)
            except (ValueError,OverflowError) as exc:
                self.saved_alignment.grid=None;self.saved_alignment.occupied=set()
                self.saved_scan_result=None;self.last_saved_scan_key=None;self.pending_saved_scan=None
                self.saved_monitor.reset(invalidate_amcl=False)
                self.saved_monitor.ingest_alignment(dict(ready=False,
                    reason='invalid static map: '+str(exc),
                    scan_stamp=self.node.get_clock().now().nanoseconds/1e9),time.monotonic())
                return
            if previous_version!=self.saved_alignment.grid['version']:
                self.saved_scan_result=None;self.last_saved_scan_key=None;self.pending_saved_scan=None
                self.saved_monitor.reset(invalidate_amcl=False)
                self.saved_monitor.ingest_alignment(dict(ready=False,
                    reason='waiting for scan agreement with current static map',
                    scan_stamp=self.node.get_clock().now().nanoseconds/1e9),time.monotonic())

    def capture_saved_scan(self,msg):
        # Keep exactly one frame: slow processing cannot build a stale scan queue.
        with self.saved_data_lock:self.latest_saved_scan=(msg,time.monotonic())

    def capture_global_tf(self):
        from rclpy.time import Time
        try:
            pose=self.buffer.lookup_transform('map',self.base_frame,Time())
            offset=self.buffer.lookup_transform('map','odom',Time.from_msg(pose.header.stamp))
            self.saved_monitor.ingest_tf(stamp=pose.header.stamp.sec+pose.header.stamp.nanosec/1e9,
                pose=self._tf_pose(pose),map_from_odom=self._tf_pose(offset),
                map_odom_stamp=offset.header.stamp.sec+offset.header.stamp.nanosec/1e9)
        except Exception as exc:
            self.saved_monitor.ingest_error('map-frame transform unavailable: '+str(exc))

    def _schedule_saved_alignment(self, delay):
        self.saved_alignment_timer.timer_period_ns=int(delay*1e9)
        self.saved_alignment_timer.reset()

    def capture_saved_alignment(self):
        from rclpy.time import Time
        self._schedule_saved_alignment(.2)
        now_ros=self.node.get_clock().now().nanoseconds/1e9
        now=time.monotonic()
        with self.saved_data_lock:
            captured=self.pending_saved_scan
            if captured is not None:
                pending_msg,pending_receipt=captured
                pending_stamp=pending_msg.header.stamp.sec+pending_msg.header.stamp.nanosec/1e9
                if not (-.05<=now_ros-pending_stamp<=.3 and 0<=now-pending_receipt<=.3):
                    captured=None;self.pending_saved_scan=None
            if captured is None:
                captured=self.latest_saved_scan
                self.pending_saved_scan=captured
        if captured is None:return
        msg,receipt=captured
        key=(msg.header.stamp.sec,msg.header.stamp.nanosec,msg.header.frame_id)
        if key==self.last_saved_scan_key:
            self.pending_saved_scan=None
            return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        now_ros=self.node.get_clock().now().nanoseconds/1e9
        with self.saved_data_lock:
            try:
                transform=self.buffer.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp))
                p=transform.transform.translation;q=transform.transform.rotation
                laser_transform=dict(translation=(p.x,p.y,p.z),quaternion=(q.x,q.y,q.z,q.w))
                result=self.saved_alignment.evaluate(ranges=msg.ranges,angle_min=msg.angle_min,
                    angle_increment=msg.angle_increment,range_min=max(.1,msg.range_min),
                    range_max=msg.range_max,laser_transform=laser_transform,
                    scan_stamp=stamp,frame=msg.header.frame_id,now_ros=now_ros)
                self.last_saved_scan_key=key
                self.pending_saved_scan=None
                self.last_saved_alignment_monotonic=time.monotonic()
            except Exception as exc:
                self._schedule_saved_alignment(.02)
                # Keep this frame until its exact historical TF arrives. A new
                # frame would otherwise repeat the same future-extrapolation race.
                # Existing evidence is usable only within the unchanged 300 ms limit.
                if self.saved_scan_result is not None:
                    previous,previous_receipt=self.saved_scan_result
                    if (-.05<=now_ros-previous['scan_stamp']<=.3
                            and 0<=time.monotonic()-previous_receipt<=.3):return
                result=dict(ready=False,reason='scan-time map/laser transform unavailable: '+str(exc),
                    scan_stamp=stamp,frame=msg.header.frame_id,computed_at_ros=now_ros,
                    endpoints_map=[],endpoint_ranges=[],
                    map_version=(self.saved_alignment.grid or {}).get('version'))
            self.saved_scan_result=(result,receipt)
            self.saved_monitor.ingest_alignment(result,receipt)

    def stationary_readiness_snapshot(self):
        return self.inbox.stationary_readiness_snapshot()

    def refine_saved_map_prior(self,prior):
        from rclpy.time import Time
        try:
            with self.saved_data_lock:
                captured=self.latest_saved_scan;grid=self.saved_alignment.grid
            if captured is None or grid is None:
                return dict(used=False,reason='fresh scan or static map not available for seeding')
            msg,receipt=captured
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            now_ros=self.node.get_clock().now().nanoseconds/1e9
            stationary,_=self.stationary_readiness_snapshot()
            if not stationary or not (-.05<=now_ros-stamp<=.3 and 0<=time.monotonic()-receipt<=.3):
                return dict(used=False,reason='scan seeding requires current stationary odometry and a fresh raw scan')
            tf=self.buffer.lookup_transform(self.base_frame,msg.header.frame_id,Time())
            p,q=tf.transform.translation,tf.transform.rotation
            mounting=dict(translation=(p.x,p.y,p.z),quaternion=(q.x,q.y,q.z,q.w))
            scan=dict(ranges=msg.ranges,angle_min=msg.angle_min,angle_increment=msg.angle_increment,
                      range_min=msg.range_min,range_max=msg.range_max)
            from saved_scan_seed import refine_initial_prior
            result=refine_initial_prior(grid,scan,mounting,prior)
            with self.saved_data_lock:
                current_grid=self.saved_alignment.grid;latest=self.latest_saved_scan
            stationary,_=self.stationary_readiness_snapshot()
            if (not stationary or not current_grid or current_grid['version']!=grid['version']
                    or latest is None or time.monotonic()-latest[1]>.3):
                result.update(used=False,reason='stationary/map/scan evidence changed during seed computation')
            result['scan_stamp']=stamp
            return result
        except Exception as exc:
            return dict(used=False,reason='scan seed unavailable; original AMCL prior retained: '+str(exc))

    def saved_map_snapshot(self):
        if self.saved_monitor is None:
            return dict(ready=False,reason='saved-map monitor disabled',sample_count=0,
                        window_span_s=0.,observation_received=False)
        result=self.saved_monitor.snapshot()
        result['thread_alive']=self.thread.is_alive()
        if not result['thread_alive']:
            result.update(ready=False,reason='saved-map localization monitor thread exited')
        return result

    def reset_saved_map_stability(self,invalidate_amcl=True):
        if self.saved_monitor is not None:
            self.saved_monitor.reset(invalidate_amcl=invalidate_amcl)

    def saved_map_history(self):
        return self.saved_monitor.export_history() if self.saved_monitor else []

    def current_scan_evidence(self,max_range_m=2.5):
        """Latest bounded source-time projection for dynamic layer refresh checks."""
        with self.saved_data_lock:
            if self.saved_scan_result is None or self.saved_alignment.grid is None:return None
            result,receipt=self.saved_scan_result;grid=self.saved_alignment.grid
            if result.get('map_version')!=grid['version']:return None
            endpoints=result.get('endpoints_map',[])
            ranges=result.get('endpoint_ranges',[])
            chosen=[(p[:],r) for p,r in zip(endpoints,ranges) if .1<r<=max_range_m]
            source_age=self.node.get_clock().now().nanoseconds/1e9-result['scan_stamp']
            receipt_age=time.monotonic()-receipt
            ready=result['ready'] and -.05<=source_age<=.3 and 0<=receipt_age<=.3
            return dict(ready=ready,reason=result['reason'] if ready or not result['ready']
                        else 'scan projection is stale',
                        scan_stamp=result['scan_stamp'],receipt_monotonic=receipt,
                        scan_stamp_age_s=source_age,scan_receipt_age_s=receipt_age,
                        frame=result['frame'],endpoints_map=[p for p,_ in chosen],
                        endpoint_ranges=[r for _,r in chosen],map=grid,map_version=grid['version'])

    def close(self):
        self.inbox.close()
        self.executor.shutdown()
        self.thread.join(timeout=3)
        self.executor.remove_node(self.node)
        for timer in self.saved_timers:self.node.destroy_timer(timer)
        for subscription in self.saved_subscriptions:self.node.destroy_subscription(subscription)
        self.listener.unregister()
        self.node.destroy_node()
