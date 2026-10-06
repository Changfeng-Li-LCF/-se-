"""RViz observation on the passive recorder; no motion commands or new node."""
from collections import deque
import math
import time


class LiveTrajectory:
    def __init__(self, node, source_frame, target_frame):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        from rclpy.duration import Duration
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from tf2_ros import Buffer
        self.node = node
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.pose_type = PoseStamped
        self.path = Path()
        self.path.header.frame_id = target_frame
        self.publisher = node.create_publisher(Path, '/live_slam_actual_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        # Existing /tf callback feeds this short cache; no TransformListener,
        # extra subscription, process, thread, or extrapolation.
        self.buffer = Buffer(cache_time=Duration(seconds=2.)) if target_frame != source_frame else None
        self.pending = deque()
        self.changed = True
        self.accepted = self.skipped = self.published = 0
        self.max_publish_ms = 0.
        self.last_error = None
        self.timer = node.create_timer(.2, self.publish)

    def offer(self, row):
        if self.buffer is None:
            self.append(row, row['x'], row['y'], row['yaw'])
            return
        self.pending.append(dict(row))
        # Only RViz observations can be dropped; saved source poses are intact.
        while len(self.pending) > 40:
            self.pending.popleft(); self.skipped += 1
        self.drain()

    def capture_tf(self, msg):
        if self.buffer is None:return
        for tf in msg.transforms:
            if (tf.header.frame_id.lstrip('/') == self.target_frame.lstrip('/')
                    and tf.child_frame_id.lstrip('/') == self.source_frame.lstrip('/')):
                try:self.buffer.set_transform(tf, 'passive_recorder')
                except Exception as exc:self.last_error = str(exc)
        self.drain()

    def drain(self):
        from rclpy.time import Time
        from tf2_ros import TransformException
        now = self.node.get_clock().now().nanoseconds * 1e-9
        while self.pending:
            row = self.pending[0]
            try:
                tf = self.buffer.lookup_transform(self.target_frame, self.source_frame,
                        Time(nanoseconds=round(row['stamp'] * 1e9)))
            except TransformException as exc:
                self.last_error = str(exc)
                if now - row['stamp'] < .8:return
                self.pending.popleft(); self.skipped += 1
                continue
            self.pending.popleft()
            p,q = tf.transform.translation,tf.transform.rotation
            a = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            c,s = math.cos(a),math.sin(a)
            self.append(row, p.x+c*row['x']-s*row['y'],
                        p.y+s*row['x']+c*row['y'], a+row['yaw'])

    def append(self, row, x, y, yaw):
        from rclpy.time import Time
        pose = self.pose_type()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = Time(nanoseconds=round(row['stamp'] * 1e9)).to_msg()
        pose.pose.position.x,pose.pose.position.y = x,y
        pose.pose.orientation.z,pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > 3000:del self.path.poses[0]
        self.changed = True
        self.accepted += 1

    def publish(self):
        if not self.changed:return
        begin = time.monotonic()
        try:
            self.publisher.publish(self.path)
            self.changed = False
            self.published += 1
        except Exception as exc:self.last_error = str(exc)
        self.max_publish_ms = max(self.max_publish_ms, (time.monotonic()-begin)*1000)

    def snapshot(self):
        return dict(accepted=self.accepted, skipped=self.skipped, pending=len(self.pending),
                    published=self.published, max_publish_ms=self.max_publish_ms,
                    last_error=self.last_error, publish_hz=5., frame=self.target_frame)
