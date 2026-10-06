#!/usr/bin/env python3
"""Estimate odometry from the latest direct odom->base TF. Never publishes TF."""
import math
import time
from dataclasses import dataclass


@dataclass
class Estimate:
    stamp_ns: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


class VelocityEstimator:
    def __init__(self, max_age=0.25, max_gap=0.3, filter_tau=0.10,
                 max_linear_speed=2.0, max_yaw_rate=6.0):
        values = (max_age, max_gap, filter_tau, max_linear_speed, max_yaw_rate)
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError('Estimator limits must be positive and finite')
        self.max_age, self.max_gap, self.filter_tau = values[:3]
        self.max_linear_speed, self.max_yaw_rate = values[3:]
        self.last_now = None
        self.reset()

    def reset(self):
        self.previous = None
        self.filtered = None
        self.status = 'waiting for two fresh transforms'

    def update(self, stamp_ns, x, y, yaw, now_ns):
        if self.last_now is not None and now_ns < self.last_now:
            self.reset()
        self.last_now = now_ns
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            self.reset()
            self.status = 'non-finite transform'
            return None
        age = (now_ns - stamp_ns) * 1e-9
        if age > self.max_age or age < -0.05:
            self.reset()
            self.status = 'stale or future transform'
            return None
        current = (stamp_ns, x, y, yaw)
        if self.previous is None:
            self.previous = current
            return None
        previous = self.previous
        dt = (stamp_ns - previous[0]) * 1e-9
        if dt <= 0.0:
            self.status = 'duplicate or out-of-order transform'
            return None
        if dt < 0.001:
            return None  # Accumulate a useful interval without losing the baseline.
        self.previous = current
        if dt > self.max_gap:
            self.filtered = None
            self.status = 'transform gap: resetting velocity filter'
            return None
        vx_world = (x - previous[1]) / dt
        vy_world = (y - previous[2]) / dt
        angle = yaw - previous[3]
        wz = math.atan2(math.sin(angle), math.cos(angle)) / dt
        # Keep valid, fresh measurements flowing. Lateral consistency belongs
        # to the driver confidence gate; rejecting speed here turns a quality
        # issue into a feedback timeout. Legacy plausibility parameters remain
        # accepted for launch compatibility; no velocity magnitude/sign gate.
        # Restore the original order: project each raw velocity into the body
        # frame before filtering, so old world directions are not reprojected
        # using the current heading during a turn.
        cosine, sine = math.cos(yaw), math.sin(yaw)
        raw = (cosine * vx_world + sine * vy_world,
               -sine * vx_world + cosine * vy_world, wz)
        if not all(math.isfinite(v) for v in raw):
            self.filtered = None
            self.status = 'non-finite derived velocity'
            return None
        alpha = -math.expm1(-dt / self.filter_tau)
        self.filtered = raw if self.filtered is None else tuple(
            old + alpha * (new - old) for old, new in zip(self.filtered, raw))
        fx, fy, fw = self.filtered
        self.status = 'ok'
        return Estimate(stamp_ns, x, y, yaw, fx, fy, fw)


class LatestTransformSlot:
    """One pending direct transform. Never compose or replay a TF history."""
    def __init__(self, parent, child):
        self.parent, self.child = parent, child
        self.pending = None
        self.highest_stamp = None
        self.last_now = None

    def offer(self, transforms, now_ns):
        if self.last_now is not None and now_ns < self.last_now:
            self.pending = None
            self.highest_stamp = None
        self.last_now = now_ns
        for transform in transforms:
            if (transform.header.frame_id != self.parent or
                    transform.child_frame_id != self.child):
                continue
            stamp = transform.header.stamp.sec * 1000000000 + transform.header.stamp.nanosec
            # A malformed future timestamp must not prevent subsequent valid samples.
            if stamp > now_ns + 50000000:
                continue
            if self.highest_stamp is None or stamp > self.highest_stamp:
                self.pending = transform
                self.highest_stamp = stamp

    def take(self):
        latest, self.pending = self.pending, None
        return latest


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    class OdomBridge(Node):
        def __init__(self):
            super().__init__('cartographer_odom_bridge')
            def param(name, default):
                return self.declare_parameter(name, default).value
            self.odom_frame = param('odom_frame', 'odom')
            self.base_frame = param('base_frame', 'base_footprint')
            topic = param('odom_topic', '/odom')
            rate = param('publish_rate', 20.0)
            if not math.isfinite(rate) or rate <= 0.0 or rate > 200.0:
                raise ValueError('publish_rate must be in (0,200] Hz')
            if not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame:
                raise ValueError('Distinct odom and base frames are required')
            self.estimator = VelocityEstimator(
                max_age=param('max_transform_age', 0.25),
                max_gap=param('max_transform_gap', 0.3),
                filter_tau=param('velocity_filter_tau', 0.10),
                max_linear_speed=param('max_plausible_linear_speed', 2.0),
                max_yaw_rate=param('max_plausible_yaw_rate', 6.0))
            self.slot = LatestTransformSlot(self.odom_frame, self.base_frame)
            # Current Cartographer modes publish this edge directly. Only the newest
            # measurement is useful for speed control; reliable retransmission of old
            # TF and a historical transform graph are unnecessary on this input.
            tf_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
            self.subscription = self.create_subscription(TFMessage, '/tf', self.on_tf, tf_qos)
            self.publisher = self.create_publisher(Odometry, topic, 1)
            self.last_tick = time.monotonic()
            self.stats_start = self.last_tick
            self.max_tick_gap = self.max_tf_age = self.max_receive_age = 0.0
            self.published_count = self.received_count = 0
            self.last_direct_receive = None
            self.timer = self.create_timer(1.0 / rate, self.tick)
            self.get_logger().info(
                'direct-latest-tf-v2: direct odom->base TF, best_effort depth=1, '
                'single latest sample; original measurement stamps and body velocity filter retained')

        def on_tf(self, msg):
            now_ns = self.get_clock().now().nanoseconds
            for t in msg.transforms:
                if t.header.frame_id == self.odom_frame and t.child_frame_id == self.base_frame:
                    stamp_ns = t.header.stamp.sec * 1000000000 + t.header.stamp.nanosec
                    self.max_receive_age = max(self.max_receive_age, (now_ns-stamp_ns)*1e-9)
                    self.received_count += 1
                    self.last_direct_receive = time.monotonic()
            self.slot.offer(msg.transforms, now_ns)

        def tick(self):
            now = time.monotonic()
            self.max_tick_gap = max(self.max_tick_gap, now - self.last_tick)
            self.last_tick = now
            transform = self.slot.take()
            if transform is not None:
                self.update_odom(transform)
            elif self.last_direct_receive is None or now-self.last_direct_receive > self.estimator.max_age:
                self.estimator.reset()
                self.estimator.status = 'waiting for fresh direct odom->base TF'
            elapsed = time.monotonic() - self.stats_start
            if elapsed >= 5.0:
                self.get_logger().info(
                    f'Odom latency: window_s={elapsed:.2f} '
                    f'max_receive_age_ms={self.max_receive_age * 1000:.1f} '
                    f'max_tf_age_ms={self.max_tf_age * 1000:.1f} '
                    f'max_tick_gap_ms={self.max_tick_gap * 1000:.1f} '
                    f'direct_tf_hz={self.received_count / elapsed:.1f} '
                    f'published_hz={self.published_count / elapsed:.1f} '
                    f'status={self.estimator.status}')
                self.stats_start = time.monotonic()
                self.max_tf_age = self.max_receive_age = self.max_tick_gap = 0.0
                self.published_count = self.received_count = 0

        def update_odom(self, transform):
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
            tf_age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
            self.max_tf_age = max(self.max_tf_age, tf_age)
            if tf_age > 0.15:
                self.get_logger().warning(
                    f'Odom input delayed: tf_age_ms={tf_age * 1000:.1f}',
                    throttle_duration_sec=5.0)
            translation = transform.transform.translation
            q = transform.transform.rotation
            norm = math.hypot(q.x, q.y, q.z, q.w)
            if not math.isfinite(norm) or norm < 1e-6:
                self.estimator.reset()
                self.get_logger().warning('Invalid TF quaternion', throttle_duration_sec=5.0)
                return
            qx, qy, qz, qw = (v / norm for v in (q.x, q.y, q.z, q.w))
            yaw = math.atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0 * (qy*qy + qz*qz))
            estimate = self.estimator.update(stamp_ns, translation.x, translation.y, yaw,
                                             self.get_clock().now().nanoseconds)
            if estimate is None:
                if 'duplicate' not in self.estimator.status and 'waiting' not in self.estimator.status:
                    self.get_logger().warning(self.estimator.status, throttle_duration_sec=5.0)
                return
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = self.odom_frame
            msg.child_frame_id = self.base_frame
            msg.pose.pose.position.x, msg.pose.pose.position.y = estimate.x, estimate.y
            msg.pose.pose.orientation.z = math.sin(estimate.yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(estimate.yaw / 2.0)
            msg.twist.twist.linear.x, msg.twist.twist.linear.y = estimate.vx, estimate.vy
            msg.twist.twist.angular.z = estimate.wz
            for index in (0, 7, 35):
                msg.pose.covariance[index] = 0.04
                msg.twist.covariance[index] = 0.09
            for index in (14, 21, 28):
                msg.pose.covariance[index] = 1e6
                msg.twist.covariance[index] = 1e6
            self.publisher.publish(msg)
            self.published_count += 1

    rclpy.init(args=args)
    node = None
    try:
        node = OdomBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
