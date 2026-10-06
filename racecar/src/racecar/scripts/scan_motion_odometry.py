#!/usr/bin/env python3
"""Independent scan/gyro odometry. No Cartographer pose, TF, or command input."""
import json
import math
import time
import copy
import threading
import ctypes
import os
from collections import deque
from dataclasses import dataclass

import numpy as np
from racecar.scan_motion import match_scans, scan_points


def twist_displacement(vx, vy, yaw, dt):
    a = 1.0 if abs(yaw) < 1e-8 else math.sin(yaw) / yaw
    b = yaw / 2.0 if abs(yaw) < 1e-8 else (1.0 - math.cos(yaw)) / yaw
    return dt * np.array((a * vx - b * vy, b * vx + a * vy))


def displacement_twist(dx, dy, yaw, dt):
    a = 1.0 if abs(yaw) < 1e-8 else math.sin(yaw) / yaw
    b = yaw / 2.0 if abs(yaw) < 1e-8 else (1.0 - math.cos(yaw)) / yaw
    scale = dt * (a*a + b*b)
    return ((a * dx + b * dy) / scale, (-b * dx + a * dy) / scale)


class GyroHistory:
    """Two seconds of stamped, integrated gyro; no absolute IMU yaw needed."""
    def __init__(self, axis, bias, frame, max_rate, max_gap):
        self.axis = np.asarray(axis, dtype=float)
        if self.axis.shape != (3,) or not np.isfinite(self.axis).all() or np.linalg.norm(self.axis) < 1e-6:
            raise ValueError('Invalid shared IMU yaw axis')
        self.axis /= np.linalg.norm(self.axis)
        self.bias, self.frame = bias, frame
        self.max_rate, self.max_gap = max_rate, max_gap
        self.data = deque(maxlen=512)
        self.generation = 0

    def add(self, stamp, xyz, frame):
        w = float(np.dot(self.axis, xyz)) - self.bias
        if frame != self.frame or not math.isfinite(w) or abs(w) > self.max_rate:
            return False
        if self.data and stamp <= self.data[-1][0]:
            return False
        angle = 0.0
        if self.data:
            old_stamp, old_w, old_angle = self.data[-1]
            dt = stamp - old_stamp
            if dt > self.max_gap:
                self.data.clear()
                self.generation += 1
            else:
                angle = old_angle + 0.5 * (w + old_w) * dt
        self.data.append((stamp, w, angle))
        while len(self.data) > 2 and stamp - self.data[0][0] > 2.0:
            self.data.popleft()
        return True

    def angles(self, stamps):
        if len(self.data) < 2:
            return None
        values = np.asarray(self.data)
        if stamps[0] < values[0, 0] or stamps[-1] > values[-1, 0]:
            return None
        return np.interp(stamps, values[:, 0], values[:, 2])

    def coverage(self, start, end):
        # Endpoint checks need no array copy/interpolation on every IMU wake-up.
        if len(self.data) < 2:
            return 'insufficient_samples'
        if start < self.data[0][0]:
            return 'history_starts_after_scan'
        if end > self.data[-1][0]:
            return 'waiting_for_scan_end'
        return 'covered'

    def snapshot(self):
        frozen = copy.copy(self)
        frozen.data = (self.data.copy() if isinstance(self.data, np.ndarray)
                       else deque(self.data, maxlen=self.data.maxlen))
        return frozen


class NativeImuReceiver:
    """C++ owns ingestion/integration; Python copies stamped history per scan.

    ctypes releases the GIL while reading/waiting. No per-IMU Python callback,
    interprocess message queue, or republished IMU timestamp is involved.
    """
    def __init__(self, gyro, topic, node_name, namespace, simulated=False):
        from ament_index_python.packages import get_package_prefix
        path = os.environ.get('SCAN_MOTION_IMU_LIBRARY') or os.path.join(
            get_package_prefix('racecar'), 'lib', 'libscan_motion_imu_runtime.so')
        self.library = lib = ctypes.CDLL(path)
        pointer = ctypes.POINTER(ctypes.c_double)
        lib.scan_motion_imu_abi.restype = ctypes.c_uint
        if lib.scan_motion_imu_abi() != 1:
            raise RuntimeError('Unsupported native IMU history ABI')
        lib.scan_motion_imu_create.argtypes = [ctypes.c_char_p]*3 + [pointer,
            ctypes.c_double, ctypes.c_char_p, ctypes.c_double, ctypes.c_double,
            ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t]
        lib.scan_motion_imu_create.restype = ctypes.c_void_p
        lib.scan_motion_imu_snapshot.argtypes = [ctypes.c_void_p, pointer, pointer]
        lib.scan_motion_imu_snapshot.restype = ctypes.c_int
        lib.scan_motion_imu_wait.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double]
        lib.scan_motion_imu_wait.restype = None
        lib.scan_motion_imu_destroy.argtypes = [ctypes.c_void_p]
        lib.scan_motion_imu_destroy.restype = None
        self.gyro = gyro
        self.rows = np.empty((512, 3), dtype=np.float64)
        self.meta = np.empty(6, dtype=np.float64)
        self.rows_pointer = self.rows.ctypes.data_as(pointer)
        self.meta_pointer = self.meta.ctypes.data_as(pointer)
        error = ctypes.create_string_buffer(1024)
        self.handle = lib.scan_motion_imu_create(topic.encode(), node_name.encode(),
            namespace.encode(), gyro.axis.ctypes.data_as(pointer), gyro.bias,
            gyro.frame.encode(), gyro.max_rate, gyro.max_gap, int(simulated), error, len(error))
        if not self.handle:
            raise RuntimeError('Native IMU receiver failed: '+error.value.decode(errors='replace'))

    def snapshot(self):
        count = self.library.scan_motion_imu_snapshot(self.handle, self.rows_pointer, self.meta_pointer)
        if count < 0:
            raise RuntimeError('Native IMU executor stopped unexpectedly')
        gyro = copy.copy(self.gyro)
        # One contiguous copy, never hundreds of individually boxed NumPy scalars.
        gyro.data = self.rows[:count].copy()
        gyro.generation = int(self.meta[0])
        return gyro, self.meta.copy()

    def wait(self, sequence, seconds):
        self.library.scan_motion_imu_wait(self.handle, sequence, max(0.0, min(seconds, 0.05)))

    def close(self):
        if self.handle:
            self.library.scan_motion_imu_destroy(self.handle)
            self.handle = None


@dataclass
class Motion:
    stamp: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float
    variance: float
    valid: bool
    reason: str
    accepted_stamp: float
    rmse: float = math.inf
    overlap: float = 0.0
    condition: float = math.inf
    processing_ms: float = 0.0


class ScanMotion:
    def __init__(self, gyro, max_gap=0.3, sensor_offset=(0.07, 0.0), max_points=180):
        self.gyro, self.max_gap = gyro, max_gap
        self.sensor_offset, self.max_points = sensor_offset, max_points
        self.previous = None
        self.pose = np.zeros(3)
        self.velocity = (0.0, 0.0)
        self.accepted_stamp = 0.0
        self.last_stamp = 0.0
        self.gyro_generation = gyro.generation

    @staticmethod
    def end_stamp(scan):
        n = len(scan.ranges)
        # The next-ray boundary is strictly after Cartographer's last valid ray.
        duration = n * scan.time_increment if scan.time_increment > 0 else 0.0
        return scan.stamp + duration

    def ready(self, scan):
        return self.gyro.angles(np.array((scan.stamp, self.end_stamp(scan)))) is not None

    def update(self, scan):
        started = time.perf_counter()
        end = self.end_stamp(scan)
        if end <= self.last_stamp:
            return None
        stamp_delta = end - self.last_stamp if self.last_stamp else 0.0
        if self.gyro_generation != self.gyro.generation:
            # A gyro-history gap resets its relative integration origin.
            # Rebaseline, never interpret that origin reset as a physical turn.
            self.previous = None
            self.last_gyro_angle = None
            self.gyro_generation = self.gyro.generation
        n = len(scan.ranges)
        ray_times = scan.stamp + np.arange(n) * scan.time_increment
        queries = np.append(ray_times, end)
        angles = self.gyro.angles(queries) if n else None
        fresh_hint = self.accepted_stamp > 0 and end - self.accepted_stamp <= self.max_gap
        hint_velocity = self.velocity if fresh_hint else None
        points = None
        result = None
        valid, reason, variance = False, 'waiting_for_two_scans', 1e6
        gyro_angle = float(angles[-1]) if angles is not None else None
        if angles is None:
            reason = 'imu_scan_interval_not_covered'
            self.previous = None
        elif scan.frame != 'laser_link' or scan.time_increment < 0:
            reason = 'invalid_scan_frame_or_timing'
            self.previous = None
        else:
            points = scan_points(
                scan.ranges, scan.angle_min, scan.angle_increment,
                range_min=max(0.22, scan.range_min), range_max=min(8.0, scan.range_max),
                time_increment=scan.time_increment,
                yaw_offsets=angles[:-1]-angles[-1], velocity_hint=hint_velocity,
                reference_time_s=n*scan.time_increment, sensor_offset_xy=self.sensor_offset,
                max_points=self.max_points)
            if self.previous is not None:
                old_end, old_points, old_angle = self.previous
                dt = end - old_end
                if 0.001 < dt <= self.max_gap:
                    yaw_hint = gyro_angle - old_angle
                    translation = (twist_displacement(*self.velocity, yaw_hint, dt)
                                   if fresh_hint else None)
                    result = match_scans(old_points, points, yaw_hint, translation)
                    valid, reason = result.valid, result.reason
                    if valid:
                        vx, vy = displacement_twist(result.dx, result.dy, result.dyaw, dt)
                        self.velocity = (vx, vy)
                        self.accepted_stamp = end
                        # Conservative residual-based uncertainty proxy, not wheel truth.
                        variance = (result.rmse / dt)**2 * (1.0 + result.condition/100.0)
                        step, yaw_step = np.array((result.dx, result.dy)), result.dyaw
                else:
                    reason = 'scan_gap_rebaseline'
            self.previous = (end, points, gyro_angle)
        if not valid:
            # Cartographer's mandatory odom queue must not starve on a bad fit.
            # Propagate only the recent credible scan estimate; mark this as prediction.
            yaw_step = (gyro_angle - self.last_gyro_angle
                        if gyro_angle is not None and getattr(self, 'last_gyro_angle', None) is not None
                        else 0.0)
            velocity = self.velocity if fresh_hint else (0.0, 0.0)
            step = twist_displacement(*velocity, yaw_step, stamp_delta)
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        self.pose[:2] += (c*step[0]-s*step[1], s*step[0]+c*step[1])
        self.pose[2] = math.remainder(self.pose[2]+yaw_step, 2.0*math.pi)
        self.last_stamp, self.last_gyro_angle = end, gyro_angle
        # Invalid frames carry only held values plus invalid covariance, never accepted as speed.
        vx, vy = self.velocity
        wz = yaw_step / stamp_delta if stamp_delta > 0 else 0.0
        motion = Motion(end, *self.pose, vx, vy, wz, variance, valid, reason, self.accepted_stamp)
        if result is not None:
            motion.rmse, motion.overlap, motion.condition = result.rmse, result.overlap, result.condition
        motion.processing_ms = (time.perf_counter()-started)*1000
        return motion


class LatestScanWorker:
    """One local compute worker; native IMU ingestion runs independently of GIL."""
    def __init__(self, gyro, max_gap, publish, failed, imu_receiver, clock_now):
        self.gyro, self.max_gap = gyro, max_gap
        self.imu_receiver = imu_receiver
        self.clock_now = clock_now
        self.publish, self.failed = publish, failed
        self.condition = threading.Condition()
        self.publish_lock = threading.Lock()
        # Age, not a two-frame count, determines whether a frame is useful.
        # The count bound only prevents unbounded allocation on invalid input.
        self.pending = deque(maxlen=32)
        self.scan_period_s = max_gap / 3.0
        self.last_scan_end = None
        self.expired_scans = 0
        self.closed = False
        self.epoch = 0
        self.last_ros = 0.0
        self.received = self.coalesced = self.processed = 0
        self.imu_accepted = self.imu_rejected = self.imu_resets = 0
        self.last_imu_callback_ros = 0.0
        self.engine = None
        self.engine_epoch = -1
        self.thread = threading.Thread(target=self.run, name='scan-registration', daemon=True)

    def start(self):
        self.thread.start()

    def check_clock(self, now):
        # Called only while holding condition. Drop work from a previous clock epoch.
        if now < self.last_ros:
            self.pending.clear()
            self.last_scan_end = None
            self.epoch += 1
        self.last_ros = now

    def scan(self, scan, ros_now):
        with self.condition:
            if self.closed:
                return
            self.check_clock(self.clock_now())
            self.received += 1
            end = ScanMotion.end_stamp(scan)
            if self.last_scan_end is not None and 0.001 < end-self.last_scan_end < self.max_gap:
                self.scan_period_s = 0.8*self.scan_period_s + 0.2*(end-self.last_scan_end)
            self.last_scan_end = end
            if len(self.pending) == self.pending.maxlen:
                self.coalesced += 1
            self.pending.append((scan, time.monotonic(), ros_now))
            self.condition.notify()

    def select_scan(self, now):
        """Called under condition; return (index, wait), or no work.

        Ready data is never invalidated by a forecast of compute cost. Only
        already stale scans are discarded. Skipping queued scans does not touch
        the engine baseline: the next aligned scan can match across that gap.
        Missing IMU may wait for one IMU freshness interval (bounded by half the
        speed freshness interval); thereafter the existing invalid odom fallback
        advances Cartographer. Such prediction is never valid velocity feedback.
        """
        while self.pending:
            scan, received, received_ros = self.pending[0]
            if received_ros+now-received-ScanMotion.end_stamp(scan) <= self.max_gap:
                break
            self.pending.popleft()
            self.coalesced += 1
            self.expired_scans += 1
        sync_wait = min(self.gyro.max_gap, self.max_gap/2.0)
        expired = None
        earliest_remaining = self.max_gap
        for index in range(len(self.pending)-1, -1, -1):
            scan, received, received_ros = self.pending[index]
            end = ScanMotion.end_stamp(scan)
            age = received_ros+now-received-end
            if age > self.max_gap:
                continue  # A late, out-of-order scan can be behind a newer head.
            coverage = self.gyro.coverage(scan.stamp, end)
            if coverage == 'covered':
                return index, 0.0
            # A history head newer than the scan cannot recover by waiting.
            if (age >= sync_wait or coverage == 'history_starts_after_scan') and expired is None:
                expired = index
            earliest_remaining = min(earliest_remaining, max(0.0, sync_wait-age))
        if expired is not None:
            return expired, 0.0
        return None, earliest_remaining if self.pending else 0.05

    def run(self):
        try:
            while True:
                gyro, meta = self.imu_receiver.snapshot()
                with self.condition:
                    if self.closed:
                        return
                    self.gyro = gyro
                    self.imu_accepted, self.imu_rejected, self.imu_resets = map(int, meta[1:4])
                    self.last_imu_callback_ros = float(meta[4])
                    if not self.pending:
                        self.condition.wait(timeout=0.05)
                        continue
                    index, wait_s = self.select_scan(time.monotonic())
                if index is None:
                    # Native IMU ingestion wakes this wait without a Python callback.
                    self.imu_receiver.wait(float(meta[5]), wait_s)
                    continue
                with self.condition:
                    # Scan callbacks only append; select again under the same lock
                    # as removal, since a new scan may now be aligned and preferable.
                    index, wait_s = self.select_scan(time.monotonic())
                    if index is None:
                        continue
                    scan, received, received_ros = self.pending[index]
                    end = ScanMotion.end_stamp(scan)
                    coverage = self.gyro.coverage(scan.stamp, end)
                    covered = coverage == 'covered'
                    # Earlier frames cannot be used after processing this stamp.
                    self.coalesced += index
                    for _ in range(index + 1):
                        self.pending.popleft()
                    # NativeImuReceiver already returned an immutable per-read
                    # snapshot. Share that compact array with this worker.
                    gyro, epoch = self.gyro, self.epoch
                    received_count, coalesced_count = self.received, self.coalesced
                    first_imu = gyro.data[0][0] if len(gyro.data) else None
                    last_imu = gyro.data[-1][0] if len(gyro.data) else None
                    last_imu_callback_ros = self.last_imu_callback_ros
                    imu_details = {
                        'imu_coverage_reason': coverage,
                        'scan_start_stamp': scan.stamp,
                        'scan_end_stamp': end,
                        'imu_history_size': len(gyro.data),
                        'imu_history_first_stamp': first_imu,
                        'imu_history_last_stamp': last_imu,
                        'imu_history_generation': gyro.generation,
                        'imu_start_gap_ms': None if first_imu is None else max(0.0, first_imu-scan.stamp)*1000,
                        'imu_end_gap_ms': None if last_imu is None else max(0.0, end-last_imu)*1000,
                        'imu_samples_accepted': self.imu_accepted,
                        'imu_samples_rejected': self.imu_rejected,
                        'imu_history_resets': self.imu_resets,
                        'pending_scans': len(self.pending),
                        'scan_freshness_limit_ms': self.max_gap*1000,
                        'estimated_scan_period_ms': self.scan_period_s*1000,
                        'scans_expired_before_dispatch': self.expired_scans,
                        'imu_receiver': 'native_integrated_history',
                    }
                if self.engine is None or self.engine_epoch != epoch:
                    self.engine = ScanMotion(gyro, max_gap=self.max_gap)
                    self.engine_epoch = epoch
                else:
                    self.engine.gyro = gyro
                compute_started, cpu_started = time.monotonic(), time.thread_time()
                # match_scans uses ctypes.CDLL: native ICP releases the GIL.
                # IMU reception/integration is entirely native even while Python
                # assembles scan points. No pickle, Pipe, or child wake-up exists.
                motion = self.engine.update(scan)
                compute_finished = time.monotonic()
                compute_timing = {
                    'compute_mode': 'local_native_worker',
                    'compute_cpu_ms': (time.thread_time()-cpu_started)*1000,
                    'compute_wall_ms': (compute_finished-compute_started)*1000,
                }
                if motion is None:
                    continue
                self.processed += 1
                details = {
                    'callback_to_compute_ms': (compute_started-received)*1000,
                    'source_age_at_callback_ms': (received_ros-motion.stamp)*1000,
                    'imu_interval_covered': covered,
                    'scan_callbacks': received_count,
                    'scans_coalesced': coalesced_count,
                    'scans_processed': self.processed,
                    'imu_callback_age_at_compute_ms': None if not last_imu_callback_ros else
                        max(0.0, received_ros+compute_started-received-last_imu_callback_ros)*1000,
                    'imu_measurement_age_at_compute_ms': None if last_imu is None else
                        (received_ros+compute_started-received-last_imu)*1000,
                }
                details.update(compute_timing)
                details.update(imu_details)
                with self.publish_lock:
                    with self.condition:
                        self.check_clock(self.clock_now())
                        if self.closed or epoch != self.epoch:
                            continue
                    self.publish(motion, details)
        except Exception as error:
            with self.publish_lock:
                with self.condition:
                    was_closed = self.closed
                    self.closed = True
                    self.condition.notify_all()
                if not was_closed:
                    self.failed(error)

    def stop(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        with self.publish_lock:
            pass
        if self.thread.is_alive():
            self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            raise RuntimeError('Scan worker did not exit; native IMU history remains owned by it')
        self.imu_receiver.close()


def main(args=None):
    import yaml
    from racecar.scan_motion import require_native
    require_native()  # Fail explicitly rather than run the slow NumPy fallback on the car.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from types import SimpleNamespace

    class ScanMotionNode(Node):
        def __init__(self):
            super().__init__('scan_motion_odometry')
            def param(name, default):
                return self.declare_parameter(name, default).value
            config = param('driver_config', '')
            with open(config, encoding='utf8') as handle:
                driver = yaml.safe_load(handle)['racecar_driver']['ros__parameters']
            gyro = GyroHistory(driver['imu_yaw_axis'], driver['imu_yaw_bias_radps'],
                               driver['imu_feedback_frame'], driver['imu_max_yaw_rate_radps'],
                               driver['imu_feedback_timeout_s'])
            self.feedback = self.create_publisher(Odometry, '/scan_motion_feedback', 1)
            self.odometry = self.create_publisher(Odometry, '/scan_motion_odom', 1)
            self.status = self.create_publisher(String, '/scan_motion/status', 1)
            self.worker_error = None
            self.failure_guard = self.create_guard_condition(self.raise_worker_error)
            receiver = NativeImuReceiver(gyro, self.resolve_topic_name(driver['imu_feedback_topic']),
                self.get_name()+'_imu_receiver', self.get_namespace(),
                bool(self.get_parameter('use_sim_time').value))
            try:
                self.worker = LatestScanWorker(gyro, driver['feedback_timeout_s'], self.publish_motion,
                    self.worker_failed, receiver, lambda: self.get_clock().now().nanoseconds*1e-9)
                self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                self.last_log = 0.0
                self.last_status = 0.0
                self.worker.start()
            except Exception:
                if hasattr(self, 'worker'):
                    self.worker.stop()
                else:
                    receiver.close()
                raise
            self.get_logger().info('scan-motion-v6: native IMU integration and local native matching; no IPC or forecast-based frame rejection; no SLAM/TF input')

        def on_scan(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            scan = SimpleNamespace(stamp=stamp, ranges=msg.ranges, angle_min=msg.angle_min,
                           angle_increment=msg.angle_increment, time_increment=msg.time_increment,
                            range_min=msg.range_min, range_max=msg.range_max, frame=msg.header.frame_id)
            self.worker.scan(scan, self.get_clock().now().nanoseconds*1e-9)

        def worker_failed(self, error):
            self.worker_error = error
            self.failure_guard.trigger()

        def raise_worker_error(self):
            raise RuntimeError('Scan registration worker failed') from self.worker_error

        def publish_motion(self, motion, timing):
            msg = Odometry()
            ns = round(motion.stamp*1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1000000000)
            msg.header.frame_id = 'scan_motion_odom'
            msg.child_frame_id = 'base_footprint'
            msg.pose.pose.position.x, msg.pose.pose.position.y = float(motion.x), float(motion.y)
            msg.pose.pose.orientation.z = math.sin(motion.yaw/2)
            msg.pose.pose.orientation.w = math.cos(motion.yaw/2)
            msg.twist.twist.linear.x, msg.twist.twist.linear.y = float(motion.vx), float(motion.vy)
            msg.twist.twist.angular.z = float(motion.wz)
            for i in (0, 7, 35):
                msg.twist.covariance[i] = float(motion.variance)
                msg.pose.covariance[i] = 0.04 if motion.valid else 1e6
            for i in (14, 21, 28):
                msg.twist.covariance[i] = msg.pose.covariance[i] = 1e6
            self.feedback.publish(msg)
            self.odometry.publish(msg)
            detail = motion.__dict__.copy()
            detail.update(timing)
            # Sample AFTER registration/publication; do not hide computation in the age field.
            detail['source_age_ms'] = (self.get_clock().now().nanoseconds*1e-9-motion.stamp)*1000
            detail['pose_is_prediction'] = not motion.valid
            detail = {k: (float(v) if isinstance(v, np.floating) else v) for k, v in detail.items()}
            detail = {k: (None if isinstance(v, float) and not math.isfinite(v) else v) for k, v in detail.items()}
            if time.monotonic()-self.last_status >= 0.2:
                self.status.publish(String(data=json.dumps(detail)))
                self.last_status = time.monotonic()
            if time.monotonic()-self.last_log >= 5.0:
                self.get_logger().info('scan motion: '+json.dumps(detail))
                self.last_log = time.monotonic()

    rclpy.init(args=args)
    node = None
    try:
        node = ScanMotionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.worker.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
