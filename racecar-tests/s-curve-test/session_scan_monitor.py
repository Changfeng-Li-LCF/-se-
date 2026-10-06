"""Independent /scan receipt monitoring; no motion commands or disk writes.

Receipt age uses a monotonic clock. Header age is diagnostic only: an old or
invalid source stamp is not called a missing scan. An armed receipt gap remains
latched even if scan messages resume before the session loop polls the monitor.
"""
from collections import deque
import math
import threading
import time


class ScanHealthState:
    """Small thread-safe inbox with bounded history and no ROS dependency."""

    def __init__(self, timeout_s=1.0, *, monotonic_clock=time.monotonic,
                 ros_clock=time.time, on_timeout=None, history_size=128):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('timeout_s must be finite and positive')
        if history_size < 1:
            raise ValueError('history_size must be positive')
        self.timeout_s = float(timeout_s)
        self.monotonic_clock = monotonic_clock
        self.ros_clock = ros_clock
        self.on_timeout = on_timeout
        self._lock = threading.Lock()
        self._recent = deque(maxlen=history_size)
        self._sample_count = 0
        self._last_receipt = None
        self._last_receipt_ros = None
        self._last_header = None
        self._last_valid_header = None
        self._last_interval = None
        self._max_interval = 0.0
        self._last_callback_stamp_age = None
        self._max_callback_stamp_age = None
        self._invalid_stamps = 0
        self._repeated_stamps = 0
        self._out_of_order_stamps = 0
        self._future_stamps = 0
        self._last_tick = None
        self._last_tick_gap = None
        self._max_tick_gap = 0.0
        self._last_heartbeat = None
        self._armed = False
        self._armed_at = None
        self._timeout_event = None
        self._monitor_error = None
        self._notification_errors = 0
        self._last_notification_error = None

    def _receipt_fresh_locked(self, now):
        return (self._last_receipt is not None
                and 0.0 <= now - self._last_receipt <= self.timeout_s)

    def _latch_locked(self, now, now_ros, source):
        if (not self._armed or self._timeout_event is not None
                or self._last_receipt is None
                or now - self._last_receipt <= self.timeout_s):
            return None
        event = dict(
            reason='scan_receipt_timeout', detected_by=source,
            detected_monotonic=now, detected_ros_s=now_ros,
            timeout_s=self.timeout_s, receipt_age_s=now - self._last_receipt,
            last_receipt_monotonic=self._last_receipt,
            last_receipt_ros_s=self._last_receipt_ros,
            last_header_stamp=self._last_header,
            last_callback_stamp_age_s=self._last_callback_stamp_age,
            sample_count=self._sample_count,
            last_monitor_tick_monotonic=self._last_tick,
            last_monitor_tick_gap_s=self._last_tick_gap,
            max_monitor_tick_gap_s=self._max_tick_gap,
        )
        self._timeout_event = event
        return dict(event)

    def _notify(self, event):
        # The callback is a lightweight session stop flag, never disk or ROS I/O.
        # It runs outside the state lock and cannot corrupt the latched evidence.
        if event is not None and self.on_timeout is not None:
            try:
                self.on_timeout(dict(event))
            except Exception as exc:
                with self._lock:
                    self._notification_errors += 1
                    self._last_notification_error = repr(exc)

    def update(self, header_stamp, *, received_monotonic=None, received_ros=None):
        """Capture receipt before any heavy work; only copy timing, never ranges."""
        with self._lock:
            now = (self.monotonic_clock() if received_monotonic is None
                   else received_monotonic)
            now_ros = self.ros_clock() if received_ros is None else received_ros
            # Check the old sample before replacing it. This catches a gap even
            # if the watchdog itself could not run while the process was delayed.
            event = self._latch_locked(now, now_ros, 'scan_callback_gap')
            interval = None if self._last_receipt is None else now - self._last_receipt
            if interval is not None:
                self._max_interval = max(self._max_interval, interval)
            self._last_interval = interval
            try:
                stamp = float(header_stamp)
                valid_stamp = math.isfinite(stamp) and stamp > 0.0
            except (TypeError, ValueError, OverflowError):
                stamp, valid_stamp = None, False
            if valid_stamp:
                if self._last_valid_header is not None:
                    if stamp == self._last_valid_header:
                        self._repeated_stamps += 1
                    elif stamp < self._last_valid_header:
                        self._out_of_order_stamps += 1
                self._last_valid_header = stamp
                stamp_age = now_ros - stamp
                if stamp_age < 0:
                    self._future_stamps += 1
                if (self._max_callback_stamp_age is None
                        or stamp_age > self._max_callback_stamp_age):
                    self._max_callback_stamp_age = stamp_age
            else:
                self._invalid_stamps += 1
                stamp, stamp_age = None, None
            self._sample_count += 1
            self._last_receipt = now
            self._last_receipt_ros = now_ros
            self._last_header = stamp
            self._last_callback_stamp_age = stamp_age
            self._last_heartbeat = now
            self._recent.append(dict(
                sequence=self._sample_count, receipt_monotonic=now,
                receipt_ros_s=now_ros, header_stamp=stamp,
                callback_stamp_age_s=stamp_age, interval_s=interval,
            ))
        self._notify(event)

    def tick(self):
        """Called independently of the session executor at least every 50 ms."""
        with self._lock:
            now, now_ros = self.monotonic_clock(), self.ros_clock()
            gap = None if self._last_tick is None else now - self._last_tick
            self._last_tick_gap = gap
            if gap is not None:
                self._max_tick_gap = max(self._max_tick_gap, gap)
            self._last_tick = now
            self._last_heartbeat = now
            event = self._latch_locked(now, now_ros, 'monitor_tick')
        self._notify(event)

    def arm(self):
        with self._lock:
            now = self.monotonic_clock()
            if self._monitor_error is not None or not self._receipt_fresh_locked(now):
                return False
            # Repeated arm() during one run must not clear a latched timeout.
            if self._armed:
                return self._timeout_event is None
            self._timeout_event = None
            self._armed = True
            self._armed_at = now
            return True

    def disarm(self):
        with self._lock:
            self._armed = False

    def mark_error(self, error):
        with self._lock:
            self._monitor_error = str(error)

    def snapshot(self, include_recent=False):
        with self._lock:
            now, now_ros = self.monotonic_clock(), self.ros_clock()
            fresh = self._receipt_fresh_locked(now)
            latched = self._timeout_event is not None
            healthy = fresh and not latched and self._monitor_error is None
            if self._monitor_error is not None:
                status = 'monitor_error'
            elif latched:
                status = 'timeout_latched'
            elif self._last_receipt is None:
                status = 'waiting_for_scan'
            elif not fresh:
                status = 'stale'
            else:
                status = 'healthy'
            result = dict(
                topic='/scan', timeout_s=self.timeout_s, status=status,
                healthy=healthy, receipt_fresh=fresh, armed=self._armed,
                armed_at_monotonic=self._armed_at, latched_timeout=latched,
                timeout_event=None if not latched else dict(self._timeout_event),
                sample_count=self._sample_count,
                last_receipt_monotonic=self._last_receipt,
                last_receipt_ros_s=self._last_receipt_ros,
                last_header_stamp=self._last_header,
                receipt_age_s=None if self._last_receipt is None else now-self._last_receipt,
                header_age_s=None if self._last_header is None else now_ros-self._last_header,
                last_interval_s=self._last_interval,
                max_interval_s=self._max_interval,
                last_callback_stamp_age_s=self._last_callback_stamp_age,
                max_callback_stamp_age_s=self._max_callback_stamp_age,
                invalid_header_count=self._invalid_stamps,
                repeated_header_count=self._repeated_stamps,
                out_of_order_header_count=self._out_of_order_stamps,
                future_header_count=self._future_stamps,
                last_monitor_tick_monotonic=self._last_tick,
                last_monitor_tick_gap_s=self._last_tick_gap,
                max_monitor_tick_gap_s=self._max_tick_gap,
                thread_heartbeat_monotonic=self._last_heartbeat,
                thread_heartbeat_age_s=(None if self._last_heartbeat is None
                                        else now-self._last_heartbeat),
                monitor_error=self._monitor_error,
                notification_error_count=self._notification_errors,
                last_notification_error=self._last_notification_error,
            )
            if include_recent:
                result['recent_receipts'] = [dict(item) for item in self._recent]
            return result

    def ready(self):
        return self.snapshot()['healthy']


class SessionScanMonitor:
    """Own ROS node/executor/thread; shares no callbacks with the session node."""

    def __init__(self, timeout_s=1.0, *, context=None, on_timeout=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import LaserScan

        self._stop = threading.Event()
        self._closed = False
        self.node = rclpy.create_node('session_scan_monitor', context=context)
        self.state = ScanHealthState(
            timeout_s=timeout_s, on_timeout=on_timeout,
            ros_clock=lambda: self.node.get_clock().now().nanoseconds / 1e9,
        )
        self.subscription = self.node.create_subscription(
            LaserScan, '/scan', self._capture,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE),
        )
        self.executor = SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(
            target=self._spin, name='session-scan-monitor', daemon=True,
        )
        self.thread.start()

    def _capture(self, msg):
        self.state.update(
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            received_monotonic=time.monotonic(),
            received_ros=self.node.get_clock().now().nanoseconds / 1e9,
        )

    def _spin(self):
        try:
            # A finite wall-clock wait provides a watchdog even with no scans.
            # Do not use a ROS-time timer: clock changes must not affect timeout.
            while not self._stop.is_set():
                if not self.node.context.ok():
                    if not self._stop.is_set():
                        self.state.mark_error('ROS context shut down')
                    break
                self.executor.spin_once(timeout_sec=0.05)
                if not self._stop.is_set():
                    self.state.tick()
        except Exception as exc:
            if not self._stop.is_set():
                self.state.mark_error(repr(exc))

    def snapshot(self, include_recent=False):
        result = self.state.snapshot(include_recent=include_recent)
        result.update(thread_alive=self.thread.is_alive(), closed=self._closed)
        return result

    def ready(self):
        return self.thread.is_alive() and self.state.ready()

    def arm(self):
        return self.thread.is_alive() and self.state.arm()

    def disarm(self):
        self.state.disarm()

    def close(self):
        if self._closed:
            return
        self.disarm()
        self._stop.set()
        self.executor.wake()
        self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            # Never destroy a node while its callback could still be using it.
            self.state.mark_error('scan monitor thread failed to stop')
            raise RuntimeError('scan monitor thread failed to stop within 3 seconds')
        self.executor.shutdown(timeout_sec=1.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self._closed = True
