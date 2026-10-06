"""Read-only odometry intake, isolated from the session's Python/TF workload."""
from collections import Counter, deque
import json
import multiprocessing
import os
import threading
import time

from session_localization import PoseInbox, StationaryGate


def _publish_state(slot, state):
    payload = json.dumps(state, separators=(',', ':'), allow_nan=False).encode('utf-8')
    if len(payload) > len(slot['data']):
        raise RuntimeError('Odometry receiver state exceeds shared slot capacity')
    # Serialize before locking. No pipe/queue feeder can accumulate old samples.
    with slot['lock']:
        slot['data'][:len(payload)] = payload
        slot['length'].value = len(payload)
        slot['sequence'].value += 1


def _receive_odom(slot, stop, settings, stationary, use_sim_time, parent_pid):
    # Spawn gives this subscriber its own interpreter, ROS context and executor.
    # The child never creates publishers, action clients or vehicle services.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry

    node = executor = None
    phase = 'initializing'
    callbacks = accepted = 0
    last_callback = last_callback_ros = last_header = None
    heartbeat = time.monotonic()
    inbox = None
    gate = StationaryGate() if stationary else None
    stationary_ready = False

    def publish(error=None):
        _publish_state(slot, dict(
            phase=phase, error=error, heartbeat=heartbeat,
            callback_count=callbacks, accepted_count=accepted,
            last_callback=last_callback, last_callback_ros=last_callback_ros,
            last_header_stamp=last_header,
            sample=inbox.sample if inbox else None,
            window=list(inbox.window) if inbox else [],
            status=inbox.status if inbox else 'receiver initializing',
            rejected=dict(inbox.rejected) if inbox else {},
            gap_resets=inbox.gap_resets if inbox else 0,
            stationary_ready=stationary_ready,
            stationary_diagnostic=gate.diagnostic if gate else {}))

    try:
        rclpy.init(args=[])
        node = rclpy.create_node(
            's_curve_session_odom_receiver', start_parameter_services=False,
            enable_rosout=False,
            parameter_overrides=[Parameter('use_sim_time', value=use_sim_time)])
        inbox = PoseInbox(**settings, ros_clock=lambda: node.get_clock().now().nanoseconds / 1e9)

        def capture(msg):
            nonlocal callbacks, accepted, last_callback, last_callback_ros
            nonlocal last_header, heartbeat, stationary_ready
            last_callback = time.monotonic()
            last_callback_ros = node.get_clock().now().nanoseconds / 1e9
            last_header = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            heartbeat = last_callback
            callbacks += 1
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            accepted += int(inbox.ingest(
                stamp=last_header, receipt=last_callback, now_ros=last_callback_ros,
                frame=msg.header.frame_id, child=msg.child_frame_id,
                x=p.x, y=p.y, quaternion=(q.x, q.y, q.z, q.w),
                vx=v.linear.x, vy=v.linear.y, wz=v.angular.z))
            if gate is not None:
                current, diagnostic = inbox.readiness_snapshot()
                stationary_ready = gate.update(current, diagnostic, last_callback)
            publish()

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Odometry, '/odom', capture, qos)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        phase = 'listening'
        publish()
        next_heartbeat = time.monotonic() + .1
        while not stop.is_set() and rclpy.ok() and os.getppid() == parent_pid:
            executor.spin_once(timeout_sec=.05)
            heartbeat = time.monotonic()
            if heartbeat >= next_heartbeat:
                publish()
                next_heartbeat = heartbeat + .1
    except (KeyboardInterrupt, SystemExit):
        pass
    except BaseException as exc:
        phase = 'failed'
        publish(error=type(exc).__name__ + ': ' + str(exc))
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class ProcessPoseInbox(PoseInbox):
    """Same validation/freshness API; every read refreshes a single shared slot."""

    def __init__(self, *, stationary=False, use_sim_time=False, **settings):
        super().__init__(**settings)
        self.lock = threading.RLock()
        context = multiprocessing.get_context('spawn')
        self.slot = dict(data=context.RawArray('B', 65536),
                         length=context.RawValue('I', 0),
                         sequence=context.RawValue('Q', 0), lock=context.Lock())
        self.stop = context.Event()
        self.worker_state = {}
        self.worker_error = None
        self.shared_read_blocked = False
        self.sequence = -1
        child_settings = dict(base_frame=self.base_frame, max_age=self.max_age,
                              max_speed=self.max_speed, max_yaw_rate=self.max_yaw_rate)
        self.process = context.Process(
            target=_receive_odom,
            args=(self.slot, self.stop, child_settings, stationary, use_sim_time, os.getpid()),
            name='session-odom-receiver', daemon=True)
        self.process.start()

    def _refresh_locked(self):
        self.worker_error = None
        self.shared_read_blocked = False
        # A dead writer must not leave the main script waiting on an IPC mutex.
        if not self.slot['lock'].acquire(timeout=.05):
            # A briefly descheduled writer does not add a new stop threshold:
            # the previous sample still has to satisfy the original 500 ms age.
            self.shared_read_blocked = True
            if not self.process.is_alive():
                self.worker_error = f'Odometry receiver exited: exit_code={self.process.exitcode}'
            return
        try:
            sequence = self.slot['sequence'].value
            payload = (bytes(self.slot['data'][:self.slot['length'].value])
                       if sequence != self.sequence else None)
        finally:
            self.slot['lock'].release()
        if payload:
            state = json.loads(payload)
            self.worker_state = state
            self.sequence = sequence
            self.sample = state['sample']
            self.window = deque(state['window'], maxlen=80)
            self.status = state['status']
            self.rejected = Counter(state['rejected'])
            self.gap_resets = state['gap_resets']
        if self.worker_state.get('error'):
            self.worker_error = 'Odometry receiver failed: ' + self.worker_state['error']
        elif not self.process.is_alive():
            self.worker_error = f'Odometry receiver exited: exit_code={self.process.exitcode}'

    def _receiver_diagnostic(self):
        now = self.monotonic_clock()
        state = self.worker_state
        def age(key):
            value = state.get(key)
            return None if value is None else now - value
        return dict(mode='independent_process', pid=self.process.pid,
                    alive=self.process.is_alive(), exit_code=self.process.exitcode,
                    phase=state.get('phase', 'starting'),
                    heartbeat_age_s=age('heartbeat'), callback_age_s=age('last_callback'),
                    last_callback_ros=state.get('last_callback_ros'),
                    last_header_stamp=state.get('last_header_stamp'),
                    callbacks=state.get('callback_count', 0),
                    accepted=state.get('accepted_count', 0),
                    shared_read_blocked=self.shared_read_blocked, error=self.worker_error)

    def _read_locked(self):
        diagnostic = self._receiver_diagnostic()
        if self.worker_error:
            raise RuntimeError(self.worker_error + '; receiver=' + json.dumps(diagnostic))
        try:
            sample, details = super()._read_locked()
        except RuntimeError as exc:
            raise RuntimeError(str(exc) + '; receiver=' + json.dumps(diagnostic)) from exc
        return sample, dict(details, receiver=diagnostic)

    def read(self):
        with self.lock:
            self._refresh_locked()
            return self._read_locked()

    def readiness_snapshot(self):
        with self.lock:
            self._refresh_locked()
            sample, details = super().readiness_snapshot()
            return sample, dict(details, receiver=self._receiver_diagnostic())

    def stationary_readiness_snapshot(self):
        with self.lock:
            self._refresh_locked()
            _, fresh = super().readiness_snapshot()
            diagnostic = dict(self.worker_state.get('stationary_diagnostic', {}))
            ready = bool(self.worker_state.get('stationary_ready')) and fresh['ready']
            if not fresh['ready']:
                diagnostic.update(ready=False, stationary_reason='localization not ready')
            diagnostic['receiver'] = self._receiver_diagnostic()
            return ready, diagnostic

    def close(self):
        self.stop.set()
        self.process.join(timeout=2)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(timeout=1)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(timeout=1)
