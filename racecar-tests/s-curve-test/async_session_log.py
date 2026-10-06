"""Bounded, nonblocking session logging in a spawned, ROS-free process.

The session thread owns this object. It copies JSON values and enqueues them;
only the helper serializes JSON, opens files, replaces session.json, or prints
events. Call close only after the vehicle has been quiesced. A final authoritative
session summary and terminal-event fallback remain the caller's responsibility.
"""
import multiprocessing as mp
import os
from pathlib import Path
import queue
import signal
import time


_EVENT_WRITTEN, _BULK_WRITTEN, _EVENT_FAILED, _BULK_FAILED = range(4)
_LAST_LAG, _MAX_LAG, _LAST_WRITE, _BUSY = range(4, 8)
_LAST_WRITE_DURATION, _MAX_WRITE_DURATION = range(8, 10)
_ALLOWED_BULK = frozenset((
    'exploration_decisions.jsonl', 'current_plan_tracking.jsonl',
    'navigation_plans.jsonl', 'scan_monitor.jsonl', 'map_trajectory.jsonl',
))


def _json_snapshot(value, budget):
    """Copy supported values without JSON serialization or mutable queue races.

    Restrict types so the multiprocessing feeder cannot fail to pickle a record
    after put_nowait has already reported success. Bound individual records as
    well as queue lengths; giant diagnostics must not grow memory without limit.
    """
    budget[0] -= 1
    if budget[0] < 0:
        raise ValueError('log record exceeds the JSON node limit')
    if value is None or type(value) in (bool, int, float):
        return value
    if type(value) is str:
        budget[1] -= len(value)
        if budget[1] < 0:
            raise ValueError('log record exceeds the text limit')
        return value
    if type(value) in (list, tuple):
        return [_json_snapshot(item, budget) for item in value]
    if type(value) is dict:
        result = {}
        for key, item in value.items():
            if type(key) is not str:
                raise TypeError('log object keys must be strings')
            key = _json_snapshot(key, budget)
            result[key] = _json_snapshot(item, budget)
        return result
    raise TypeError('unsupported log value: ' + type(value).__name__)


def _set_error(shared_error, error):
    data = (type(error).__name__ + ': ' + str(error)).encode('utf-8', 'replace')
    shared_error.value = data[:len(shared_error) - 1]


def _write_record(run_dir, kind, payload, print_events):
    # This function executes in the helper process, never in a ROS callback.
    import json
    if kind == 'event':
        row, state = payload
        with (run_dir / 'events.jsonl').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n')
        temporary = run_dir / ('.session.json.writer-%s.tmp' % os.getpid())
        temporary.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding='utf-8')
        os.replace(temporary, run_dir / 'session.json')
        if print_events:
            compact = {key: row[key] for key in (
                'stage', 'session_elapsed_s', 'elapsed_s', 'reason',
                'generation', 'note', 'action_status',
            ) if key in row}
            try:
                print(json.dumps(compact, ensure_ascii=False), flush=True)
            except (OSError, ValueError):
                # A disconnected console must not lose the disk record.
                pass
    else:
        name, row = payload
        with (run_dir / name).open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, ensure_ascii=False, separators=(',', ':')) + '\n')


class _ClosingFlag:
    """One-writer shutdown flag without a lock a dead child can abandon."""
    def __init__(self, ctx):
        self._value = ctx.RawValue('b', False)

    def set(self):
        self._value.value = True

    def is_set(self):
        return bool(self._value.value)


def _writer_main(run_dir, event_queue, bulk_queue, closing, targets, metrics,
                 shared_error, print_events):
    """Drain by acknowledged counts, not Queue.empty()/qsize() observations."""
    # The session handles Ctrl+C and drains this helper after stopping the car.
    # Its bounded close() can still terminate a genuinely stuck writer.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    run_dir = Path(run_dir)
    try:
        while True:
            if closing.is_set() and (
                metrics[_EVENT_WRITTEN] + metrics[_EVENT_FAILED] >= targets[0]
                and metrics[_BULK_WRITTEN] + metrics[_BULK_FAILED] >= targets[1]
            ):
                return
            # Bulk congestion never consumes slots reserved for events.
            try:
                queued_at, payload = event_queue.get_nowait()
                kind = 'event'
            except queue.Empty:
                try:
                    queued_at, payload = bulk_queue.get_nowait()
                    kind = 'bulk'
                except queue.Empty:
                    try:
                        queued_at, payload = event_queue.get(timeout=0.01)
                        kind = 'event'
                    except queue.Empty:
                        continue
            metrics[_BUSY] = 1
            lag = max(0.0, time.monotonic() - queued_at)
            metrics[_LAST_LAG] = lag
            metrics[_MAX_LAG] = max(metrics[_MAX_LAG], lag)
            write_started = time.perf_counter()
            try:
                _write_record(run_dir, kind, payload, print_events)
            except BaseException:
                metrics[_EVENT_FAILED if kind == 'event' else _BULK_FAILED] += 1
                raise
            metrics[_EVENT_WRITTEN if kind == 'event' else _BULK_WRITTEN] += 1
            metrics[_LAST_WRITE] = time.monotonic()
            duration = time.perf_counter() - write_started
            metrics[_LAST_WRITE_DURATION] = duration
            metrics[_MAX_WRITE_DURATION] = max(metrics[_MAX_WRITE_DURATION], duration)
            metrics[_BUSY] = 0
    except BaseException as error:
        _set_error(shared_error, error)
        raise
    finally:
        metrics[_BUSY] = 0


class AsyncSessionLog:
    def __init__(self, run_dir, event_capacity=256, bulk_capacity=512,
                 print_events=True, max_record_nodes=50000,
                 max_record_text_chars=1048576, _worker_target=None):
        if event_capacity < 1 or bulk_capacity < 1:
            raise ValueError('log queue capacities must be positive')
        self.run_dir = Path(run_dir)
        self._max_record_nodes = max_record_nodes
        self._max_record_text_chars = max_record_text_chars
        self._enqueued = {'event': 0, 'bulk': 0}
        self._dropped = {'event': 0, 'bulk': 0}
        self._last_drop_reason = None
        self._closed = False
        self._forced_termination = False
        self._startup_error = None
        self._last_enqueue_ms = 0.0
        self._max_enqueue_ms = 0.0
        ctx = mp.get_context('spawn')
        self._event_queue = ctx.Queue(maxsize=event_capacity)
        self._bulk_queue = ctx.Queue(maxsize=bulk_capacity)
        self._closing = _ClosingFlag(ctx)
        self._targets = ctx.RawArray('q', 2)
        self._metrics = ctx.RawArray('d', 10)
        self._shared_error = ctx.RawArray('c', 2048)
        self._process = ctx.Process(
            name='session-json-writer', target=_worker_target or _writer_main,
            args=(str(self.run_dir), self._event_queue, self._bulk_queue,
                  self._closing, self._targets, self._metrics,
                  self._shared_error, bool(print_events)), daemon=True,
        )
        try:
            self._process.start()
        except Exception as error:
            # Logging failure must not prevent callbacks or the stop path.
            self._startup_error = type(error).__name__ + ': ' + str(error)

    def _enqueue(self, kind, payload):
        started = time.perf_counter()
        if self._closed:
            reason = 'writer_closed'
        elif not self._process.is_alive():
            reason = 'writer_not_alive'
        else:
            try:
                snapshot = _json_snapshot(payload, [
                    self._max_record_nodes, self._max_record_text_chars,
                ])
                target = self._event_queue if kind == 'event' else self._bulk_queue
                target.put_nowait((time.monotonic(), snapshot))
            except queue.Full:
                reason = kind + '_queue_full'
            except Exception as error:
                reason = type(error).__name__ + ': ' + str(error)
            else:
                self._enqueued[kind] += 1
                self._last_enqueue_ms = 1000.0 * (time.perf_counter() - started)
                self._max_enqueue_ms = max(self._max_enqueue_ms, self._last_enqueue_ms)
                return True
        self._dropped[kind] += 1
        self._last_drop_reason = reason
        self._last_enqueue_ms = 1000.0 * (time.perf_counter() - started)
        self._max_enqueue_ms = max(self._max_enqueue_ms, self._last_enqueue_ms)
        return False

    def event(self, row, state):
        return self._enqueue('event', (row, state))

    def append(self, name, row):
        if name not in _ALLOWED_BULK:
            self._dropped['bulk'] += 1
            self._last_drop_reason = 'unsupported_log_name: ' + str(name)
            return False
        return self._enqueue('bulk', (name, row))

    def snapshot(self):
        # These shared counters are diagnostic snapshots, not synchronization.
        # The worker exclusively writes them; callbacks never wait on a lock.
        metrics = list(self._metrics)
        priorities = {}
        for kind, written_index, failed_index in (
            ('event', _EVENT_WRITTEN, _EVENT_FAILED),
            ('bulk', _BULK_WRITTEN, _BULK_FAILED),
        ):
            written, failed = int(metrics[written_index]), int(metrics[failed_index])
            priorities[kind] = dict(
                enqueued=self._enqueued[kind], written=written, failed=failed,
                dropped=self._dropped[kind],
                pending=max(0, self._enqueued[kind] - written - failed),
            )
        alive = self._process.is_alive()
        error = self._startup_error or self._shared_error.value.decode('utf-8', 'replace') or None
        if not alive and error is None and (
            not self._closed or self._process.exitcode not in (None, 0)
        ):
            error = 'writer exited unexpectedly (exit code %s)' % self._process.exitcode
        return dict(
            **priorities, writer_alive=alive, writer_pid=self._process.pid,
            writer_exit_code=self._process.exitcode, error=error,
            last_drop_reason=self._last_drop_reason, closed=self._closed,
            forced_termination=self._forced_termination,
            writer_busy=bool(metrics[_BUSY]),
            last_queue_lag_ms=1000.0 * metrics[_LAST_LAG],
            max_queue_lag_ms=1000.0 * metrics[_MAX_LAG],
            last_write_duration_ms=1000.0 * metrics[_LAST_WRITE_DURATION],
            max_write_duration_ms=1000.0 * metrics[_MAX_WRITE_DURATION],
            last_enqueue_ms=self._last_enqueue_ms,
            max_enqueue_ms=self._max_enqueue_ms,
            last_write_age_s=(max(0.0, time.monotonic() - metrics[_LAST_WRITE])
                              if metrics[_LAST_WRITE] else None),
        )

    def close(self, timeout_s=3.0):
        """Bounded drain, then terminate only this helper if disk I/O is stuck."""
        if not self._closed:
            self._closed = True
            self._targets[0] = self._enqueued['event']
            self._targets[1] = self._enqueued['bulk']
            self._closing.set()
            timeout_s = max(0.0, float(timeout_s))
            deadline = time.monotonic() + timeout_s
            if self._process.pid is not None:
                self._process.join(timeout=max(0.0, timeout_s - 0.25))
                if self._process.is_alive():
                    self._forced_termination = True
                    self._process.terminate()
                    self._process.join(timeout=max(0.0, deadline - time.monotonic() - 0.05))
                if self._process.is_alive() and hasattr(self._process, 'kill'):
                    self._process.kill()
                    self._process.join(timeout=max(0.0, deadline - time.monotonic()))
            for pending_queue in (self._event_queue, self._bulk_queue):
                # Never wait for a feeder whose reader was killed or lost.
                pending_queue.cancel_join_thread()
                pending_queue.close()
        result = self.snapshot()
        result['incomplete'] = bool(
            result['writer_alive'] or result['forced_termination'] or result['error']
            or any(result[kind][field] for kind in ('event', 'bulk')
                   for field in ('pending', 'failed', 'dropped'))
        )
        return result
