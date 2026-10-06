"""Bounded lifecycle polling without overlapping requests or short inner timeouts."""
import time


def wait_for_active(client, request_factory, spin, check_abort, progress,
                    timeout=60.0, clock=time.monotonic):
    start = clock()
    deadline = start + timeout
    future = None
    last = 'service unavailable'
    next_query = start
    next_log = start
    try:
        while clock() < deadline:
            check_abort()
            now = clock()
            if future is not None and future.done():
                answer = future.result()
                future = None
                if answer is None:
                    raise RuntimeError('Lifecycle service returned an empty response')
                last = answer.current_state.label
                if answer.current_state.id == 3:
                    return
                next_query = now + 0.4
            if future is None and now >= next_query and client.service_is_ready():
                future = client.call_async(request_factory())
                last = 'waiting for state response (last state: ' + last + ')'
            if now >= next_log:
                progress(last, now - start)
                next_log = now + 5.0
            spin(min(0.05, max(0.0, deadline - clock())))
        raise RuntimeError('Lifecycle activation timed out after %.1f s: %s' % (timeout, last))
    finally:
        if future is not None and not future.done():
            future.cancel()
            # rclpy retains pending requests unless explicitly removed.
            client.remove_pending_request(future)


def wait_for_response(client, request_factory, spin, check_abort, progress,
                      timeout=60.0, description='startup service', clock=time.monotonic):
    """Send one initialization command and await completion, without resending it."""
    start = clock()
    deadline = start + timeout
    future = None
    next_log = start
    status = 'waiting for service discovery'
    try:
        while True:
            check_abort()
            if future is not None and future.done():
                answer = future.result()
                if answer is None:
                    raise RuntimeError(description + ': empty service response')
                return answer
            now = clock()
            if now >= deadline:
                raise RuntimeError('%s: startup stalled after %.1f s (%s)' %
                                   (description, timeout, status))
            if future is None and client.service_is_ready():
                future = client.call_async(request_factory())
                status = 'initialization request sent; waiting for completion'
            if now >= next_log:
                progress(status, now - start)
                next_log = now + 5.0
            spin(min(0.05, max(0.0, deadline - clock())))
    finally:
        if future is not None and not future.done():
            future.cancel()
            client.remove_pending_request(future)
