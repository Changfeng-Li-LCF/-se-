"""Pure lifecycle state for proposed and BT-confirmed rolling goals.

The session thread owns this object. ROS callbacks should queue decoded status
messages for that thread to pass to ``handle_ack``. This module imports no ROS,
publishes nothing, and never treats a timeout or an absent plan as acceptance.
"""
from collections.abc import Mapping
from dataclasses import dataclass
import math
from numbers import Real
import time


def _goal(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('Goal must be a three-element list or tuple')
    if any(isinstance(v, bool) or not isinstance(v, Real) for v in value):
        raise ValueError('Goal coordinates must be real numbers')
    try:
        result = tuple(float(v) for v in value)
    except (ValueError, OverflowError) as exc:
        raise ValueError('Goal coordinates must be finite') from exc
    if not all(math.isfinite(v) for v in result):
        raise ValueError('Goal coordinates must be finite')
    return result


def _stamp(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError('Stamp must be (seconds, nanoseconds)')
    sec, nanosec = value
    if any(isinstance(v, bool) or not isinstance(v, int) for v in value):
        raise ValueError('Stamp components must be integers')
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError('Stamp components are outside ROS time bounds')
    return sec, nanosec


def _now(value):
    value = time.monotonic() if value is None else value
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError('Monotonic time must be a finite number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('Monotonic time must be a finite number')
    return value


@dataclass(frozen=True)
class PendingGoalRequest:
    goal: tuple
    stamp: tuple
    submitted_at: float
    initial: bool = False


class RollingGoalTracker:
    """Keep a single pending request separate from the last accepted goal.

    ``initial_goal`` is a planned starting anchor, explicitly unconfirmed until
    its registered request receives a matching BT acknowledgement. ``anchor_goal``
    falls back to that seed while ``accepted_goal`` stays None. Stamp registration
    is strictly increasing, including after rejection; IDs must never be reused.
    Supply the current ROS time as ``session_start_stamp`` to reject earlier-session
    requests/acks. Request stamps must also be unique across sessions when ROS
    time is rewound: this protocol carries no separate session identifier.
    """

    def __init__(self, initial_goal=None, *, session_start_stamp=None, xy_tolerance_m=.001):
        if (isinstance(xy_tolerance_m, bool) or not isinstance(xy_tolerance_m, Real)
                or not math.isfinite(xy_tolerance_m) or xy_tolerance_m < 0):
            raise ValueError('XY tolerance must be finite and nonnegative')
        self._xy_tolerance = float(xy_tolerance_m)
        self._initial_goal = _goal(initial_goal) if initial_goal is not None else None
        self._session_start_stamp = _stamp(session_start_stamp) if session_start_stamp is not None else None
        self._accepted_goal = None
        self._accepted_stamp = None
        self._pending = None
        self._last_request_stamp = None
        self._completed_stamp = None
        self._last_resolution_status = None
        self._last_request_rejection_reason = None

    @property
    def pending(self):
        return self._pending

    @property
    def can_request(self):
        return self._pending is None

    @property
    def accepted_goal(self):
        return self._accepted_goal

    @property
    def accepted_stamp(self):
        return self._accepted_stamp

    @property
    def completed_stamp(self):
        return self._completed_stamp

    @property
    def last_request_stamp(self):
        return self._last_request_stamp

    @property
    def anchor_goal(self):
        return self._accepted_goal if self._accepted_goal is not None else self._initial_goal

    @property
    def anchor_source(self):
        if self._accepted_goal is not None:
            return 'bt_accepted'
        return 'planned_initial' if self._initial_goal is not None else 'none'

    def _same_xy(self, first, second):
        return math.hypot(first[0]-second[0], first[1]-second[1]) <= self._xy_tolerance

    def register_request(self, goal, stamp, *, now=None, initial=False):
        """Register before publishing; return False if occupied or stamp is stale.

        No delay is imposed after a matching acknowledgement. The runner owns
        the desired 0.2-second selection cadence and the actual publish call.
        Invalid values raise ValueError without modifying lifecycle state.
        """
        goal, stamp, submitted_at = _goal(goal), _stamp(stamp), _now(now)
        if not isinstance(initial, bool):
            raise ValueError('initial must be a bool')
        if self._pending is not None:
            self._last_request_rejection_reason = 'request_already_pending'
            return False
        if self._session_start_stamp is not None and stamp < self._session_start_stamp:
            self._last_request_rejection_reason = 'request_before_session'
            return False
        if self._last_request_stamp is not None and stamp <= self._last_request_stamp:
            self._last_request_rejection_reason = 'request_stamp_not_increasing'
            return False
        if initial and self._last_request_stamp is not None:
            self._last_request_rejection_reason = 'initial_request_already_registered'
            return False
        if initial and self._initial_goal is not None and not self._same_xy(goal, self._initial_goal):
            self._last_request_rejection_reason = 'initial_goal_mismatch'
            return False
        if initial and self._initial_goal is None:
            self._initial_goal = goal
        self._pending = PendingGoalRequest(goal, stamp, submitted_at, initial)
        self._last_request_stamp = stamp
        self._last_request_rejection_reason = None
        return True

    def _ignored(self, reason, status=None):
        return dict(matched=False, changed=False, status=status, reason=reason,
                    accepted_goal=self._accepted_goal, accepted_stamp=self._accepted_stamp)

    def finish_navigation(self):
        """Retire proposals owned by a completed action, keeping the accepted anchor.

        Call only after the action result has arrived. Its BT can no longer
        commit a pending proposal. A later action must use a strictly new stamp;
        a delayed acknowledgement from the completed action is never acceptance.
        """
        pending = self._pending
        self._pending = None
        if pending is not None:
            self._completed_stamp = pending.stamp
            self._last_resolution_status = 'action_completed_before_ack'
        return pending

    def handle_ack(self, message, *, now=None):
        """Apply decoded protocol data; malformed/stale data leaves state intact.

        The caller catches JSON decoding errors. This function checks all decoded
        fields, including finite request/accepted coordinates. An accepted heading
        may differ after planner fallback; its XY must still match the request.
        """
        if not isinstance(message, Mapping):
            return self._ignored('invalid_payload')
        status = message.get('status')
        if status not in ('accepted', 'rejected'):
            return self._ignored('invalid_status')
        try:
            stamp = _stamp((message.get('request_stamp_sec'), message.get('request_stamp_nanosec')))
            request_goal = _goal(message.get('request_goal'))
            # Validate even an optional caller-supplied clock without changing state.
            if now is not None:
                _now(now)
        except (ValueError, TypeError, OverflowError):
            return self._ignored('invalid_request_fields', status)
        if self._session_start_stamp is not None and stamp < self._session_start_stamp:
            return self._ignored('ack_before_session', status)
        pending = self._pending
        if pending is None:
            return self._ignored('duplicate_ack' if stamp == self._completed_stamp else 'no_pending_request', status)
        if stamp != pending.stamp:
            return self._ignored('stale_ack' if stamp < pending.stamp else 'unmatched_future_ack', status)
        if not self._same_xy(request_goal, pending.goal):
            return self._ignored('request_goal_mismatch', status)
        accepted = None
        if status == 'accepted':
            try:
                accepted = _goal(message.get('accepted_goal'))
            except (ValueError, TypeError, OverflowError):
                return self._ignored('invalid_accepted_goal', status)
            if not self._same_xy(accepted, pending.goal):
                return self._ignored('accepted_goal_mismatch', status)
        previous = self._accepted_goal
        self._pending = None
        self._completed_stamp = stamp
        self._last_resolution_status = status
        if status == 'accepted':
            self._accepted_goal, self._accepted_stamp = accepted, stamp
        return dict(matched=True, changed=True, status=status,
                    reason='request_accepted' if status == 'accepted' else 'request_rejected',
                    request_goal=pending.goal, request_stamp=stamp, initial=pending.initial,
                    accepted_goal=self._accepted_goal, accepted_stamp=self._accepted_stamp,
                    previous_accepted_goal=previous)

    def snapshot(self, *, now=None):
        """JSON-friendly diagnostics; long pending requests remain pending."""
        current = _now(now)
        pending = self._pending
        return dict(accepted_goal=list(self._accepted_goal) if self._accepted_goal is not None else None,
                    accepted_stamp=list(self._accepted_stamp) if self._accepted_stamp is not None else None,
                    planned_initial_goal=list(self._initial_goal) if self._initial_goal is not None else None,
                    anchor_goal=list(self.anchor_goal) if self.anchor_goal is not None else None,
                    anchor_source=self.anchor_source, can_request=self.can_request,
                    pending=(dict(goal=list(pending.goal), stamp=list(pending.stamp),
                                  submitted_at=pending.submitted_at, initial=pending.initial)
                             if pending is not None else None),
                    pending_age_s=max(0., current-pending.submitted_at) if pending is not None else None,
                    completed_stamp=list(self._completed_stamp) if self._completed_stamp is not None else None,
                    last_request_stamp=list(self._last_request_stamp) if self._last_request_stamp is not None else None,
                    last_resolution_status=self._last_resolution_status,
                    last_request_rejection_reason=self._last_request_rejection_reason)
