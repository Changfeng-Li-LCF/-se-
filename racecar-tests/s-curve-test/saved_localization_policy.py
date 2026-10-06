"""Pure decisions for retaining a verified saved-map localization estimate.

The continuously sampled monitor owns freshness, covariance and scan/map
alignment limits.  This module never turns an unverified estimate into a
reusable one and never compares heading with route goals.
"""
import math


def angle_difference(a, b):
    """Signed angular difference, including poses straddling +/- pi."""
    if not math.isfinite(a) or not math.isfinite(b):
        raise ValueError('Non-finite heading')
    return math.atan2(math.sin(a-b), math.cos(a-b))


def verified_snapshot(snapshot):
    """Validate the monitor's ready verdict without duplicating its limits."""
    if not isinstance(snapshot, dict):
        return False, 'saved-map monitor has no assessment'
    if not snapshot.get('ready', False):
        return False, snapshot.get('reason', 'saved-map monitor is not ready')
    if not snapshot.get('stable', False):
        return False, 'map pose is not stable'
    try:
        pose = snapshot['pose']
        std = snapshot['amcl_std']
        if len(pose) != 3 or len(std) != 3:
            return False, 'incomplete saved-map pose or covariance'
        if not all(math.isfinite(value) for value in (*pose, *std)):
            return False, 'non-finite saved-map pose or covariance'
        if any(value < 0 for value in std):
            return False, 'negative AMCL covariance standard deviation'
    except (KeyError, TypeError, ValueError):
        return False, 'invalid saved-map pose or covariance'
    return True, 'verified map pose, scan alignment and covariance'


def reuse_decision(snapshot, *, reused=False, force_reinitialize=False):
    """`reused` must identify an active stack with the same map fingerprint."""
    if force_reinitialize:
        return dict(reuse=False, reason='explicit relocalization requested')
    if not reused:
        return dict(reuse=False, reason='new stack or different saved map')
    valid, reason = verified_snapshot(snapshot)
    return dict(reuse=valid, reason=reason)


def reuse_observation(snapshot):
    """Distinguish missing discovery/history from a measured unhealthy pose.

    A missing transient-local map/covariance or temporarily unavailable scan
    transform is not evidence that the retained estimate should be reset.
    """
    valid, reason = verified_snapshot(snapshot)
    if valid:
        return dict(complete=True, healthy=True, reason=reason)
    if not isinstance(snapshot, dict) or not snapshot.get('observation_received', False):
        return dict(complete=False, healthy=False, reason=reason)
    try:
        ages=(snapshot['tf_stamp_age_s'],snapshot['tf_receipt_age_s'])
        alignment=snapshot['alignment']
        scan_ages=(alignment['scan_stamp_age_s'],alignment['scan_receipt_age_s'])
        span=snapshot['window_span_s']
        fresh=(all(math.isfinite(v) for v in (*ages,*scan_ages,span))
               and -.05<=ages[0]<=.3 and 0<=ages[1]<=.3
               and -.05<=scan_ages[0]<=.3 and 0<=scan_ages[1]<=.3)
        enough=(span>=1.5 and snapshot['sample_count']>=3 and alignment['known_points']>=12)
        if not fresh or not enough:
            return dict(complete=False,healthy=False,reason=reason)
        # The monitor owns numerical quality limits. Here we only require all
        # components to have arrived so that its unhealthy verdict is meaningful.
        if snapshot.get('pose') is None or snapshot.get('amcl_std') is None:
            return dict(complete=False,healthy=False,reason=reason)
    except (KeyError,TypeError,ValueError):
        return dict(complete=False,healthy=False,reason=reason)
    return dict(complete=True,healthy=False,reason=reason)


def force_consumed(snapshot, request, acknowledged):
    """A service acknowledgement only sets a flag; require a later AMCL scan.

    AMCL may consume an older queued scan than the scan monitor most recently
    saw. Require its source stamp to advance and its receipt to follow the
    request, plus independent scan-source progress. This is an observed
    barrier, not an acknowledgement of AMCL's internal flag state.
    """
    if request is None:
        return True
    if not acknowledged or not isinstance(snapshot,dict):
        return False
    current=snapshot.get('amcl_stamp')
    if current is None or not math.isfinite(current):
        return False
    previous=request.get('previous_amcl_stamp')
    boundary=request.get('scan_source_boundary')
    if previous is not None and current<=previous:
        return False
    request_receipt=request.get('requested_monotonic')
    if request_receipt is not None:
        received=snapshot.get('amcl_received_monotonic')
        if received is None or not math.isfinite(received) or received<request_receipt:
            return False
    if boundary is not None:
        scan=(snapshot.get('alignment') or {}).get('scan_stamp')
        if scan is None or not math.isfinite(scan) or scan<=boundary:
            return False
    return True
