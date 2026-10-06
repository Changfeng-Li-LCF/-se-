"""Bounded saved-map pose and raw scan agreement checks, without ROS imports.

The map here is the original OccupancyGrid, never an inflated/live costmap.
The caller supplies map<-laser at the LaserScan source timestamp. No heading
or obstacle position is inferred from a requested navigation goal.
"""
from array import array
from collections import deque
import hashlib
import math
import threading
import time
from types import MappingProxyType


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(q):
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid transform quaternion')
    x, y, z, w = (v / norm for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def transform_endpoint(transform, x, y):
    """Use the full mounting quaternion, including an upside-down lidar."""
    translation = transform['translation']
    q = transform['quaternion']
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid laser transform quaternion')
    qx, qy, qz, qw = (v / norm for v in q)
    return (translation[0] + (1 - 2 * (qy * qy + qz * qz)) * x
            + 2 * (qx * qy - qz * qw) * y,
            translation[1] + 2 * (qx * qy + qz * qw) * x
            + (1 - 2 * (qx * qx + qz * qz)) * y)


class ScanToMapAlignment:
    """At most 90 raw scan endpoints checked against nearby occupied cells."""
    def __init__(self, max_points=90, min_known_points=12, min_match_ratio=.6):
        self.max_points = max_points
        self.min_known_points = min_known_points
        self.min_match_ratio = min_match_ratio
        self.grid = None
        self.occupied = set()

    def update_map(self, *, width, height, resolution, origin, data):
        # A malformed replacement must never leave a previous map usable.
        self.grid = None
        self.occupied = set()
        if width < 1 or height < 1 or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid static map dimensions')
        if len(data) != width * height or not all(math.isfinite(v) for v in origin):
            raise ValueError('invalid static map data/origin')
        values = array('b', data)
        fingerprint=hashlib.blake2b(digest_size=16)
        fingerprint.update(repr((width,height,resolution,tuple(origin))).encode('ascii'))
        fingerprint.update(values)
        self.grid = MappingProxyType(dict(width=width, height=height, resolution=resolution,
                         origin=tuple(origin), data=memoryview(values).toreadonly(),
                         version=fingerprint.hexdigest()))
        self.occupied = {i for i, value in enumerate(values) if value >= 65}

    def evaluate(self, *, ranges, angle_min, angle_increment, range_min, range_max,
                 laser_transform, scan_stamp, frame='', now_ros=None):
        begun = time.perf_counter()
        result = dict(ready=False, reason='static map unavailable', scan_stamp=scan_stamp,
                      frame=frame, valid_points=0, known_points=0, matched_points=0,
                      match_ratio=None, mean_distance_m=None, endpoints_map=[], endpoint_ranges=[],
                      computed_at_ros=now_ros,compute_ms=0.,map_version=None)
        if self.grid is None:
            return result
        grid = self.grid
        result['map_version'] = grid['version']
        width, height, res = grid['width'], grid['height'], grid['resolution']
        ox, oy, oa = grid['origin']
        oc, os = math.cos(oa), math.sin(oa)
        threshold = max(2 * res, .1)
        radius = math.ceil(threshold / res) + 1
        result['matching_distance_m'] = threshold
        if (not ranges or not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min))
                or angle_increment == 0 or math.isnan(range_max) or range_max <= range_min):
            result['reason'] = 'invalid scan geometry'
            return result
        # Select evenly around the complete scan rather than its first 90 beams.
        stride = max(1, math.ceil(len(ranges) / self.max_points))
        distances = []
        for index in range(0, len(ranges), stride):
            distance = ranges[index]
            if not math.isfinite(distance) or distance <= range_min or distance >= range_max:
                continue
            angle = angle_min + index * angle_increment
            wx, wy = transform_endpoint(laser_transform, distance * math.cos(angle),
                                         distance * math.sin(angle))
            result['endpoints_map'].append([wx, wy])
            result['endpoint_ranges'].append(distance)
            result['valid_points'] += 1
            dx, dy = wx - ox, wy - oy
            mx, my = oc * dx + os * dy, -os * dx + oc * dy
            gx, gy = math.floor(mx / res), math.floor(my / res)
            if not (0 <= gx < width and 0 <= gy < height):
                continue
            if grid['data'][gy * width + gx] < 0:
                continue
            result['known_points'] += 1
            nearest_squared = threshold * threshold
            matched = False
            for cy in range(max(0, gy - radius), min(height, gy + radius + 1)):
                for cx in range(max(0, gx - radius), min(width, gx + radius + 1)):
                    if cy * width + cx not in self.occupied:
                        continue
                    squared = ((cx + .5) * res - mx) ** 2 + ((cy + .5) * res - my) ** 2
                    if squared <= nearest_squared:
                        nearest_squared = squared
                        matched = True
            distances.append(math.sqrt(nearest_squared))
            result['matched_points'] += int(matched)
        known = result['known_points']
        result['match_ratio'] = result['matched_points'] / known if known else None
        result['mean_distance_m'] = sum(distances) / known if known else None
        result['distance_capped_at_m'] = threshold
        if known < self.min_known_points:
            result['reason'] = 'insufficient scan/static-map evidence: %d known endpoints, need %d' % (
                known, self.min_known_points)
        elif result['match_ratio'] < self.min_match_ratio:
            result['reason'] = 'scan/static-map mismatch: %.3f matched, need %.3f' % (
                result['match_ratio'], self.min_match_ratio)
        else:
            result.update(ready=True, reason='ok')
        result['compute_ms'] = (time.perf_counter() - begun) * 1000
        return result


class SavedMapPoseMonitor:
    """A continuous global-frame stationary gate plus bounded diagnostic history."""
    def __init__(self, *, ros_clock=time.time, monotonic_clock=time.monotonic,
                 stable_window_s=1.5, max_translation_m=.04, max_rotation_rad=.06,
                 max_tf_age_s=.3, max_history=2400,max_xy_std_m=.15,max_yaw_std_rad=.12):
        self.ros_clock = ros_clock
        self.monotonic_clock = monotonic_clock
        self.stable_window_s = stable_window_s
        self.max_translation_m = max_translation_m
        self.max_rotation_rad = max_rotation_rad
        self.max_tf_age_s = max_tf_age_s
        self.max_xy_std_m = max_xy_std_m
        self.max_yaw_std_rad = max_yaw_std_rad
        self.lock = threading.Lock()
        self.window = deque(maxlen=120)
        self.history = deque(maxlen=max_history)
        self.amcl = None
        self.alignment = None
        self.latest = None
        self.error = 'no map-frame transform received'
        self.amcl_source_fence = None
        self.amcl_receipt_fence = None
        self.amcl_error = None
        self.resets = 0
        self.history_dropped = 0

    def _record(self, row):
        if len(self.history) == self.history.maxlen:
            self.history_dropped += 1
        self.history.append(row)

    def reset(self, invalidate_amcl=True):
        with self.lock:
            self.window.clear()
            self.resets += 1
            if invalidate_amcl:
                # AMCL poses carry laser acquisition time, not request time.
                # Compare with the last known AMCL source, never wall ROS time;
                # a legitimate queued scan can predate the initialpose request.
                self.amcl_source_fence = self.amcl['stamp'] if self.amcl else None
                self.amcl_receipt_fence = self.monotonic_clock()
                self.amcl = None
                self.amcl_error = 'waiting for AMCL covariance after initial-pose reset'
            self._record(dict(kind='reset', received=self.monotonic_clock(),
                              ros_time=self.ros_clock(), invalidate_amcl=invalidate_amcl))

    def ingest_amcl(self, *, stamp, pose, std):
        with self.lock:
            receipt = self.monotonic_clock()
            if (not all(math.isfinite(v) for v in (stamp, *pose, *std))
                    or any(v < 0 for v in std) or stamp > self.ros_clock()+.1):
                self.amcl = None
                self.amcl_error = 'invalid AMCL pose/covariance/source stamp'
                self._record(dict(kind='amcl_rejected', reason='invalid AMCL pose/covariance',
                                  stamp=stamp, received=receipt))
                return False
            if ((self.amcl_source_fence is not None and stamp <= self.amcl_source_fence)
                    or (self.amcl_receipt_fence is not None and receipt < self.amcl_receipt_fence)):
                self._record(dict(kind='amcl_rejected', reason='AMCL sample precedes initial-pose reset',
                                  stamp=stamp, received=receipt))
                return False
            self.amcl = dict(stamp=stamp, pose=list(pose), std=list(std), received=receipt)
            self.amcl_error = None
            self._record(dict(kind='amcl', **self.amcl))
            return True

    def ingest_alignment(self, result, receipt):
        with self.lock:
            self.alignment = dict(result, received=receipt)
            # Endpoints live only in the latest alignment; do not multiply their
            # memory/disk cost in a long monitor history.
            self._record(dict(kind='alignment', **{k: v for k, v in self.alignment.items()
                                                  if k not in ('endpoints_map','endpoint_ranges')}))

    def ingest_tf(self, *, stamp, pose, map_from_odom, map_odom_stamp=None):
        with self.lock:
            now_ros, receipt = self.ros_clock(), self.monotonic_clock()
            if not all(math.isfinite(v) for v in (stamp, *pose, *map_from_odom)):
                self.error = 'nonfinite map-frame transform'
                self.window.clear()
                return False
            if self.latest and stamp < self.latest['stamp'] - .05:
                self.amcl = None
                self.amcl_source_fence = None
                self.amcl_receipt_fence = receipt
                self.amcl_error = 'source time moved backwards; waiting for new AMCL covariance'
                self._record(dict(kind='clock_reset', stamp=stamp, ros_time=now_ros, received=receipt))
                self.window.clear()
            elif self.latest and receipt - self.latest['received'] > .3:
                self.window.clear()
            current = dict(stamp=stamp, received=receipt, ros_time=now_ros,
                           pose=list(pose), map_from_odom=list(map_from_odom),
                           map_odom_stamp=map_odom_stamp)
            self.latest = current
            self.error = None
            if not -.05 <= now_ros - stamp <= self.max_tf_age_s:
                self.window.clear()
            else:
                self.window.append(current)
                cutoff = receipt - self.stable_window_s
                # Keep one sample bracketing the required complete window.
                while len(self.window) > 1 and self.window[1]['received'] < cutoff:
                    self.window.popleft()
            self._record(dict(kind='tf', **current))
            return True

    def ingest_error(self, reason):
        with self.lock:
            self.error = reason
            self.window.clear()
            self._record(dict(kind='tf_error', reason=reason, received=self.monotonic_clock(),
                              ros_time=self.ros_clock()))

    def snapshot(self):
        with self.lock:
            now_ros, now_mono = self.ros_clock(), self.monotonic_clock()
            result = dict(ready=False, reason='', pose=None, map_from_odom=None,
                          stable=False, amcl_std=self.amcl['std'][:] if self.amcl else None,
                          amcl_received=self.amcl is not None, alignment=None,
                          observation_received=False, sample_count=len(self.window),
                          window_span_s=0., resets=self.resets, history_dropped=self.history_dropped,
                          limits=dict(stable_window_s=self.stable_window_s,
                                      max_translation_m=self.max_translation_m,
                                      max_rotation_rad=self.max_rotation_rad,
                                      max_tf_age_s=self.max_tf_age_s,
                                      max_xy_std_m=self.max_xy_std_m,
                                      max_yaw_std_rad=self.max_yaw_std_rad))
            reasons = []
            if self.error:
                reasons.append(self.error)
            if self.latest is not None:
                latest = self.latest
                result.update(pose=latest['pose'][:], map_from_odom=latest['map_from_odom'][:],
                              tf_stamp=latest['stamp'], tf_stamp_age_s=now_ros-latest['stamp'],
                              tf_receipt_age_s=now_mono-latest['received'],
                              map_odom_stamp=latest['map_odom_stamp'])
                if not -.05 <= result['tf_stamp_age_s'] <= self.max_tf_age_s:
                    reasons.append('map/base transform source is stale or future dated')
                if not 0 <= result['tf_receipt_age_s'] <= self.max_tf_age_s:
                    reasons.append('map/base transform monitor receipt is stale')
            if self.amcl is None:
                reasons.append(self.amcl_error or 'waiting for valid AMCL covariance')
            else:
                result.update(amcl_stamp=self.amcl['stamp'], amcl_pose=self.amcl['pose'][:],
                              amcl_received_monotonic=self.amcl['received'])
                # AMCL may not publish while stationary. Fresh TF and raw scan
                # agreement, rather than old covariance publication age, guard it.
                if (max(self.amcl['std'][:2]) > self.max_xy_std_m
                        or self.amcl['std'][2] > self.max_yaw_std_rad):
                    reasons.append('AMCL uncertainty exceeds %.2fm/%.2frad' % (
                        self.max_xy_std_m,self.max_yaw_std_rad))
            if self.alignment is None:
                reasons.append('waiting for scan/static-map agreement')
            else:
                result['alignment'] = {k: v for k, v in self.alignment.items()
                                       if k not in ('endpoints_map','endpoint_ranges')}
                scan_age = now_ros-self.alignment['scan_stamp']
                receipt_age = now_mono-self.alignment['received']
                result['alignment'].update(scan_stamp_age_s=scan_age,
                                           scan_receipt_age_s=receipt_age)
                if not -.05 <= scan_age <= .3 or not 0 <= receipt_age <= .3:
                    reasons.append('scan/static-map agreement sample is stale')
                elif not self.alignment['ready']:
                    reasons.append(self.alignment['reason'])
            # Quality can converge before the stationary window. Stop forced
            # AMCL updates then, while still requiring the full window to start.
            result['quality_ready'] = not reasons
            if self.window:
                span = self.window[-1]['received'] - self.window[0]['received']
                changes = []
                for key in ('pose', 'map_from_odom'):
                    xs = [p[key][0] for p in self.window]
                    ys = [p[key][1] for p in self.window]
                    ref = self.window[-1][key][2]
                    angles = [wrap(p[key][2]-ref) for p in self.window]
                    changes.append([math.hypot(max(xs)-min(xs), max(ys)-min(ys)),
                                    max(angles)-min(angles)])
                result.update(window_span_s=span, pose_window_change=changes[0],
                              map_odom_window_change=changes[1])
                result['stable'] = (span >= self.stable_window_s and len(self.window) >= 3
                                    and all(c[0] <= self.max_translation_m
                                            and c[1] <= self.max_rotation_rad for c in changes))
            if not result['stable']:
                reasons.append('waiting for stable global pose/map-odom window (1.5s, 0.04m, 0.06rad)')
            result['observation_received'] = (self.latest is not None and self.amcl is not None
                                               and self.alignment is not None)
            result.update(ready=not reasons, reason='; '.join(reasons) or 'ok')
            return result

    def export_history(self):
        with self.lock:
            return list(self.history)
