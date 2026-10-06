"""Stopped saved-map costmap reset and evidence barrier (Nav2 Humble).

GetCostmap's update_time is its service-response time in Humble. It is not
proof that the map update thread has run or consumed a new scan. This barrier
combines fresh advancing scans, completed update cycles (published_footprint),
restored static lethal cells and current scan endpoint representation. It is
not an internal ObservationBuffer acknowledgement and never permits motion.
"""
from collections import deque
import json
import math
from pathlib import Path
import time


class CostmapLocalizationChanged(RuntimeError):
    """Recover at the stop latch: wait for global stability, rebuild and replan."""


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def pose_change(a, b):
    return [math.hypot(a[0]-b[0], a[1]-b[1]), abs(wrap(a[2]-b[2]))]


def barrier_evidence(scan, cycles, *, reset_mono, reset_ros, now_mono, now_ros):
    """Pure evidence check: cached receipts and response timestamps never count."""
    scans = []
    last = reset_ros
    for p in scan.get('recent_receipts', []):
        source = p.get('header_stamp')
        if (source is None or not math.isfinite(source) or source <= last
                or source > now_ros+.05 or p['receipt_monotonic'] <= reset_mono):
            continue
        scans.append(p); last = source
    result = dict(post_reset_scans=len(scans), ready=False,
                  reason='waiting for 3 fresh advancing post-reset scans')
    if len(scans) < 3 or not scan.get('healthy', False):
        return result
    # Once acquired, keep a fixed scan boundary. Requiring all costmap cycles
    # to trail the continuously newest scan would introduce a moving deadline.
    boundary = scans[2]
    result.update(scan_boundary_stamp=boundary['header_stamp'],
                  scan_boundary_receipt=boundary['receipt_monotonic'])
    if now_mono-scans[-1]['receipt_monotonic'] > 1.:
        result['reason'] = 'post-reset scan stream is stale'; return result
    counts = {}
    for name in ('global', 'local'):
        count = 0; latest_source = boundary['header_stamp']
        for p in cycles.get(name, []):
            if (p['receipt'] <= boundary['receipt_monotonic']
                    or p['stamp'] <= latest_source or p['stamp'] > now_ros+.5):
                continue
            count += 1; latest_source = p['stamp']
        counts[name] = count
    result['completed_update_cycles'] = counts
    result.update(ready=min(counts.values()) >= 2,
                  reason='ok' if min(counts.values()) >= 2 else 'waiting for post-scan completed costmap update cycles')
    return result


def _cell(snapshot, x, y):
    ox, oy, angle = snapshot['origin']
    c, s = math.cos(angle), math.sin(angle)
    dx, dy = x-ox, y-oy
    return math.floor((c*dx+s*dy)/snapshot['resolution']), math.floor((-s*dx+c*dy)/snapshot['resolution'])


def _value(snapshot, x, y):
    ix, iy = _cell(snapshot, x, y)
    if not 0 <= ix < snapshot['width'] or not 0 <= iy < snapshot['height']:
        return None
    return snapshot['data'][iy*snapshot['width']+ix]


def static_restoration(snapshot, static, lethal_threshold=100):
    """Check every occupied static-map cell, not a service's refreshed stamp."""
    if not static:
        return dict(ready=False, reason='static map evidence missing')
    ox, oy, angle = static['origin']; c, s = math.cos(angle), math.sin(angle)
    total = misses = 0; first = None
    for index, value in enumerate(static['data']):
        if value < lethal_threshold:
            continue
        total += 1
        px = (index % static['width']+.5)*static['resolution']
        py = (index // static['width']+.5)*static['resolution']
        x, y = ox+c*px-s*py, oy+s*px+c*py
        if _value(snapshot, x, y) != 254:
            misses += 1
            if first is None: first = [x, y]
    return dict(ready=misses == 0, occupied_cells=total, missing_cells=misses,
                first_missing=first, reason='ok' if misses == 0 else 'static lethal cells not yet restored')


def scan_representation(snapshot, evidence, *, min_range=0., max_range=2.5, base_pose=None, footprint=None):
    """Current in-range hits must exist in the rebuilt master costmap.

    A one-cell neighbourhood accounts for LaserScan projection and voxel/grid
    quantisation. Exclude hits deliberately erased by footprint clearing.
    An empty, valid in-range set is permitted (large open space), but the scan
    and completed-update barriers still apply.
    """
    hits = evidence.get('endpoints_map', [])
    distances = evidence.get('endpoint_ranges', [])
    supported = checked = outside = ignored = missing_near = 0; first = None
    for i, point in enumerate(hits):
        x, y = point[:2]
        if i < len(distances) and not min_range <= distances[i] < max_range:
            ignored += 1; continue
        if base_pose is not None and footprint:
            dx, dy = x-base_pose[0], y-base_pose[1]
            c, s = math.cos(base_pose[2]), math.sin(base_pose[2])
            bx, by = c*dx+s*dy, -s*dx+c*dy
            # Convex runtime footprint, either winding.
            signs = [(b[0]-a[0])*(by-a[1])-(b[1]-a[1])*(bx-a[0])
                     for a, b in zip(footprint, footprint[1:]+footprint[:1])]
            if all(v >= -1e-9 for v in signs) or all(v <= 1e-9 for v in signs):
                ignored += 1; continue
        ix, iy = _cell(snapshot, x, y)
        if not 0 <= ix < snapshot['width'] or not 0 <= iy < snapshot['height']:
            outside += 1; continue
        checked += 1
        found = any(snapshot['data'][j*snapshot['width']+k] == 254
                    for j in range(max(0, iy-1), min(snapshot['height'], iy+2))
                    for k in range(max(0, ix-1), min(snapshot['width'], ix+2)))
        if found: supported += 1
        else:
            if i < len(distances) and distances[i] <= 1.0: missing_near += 1
            if first is None: first = [x, y]
    ratio = supported/checked if checked else None
    # A missed new near obstacle is more serious than a missing distant point.
    ready = missing_near == 0 and (checked == 0 or ratio >= .85)
    return dict(ready=ready, checked_hits=checked,
                supported_hits=supported, support_ratio=ratio, outside_hits=outside,
                footprint_or_range_ignored=ignored, first_missing=first,
                unrepresented_near_hits=missing_near,
                reason='ok' if ready else 'current scan hits not yet represented in master costmap')


class SavedCostmapSync:
    def __init__(self, node, scan_monitor, localization, buffer, event,
                 await_future, wait_until, run):
        from geometry_msgs.msg import PolygonStamped
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        self.node, self.scan_monitor, self.localization = node, scan_monitor, localization
        self.buffer, self.event = buffer, event
        self.await_future, self.wait_until, self.run = await_future, wait_until, Path(run)
        self.cycles = {'global': deque(maxlen=160), 'local': deque(maxlen=160)}
        self.subscriptions = []
        self.sequence = 0
        qos = QoSProfile(depth=5, durability=DurabilityPolicy.VOLATILE,
                         reliability=ReliabilityPolicy.RELIABLE)
        for name in ('global', 'local'):
            self.subscriptions.append(node.create_subscription(PolygonStamped,
                '/'+name+'_costmap/published_footprint',
                lambda msg, name=name: self._capture(name, msg), qos))

    def _capture(self, name, msg):
        source = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if msg.header.frame_id.lstrip('/') != ('map' if name == 'global' else 'odom'):
            return
        if not math.isfinite(source) or source <= 0:
            return
        self.cycles[name].append(dict(stamp=source, receipt=time.monotonic()))

    def map_from_odom(self):
        from rclpy.time import Time
        tf = self.buffer.lookup_transform('map', 'odom', Time())
        q = tf.transform.rotation; p = tf.transform.translation
        return [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]

    def _parameters(self, target, names, remaining):
        from rcl_interfaces.srv import GetParameters
        client = self.node.create_client(GetParameters, target+'/get_parameters')
        try:
            self.wait_until(client.service_is_ready, remaining(), target+' parameter discovery')
            response = self.await_future(client.call_async(GetParameters.Request(names=names)), remaining())
            fields = {1:'bool_value', 2:'integer_value', 3:'double_value', 4:'string_value', 9:'string_array_value'}
            values = {}
            for name, p in zip(names, response.values):
                if p.type not in fields: raise RuntimeError('Required costmap parameter missing: '+target+':'+name)
                values[name] = getattr(p, fields[p.type])
            if len(values) != len(names): raise RuntimeError('Incomplete costmap parameters')
            return values
        finally:
            self.node.destroy_client(client)

    def refresh(self, reason):
        """Caller owns a stopped, quiescent, active Nav2 stack throughout."""
        from nav2_msgs.srv import ClearEntireCostmap
        from planner_costmap_audit import snapshot_global_costmap, current_pose, CurrentPoseNotReady
        self.sequence += 1
        diag = dict(reason=reason, sequence=self.sequence, status='checking',
            stamp_semantics='Humble GetCostmap stamp/update_stamp are service-response times, not layer update acknowledgements',
            evidence_semantics='fresh advancing scans plus completed update cycles and master-grid representation; not internal observation-buffer acknowledgement')
        begun = time.monotonic(); deadline = begun+15.
        directory = self.run/'planning_audit'; directory.mkdir(exist_ok=True)
        def remaining():
            value = deadline-time.monotonic()
            if value <= 0: raise RuntimeError('Saved costmap rebuild lacks fresh sensor/map evidence after 15s; see costmap_sync report')
            return value
        self.event('saved_costmap_rebuilding', reason=reason, sequence=self.sequence)
        try:
            # Current settings govern endpoint ranges and static restoration.
            p = self._parameters('/global_costmap/global_costmap',
                ['obstacle_layer.enabled', 'obstacle_layer.scan.marking',
                 'obstacle_layer.scan.observation_persistence', 'obstacle_layer.scan.obstacle_max_range',
                 'obstacle_layer.scan.obstacle_min_range', 'static_layer.enabled',
                 'lethal_cost_threshold'], remaining)
            if not p['static_layer.enabled'] or not p['obstacle_layer.enabled'] or not p['obstacle_layer.scan.marking']:
                raise RuntimeError('Saved costmap rebuild requires active static and scan-marking obstacle layers')
            if p['obstacle_layer.scan.observation_persistence'] != 0.:
                raise RuntimeError('Saved costmap rebuild requires zero observation_persistence so new frames replace pre-reset observations')
            diag['runtime_parameters'] = p
            local_p = self._parameters('/local_costmap/local_costmap',
                ['voxel_layer.enabled', 'voxel_layer.scan.marking',
                 'voxel_layer.scan.observation_persistence'], remaining)
            if not local_p['voxel_layer.enabled'] or not local_p['voxel_layer.scan.marking'] or local_p['voxel_layer.scan.observation_persistence'] != 0.:
                raise RuntimeError('Saved costmap rebuild requires active local scan marking with zero observation_persistence')
            diag['local_runtime_parameters'] = local_p
            baseline = self.map_from_odom()
            for name in ('global', 'local'):
                service = '/'+name+'_costmap/clear_entirely_'+name+'_costmap'
                client = self.node.create_client(ClearEntireCostmap, service)
                try:
                    self.wait_until(client.service_is_ready, remaining(), service+' discovery')
                    self.await_future(client.call_async(ClearEntireCostmap.Request()), remaining())
                finally: self.node.destroy_client(client)
            reset_mono = time.monotonic(); reset_ros = self.node.get_clock().now().nanoseconds/1e9
            diag.update(reset_monotonic=reset_mono, reset_ros_s=reset_ros, map_from_odom_before=baseline)
            last_log = 0.; last_snapshot = 0.; completed = None
            def rebuilt():
                nonlocal last_log, last_snapshot, completed
                now = time.monotonic(); now_ros = self.node.get_clock().now().nanoseconds/1e9
                current = self.map_from_odom(); shift = pose_change(current, baseline)
                diag.update(map_from_odom=current, localization_shift=shift)
                if shift[0] > .04 or shift[1] > .06:
                    raise CostmapLocalizationChanged('Global localization changed during costmap rebuild; stabilize, rebuild and replan')
                barrier = barrier_evidence(self.scan_monitor.snapshot(include_recent=True), self.cycles,
                    reset_mono=reset_mono, reset_ros=reset_ros, now_mono=now, now_ros=now_ros)
                diag['barrier'] = barrier
                if barrier['ready'] and now-last_snapshot >= .2:
                    last_snapshot = now
                    evidence = self.localization.current_scan_evidence(max_range_m=p['obstacle_layer.scan.obstacle_max_range']) or {}
                    diag['scan_evidence'] = {k:v for k,v in evidence.items() if k not in ('map', 'endpoints_map', 'endpoint_ranges')}
                    if (evidence.get('scan_stamp', 0) >= barrier['scan_boundary_stamp']
                            and evidence.get('receipt_monotonic', 0) > reset_mono
                            and now-evidence.get('receipt_monotonic', 0) <= 1.
                            and evidence.get('ready', False)):
                        bounded_future = lambda future, timeout=5.: self.await_future(future, min(timeout, remaining()))
                        grid, info = snapshot_global_costmap(self.node, bounded_future, directory,
                            'costmap-sync-'+str(self.sequence))
                        restored = static_restoration(info, evidence.get('map'), p['lethal_cost_threshold'])
                        try:
                            base = current_pose(self.node, self.buffer, 'map', grid.rules['base_frame'])
                        except CurrentPoseNotReady as exc:
                            diag['pose_wait_reason'] = str(exc)
                            return False
                        represented = scan_representation(info, evidence,
                            min_range=p['obstacle_layer.scan.obstacle_min_range'],
                            max_range=p['obstacle_layer.scan.obstacle_max_range'],
                            base_pose=base,
                            footprint=grid.rules['padded_footprint'])
                        diag.update(static_restoration=restored, scan_representation=represented)
                        after = self.map_from_odom()
                        if max(pose_change(after, baseline)[0]/.04, pose_change(after, baseline)[1]/.06) > 1:
                            raise CostmapLocalizationChanged('Global localization changed while checking rebuilt map; stabilize, rebuild and replan')
                        if restored['ready'] and represented['ready']:
                            completed = dict(map_from_odom=after, diagnostic=diag, snapshot=info)
                            return True
                if now-last_log >= 1.:
                    self.event('saved_costmap_rebuild_wait', reason=reason, elapsed_s=now-begun,
                        barrier=barrier, static_restoration=diag.get('static_restoration'),
                        scan_representation=diag.get('scan_representation'))
                    last_log = now
                return False
            self.wait_until(rebuilt, remaining(), 'fresh scan and completed saved costmap rebuild')
            diag['status'] = 'ready'
            self.event('saved_costmap_rebuilt', reason=reason, elapsed_s=time.monotonic()-begun,
                       map_from_odom=completed['map_from_odom'], barrier=diag['barrier'])
            return completed
        except Exception as exc:
            diag.update(status='localization_changed' if isinstance(exc, CostmapLocalizationChanged) else 'failed', error=str(exc))
            raise
        finally:
            diag['elapsed_s'] = time.monotonic()-begun
            (directory/('costmap_sync-'+str(self.sequence)+'.json')).write_text(
                json.dumps(diag, ensure_ascii=False, indent=2), encoding='utf-8')

    def close(self):
        for sub in self.subscriptions: self.node.destroy_subscription(sub)
        self.subscriptions.clear()
