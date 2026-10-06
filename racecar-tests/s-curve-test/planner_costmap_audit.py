"""Read-only Humble Smac costmap audit. No command/goal publishers.

Uses Nav2 uint8 costs, runtime padded footprint and Smac's collision thresholds.
Path interpolation adds coverage between returned poses; it is not a steering
trajectory generator. Artifacts state this distinction explicitly.
"""
import json
import math
import time
from pathlib import Path


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def line_cells(a, b):
    """Nav2 LineIterator's integer tie convention, including both endpoints."""
    x, y = a
    dx, dy = abs(b[0] - x), abs(b[1] - y)
    sx, sy = (1 if b[0] >= x else -1), (1 if b[1] >= y else -1)
    major, minor = max(dx, dy), min(dx, dy)
    numerator = major // 2
    for _ in range(major + 1):
        yield x, y
        numerator += minor
        if numerator >= major:
            numerator -= major
            if dx >= dy:
                y += sy
            else:
                x += sx
        if dx >= dy:
            x += sx
        else:
            y += sy


def padded_footprint(points, padding):
    if len(points) < 3 or padding < 0:
        raise ValueError('Invalid runtime polygon footprint or padding')
    return [[float(v) + (padding if v > 0 else -padding if v < 0 else 0.) for v in p] for p in points]


def radii(footprint):
    outer = max(math.hypot(*p) for p in footprint)
    inner = outer
    for a, b in zip(footprint, footprint[1:] + footprint[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        den = dx * dx + dy * dy
        t = max(0., min(1., -(a[0] * dx + a[1] * dy) / den)) if den else 0.
        inner = min(inner, math.hypot(a[0] + t * dx, a[1] + t * dy))
    return inner, outer


class CostmapAudit:
    def __init__(self, snapshot, rules):
        self.snapshot, self.rules = snapshot, rules
        self.width, self.height = int(snapshot['width']), int(snapshot['height'])
        self.res = float(snapshot['resolution'])
        self.ox, self.oy = snapshot['origin'][:2]
        if abs(snapshot['origin'][2]) > 1e-6:
            raise ValueError('Nav2 Costmap2D audit requires an axis-aligned costmap')
        self.data = snapshot['data']
        if min(self.width, self.height) <= 0 or not math.isfinite(self.res) or self.res <= 0 or len(self.data) != self.width * self.height:
            raise ValueError('Invalid GetCostmap dimensions')
        if any(v < 0 or v > 255 for v in self.data):
            raise ValueError('GetCostmap must contain uint8 Nav2 costs, not OccupancyGrid probabilities')
        self.footprint = rules.get('collision_check_footprint', rules['padded_footprint'])
        self.radius = max(math.hypot(*p) for p in self.footprint)
        self.bins = int(rules['angle_quantization_bins'])
        self.allow_unknown = bool(rules['allow_unknown'])
        self.possible_cost = float(rules['possible_inscribed_cost'])
        if self.bins <= 0 or rules['downsample_costmap']:
            raise ValueError('Audit requires positive angle bins and the current non-downsampled planner')

    def cell(self, x, y):
        return math.floor((x - self.ox) / self.res), math.floor((y - self.oy) / self.res)

    def detail(self, cell, where):
        ix, iy = cell
        inside = 0 <= ix < self.width and 0 <= iy < self.height
        value = int(self.data[iy * self.width + ix]) if inside else None
        classification = ('outside' if not inside else 'unknown' if value == 255 else
                          'lethal_obstacle' if value == 254 else 'inscribed_inflation' if value == 253 else 'traversable_cost')
        return {'classification': classification, 'cell': [ix, iy], 'cost': value,
                'cell_center': [self.ox + (ix + .5) * self.res, self.oy + (iy + .5) * self.res], 'where': where}

    def pose_rejection(self, pose, quantize=True):
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            return {'classification': 'invalid_pose', 'pose': pose}
        x, y, angle = pose
        center = self.detail(self.cell(x, y), 'center')
        cost = center['cost']
        if cost is None:
            return center
        if self.rules['use_radius']:
            return center if cost >= 253 and not (cost == 255 and self.allow_unknown) else None
        # Same Smac optimized center-cost decision, including unknown policy.
        if self.possible_cost > 0 and cost < self.possible_cost:
            return None
        relaxed = self.rules.get('final_path_collision_tolerance_m', 0.0) > 0.0
        if cost == 254 or (cost == 253 and not relaxed) or (cost == 255 and not self.allow_unknown):
            return center
        if quantize:
            # C++ std::round (ties away from zero), then wrap the orientation bin.
            value = angle * self.bins / (2 * math.pi)
            index = (math.floor(value + .5) if value >= 0 else math.ceil(value - .5)) % self.bins
            angle = index * 2 * math.pi / self.bins
        c, s = math.cos(angle), math.sin(angle)
        vertices = [self.cell(x + c * px - s * py, y + s * px + c * py) for px, py in self.footprint]
        unknown = None
        for vertex in vertices:
            d = self.detail(vertex, 'footprint_vertex')
            if d['cost'] is None:
                return d
        for a, b in zip(vertices, vertices[1:] + vertices[:1]):
            for cell in line_cells(a, b):
                d = self.detail(cell, 'footprint_edge')
                if d['cost'] == 254:
                    return d
                if d['cost'] == 255 and unknown is None:
                    unknown = d
        return unknown if not self.allow_unknown else None

    def audit(self, poses):
        poses = [list(p) for p in poses]
        if not poses:
            return {'clear': False, 'checked_poses': 0, 'first_rejection': {'classification': 'empty_path'}}
        count = 0
        for index, a in enumerate(poses):
            b = poses[min(index + 1, len(poses) - 1)]
            distance, turn = math.hypot(b[0] - a[0], b[1] - a[1]), wrap(b[2] - a[2])
            steps = max(1, math.ceil((distance + abs(turn) * self.radius) / (self.res / 2)))
            for j in range(steps if index + 1 < len(poses) else 1):
                t = j / steps
                pose = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), wrap(a[2] + t * turn)]
                count += 1
                rejection = self.pose_rejection(pose)
                if rejection:
                    return {'clear': False, 'checked_poses': count, 'first_rejection': dict(rejection, segment_index=index, segment_fraction=t, pose=pose)}
        return {'clear': True, 'checked_poses': count, 'first_rejection': None}


def _params(node, target, names, await_future):
    from rcl_interfaces.srv import GetParameters
    client = node.create_client(GetParameters, target + '/get_parameters')
    try:
        if not client.wait_for_service(timeout_sec=8.):
            raise RuntimeError('Runtime parameter service unavailable: ' + target)
        response = await_future(client.call_async(GetParameters.Request(names=names)), 15)
        fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value', 9: 'string_array_value'}
        result = {}
        for name, value in zip(names, response.values):
            if value.type not in fields:
                raise RuntimeError('Required runtime parameter unavailable: ' + target + ':' + name)
            result[name] = getattr(value, fields[value.type])
        if len(result) != len(names):
            raise RuntimeError('Incomplete runtime parameter response: ' + target)
        return result
    finally:
        node.destroy_client(client)


def runtime_rules(node, await_future):
    import yaml
    costmap = _params(node, '/global_costmap/global_costmap',
                      ['footprint', 'footprint_padding', 'robot_radius', 'robot_base_frame', 'plugins'], await_future)
    planner = _params(node, '/planner_server',
                      ['GridBased.allow_unknown', 'GridBased.angle_quantization_bins', 'GridBased.downsample_costmap', 'GridBased.final_path_collision_tolerance_m'], await_future)
    footprint = yaml.safe_load(costmap['footprint']) if costmap['footprint'] else []
    use_radius = not footprint or len(footprint) < 3
    if use_radius:
        r = costmap['robot_radius']
        footprint = [[r * math.cos(i * math.pi / 8), r * math.sin(i * math.pi / 8)] for i in range(16)]
    padded = padded_footprint(footprint, costmap['footprint_padding'])
    collision_tolerance = max(0.0, planner['GridBased.final_path_collision_tolerance_m'])
    collision_footprint = padded if use_radius else [
        [math.copysign(max(0.0, abs(v) - collision_tolerance), v) for v in point]
        for point in padded]
    inner, outer = radii(padded)
    possible, inflation = -1., []
    names = [name + '.plugin' for name in costmap['plugins']]
    plugins = _params(node, '/global_costmap/global_costmap', names, await_future) if names else {}
    for name in costmap['plugins']:
        if plugins[name + '.plugin'].split('::')[-1] != 'InflationLayer':
            continue
        values = _params(node, '/global_costmap/global_costmap',
                         [name + '.cost_scaling_factor', name + '.inflation_radius', name + '.enabled'], await_future)
        weight = values[name + '.cost_scaling_factor']
        possible = 253 if outer <= inner else int(252 * math.exp(-weight * (outer - inner)))
        inflation.append(dict(name=name, **values))
    return {'schema': 1, 'semantics': 'nav2_humble_smac_uint8_center_and_outline',
            'allow_unknown': bool(planner['GridBased.allow_unknown']), 'angle_quantization_bins': int(planner['GridBased.angle_quantization_bins']),
            'downsample_costmap': bool(planner['GridBased.downsample_costmap']), 'base_frame': costmap['robot_base_frame'],
            'use_radius': use_radius, 'footprint': footprint, 'footprint_padding': costmap['footprint_padding'], 'padded_footprint': padded,
            'collision_check_footprint': collision_footprint, 'final_path_collision_tolerance_m': collision_tolerance,
            'possible_inscribed_cost': possible, 'inflation_layers': inflation,
            'sampling': 'half-cell translation plus angular footprint travel; current-to-first-pose is a geometric connector, not a kinematic plan'}


def snapshot_global_costmap(node, await_future, artifact_dir, label, rules=None):
    from nav2_msgs.srv import GetCostmap
    directory = Path(artifact_dir)
    directory.mkdir(parents=True, exist_ok=True)
    info = {'service': '/global_costmap/get_costmap', 'label': label,
            'stamp_semantics': 'service_response_time_not_update_cycle_ack',
            'received_ros_s': node.get_clock().now().nanoseconds * 1e-9}
    client = node.create_client(GetCostmap, info['service'])
    try:
        rules = rules or runtime_rules(node, await_future)
        if not client.wait_for_service(timeout_sec=8.):
            raise RuntimeError('Planner global costmap service unavailable')
        msg = await_future(client.call_async(GetCostmap.Request()), 15).map
        m = msg.metadata
        info.update(frame=msg.header.frame_id, stamp=stamp(msg.header.stamp), update_stamp=stamp(m.update_time),
                    width=m.size_x, height=m.size_y, resolution=m.resolution,
                    origin=[m.origin.position.x, m.origin.position.y, yaw(m.origin.orientation)], data=list(msg.data), rules=rules)
        info['received_ros_s'] = node.get_clock().now().nanoseconds * 1e-9
        if not info['frame'] or info['update_stamp'] <= 0 or not -.05 <= info['received_ros_s'] - info['update_stamp'] <= 3.:
            raise RuntimeError('Planner global costmap has stale/invalid update time')
        grid = CostmapAudit(info, rules)
        return grid, info
    except Exception as e:
        info['error'] = str(e)
        raise
    finally:
        (directory / (label + '.costmap.json')).write_text(json.dumps(info, ensure_ascii=False), encoding='utf-8')
        node.destroy_client(client)


class CurrentPoseNotReady(RuntimeError):
    """Only unavailable/old TF is eligible for a stopped-start retry."""


def current_pose(node, buffer, frame, base_frame, diagnostic=None):
    import rclpy
    from tf2_ros import TransformException
    try:
        tf = buffer.lookup_transform(frame, base_frame, rclpy.time.Time())
    except TransformException as exc:
        raise CurrentPoseNotReady('Current pose transform unavailable: ' + str(exc)) from exc
    now_ros = node.get_clock().now().nanoseconds * 1e-9
    age = now_ros - stamp(tf.header.stamp)
    if diagnostic is not None:
        diagnostic.update(frame=frame, base_frame=base_frame, last_tf_stamp=stamp(tf.header.stamp),
                          last_checked_ros_s=now_ros, last_pose_age_s=age)
    if not -.05 <= age <= .3:
        raise CurrentPoseNotReady('Current pose stale/invalid: age=' + str(age))
    tr = tf.transform
    return [tr.translation.x, tr.translation.y, yaw(tr.rotation)]


def wait_for_current_pose(read_pose, *, wait, diagnostic, timeout_s=2.,
                          check_abort=None, on_wait=None, clock=time.monotonic):
    """Retry only transient TF failures while the caller holds the stop latch.

    wait() services the main ROS executor; TF has its own existing receiver.
    This never rearms the driver, publishes commands, or retries navigation.
    """
    # Saved-map startup may wait without a deadline, still aborting on
    # user stop or stack exit. Other callers retain their finite default.
    if timeout_s is not None and (not math.isfinite(timeout_s) or timeout_s < 0):
        raise ValueError('TF wait timeout must be None or finite and nonnegative')
    begin = clock(); deadline = None if timeout_s is None else begin + timeout_s; last_notice = None
    diagnostic.update(timeout_s=timeout_s, poll_interval_s=.05, attempts=0, waited_s=0., status='checking')
    try:
        while True:
            if check_abort is not None:check_abort()
            diagnostic['attempts'] += 1
            try:
                pose = read_pose()
            except CurrentPoseNotReady as exc:
                diagnostic.update(last_error=str(exc), status='waiting')
                diagnostic.setdefault('first_error', str(exc))
            else:
                if check_abort is not None:check_abort()
                diagnostic.update(status='ready', waited_s=clock()-begin)
                return pose
            now = clock(); diagnostic['waited_s'] = now-begin
            remaining = None if deadline is None else deadline-now
            if remaining is not None and remaining <= 0:
                diagnostic['status'] = 'timeout'
                raise CurrentPoseNotReady('Current pose did not become fresh within '
                    + str(timeout_s) + 's: ' + diagnostic['last_error'])
            if on_wait is not None and (last_notice is None or now-last_notice >= .5):
                on_wait(dict(diagnostic)); last_notice = now
            wait(.05 if remaining is None else min(.05, remaining))
    except Exception:
        diagnostic['waited_s'] = clock()-begin
        if diagnostic['status'] != 'timeout':diagnostic['status'] = 'aborted'
        raise


def transform_poses(buffer, target, source, poses, *, source_stamp=None,
                    transform_tolerance=None, now_ros=None, diagnostic=None):
    """Transform geometry, or a robot sample at its own source time.

    Unstamped fixed route geometry retains its existing latest-frame conversion.
    Robot samples use the same short correction-hold policy as the controller;
    a held correction never changes the robot sample's timestamp.
    """
    if target == source:
        if diagnostic is not None:
            diagnostic.update(pose_stamp=source_stamp, mode='same_frame', correction_lag_s=0.)
        return [list(p) for p in poses]
    import rclpy
    from tf2_ros import ExtrapolationException
    requested=(rclpy.time.Time() if source_stamp is None else
               rclpy.time.Time(nanoseconds=round(source_stamp*1e9)))
    mode='latest_geometry' if source_stamp is None else 'source_time'
    lag=0.
    try:
        tf = buffer.lookup_transform(target, source, requested)
    except ExtrapolationException:
        if source_stamp is None or transform_tolerance is None or now_ros is None:
            raise
        # Do not wait for map/odom while the car is moving. Match the existing
        # controller tolerance, including its separate pose-age and lag checks.
        tf = buffer.lookup_transform(target, source, rclpy.time.Time())
        lag=source_stamp-stamp(tf.header.stamp)
        if lag<0. or lag>transform_tolerance or now_ros-source_stamp>transform_tolerance:
            raise
        mode='held_short_skew'
    if diagnostic is not None:
        diagnostic.update(pose_stamp=source_stamp, correction_stamp=stamp(tf.header.stamp),
                          mode=mode, correction_lag_s=lag)
    tr = tf.transform
    angle = yaw(tr.rotation)
    c, s = math.cos(angle), math.sin(angle)
    return [[tr.translation.x + c * x - s * y, tr.translation.y + s * x + c * y, wrap(a + angle)] for x, y, a in poses]


def recheck_start(node, buffer, await_future, run, plan_result, remaining_index=0, reference_odom=None,
                  pose_wait=None, check_abort=None, on_pose_wait=None, pose_timeout_s=2., reference_map=None):
    """Snapshot-only audit before FollowPath; never resets stops or publishes motion.

    reference_odom can override the original planned odom poses with the exact
    final FollowPath points. The caller owns any movement-tolerance decision.
    """
    directory = Path(run) / 'planning_audit'
    label = 'start-recheck-' + str(time.time_ns())
    result = {'label': label, 'clear': False, 'remaining_index': remaining_index, 'first_rejection': None}
    try:
        grid, snapshot = snapshot_global_costmap(node, await_future, directory, label)
        result.update(frame=snapshot['frame'], costmap_artifact=str(directory / (label + '.costmap.json')), rules=grid.rules)
        pose_diagnostic = result['pose_wait'] = {}
        read_pose = lambda: current_pose(node, buffer, snapshot['frame'], grid.rules['base_frame'],
                                         diagnostic=pose_diagnostic)
        if pose_wait is None:
            current = read_pose()  # Preserve one-shot behavior for other callers.
        else:
            current = wait_for_current_pose(read_pose, wait=pose_wait, diagnostic=pose_diagnostic,
                timeout_s=pose_timeout_s, check_abort=check_abort, on_wait=on_pose_wait)
            # A successful TF retry must not silently authorize an expired map.
            map_age = node.get_clock().now().nanoseconds * 1e-9 - snapshot['update_stamp']
            result['costmap_age_after_pose_wait_s'] = map_age
            while reference_map is not None and map_age > 3.:
                # Waiting for TF must not make a saved-map startup reject its
                # own old snapshot. Retrieve the current grid after the wait.
                grid, snapshot = snapshot_global_costmap(node, await_future, directory, label+'-after-pose-wait')
                result.update(frame=snapshot['frame'],costmap_artifact=str(directory/(label+'-after-pose-wait.costmap.json')),
                              rules=grid.rules,costmap_refreshed_after_pose_wait=True)
                read_pose=lambda:current_pose(node,buffer,snapshot['frame'],grid.rules['base_frame'],diagnostic=pose_diagnostic)
                current=wait_for_current_pose(read_pose,wait=pose_wait,diagnostic=pose_diagnostic,
                    timeout_s=pose_timeout_s,check_abort=check_abort,on_wait=on_pose_wait)
                map_age=node.get_clock().now().nanoseconds*1e-9-snapshot['update_stamp']
                result['costmap_age_after_pose_wait_s']=map_age
            if not -.05 <= map_age <= 3.:
                raise RuntimeError('Planner global costmap expired during pose wait: age=' + str(map_age))
        sequence = reference_map if reference_map is not None else (reference_odom if reference_odom is not None else plan_result['path_odom'])
        if remaining_index < 0 or remaining_index >= len(sequence):
            raise ValueError('Remaining path index out of range')
        source_frame = 'map' if reference_map is not None else 'odom'
        poses = transform_poses(buffer, snapshot['frame'], source_frame, sequence[remaining_index:])
        result['reference_frame'] = source_frame
        result.update(current_pose=current, path=poses, connector=[current, poses[0]])
        for phase, sequence in [('current_footprint', [current]), ('start_connector', [current, poses[0]]), ('remaining_path', poses)]:
            if check_abort is not None:check_abort()
            checked = grid.audit(sequence)
            result[phase] = checked
            if not checked['clear']:
                result.update(reason=phase, first_rejection=checked['first_rejection'])
                return result
        result.update(clear=True, reason='clear')
        return result
    except Exception as e:
        result.update(reason='audit_unavailable', error=str(e), first_rejection={'classification': 'audit_unavailable'})
        return result
    finally:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / (label + '.json')).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
