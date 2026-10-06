"""Select open-space exploration goals; frontier proximity is a soft benefit.

No commands or stop resets. Smac remains responsible for footprint, curvature,
and the actual path; grid reachability alone does not certify a drivable route.
"""
import math
import os
import time
from itertools import chain
import numpy as np
from fast_frontier_kernel import flood_fill_arrays as flood_fill
from exploration_fields import NativeExplorationFields
from boundary_approach import estimate_boundary_normal
from rolling_goal_policy import CONTINUATION_ANGLE_DEG

_PREPARED = None
_FIELD_ENGINE = None
EXPLORATION_CHECK_PERIOD_S = 0.2
EXPLORATION_MIN_GOAL_INTERVAL_S = 0.2


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def grid_from_message(msg):
    q = msg.info.origin.orientation
    angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return dict(frame=msg.header.frame_id, width=msg.info.width, height=msg.info.height,
                resolution=msg.info.resolution,
                origin=[msg.info.origin.position.x, msg.info.origin.position.y, angle],
                data=msg.data)


def erode(mask, steps):
    result = mask.copy()
    for _ in range(steps):
        padded = np.pad(result, 1, constant_values=False)
        result = np.logical_and.reduce([padded[y:y+mask.shape[0], x:x+mask.shape[1]]
                                        for y in range(3) for x in range(3)])
    return result


def clear_grid_chord(mask, first, last, width):
    ax, ay = first % width, first // width
    gx, gy = last % width, last // width
    dx, dy = gx-ax, gy-ay
    denominator = 2*max(abs(dx), abs(dy))
    if not denominator:
        return bool(mask[ay, ax])
    for step in range(denominator+1):
        lx, rx = divmod(ax*denominator+dx*step, denominator)
        ly, ry = divmod(ay*denominator+dy*step, denominator)
        if not (mask[ly, lx] and mask[ly, lx+bool(rx)]
                and mask[ly+bool(ry), lx] and mask[ly+bool(ry), lx+bool(rx)]):
            return False
    return True


def select_frontiers(snapshot, pose, *, clearance_m=.29, step_m=3.0,
                     min_goal_m=1.5, min_frontier_m=.2, max_candidates=8,
                     max_search_cells=60000, exclude=(), separation_m=.6,
                     continuation_goal=None):
    """The historical entry-point name is retained for the session runner.

    Candidates come from known-free space regardless of frontier presence or
    group size. Clearance is preferred, not a mandatory geometric centerline.
    """
    width, height = int(snapshot['width']), int(snapshot['height'])
    res = float(snapshot['resolution'])
    if min(width, height) <= 0 or res <= 0 or not math.isfinite(res):
        raise ValueError('Invalid exploration grid dimensions')
    if width*height > 2_000_000:
        raise ValueError('Exploration grid exceeds bounded selector size')
    if not (math.isfinite(min_goal_m) and math.isfinite(step_m)
            and 0 < min_goal_m <= step_m):
        raise ValueError('Invalid exploration distance range')
    data = np.asarray(snapshot['data'], dtype=np.int16).reshape(height, width)
    ox, oy, angle = snapshot['origin']
    c, s = math.cos(angle), math.sin(angle)

    def cell(x, y):
        dx, dy = x-ox, y-oy
        return (int(math.floor((c*dx+s*dy)/res+1e-9)),
                int(math.floor((-s*dx+c*dy)/res+1e-9)))

    def world(index):
        px, py = (index % width+.5)*res, (index // width+.5)*res
        return ox+c*px-s*py, oy+s*px+c*py

    global _PREPARED, _FIELD_ENGINE
    key = (width, height, res, tuple(snapshot['origin']), clearance_m, snapshot['frame'])
    cached = (_PREPARED is not None and _PREPARED['key'] == key
              and np.array_equal(data, _PREPARED['data']))
    if not cached:
        free = (data >= 0) & (data < 50)
        unknown = data < 0
        padded = np.pad(unknown, 1, constant_values=True)
        boundary = free & (padded[:-2, 1:-1] | padded[2:, 1:-1]
                           | padded[1:-1, :-2] | padded[1:-1, 2:])
        if _FIELD_ENGINE is None:
            _FIELD_ENGINE = NativeExplorationFields()
        fields = _FIELD_ENGINE.compute(free, boundary, copy=True)
        # Exact equivalence to repeated square erosion, including map edges.
        # Reuse the native distance transform instead of allocating nine padded
        # shifted masks for every erosion step after each map update.
        safe = fields.clearance_cells > max(1, math.ceil(clearance_m/res))
        _PREPARED = dict(key=key, data=data.copy(), free=free, unknown=unknown,
                         boundary=boundary, safe=safe, fields=fields, normals={})
    free, unknown = _PREPARED['free'], _PREPARED['unknown']
    boundary, safe = _PREPARED['boundary'], _PREPARED['safe']
    fields, normals = _PREPARED['fields'], _PREPARED['normals']
    ix, iy = cell(pose[0], pose[1])
    diagnostic = dict(frame=snapshot['frame'], frontier_cells=int(boundary.sum()),
                      candidates=0, searched_cells=0, exhausted_search_budget=False,
                      map_masks_cached=cached, search_backend='native_bfs_and_distance_fields',
                      goal_position_policy='open_space_clearance_and_exploration',
                      goal_heading_policy='reliable_boundary_normal_else_direct',
                      frontier_required=False, min_goal_m=min_goal_m)
    if not (0 <= ix < width and 0 <= iy < height) or not free[iy, ix]:
        return [], dict(diagnostic, reason='current_pose_not_in_known_free_space')
    start = iy*width+ix
    raw_distance, _, found, count, exhausted = flood_fill(free, boundary, start, max_search_cells)
    diagnostic.update(searched_cells=count, exhausted_search_budget=exhausted,
                      reachable_frontier_cells=len(found))
    raw_distance = np.asarray(raw_distance, dtype=np.int32)
    if safe.flat[start]:
        distance, _, _, searched, limited = flood_fill(safe, boundary, start, max_search_cells)
        distance = np.asarray(distance, dtype=np.int32)
        reachability = 'clearance_safe_grid'
        diagnostic.update(interior_searched_cells=searched, interior_search_budget_exhausted=limited)
    else:
        # Endpoint safety is unchanged; do not invent a clearance-safe start.
        # Smac must validate the connector from this actual vehicle footprint.
        distance = raw_distance
        reachability = 'known_free_grid_start_outside_clearance'

    def pool_for(distances):
        lengths = distances*res
        pool = np.flatnonzero(safe.ravel() & (lengths >= min_goal_m) & (lengths <= step_m))
        px, py = (pool % width+.5)*res, (pool // width+.5)*res
        wx, wy = ox+c*px-s*py, oy+s*px+c*py
        direct = np.hypot(wx-pose[0], wy-pose[1])
        bearing = np.arctan2(wy-pose[1], wx-pose[0])
        heading = np.abs(np.arctan2(np.sin(bearing-pose[2]), np.cos(bearing-pose[2])))
        visits = np.full(len(pool), step_m)
        for ex, ey, *_ in exclude:
            visits = np.minimum(visits, np.hypot(wx-ex, wy-ey))
        valid = (direct >= min_goal_m) & (heading <= math.pi*.55) & (visits >= separation_m)
        return pool[valid], wx[valid], wy[valid], direct[valid], bearing[valid], heading[valid], visits[valid]

    pool, wx, wy, direct, bearing, heading, visits = pool_for(distance)
    if not len(pool) and distance is not raw_distance:
        # A conservative square margin may split a narrow but orientation-
        # dependent passage. Only relax point reachability, never goal safety.
        distance = raw_distance
        pool, wx, wy, direct, bearing, heading, visits = pool_for(distance)
        reachability = 'known_free_grid_no_safe_horizon'
    diagnostic.update(interior_candidate_cells=len(pool), goal_reachability=reachability)
    if not len(pool):
        return [], dict(diagnostic, reason='no_unvisited_safe_open_space_goal')

    clearance = np.maximum(fields.clearance_cells.ravel()[pool]-1, 0)*res
    frontier_steps = fields.frontier_distance_cells.ravel()
    start_frontier = int(frontier_steps[start])
    progress = np.zeros(len(pool))
    if start_frontier >= 0:
        connected = frontier_steps[pool] >= 0
        progress[connected] = np.clip((start_frontier-frontier_steps[pool[connected]])*res,
                                      -step_m, step_m)
    # Clearance dominates small differences in distance or frontier proximity.
    # A distant wall's unknown region cannot attract across that wall: progress
    # is based on a known-free geodesic field, not straight-line unknown counts.
    costs = (-4.0*np.minimum(clearance, 1.0) - .5*progress
             -.35*np.minimum(visits, 1.0) + .5*np.abs(direct-step_m) + .35*heading)

    # Keep a continuation option in the bounded shortlist. Otherwise global
    # score truncation can hide it before the rolling hysteresis sees it.
    ordered = np.argsort(costs, kind='stable')
    continuation_offsets = []
    anchor_bearing = None
    if continuation_goal is not None:
        gx, gy, *_ = continuation_goal
        old_distance = math.hypot(gx-pose[0], gy-pose[1])
        anchor_bearing = math.atan2(gy-pose[1], gx-pose[0])
        if old_distance < .2 or abs(wrap(anchor_bearing-pose[2])) > math.pi*.55:
            anchor_bearing = pose[2]
        delta = np.abs(np.arctan2(np.sin(bearing-anchor_bearing), np.cos(bearing-anchor_bearing)))
        eligible = ((delta <= math.radians(CONTINUATION_ANGLE_DEG)) & (direct >= old_distance+.15)
                    & (np.hypot(wx-gx, wy-gy) >= .2))
        for offset in ordered[eligible[ordered]]:
            if all(math.hypot(wx[offset]-wx[old], wy[offset]-wy[old]) >= .5
                   for old in continuation_offsets):
                continuation_offsets.append(int(offset))
            if len(continuation_offsets) == 2:
                break
    checked, normal_checks = [], 0
    spacing = .25
    buckets = {}
    for offset in chain(continuation_offsets, ordered):
        chosen = int(pool[offset]); x, y = float(wx[offset]), float(wy[offset])
        bx, by = math.floor(x/spacing), math.floor(y/spacing)
        nearby = (point for dy in (-1, 0, 1) for dx in (-1, 0, 1)
                  for point in buckets.get((bx+dx, by+dy), ()))
        if any(math.hypot(x-px, y-py) < spacing for px, py in nearby):
            continue
        buckets.setdefault((bx, by), []).append((x, y))
        owner = int(fields.owner_source.ravel()[chosen])
        frontier = list(world(owner)) if owner >= 0 else None
        normal = None
        if owner >= 0 and 0 <= frontier_steps[chosen]*res <= 1.2:
            if owner not in normals and normal_checks < 32:
                normals[owner] = estimate_boundary_normal(free, unknown, boundary, owner, res)
                normal_checks += 1
            normal = normals.get(owner)
        normal_yaw = wrap(normal['grid_yaw']+angle) if normal is not None else None
        if normal is not None:
            to_frontier = math.atan2(frontier[1]-y, frontier[0]-x)
            if (abs(wrap(to_frontier-normal_yaw)) > math.pi/3
                    or not clear_grid_chord(free, chosen, owner, width)):
                normal = None
                normal_yaw = None
        alignment = math.cos(wrap(float(bearing[offset])-normal_yaw)) if normal is not None else None
        bonus = -.35*max(0., alignment) if alignment is not None else 0.
        goal_yaw, heading_kind = float(bearing[offset]), 'direct'
        # Do not force a sharp terminal rotation just to satisfy a local edge
        # estimate. The normal is a preference with a direct-heading fallback.
        if normal is not None and abs(wrap(goal_yaw-normal_yaw)) <= math.pi/6:
            goal_yaw, heading_kind = normal_yaw, 'boundary_normal'
        cost = float(costs[offset])+bonus
        checked.append(dict(goal=[x, y, goal_yaw], frontier=frontier,
                            distance_m=float(distance[chosen])*res,
                            score=cost, selection_cost=cost,
                            selection_distance_m=float(direct[offset]),
                            selection_heading_error_rad=float(heading[offset]),
                            selection_preferred_distance_m=step_m,
                            clearance_m=float(clearance[offset]),
                            exploration_progress_m=float(progress[offset]),
                            goal_source='open_space', goal_heading_kind=heading_kind,
                            boundary_normal_yaw=normal_yaw,
                            boundary_normal_alignment=alignment,
                            frontier_distance_m=float(frontier_steps[chosen])*res if owner >= 0 else None))
        if len(checked) >= max(32, max_candidates*16):
            break
    checked.sort(key=lambda p: p['selection_cost'])
    continuing = []
    if anchor_bearing is not None:
        for proposal in checked:
            gx, gy, *_ = proposal['goal']
            if (abs(wrap(math.atan2(gy-pose[1], gx-pose[0])-anchor_bearing)) <= math.radians(CONTINUATION_ANGLE_DEG)
                    and math.hypot(gx-pose[0], gy-pose[1]) >= old_distance+.15
                    and math.dist(proposal['goal'][:2], continuation_goal[:2]) >= .2
                    and all(math.dist(proposal['goal'][:2], p['goal'][:2]) >= .5 for p in continuing)):
                continuing.append(proposal)
            if len(continuing) >= min(2, max_candidates):
                break
    proposals = []
    for proposal in continuing + checked:
        if any(math.dist(proposal['goal'][:2], old['goal'][:2]) < .5 for old in proposals):
            continue
        proposals.append(proposal)
        if len(proposals) >= max_candidates:
            break
    diagnostic.update(candidates=len(proposals), normal_estimates_computed=normal_checks,
                      continuation_candidates_retained=len(continuing),
                      scored_open_space_candidates=len(checked),
                      exploration_signal_available=start_frontier >= 0,
                      reason='open_space_waypoints_available' if proposals else 'no_safe_open_space_waypoint')
    return proposals, diagnostic


def select_frontiers_job(request):
    grid, pose, options = request
    started = time.perf_counter()
    choices, diagnostic = select_frontiers(grid, pose, **options)
    diagnostic.update(pose_map=pose, selector_compute_ms=(time.perf_counter()-started)*1000,
                      selector_worker_pid=os.getpid())
    return choices, diagnostic
