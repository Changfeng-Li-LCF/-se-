"""Smooth S-curve geometry; no ROS or hardware dependencies."""
import json
import math
from pathlib import Path


def make_route(length=4.0, amplitude=0.25, wheelbase=0.248, step=0.02):
    if not all(math.isfinite(v) and v > 0 for v in (length, amplitude, wheelbase, step)):
        raise ValueError('All dimensions must be finite and positive')
    count = max(100, math.ceil(length / step))
    scale = amplitude / (3 * math.sqrt(3) / 8)
    poses = []
    distance = 0.0
    for i in range(count + 1):
        x = length * i / count
        u = 2 * math.pi * i / count
        y = scale * (0.5 * math.sin(u) - 0.25 * math.sin(2 * u))
        dy = scale * math.pi / length * (math.cos(u) - math.cos(2 * u))
        ddy = scale * math.pi**2 / length**2 * (-2 * math.sin(u) + 4 * math.sin(2 * u))
        curvature = ddy / (1 + dy * dy)**1.5
        if poses:
            distance += math.hypot(x - poses[-1]['x_m'], y - poses[-1]['y_m'])
        poses.append(dict(x_m=x, y_m=y, yaw_rad=math.atan(dy),
                          curvature_per_m=curvature, distance_m=distance,
                          model_steering_deg=math.degrees(math.atan(wheelbase * curvature))))
    peak_curvature = max(abs(p['curvature_per_m']) for p in poses)
    return dict(frame='test_start', x_direction='forward', y_direction='left',
                forward_m=length, lateral_amplitude_m=amplitude,
                assumed_wheelbase_m=wheelbase, length_m=distance,
                min_model_radius_m=1 / peak_curvature,
                max_model_steering_deg=max(abs(p['model_steering_deg']) for p in poses),
                poses=poses)


def transform_pose(pose, anchor):
    ax, ay, yaw = anchor
    c, s = math.cos(yaw), math.sin(yaw)
    x, y = pose['x_m'], pose['y_m']
    return ax + c * x - s * y, ay + s * x + c * y, yaw + pose['yaw_rad']


def relative_pose(x, y, yaw, anchor):
    ax, ay, heading = anchor
    c, s = math.cos(heading), math.sin(heading)
    dx, dy = x - ax, y - ay
    return c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(yaw-heading), math.cos(yaw-heading))


def nearest_error(x, y, poses):
    """Return signed distance to nearest path segment and progress in metres."""
    best = None
    for a, b in zip(poses, poses[1:]):
        dx, dy = b['x_m'] - a['x_m'], b['y_m'] - a['y_m']
        length_sq = dx*dx + dy*dy
        if length_sq == 0:
            continue
        t = max(0.0, min(1.0, ((x-a['x_m'])*dx + (y-a['y_m'])*dy) / length_sq))
        ex, ey = x-a['x_m']-t*dx, y-a['y_m']-t*dy
        distance = math.hypot(ex, ey)
        side = dx*ey - dy*ex
        error = math.copysign(distance, side if side != 0 else 1)
        progress = a['distance_m'] + t * math.sqrt(length_sq)
        if best is None or distance < best[0]:
            best = distance, error, progress
    if best is None:
        raise ValueError('Path must contain at least two distinct poses')
    return best[1], best[2]


def load_route(filename):
    data = json.loads(Path(filename).read_text(encoding='utf-8'))
    if data.get('frame') != 'test_start' or len(data.get('poses', [])) < 2:
        raise ValueError('Expected a test_start reference path')
    for p in data['poses']:
        if not all(math.isfinite(p[k]) for k in ('x_m', 'y_m', 'yaw_rad', 'distance_m')):
            raise ValueError('Non-finite reference pose')
    return data
