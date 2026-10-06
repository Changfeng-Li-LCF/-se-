"""Offline geometry and metrics. No ROS, network or hardware dependencies."""
import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def generate(radius=1.0, heading_deg=80.0, lead=0.5, step=0.02):
    if not (radius > 0 and 0 < heading_deg < 90 and lead >= 0 and step > 0):
        raise ValueError('Invalid path geometry')
    angle = math.radians(heading_deg)
    curve_length = math.pi * radius * angle
    x = y = s = 0.0
    points = [dict(s=s, x=x, y=y, yaw=0.0, curvature=0.0)]
    for kind, length in [('lead', lead), ('curve', curve_length), ('tail', lead)]:
        count = max(1, math.ceil(length / step))
        ds = length / count
        for i in range(count):
            u = (i + 0.5) * ds
            mid_yaw = angle / 2 * (1 - math.cos(2 * math.pi * u / length)) if kind == 'curve' else 0.0
            x += ds * math.cos(mid_yaw)
            y += ds * math.sin(mid_yaw)
            s += ds
            u = (i + 1) * ds
            yaw = angle / 2 * (1 - math.cos(2 * math.pi * u / length)) if kind == 'curve' else 0.0
            k = math.sin(2 * math.pi * u / length) / radius if kind == 'curve' else 0.0
            points.append(dict(s=s, x=x, y=y, yaw=yaw, curvature=k))
    return {'schema': 1, 'frame': 'start_local', 'radius_min_m': radius,
            'heading_max_deg': heading_deg, 'length_m': s,
            'bounds_m': {'x': [0.0, x], 'y': [0.0, y]}, 'points': points}


def anchor(path, x, y, yaw, frame='odom'):
    c, s = math.cos(yaw), math.sin(yaw)
    result = dict(path, frame=frame, anchor={'x': x, 'y': y, 'yaw': yaw})
    result['points'] = [dict(p, x=x+c*p['x']-s*p['y'], y=y+s*p['x']+c*p['y'],
                             yaw=wrap(yaw+p['yaw'])) for p in path['points']]
    result['bounds_m'] = {axis:[min(p[axis] for p in result['points']),
                              max(p[axis] for p in result['points'])] for axis in ('x','y')}
    return result


def project(path, x, y, yaw):
    best = None
    for a, b in zip(path['points'], path['points'][1:]):
        dx, dy = b['x']-a['x'], b['y']-a['y']
        d2 = dx*dx+dy*dy
        if d2 <= 1e-15:
            continue
        u = min(1.0, max(0.0, ((x-a['x'])*dx+(y-a['y'])*dy)/d2))
        px, py = a['x']+u*dx, a['y']+u*dy
        ex, ey = x-px, y-py
        distance2 = ex*ex+ey*ey
        if best is None or distance2 < best[0]:
            ref_yaw = wrap(a['yaw']+u*wrap(b['yaw']-a['yaw']))
            best = (distance2, {'s_m': a['s']+u*(b['s']-a['s']),
                    'cross_track_m': (dx*ey-dy*ex)/math.sqrt(d2),
                    'heading_error_deg': math.degrees(wrap(yaw-ref_yaw)),
                    'distance_m': math.sqrt(distance2), 'ref_x': px, 'ref_y': py,
                    'curvature': a['curvature']+u*(b['curvature']-a['curvature'])})
    if best is None:
        raise ValueError('Path must contain distinct points')
    return best[1]


def percentile(values, q):
    v = sorted(values)
    if not v:
        return None
    f = (len(v)-1)*q
    i = int(f)
    return v[i]+(v[min(i+1, len(v)-1)]-v[i])*(f-i)


def stats(rows, path):
    valid = [r for r in rows if r.get('quality', 'ok') == 'ok']
    result = {'samples': len(rows), 'valid_samples': len(valid),
              'pose_source': 'estimated localization, not external ground truth'}
    if not valid:
        return dict(result, status='no_data', tracking_metrics=None)
    errors = [dict(r, **project(path, r['x'], r['y'], r['yaw'])) for r in valid]
    span = max(r['s_m'] for r in errors)-min(r['s_m'] for r in errors)
    extent = math.hypot(max(r['x'] for r in valid)-min(r['x'] for r in valid),
                        max(r['y'] for r in valid)-min(r['y'] for r in valid))
    goal = path['points'][-1]
    goal_gap = math.hypot(valid[-1]['x']-goal['x'], valid[-1]['y']-goal['y'])
    result.update(progress_span_m=span, progress_fraction=span/path['length_m'],
                  pose_extent_m=extent, endpoint_gap_m=goal_gap)
    if span < 0.5 or extent < 0.3:
        return dict(result, status='insufficient_motion', tracking_metrics=None)
    metric = {}
    for label, subset in [('all', errors), ('left', [r for r in errors if r['curvature'] > .05]),
                          ('right', [r for r in errors if r['curvature'] < -.05])]:
        if not subset:
            continue
        cross = [r['cross_track_m'] for r in subset]
        metric[label] = {'n': len(subset), 'signed_mean_m': sum(cross)/len(cross),
                         'rmse_m': math.sqrt(sum(v*v for v in cross)/len(cross)),
                         'p95_abs_m': percentile([abs(v) for v in cross], .95),
                         'max_abs_m': max(abs(v) for v in cross),
                         'heading_rmse_deg': math.sqrt(sum(r['heading_error_deg']**2 for r in subset)/len(subset))}
    status = 'complete_geometry' if span/path['length_m'] >= .95 and goal_gap <= .2 else 'partial'
    if len(valid) < len(rows):
        status = 'review_pose_quality'
    return dict(result, status=status, tracking_metrics=metric)


def inferred_pwm(v, w, p):
    """Driver mapping estimate only; never actuator feedback or a serial command."""
    if not all(math.isfinite(t) for t in (v, w)) or abs(v) < p['speed_epsilon_mps']:
        return p['motor_neutral_pwm'], p['servo_center_pwm'], 0.0, False
    requested = math.degrees(math.atan(p['wheelbase_m']*w/v))
    angle = min(p['left_angle_deg'], max(-p['right_angle_deg'], requested))
    endpoint = p['servo_left_pwm'] if angle >= 0 else p['servo_right_pwm']
    full = p['left_angle_deg'] if angle >= 0 else p['right_angle_deg']
    servo = p['servo_center_pwm']+abs(angle)/full*(endpoint-p['servo_center_pwm'])
    speed = min(p['max_speed_mps'], max(-p['max_speed_mps'], v))
    gain = p['forward_pwm_per_mps'] if speed >= 0 else p['reverse_pwm_per_mps']
    motor = min(p['motor_max_pwm'], max(p['motor_min_pwm'], p['motor_neutral_pwm']+speed*gain))
    return math.floor(motor+.5), math.floor(servo+.5), requested, abs(requested-angle)>1e-6
