"""Read saved marked-route logs only; also runs over SSH without installation."""
import csv
import json
import math
from pathlib import Path
import re

REQUESTED_RUN = None
PATTERN = r'marked-route-\d{8}-\d{6}(?:-\d+)?'


def read_json(path, default=None):
    try:
        return json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError, ValueError):
        return default


def rows(path):
    if not path.is_file():
        return []
    with path.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def lines(path):
    if not path.is_file():
        return
    with path.open(encoding='utf-8-sig') as f:
        for line in f:
            try:
                yield json.loads(line)
            except ValueError:
                continue


def session_for(run):
    return read_json(run/'session_final.json') or read_json(run/'session.json', {})


def collect(run):
    session = session_for(run)
    if session.get('stage') != 'finished':
        raise ValueError('该轮尚未结束；报告只读取已结束运行。')
    run_id = run.name if re.fullmatch(PATTERN, run.name) else Path(session.get('run', '')).name
    if not re.fullmatch(PATTERN, run_id):
        raise ValueError('这不是标定路线运行记录。')
    states = rows(run/'tracking/closed_loop_state.csv')
    need_states = not states
    imu, transforms = [], []
    for entry in lines(run/'tracking/raw.jsonl'):
        try:
            msg = entry['message']
            if need_states and entry['topic'] == '/racecar_driver/closed_loop_state':
                states.append(json.loads(msg['data']))
            elif entry['topic'] == '/IMU_data':
                stamp = msg['header']['stamp']
                imu.append(dict(stamp=stamp['sec']+stamp['nanosec']*1e-9,
                    angular_velocity=msg.get('angular_velocity', {}), orientation=msg.get('orientation', {}),
                    orientation_available=(msg.get('orientation_covariance') or [0])[0] != -1,
                    gyro_available=(msg.get('angular_velocity_covariance') or [0])[0] != -1))
            elif entry['topic'] == '/tf':
                for tf in msg.get('transforms', []):
                    if tf['header']['frame_id'].lstrip('/') == 'map' and tf['child_frame_id'].lstrip('/') == 'odom':
                        stamp = tf['header']['stamp']
                        transforms.append(dict(stamp=stamp['sec']+stamp['nanosec']*1e-9, **tf['transform']))
        except (KeyError, TypeError, ValueError):
            continue
    # Prefer the small dedicated transform log over the lossy bulk JSON log.
    map_tf_rows = rows(run/'tracking/map_transforms.csv')
    transform_source = 'callback' if (run/'tracking/map_transforms.csv').is_file() else None
    if transform_source:
        transforms = [dict(stamp=float(r['stamp']),
            translation={k:float(r[k]) for k in ('x','y','z')},
            rotation={k:float(r['q'+k]) for k in ('x','y','z','w')}) for r in map_tf_rows]
    else:
        # Existing runs already saved source-time transforms in the monitor.
        # Add them without inventing missing odometry or extrapolating positions.
        history = read_json(run/'saved_map_localization_history.json', [])
        recovered = []
        for r in history if isinstance(history,list) else []:
            if r.get('kind') != 'tf' or r.get('map_odom_stamp') is None:continue
            try:
                x,y,a = r['map_from_odom']
                recovered.append(dict(stamp=float(r['map_odom_stamp']),
                    translation=dict(x=x,y=y,z=0.),rotation=dict(x=0.,y=0.,z=math.sin(a/2),w=math.cos(a/2))))
            except (KeyError,TypeError,ValueError):continue
        if recovered:
            transforms = recovered + transforms
            transform_source = 'saved_history'
    snapshot = read_json(run/'map_snapshot.json')
    map_kind = '本次运行保存的占据地图'
    if not snapshot:
        snapshot = read_json(run/'planning_audit/marked-route.before.costmap.json')
        map_kind = '规划前代价地图快照（含障碍膨胀；不是失败瞬间地图）'
    if not snapshot:
        map_kind = '本轮没有保存地图快照'
    stack_excerpt = []
    if (run/'stack.log').is_file():
        with (run/'stack.log').open(encoding='utf-8', errors='replace') as f:
            for line in f:
                if any(s in line.lower() for s in ('failed to', 'no valid path', 'aborting handle', 'timed out', 'exception', 'error]')):
                    stack_excerpt.append(line.strip()[:1800])
    return dict(run_id=run_id, source=session.get('run', str(run)), session=session,
        metadata=read_json(run/'tracking/metadata.json', {}), states=states, imu=imu,
        poses=rows(run/'tracking/poses.csv'), transforms=transforms,
        map_transform_source=transform_source,
        map_poses=list(lines(run/'map_trajectory.jsonl')), snapshot=snapshot, map_kind=map_kind,
        route=read_json(run/'marked_route.json', {}), live_plan=read_json(run/'live_plan.json', {}),
        plans=list(lines(run/'navigation_plans.jsonl')), tracking=list(lines(run/'current_plan_tracking.jsonl')),
        events=list(lines(run/'events.jsonl')), terminal=read_json(run/'terminal_event.json', {}),
        stop_summary=read_json(run/'stop_summary.json', {}),
        stop_report=(run/'stop_report.md').read_text(encoding='utf-8') if (run/'stop_report.md').is_file() else '',
        stack_excerpt=stack_excerpt[-70:])


def latest(root, requested=None):
    if requested:
        if not re.fullmatch(PATTERN, requested):
            raise ValueError('运行编号不正确。')
        return collect(root/requested)
    skipped = []
    for run in sorted(root.glob('marked-route-*'), reverse=True):
        if session_for(run).get('stage') == 'finished':
            result = collect(run)
            result['skipped_running'] = skipped
            return result
        skipped.append(run.name)
    raise ValueError('车端没有已结束的标定路线记录。')


if __name__ == '__main__':
    print(json.dumps(latest(Path('/home/bianbu/racecar-tests/s-curve-test/runs'), REQUESTED_RUN),
                     ensure_ascii=True, separators=(',', ':')))
