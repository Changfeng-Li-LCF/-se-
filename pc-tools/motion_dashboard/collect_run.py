"""Read-only collector; executed over SSH or imported for a saved local run."""
import csv
import json
from pathlib import Path
import re

REQUESTED_RUN = None


def read_json(path, default=None):
    try:
        return json.loads(path.read_text(encoding='utf-8-sig'))
    except (OSError, ValueError):
        return default


def read_csv(path):
    if not path.exists():
        return []
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def eligible(run):
    session = read_json(run/'session.json', {})
    return (session.get('stage') == 'finished' and session.get('goal_sent') is True
            and (run/'tracking/metadata.json').is_file())


def collect_run(run):
    if not eligible(run):
        raise ValueError('该记录尚未结束、记录器未完成保存，或没有实际发送行驶任务')
    tracking = run/'tracking'
    states = read_csv(tracking/'closed_loop_state.csv')
    imu = []
    need_states = not states
    # Analysis is asynchronous. Extract only diagnostic messages if its CSV is not ready.
    if (tracking/'raw.jsonl').exists():
        with (tracking/'raw.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                try:
                    entry = json.loads(line)
                    if need_states and entry.get('topic') == '/racecar_driver/closed_loop_state':
                        states.append(dict(receipt_t_s=entry['t'], **json.loads(entry['message']['data'])))
                    elif entry.get('topic') == '/IMU_data':
                        msg = entry['message'];stamp = msg['header']['stamp']
                        imu.append(dict(stamp=stamp['sec']+stamp['nanosec']*1e-9,
                            angular_velocity=msg.get('angular_velocity',{}),
                            orientation=msg.get('orientation',{}),
                            orientation_available=(msg.get('orientation_covariance') or [0])[0] != -1,
                            gyro_available=(msg.get('angular_velocity_covariance') or [0])[0] != -1))
                except (ValueError, KeyError, TypeError):
                    continue
    return dict(run_id=run.name, source=str(run), session=read_json(run/'session.json',{}),
                metadata=read_json(tracking/'metadata.json',{}),
                reference=read_json(run/'reference_world.json',read_json(tracking/'reference_world.json',{})),
                poses=read_csv(tracking/'poses.csv'), states=states, imu=imu)


def collect_latest(root, requested=None):
    if requested:
        if not re.fullmatch(r's-curve-\d{8}-\d{6}(?:-\d+)?',requested):
            raise ValueError('运行编号格式不正确')
        result=collect_run(root/requested)
        result['skipped_newer']=[]
        return result
    skipped=[]
    for run in sorted(root.glob('s-curve-*'), reverse=True):
        if eligible(run):
            result=collect_run(run)
            result['skipped_newer']=skipped
            return result
        skipped.append(run.name)
    raise ValueError('未找到已完成且有行驶记录的运行')


if __name__ == '__main__':
    print(json.dumps(collect_latest(Path('/home/bianbu/racecar-tests/s-curve-test/runs'),REQUESTED_RUN),
                     ensure_ascii=True, separators=(',',':')))
