"""Read the same recorded poses used by offline analysis; no ROS dependency."""
import csv
import io
import json
import math
from pathlib import Path


def snapshot(root):
    run = Path((root / 'latest_run.txt').read_text().strip()).resolve()
    if run.parent != (root / 'runs').resolve():
        raise ValueError('Run must belong to this S-curve tool')
    reference_file = run / 'reference_world.json'
    if not reference_file.exists():
        return run.name, [], [], 'Preparing reference'
    ref = json.loads(reference_file.read_text())
    if ref['frame'] != 'odom':
        raise ValueError('Expected odom reference')
    anchor = ref['anchor']
    c, s = math.cos(anchor['yaw']), math.sin(anchor['yaw'])
    def point(row):
        dx, dy = float(row['x'])-anchor['x'], float(row['y'])-anchor['y']
        return c*dx+s*dy, -s*dx+c*dy
    target = [point(p) for p in ref['points']]
    actual = []
    poses = run / 'tracking/poses.csv'
    if poses.exists():
        content = poses.read_text()
        # A concurrent flush can end in a partial row. Retry it next refresh.
        content = content[:content.rfind('\n')+1]
        for row in csv.DictReader(io.StringIO(content)):
            if row.get('quality') != 'ok':
                continue
            p = point(row)
            if all(math.isfinite(v) for v in p):
                actual.append(p)
    status = 'Recording'
    try:
        session = json.loads((run / 'session.json').read_text())
        status = session.get('outcome') or session.get('stage', status)
    except (OSError, ValueError):
        pass
    return run.name, target, actual, status
