"""Bounded in-memory latest plan, with full replanning history streamed to disk."""
import json
import math


def reference_from_poses(frame, poses):
    points = []
    length = 0.0
    for x, y, heading in poses:
        if not all(math.isfinite(v) for v in (x, y, heading)):
            raise ValueError('Nonfinite planned pose')
        if points:
            distance = math.hypot(x-points[-1]['x'], y-points[-1]['y'])
            if distance < 1e-8:
                continue
            length += distance
        points.append(dict(x=x, y=y, yaw=heading, s=length, curvature=0.0))
    if not frame or len(points) < 2:
        raise ValueError('Empty frame or degenerate plan')
    for a, b in zip(points, points[1:]):
        delta = b['yaw']-a['yaw']
        a['curvature'] = math.atan2(math.sin(delta), math.cos(delta))/(b['s']-a['s'])
    points[-1]['curvature'] = points[-2]['curvature']
    return dict(frame=frame, points=points, length_m=length)


class NavigationPlanLog:
    def __init__(self, path, sink=None):
        self.path = path
        self.sink = sink
        self.latest = None
        self.sequence = 0
        self.dropped = 0

    def capture(self, frame, poses, stamp, received):
        reference = reference_from_poses(frame, poses)
        self.sequence += 1
        entry = dict(sequence=self.sequence, stamp=stamp, received_ros_s=received,
                     source='/plan', role='planner output; not measured motion',
                     **reference)
        # A supplied sink owns copying/queuing. Retain current geometry even
        # when a congested diagnostic queue drops its history entry.
        if self.sink is not None:
            if not self.sink(entry):
                self.dropped += 1
        else:
            with self.path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(entry, separators=(',', ':'))+'\n')
        self.latest = entry
        return entry
