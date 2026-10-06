"""Saved-map startup epochs. No ROS imports, reset commands or motion outputs."""
import math


def epoch_change(previous, current, position_limit_m=.04, angle_limit_rad=.06):
    """Map goals remain authoritative when the map-to-odom transform changes.

    Check both the global transform and vehicle map pose. A stationary local
    odometry window alone does not establish a stable saved-map start.
    """
    changes = {}
    for key in ('map_from_odom', 'pose'):
        a, b = previous.get(key), current.get(key)
        if a is None or b is None or len(a) != 3 or len(b) != 3:
            return dict(changed=True, reason='missing saved-map epoch: ' + key)
        if not all(math.isfinite(v) for v in (*a, *b)):
            return dict(changed=True, reason='nonfinite saved-map epoch: ' + key)
        distance = math.hypot(b[0]-a[0], b[1]-a[1])
        angle = abs(math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2])))
        changes[key] = dict(position_change_m=distance, heading_change_rad=angle)
    changed = any(p['position_change_m'] > position_limit_m or
                  p['heading_change_rad'] > angle_limit_rad for p in changes.values())
    return dict(changed=changed, reason='global localization epoch changed' if changed else 'same epoch',
                changes=changes, position_limit_m=position_limit_m,
                angle_limit_rad=angle_limit_rad)


class SavedLocalizationChanged(RuntimeError):
    """A stopped preparation must regenerate its costmap/plan, never its stop latch."""
