"""Optional exploration goal headings; no route or motion is authorized here."""
import math


def heading_preferences(start, goal):
    """Try the selected approach heading, then direct bearing if distinct.

    Open-space selection supplies a local boundary normal only when reliable
    and already close to the direct approach; ordinary goals use direct yaw.
    """
    dx, dy = goal[0]-start[0], goal[1]-start[1]
    preferred = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else goal[2]
    delta = math.atan2(math.sin(preferred-goal[2]), math.cos(preferred-goal[2]))
    return [goal[2], preferred] if abs(delta) > 1e-4 else [preferred]
