"""Pure rolling-target ranking; the caller supplies the last accepted goal.

Candidates remain proposals. This module cannot certify a path, accept a goal,
publish motion, or clear a stop. It does not mutate the supplied candidates.
"""
import math

CONTINUATION_ANGLE_DEG = 35.0


def rolling_distances(speed):
    speed = abs(float(speed))
    if not math.isfinite(speed):
        raise ValueError('Nonfinite rolling speed')
    return 1.5, max(3.0, 3.5 * speed)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _pose(value, label):
    try:
        result = tuple(float(v) for v in value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError('Invalid ' + label) from error
    if len(result) != 3 or not all(math.isfinite(v) for v in result):
        raise ValueError('Invalid ' + label)
    return result


def choose_rolling_goal(candidates, pose, current_goal, speed, *,
                        continuation_angle_deg=CONTINUATION_ANGLE_DEG,
                        direction_penalty_weight=.5, switch_margin=.25):
    """Return a candidate copy with decision metadata, or None if none qualifies.

    current_goal must be the last planner-accepted goal, not an unaccepted
    publication. Direction is measured from today's pose towards that goal so
    the reference follows a bend. Close/passed goals use the current body yaw.

    Continuation candidates must pass the same distance/forward/extension gates
    as every other candidate. A shorter, ineligible point cannot prevent a turn
    when no eligible continuation exists. A competing direction must improve
    the score by switch_margin over the best eligible continuation after a
    small, bounded direction penalty; sufficiently better clearance may win.
    """
    minimum, preferred = rolling_distances(speed)
    pose = _pose(pose, 'current pose')
    accepted = _pose(current_goal, 'accepted goal') if current_goal is not None else None
    parameters = (continuation_angle_deg, direction_penalty_weight, switch_margin)
    if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in parameters)
            or not 0 < continuation_angle_deg <= 180
            or direction_penalty_weight < 0 or switch_margin < 0):
        raise ValueError('Invalid rolling continuity parameters')
    continuation_angle = math.radians(continuation_angle_deg)
    old_distance = math.dist(pose[:2], accepted[:2]) if accepted is not None else 0.
    anchor = pose[2]
    anchor_kind = 'body_yaw_no_accepted_goal'
    if accepted is not None:
        goal_bearing = math.atan2(accepted[1] - pose[1], accepted[0] - pose[0])
        if old_distance >= .2 and abs(_wrap(goal_bearing - pose[2])) <= math.pi * .55:
            anchor, anchor_kind = goal_bearing, 'bearing_to_accepted_goal'
        else:
            anchor_kind = 'body_yaw_accepted_goal_close_or_passed'

    choices = []
    for candidate in candidates:
        try:
            goal = _pose(candidate['goal'], 'candidate goal')
            distance = math.dist(pose[:2], goal[:2])
            if distance < minimum:
                continue
            bearing = math.atan2(goal[1] - pose[1], goal[0] - pose[0])
            heading = abs(_wrap(bearing - pose[2]))
            if heading > math.pi * .55:
                continue
            if accepted is not None:
                if math.dist(goal[:2], accepted[:2]) < .2 or distance < old_distance + .15:
                    continue
            if 'selection_cost' in candidate:
                values = tuple(float(candidate[key]) for key in (
                    'selection_cost', 'selection_distance_m',
                    'selection_heading_error_rad', 'selection_preferred_distance_m'))
                if not all(math.isfinite(v) for v in values):
                    continue
                score, previous_distance, previous_heading, previous_preferred = values
                if previous_distance < 0 or previous_heading < 0 or previous_preferred <= 0:
                    continue
                base_score = (score
                              + .5 * (abs(distance - preferred)
                                      - abs(previous_distance - previous_preferred))
                              + .35 * (heading - previous_heading))
            else:
                score = float(candidate.get('score', 0.))
                if not math.isfinite(score):
                    continue
                base_score = abs(distance - preferred) + .5 * heading + .05 * score
            delta = abs(_wrap(bearing - anchor))
            penalty = direction_penalty_weight * (1. - math.cos(delta)) if accepted is not None else 0.
            adjusted_score = base_score + penalty
            if not math.isfinite(adjusted_score):
                continue
        except (KeyError, TypeError, ValueError, OverflowError):
            # Malformed/nonfinite proposals cannot participate in ranking.
            continue
        choices.append(dict(candidate=candidate, goal=goal, distance=distance,
                            heading_delta=delta, base_score=base_score,
                            direction_penalty=penalty, adjusted_score=adjusted_score,
                            continuation=accepted is not None and delta <= continuation_angle))
    if not choices:
        return None

    best = min(choices, key=lambda item: item['adjusted_score'])
    continuing = [item for item in choices if item['continuation']]
    baseline = min(continuing, key=lambda item: item['adjusted_score']) if continuing else None
    winner = best
    if accepted is None:
        decision = 'initial_best_score'
    elif baseline is None:
        decision = 'switch_no_eligible_continuation'
    elif best['continuation']:
        decision = 'continue_accepted_direction'
    elif baseline['adjusted_score'] - best['adjusted_score'] >= switch_margin:
        decision = 'switch_meaningful_score_advantage'
    else:
        winner = baseline
        decision = 'continue_switch_advantage_below_margin'

    result = dict(winner['candidate'])
    result['rolling_policy'] = dict(
        decision=decision,
        accepted_goal=list(accepted) if accepted is not None else None,
        direction_anchor_yaw=anchor, direction_anchor_kind=anchor_kind,
        heading_delta_rad=winner['heading_delta'],
        heading_delta_deg=math.degrees(winner['heading_delta']),
        continuation_angle_deg=continuation_angle_deg,
        eligible_candidates=len(choices), eligible_continuation_candidates=len(continuing),
        accepted_distance_m=old_distance, candidate_distance_m=winner['distance'],
        base_score=winner['base_score'], direction_penalty=winner['direction_penalty'],
        adjusted_score=winner['adjusted_score'], required_switch_margin=switch_margin,
        continuation_baseline_goal=list(baseline['goal']) if baseline else None,
        continuation_baseline_score=baseline['adjusted_score'] if baseline else None,
        best_adjusted_candidate_goal=list(best['goal']),
        best_adjusted_score=best['adjusted_score'],
        best_advantage_over_continuation=(baseline['adjusted_score'] - best['adjusted_score'])
        if baseline else None)
    return result
