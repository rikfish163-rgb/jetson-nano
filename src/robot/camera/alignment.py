"""Measured rear-blue target selection for a bounded U-turn alignment phase."""
import math
from robot.common.geometry import local
from robot.common.geometry import distance
from robot.common.geometry import wrap
from robot.common.contracts import model_to_command_steering


def rear_target(pose, lines, stamp, now, cfg, state):
    if not 0 <= now-stamp <= cfg['sensor_timeout'] or stamp <= state['phase_started']:
        state['rear_blue_reason'] = 'rear_blue_stale'
        return None
    if stamp > state.get('rear_blue_stamp', -1):
        state['rear_blue_stamp'] = stamp
        locked = state.get('rear_blue_target')
        candidates = []
        for line in lines:
            x, y = local(pose, line['point'])
            heading = wrap(line['yaw']-pose[2])
            if not (-1.6 < x < 0 and abs(y) <= cfg['lane_width']/2 and
                    abs(heading) < math.radians(75) and line['length'] >= cfg['blue']['long_min']):
                continue
            if locked is not None and (distance(line['point'], locked['point']) > .15 or
                    abs(wrap(line['yaw']-locked['yaw'])) > math.radians(20)):
                continue
            candidates.append(line)
        current = max(candidates, key=lambda line: local(pose, line['point'])[0]) if candidates else None
        previous = state.get('rear_blue_candidate')
        same = current is not None and previous is not None and distance(current['point'], previous['point']) <= .15
        state['rear_blue_votes'] = state.get('rear_blue_votes', 0)+1 if same else (1 if current else 0)
        state['rear_blue_candidate'] = current
        state['rear_blue_visible'] = current is not None and (locked is not None or state['rear_blue_votes'] >= 2)
        if state['rear_blue_visible']:
            state['rear_blue_target'] = current
    state['rear_blue_reason'] = 'ok' if state.get('rear_blue_visible') else 'rear_blue_unconfirmed'
    return state.get('rear_blue_target') if state.get('rear_blue_visible') else None


def steering_for_heading(heading, cfg):
    """Convert bounded model-wheel correction to the existing command scale."""
    physical = max(-cfg['max_steer'], min(cfg['max_steer'], 1.5*heading))
    return model_to_command_steering(physical,cfg)
