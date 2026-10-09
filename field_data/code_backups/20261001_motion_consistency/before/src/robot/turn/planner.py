"""turn_planner: independently testable geometry; no mission state or ROS."""
from __future__ import division
import math
from robot.common.geometry import bicycle


def turn_parameters(cfg, action):
    """One source for planning, exit handoff and displayed left/right settings."""
    prefix = {'LEFT':'left_', 'RIGHT':'right_'}.get(action,'')
    values = dict((key,cfg.get(prefix+key,cfg[key]))
                  for key in ('turn_entry','turn_radius','turn_exit'))
    values['turn_angle_deg'] = cfg.get(prefix+'turn_angle_deg',cfg.get('turn_angle_deg',90.0))
    values['effective_radius'] = max(values['turn_radius'],cfg['wheelbase']/math.tan(cfg['max_steer']))
    if action=='LEFT' and cfg.get('left_turn_full_lock',False):
        values['effective_radius']=cfg['wheelbase']/math.tan(cfg['max_steer'])
    return values


def intersection_path(start, action, cfg):
    pose, path = [start], [tuple(start) + (1, 0.0)]
    def append(length, steer):
        count = max(1, int(math.ceil(length/0.025)))
        for _ in range(count):
            pose[0] = bicycle(pose[0], length/count, steer, cfg['wheelbase'])
            path.append(tuple(pose[0]) + (1, steer))
    if action == 'STRAIGHT':
        append(cfg['straight_distance'], 0.0)
    else:
        turn = turn_parameters(cfg,action)
        sign = 1 if action in ('LEFT', 'UTURN') else -1
        radius = turn['effective_radius']
        append(turn['turn_entry'], 0.0)
        append(radius*(math.pi if action == 'UTURN' else math.radians(turn['turn_angle_deg'])),
               sign*math.atan(cfg['wheelbase']/radius))
        append(turn['turn_exit'], 0.0)
    return path
