"""obstacle_planner: independently testable geometry; no mission state or ROS."""
from __future__ import division
import math
from robot.common.geometry import wrap
from robot.common.geometry import world


def bypass_path(start, cfg):
    """Continuous shift out, parallel pass, shift back. Validate complete sweep."""
    length, offset, passing = cfg['bypass_transition'], cfg['bypass_offset'], cfg['bypass_pass']
    total = 2*length+passing
    path = []
    for i in range(int(math.ceil(total/0.025))+1):
        x = min(total, i*0.025)
        if x <= length:
            t, sign = x/length, 1
        elif x >= length+passing:
            t, sign = (x-length-passing)/length, -1
        else:
            t, sign = 1, 0
        s = 10*t**3-15*t**4+6*t**5
        y = offset*(s if sign == 1 else 1-s if sign == -1 else 1)
        dy = sign*offset*(30*t*t-60*t**3+30*t**4)/length
        ddy = sign*offset*(60*t-180*t*t+120*t**3)/(length*length)
        steer = math.atan(cfg['wheelbase']*ddy/(1+dy*dy)**1.5)
        if abs(steer) > cfg['max_steer']:
            return []
        path.append(tuple(world(start, (x,y))) + (wrap(start[2]+math.atan(dy)), 1, steer))
    return path
