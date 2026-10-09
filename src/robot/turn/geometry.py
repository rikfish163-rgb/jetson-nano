"""Reusable geometric exit checks; all poses are in the same metric frame.

The planned arc supplies an estimated direction, never a measured heading.
Vision must independently provide a fresh forward-followable exit path.
"""
import math
from robot.common.geometry import local
from robot.common.geometry import world
from robot.common.geometry import wrap
from robot.common.planning import turn_parameters


def arc_exit_reached(pose, exit_pose, action, cfg):
    if exit_pose is None:
        return False
    tail = turn_parameters(cfg, action)['turn_exit'] if action in ('LEFT', 'RIGHT') else 0
    arc_end = tuple(world(exit_pose, (-tail, 0))) + (exit_pose[2],)
    along, lateral = local(arc_end, pose)
    return (along >= 0 and abs(lateral) <= cfg['lane_width'] / 2 and
            abs(wrap(pose[2] - exit_pose[2])) <= cfg['exit_yaw_tolerance'])


def exit_candidate(pose, exit_pose, points, action, cfg):
    """Return a diagnostic reason instead of silently accepting a crossing lane."""
    if exit_pose is None or len(points) < 2:
        return 'missing_path'
    if abs(wrap(pose[2] - exit_pose[2])) > cfg['exit_yaw_tolerance']:
        return 'estimated_yaw'
    a, b = points[:2]
    heading = math.atan2(b[1] - a[1], b[0] - a[0])
    reference = pose[2] if action in ('LEFT', 'RIGHT', 'STRAIGHT', 'UTURN') else exit_pose[2]
    tolerance = (cfg.get('exit_lane_heading_tolerance', math.pi / 3)
                 if action in ('LEFT', 'RIGHT', 'STRAIGHT', 'UTURN') else cfg['exit_yaw_tolerance'])
    if abs(wrap(heading - reference)) >= tolerance:
        return 'path_heading'
    if local(pose, a)[0] <= 0:
        return 'path_behind'
    if abs(local(pose, a)[1]) >= cfg['exit_lateral_tolerance']:
        return 'path_lateral'
    return 'ok'
