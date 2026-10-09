"""Bounded local Hybrid-A* and forward/reverse pure pursuit. No ROS dependency."""
from __future__ import division

import heapq
import math
import time
from robot.common.geometry import _footprint_offsets
from robot.common.geometry import bicycle
from robot.common.geometry import collision
from robot.common.geometry import distance
from robot.common.geometry import footprint
from robot.common.geometry import local
from robot.common.geometry import slot_samples
from robot.common.geometry import wrap
from robot.common.geometry import world

try:
    _monotonic = time.monotonic
except AttributeError:
    _monotonic = time.time


def hybrid_plan(start, goal, cfg, obstacles=(), allowed=None, final_direction=-1):
    """Return (path, reason). Path points: x,y,yaw,gear,feedforward_steering.

    Search is bounded; it does not silently substitute a timed manoeuvre when
    geometry is infeasible. Allowed tests the BODY, not just the axle point.
    """
    budget_start = _monotonic()
    if final_direction == -1:
        candidate = _reverse_parking_candidate(start,goal,cfg,obstacles,allowed,
                                               budget_start+cfg['planner_timeout'])
        if candidate:
            return candidate, 'planned'
    step = cfg['planner_step']
    res, ares = cfg['planner_xy_resolution'], cfg['planner_yaw_resolution']
    radius = cfg['wheelbase']/math.tan(cfg['max_steer'])
    reverse = final_direction == -1
    search_start = goal if reverse else start
    search_goal = start if reverse else goal
    initial_gear = -final_direction if reverse else 0
    require_initial_forward = reverse and cfg.get('planner_require_initial_forward', False)
    terminal_gears = (-1,) if require_initial_forward else ((-1, 1) if reverse else (final_direction,))
    footprint_points = (_footprint_offsets(cfg, cfg['planner_body_spacing'])
                        if 'planner_body_spacing' in cfg else _footprint_offsets(cfg))
    started = budget_start
    substeps = max(2, int(math.ceil(step/0.025)))
    from robot.uturn.calibration import limit
    calibrated=cfg.get('planner_calibrated_uturn',False)
    options={}
    for d in (1,-1):
        left=limit(cfg,d,1) if calibrated else cfg['max_steer']
        right=limit(cfg,d,-1) if calibrated else cfg['max_steer']
        options[d]=(-right,-.5*right,0.,.5*left,left)
        if calibrated and cfg.get('planner_full_lock_only',False):
            options[d]=(-right,0.,left)
    # Bicycle primitives are invariant under a rigid planar transform.
    # Calculate their trigonometry once, without coarsening collision samples.
    primitives = {(d,steer): [bicycle((0,0,0),d*step*i/substeps,steer,cfg['wheelbase'])
                              for i in range(1,substeps+1)]
                  for d in (1,-1) for steer in options[d]}
    body_primitives = {}
    if allowed is not None:
        for primitive_key, samples in primitives.items():
            body = []
            for x,y,a in samples:
                c,s = math.cos(a),math.sin(a)
                body.extend((x+c*bx-s*by,y+s*bx+c*by) for bx,by in footprint_points)
            body_primitives[primitive_key] = body
    def cell(value):
        n = int(math.floor(value))
        fraction = value-n
        return n+int(fraction > 0.5 or (fraction == 0.5 and n % 2 != 0))
    steer_change_weight=cfg.get('planner_steer_change_penalty',0.)
    def key(p, gear, cusps, steer=0.):
        # Python 2 and 3 round half-cells differently; keep the search identical.
        result=(cell(p[0]/res), cell(p[1]/res), cell(wrap(p[2])/ares), gear, cusps)
        return result+(steer,) if steer_change_weight else result
    def heuristic(p):
        return 1.6*distance(p, search_goal) + radius*abs(wrap(search_goal[2]-p[2]))
    nodes = [(tuple(search_start), initial_gear, 0, 0, -1, [])]
    queue, best = [(heuristic(search_start), 0.0, 0)], {}
    expansions = 0
    while queue and expansions < cfg['planner_max_expansions']:
        if _monotonic()-started > cfg['planner_timeout']:
            return [], 'planner_timeout'
        _, cost, n = heapq.heappop(queue)
        pose, gear, previous_steer, cusps, parent, _ = nodes[n]
        if cost > best.get(key(pose, gear, cusps,previous_steer), float('inf'))+1e-9:
            continue
        if (distance(pose, search_goal) <= cfg['path_goal_tolerance'] and
                abs(wrap(pose[2]-search_goal[2])) <= cfg['path_yaw_tolerance'] and gear in terminal_gears):
            pieces = []
            while n > 0:
                pieces.append((nodes[n][5], nodes[n][4])); n = nodes[n][4]
            if reverse:
                if not pieces:
                    return [tuple(start) + (final_direction, 0.0)], 'planned'
                path = [tuple(start) + (pieces[0][0][-1][3], 0.0)]
                for part, parent_index in pieces:
                    path.extend(reversed(part))
                    path.append(tuple(nodes[parent_index][0]) + (part[0][3], part[0][4]))
                return path, 'planned'
            return [tuple(start) + (pieces[-1][0][0][3], 0.0)] + [p for part, unused in reversed(pieces) for p in part], 'planned'
        expansions += 1
        pose_c,pose_s = math.cos(pose[2]),math.sin(pose[2])
        for d in (1, -1):
            if not reverse and n == 0 and cfg.get('planner_initial_forward',False) and d != 1:
                continue
            if reverse and n == 0 and d != initial_gear:
                continue
            nc = cusps + int(gear != 0 and gear != d)
            if nc > cfg['parking_max_cusps']:
                continue
            if calibrated:
                steers=options[d]
            elif (reverse and d == initial_gear and
                    abs(wrap(search_start[2]-search_goal[2])) > math.pi/4 and
                    distance(pose, search_start) <= 0.75):
                steers = (-cfg['max_steer'], -0.5*cfg['max_steer'], 0.0,
                          0.5*cfg['max_steer'], cfg['max_steer'])
            else:
                steers = (-cfg['max_steer'], 0.0, cfg['max_steer'])
            for steer in steers:
                turn_direction=cfg.get('planner_turn_direction',0)
                if turn_direction and not reverse:
                    # Explicit maneuver direction: yaw must not turn the other
                    # way, and the first forward primitive must actually turn.
                    signed_yaw=d*steer*turn_direction
                    if signed_yaw<0 or (n==0 and signed_yaw==0):
                        continue
                path_gear = -d if reverse else d
                primitive = primitives[d,steer]
                x,y,a = primitive[-1]
                end = (pose[0]+pose_c*x-pose_s*y,pose[1]+pose_s*x+pose_c*y,wrap(pose[2]+a))
                g = (cost+step*(1.0 if path_gear > 0 else cfg.get('planner_reverse_penalty',1.1))+
                     cfg.get('planner_gear_change_penalty',.25)*(nc-cusps)+.01*abs(steer)+
                     steer_change_weight*abs(steer-previous_steer))
                k = key(end, d, nc,steer)
                # A dominated state cannot enter the queue even if its sweep
                # is clear. Prune it before the expensive body/area checks.
                if g >= best.get(k, float('inf')):
                    continue
                if allowed is not None and not all(
                        allowed((pose[0]+pose_c*x-pose_s*y,pose[1]+pose_s*x+pose_c*y))
                        for x,y in body_primitives[d,steer]):
                    continue
                segment = [(pose[0]+pose_c*x-pose_s*y,pose[1]+pose_s*x+pose_c*y,
                            wrap(pose[2]+a),path_gear,steer) for x,y,a in primitive]
                blocked = False
                for p in segment:
                    if obstacles and collision(p, obstacles, cfg):
                        blocked = True
                        break
                if blocked:
                    continue
                best[k] = g
                idx = len(nodes)
                nodes.append((end, d, steer, nc, n, segment))
                heapq.heappush(queue, (g+heuristic(end), g, idx))
    return [], 'no_path' if not queue else 'planner_expansion_limit'

# Stable public imports for existing tools; implementations have separate owners.
from robot.parking.planner import rank_parking_slots
from robot.parking.planner import parking_area
from robot.parking.planner import auto_parking_plan
from robot.parking.planner import _reverse_parking_candidate
from robot.turn.planner import turn_parameters
from robot.turn.planner import intersection_path
from robot.obstacle.planner import bypass_path
from robot.motion.tracker import Follower
