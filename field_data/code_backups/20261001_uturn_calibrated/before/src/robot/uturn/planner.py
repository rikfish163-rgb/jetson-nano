"""Relative U-turn goals and geometry-derived candidates; no actuator access.

The caller supplies the followed boundary identity and observed drivable area.
Neither is inferred from a missing/opposite boundary or a command-model pose.
"""
import math
import time
from robot.common.geometry import bicycle
from robot.common.geometry import collision
from robot.common.geometry import footprint
from robot.common.geometry import local
from robot.common.geometry import world
from robot.common.geometry import wrap
from robot.common.planning import hybrid_plan


try:
    _isfinite = math.isfinite
except AttributeError:
    def _isfinite(value):
        return not math.isnan(value) and not math.isinf(value)


_DEFAULT_LANE_SPACING = .60
_DEFAULT_GOAL_FORWARD_MIN = 0.0
_DEFAULT_GOAL_FORWARD_MAX = 0.0
_DEFAULT_GOAL_FORWARD_STEP = .10
_DEFAULT_GOAL_HEADING_TOLERANCE_DEG = 10.0
_MAX_LOCAL_DISTANCE = 20.0
_MAX_GOAL_CANDIDATES = 101


def search_options(cfg):
    from robot.uturn.calibration import validate
    validate(cfg)
    mode=cfg.get('uturn_planner_mode','auto')
    if mode not in ('auto','search'):
        raise ValueError('uturn_planner_mode must be auto or search')
    if cfg.get('uturn_calibration') is not None and mode != 'search':
        raise ValueError('calibrated U-turn requires search mode')
    values={}
    for key,default,low,high in (
            ('uturn_max_cusps',6,0,10),
            ('uturn_reverse_penalty',1.1,1,5),
            ('uturn_gear_change_penalty',.25,0,5),
            ('uturn_steer_change_penalty',0.,0,2)):
        value=_finite(cfg.get(key,default),key)
        if not low <= value <= high or (key=='uturn_max_cusps' and value!=int(value)):
            raise ValueError('invalid '+key)
        values[key]=value
    return mode,values

try:
    _monotonic = time.monotonic
except AttributeError:
    _monotonic = time.time


def _finite(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError('invalid '+name)
    if not _isfinite(value):
        raise ValueError('invalid '+name)
    return value


def _pose(start):
    if not isinstance(start, (list, tuple)) or len(start) != 3:
        raise ValueError('invalid start pose')
    result = tuple(_finite(v, 'start pose') for v in start)
    if max(abs(v) for v in result[:2]) > _MAX_LOCAL_DISTANCE:
        raise ValueError('start pose outside local area')
    if not -math.pi <= result[2] <= math.pi:
        raise ValueError('start yaw must be in [-pi,pi]')
    return result


def _lane_spacing(value):
    value = _finite(value, 'lane spacing')
    if not 0.05 <= value <= 2.0:
        raise ValueError('lane spacing must be in [0.05,2.0] m')
    return value


def _goal_forward_range(cfg):
    """Return a bounded, deterministic set of longitudinal goal offsets.

    The map image is not a metric map.  These offsets therefore remain
    configuration inputs and are never inferred from pixels.
    """
    cfg = cfg or {}
    low = _finite(cfg.get('uturn_goal_forward_min', _DEFAULT_GOAL_FORWARD_MIN),
                  'uturn goal forward minimum')
    high = _finite(cfg.get('uturn_goal_forward_max', _DEFAULT_GOAL_FORWARD_MAX),
                   'uturn goal forward maximum')
    step = _finite(cfg.get('uturn_goal_forward_step', _DEFAULT_GOAL_FORWARD_STEP),
                   'uturn goal forward step')
    if not -2.0 <= low <= high <= 2.0:
        raise ValueError('uturn goal forward range must be within [-2,2] m')
    if not 0.01 <= step <= 1.0:
        raise ValueError('uturn goal forward step must be in [0.01,1.0] m')
    estimated_count = int(math.floor((high-low)/step+1e-9))+1
    if estimated_count > _MAX_GOAL_CANDIDATES:
        raise ValueError('uturn goal forward range has too many candidates')
    values = []
    value = low
    while value <= high + 1e-9:
        values.append(round(value, 10))
        value += step
    if not values or values[-1] < high - 1e-9:
        values.append(high)
    # A malformed floating point increment must not create an unbounded loop.
    if len(values) > _MAX_GOAL_CANDIDATES:
        raise ValueError('uturn goal forward range has too many candidates')
    return values


def _goal_heading_tolerance(cfg):
    value = _finite((cfg or {}).get('uturn_goal_heading_tolerance_deg',
                                    _DEFAULT_GOAL_HEADING_TOLERANCE_DEG),
                    'uturn goal heading tolerance')
    if not 1.0 <= value <= 45.0:
        raise ValueError('uturn goal heading tolerance must be in [1,45] deg')
    return math.radians(value)


def uturn_goal(start, followed_boundary, lane_spacing=_DEFAULT_LANE_SPACING,
               forward=0.0):
    start = _pose(start)
    if followed_boundary not in ('LEFT','RIGHT'):
        raise ValueError('followed boundary must be explicitly LEFT or RIGHT')
    lane_spacing = _lane_spacing(lane_spacing)
    forward = _finite(forward, 'goal forward offset')
    if not -2.0 <= forward <= 2.0:
        raise ValueError('goal forward offset must be in [-2,2] m')
    side = -1 if followed_boundary == 'LEFT' else 1
    return tuple(world(start,(forward,side*lane_spacing)))+(wrap(start[2]+math.pi),)


def uturn_goal_candidates(start, followed_boundary, cfg, lane_spacing=None):
    """Return ``(goal_pose, forward_offset)`` candidates in local metric units."""
    cfg = cfg or {}
    spacing = cfg.get('uturn_lane_spacing',
                      _DEFAULT_LANE_SPACING if lane_spacing is None else lane_spacing)
    spacing = _lane_spacing(spacing)
    _goal_heading_tolerance(cfg)
    return [(uturn_goal(start, followed_boundary, spacing, forward), forward)
            for forward in _goal_forward_range(cfg)]


def three_arc_candidate(start, followed_boundary, radius, wheelbase,
                        lane_spacing=_DEFAULT_LANE_SPACING, forward=0.0):
    """F/R/F arcs solve a relative lane shift without fixed timings.

    alpha=acos((2R-spacing)/(4R)); beta=pi-2alpha.
    Every leg increases heading toward the target; reversing flips steering.
    A nonzero ``forward`` offset is completed with a straight terminal leg.
    Steering in the path is always the model angle in radians; the raw
    actuator limit (normally +/-22) is applied by the command adapter.
    """
    start = _pose(start)
    uturn_goal(start, followed_boundary, lane_spacing, forward)
    radius = _finite(radius, 'turn radius')
    wheelbase = _finite(wheelbase, 'wheelbase')
    if not 0 < radius <= 10.0 or not 0 < wheelbase <= 2.0:
        raise ValueError('invalid vehicle geometry')
    if not -2.0 <= _finite(forward, 'goal forward offset') <= 2.0:
        raise ValueError('goal forward offset must be in [-2,2] m')
    if lane_spacing > 2*radius:
        return []
    lane_spacing = _lane_spacing(lane_spacing)
    side=-1 if followed_boundary=='LEFT' else 1
    alpha=math.acos((2*radius-lane_spacing)/(4*radius))
    beta=math.pi-2*alpha
    steering=side*math.atan(wheelbase/radius)
    pose=tuple(start)
    path=[pose+(1,steering)]
    for angle,gear,steer in ((alpha,1,steering),(beta,-1,-steering),(alpha,1,steering)):
        if angle<=1e-10:
            continue
        if path[-1][3]!=gear:
            path.append(pose+(gear,steer))
        count=max(1,int(math.ceil(radius*angle/.025)))
        origin=pose
        for i in range(1,count+1):
            pose=tuple(bicycle(origin,gear*radius*angle*i/count,steer,wheelbase))
            path.append(pose+(gear,steer))
    # At yaw=pi relative to the start, reverse motion advances in the
    # original forward direction.  This realizes the selected x offset while
    # keeping the target heading and lane offset unchanged.
    forward = _finite(forward, 'goal forward offset')
    if abs(forward) > 1e-9:
        gear = -1 if forward > 0 else 1
        length = abs(forward)
        if path[-1][3] != gear:
            path.append(pose+(gear,0.0))
        count=max(1,int(math.ceil(length/.025)))
        origin=pose
        for i in range(1,count+1):
            pose=tuple(bicycle(origin,gear*length*i/count,0.0,wheelbase))
            path.append(pose+(gear,0.0))
    return path


def plan_uturn(start, followed_boundary, cfg, allowed, obstacles=(), lane_spacing=.60,
               goal=None):
    """Return path/reason. Unknown area refuses planning; no live integration.

    Uses sampled body constraints, as does the existing Hybrid-A* planner.
    Callers must conservatively account for map and pose uncertainty in allowed.
    """
    cfg = cfg or {}
    mode,options=search_options(cfg)
    start = _pose(start)
    spacing = cfg.get('uturn_lane_spacing', lane_spacing)
    spacing = _lane_spacing(spacing)
    heading_tolerance = _goal_heading_tolerance(cfg)
    if allowed is None:
        return [],'uturn_area_unknown'

    wheelbase = _finite(cfg.get('wheelbase'), 'wheelbase')
    max_steer = _finite(cfg.get('max_steer'), 'max steer')
    if not 0 < wheelbase <= 2.0 or not 0 < max_steer < math.pi/2:
        raise ValueError('invalid vehicle steering geometry')

    budget = _finite(cfg.get('planner_timeout',6.0), 'planner timeout')
    if budget <= 0:
        raise ValueError('planner timeout must be positive')
    deadline = _monotonic()+budget
    timed_out = [False]
    def clear(path):
        for p in path:
            if _monotonic() >= deadline:
                timed_out[0] = True
                return False
            if collision(p,obstacles,cfg):
                return False
            for q in footprint(p,cfg,spacing=.025):
                if _monotonic() >= deadline:
                    timed_out[0] = True
                    return False
                if not allowed(q):
                    return False
        return True
    radius=wheelbase/math.tan(max_steer)
    preferred=_finite(cfg.get('uturn_turn_radius_m',cfg.get('left_turn_radius',radius)), 'preferred turn radius')
    if preferred <= 0:
        raise ValueError('invalid preferred turn radius')
    # Leave steering authority for tracking error whenever the corridor permits.
    # The tightest full-lock arc is a fallback, not the first candidate.
    radii = [max(radius,preferred)]
    if radii[0] != radius: radii.append(radius)
    if goal is not None:
        goal = _pose(goal)
        candidates = [(goal, local(start, goal)[0])]
        if (abs(local(start, goal)[1] -
                (-1 if followed_boundary == 'LEFT' else 1)*spacing) >
                1e-6 or
                abs(wrap(goal[2]-start[2]-math.pi)) > 1e-6):
            # A replan starts from the measured pose but keeps the original
            # world-frame target.  The analytic F/R/F construction is only
            # valid for its exact relative-lane assumptions; Hybrid-A* is
            # used for the general recovery case below.
            candidates = [(goal, None)]
    else:
        candidates = uturn_goal_candidates(start, followed_boundary, cfg, spacing)
        # Try the smallest longitudinal adjustment first.  The public
        # candidate API keeps its deterministic interval order, while the
        # planner avoids selecting an unnecessarily distant target when the
        # zero-offset lane change is already clear.
        candidates = sorted(candidates, key=lambda item: (abs(item[1]), item[1]))
    endpoint_blocked = True
    feasible = []
    # Evaluate every configured target with the cheap analytic candidate
    # first.  A broad goal window must never multiply the Hybrid-A* budget.
    for goal, forward in candidates:
        if _monotonic() >= deadline:
            return [],'planner_timeout'
        if not clear([start,goal]):
            if timed_out[0]:
                return [],'planner_timeout'
            continue
        endpoint_blocked = False
        feasible.append((goal,forward))
        if forward is not None and mode != 'search':
            for value in radii:
                if _monotonic() >= deadline:
                    return [],'planner_timeout'
                path=three_arc_candidate(start, followed_boundary, value, wheelbase,
                                         spacing, forward)
                if path and clear(path):
                    return path,'uturn_three_arcs'
                if timed_out[0]:
                    return [],'planner_timeout'
    # Use one total budget for all fallback targets.  A local copy prevents
    # this U-turn tolerance from changing normal A* configuration.
    for goal, unused_forward in feasible:
        remaining = deadline-_monotonic()
        if remaining <= 0:
            break
        search_cfg = dict(cfg)
        if cfg.get('uturn_calibration') is not None:
            search_cfg['planner_calibrated_uturn']=True
        if mode == 'search':
            search_cfg.update(parking_max_cusps=int(options['uturn_max_cusps']),
                planner_reverse_penalty=options['uturn_reverse_penalty'],
                planner_gear_change_penalty=options['uturn_gear_change_penalty'],
                planner_steer_change_penalty=options['uturn_steer_change_penalty'],
                planner_initial_forward=True, planner_body_spacing=.05)
            if cfg.get('uturn_calibration') is not None:
                table=cfg['uturn_calibration']
                search_cfg['planner_reverse_penalty'] *= table['forward_mps_per_raw']/table['reverse_mps_per_raw']
        search_cfg['path_yaw_tolerance'] = min(
            float(cfg.get('path_yaw_tolerance', heading_tolerance)),
            heading_tolerance)
        search_cfg['planner_timeout'] = remaining
        path, reason = hybrid_plan(start, goal, search_cfg, obstacles, allowed,
                                   final_direction=1)
        if path and clear(path):
            return path,'uturn_hybrid_search' if mode == 'search' else reason
        if reason == 'planner_timeout' or _monotonic() >= deadline:
            break
    if endpoint_blocked:
        return [],'uturn_endpoint_blocked'
    if _monotonic() >= deadline:
        return [],'planner_timeout'
    return [],'uturn_no_feasible_goal'
