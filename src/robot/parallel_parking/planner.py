# -*- coding: utf-8 -*-
"""Measured-scene parallel parking planner and executor.

This module is an independent first framework for the P1/P2/P3 side-facing
bays.  It intentionally does not share the P4/P5 perpendicular-bay entry
logic.  The caller supplies a fixed local scene and a measured pose; a
command-model pose is rejected at the input boundary.

The planner prefers a geometry-derived reverse double arc.  If that sweep is
not legal in the observed free-space polygons, it falls back to the bounded
Hybrid-A* planner already used by the competition stack.  Both paths are
checked with the complete vehicle footprint.  The executor returns the
Follower's physical steering angle; the controller owns command/raw encoding.
"""
from __future__ import division

import math

from robot.common.contracts import number
from robot.common.geometry import bicycle
from robot.common.geometry import collision
from robot.common.geometry import distance
from robot.common.geometry import footprint
from robot.common.geometry import inside_slot
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.common.geometry import world
from robot.common.planning import Follower
from robot.common.planning import hybrid_plan


try:
    _string_types = (basestring,)
except NameError:
    _string_types = (str,)


_EPS = 1e-9

try:
    _isfinite = math.isfinite
except AttributeError:
    def _isfinite(value):
        return not math.isnan(value) and not math.isinf(value)


def _finite_pose(value, label):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError('invalid %s' % label)
    pose = tuple(number(v) for v in value)
    if max(abs(v) for v in pose[:2]) > 20:
        raise ValueError('%s outside local area' % label)
    if not -math.pi <= pose[2] <= math.pi:
        raise ValueError('%s yaw must be in [-pi,pi]' % label)
    return pose


def _orientation(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) -
            (b[1] - a[1]) * (c[0] - a[0]))


def _on_segment(a, b, p):
    return (min(a[0], b[0]) - _EPS <= p[0] <= max(a[0], b[0]) + _EPS and
            min(a[1], b[1]) - _EPS <= p[1] <= max(a[1], b[1]) + _EPS)


def _segments_intersect(a, b, c, d):
    first = _orientation(a, b, c)
    second = _orientation(a, b, d)
    third = _orientation(c, d, a)
    fourth = _orientation(c, d, b)
    if ((first > _EPS and second < -_EPS) or
            (first < -_EPS and second > _EPS)) and ((third > _EPS and
            fourth < -_EPS) or (third < -_EPS and fourth > _EPS)):
        return True
    return ((abs(first) <= _EPS and _on_segment(a, b, c)) or
            (abs(second) <= _EPS and _on_segment(a, b, d)) or
            (abs(third) <= _EPS and _on_segment(c, d, a)) or
            (abs(fourth) <= _EPS and _on_segment(c, d, b)))


def _polygon_contains(polygon, point, tolerance=1e-9):
    """Return true for a point inside or on a polygon boundary."""
    x, y = point
    inside = False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= _EPS:
            continue
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
        qx, qy = ax + t * dx, ay + t * dy
        if math.hypot(x - qx, y - qy) <= tolerance:
            return True
        if (ay > y) != (by > y):
            crossing = ax + (y - ay) * dx / dy
            if x < crossing:
                inside = not inside
    return inside


def _compile_regions(regions):
    """Precompute strict-convex half-planes for the 20 Hz control loop."""
    compiled = []
    for polygon in regions:
        signed_area = sum(a[0] * b[1] - b[0] * a[1]
                          for a, b in zip(polygon, polygon[1:] + polygon[:1]))
        orientation = 1.0 if signed_area >= 0 else -1.0
        edges = []
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            edges.append((a[0], a[1], b[0] - a[0], b[1] - a[1]))
        compiled.append((orientation, tuple(edges)))
    return tuple(compiled)


def _compiled_region_contains(compiled, point):
    x, y = point
    for orientation, edges in compiled:
        if all(orientation * (dx * (y - ay) - dy * (x - ax)) >= -1e-9
               for ax, ay, dx, dy in edges):
            return True
    return False


def _decode_polygon(rows):
    if not isinstance(rows, list) or not 3 <= len(rows) <= 64:
        raise ValueError('invalid parking region')
    polygon = []
    for row in rows:
        if not isinstance(row, (list, tuple)) or len(row) != 2:
            raise ValueError('invalid parking region point')
        point = tuple(number(v) for v in row)
        if max(abs(v) for v in point) > 20:
            raise ValueError('parking region point outside local area')
        polygon.append(point)
    if len(set(polygon)) != len(polygon):
        raise ValueError('parking region has duplicate vertices')
    # The planner accepts a union of convex patches.  This prevents a
    # self-crossing polygon from accidentally making an unknown area free.
    turns = []
    for i in range(len(polygon)):
        a, b, c = polygon[i - 2], polygon[i - 1], polygon[i]
        turns.append((b[0] - a[0]) * (c[1] - b[1]) -
                     (b[1] - a[1]) * (c[0] - b[0]))
    if not (all(value > _EPS for value in turns) or
            all(value < -_EPS for value in turns)):
        raise ValueError('parking region must be strictly convex')
    for i, first in enumerate(polygon):
        second = polygon[(i + 1) % len(polygon)]
        for j in range(i + 1, len(polygon)):
            if j in ((i - 1) % len(polygon), i, (i + 1) % len(polygon)):
                continue
            other = polygon[j]
            other_end = polygon[(j + 1) % len(polygon)]
            if _segments_intersect(first, second, other, other_end):
                raise ValueError('parking region self-intersects')
    return polygon


def _decode_obstacles(rows):
    if not isinstance(rows, list) or len(rows) > 2000:
        raise ValueError('invalid parking obstacles')
    result = []
    for row in rows:
        if not isinstance(row, (list, tuple)) or len(row) != 2:
            raise ValueError('invalid parking obstacle')
        point = tuple(number(v) for v in row)
        if max(abs(v) for v in point) > 20:
            raise ValueError('parking obstacle outside local area')
        result.append(point)
    return result


def decode_parallel_scene(data, allow_perpendicular=False):
    """Validate and normalize a measured parallel-parking scene.

    Required fields are ``frame``, ``pose``, ``stamp``, ``pose_source``,
    ``regions`` and ``slot``.  ``slot.pose`` is the slot centre and heading;
    the P1/P2/P3 nominal dimensions are 0.70 by 0.36 m, but dimensions are
    always taken from the observed/configured slot so they remain tunable.
    """
    if not isinstance(data, dict):
        raise ValueError('parking scene must be an object')
    frame = data.get('frame')
    if not isinstance(frame, _string_types) or not frame or len(frame) > 100:
        raise ValueError('invalid parking scene frame')
    source = data.get('pose_source')
    if source not in ('vision', 'lidar', 'fused', 'odom'):
        raise ValueError('measured parking pose required')
    pose = _finite_pose(data.get('pose'), 'parking scene pose')
    stamp = number(data.get('stamp'))
    regions = data.get('regions')
    if not isinstance(regions, list) or not 1 <= len(regions) <= 16:
        raise ValueError('unknown parking drivable area')
    regions = [_decode_polygon(region) for region in regions]
    slot = data.get('slot')
    if not isinstance(slot, dict):
        raise ValueError('parking slot is required')
    slot_id = slot.get('id')
    if not isinstance(slot_id, _string_types) or not slot_id or len(slot_id) > 32:
        raise ValueError('invalid parallel parking slot id')
    kind = slot.get('kind') or 'parallel'
    allowed_kinds = ('parallel', 'perpendicular') if allow_perpendicular else ('parallel',)
    if kind not in allowed_kinds:
        raise ValueError('parallel parking requires a supported slot kind')
    if not allow_perpendicular and slot_id in ('P4', 'P5'):
        raise ValueError('parallel parking requires a side-facing slot')
    if kind == 'perpendicular' and slot_id not in ('P4', 'P5'):
        raise ValueError('perpendicular parking requires P4 or P5')
    if kind == 'parallel' and slot_id in ('P4', 'P5'):
        raise ValueError('P4/P5 require a perpendicular slot')
    if kind == 'perpendicular' and slot_id in ('P1', 'P2', 'P3'):
        raise ValueError('P1/P2/P3 require a parallel slot')
    slot_pose = _finite_pose(slot.get('pose'), 'parking slot pose')
    length = number(slot.get('length'))
    width = number(slot.get('width'))
    if not 0 < length <= 5 or not 0 < width <= 5:
        raise ValueError('invalid parallel parking slot dimensions')
    result = dict(frame=frame, pose=pose, stamp=stamp, pose_source=source,
                  regions=regions,
                  obstacles=_decode_obstacles(data.get('obstacles', [])),
                  slot=dict(id=slot_id, kind=kind, pose=slot_pose,
                            length=length, width=width))
    if 'observation_source' in data or 'source' in data:
        observation_source = data.get('observation_source', data.get('source'))
        if observation_source not in ('front', 'rear'):
            raise ValueError('invalid parking observation source')
        result['observation_source'] = observation_source
    # Camera-produced scenes carry readiness and occupancy evidence. Keep
    # those fields through the wire decoder so an active task can stop on a
    # fresh UNKNOWN/OCCUPIED frame. Metadata-free geometric payloads remain
    # accepted here for low-level planner/replay callers; the runtime
    # controller applies the production readiness gate before motion.
    if 'ready' in data:
        if type(data['ready']) is not bool:
            raise ValueError('invalid parallel parking readiness')
        result['ready'] = data['ready']
    if 'occupancy' in data:
        occupancy = data['occupancy']
        if (not isinstance(occupancy, _string_types) or
                occupancy not in ('FREE', 'OCCUPIED', 'UNKNOWN')):
            raise ValueError('invalid parallel parking occupancy')
        result['occupancy'] = occupancy
    if 'occupancy_detail' in data:
        if not isinstance(data['occupancy_detail'], dict):
            raise ValueError('invalid parallel parking occupancy detail')
        result['occupancy_detail'] = dict(data['occupancy_detail'])
    if 'confirmations' in data:
        confirmations = number(data['confirmations'])
        if confirmations != int(confirmations) or confirmations < 0:
            raise ValueError('invalid parallel parking confirmations')
        result['confirmations'] = int(confirmations)
    if 'target_id' in data:
        target_id = data['target_id']
        if (not isinstance(target_id, _string_types) or
                not target_id or len(target_id) > 32):
            raise ValueError('invalid parallel parking target id')
        result['target_id'] = target_id
    return result


def parallel_goal(start, slot, wheelbase=.26):
    """Return a rear-axle pose centred in a side-facing bay.

    ``slot.pose`` is its geometric centre and heading.  The optional
    wheelbase argument keeps this helper useful to callers that do not yet
    have a complete vehicle configuration.
    """
    del start  # The goal is expressed in the slot frame; start is API context.
    slot_pose = _finite_pose(slot.get('pose'), 'parking slot pose')
    if not 0 < number(slot.get('length')) or not 0 < number(slot.get('width')):
        raise ValueError('invalid parallel parking slot dimensions')
    wheelbase = number(wheelbase)
    if wheelbase <= 0:
        raise ValueError('invalid wheelbase')
    xy = world(slot_pose, (-wheelbase / 2.0, 0.0))
    return tuple(xy) + (slot_pose[2],)


def _goal_from_cfg(start, slot, cfg):
    """Build the goal without requiring a constructor-specific pose type."""
    return parallel_goal(start, slot, number(cfg['wheelbase']))


def _cfg_value(cfg, key, default):
    value = cfg.get(key, default)
    value = number(value)
    if value <= 0:
        raise ValueError('invalid '+key)
    return value


def _cfg_nonnegative(cfg, key, default):
    value = number(cfg.get(key, default))
    if value < 0:
        raise ValueError('invalid '+key)
    return value


def _path_point(pose, gear, steering):
    return tuple(pose) + (int(gear), float(steering))


def _area_allowed(allowed, pose, cfg, obstacles):
    if collision(pose, obstacles, cfg):
        return False
    return all(allowed(point) for point in footprint(pose, cfg))


def _append_segment(path, pose, length, gear, steering, cfg, allowed, obstacles):
    """Append a constant-curvature segment and validate every body sample."""
    if length <= _EPS:
        return tuple(pose)
    if path and path[-1][3] != gear:
        path.append(_path_point(pose, gear, steering))
    elif not path:
        path.append(_path_point(pose, gear, steering))
    step = _cfg_value(cfg, 'parallel_parking_sample_step_m', .025)
    count = max(1, int(math.ceil(length / step)))
    origin = tuple(pose)
    for index in range(1, count + 1):
        pose = tuple(bicycle(origin, gear * length * index / count,
                             steering, cfg['wheelbase']))
        if not _area_allowed(allowed, pose, cfg, obstacles):
            return None
        path.append(_path_point(pose, gear, steering))
    return tuple(pose)


def double_arc_candidate(start, slot, cfg, allowed, obstacles=()):
    """Generate a reverse double-arc candidate for a parallel bay.

    The first reverse arc moves toward the bay and the second applies the
    mirrored steering to restore the slot heading.  A straight setup prefix
    is derived from the measured slot centre.  The candidate is rejected when
    its target heading, sweep, or complete vehicle footprint is not legal.
    """
    if allowed is None:
        return None
    goal = _goal_from_cfg(start, slot, cfg)
    relative = local(start, goal)
    yaw_error = abs(wrap(goal[2] - start[2]))
    yaw_limit = math.radians(_cfg_value(
        cfg, 'parallel_parking_double_arc_yaw_tolerance_deg', 3.0))
    if yaw_error > yaw_limit:
        return None
    lateral = relative[1]
    if abs(lateral) <= .01:
        return None
    wheelbase = number(cfg['wheelbase'])
    max_steer = number(cfg['max_steer'])
    if not 0 < wheelbase or not 0 < max_steer < math.pi / 2:
        raise ValueError('invalid vehicle steering geometry')
    kinematic_radius = wheelbase / math.tan(max_steer)
    requested_radius = _cfg_value(cfg, 'parallel_parking_turn_radius_m',
                                  kinematic_radius)
    radius = max(kinematic_radius, requested_radius)
    if abs(lateral) > 2.0 * radius:
        return None
    theta = math.acos(max(-1.0, min(1.0, 1.0 - abs(lateral) / (2.0 * radius))))
    steering = math.atan(wheelbase / radius)
    side = 1.0 if lateral > 0 else -1.0
    # Two equal reverse arcs displace x by -2R sin(theta).  Positioning their
    # start at this derived x makes the terminal rear axle exactly the goal.
    reverse_x = -2.0 * radius * math.sin(theta)
    setup_x = relative[0] - reverse_x
    max_setup = _cfg_value(cfg, 'parallel_parking_max_setup_m', 2.0)
    if abs(setup_x) > max_setup:
        return None

    path = []
    pose = tuple(start)
    if setup_x > _EPS:
        pose = _append_segment(path, pose, setup_x, 1, 0.0,
                               cfg, allowed, obstacles)
    elif setup_x < -_EPS:
        pose = _append_segment(path, pose, -setup_x, -1, 0.0,
                               cfg, allowed, obstacles)
    if pose is None:
        return None
    pose = _append_segment(path, pose, radius * theta, -1,
                           side * steering, cfg, allowed, obstacles)
    if pose is None:
        return None
    pose = _append_segment(path, pose, radius * theta, -1,
                           -side * steering, cfg, allowed, obstacles)
    if pose is None:
        return None
    if distance(pose, goal) > 1e-6 or abs(wrap(pose[2] - goal[2])) > 1e-6:
        return None
    return path


def _slot_is_feasible(goal, slot, cfg):
    try:
        return inside_slot(goal, slot, cfg)
    except (KeyError, TypeError, ValueError):
        return False


def _slot_body_inside(pose, slot, cfg):
    """Require both four-wheel containment and the full body in the bay."""
    if not _slot_is_feasible(pose, slot, cfg):
        return False
    try:
        margin = _cfg_nonnegative(cfg, 'parallel_parking_slot_body_margin_m', 0.0)
        half_length = slot['length'] / 2.0 - margin
        half_width = slot['width'] / 2.0 - margin
        if half_length <= 0 or half_width <= 0:
            return False
        return all(abs(local(slot['pose'], point)[0]) <= half_length and
                   abs(local(slot['pose'], point)[1]) <= half_width
                   for point in footprint(pose, cfg))
    except (KeyError, TypeError, ValueError):
        return False


def plan_parallel_parking(start, slot, cfg, allowed, obstacles=()):
    """Return ``(path, reason)`` for a measured P1/P2/P3 slot.

    Unknown free space and a slot too narrow for all four tyre centres are
    rejected before any search.  Hybrid-A* is used only after the bounded
    double-arc candidate has failed its body and obstacle checks.
    """
    if allowed is None:
        return [], 'parallel_parking_area_unknown'
    goal = _goal_from_cfg(start, slot, cfg)
    if not _slot_body_inside(goal, slot, cfg):
        return [], 'parallel_parking_slot_too_narrow'
    if not _area_allowed(allowed, tuple(start), cfg, obstacles):
        return [], 'parallel_parking_start_blocked'
    candidate = double_arc_candidate(start, slot, cfg, allowed, obstacles)
    if candidate:
        return candidate, 'parallel_parking_double_arc'
    path, reason = hybrid_plan(tuple(start), goal, cfg, obstacles,
                               allowed, final_direction=-1)
    if path:
        return path, 'parallel_parking_hybrid'
    return [], 'parallel_parking_'+reason


def _forward_search_cfg(cfg):
    """Return a local search view that cannot emit a reverse primitive."""
    search = dict(cfg)
    # hybrid_plan supports both gears internally.  A zero cusp budget plus
    # the initial-forward constraint makes that implementation a forward-only
    # search without changing the shared planner used by reverse parking.
    search['parking_max_cusps'] = 0
    search['planner_initial_forward'] = True
    return search


def _forward_goal(start, slot, cfg):
    """Choose a full-body-valid forward terminal pose in the bay."""
    goal = _goal_from_cfg(None, slot, cfg)
    if slot.get('kind', 'parallel') != 'parallel':
        return goal
    # Preserve the centered longitudinal target.  A small road-side bias is
    # useful for a forward side-bay entry, but is bounded by half the actual
    # body clearance so it cannot turn a marginal target into a success.
    margin = _cfg_nonnegative(cfg, 'parallel_parking_slot_body_margin_m', .01)
    clearance = slot['width'] / 2.0 - cfg['body_width'] / 2.0 - margin
    if clearance <= 0:
        return goal
    side = 1.0 if local(slot['pose'], start)[1] >= 0 else -1.0
    shift = min(.025, clearance * .5)
    goal_local = local(slot['pose'], goal)
    xy = world(slot['pose'], (goal_local[0], side * shift))
    return tuple(xy) + (goal[2],)


def _forward_goal_tolerance(goal, slot, cfg):
    """Keep the search/follower endpoint inside the bay's real body margin."""
    margin = _cfg_nonnegative(cfg, 'parallel_parking_slot_body_margin_m', .01)
    half_length = slot['length'] / 2.0 - margin
    half_width = slot['width'] / 2.0 - margin
    if half_length <= 0 or half_width <= 0:
        return .005
    clearance = []
    for point in footprint(goal, cfg):
        point = local(slot['pose'], point)
        clearance.extend((half_length - abs(point[0]),
                          half_width - abs(point[1])))
    positive = [item for item in clearance if item > 0]
    if not positive:
        return .005
    # Half the tightest remaining margin leaves room for odometry and yaw
    # quantisation before the measured confirmation gate makes DONE.
    return max(.003, min(_cfg_value(
        cfg, 'parallel_parking_goal_position_tolerance_m',
        cfg.get('path_goal_tolerance', .03)), min(positive) * .5))


def _forward_double_arc_candidate(start, slot, cfg, allowed, obstacles=()):
    """Generate a bounded, forward-only two-arc entry for a parallel bay."""
    if allowed is None:
        return None
    goal = _forward_goal(start, slot, cfg)
    relative = local(start, goal)
    yaw_error = abs(wrap(goal[2] - start[2]))
    yaw_limit = math.radians(_cfg_value(
        cfg, 'parallel_parking_double_arc_yaw_tolerance_deg', 3.0))
    if yaw_error > yaw_limit or abs(relative[1]) <= .01:
        return None
    wheelbase = number(cfg['wheelbase'])
    max_steer = number(cfg['max_steer'])
    if not 0 < wheelbase or not 0 < max_steer < math.pi / 2:
        raise ValueError('invalid vehicle steering geometry')
    kinematic_radius = wheelbase / math.tan(max_steer)
    # The reverse profile's comfortable radius can overshoot a short bay
    # when used as a forward entry (P1 is only 0.70 m long).  Start at the
    # legal kinematic radius; this keeps the candidate forward-feasible while
    # still respecting the configured steering limit.
    radius = kinematic_radius
    if abs(relative[1]) > 2.0 * radius:
        return None
    theta = math.acos(max(-1.0, min(1.0,
                                    1.0 - abs(relative[1]) / (2.0 * radius))))
    steering = math.atan(wheelbase / radius)
    side = 1.0 if relative[1] > 0 else -1.0
    forward_x = 2.0 * radius * math.sin(theta)
    setup_x = relative[0] - forward_x
    max_setup = _cfg_value(cfg, 'parallel_parking_max_setup_m', 2.0)
    # A forward entry cannot repair a goal that is already behind the axle.
    if setup_x < -_EPS or setup_x > max_setup:
        return None

    path = []
    pose = tuple(start)
    if setup_x > _EPS:
        pose = _append_segment(path, pose, setup_x, 1, 0.0,
                               cfg, allowed, obstacles)
    if pose is None:
        return None
    pose = _append_segment(path, pose, radius * theta, 1,
                           side * steering, cfg, allowed, obstacles)
    if pose is None:
        return None
    pose = _append_segment(path, pose, radius * theta, 1,
                           -side * steering, cfg, allowed, obstacles)
    if pose is None:
        return None
    if (distance(pose, goal) > 1e-6 or
            abs(wrap(pose[2] - goal[2])) > 1e-6):
        return None
    return path


def _forward_straight_candidate(start, goal, cfg, allowed, obstacles=()):
    """Accept an already aligned forward approach without lattice search."""
    yaw_limit = math.radians(_cfg_value(
        cfg, 'parallel_parking_goal_yaw_tolerance_deg', 10.0))
    if abs(wrap(goal[2] - start[2])) > yaw_limit:
        return None
    relative = local(start, goal)
    if abs(relative[1]) > .02 or relative[0] < -_EPS:
        return None
    path = []
    end = _append_segment(path, tuple(start), relative[0], 1, 0.0,
                          cfg, allowed, obstacles)
    if end is None or not path:
        return None
    if distance(end, goal) > 1e-6 or abs(wrap(end[2] - goal[2])) > 1e-6:
        return None
    return path


def _forward_turn_tail_candidate(start, goal, cfg, allowed, obstacles=()):
    """Solve a bounded straight/turn/straight forward entry.

    The same construction covers the perpendicular quarter-turn and a
    prepared parallel approach with a measured sub-quarter-turn heading.
    """
    delta = wrap(goal[2] - start[2])
    if abs(delta) < math.radians(12.0) or abs(delta) > math.pi - math.radians(12.0):
        return None
    wheelbase = number(cfg['wheelbase'])
    max_steer = number(cfg['max_steer'])
    radius = wheelbase / math.tan(max_steer)
    sign = 1.0 if delta > 0 else -1.0
    angle = abs(delta)
    steering = sign * math.atan(wheelbase / radius)
    # Integrate the arc once from the origin; its displacement is then
    # rotated into the measured start heading.
    arc = bicycle((0.0, 0.0, 0.0), radius * angle, steering, wheelbase)
    c, s = math.cos(start[2]), math.sin(start[2])
    arc_world = (c * arc[0] - s * arc[1],
                 s * arc[0] + c * arc[1])
    forward = (math.cos(start[2]), math.sin(start[2]))
    terminal = (math.cos(goal[2]), math.sin(goal[2]))
    residual = (goal[0] - start[0] - arc_world[0],
                goal[1] - start[1] - arc_world[1])
    determinant = forward[0] * terminal[1] - forward[1] * terminal[0]
    if abs(determinant) < .2:
        return None
    setup = ((residual[0] * terminal[1] - residual[1] * terminal[0]) /
             determinant)
    tail = ((forward[0] * residual[1] - forward[1] * residual[0]) /
            determinant)
    max_setup = _cfg_value(cfg, 'parallel_parking_max_setup_m', 2.0)
    if setup < -_EPS or tail < -_EPS or setup > max_setup or tail > max_setup:
        return None
    path = []
    pose = tuple(start)
    if setup > _EPS:
        pose = _append_segment(path, pose, setup, 1, 0.0,
                               cfg, allowed, obstacles)
    if pose is None:
        return None
    pose = _append_segment(path, pose, radius * angle, 1, steering,
                           cfg, allowed, obstacles)
    if pose is None:
        return None
    if tail > _EPS:
        pose = _append_segment(path, pose, tail, 1, 0.0,
                               cfg, allowed, obstacles)
    if pose is None or distance(pose, goal) > 1e-6:
        return None
    return path


def plan_forward_parking(start, slot, cfg, allowed, obstacles=()):
    """Plan a measured P1--P5 entry using forward gear only.

    ``allowed`` is supplied by the scene producer and must cover the road
    approach plus the exact bay rectangle.  No timed or reverse fallback is
    permitted when the measured corridor cannot support this path.
    """
    if allowed is None:
        return [], 'forward_parking_area_unknown'
    kind = slot.get('kind', 'parallel')
    if kind not in ('parallel', 'perpendicular'):
        return [], 'forward_parking_slot_kind_unknown'
    goal = _forward_goal(start, slot, cfg)
    if not _slot_body_inside(goal, slot, cfg):
        return [], 'forward_parking_slot_too_narrow'
    if not _area_allowed(allowed, tuple(start), cfg, obstacles):
        return [], 'forward_parking_start_blocked'
    # A forward-only path must end ahead of the current rear axle in the
    # slot frame.  This also rejects a passed bay instead of silently
    # switching to reverse.
    goal_relative = local(slot['pose'], goal)
    start_relative = local(slot['pose'], start)
    if goal_relative[0] < start_relative[0] - .02:
        return [], 'forward_parking_bay_passed'

    if kind == 'parallel':
        candidate = _forward_double_arc_candidate(
            start, slot, cfg, allowed, obstacles)
        if candidate:
            return candidate, 'forward_parking_double_arc'
    candidate = _forward_straight_candidate(
        start, goal, cfg, allowed, obstacles)
    if candidate:
        return candidate, 'forward_parking_straight'
    # A prepared parallel approach may already be part-way through its turn;
    # reuse the bounded analytic turn/tail instead of widening the corridor
    # or enabling a reverse fallback.
    if kind in ('parallel', 'perpendicular'):
        candidate = _forward_turn_tail_candidate(
            start, goal, cfg, allowed, obstacles)
        if candidate:
            return candidate, 'forward_parking_turn_tail'
    search_cfg = _forward_search_cfg(cfg)
    path, reason = hybrid_plan(tuple(start), goal, search_cfg, obstacles,
                               allowed, final_direction=1)
    if not path:
        return [], 'forward_parking_'+reason
    if any(row[3] != 1 for row in path):
        return [], 'forward_parking_reverse_segment'
    if not _slot_body_inside(path[-1], slot, cfg):
        return [], 'forward_parking_terminal_not_in_slot'
    return path, 'forward_parking_hybrid'


class ParallelParking(object):
    """Plan and follow one locked, measured parallel parking slot.

    ``command`` returns ``(speed_raw, physical_steering_rad)``.  Conversion
    to the ±22 actuator field belongs to the controller adapter, just as it
    does for the shared :class:`planning.Follower`.
    """
    PHASES = ('PLAN', 'TRACK', 'CONFIRM', 'DONE', 'BLOCKED')

    def __init__(self, cfg, scene, allow_perpendicular=False):
        self.cfg = dict(cfg)
        # Keep the class usable with a small unit-test configuration while
        # preserving the competition config's existing values when present.
        self.cfg.setdefault('parking_lookahead', .15)
        self.cfg.setdefault('path_goal_tolerance', .045)
        self.cfg.setdefault('path_yaw_tolerance', .13)
        self.cfg.setdefault('sensor_timeout', .50)
        self.cfg.setdefault('cusp_pause', .40)
        self.cfg['path_goal_tolerance'] = _cfg_value(
            self.cfg, 'parallel_parking_goal_position_tolerance_m',
            self.cfg['path_goal_tolerance'])
        speed_raw = dict(self.cfg.get('speed_raw', {}))
        parking_speed = self.cfg.get('parallel_parking_speed_raw',
                                     speed_raw.get('parking',
                                                   self.cfg.get('parking_entry_speed_raw', 12)))
        speed_raw.setdefault('lane', parking_speed)
        speed_raw.setdefault('action', parking_speed)
        speed_raw.setdefault('gap', parking_speed)
        # This mode has its own tunable speed even when the global parking
        # speed is present; do not silently inherit P4/P5's setting.
        speed_raw['parking'] = parking_speed
        self.cfg['speed_raw'] = speed_raw
        self._allow_perpendicular = bool(allow_perpendicular)
        self.required_gear = None
        self.require_measurement_gate = False
        self.scene = decode_parallel_scene(scene, allow_perpendicular=self._allow_perpendicular)
        self._compiled_regions = _compile_regions(self.scene['regions'])
        self._scene_revision = 0
        self._path_clear_revision = -1
        self.frame = self.scene['frame']
        self.slot_id = self.scene['slot']['id']
        self.slot = self.scene['slot']
        self.start = self.scene['pose']
        self.goal = _goal_from_cfg(self.start, self.slot, self.cfg)
        goal_yaw = math.radians(_cfg_value(
            self.cfg, 'parallel_parking_goal_yaw_tolerance_deg', 10.0))
        self.cfg['path_yaw_tolerance'] = goal_yaw
        self.follower = None
        self.phase = 'PLAN'
        self.reason = 'parallel_parking_plan_pending'
        self.started_at = self.scene['stamp']
        self.last_confirm_stamp = -1.0
        self.confirmations = 0

    def _allowed(self, point):
        return _compiled_region_contains(self._compiled_regions, point)

    def _clear(self, pose):
        return _area_allowed(self._allowed, pose, self.cfg,
                             self.scene['obstacles'])

    def _terminal_ok(self, pose):
        yaw_tol = math.radians(_cfg_value(
            self.cfg, 'parallel_parking_goal_yaw_tolerance_deg', 10.0))
        return (distance(pose, self.goal) <= self.cfg['path_goal_tolerance'] and
                _slot_body_inside(pose, self.slot, self.cfg) and
                abs(wrap(pose[2] - self.slot['pose'][2])) <= yaw_tol)

    def plan(self):
        if self.phase != 'PLAN':
            return [], 'parallel_parking_plan_not_pending'
        # Capture one immutable planning snapshot.  Observation callbacks may
        # replace self.scene while a caller runs the bounded search.
        scene = self.scene
        regions = tuple(tuple(region) for region in scene['regions'])
        compiled_regions = _compile_regions(regions)
        obstacles = tuple(scene['obstacles'])
        allowed = lambda point: _compiled_region_contains(compiled_regions, point)
        return plan_parallel_parking(self.start, self.slot, self.cfg,
                                     allowed, obstacles)

    def accept_plan(self, result):
        """Validate and accept a planner result, otherwise fail closed."""
        if (not isinstance(result, (list, tuple)) or len(result) != 2 or
                not isinstance(result[0], (list, tuple))):
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_invalid_plan'
            return
        path, reason = list(result[0]), result[1]
        self.reason = reason
        if not path:
            self.phase = 'BLOCKED'
            return
        for row in path:
            if (not isinstance(row, (list, tuple)) or len(row) != 5 or
                    row[3] not in (-1, 1) or
                    not all(_isfinite(float(v)) for v in row)):
                self.phase = 'BLOCKED'
                self.reason = 'parallel_parking_invalid_plan'
                return
            if (self.required_gear is not None and
                    row[3] != self.required_gear):
                self.phase = 'BLOCKED'
                self.reason = 'parallel_parking_reverse_segment'
                return
        if (distance(path[-1], self.goal) > self.cfg['path_goal_tolerance'] or
                abs(wrap(path[-1][2] - self.goal[2])) > self.cfg['path_yaw_tolerance'] or
                not self._terminal_ok(path[-1][:3])):
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_terminal_not_in_slot'
            return
        if not all(self._clear(row[:3]) for row in path):
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_path_blocked'
            return
        self.follower = Follower(path, self.cfg, parking=True)
        self.started_at = self.scene['stamp']
        self._path_clear_revision = -1
        self.phase = 'TRACK'
        self.reason = 'parallel_parking_tracking'

    def observe(self, scene):
        """Accept a newer real observation without moving the locked target."""
        observed = decode_parallel_scene(
            scene, allow_perpendicular=self._allow_perpendicular)
        if observed['frame'] != self.frame or observed['slot']['id'] != self.slot_id:
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_reference_changed'
            return
        if (abs(observed['slot']['length'] - self.slot['length']) > .02 or
                abs(observed['slot']['width'] - self.slot['width']) > .02):
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_slot_dimensions_changed'
            return
        drift = _cfg_value(self.cfg, 'parallel_parking_slot_drift_m', .12)
        if (distance(observed['slot']['pose'], self.slot['pose']) > drift or
                abs(wrap(observed['slot']['pose'][2] - self.slot['pose'][2])) >
                math.radians(12)):
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_slot_reference_changed'
            return
        if observed['stamp'] <= self.scene['stamp']:
            return
        # The original slot geometry remains locked; only fresh measured pose,
        # regions, and obstacles are allowed to update the execution snapshot.
        self.scene = dict(observed, slot=self.slot)
        self._compiled_regions = _compile_regions(self.scene['regions'])
        self._scene_revision += 1
        self._path_clear_revision = -1

    def command(self, now):
        if self.phase not in ('TRACK', 'CONFIRM'):
            return 0, 0.0
        if (self.require_measurement_gate and
                (self.scene.get('ready') is not True or
                 self.scene.get('occupancy') != 'FREE' or
                 (getattr(self, 'require_front_observation', False) and
                  self.scene.get('observation_source') != 'front'))):
            self.confirmations = 0
            self.reason = 'parallel_parking_scene_unconfirmed'
            return 0, 0.0
        if (self.scene.get('ready') is False or
                self.scene.get('occupancy') in ('UNKNOWN', 'OCCUPIED')):
            self.confirmations = 0
            self.reason = 'parallel_parking_scene_unconfirmed'
            return 0, 0.0
        if not 0 <= now - self.scene['stamp'] <= self.cfg['sensor_timeout']:
            self.confirmations = 0
            self.reason = 'parallel_parking_pose_stale'
            return 0, 0.0
        if now - self.started_at > _cfg_value(
                self.cfg, 'parallel_parking_timeout_s', 60.0):
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_timeout'
            return 0, 0.0
        pose = self.scene['pose']
        if not self._clear(pose):
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_observed_pose_blocked'
            return 0, 0.0
        if self.phase == 'CONFIRM':
            good = self._terminal_ok(pose)
            if self.scene['stamp'] > self.last_confirm_stamp:
                self.last_confirm_stamp = self.scene['stamp']
                self.confirmations = self.confirmations + 1 if good else 0
            if self.confirmations >= int(_cfg_value(
                    self.cfg, 'parallel_parking_confirm_frames',
                    self.cfg.get('exit_frames', 3))):
                self.phase = 'DONE'
                self.reason = 'parallel_parking_complete'
            return 0, 0.0
        if self._path_clear_revision != self._scene_revision:
            if not all(self._clear(row[:3]) for row in self.follower.path[self.follower.index:]):
                self.phase = 'BLOCKED'
                self.reason = 'parallel_parking_remaining_path_blocked'
                return 0, 0.0
            self._path_clear_revision = self._scene_revision
        remaining = self.follower.path[self.follower.index:]
        if not remaining:
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_invalid_plan'
            return 0, 0.0
        nearest = min(remaining, key=lambda row: distance(pose, row))
        if distance(pose, nearest) > _cfg_value(
                self.cfg, 'parallel_parking_tracking_error_m', .15):
            self.phase = 'BLOCKED'
            self.reason = 'parallel_parking_tracking_error'
            return 0, 0.0
        speed, steering = self.follower.command(pose, now)
        if self.follower.done:
            self.phase = 'CONFIRM'
            self.reason = 'parallel_parking_confirming'
            return 0, 0.0
        self.reason = 'parallel_parking_tracking'
        return speed, steering


class ForwardParking(ParallelParking):
    """Forward-only measured parking task for parallel or perpendicular bays."""

    def __init__(self, cfg, scene):
        ParallelParking.__init__(self, cfg, scene, allow_perpendicular=True)
        self.required_gear = 1
        self.require_measurement_gate = True
        self.require_front_observation = True
        self.goal = _forward_goal(self.start, self.slot, self.cfg)
        self.cfg['path_goal_tolerance'] = _forward_goal_tolerance(
            self.goal, self.slot, self.cfg)
        self.cfg['parking_goal_tolerance'] = self.cfg['path_goal_tolerance']

    def plan(self):
        if self.phase != 'PLAN':
            return [], 'forward_parking_plan_not_pending'
        scene = self.scene
        regions = tuple(tuple(region) for region in scene['regions'])
        compiled_regions = _compile_regions(regions)
        obstacles = tuple(scene['obstacles'])
        allowed = lambda point: _compiled_region_contains(compiled_regions, point)
        return plan_forward_parking(self.start, self.slot, self.cfg,
                                    allowed, obstacles)

    def observe(self, scene):
        # Decode first so a producer cannot switch a locked perpendicular bay
        # to a parallel row (or vice versa) while retaining the same id.
        observed = decode_parallel_scene(scene, allow_perpendicular=True)
        if observed['slot'].get('kind') != self.slot.get('kind'):
            self.phase = 'BLOCKED'
            self.reason = 'forward_parking_slot_kind_changed'
            return
        ParallelParking.observe(self, observed)
