"""parking_planner: independently testable geometry; no mission state or ROS."""
from __future__ import division
import math
import time
from robot.common.geometry import bicycle
from robot.common.geometry import collision
from robot.common.geometry import distance
from robot.common.geometry import footprint
from robot.common.geometry import local
from robot.common.geometry import slot_samples
from robot.common.geometry import wrap
from robot.common.geometry import world

try:
    _string_types = (basestring,)
except NameError:
    _string_types = (str,)

try:
    _monotonic = time.monotonic
except AttributeError:
    _monotonic = time.time


def rank_parking_slots(start, slots, scan, cfg):
    """Classify whole observed bays, then rank by required straight approach."""
    offset = cfg.get('parking_prep_offset', 0.0)
    if offset == 0:
        offset = cfg['wheelbase']/math.tan(cfg['max_steer']) + cfg['wheelbase']/2
    rows = []
    for slot in slots:
        row = dict(slot)
        row['approach_distance'] = max(0.0, local(start, slot['pose'])[0]-offset)
        row['coverage'] = scan.coverage(slot_samples(slot))
        occupied = any(abs(local(slot['pose'],p)[0]) <= slot['length']/2 + cfg['obstacle_margin']
                       and abs(local(slot['pose'],p)[1]) <= slot['width']/2 + cfg['obstacle_margin']
                       for p in scan.obstacles)
        row['occupancy'] = ('OCCUPIED' if occupied else
                            'FREE' if row['coverage'] >= cfg['slot_min_coverage'] else 'UNKNOWN')
        rows.append(row)
    return sorted(rows, key=lambda s: (s['approach_distance'], distance(start,s['pose'])))


def parking_area(start, slot, cfg):
    """Road union one bay, with the mouth inferred from the observed bay side."""
    rel = local(start,slot['pose'])
    margin = cfg['parking_search_x_margin']
    xlo, xhi = min(0,rel[0])-margin, max(0,rel[0])+margin
    corners = [local(start,world(slot['pose'],(x,y)))
               for x in (-slot['length']/2,slot['length']/2)
               for y in (-slot['width']/2,slot['width']/2)]
    right = rel[1] < 0
    mouth = max(p[1] for p in corners) if right else min(p[1] for p in corners)
    start_c,start_s = math.cos(start[2]),math.sin(start[2])
    slot_pose = slot['pose']
    slot_c,slot_s = math.cos(slot_pose[2]),math.sin(slot_pose[2])
    half_length,half_width = slot['length']/2,slot['width']/2
    road_width = cfg['parking_road_half_width']
    def allowed(q):
        dx,dy = q[0]-start[0],q[1]-start[1]
        x,y = start_c*dx+start_s*dy,-start_s*dx+start_c*dy
        road = (xlo <= x <= xhi and
                (mouth <= y <= road_width if right else -road_width <= y <= mouth))
        if road:
            return True
        dx,dy = q[0]-slot_pose[0],q[1]-slot_pose[1]
        sx,sy = slot_c*dx+slot_s*dy,-slot_s*dx+slot_c*dy
        return abs(sx) <= half_length and abs(sy) <= half_width
    return allowed


def _auto_reverse_slot_allowed(slot, cfg):
    """Keep the reverse perpendicular route on the P4/P5 bay family.

    Some camera/replay inputs carry a semantic id while older frames only
    carry ``kind``.  Reject an explicit P1/P2/P3 perpendicular mismatch, but
    retain generic unlabelled candidates for backwards-compatible diagnostics
    and tests; the designated-target producer is responsible for assigning
    the concrete P4/P5 identity before motion.
    """
    kind = slot.get('kind')
    mode = cfg.get('parking_mode')
    if mode == 'parallel_reverse':
        return kind == 'parallel'
    if mode == 'reverse_plan' and kind != 'perpendicular':
        # reverse_plan is the perpendicular P4/P5 route.  Parallel bays use
        # the separate parallel_reverse executor and must never be selected
        # by AUTO reverse dispatch.
        return False
    if kind != 'perpendicular':
        return True
    raw_id = slot.get('id', slot.get('slot_id'))
    slot_id = raw_id.strip().upper() if isinstance(raw_id, _string_types) else None
    if slot_id in ('P1', 'P2', 'P3'):
        return False
    allowed = cfg.get('reverse_perpendicular_slot_ids', ('P4', 'P5'))
    try:
        allowed = tuple(str(value).strip().upper() for value in allowed)
    except TypeError:
        allowed = ('P4', 'P5')
    return (slot_id is None or slot_id.startswith('AUTO_') or
            slot_id in allowed)


def auto_parking_plan(start, slots, scan, cfg):
    """Try empty bays near-first using the existing bounded planner/follower.

    A far bay adds a straight forward prefix; a near bay does not. Final
    approach is always reverse. All snapshots are immutable while searching.
    """
    from robot.common.planning import hybrid_plan
    rows = rank_parking_slots(start,slots,scan,cfg)
    deadline = _monotonic()+2*cfg['planner_timeout']
    for slot in rows:
        if slot['occupancy'] != 'FREE':
            continue
        if not _auto_reverse_slot_allowed(slot, cfg):
            slot['plan'] = 'unsupported_reverse_slot'
            continue
        allowed = parking_area(start,slot,cfg)
        advance = slot['approach_distance']
        count = max(1,int(math.ceil(advance/.025)))
        prefix = [tuple(bicycle(start,advance*i/count,0,cfg['wheelbase']))+(1,0.0)
                  for i in range(count+1)] if advance > .001 else []
        if any(collision(p,scan.obstacles,cfg) or not all(allowed(q) for q in footprint(p,cfg))
               for p in prefix):
            slot['plan'] = 'approach_blocked'
            continue
        prep = prefix[-1][:3] if prefix else start
        goal = tuple(world(slot['pose'],(-cfg['wheelbase']/2,0)))+(slot['pose'][2],)
        remaining = deadline-_monotonic()
        if remaining <= 0:
            break
        search_cfg = dict(cfg,planner_timeout=min(cfg['planner_timeout'],remaining))
        path, reason = hybrid_plan(prep,goal,search_cfg,list(scan.obstacles),allowed,-1)
        slot['plan'] = reason
        if path:
            # Retain the coincident gear-change endpoint for Follower's cusp pause.
            return prefix+path, 'planned', dict(slot), rows
    return [], 'no_empty_reachable_bay', None, rows


def _reverse_parking_candidate(start, goal, cfg, obstacles, allowed, deadline=None):
    """Exact forward setup + reverse arc + reverse tail for a side-facing bay.

    This is a geometry-derived candidate, not a timed maneuver or a fixed map.
    Every sampled body pose must satisfy the same constraints as Hybrid A*.
    """
    angle = wrap(goal[2]-start[2])
    if not math.radians(70) <= abs(angle) <= math.radians(110):
        return None
    radius = cfg['wheelbase']/math.tan(cfg['max_steer'])
    steer = -math.copysign(cfg['max_steer'], angle)
    arc = bicycle((0,0,0), -radius*abs(angle), steer, cfg['wheelbase'])
    gx,gy = local(start,goal)
    tail = (arc[1]-gy)/math.sin(angle)
    entry = gx-arc[0]+tail*math.cos(angle)
    if tail < 0 or entry < 0 or (entry > 1e-9 and cfg['parking_max_cusps'] < 1):
        return None
    path = [tuple(start)+(1,0.0)]
    pose = tuple(start)
    for length,direction,steering in ((entry,1,0.0),(radius*abs(angle),-1,steer),(tail,-1,0.0)):
        if length <= 1e-9:
            continue
        if path[-1][3] != direction:
            path.append(pose+(direction,steering))
        count = max(1,int(math.ceil(length/.025)))
        origin = pose
        for i in range(1,count+1):
            if deadline is not None and _monotonic() >= deadline:
                return None
            pose = tuple(bicycle(origin,direction*length*i/count,steering,cfg['wheelbase']))
            if collision(pose,obstacles,cfg) or (allowed is not None and
                    not all(allowed(p) for p in footprint(pose,cfg))):
                return None
            path.append(pose+(direction,steering))
    if (collision(start,obstacles,cfg) or (allowed is not None and
            not all(allowed(p) for p in footprint(start,cfg)))):
        return None
    return path if distance(pose,goal) < 1e-6 else None
