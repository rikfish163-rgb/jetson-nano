"""lane module: explicit context input; coordinator applies returned updates."""
from __future__ import division

import math
from robot.common.geometry import distance, _isfinite, bicycle, footprint
from robot.common.geometry import local, world
from robot.common.contracts import model_to_command_steering, encode_command
from robot.motion import calibration as chassis
from vehicle_control.pure_pursuit import PurePursuit
from robot.lane.preview import preview_steering, lane_speed



FIELDS = ('action', 'action_started', 'cfg', 'follow_left_boundary', 'front_marker_stamp', 'gap_origin', 'gap_start',
 'gap_steer', 'lane', 'lane_boundaries', 'lane_confidence', 'lane_source', 'lane_stamp', 'last_completed_action',
 'left_boundary', 'left_boundary_stamp', 'left_fit_diagnostic', 'left_reference',
 'park_line_side', 'pending', 'pose', 'right_lock', 'right_tail_origin', 'state', 'lane_recovery',
 'startup_curve_lock', 'lane_target', 'lane_curve_lock', 'lane_preview', 'issued_steer', 'lane_speed_state', 'uturn_exit_straight')
CALLS = (('lane', 'lane_valid'), ('lane', 'left_exit_line'), ('lane', 'left_reference_command'),
 ('lane', 'park_line_command'), ('motion', 'current_steering'), ('motion', 'stop'))
OPERATIONS = ('lane_valid', 'lane_tracking_ready', 'forward_lane_ready', 'lane_command', 'park_line_command', 'left_reference_command',
 'left_exit_line')


def lane_valid(ctx, now):
    if not (0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout'] and
            ctx.lane_confidence >= ctx.cfg['lane_min_confidence']):
        return False
    points = sorted(local(ctx.pose,p) for p in ctx.lane)
    points = [p for p in points if p[0] > .05]
    return len(points) >= 2 and points[-1][0]-points[0][0] >= ctx.cfg.get('lane_min_path_span',.15)-1e-9


def lane_tracking_ready(ctx, now, points=None):
    """Allow coherent current sparse paint at bend speed, never stale guesses."""
    if lane_valid(ctx, now):
        return True
    if not 0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout']:
        return False
    if points is None:
        points = sorted(local(ctx.pose,p) for p in ctx.lane)
    # Slanted normal offsets shorten forward span; also check the actual
    # measured segment length instead of applying the full-path 15 cm gate.
    if (len(points) < 2 or not all(_isfinite(v) for p in points for v in p) or
            not all(.05 < x <= 1.5 and abs(y) <= .8 for x,y in points) or
            points[0][0] > 1.2 or points[-1][0]-points[0][0] < .06-1e-9 or
            distance(points[0],points[-1]) < .08-1e-9):
        return False
    # Coverage lowers the published confidence even when the remaining
    # measured centers are consistent. Three points also support a bend check.
    floor = .20 if len(points) >= 3 else .7*ctx.cfg['lane_min_confidence']
    if ctx.lane_confidence < floor:
        return False
    angles = []
    for a,b in zip(points,points[1:]):
        dx,dy = b[0]-a[0],b[1]-a[1]
        if not .015 <= dx <= .35:
            return False
        angles.append(math.atan2(dy,dx))
    return (all(abs(a) <= math.radians(65) for a in angles) and
            all(abs(b-a) <= math.radians(35) for a,b in zip(angles,angles[1:])))


def forward_lane_ready(ctx, now):
    """Only hand straight motion to a fresh, near, forward-facing path."""
    if not ctx.call('lane', 'lane_valid', now):
        return False
    points = sorted(local(ctx.pose,p) for p in ctx.lane)
    points = [p for p in points if .05 < p[0] <= 1.10]
    if len(points) < 2 or points[-1][0]-points[0][0] < ctx.cfg.get('lane_min_path_span',.15):
        return False
    if any(abs(y) > ctx.cfg.get('exit_lateral_tolerance',.18) for x,y in points):
        return False
    limit = math.radians(ctx.cfg.get('straight_search_heading_deg',20))
    return all(abs(math.atan2(b[1]-a[1],b[0]-a[0])) <= limit
               for a,b in zip(points,points[1:]))


def lane_gap_command(ctx, now):
    """Bound empty or rejected paths by the last accepted observation."""
    if ctx.gap_origin is None:
        return ctx.call('motion', 'stop', 'no_initial_lane')
    if ctx.state != 'GAP':
        ctx.gap_steer = ctx.call('motion', 'current_steering', now)
    budget = min(.25,ctx.cfg['gap_max_seconds'])
    if (distance(ctx.pose,ctx.gap_origin) > ctx.cfg['gap_max_distance'] or
            now-ctx.gap_start > budget):
        return ctx.call('motion', 'stop', 'gap_limit_wait_for_lane')
    ctx.state = 'GAP'
    return lane_speed(ctx, ctx.cfg['speed_raw']['gap']), ctx.gap_steer


def _uturn_exit_command(ctx, now):
    """Center between current paint, with straight fallback when paint is absent."""
    ctx.follow_left_boundary = False
    ctx.left_reference = None
    ctx.lane_speed_state = None
    ctx.state = 'LANE'
    if not 0 <= now-ctx.front_marker_stamp <= ctx.cfg.get('ground_timeout', 1.25):
        return ctx.call('motion', 'stop', 'uturn_exit_front_stale')
    edges = {}
    if 0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout']:
        for side in ('LEFT', 'RIGHT'):
            points = sorted(local(ctx.pose, p) for p in ctx.lane_boundaries.get(side, []))
            points = [p for p in points if all(_isfinite(v) for v in p) and .05 < p[0] <= 1.2]
            if len(points) >= 2 and points[-1][0]-points[0][0] >= .10:
                edges[side] = points
    if not edges:
        ctx.lane_preview = None
        ctx.lane_source = 'uturn_exit_straight'
        return ctx.cfg['straight_speed_raw'], 0.

    def interpolate(points, x):
        for a, b in zip(points, points[1:]):
            if a[0] <= x <= b[0] and b[0] > a[0]:
                return a[1]+(b[1]-a[1])*(x-a[0])/(b[0]-a[0])
        return points[-1][1]

    lines = {}
    offset = ctx.cfg['lane_width']/2
    for side, points in edges.items():
        near = points[:3]
        mx = sum(p[0] for p in near)/len(near)
        my = sum(p[1] for p in near)/len(near)
        variance = sum((p[0]-mx)**2 for p in near)
        if variance <= 1e-9:
            return ctx.call('motion', 'stop', 'uturn_exit_lane_geometry_invalid')
        slope = sum((p[0]-mx)*(p[1]-my) for p in near)/variance
        intercept = my-slope*mx
        if max(abs(y-slope*x-intercept) for x, y in near) > .035:
            return ctx.call('motion', 'stop', 'uturn_exit_lane_geometry_invalid')
        normal_distance = intercept/math.hypot(1., slope)
        lines[side] = dict(heading=math.atan(slope), lateral=normal_distance-offset,
                          offset_m=offset, boundary_distance_m=normal_distance,
                          side=side, observed_points=points)
    if len(edges) == 2:
        left, right = edges['LEFT'], edges['RIGHT']
        start = max(left[0][0], right[0][0])
        end = min(left[-1][0], right[-1][0])
        xs = sorted(set(x for points in edges.values() for x, y in points if start <= x <= end))
        if len(xs) < 2 or end-start < .10:
            return ctx.call('motion', 'stop', 'uturn_exit_lane_geometry_invalid')
        pair = [(x, interpolate(left, x), interpolate(right, x)) for x in xs]
        minimum = ctx.cfg['body_width']+2*ctx.cfg.get('obstacle_margin', .035)
        if any(not minimum < ly-ry <= 2*ctx.cfg['lane_width'] for x, ly, ry in pair):
            return ctx.call('motion', 'stop', 'uturn_exit_lane_geometry_invalid')
        centers = [(x, (ly+ry)/2) for x, ly, ry in pair]
    else:
        side = next(iter(edges))
        heading = lines[side]['heading']
        inward = -1 if side == 'LEFT' else 1
        centers = [(x-inward*offset*math.sin(heading), y+inward*offset*math.cos(heading))
                   for x, y in edges[side]]
    ctx.lane_source = 'uturn_exit_center'
    steer = preview_steering(ctx, centers, 0.)
    if ctx.lane_preview is None:
        return ctx.call('motion', 'stop', 'uturn_exit_lane_geometry_invalid')
    aligned = (abs(centers[0][1]) <= .08 and
               all(abs(math.atan2(b[1]-a[1], b[0]-a[0])) <= math.radians(8)
                   for a, b in zip(centers, centers[1:])))
    speed = ctx.cfg['straight_speed_raw']
    if not aligned:
        speed = min(speed, ctx.cfg.get('lane_curve_speed_raw', 12))
    ctx.lane_target = dict(stamp=now, source_stamp=ctx.lane_stamp, path=centers,
                          valid=True, selection='uturn_exit_boundary_center', angle=steer)
    for side in ('LEFT', 'RIGHT'):
        if side in lines and not _left_boundary_sweep_clear(ctx, lines[side], speed, steer):
            ctx.lane_target.update(valid=False, guard_side=side)
            return ctx.call('motion', 'stop', 'uturn_exit_boundary_crossing')
    ctx.gap_origin, ctx.gap_start, ctx.gap_steer = ctx.pose, now, steer
    return speed, steer


def lane_command(ctx, now, approach=False):
    """Follow measured centers, including coherent sparse paint at bend speed."""
    ctx.lane_recovery = None
    ctx.lane_curve_lock = None
    if ctx.uturn_exit_straight and ctx.action is None:
        return _uturn_exit_command(ctx, now)
    if not ctx.call('lane','lane_valid',now):
        ctx.lane_speed_state = None
    if ctx.follow_left_boundary and ctx.left_reference is not None:
        line = ctx.call('lane', 'left_exit_line', now)
        if line is None:
            ctx.state = 'GAP'
            ctx.lane_source = 'left_boundary'
            return ctx.call('motion', 'stop', 'left_boundary_follow_missing')
        else:
            command = ctx.call('lane', 'left_reference_command', line, now)
            ctx.follow_left_boundary = ctx.left_reference is not None
            if command[0]:
                ctx.state = 'LANE'
            ctx.lane_source = 'left_boundary'
            return command
    # Pending signs reserve a later blue trigger; road driving still follows
    # lane centers until dispatch actually enters a parking action.
    ctx.follow_left_boundary = False
    ctx.lane_source = 'center'
    partial = False
    if not 0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout']:
        ctx.lane_preview = None
        return ctx.call('motion', 'stop', 'lane_stream_stale')
    if not ctx.lane:
        return lane_gap_command(ctx, now)
    if not ctx.call('lane', 'lane_valid', now):
        partial = lane_tracking_ready(ctx,now)
        if partial:
            ctx.lane_source = 'partial_center'
    if not lane_tracking_ready(ctx,now):
        points = sorted(local(ctx.pose,p) for p in ctx.lane)
        points = [p for p in points if p[0] > .05]
        # One bad fresh image must not pulse the motor to zero. Hold only the
        # last accepted angle for 250 ms; rejected frames never renew this clock.
        if ctx.gap_origin is not None and 0 <= now-ctx.gap_start <= .25:
            grace = 'visual_frame_grace'
            ctx.lane_target = dict(stamp=now,source_stamp=ctx.lane_stamp,path=points,
                target=None,valid=False,reason=grace,selection=grace)
            speed,steer = lane_gap_command(ctx,now)
            return min(speed,ctx.cfg.get('lane_curve_speed_raw',12)),steer
        ctx.lane_preview = None
        ctx.lane_target = dict(stamp=now,source_stamp=ctx.lane_stamp,path=[],
            target=None,valid=False,reason='lane_unreliable',selection='lane_unreliable')
        return ctx.call('motion', 'stop', 'lane_unreliable')
    candidates = [local(ctx.pose,p) for p in ctx.lane]
    forward = [(i,p) for i,p in enumerate(candidates) if p[0] > 0.]
    if not forward:
        return ctx.call('motion', 'stop', 'lane_behind_vehicle')
    steer = preview_steering(ctx, candidates, 0.)
    if ctx.lane_preview is None:
        return ctx.call('motion','stop','lane_geometry_invalid')
    target = ctx.lane_preview['tracking_target']
    target_index = min(forward,key=lambda item:distance(item[1],target))[0]
    desired_steer = steer
    if ctx.issued_steer is not None:
        dt = max(0.,min(.10,now-ctx.gap_start)) if ctx.gap_origin is not None else .05
        before = ctx.call('motion','current_steering',now)
        limit = 90.*dt/ctx.cfg['steering_raw_limit']*ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
        steer = max(before-limit,min(before+limit,steer))
    ctx.lane_target = dict(stamp=now, source_stamp=ctx.lane_stamp,
        path=candidates, target=target, target_index=target_index,
        intersection=None, line_x=None, lateral_offset_m=target[1],
        angle=steer, desired_angle=desired_steer, valid=True, reason='ok',
        curvature_preview=ctx.lane_preview,selection='center_pure_pursuit')
    ctx.state = 'LANE'
    ctx.gap_origin, ctx.gap_start, ctx.gap_steer = ctx.pose, now, steer
    if partial:
        ctx.lane_speed_state = None
        return min(ctx.cfg['speed_raw']['lane'],ctx.cfg.get('lane_curve_speed_raw',12)),steer
    return lane_speed(ctx, ctx.cfg['speed_raw']['lane']), steer


def park_line_command(ctx, now):
    """Follow the observed paint itself while awaiting the parking marker."""
    ctx.lane_curve_lock = None
    ctx.lane_preview = None
    if (not 0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout'] or
            (ctx.cfg.get('parking_mode')!='forward_center' and
             ctx.lane_confidence < ctx.cfg['lane_min_confidence'])):
        return ctx.call('motion', 'stop', 'park_line_stale_or_low_confidence')
    candidates = {}
    for side, rows in getattr(ctx,'lane_boundaries',{}).items():
        points = sorted(local(ctx.pose,p) for p in rows)
        points = [p for p in points if p[0] > .05]
        if len(points) >= 3 and points[-1][0]-points[0][0] >= ctx.cfg.get('lane_min_path_span',.15)-1e-9:
            candidates[side] = points
    side = getattr(ctx,'park_line_side',None)
    if candidates and (side is None or
            (ctx.cfg.get('parking_mode') == 'forward_center' and side not in candidates)):
        side = min(candidates,key=lambda s:abs(candidates[s][0][1]))
        ctx.park_line_side = side
        ctx.left_reference = None
    ctx.lane_source = 'park_line_' + (side.lower() if side else 'missing')
    if side not in candidates:
        ctx.state = 'GAP'
        return ctx.call('motion', 'stop', 'park_line_missing')
    points = candidates[side]
    heading = math.atan2(points[-1][1]-points[0][1],points[-1][0]-points[0][0])
    ctx.state = 'LANE'
    return ctx.call('lane', 'left_reference_command', dict(points=points,heading=heading,
        lateral=points[0][1],offset_m=0.),now)


def _observed_boundary_clearance(ctx, points, side, pose):
    """Check measured paint segments against the body within their support."""
    cfg = ctx.cfg
    back, front = -cfg['rear_overhang'], cfg['wheelbase']+cfg['front_overhang']
    half = cfg['body_width']/2
    polygon = [local(ctx.pose, world(pose, p)) for p in
               ((back, -half), (front, -half), (front, half), (back, half))]
    margin = cfg.get('obstacle_margin', .035)
    inside = -1 if side == 'RIGHT' else 1
    clearance = float('inf')
    for a, b in zip(points, points[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        if dx <= 0:
            continue
        # Clip the body polygon, including edge/strip intersections. Testing
        # corners alone misses tape passing between two body corners.
        clipped = polygon
        for bound, direction in ((a[0]-margin, 1), (b[0]+margin, -1)):
            if not clipped:
                break
            result = []
            previous = clipped[-1]
            previous_inside = direction*(previous[0]-bound) >= 0
            for point in clipped:
                point_inside = direction*(point[0]-bound) >= 0
                if point_inside != previous_inside:
                    fraction = (bound-previous[0])/(point[0]-previous[0])
                    result.append((bound, previous[1]+fraction*(point[1]-previous[1])))
                if point_inside:
                    result.append(point)
                previous, previous_inside = point, point_inside
            clipped = result
        length = math.hypot(dx, dy)
        for x, y in clipped:
            signed = (dx*(a[1]-y)+dy*(x-a[0]))/length
            clearance = min(clearance, inside*signed-margin)
    return clearance


def _left_boundary_sweep_clear(ctx, line, speed, steer):
    """Guard measured exit paint; retain the legacy fitted left-line guard."""
    offset = line.get('offset_m', ctx.cfg['lane_width'] / 2)
    # Parking follows a raw bay line (offset_m=0), not a road boundary.  It
    # must retain its existing behaviour and is intentionally not guarded.
    if offset <= 0:
        return True
    heading = line.get('heading')
    lateral = line.get('lateral')
    if (heading is None or lateral is None or not _isfinite(heading) or
            not _isfinite(lateral)):
        diagnostic = ctx.left_fit_diagnostic or {}
        diagnostic.update(guard_reason='invalid_left_boundary_geometry',
                          guard_clearance_m=None, sweep_clearance_m=None,
                          guard_horizon_s=.25)
        ctx.left_fit_diagnostic = diagnostic
        return False

    cfg = ctx.cfg
    # The fitted center line is lane_width/2 to the right of the tape.  Use
    # its normal directly so an angled tape is checked without a slope limit.
    boundary_distance = line.get('boundary_distance_m', lateral + cfg['lane_width'] / 2)
    side = line.get('side', 'LEFT')
    inside = -1 if side == 'RIGHT' else 1
    normal = (-math.sin(heading), math.cos(heading))
    margin = cfg.get('obstacle_margin', .035)
    raw = encode_command(speed, steer, cfg, 0)['steering_raw']
    physical = chassis.raw_angle(cfg, speed, raw)
    metres_per_second = abs(speed) * abs(chassis.speed_gain(cfg, speed))
    horizon = .25
    travel = metres_per_second * horizon
    direction = 1 if speed >= 0 else -1
    poses = [ctx.pose]
    for i in range(1, 17):
        poses.append(bicycle(ctx.pose, direction * travel * i / 16,
                             physical, cfg['wheelbase']))

    current_clearance = float('inf')
    sweep_clearance = float('inf')
    observed = line.get('observed_points')
    for index, pose in enumerate(poses):
        if observed:
            clearance = _observed_boundary_clearance(ctx, observed, side, pose)
            if index == 0:
                current_clearance = clearance
            sweep_clearance = min(sweep_clearance, clearance)
            continue
        for point in footprint(pose, cfg):
            x, y = local(ctx.pose, point)
            # Positive clearance keeps the vehicle on the tape's inward side;
            # margin matches the obstacle footprint guard.
            clearance = inside*(boundary_distance - (normal[0]*x+normal[1]*y)) - margin
            if index == 0:
                current_clearance = min(current_clearance, clearance)
            sweep_clearance = min(sweep_clearance, clearance)

    diagnostic = ctx.left_fit_diagnostic or {}
    diagnostic.update(boundary_normal_distance_m=boundary_distance,
                      guard_side=side,
                      guard_basis='observed_segments' if observed else 'fitted_line',
                      observed_span_m=[observed[0][0], observed[-1][0]] if observed else None,
                      footprint_clearance_m=current_clearance if _isfinite(current_clearance) else None,
                      sweep_clearance_m=sweep_clearance if _isfinite(sweep_clearance) else None,
                      guard_clearance_m=sweep_clearance if _isfinite(sweep_clearance) else None,
                      guard_horizon_s=horizon,
                      guard_speed_mps=metres_per_second,
                      guard_travel_m=travel,
                      guard_margin_m=margin,
                      guard_reason='clear')
    ctx.left_fit_diagnostic = diagnostic
    if sweep_clearance <= 0:
        diagnostic['guard_reason'] = 'footprint_crosses_'+side.lower()+'_boundary'
        return False
    return True


def left_reference_command(ctx, line, now):
    """Track the 30 cm inward offset with consistent steering units and slew."""
    cfg = ctx.cfg
    points = [p for p in line['points'] if p[0] > .05]
    if not points:
        ctx.left_reference = None
        return ctx.call('motion', 'stop', 'left_reference_behind')
    aligning = ctx.right_lock is not None and ctx.right_lock['phase'] in ('ALIGN_LEFT', 'ALIGN_LANE')
    lookahead = cfg.get('right_align_lookahead',.30) if aligning else cfg['lookahead']
    result = PurePursuit(cfg['wheelbase'],lookahead).compute(points)
    if not result.valid:
        return ctx.call('motion', 'stop', 'left_reference_invalid')
    physical = result.steering_angle
    scale = cfg.get('steering_command_scale_rad',cfg['max_steer'])
    target = model_to_command_steering(physical,cfg)
    previous = ctx.left_reference
    dt = max(0,min(.2,now-previous['stamp'])) if previous else .05
    before = ctx.call('motion', 'current_steering', now)
    limit = cfg.get('left_reference_raw_rate',40.0)*dt/cfg['steering_raw_limit']*scale
    steer = max(before-limit,min(before+limit,target))
    aligned = (abs(line['heading']) <= math.radians(cfg.get('left_reference_heading_deg',8)) and
               abs(line['lateral']) <= cfg.get('left_reference_lateral_m',.08))
    # Road and RIGHT-exit references use the configured road speeds. Sweep
    # that actual command, including while only one measured side is visible.
    if ctx.action == 'UTURN':
        speed = cfg.get('left_reference_speed_raw',12)
    else:
        speed = cfg['speed_raw']['lane']
        if not aligned:
            speed = min(speed,cfg.get('lane_curve_speed_raw',12))
    if not _left_boundary_sweep_clear(ctx, line, speed=speed, steer=steer):
        offset = line.get('offset_m', cfg['lane_width'] / 2)
        ctx.left_reference = dict(stamp=now, heading_deg=math.degrees(line['heading']),
            lateral_m=line['lateral'], offset_m=offset, aligned=False,
            command_steer=steer, target_steer=target, lookahead_m=lookahead,
            guard_reason='footprint_crosses_left_boundary')
        return ctx.call('motion', 'stop', 'left_boundary_footprint_crossing')
    ctx.left_reference = dict(stamp=now,heading_deg=math.degrees(line['heading']),
        lateral_m=line['lateral'],offset_m=line.get('offset_m',cfg['lane_width']/2),aligned=aligned,
        command_steer=steer,target_steer=target,lookahead_m=lookahead)
    ctx.gap_origin,ctx.gap_start,ctx.gap_steer = ctx.pose,now,steer
    return speed,steer


def left_exit_line(ctx, now):
    """Fit only current observed near-left points, never an inferred center."""
    capture = ctx.right_lock is not None and ctx.cfg.get('right_direct_left_follow',False)
    maximum = ctx.cfg.get('right_left_capture_max_m',.80) if capture else ctx.cfg['lane_width']
    age = now-ctx.left_boundary_stamp
    diagnostic = dict(stamp=now,source_stamp=ctx.left_boundary_stamp,
        age_s=age if ctx.left_boundary_stamp >= 0 else None,
        observed_points=len(ctx.left_boundary),near_points=None,span_m=None,
        error_m=None,distance_m=None,heading_deg=None,min_distance_m=.10,
        max_distance_m=maximum,max_error_m=.035,min_span_m=.10,
        min_points=3,valid=False,reason='not_received')
    ctx.left_fit_diagnostic = diagnostic
    def reject(reason):
        diagnostic['reason'] = reason
        return None
    if ctx.left_boundary_stamp < 0:
        return reject('not_received')
    if age < 0:
        return reject('future_stamp')
    if age > ctx.cfg['sensor_timeout']:
        return reject('stale')
    if ctx.right_lock is not None and ctx.left_boundary_stamp <= ctx.action_started:
        return reject('before_action')
    points = sorted(local(ctx.pose,p) for p in ctx.left_boundary)
    points = [p for p in points if .10 <= p[0] <= 1.10]
    diagnostic['near_points'] = len(points)
    diagnostic['span_m'] = points[-1][0]-points[0][0] if points else 0.0
    if len(points) < 3:
        return reject('too_few_near_points')
    # The circular road needs a local tangent, not a chord fitted across a
    # metre of curved tape. Keep at least three measured points and include
    # the nearby alignment-lookahead segment. Distant curvature must neither
    # reject this segment nor extrapolate a false boundary through the body.
    near_end = points[0][0] + ctx.cfg.get('right_align_lookahead', .30)
    count = max(3, sum(p[0] <= near_end for p in points))
    points = points[:count]
    diagnostic['fit_points'] = len(points)
    diagnostic['span_m'] = points[-1][0]-points[0][0]
    if diagnostic['span_m'] < .10-1e-9:
        return reject('span_too_short')
    mx = sum(p[0] for p in points)/len(points)
    my = sum(p[1] for p in points)/len(points)
    xx = sum((x-mx)**2 for x,y in points)
    slope = sum((x-mx)*(y-my) for x,y in points)/xx
    intercept = my-slope*mx
    norm = math.hypot(1,slope)
    error = max(abs(y-slope*x-intercept)/norm for x,y in points)
    diagnostic.update(error_m=error,distance_m=intercept/norm,heading_deg=math.degrees(math.atan(slope)))
    if error > .035:
        return reject('fit_error_too_large')
    if intercept/norm < .10:
        return reject('not_left_or_too_close')
    if intercept/norm > maximum:
        return reject('left_distance_too_far')
    diagnostic.update(valid=True,reason='ok')
    # Offset by half a lane along the inward normal, not toward the line.
    half = ctx.cfg['lane_width']/2
    centers = [(x+half*slope/norm,slope*x+intercept-half/norm) for x,y in points]
    return dict(points=centers,heading=math.atan(slope),
                lateral=intercept/norm-half,offset_m=half,error=error)
