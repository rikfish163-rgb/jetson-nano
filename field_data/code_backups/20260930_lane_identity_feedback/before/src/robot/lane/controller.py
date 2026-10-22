"""lane module: explicit context input; coordinator applies returned updates."""
from __future__ import division

import math
from robot.common.geometry import distance, _isfinite
from robot.common.geometry import local
from robot.common.contracts import model_to_command_steering
from vehicle_control.pure_pursuit import PurePursuit
from robot.lane.preview import preview_steering, lane_speed



FIELDS = ('action', 'action_started', 'cfg', 'follow_left_boundary', 'gap_origin', 'gap_start',
 'gap_steer', 'lane', 'lane_boundaries', 'lane_confidence', 'lane_source', 'lane_stamp',
 'left_boundary', 'left_boundary_stamp', 'left_fit_diagnostic', 'left_reference',
 'park_line_side', 'pending', 'pose', 'right_lock', 'right_tail_origin', 'state', 'lane_recovery',
 'startup_curve_lock', 'lane_target', 'lane_curve_lock', 'lane_preview', 'issued_steer', 'lane_speed_state')
CALLS = (('lane', 'lane_valid'), ('lane', 'left_exit_line'), ('lane', 'left_reference_command'),
 ('lane', 'park_line_command'), ('motion', 'current_steering'), ('motion', 'stop'))
OPERATIONS = ('lane_valid', 'lane_tracking_ready', 'forward_lane_ready', 'lane_command', 'park_line_command', 'left_reference_command',
 'left_exit_line')


def lane_valid(ctx, now):
    if not (0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout'] and
            ctx.lane_confidence >= ctx.cfg['lane_min_confidence']):
        return False
    if not ctx.cfg.get('lane_curvature_preview',False):
        return len(ctx.lane) >= 2
    points = sorted(local(ctx.pose,p) for p in ctx.lane)
    points = [p for p in points if p[0] > .05]
    return len(points) >= 2 and points[-1][0]-points[0][0] >= ctx.cfg.get('lane_min_path_span',.15)-1e-9


def lane_tracking_ready(ctx, now):
    """Allow coherent current sparse paint at bend speed, never stale guesses."""
    if lane_valid(ctx, now):
        return True
    if (not ctx.cfg.get('lane_curvature_preview',False) or
            not 0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout']):
        return False
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


def right_curve_command(ctx, points, steer):
    """Retain right-bend intent, allowing fresh path corrections in preview mode.

    This is ordinary-lane intent, separate from signed intersection maneuvers.
    Never count repeated control ticks as new visual evidence.
    """
    if ctx.action is not None:
        ctx.lane_curve_lock = None
        return steer, False
    points = sorted(p for p in points if p[0] > 0.)
    span = points[-1][0]-points[0][0] if len(points) >= 2 else 0.
    heading = math.atan2(points[-1][1]-points[0][1],span) if span > 0 else 0.
    coherent = (len(points) >= 3 and span >= .20 and
                all(b[1]-a[1] <= .04 for a,b in zip(points,points[1:])))
    full_right = -ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    lock = ctx.lane_curve_lock
    if lock is None:
        if not (coherent and heading <= -math.radians(10) and points[-1][1] <= -.03):
            return steer, False
        lock = dict(direction='RIGHT',started=ctx.lane_stamp,stamp=-1.,
                    steer=full_right,exit_since=None,exit_frames=0)
        ctx.lane_curve_lock = lock
    # A straight exit may point left while the car still needs countersteer.
    # Judge its position near the car, not whether every far point is centered.
    # Keep right-facing bends and straight paths in another lane protected.
    exit_line = False
    inside_correction = False
    if len(points) >= 3 and span >= .20:
        mx = sum(p[0] for p in points)/len(points)
        my = sum(p[1] for p in points)/len(points)
        slope = sum((x-mx)*(y-my) for x,y in points)/sum((x-mx)**2 for x,y in points)
        residual = max(abs(y-my-slope*(x-mx)) for x,y in points)
        near_offset = my+slope*(.50-mx)
        tail_start = len(points)-3
        while (tail_start >= 0 and
               points[-1][0]-points[tail_start][0] < .20-1e-9):
            tail_start -= 1
        tail_angle = None
        if tail_start >= 0:
            tail = points[tail_start:]
            tail_mx = sum(x for x,y in tail)/len(tail)
            tail_my = sum(y for x,y in tail)/len(tail)
            tail_xx = sum((x-tail_mx)**2 for x,y in tail)
            tail_slope = sum((x-tail_mx)*(y-tail_my) for x,y in tail)/tail_xx
            tail_angle = math.atan(tail_slope)
        exit_line = (-math.radians(8) <= math.atan(slope) <= math.radians(25)
                     and residual <= .025 and abs(near_offset) <= .15
                     and tail_angle is not None
                     and tail_angle >= -math.radians(8))
        inside_correction = (ctx.cfg.get('lane_curvature_preview',False) and
            steer > 0 and coherent and residual <= .025 and
            .04 <= near_offset <= .15 and points[-1][1] >= .02 and
            all(0 < y <= .18 for x,y in points) and
            -math.radians(25) <= math.atan(slope) <= math.radians(25))
    if ctx.lane_stamp > lock['stamp']:
        if ctx.lane_stamp-lock['stamp'] > ctx.cfg['sensor_timeout']:
            lock['correction_frames'] = 0
        lock['correction_frames'] = (lock.get('correction_frames',0)+1
                                     if inside_correction else 0)
        if exit_line:
            if lock['exit_since'] is None or ctx.lane_stamp-lock['stamp'] > ctx.cfg['sensor_timeout']:
                lock['exit_since'],lock['exit_frames'] = ctx.lane_stamp,0
            lock['exit_frames'] += 1
        else:
            lock['exit_since'],lock['exit_frames'] = None,0
        lock['stamp'] = ctx.lane_stamp
        if lock['correction_frames'] >= 2:
            ctx.lane_curve_lock = None
            return steer, False
        if lock['exit_frames'] >= 5 and ctx.lane_stamp-lock['exit_since'] >= .4-1e-9:
            ctx.lane_curve_lock = None
            return steer, False
    # A smaller offset alone does not clear bend intent. Preview mode can
    # reduce the angle without dropping the gap/exit guard.
    if ctx.cfg.get('lane_curvature_preview',False) and (steer <= 0 or exit_line):
        # Fresh center observations set the angle. Keep a small right command
        # until exit confirmation; a gap retains this last accepted angle.
        lock['steer'] = min(steer,full_right*.20)
        return lock['steer'], False
    lock['steer'] = full_right
    return full_right, steer > 0. and not exit_line


def lane_gap_command(ctx, now):
    """Bound empty or rejected paths by the last accepted observation."""
    if ctx.gap_origin is None:
        return ctx.call('motion', 'stop', 'no_initial_lane')
    lock = ctx.lane_curve_lock
    if lock is not None:
        ctx.gap_steer = lock['steer']
        lock['exit_since'],lock['exit_frames'] = None,0
        if (not ctx.lane or ctx.lane_target is None or
                ctx.lane_target.get('reason') != 'opposite_target_during_right_curve'):
            lock['correction_frames'] = 0
        lock['stamp'] = ctx.lane_stamp
    elif ctx.state != 'GAP':
        ctx.gap_steer = ctx.call('motion', 'current_steering', now)
    if (distance(ctx.pose,ctx.gap_origin) > ctx.cfg['gap_max_distance'] or
            now-ctx.gap_start > ctx.cfg['gap_max_seconds']):
        return ctx.call('motion', 'stop', 'gap_limit_wait_for_lane')
    ctx.state = 'GAP'
    return lane_speed(ctx, ctx.cfg['speed_raw']['gap']), ctx.gap_steer


def lane_command(ctx, now, approach=False):
    """Follow measured centers, including coherent sparse paint at bend speed."""
    ctx.lane_recovery = None
    if not ctx.call('lane','lane_valid',now):
        ctx.lane_speed_state = None
    if ctx.pending == 'PARKING' and ctx.action is None and ctx.cfg.get('parking_mode') != 'forward_center':
        return ctx.call('lane', 'park_line_command', now)
    ctx.follow_left_boundary = False
    ctx.lane_source = 'center'
    partial = False
    if not 0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout']:
        ctx.lane_preview = None
        return ctx.call('motion', 'stop', 'lane_stream_stale')
    if not ctx.lane:
        return lane_gap_command(ctx, now)
    if ctx.cfg.get('lane_curvature_preview',False) and not ctx.call('lane', 'lane_valid', now):
        partial = lane_tracking_ready(ctx,now)
        if partial:
            ctx.lane_source = 'partial_center'
    if ctx.cfg.get('lane_curvature_preview',False) and not lane_tracking_ready(ctx,now):
        points = sorted(local(ctx.pose,p) for p in ctx.lane)
        points = [p for p in points if p[0] > .05]
        # Bridge a minor confidence dip using only the previous trusted angle.
        # Bad frames cannot extend this interval or supply new steering.
        span = points[-1][0]-points[0][0] if len(points) >= 2 else 0.
        minor_confidence_dip = (
            ctx.cfg['lane_min_confidence']*.7 <= ctx.lane_confidence < ctx.cfg['lane_min_confidence'] and
            span >= ctx.cfg.get('lane_min_path_span',.15)-1e-9)
        minor_span_dip = (ctx.lane_confidence >= ctx.cfg['lane_min_confidence'] and
                         len(points) >= 2 and span >= .10-1e-9)
        if (ctx.gap_origin is not None and 0 <= now-ctx.gap_start <= .25 and
                (minor_confidence_dip or minor_span_dip)):
            grace = 'confidence_grace' if minor_confidence_dip else 'path_span_grace'
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
    target_index,target = max(forward,key=lambda item:item[1][0])
    offset = target[1]  # y-left = (BEV midpoint u - target u) / 400.
    fraction = max(-1.,min(1.,offset/ctx.cfg.get('lane_lateral_full_scale_m',.075)))
    steer = fraction*ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    steer = preview_steering(ctx, candidates, steer)
    desired_steer = steer
    if ctx.cfg.get('lane_curvature_preview',False):
        # One measured-center controller owns the angle in preview mode.
        # The old right-only lock must not override center corrections.
        ctx.lane_curve_lock = None
        rejected = False
        if ctx.issued_steer is not None:
            dt = max(0.,min(.10,now-ctx.gap_start)) if ctx.gap_origin is not None else .05
            before = ctx.call('motion','current_steering',now)
            limit = 90.*dt/ctx.cfg['steering_raw_limit']*ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
            steer = max(before-limit,min(before+limit,steer))
    else:
        steer, rejected = right_curve_command(ctx, candidates, steer)
    ctx.lane_target = dict(stamp=now, source_stamp=ctx.lane_stamp,
        path=candidates, target=target, target_index=target_index,
        intersection=None, line_x=None, lateral_offset_m=offset,
        angle=steer, desired_angle=desired_steer, valid=True, reason='ok',curvature_preview=ctx.lane_preview,
        selection=('partial_center_geometry' if partial else 'center_geometry' if ctx.cfg.get('lane_curvature_preview',False)
            else 'right_curve_full_lock' if ctx.lane_curve_lock else 'topmost_lateral_offset'))
    if rejected:
        ctx.lane_source = 'right_curve_hold'
        ctx.lane_target.update(target=None,rejected_target=target,valid=False,
            reason='opposite_target_during_right_curve',selection='right_curve_hold')
        command = lane_gap_command(ctx, now)
        ctx.lane_target['angle'] = command[1]
        return command
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


def left_reference_command(ctx, line, now):
    """Track the 30 cm inward offset with consistent steering units and slew."""
    cfg = ctx.cfg
    points = [p for p in line['points'] if p[0] > .05]
    if not points:
        ctx.left_reference = None
        return ctx.call('motion', 'stop', 'left_reference_behind')
    aligning = ctx.right_lock is not None and ctx.right_lock['phase'] == 'ALIGN_LEFT'
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
    ctx.left_reference = dict(stamp=now,heading_deg=math.degrees(line['heading']),
        lateral_m=line['lateral'],offset_m=line.get('offset_m',cfg['lane_width']/2),aligned=aligned,
        command_steer=steer,target_steer=target,lookahead_m=lookahead)
    speed = cfg['speed_raw']['lane'] if aligned and ctx.action != 'RIGHT' else cfg.get('left_reference_speed_raw',12)
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
                lateral=intercept/norm-half,error=error)
