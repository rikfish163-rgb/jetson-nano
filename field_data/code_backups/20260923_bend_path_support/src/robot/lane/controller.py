"""lane module: explicit context input; coordinator applies returned updates."""
from __future__ import division

import math
from robot.common.geometry import distance
from robot.common.geometry import local



FIELDS = ('action', 'action_started', 'cfg', 'follow_left_boundary', 'gap_origin', 'gap_start',
 'gap_steer', 'lane', 'lane_boundaries', 'lane_confidence', 'lane_source', 'lane_stamp',
 'left_boundary', 'left_boundary_stamp', 'left_fit_diagnostic', 'left_reference',
 'park_line_side', 'pending', 'pose', 'right_lock', 'right_tail_origin', 'state', 'lane_recovery',
 'lane_bend_direction', 'lane_bend_evidence_frames', 'lane_bend_stamp')
CALLS = (('lane', 'lane_valid'), ('lane', 'left_exit_line'), ('lane', 'left_reference_command'),
 ('lane', 'park_line_command'), ('motion', 'current_steering'), ('motion', 'stop'))
OPERATIONS = ('lane_valid', 'forward_lane_ready', 'lane_command', 'park_line_command', 'left_reference_command',
 'left_exit_line')

def lane_valid(ctx, now):
    return (0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout'] and len(ctx.lane) >= 2
            and ctx.lane_confidence >= ctx.cfg['lane_min_confidence'])


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


def lane_command(ctx, now, approach=False):
    ordinary = ctx.action is None or approach
    if (not ordinary or
            ctx.follow_left_boundary or ctx.right_tail_origin is not None):
        ctx.lane_recovery = None
    if ctx.pending == 'PARKING' and ctx.action is None and ctx.cfg.get('parking_mode') != 'forward_center':
        return ctx.call('lane', 'park_line_command', now)
    left = None
    if ctx.follow_left_boundary:
        left = ctx.call('lane', 'left_exit_line', now)
    elif ctx.right_tail_origin is not None:
        if distance(ctx.pose,ctx.right_tail_origin) <= ctx.cfg.get('right_turn_exit',ctx.cfg['turn_exit']):
            left = ctx.call('lane', 'left_exit_line', now)
        else:
            ctx.right_tail_origin = None
    ctx.lane_source = 'left_boundary' if left or ctx.follow_left_boundary else 'center'
    if ctx.follow_left_boundary:
        if left is None:
            ctx.state = 'GAP'
            ctx.left_reference = None
            return ctx.call('motion', 'stop', 'left_reference_missing')
        ctx.state = 'LANE'
        return ctx.call('lane', 'left_reference_command', left,now)
    if left is None and not 0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout']:
        if ctx.gap_origin is not None and ordinary and ctx.lane_recovery is None:
            ctx.lane_recovery = dict(stamp=now)
        return ctx.call('motion', 'stop', 'lane_stream_stale')
    if left is None and (ctx.follow_left_boundary or not ctx.call('lane', 'lane_valid', now)):
        if ctx.gap_origin is None:
            return ctx.call('motion', 'stop', 'no_initial_lane')
        if ordinary and ctx.lane_recovery is None:
            ctx.lane_recovery = dict(stamp=now)
        if ctx.state != 'GAP':
            ctx.gap_steer = ctx.call('motion', 'current_steering', now)
        if (distance(ctx.pose,ctx.gap_origin) > ctx.cfg['gap_max_distance'] or
                now-ctx.gap_start > ctx.cfg['gap_max_seconds']):
            return ctx.call('motion', 'stop', 'gap_limit_wait_for_lane')
        ctx.state = 'GAP'
        return ctx.cfg['speed_raw']['gap'], ctx.gap_steer
    ctx.state = 'LANE'
    candidates = left['points'] if left else [local(ctx.pose,p) for p in ctx.lane]
    candidates = [p for p in candidates if p[0] > 0.05]
    if not candidates:
        if (ctx.gap_origin is not None and ordinary and left is None
                and ctx.lane_recovery is None):
            ctx.lane_recovery = dict(stamp=now)
        return ctx.call('motion', 'stop', 'lane_behind_vehicle')
    target = min(candidates, key=lambda p: abs(math.hypot(*p)-ctx.cfg['lookahead']))
    x,y = target
    steer = math.atan2(2*ctx.cfg['wheelbase']*y, max(0.0025,x*x+y*y))
    # A distant target can understate a tight near bend. Only strengthen an
    # existing turn when an observed nearer target agrees with its direction.
    near_lookahead=ctx.cfg.get('lane_curve_lookahead_m',ctx.cfg['lookahead'])
    scale=ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    if (ordinary and left is None and near_lookahead<ctx.cfg['lookahead']
            and abs(steer)>.35*scale):
        nx,ny=min(candidates,key=lambda p:abs(math.hypot(*p)-near_lookahead))
        near_steer=math.atan2(2*ctx.cfg['wheelbase']*ny,max(.0025,nx*nx+ny*ny))
        if near_steer*steer>0 and abs(near_steer)>abs(steer):
            steer=near_steer
    steer = max(-ctx.cfg['max_steer'],min(ctx.cfg['max_steer'],steer))
    speed = ctx.cfg['speed_raw']['lane']
    if ordinary and left is None:
        # Slow translation, not steering, to reduce travel during visual delay.
        # Use the unfiltered demand AND current command: crossing zero during
        # a recovery reversal must not be mistaken for a straight section.
        scale = ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
        demand = min(1.,max(abs(steer),
            abs(ctx.call('motion','current_steering',now)))/scale)
        blend = max(0.,(demand-.35)/.65)
        curve_speed = min(speed,ctx.cfg.get('lane_curve_speed_raw',speed))
        speed -= (speed-curve_speed)*blend
    if ctx.lane_recovery is not None and ordinary and left is None:
        steer = recovery_steering(ctx,steer,now)
    if ordinary and left is None:
        steer, bend_locked = bend_direction_steering(ctx,steer,now)
        if bend_locked:
            ctx.lane_source = 'center_bend_' + ctx.lane_bend_direction.lower()
    ctx.gap_origin, ctx.gap_start, ctx.gap_steer = ctx.pose, now, steer
    return speed, steer


def bend_direction_steering(ctx, requested, now):
    """Use this frame's confirmed bend for sign and path demand for magnitude."""
    direction = {'LEFT':1, 'RIGHT':-1}.get(ctx.lane_bend_direction,0)
    if (not direction or ctx.lane_bend_evidence_frames < 3 or
            ctx.lane_bend_stamp != ctx.lane_stamp or
            not 0 <= now-ctx.lane_bend_stamp <= ctx.cfg['sensor_timeout'] or
            requested*direction >= 0):
        return requested,False
    scale = ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    return direction*min(scale,abs(requested)),True


def recovery_steering(ctx, requested, now):
    """After loss, slew only reversal; never delay same-side cornering."""
    scale = ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    before = max(-scale,min(scale,ctx.call('motion','current_steering',now)))
    target = max(-scale,min(scale,requested))
    origin_side = ctx.lane_recovery.get('origin_side',
        1 if before>0 else -1 if before<0 else 0)
    reducing = before*target>0 and abs(target)<=abs(before)
    if origin_side==0 or target*origin_side>=0 or reducing:
        ctx.lane_recovery = None
        return requested
    # Bound one reversal episode, including intervening missing observations.
    # Start at reacquisition, not at loss: a long gap must not skip the guard.
    started = ctx.lane_recovery.get('started',now)
    rate = ctx.cfg.get('lane_recovery_raw_rate',88.)
    window = 2.*ctx.cfg['steering_raw_limit']/rate
    if now-started >= window-1e-9:
        ctx.lane_recovery = None
        return requested
    dt = max(0.,min(.1,now-ctx.lane_recovery['stamp']))
    limit = rate*dt/ctx.cfg['steering_raw_limit']*scale
    if abs(target-before)<=limit+1e-9:
        ctx.lane_recovery = None
        return requested  # Restore the original algorithm immediately on catch-up.
    steer = max(before-limit,min(before+limit,target))
    # Keep the pre-reversal side across zero, so crossing center cannot cause
    # a one-tick jump from a small reverse angle to opposite full lock.
    ctx.lane_recovery = dict(stamp=now,started=started,target=target,command=steer,
                             origin_side=origin_side)
    return steer


def park_line_command(ctx, now):
    """Follow the observed paint itself while awaiting the parking marker."""
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
    x,y = min(points,key=lambda p:abs(math.hypot(*p)-lookahead))
    physical = math.atan2(2*cfg['wheelbase']*y,max(.0025,x*x+y*y))
    physical = max(-cfg['max_steer'],min(cfg['max_steer'],physical))
    scale = cfg.get('steering_command_scale_rad',cfg['max_steer'])
    target = physical/cfg['max_steer']*scale
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
