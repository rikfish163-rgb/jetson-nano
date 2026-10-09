"""Production RIGHT: reverse, forward right, reverse, stop, then lane handoff.
Durations measure outgoing commands, not wheel motion.
"""
import math
import numpy as np
from robot.common.geometry import local, distance

STAGES = (
    ('TIMED_REVERSE', 'reverse', 1.0, -30, 0),
    ('TIMED_TURN', 'turn', 2.0, 30, -22),
    ('TIMED_EXIT_REVERSE', 'exit_reverse', 1.0, -30, 0),
)


def search_straight(ctx, now):
    task = ctx.right_lock
    task.update(stable_since=None, stable_s=0., aligned_frames=0,
                reference_reject_reason=ctx.exit_reason)
    ctx.exit_count = 0
    stamp = task.get('exit_track_stamp')
    age = None if stamp is None else now-stamp
    task['held_reference_age_s'] = age
    # Only an accepted exit-lane command may bridge a missing/weak frame.
    # Camera source time, not repeated control ticks, bounds its lifetime.
    hold = age is not None and 0 <= age <= 1.
    steer = task['exit_track_steer'] if hold else 0.
    if not hold:
        ctx.left_reference = None
    command = (ctx.cfg['straight_speed_raw'], steer)
    result = ctx.call('obstacle', 'checked_command', command, now, False)
    if result == command:
        ctx.reason = ('right_timed_exit_hold_reference' if hold else
                      'right_timed_exit_search_straight')
    return result


def next_blue_handoff(ctx, now):
    """A queued sign plus a confirmed NEW transverse blue can finish the exit."""
    cfg, task = ctx.cfg, ctx.right_lock
    # P1-P3 and explicit S start on P itself, without a junction blue.
    if (ctx.next_direction == 'PARKING' and cfg.get('parking_enabled',True) and
            (cfg.get('parking_mode') == 'timed_sequence' or
             (cfg.get('parking_mode') == 'forward_center' and
              cfg.get('parking_entry_style') == 'S'))):
        ctx.call('mission','resume_lane')
        return ctx.call('mission','dispatch',now)
    marker = ctx.marker
    valid = ctx.next_direction is not None and marker is not None
    if valid:
        point, stamp = marker
        x,y = local(ctx.pose,point)
        valid = (stamp > task['exit_started'] and
                 0 <= now-stamp <= cfg.get('ground_timeout',1.25) and
                 0 <= now-ctx.front_marker_stamp <= cfg.get('ground_timeout',1.25) and
                 0 < x <= cfg.get('straight_align_distance',1.6) and abs(y) <= cfg['lane_width'] and
                 (ctx.consumed_marker is None or
                  distance(point,ctx.consumed_marker) >= cfg['marker_rearm_distance']))
        lines = [line for line in ctx.front_blue_lines if distance(line['point'],point) < .05]
        valid = valid and any(line['length'] >= cfg['blue']['long_min'] and
            abs((line['yaw']-ctx.pose[2]+math.pi/2)%math.pi-math.pi/2) <= math.radians(60)
            for line in lines)
    if not valid:
        task['exit_blue_count'] = 0
        return None
    previous = task.get('exit_blue_stamp',-1.)
    if stamp > previous:
        if (stamp-previous > .5 or
                distance(point,task.get('exit_blue_point',point)) > .15 or
                not task.get('exit_blue_count',0)):
            task.update(exit_blue_count=0,exit_blue_since=stamp)
        task.update(exit_blue_stamp=stamp,exit_blue_point=point,
                    exit_blue_count=task.get('exit_blue_count',0)+1)
    if task.get('exit_blue_count',0) < 3 or stamp-task['exit_blue_since'] < .15:
        return None
    ctx.call('mission','resume_lane')
    ctx.follow_left_boundary = False
    ctx.left_reference = None
    # resume_lane clears the old maneuver's markers. Preserve this confirmed
    # next junction for ordinary sign/blue dispatch and its alignment sequence.
    ctx.marker = marker
    result = ctx.call('mission','dispatch',now)
    return result if result is not None else ctx.call(
        'obstacle','checked_command',(cfg['straight_speed_raw'],0.),now,False)


def align_exit(ctx, now):
    """Finish on the normal road center, independent of the U-turn left rule."""
    cfg, task = ctx.cfg, ctx.right_lock
    if task['phase'] == 'WAIT_LANE':
        task.update(phase='ALIGN_LANE', stable_since=None, aligned_frames=0,
                    exit_started=task.get('exit_started', now))
        ctx.exit_count, ctx.exit_stamp = 0, -1.
        ctx.left_reference = None
    ctx.lane_source = 'right_timed_exit_alignment'
    task['exit_source'] = 'lane_center'
    handoff = next_blue_handoff(ctx,now)
    if handoff is not None:
        return handoff
    stamp = ctx.lane_stamp
    points = sorted(local(ctx.pose, p) for p in ctx.lane)
    points = [p for p in points if .10 <= p[0] <= 1.10]
    if (stamp <= task['exit_started'] or not ctx.call('lane', 'lane_tracking_ready', now, points)
            or len(points) < 2 or points[-1][0]-points[0][0] < .10):
        task.update(stable_since=None, stable_s=0., aligned_frames=0)
        ctx.exit_count = 0
        ctx.exit_reason = ('before_timed_completion' if stamp <= task['exit_started']
                           else 'missing_near_lane')
        return search_straight(ctx,now)

    near_end = points[0][0] + cfg.get('right_align_lookahead', .30)
    points = points[:max(3, sum(p[0] <= near_end for p in points))]
    mx = sum(p[0] for p in points)/len(points)
    xx = sum((x-mx)**2 for x,y in points)
    if xx <= 1e-9:
        task.update(stable_since=None, stable_s=0., aligned_frames=0)
        ctx.exit_count = 0
        ctx.exit_reason = 'degenerate_near_lane'
        return search_straight(ctx,now)
    # Evaluate the near curve's tangent at the vehicle, not the forward
    # chord angle. A car correctly following the circular road must be able
    # to finish the turn without straightening that road's visible arc.
    fit = np.polyfit([x-mx for x,y in points], [y for x,y in points],
                     2 if len(points) >= 3 else 1)
    slope = float(np.polyval(np.polyder(fit), -mx))
    intercept = float(np.polyval(fit, -mx))
    error = max(abs(y-float(np.polyval(fit, x-mx))) for x,y in points)
    if error > .035 or abs(math.atan(slope)) >= math.radians(60.):
        task.update(stable_since=None, stable_s=0., aligned_frames=0)
        ctx.exit_count = 0
        ctx.exit_reason = 'unreliable_near_lane'
        return search_straight(ctx,now)
    line = dict(points=points, heading=math.atan(slope),
                lateral=intercept/math.hypot(1., slope), offset_m=0.)
    task['reference_source_stamp'] = stamp
    task['reference_fit_error_m'] = error

    command = ctx.call('lane', 'left_reference_command', line, now)
    reference = ctx.left_reference
    if reference is None or reference.get('stamp') != now or reference.get('guard_reason'):
        task.update(stable_since=None, stable_s=0., aligned_frames=0)
        ctx.exit_count = 0
        ctx.exit_reason = 'invalid_lane_reference'
        if reference is not None and reference.get('guard_reason'):
            return command
        return search_straight(ctx,now)
    result = ctx.call('obstacle', 'checked_command', command, now, False)
    if result != command:
        task.update(stable_since=None, stable_s=0., aligned_frames=0)
        ctx.exit_count = 0
        return result
    task.update(alignment_heading_deg=reference['heading_deg'],
                alignment_lateral_m=reference['lateral_m'],
                reference_reject_reason=None, held_reference_age_s=None)
    if stamp >= task.get('exit_track_stamp', -1.):
        task.update(exit_track_stamp=stamp, exit_track_steer=result[1])
    if stamp > ctx.exit_stamp:
        if stamp-ctx.exit_stamp > cfg['sensor_timeout']:
            task['stable_since'] = None
            ctx.exit_count = 0
        ctx.exit_stamp = stamp
        if reference['aligned']:
            if task['stable_since'] is None:
                task['stable_since'] = stamp
            ctx.exit_count += 1
        else:
            task['stable_since'] = None
            ctx.exit_count = 0
    task['aligned_frames'] = ctx.exit_count
    task['stable_s'] = 0. if task['stable_since'] is None else stamp-task['stable_since']
    ctx.exit_reason = 'ok' if reference['aligned'] else 'lane_reference_not_aligned'
    ctx.reason = 'right_timed_exit_align'
    # Alignment is ongoing feedback, not a completion gate. Keep correcting
    # even after a few aligned frames; the next sign/blue owns the handoff.
    return result


def resume_normal_lane(ctx, now):
    """The fixed maneuver is complete; ordinary lane control owns the output."""
    ctx.call('mission', 'resume_lane')
    command = ctx.call('lane', 'lane_command', now)
    return ctx.call('obstacle', 'checked_command', command, now, True)


def tick(ctx, now):
    cfg, task = ctx.cfg, ctx.right_lock
    if task.get('phase') in ('WAIT_LANE', 'ALIGN_LANE'):
        return resume_normal_lane(ctx, now)
    if not 0 <= now-ctx.front_marker_stamp <= cfg.get('ground_timeout', 1.25):
        ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'right_timed_front_stale')
    if task.get('timed_started') is None:
        task.update(phase=STAGES[0][0], timed_started=now, timed_last=now)
    if not 0 <= now-task['timed_last'] <= 2.00:
        ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'right_timed_control_gap')
    task['timed_last'] = now
    if task['phase'] == 'EXIT_SETTLE':
        if now < task['settle_until']:
            return ctx.call('motion', 'stop', 'right_timed_exit_settle')
        return resume_normal_lane(ctx, now)
    index = next(i for i, stage in enumerate(STAGES) if stage[0] == task['phase'])
    phase, key, seconds, default_speed, default_raw = STAGES[index]
    duration = cfg.get('right_timed_'+key+'_s', seconds)
    if now-task['timed_started'] >= duration:
        index += 1
        if index == len(STAGES):
            task.update(phase='EXIT_SETTLE',
                        settle_until=now+cfg.get('right_timed_exit_stop_s',2.0))
            ctx.exit_count, ctx.exit_stamp = 0, -1.
            return ctx.call('motion', 'stop', 'right_timed_exit_settle')
        phase, key, seconds, default_speed, default_raw = STAGES[index]
        task.update(phase=phase, timed_started=now)
    speed = cfg.get('right_timed_'+key+'_speed_raw', default_speed)
    raw = cfg.get('right_timed_'+key+'_steering_raw', default_raw)
    scale = cfg.get('steering_command_scale_rad', cfg['max_steer'])
    command = (float(speed)/cfg['speed_sign'],
               float(raw)/cfg['steering_raw_limit']*scale/cfg['steering_sign'])
    ctx.lane_source = 'right_timed_'+key
    result = ctx.call('obstacle', 'checked_command', command, now, False)
    if result != command:
        ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'right_timed_guard:'+ctx.reason)
    ctx.reason = ctx.lane_source
    return result
