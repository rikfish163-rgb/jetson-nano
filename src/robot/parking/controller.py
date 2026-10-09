"""parking module: explicit context input; coordinator applies returned updates."""
from __future__ import division

import math
from robot.common.geometry import distance
from robot.common.geometry import local
from robot.common.geometry import slot_samples
from robot.common.geometry import world
from robot.common.geometry import wrap
from robot.parking.planner import auto_parking_plan
from robot.parking.planner import rank_parking_slots
from robot.common.planning import hybrid_plan
from robot.parking.entry import select_entry
from robot.parking.entry import ExplicitStraightEntry
from robot.parking.entry import TimedRightEntry
from robot.common.lidar_policy import explicit_parking_lidar_disabled



FIELDS = ('action', 'action_started', 'cfg', 'executor', 'follower', 'future',
 'parking_candidate_stamp', 'parking_candidates', 'parking_detection',
 'parking_diagnostics', 'parking_entry', 'parking_front_frames', 'parking_lines',
 'parking_sign',
 'parking_lines_stamp', 'parking_trigger_at', 'parking_trigger_pose', 'pose', 'reason',
 'retry_at', 'scan', 'slot', 'slot_locked', 'slot_stamp', 'state', 'pending',
 'pending_at')
CALLS = (('mission', 'start_follow'), ('motion', 'stop'),
         ('obstacle', 'scan_ready'), ('obstacle', 'checked_command'))
OPERATIONS = ('start_parking', 'auto_parking_tick',
              'start_explicit_parking', 'explicit_parking_tick')


def _explicit_style(ctx):
    style = str(ctx.cfg.get('parking_entry_style', '')).strip().upper()
    slot = str(ctx.cfg.get('parking_slot', '')).strip().upper()
    if slot not in ('P4', 'P5') or style not in ('S', 'T'):
        raise ValueError('explicit parking requires P4/P5 and parking_entry_style S or T')
    return style


def _explicit_fault(ctx, reason):
    ctx.state = 'FAULT'
    ctx.reason = reason
    return ctx.call('motion', 'stop', reason)


def _fresh_sign_anchor(ctx, now, sign_anchor):
    """Return only a point known to belong to the current PARKING sign."""
    if isinstance(sign_anchor, dict):
        point = sign_anchor.get('point')
        stamp = sign_anchor.get('stamp')
        try:
            age = now - float(stamp)
        except (TypeError, ValueError):
            return None
        if point is not None and 0 <= age <= ctx.cfg.get('sign_timeout', .8):
            return point
        return None
    if sign_anchor is not None:
        return sign_anchor
    sign = ctx.parking_sign
    if not isinstance(sign, dict) or sign.get('point') is None:
        return None
    try:
        age = now - float(sign.get('stamp'))
    except (TypeError, ValueError):
        return None
    if 0 <= age <= ctx.cfg.get('sign_timeout', .8):
        return sign['point']
    return None


def start_explicit_parking(ctx, now, sign_anchor=None, trigger=None):
    """Create the selected P4/P5 entry task at its authorized trigger.

    ``S`` is armed by the PARKING sign and consumes only measured bay lines.
    ``T`` is armed by the already-confirmed blue line; its one-second task is
    then run by :func:`explicit_parking_tick`.
    """
    try:
        style = _explicit_style(ctx)
    except ValueError as exc:
        return _explicit_fault(ctx, 'parking_explicit_invalid:'+str(exc))
    trigger = (('sign' if style == 'S' else 'blue') if trigger is None else
               str(trigger).strip().lower())
    if (style == 'S' and trigger != 'sign') or (style == 'T' and trigger != 'blue'):
        return _explicit_fault(ctx, 'parking_explicit_wrong_trigger')
    if style == 'S':
        anchor = _fresh_sign_anchor(ctx, now, sign_anchor)
        entry = ExplicitStraightEntry(ctx.cfg, ctx.pose, now, anchor)
        reason = 'parking_s_wait_white_geometry'
    else:
        entry = TimedRightEntry(ctx.cfg, now)
        reason = 'parking_t_entry_start'
    ctx.parking_entry = entry
    ctx.state, ctx.action, ctx.action_started = 'PARKING', 'PARKING', now
    # The confirmed sign is consumed by this task.  Keeping it pending would
    # let the next mission tick try to dispatch the same P instruction again.
    ctx.pending, ctx.pending_at = None, 0.0
    ctx.reason = reason
    return ctx.call('motion', 'stop', reason)


def explicit_parking_tick(ctx, now):
    """Advance an explicit S/T task while retaining a single safety owner."""
    entry = ctx.parking_entry
    if entry is None:
        return _explicit_fault(ctx, 'parking_explicit_task_missing')
    is_timed = isinstance(entry, TimedRightEntry)
    # Only the explicit P4/P5 switch can waive this task's scan requirement.
    if (not explicit_parking_lidar_disabled(ctx.cfg, ctx.state, ctx.action) and
            not ctx.call('obstacle', 'scan_ready', now)):
        if is_timed:
            entry.abort('parking_t_entry_scan_stale')
            return _explicit_fault(ctx, entry.reason)
        return ctx.call('motion', 'stop', 'parking_scan_missing_or_stale')
    command = entry.command(now) if is_timed else entry.command(now, ctx.pose)
    ctx.reason = entry.reason
    if entry.status == 'FINISHED' or entry.phase == 'FINISHED':
        ctx.state = 'FINISHED'
        return ctx.call('motion', 'stop', entry.reason)
    if entry.status == 'FAULT' or entry.phase == 'FAULT':
        return _explicit_fault(ctx, entry.reason)
    if command[0] == 0:
        return ctx.call('motion', 'stop', entry.reason)
    result = ctx.call('obstacle', 'checked_command', command, now, False)
    if is_timed and result != command:
        entry.abort('parking_t_entry_guard:'+str(ctx.reason))
        return _explicit_fault(ctx, entry.reason)
    return result

def start_parking(ctx, now):
    if not ctx.call('obstacle', 'scan_ready', now):
        return ctx.call('motion', 'stop', 'scan_missing_or_stale')
    if not ctx.slot or now-ctx.slot_stamp > ctx.cfg['slot_memory_s']:
        ctx.state = 'WAIT_SLOT'
        return ctx.call('motion', 'stop', 'requested_slot_not_observed')
    if ctx.scan.coverage(slot_samples(ctx.slot)) < ctx.cfg['slot_min_coverage']:
        ctx.state = 'WAIT_SLOT'
        return ctx.call('motion', 'stop', 'requested_slot_occupied_or_unknown')
    ctx.slot_locked = True
    slot, start = dict(ctx.slot), ctx.pose
    goalxy = world(slot['pose'],(-ctx.cfg['wheelbase']/2,0))
    goal = tuple(goalxy)+(slot['pose'][2],)
    # Road rectangle union bay; permits entering through the open mouth,
    # never going through the bay side/back. Dimensions MUST be measured.
    rel = local(start,slot['pose'])
    xlo, xhi = min(0,rel[0])-ctx.cfg['parking_search_x_margin'], max(0,rel[0])+ctx.cfg['parking_search_x_margin']
    side = -1 if ctx.cfg['parking_mouth_side'] == 'right' else 1
    # Estimate mouth at the road-facing bay edge in start frame.
    slot_corners = [local(start,world(slot['pose'],(x,y)))
                    for x in (-slot['length']/2,slot['length']/2)
                    for y in (-slot['width']/2,slot['width']/2)]
    mouth = max(p[1] for p in slot_corners) if side < 0 else min(p[1] for p in slot_corners)
    def allowed(q):
        x,y = local(start,q)
        in_road = (xlo <= x <= xhi and
                   (mouth <= y <= ctx.cfg['parking_road_half_width'] if side < 0
                    else -ctx.cfg['parking_road_half_width'] <= y <= mouth))
        sx,sy = local(slot['pose'],q)
        return in_road or (abs(sx) <= slot['length']/2 and abs(sy) <= slot['width']/2)
    ctx.future = ctx.executor.submit(hybrid_plan,start,goal,ctx.cfg,list(ctx.scan.obstacles),allowed,-1)
    ctx.state = 'PLANNING'
    ctx.action = 'PARKING'
    ctx.action_started = now
    return ctx.call('motion', 'stop', 'planning_reverse_entry')


def auto_parking_tick(ctx, now):
    cfg = ctx.cfg
    if not ctx.call('obstacle', 'scan_ready', now):
        return ctx.call('motion', 'stop', 'scan_missing_or_stale')
    fresh_front = (ctx.parking_candidate_stamp >= ctx.parking_trigger_at and
                   0 <= now-ctx.parking_candidate_stamp <= cfg.get('ground_timeout',1.25) and
                   abs(ctx.scan.stamp-ctx.parking_candidate_stamp) <= cfg.get('parking_sync_s',.5))
    if cfg.get('parking_mode')=='forward_white':
        # After stopping at blue, static world-space bay geometry may be
        # paired with the LATEST live scan. Do not widen moving-data timeouts.
        stationary=(ctx.parking_trigger_pose is not None and
                    distance(ctx.pose,ctx.parking_trigger_pose)<=.03 and
                    abs(wrap(ctx.pose[2]-ctx.parking_trigger_pose[2]))<=math.radians(3))
        fresh_front = fresh_front or (stationary and
            ctx.parking_candidate_stamp>=ctx.parking_trigger_at+cfg.get('parking_observe_s',.8)/2 and
            0<=now-ctx.parking_candidate_stamp<=cfg.get('ground_timeout',1.25) and
            ctx.scan.stamp>=ctx.parking_trigger_at and
            0<=now-ctx.scan.stamp<=cfg['sensor_timeout'])
        ctx.parking_detection.update(stationary_pairing=stationary,
            image_scan_delta_s=abs(ctx.scan.stamp-ctx.parking_candidate_stamp),
            sensor_pair_ready=fresh_front)
    if ctx.state == 'AUTO_PLANNING':
        if not ctx.future.done():
            return ctx.call('motion', 'stop', 'planning_empty_bay')
        path, reason, slot, rows = ctx.future.result()
        ctx.parking_diagnostics = rows
        if not path:
            ctx.state = 'PARKING_SCAN'
            ctx.retry_at = now+2.0
            return ctx.call('motion', 'stop', reason)
        if not fresh_front:
            return ctx.call('motion', 'stop', 'waiting_selected_bay_confirmation')
        # The vehicle remained stopped during planning. Recheck the SAME
        # physical bay against fresh front geometry and the latest lidar.
        visible = fresh_front and any(s['kind'] == slot['kind'] and
                                       abs(wrap(s['pose'][2]-slot['pose'][2])) <= cfg['parking_yaw_match_tolerance'] and
                                       distance(s['pose'],slot['pose']) <= cfg['slot_match_distance']
                                       for s in ctx.parking_candidates)
        free = rank_parking_slots(ctx.pose,[slot],ctx.scan,cfg)[0]['occupancy'] == 'FREE'
        if not visible or not free:
            ctx.state = 'PARKING_SCAN'
            return ctx.call('motion', 'stop', 'selected_bay_needs_fresh_confirmation')
        if distance(ctx.pose,path[0]) > cfg['slot_match_distance'] or abs(wrap(ctx.pose[2]-path[0][2])) > cfg['exit_yaw_tolerance']:
            ctx.state = 'PARKING_SCAN'
            return ctx.call('motion', 'stop', 'parking_pose_changed_reobserve')
        ctx.slot, ctx.slot_stamp, ctx.slot_locked = slot,ctx.parking_candidate_stamp,True
        ctx.call('mission', 'start_follow', path,'PARKING',now)
        return ctx.call('motion', 'stop', 'empty_bay_locked')
    if (now-ctx.parking_trigger_at < cfg.get('parking_observe_s',.8) or
            ctx.parking_front_frames < 2 or not fresh_front):
        return ctx.call('motion', 'stop', 'waiting_fresh_front_and_scan')
    if cfg.get('parking_mode') == 'forward_white':
        if (ctx.parking_lines_stamp < ctx.parking_trigger_at or
                not 0 <= now-ctx.parking_lines_stamp <= cfg.get('ground_timeout',1.25)):
            return ctx.call('motion', 'stop', 'parking_wait_fresh_white_lines')
        entry = select_entry(ctx,now)
        if entry is None:
            return ctx.call('motion', 'stop', ctx.reason)
        ctx.parking_entry = entry
        ctx.slot,ctx.slot_stamp,ctx.slot_locked = entry.slot,ctx.parking_candidate_stamp,True
        ctx.state,ctx.action,ctx.action_started = 'PARKING','PARKING',now
        ctx.follower = None
        return ctx.call('motion', 'stop', 'parking_empty_bay_locked')
    ctx.parking_diagnostics = rank_parking_slots(ctx.pose,ctx.parking_candidates,ctx.scan,cfg)
    if not any(s['occupancy'] == 'FREE' for s in ctx.parking_diagnostics):
        return ctx.call('motion', 'stop', 'no_observed_empty_bay')
    if now < ctx.retry_at:
        return ctx.call('motion', 'stop', 'parking_reobserve_after_failed_plan')
    ctx.future = ctx.executor.submit(auto_parking_plan,ctx.pose,
                                      list(ctx.parking_candidates),ctx.scan,cfg)
    ctx.state, ctx.action, ctx.action_started = 'AUTO_PLANNING','PARKING',now
    return ctx.call('motion', 'stop', 'planning_empty_bay')
