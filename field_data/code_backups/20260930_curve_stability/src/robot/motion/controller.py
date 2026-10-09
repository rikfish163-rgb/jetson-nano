"""motion module: explicit context input; coordinator applies returned updates."""
from __future__ import division

from robot.common.geometry import _isfinite
from robot.common.geometry import distance
from robot.common.geometry import inside_slot
from robot.common.geometry import wrap
from robot.common.contracts import model_to_command_steering



FIELDS = ('action', 'applied_stamp', 'applied_steer', 'cfg', 'finish_pose_stamp', 'finished_count',
 'follower', 'front_marker_stamp', 'gap_steer', 'issued_steer', 'lane_stamp', 'pose',
 'pose_stamp', 'reason', 'right_lock', 'slot', 'slot_stamp', 'state', 'straight_search',
 'timed_uturn', 'timed_bypass', 'lane_curve_lock')
CALLS = (('lane', 'lane_command'), ('mission', 'resume_lane'), ('motion', 'stop'),
 ('obstacle', 'checked_command'), ('turn', 'direction_exit_reached'),
 ('turn', 'exit_lane_confirmed'), ('turn', 'right_lock_tick'),
 ('turn', 'straight_search_tick'))
OPERATIONS = ('observe_applied_steering', 'current_steering', 'stop', 'follow_path_tick')

def observe_applied_steering(ctx, steer, stamp):
    if stamp >= ctx.applied_stamp and _isfinite(steer):
        ctx.applied_steer, ctx.applied_stamp = steer, stamp


def current_steering(ctx, now):
    if 0 <= now-ctx.applied_stamp < .25:
        return ctx.applied_steer
    return ctx.issued_steer if ctx.issued_steer is not None else ctx.gap_steer


def stop(ctx, reason):
    if ctx.timed_bypass is not None:
        ctx.timed_bypass['last']=None
    if ctx.timed_uturn is not None:
        ctx.timed_uturn.pause()
    ctx.reason = reason
    steer = 0.0
    if (reason in ('lane_stream_stale', 'scan_missing_or_stale') and
            ctx.action is None and ctx.state in ('LANE', 'GAP') and
            ctx.lane_curve_lock is not None):
        # Stop propulsion without straightening the wheels while the car
        # may still be slowing through the confirmed bend.
        steer = ctx.lane_curve_lock['steer']
        ctx.lane_curve_lock['exit_since'] = None
        ctx.lane_curve_lock['exit_frames'] = 0
    ctx.issued_steer = steer
    return 0, steer


def follow_path_tick(ctx, now):
    """Execute the selected path and report its completion or control command."""
    cfg = ctx.cfg
    if ctx.straight_search is not None:
        return ctx.call('turn', 'straight_search_tick', now)
    if ctx.right_lock is not None:
        return ctx.call('turn', 'right_lock_tick', now)
    if (ctx.state == 'MANEUVER' and
            not 0 <= now-ctx.lane_stamp <= cfg['sensor_timeout'] and
            not 0 <= now-ctx.front_marker_stamp <= cfg.get('ground_timeout',1.25)):
        return ctx.call('motion', 'stop', 'maneuver_front_stale')
    if (ctx.state == 'MANEUVER' and ctx.action in ('LEFT','RIGHT','STRAIGHT') and
            ctx.call('turn', 'direction_exit_reached') and ctx.call('turn', 'exit_lane_confirmed', now)):
        ctx.call('mission', 'resume_lane')
        return ctx.call('obstacle', 'checked_command', ctx.call('lane', 'lane_command', now),now,True)
    if ctx.state == 'PARKING' and now-ctx.slot_stamp > cfg['slot_memory_s']:
        return ctx.call('motion', 'stop', 'locked_slot_memory_expired')
    speed,physical = ctx.follower.command(ctx.pose,now)
    command = speed,model_to_command_steering(physical,cfg)
    if ctx.follower.done:
        if ctx.action == 'PARKING':
            if inside_slot(ctx.pose,ctx.slot,cfg) and abs(wrap(ctx.pose[2]-ctx.slot['pose'][2])) <= cfg['path_yaw_tolerance']:
                if ctx.pose_stamp > ctx.finish_pose_stamp:
                    ctx.finished_count += 1
                    ctx.finish_pose_stamp = ctx.pose_stamp
                if ctx.finished_count >= cfg['parking_settle_frames']:
                    ctx.state = 'FINISHED'
                return ctx.call('motion', 'stop', 'parking_settle')
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', 'parking_goal_not_inside_slot')
        ctx.state = 'REACQUIRE'
    else:
        return ctx.call('obstacle', 'checked_command', command,now,allow_bypass=False)

    return None
