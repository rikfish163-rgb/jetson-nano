"""obstacle module: explicit context input; coordinator applies returned updates."""
from __future__ import division
import math
import numpy as np

from robot.common.geometry import collision
from robot.common.geometry import distance
from robot.common.geometry import footprint
from robot.common.geometry import local
from robot.common.geometry import world
from robot.common.geometry import wrap, _isfinite
from robot.common.lidar_policy import lidar_guard_disabled_reason



FIELDS = ('action', 'cfg', 'follower', 'issued_steer', 'obstacle_check', 'parallel_parking', 'timed_bypass', 'timed_bypass_completed', 'timed_bypass_seen', 'lane', 'lane_stamp', 'lane_confidence',
 'parking_entry', 'timed_parking', 'pose', 'reason', 'relative_uturn', 'retry_at', 'scan', 'state', 'lane_recovery', 'exit_count', 'exit_stamp', 'lane_curve_lock', 'timed_bypass_candidate', 'timed_uturn')
CALLS = (('mission', 'start_follow'), ('motion', 'stop'), ('obstacle', 'scan_ready'),
 ('lane', 'search_command'), ('lane', 'forward_lane_ready'), ('lane', 'lane_valid'), ('lane', 'lane_tracking_ready'), ('lane', 'lane_command'), ('mission','resume_lane'), ('obstacle', 'checked_command'),
 ('obstacle', 'begin_timed_bypass'), ('obstacle', 'timed_bypass_tick'),
 ('obstacle', 'sweep_clear'))
OPERATIONS = ('scan_ready', 'sweep_clear', 'checked_command', 'begin_timed_bypass', 'timed_bypass_tick')


def _seen_points(ctx):
    """Return the bounded world-frame history owned by the coordinator."""
    return ctx.timed_bypass_seen


def _same_seen_entity(ctx, point):
    """Whether a world-frame point belongs to a bypass already completed."""
    radius = ctx.cfg.get('body_width', .24)/2 + .10
    for item in _seen_points(ctx):
        candidate = item.get('point') if isinstance(item, dict) else item
        if candidate is not None and distance(candidate, point) <= radius:
            return True
    return False


def _remember_trigger(ctx, now):
    """Persist the current trigger so protection can re-arm for new objects."""
    task = ctx.timed_bypass
    point = task.get('trigger_world')
    if point is None and task.get('trigger_point') is not None:
        point = world(ctx.pose, task['trigger_point'])
    if point is None:
        return
    seen = list(_seen_points(ctx))
    if not _same_seen_entity(ctx, point):
        seen.append(dict(point=tuple(point), stamp=now))
    # Keep a bounded history; each entry is a completed physical target.
    ctx.timed_bypass_seen = seen[-32:]


def begin_timed_bypass(ctx,now):
    if (not ctx.cfg.get('timed_bypass_enabled',False) or not ctx.cfg.get('lidar_enabled',True)
            or (ctx.cfg.get('straight_lidar_once',False) and ctx.timed_bypass_completed)
            or ctx.action is not None or ctx.state not in ('LANE','GAP','WAIT_OBSTACLE')
            or not ctx.call('obstacle','scan_ready',now)
            or not ctx.call('lane','lane_valid',now)):
        ctx.timed_bypass_candidate = None
        return None
    points=ctx.scan.motion_obstacles if ctx.scan.shape_filter else ctx.scan.obstacles
    hit=[(local(ctx.pose,p),p) for p in points]
    # Coordinates are rear-axle-relative, not clearance from the front bumper.
    trigger_distance=ctx.cfg.get('timed_bypass_trigger_distance_m',.50)
    hit=[(local_point,world_point) for local_point,world_point in hit
         if 0<local_point[0] and math.hypot(*local_point)<=trigger_distance
         and abs(local_point[1])<=ctx.cfg['body_width']/2+.10
         and not _same_seen_entity(ctx,world_point)]
    if not hit:
        ctx.timed_bypass_candidate = None
        return None
    trigger_point, trigger_world = min(hit,key=lambda pair:math.hypot(*pair[0]))
    candidate = ctx.timed_bypass_candidate
    stamp = ctx.scan.stamp
    if (candidate is None or not 0 <= stamp-candidate['stamp'] <= ctx.cfg.get('lidar_timeout',.8)
            or distance(candidate['point'],trigger_world) > .10):
        candidate = dict(point=trigger_world,stamp=stamp,since=stamp,frames=1)
        ctx.timed_bypass_candidate = candidate
    elif stamp > candidate['stamp']:
        candidate.update(point=trigger_world,stamp=stamp,frames=candidate['frames']+1)
    if candidate['frames'] < 3 or stamp-candidate['since'] < .15:
        return None
    ctx.timed_bypass_candidate = None
    ctx.lane_curve_lock = None
    phase='SETTLE_LEFT' if ctx.cfg.get('timed_bypass_settle_s',0.)>0 else 'LEFT'
    ctx.timed_bypass=dict(phase=phase,elapsed_s=0.,last=None,
                          trigger_point=trigger_point,trigger_world=trigger_world)
    ctx.exit_count,ctx.exit_stamp=0,-1.
    ctx.state='TIMED_BYPASS';ctx.action='BYPASS'
    return ctx.call('obstacle','timed_bypass_tick',now)


def timed_bypass_tick(ctx, now):
    """Fixed LEFT -> RIGHT -> RETURN_LEFT, then normal lane control."""
    task = ctx.timed_bypass
    phases = {
        'SETTLE_LEFT': ('timed_bypass_settle_s', 0., 'LEFT'),
        'LEFT': ('timed_bypass_left_s', 1.5, 'RIGHT'),
        'RIGHT': ('timed_bypass_right_s', 3., 'RETURN_LEFT'),
        'RETURN_LEFT': ('timed_bypass_return_left_s', 1., 'RESUME'),
    }
    if task['phase'] not in phases:
        return ctx.call('motion', 'stop', 'bypass_invalid_phase')
    if task['phase'] == 'SETTLE_LEFT':
        if task['last'] is None:
            task['elapsed_s'] = 0.
        if not ctx.call('obstacle', 'scan_ready', now):
            return ctx.call('motion', 'stop', 'scan_missing_or_stale')
    dt = 0. if task['last'] is None else now-task['last']
    if not 0 <= dt <= .5:
        return ctx.call('motion', 'stop', 'bypass_control_clock_gap')
    task['last'] = now
    task['elapsed_s'] += dt
    duration_key, default_duration, next_phase = phases[task['phase']]
    if task['elapsed_s'] >= ctx.cfg.get(duration_key, default_duration)-1e-9:
        # Each new command starts its own timer; never skip an arc on a late tick.
        task['phase'], task['elapsed_s'] = next_phase, 0.
    if task['phase'] == 'RESUME':
        _remember_trigger(ctx, now)
        ctx.timed_bypass_completed = True
        ctx.timed_bypass = None
        ctx.lane_recovery = None
        ctx.call('mission', 'resume_lane')
        command = ctx.call('lane', 'lane_command', now)
        result = ctx.call('obstacle', 'checked_command', command, now, False)
        if result[0] > 0:
            ctx.reason = 'bypass_timed_lane_handoff'
        return result
    phase = ('left' if task['phase'] == 'SETTLE_LEFT'
             else task['phase'].lower())
    speed = (0 if task['phase'] == 'SETTLE_LEFT' else
             ctx.cfg.get('timed_bypass_'+phase+'_speed_raw',
                         ctx.cfg.get('timed_bypass_speed_raw', 30)))
    raw = ctx.cfg.get('timed_bypass_'+phase+'_steering_raw',
                      -22 if phase == 'right' else 22)
    steer = (float(raw)/ctx.cfg['steering_raw_limit'] *
             ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer']) /
             ctx.cfg['steering_sign'])
    command = (speed, steer)
    result = ctx.call('obstacle', 'checked_command', command, now, False)
    if result == command:
        ctx.reason = 'bypass_'+task['phase'].lower()
    return result


def scan_ready(ctx, now):
    return (ctx.scan is not None and
            0 <= now-getattr(ctx.scan,'completed_stamp',ctx.scan.stamp) <= ctx.cfg.get('lidar_timeout',ctx.cfg['sensor_timeout']) and
            ctx.scan.valid_rays >= ctx.cfg['lidar']['min_rays'])


def sweep_clear(ctx, path, require_coverage=True, report=False):
    if report:
        ctx.obstacle_check = dict(kind='clear',scan_stamp=ctx.scan.stamp if ctx.scan else None)
    if ctx.scan is None:
        if report:
            ctx.obstacle_check['kind'] = 'missing_scan'
        return False
    filtered = ctx.scan.shape_filter and ctx.action != 'PARKING'
    # Shape filtering selects bypass targets; every raw return still protects
    # the imminent vehicle footprint, including non-target walls or objects.
    obstacles = ctx.scan.obstacles
    # Convert once per sweep, not once per pose. Do not cache mutable scans.
    obstacle_array = np.asarray(obstacles,dtype=float).reshape(-1,2)
    if filtered:
        if report:
            ctx.obstacle_check.update(mode='raw_collision_guard',
                target_filter=ctx.scan.shape_mode,round_clusters=ctx.scan.round_clusters)
    for p in path:
        if collision(p, obstacle_array,ctx.cfg):
            if report:
                hit = next(q for q in obstacles if collision(p,[q],ctx.cfg))
                # Spatial support is an observation count, not a probability.
                support = sum(distance(q,hit) <= .06 for q in ctx.scan.obstacles)
                ctx.obstacle_check.update(kind='obstacle',point_local=local(ctx.pose,hit),
                    support_points=support,support_radius_m=.06,
                    ray=ctx.scan.evidence(hit))
                if filtered:
                    ctx.obstacle_check['evidence_type'] = 'raw_scan_points'
            return False
        if require_coverage:
            for q in footprint(p,ctx.cfg):
                x,y = local(ctx.pose,q)
                # Lidar cannot observe inside its own current vehicle footprint.
                if (-ctx.cfg['rear_overhang'] <= x <= ctx.cfg['wheelbase']+ctx.cfg['front_overhang']
                        and abs(y) <= ctx.cfg['body_width']/2):
                    continue
                classify = getattr(ctx.scan, 'classify', None)
                if classify is None or classify(q) != 'FREE':
                    if report:
                        ctx.obstacle_check.update(kind='unknown',point_local=(x,y),
                            ray=ctx.scan.evidence(q))
                    return False
    return True


def checked_command(ctx, command, now, allow_bypass):
    speed, steer = command
    disabled_reason = lidar_guard_disabled_reason(
        ctx.cfg, ctx.state, ctx.action, ctx.timed_bypass_completed)
    if disabled_reason is not None:
        ctx.obstacle_check = dict(kind='disabled',reason=disabled_reason)
        ctx.reason = 'tracking_'+ctx.state.lower() if speed else ctx.reason
        ctx.issued_steer = steer
        return command
    if speed == 0:
        ctx.issued_steer = steer
        return command
    if allow_bypass and ctx.timed_bypass is None:
        bypass=ctx.call('obstacle','begin_timed_bypass',now)
        if bypass is not None:return bypass
    if ctx.action is None and not ctx.call('obstacle', 'scan_ready', now):
        ctx.obstacle_check = dict(kind='unknown',reason='scan_missing_or_stale')
        ctx.reason = 'lidar_unavailable_lane_search'
        ctx.issued_steer = steer
        return min(speed,12) if speed > 0 else speed,steer
    # Lidar locates the bypass trigger; predicted sweeps no longer veto motion.
    ctx.obstacle_check = dict(kind='disabled',reason='prediction_removed')
    ctx.reason = 'tracking_'+ctx.state.lower()
    ctx.issued_steer = steer
    return speed,steer
