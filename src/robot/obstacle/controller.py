"""obstacle module: explicit context input; coordinator applies returned updates."""
from __future__ import division
import math
import numpy as np

from robot.common.geometry import bicycle
from robot.common.geometry import collision
from robot.common.geometry import distance
from robot.common.geometry import footprint
from robot.common.geometry import in_box
from robot.common.geometry import local
from robot.common.geometry import world
from robot.common.geometry import wrap, _isfinite
from robot.obstacle.planner import bypass_path
from robot.common.contracts import encode_command
from robot.uturn.calibration import trial_active, raw_angle
from robot.motion import calibration as chassis
from robot.common.lidar_policy import lidar_guard_disabled_reason



FIELDS = ('action', 'cfg', 'follower', 'issued_steer', 'obstacle_check', 'parallel_parking', 'timed_bypass', 'timed_bypass_completed', 'timed_bypass_seen', 'lane', 'lane_stamp', 'lane_confidence',
 'parking_entry', 'timed_parking', 'pose', 'reason', 'relative_uturn', 'retry_at', 'scan', 'state', 'lane_recovery', 'exit_count', 'exit_stamp', 'lane_curve_lock', 'timed_bypass_candidate', 'timed_uturn')
CALLS = (('mission', 'start_follow'), ('motion', 'stop'), ('obstacle', 'scan_ready'),
 ('lane', 'forward_lane_ready'), ('lane', 'lane_valid'), ('lane', 'lane_tracking_ready'), ('lane', 'lane_command'), ('mission','resume_lane'), ('obstacle', 'checked_command'),
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


def _bypass_lane_exit(ctx,task,now):
    """Confirm a capturable lane across distinct, motion-compensated images."""
    def reject(reason):
        task['lane_exit']=dict(valid=False,reason=reason,frames=0)
        return False
    if not ctx.call('obstacle','scan_ready',now):
        return reject('scan_stale')
    trigger=task.get('trigger_world')
    if (trigger is not None and local(ctx.pose,trigger)[0] >=
            -ctx.cfg['rear_overhang']-ctx.cfg['obstacle_margin']):
        return reject('obstacle_not_passed')
    if not ctx.call('lane','lane_tracking_ready',now):
        return reject('lane_not_trackable')
    points=sorted(local(ctx.pose,p) for p in ctx.lane)
    if not all(_isfinite(v) for p in points for v in p):
        return reject('nonfinite_path')
    points=[p for p in points if .05 < p[0] <= 1.2]
    if len(points)<2 or points[0][0]>.85 or points[-1][0]-points[0][0]<.08-1e-9:
        return reject('insufficient_near_path')
    floor=ctx.cfg['lane_min_confidence'] if len(points)>=3 else .7*ctx.cfg['lane_min_confidence']
    if ctx.lane_confidence < floor or any(abs(y)>.45 for x,y in points):
        return reject('low_confidence_or_far_lane')
    angles=[]
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1]
        if not .015<=dx<=.35:
            return reject('disjoint_path')
        angles.append(math.atan2(dy,dx))
    if (any(abs(a)>math.radians(35) for a in angles) or
            any(abs(b-a)>math.radians(35) for a,b in zip(angles,angles[1:]))):
        return reject('path_heading_or_bend')
    mx=sum(x for x,y in points)/len(points)
    my=sum(y for x,y in points)/len(points)
    xx=sum((x-mx)**2 for x,y in points)
    slope=sum((x-mx)*(y-my) for x,y in points)/xx
    intercept=my-slope*mx
    norm=math.hypot(1.,slope)
    if max(abs(y-slope*x-intercept)/norm for x,y in points)>.06:
        return reject('inconsistent_centers')
    heading=wrap(ctx.pose[2]+math.atan(slope))
    evidence=task.get('lane_exit')
    consistent=(evidence is not None and evidence.get('valid',False) and
                0<=ctx.lane_stamp-evidence['stamp']<=ctx.cfg['sensor_timeout'])
    if consistent:
        previous=local(ctx.pose,evidence['point_world'])
        consistent=(abs(previous[1]-slope*previous[0]-intercept)/norm<=.12 and
                    abs(wrap(heading-evidence['heading_world']))<=math.radians(15))
    if not consistent:
        evidence=dict(valid=True,reason='confirming_lane',stamp=-1.,
                      since=ctx.lane_stamp,frames=0)
        task['lane_exit']=evidence
    if ctx.lane_stamp>evidence['stamp']:
        x=max(points[0][0],min(.70,points[-1][0]))
        evidence.update(stamp=ctx.lane_stamp,frames=evidence['frames']+1,
            point_world=world(ctx.pose,(x,slope*x+intercept)),heading_world=heading,
            near_lateral_m=points[0][1],heading_deg=math.degrees(math.atan(slope)))
    confirmed=(evidence['frames']>=3 and
               ctx.lane_stamp-evidence['since']>=.15-1e-9)
    evidence['reason']='lane_confirmed' if confirmed else 'confirming_lane'
    return confirmed


def timed_bypass_tick(ctx,now):
    task=ctx.timed_bypass
    right_minimum=ctx.cfg.get('timed_bypass_right_s',4.5)
    right_limit=ctx.cfg.get('timed_bypass_right_max_s',max(6.,right_minimum))
    if task['phase']=='SETTLE_LEFT':
        if task['last'] is None:
            task['elapsed_s']=0.  # A stop centers the command; settle again.
        if not ctx.call('obstacle','scan_ready',now):
            return ctx.call('motion','stop','scan_missing_or_stale')
    dt=0. if task['last'] is None else now-task['last']
    if not 0<=dt<=.5:
        return ctx.call('motion','stop','bypass_control_clock_gap')
    task['last']=now
    if task['phase']!='REACQUIRE':
        task['elapsed_s']+=dt
        duration_s=(ctx.cfg.get('timed_bypass_settle_s',0.) if task['phase']=='SETTLE_LEFT'
                    else ctx.cfg.get('timed_bypass_left_s',2.) if task['phase']=='LEFT' else right_limit)
        # Stop the arc as soon as a capturable lane appears after the minimum.
        # Confirm it while stopped so full lock cannot turn past that lane.
        lane_candidate=False
        if task['phase']=='RIGHT' and task['elapsed_s']>=right_minimum-1e-9:
            _bypass_lane_exit(ctx,task,now)
            lane_candidate=task['lane_exit']['valid']
        if lane_candidate or task['elapsed_s']>=duration_s-1e-9:
            if task['phase']=='LEFT':
                if not ctx.call('obstacle','scan_ready',now):
                    return ctx.call('motion','stop','scan_missing_or_stale')
                # After the configured left duration, before reversing the
                # wheels, check the first two seconds of the right arc:
                # the 8 cm imminent guard alone notices the cone too late.
                speed=ctx.cfg.get('timed_bypass_speed_raw',ctx.cfg['speed_raw']['action'])
                horizon=max(.08,min(.40,2*speed*chassis.speed_gain(ctx.cfg,speed)))
                steer=ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
                raw = encode_command(speed,-steer,ctx.cfg,0)['steering_raw']
                angle = chassis.raw_angle(ctx.cfg,speed,raw)
                obstacles=np.asarray(ctx.scan.obstacles,dtype=float).reshape(-1,2)
                path=[bicycle(ctx.pose,horizon*i/32,angle,ctx.cfg['wheelbase'])
                      for i in range(1,33)]
                blocked=any(collision(p,obstacles,ctx.cfg) for p in path)
                task['right_entry']=dict(clear=not blocked,horizon_m=horizon,
                    scan_stamp=ctx.scan.stamp,extra_left_s=task['elapsed_s']-duration_s)
                if blocked:
                    if task['elapsed_s']>=duration_s+1.5-1e-9:
                        return ctx.call('motion','stop','bypass_right_entry_blocked')
                    # Continue the original left arc only when its imminent
                    # raw-point and unknown-space checks also permit motion.
                    command=(speed,steer)
                    result=ctx.call('obstacle','checked_command',command,now,False)
                    if result==command:
                        ctx.reason='bypass_extend_left_for_clearance'
                    return result
            if task['phase']=='RIGHT':
                task['right_completed_s']=task['elapsed_s']
            task['phase']=('LEFT' if task['phase']=='SETTLE_LEFT' else
                           'RIGHT' if task['phase']=='LEFT' else 'REACQUIRE')
            task['elapsed_s']=0.
    if task['phase']=='REACQUIRE':
        ctx.lane_recovery=None
        # The same confirmation applies while stopped at the search limit.
        # An offset lane may take over; it need not already be centered.
        if not _bypass_lane_exit(ctx,task,now):
            return ctx.call('motion','stop','bypass_wait_exit_lane')
        _remember_trigger(ctx, now)
        ctx.timed_bypass_completed=True
        ctx.timed_bypass=None
        ctx.call('mission','resume_lane')
        speed,steer=ctx.call('lane','lane_command',now)
        speed=min(speed,ctx.cfg.get('timed_bypass_speed_raw',ctx.cfg['speed_raw']['action']))
        result=ctx.call('obstacle','checked_command',(speed,steer),now,False)
        if result[0]>0:
            ctx.reason='bypass_visual_lane_handoff'
        return result
    steer=ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    if task['phase']=='SETTLE_LEFT':
        ctx.issued_steer=steer
        ctx.reason='bypass_settle_left'
        return 0,steer
    command=(ctx.cfg.get('timed_bypass_speed_raw',ctx.cfg['speed_raw']['action']),
             steer if task['phase']=='LEFT' else -steer)
    result=ctx.call('obstacle','checked_command',command,now,False)
    if result==command:
        ctx.reason=('bypass_left_%gs'%ctx.cfg.get('timed_bypass_left_s',2.)
                    if task['phase']=='LEFT' else 'bypass_right_find_lane'
                    if task['elapsed_s']>=right_minimum-1e-9 else 'bypass_right_min_%gs'%right_minimum)
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
    if not ctx.call('obstacle', 'scan_ready', now):
        ctx.obstacle_check = dict(kind='unknown',reason='scan_missing_or_stale',
                                  valid_rays=ctx.scan.valid_rays if ctx.scan else 0)
        return ctx.call('motion', 'stop', 'scan_missing_or_stale')
    d = 1 if speed > 0 else -1
    horizon = (ctx.cfg['obstacle_preview_distance'] if allow_bypass and ctx.cfg['bypass_enabled']
               else ctx.cfg['obstacle_stop_distance'])
    active_timed_bypass = (ctx.state == 'TIMED_BYPASS' and
                           ctx.action == 'BYPASS' and
                           ctx.timed_bypass is not None)
    approaching_timed_trigger=(allow_bypass and speed>0 and ctx.action is None and
        ctx.state in ('LANE','GAP','WAIT_OBSTACLE') and ctx.cfg.get('timed_bypass_enabled',False))
    if approaching_timed_trigger or active_timed_bypass:
        # Keep a short near-field guard both immediately before the trigger
        # and throughout the fixed maneuver. The trigger itself is farther
        # ahead; projecting the full stop horizon would veto every bypass.
        horizon=min(horizon,.08)
    if ctx.state == 'PARKING' and ctx.follower is not None:
        # Do not project beyond a planned stop/cusp into the bay's back wall.
        # Retain a short stopping sweep; obstacles within it still stop motion.
        endpoint = ctx.follower.path[ctx.follower.segment_end]
        horizon = min(horizon,max(.04,distance(ctx.pose,endpoint)))
    scene_task = (ctx.relative_uturn if ctx.state == 'UTURN' else
                  ctx.parallel_parking if ctx.state == 'PARALLEL_PARKING' else None)
    if scene_task is not None and scene_task.follower is not None:
        endpoint = scene_task.follower.path[scene_task.follower.segment_end]
        remaining = distance(scene_task.scene['pose'],endpoint)
        # Distance is computed in the measured frame; the scan sweep below
        # remains in the scan's existing command-local frame. Never project
        # the obstacle guard through the planned stop into a bay end wall.
        horizon = min(horizon,max(.04,remaining))
    raw = encode_command(speed,steer,ctx.cfg,0)['steering_raw']
    physical = chassis.raw_angle(ctx.cfg,speed,raw)
    if trial_active(ctx.cfg,ctx.state):
        physical = raw_angle(ctx.cfg,speed,raw)
        if ctx.timed_uturn is not None:
            remaining = ctx.timed_uturn.remaining_distance(ctx.cfg)
            if remaining is not None:
                horizon = min(horizon,max(.04,remaining))
    if ctx.parking_entry is not None and ctx.state == 'PARKING':
        # Dedicated entry emits command-space angles; collision sweeps use
        # physical bicycle angles, exactly as applied feedback integration.
        horizon = min(horizon,.08)
        remaining = ctx.parking_entry.debug.get('remaining_m')
        if remaining is not None and remaining>0:
            horizon = min(horizon,max(.02,remaining))
    if (ctx.state == 'TIMED_PARKING' and ctx.timed_parking is not None and
            ctx.timed_parking.phase == 'REVERSE'):
        # Match the existing parking-entry near-field sweep. Adjacent bay
        # walls must not veto an otherwise clear short parking step.
        horizon = min(horizon,.08)
    path = [bicycle(ctx.pose,d*horizon*i/16,physical,ctx.cfg['wheelbase']) for i in range(1,17)]
    if ctx.call('obstacle', 'sweep_clear', path,ctx.cfg['lidar']['unknown_is_obstacle'],report=True):
        if active_timed_bypass:
            ctx.timed_bypass.pop('side_clearance',None)
        ctx.reason = 'tracking_'+ctx.state.lower()
        ctx.issued_steer = steer
        return speed,steer
    if active_timed_bypass and speed > 0 and ctx.obstacle_check['kind'] == 'obstacle':
        hit = ctx.obstacle_check.get('point_local')
        # A side point entering the inflated turning sweep can still leave
        # room for a checked straight/shallower sweep. Never bypass a point
        # in the current actual body or unresolved lidar coverage.
        body_cfg = dict(ctx.cfg,obstacle_margin=0.)
        side = (hit is not None and abs(hit[1]) > ctx.cfg['body_width']/2+.01 and
                -ctx.cfg['rear_overhang'] <= hit[0] <= ctx.cfg['wheelbase']+ctx.cfg['front_overhang'])
        if side and not collision(ctx.pose,ctx.scan.obstacles,body_cfg):
            task = ctx.timed_bypass
            clearance = task.get('side_clearance')
            if clearance is None:
                clearance = dict(since=now,origin=ctx.pose)
                task['side_clearance'] = clearance
            # The recorded point starts 22.6 cm ahead of the rear axle. A
            # 15 cm cap would stop before it clears the rotating rear sweep.
            if now-clearance['since'] <= 1.5 and distance(ctx.pose,clearance['origin']) <= .25:
                for alternative in (steer*.5,0.,-steer*.5):
                    raw = encode_command(speed,alternative,ctx.cfg,0)['steering_raw']
                    angle = chassis.raw_angle(ctx.cfg,speed,raw)
                    safe_path = [bicycle(ctx.pose,horizon*i/16,angle,ctx.cfg['wheelbase'])
                                 for i in range(1,17)]
                    if ctx.call('obstacle','sweep_clear',safe_path,True):
                        task['last'] = None  # Alternate motion is not timed full-lock turning.
                        ctx.obstacle_check.update(kind='side_clearance',
                            requested_steer=steer,applied_steer=alternative)
                        ctx.reason = 'bypass_side_clearance'
                        ctx.issued_steer = alternative
                        return speed,alternative
            else:
                ctx.obstacle_check['clearance_limit']=dict(
                    elapsed_s=now-clearance['since'],
                    distance_m=distance(ctx.pose,clearance['origin']))
    if allow_bypass:
        ctx.state = 'WAIT_OBSTACLE'
        if now >= ctx.retry_at and ctx.cfg['bypass_enabled']:
            ctx.retry_at = now+ctx.cfg['bypass_retry_s']
            bypass = bypass_path(ctx.pose,ctx.cfg)
            zones = ctx.cfg['bypass_zones']
            legal = bool(bypass) and all(any(in_box(q,z) for z in zones)
                                         for p in bypass for q in footprint(p,ctx.cfg))
            # The return lane behind a cone can be occluded from this
            # viewpoint. Plan around known obstacles, then require fresh
            # FREE coverage of EACH imminent sweep while executing.
            if legal and ctx.call('obstacle', 'sweep_clear', bypass,False):
                ctx.call('mission', 'start_follow', bypass,'BYPASS',now)
                return ctx.call('motion', 'stop', 'bypass_planned')
    return ctx.call('motion', 'stop', 'lidar_obstacle_in_sweep' if ctx.obstacle_check['kind'] == 'obstacle'
                     else 'lidar_unknown_in_sweep')
