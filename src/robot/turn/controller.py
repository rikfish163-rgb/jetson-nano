"""turn module: explicit context input; coordinator applies returned updates."""
from __future__ import division

import math
from robot.common.geometry import distance
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.turn.planner import intersection_path
from robot.turn.geometry import arc_exit_reached
from robot.turn.geometry import exit_candidate
from robot.camera.alignment import steering_for_heading
from robot.common.contracts import model_to_command_steering



FIELDS = ('action', 'action_source', 'action_started', 'cfg', 'completed_at_pose',
 'consumed_marker', 'exit_count', 'exit_pose', 'exit_reason', 'exit_stamp',
 'follow_left_boundary', 'front_blue_lines', 'front_blue_image_lines', 'front_marker_stamp', 'lane', 'lane_source',
 'lane_stamp', 'last_completed_action', 'left_boundary_stamp', 'left_reference', 'marker',
 'parking_marker', 'pose', 'reason', 'left_lock', 'right_lock', 'right_tail_origin', 'state',
 'straight_search', 'blue_approach', 'next_direction')
CALLS = (('lane', 'lane_command'), ('lane', 'lane_valid'), ('lane', 'lane_tracking_ready'), ('lane', 'left_exit_line'),
 ('lane', 'left_reference_command'), ('mission', 'resume_lane'), ('mission', 'dispatch'),
 ('mission', 'start_follow'), ('mission','begin_blue_action'), ('turn','exit_lane_confirmed'),
 ('motion', 'current_steering'), ('motion', 'stop'),
 ('obstacle', 'checked_command'), ('turn', 'front_straight_reference'),
 ('turn', 'right_blue_exit_tick'), ('turn', 'right_center_exit'),
 ('turn', 'straight_heading_command'), ('turn', 'handoff_lane_confirmed'))
OPERATIONS = ('front_straight_reference', 'straight_heading_command', 'straight_search_tick', 'blue_approach_tick',
 'right_center_exit', 'right_blue_exit_tick', 'left_lock_tick', 'right_lock_tick', 'direction_exit_reached',
 'exit_lane_confirmed', 'handoff_lane_confirmed')

def front_straight_reference(ctx, now, target=None):
    """Blue yaw is the measured normal, not the stripe tangent."""
    if not 0 <= now-ctx.front_marker_stamp <= ctx.cfg['sensor_timeout']:
        return None
    candidates = []
    for line in ctx.front_blue_lines:
        x,y = local(ctx.pose,line['point'])
        heading = (line['yaw']-ctx.pose[2]+math.pi/2) % math.pi-math.pi/2
        if (not 0 < x <= ctx.cfg.get('straight_align_distance',1.6) or
                abs(y) > ctx.cfg['lane_width'] or
                abs(heading) > math.radians(60) or
                line['length'] < ctx.cfg['blue']['long_min']):
            continue
        if target is not None and distance(line['point'],target) > .20:
            continue
        candidates.append((x,dict(point=line['point'],yaw=ctx.pose[2]+heading,
                                  heading=heading,stamp=ctx.front_marker_stamp)))
    return min(candidates,key=lambda item:item[0])[1] if candidates else None


def straight_heading_command(ctx, heading, now):
    command = ctx.call('obstacle', 'checked_command', (ctx.cfg['speed_raw']['gap'],
        steering_for_heading(heading,ctx.cfg)),now,False)
    ctx.lane_source = 'straight_blue_normal'
    if command[0]: ctx.reason = 'straight_align_blue'
    return command


def right_blue_reference(ctx, now):
    """Build a parallel target to the left of the right longitudinal stripe."""
    if not 0 <= now-ctx.front_marker_stamp <= ctx.cfg['sensor_timeout']:
        return None
    candidates = []
    for line in ctx.front_blue_lines:
        x,y = local(ctx.pose,line['point'])
        # GroundDetector publishes the stripe NORMAL. Convert to its forward
        # tangent modulo pi so reversed contour endpoints give the same angle.
        heading = (line['yaw']-ctx.pose[2]) % math.pi-math.pi/2
        if (not .05 < x <= ctx.cfg.get('straight_align_distance',1.6) or
                not -ctx.cfg['lane_width'] <= y <= -.08 or
                abs(heading) > math.radians(35) or
                line['length'] < ctx.cfg['blue']['long_min']):
            continue
        # Signed normal distance is invariant when the visible segment center
        # moves along the same stripe. y alone is not the car-to-line distance.
        signed_distance=-x*math.sin(heading)+y*math.cos(heading)
        if signed_distance >= 0:
            continue
        offset=ctx.cfg.get('straight_right_blue_offset_m',.30)
        lookahead=ctx.cfg['lookahead']
        error=signed_distance+offset
        target_y=(error+lookahead*math.sin(heading))/math.cos(heading)
        candidates.append((-signed_distance, x, dict(point=[x,y], heading_deg=math.degrees(heading),
            heading=heading, stamp=ctx.front_marker_stamp,distance_m=-signed_distance,
            target_offset_m=offset,lateral_error_m=error,target=[lookahead,target_y])))
    return min(candidates,key=lambda item:item[:2])[2] if candidates else None


def blue_approach_tick(ctx, now):
    """Stop STRAIGHT and UTURN at the measured line before starting an action."""
    s = ctx.blue_approach
    if s.get('image_timed',False) and ctx.action in ('LEFT','RIGHT','STRAIGHT','UTURN'):
        from robot.turn.blue_timed import production_tick
        return production_tick(ctx,now)
    if s['phase'] == 'STOP':
        if now < s['stop_until']:
            return ctx.call('motion','stop','blue_stop_wait_'+ctx.action.lower())
        if not 0 <= now-ctx.front_marker_stamp <= ctx.cfg.get('ground_timeout',1.25):
            return ctx.call('motion','stop','blue_stop_front_stale')
        if (ctx.action == 'UTURN' and
                abs(wrap(s['yaw']-ctx.pose[2])) > math.radians(ctx.cfg['uturn_align_heading_deg'])):
            return ctx.call('motion','stop','uturn_blue_heading_not_aligned')
        return ctx.call('mission','begin_blue_action',now)
    s.update(point_local=local(ctx.pose,s['point']),
             observation_age_s=now-s['observed_stamp'])
    if not 0 <= now-ctx.front_marker_stamp <= ctx.cfg.get('ground_timeout',1.25):
        s['align_frames']=0
        return ctx.call('motion','stop','blue_approach_front_stale')
    if now-s['started'] > ctx.cfg['action_timeout']:
        return ctx.call('motion','stop','blue_approach_timeout')
    if s['phase']=='STOP_LINE':
        # Project the required vehicle reference onto the transverse plane.
        # UTURN and timed LEFT/RIGHT use the front axle; STRAIGHT uses the bumper.
        x,y=local(ctx.pose,s['point'])
        heading=wrap(s['yaw']-ctx.pose[2])
        plane_distance=x*math.cos(heading)+y*math.sin(heading)
        if (ctx.action == 'UTURN' or (ctx.action == 'LEFT' and ctx.cfg.get('left_timed_enabled',False)) or
                (ctx.action == 'RIGHT' and ctx.cfg.get('right_timed_enabled',False))):
            remaining=plane_distance-ctx.cfg['wheelbase']*math.cos(heading)
            s['stop_reference']='front_axle'
        else:
            front=ctx.cfg['wheelbase']+ctx.cfg['front_overhang']
            body_extent=front*math.cos(heading)+ctx.cfg['body_width']/2*abs(math.sin(heading))
            remaining=plane_distance-body_extent-.02
            s['stop_reference']='front_bumper'
        s.update(remaining_m=remaining,heading_error_deg=math.degrees(heading),
                 stop_line_distance_m=plane_distance)
        if remaining<=1e-9:
            if (ctx.action == 'UTURN' and
                    abs(heading) > math.radians(ctx.cfg['uturn_align_heading_deg'])):
                return ctx.call('motion','stop','uturn_blue_heading_not_aligned')
            s.update(phase='STOP',stop_until=now+ctx.cfg.get('intersection_wait_s',1.),stop_pose=ctx.pose)
            ctx.state='BLUE_STOP'
            return ctx.call('motion','stop','blue_stop_wait_'+ctx.action.lower())
        reference=right_blue_reference(ctx,now) if ctx.action == 'STRAIGHT' else None
        s['right_blue']=reference
        steer=right_blue_steering(ctx,reference) if reference else steering_for_heading(heading,ctx.cfg)
        ctx.lane_source='straight_right_blue' if reference else 'blue_stop_normal'
        result=ctx.call('obstacle','checked_command',(ctx.cfg['straight_speed_raw'],steer),now,False)
        if result[0]:ctx.reason=ctx.action.lower()+'_approach_stop_line'
        return result
    if s['phase'] == 'ALIGN':
        reference=ctx.call('turn','front_straight_reference',now,s['point'])
        measured=(reference is not None and reference['stamp'] == s['observed_stamp'])
        # Dispatch already confirmed this blue event and its normal. Once the
        # stripe leaves the image, finish against the retained normal using
        # pose feedback; an empty image must not cancel an accepted action.
        # Rejected/new stripes cannot replace the accepted target here.
        target_yaw=reference['yaw'] if measured else s['yaw']
        heading=wrap(target_yaw-ctx.pose[2])
        s['alignment_source']='visual' if measured else 'latched_pose'
        s['heading_error_deg']=math.degrees(heading)
        aligned=abs(heading) <= math.radians(ctx.cfg['straight_align_tolerance_deg'])
        if measured and reference['stamp'] > s['align_stamp']:
            if reference['stamp']-s['align_stamp'] > ctx.cfg['sensor_timeout']:
                s['align_frames']=0
            s['align_stamp']=reference['stamp']
            s['align_frames']=s['align_frames']+1 if aligned else 0
        elapsed=max(0.,now-s['started'])
        duration=ctx.cfg.get('blue_align_duration_s',1.)
        s.update(alignment_elapsed_s=elapsed,alignment_duration_s=duration)
        if elapsed < duration-1e-9:
            ctx.lane_source='blue_normal_alignment'
            command=(ctx.cfg['speed_raw']['gap'],
                     0. if aligned else steering_for_heading(heading,ctx.cfg))
            result=ctx.call('obstacle','checked_command',command,now,False)
            if result[0]:
                ctx.reason='blue_align_heading' if measured else 'blue_align_latched_heading'
            return result
        # The one-second alignment phase has a fixed deadline. Continue the
        # entry distance even if its measured heading has not converged.
        origin=tuple(ctx.pose)
        s.update(phase='ADVANCE',advance_origin=origin,aligned_at=now,
                 alignment_completion='timed',heading_within_tolerance=aligned,
                 advance_distance_m=ctx.cfg['blue_aligned_advance_m'],
                 advance_travelled_m=0.)
    travelled=local(s['advance_origin'],ctx.pose)[0]
    remaining=s['advance_distance_m']-travelled
    s.update(advance_travelled_m=travelled,remaining_m=remaining)
    if remaining <= 1e-9:
        s.update(phase='STOP',stop_until=now+ctx.cfg.get('intersection_wait_s',1.),stop_pose=ctx.pose)
        ctx.state='BLUE_STOP'
        return ctx.call('motion','stop','blue_stop_wait_'+ctx.action.lower())
    # A STRAIGHT entry may still have residual yaw after the fixed one-second
    # alignment window. Keep correcting toward the accepted blue normal while
    # advancing; otherwise the car travels 0.36 m along an unaligned heading.
    # Other maneuvers retain their existing entry-heading hold.
    target_yaw=s['yaw'] if ctx.action == 'STRAIGHT' else s['advance_origin'][2]
    heading=wrap(target_yaw-ctx.pose[2])
    s['heading_error_deg']=math.degrees(heading)
    steer=steering_for_heading(heading,ctx.cfg) if abs(heading)>math.radians(2) else 0.
    ctx.lane_source='blue_aligned_advance'
    result=ctx.call('obstacle','checked_command',(ctx.cfg['straight_speed_raw'],steer),now,False)
    if result[0]:ctx.reason='blue_advance_distance'
    return result


def straight_search_tick(ctx, now):
    """Drive the independent straight segment, then return control to lane."""
    s = ctx.straight_search
    travelled = local(s['origin'],ctx.pose)[0]
    minimum = ctx.cfg.get('straight_distance',1.25)
    s.update(elapsed_s=now-s['started'],travelled_m=travelled,
             minimum_distance_m=minimum,
             phase='STRAIGHT_DISTANCE')
    if travelled >= minimum:
        # Every blue-triggered STRAIGHT completes its distance before handing
        # back to lane. A queued sign cannot preempt it; the next lane tick
        # may dispatch the next confirmed blue event through the normal path.
        ctx.call('mission', 'resume_lane')
        return ctx.call('obstacle', 'checked_command', ctx.call('lane', 'lane_command', now),now,True)
    if now-s['started'] >= s['timeout_s']:
        return ctx.call('motion','stop','straight_motion_timeout')
    if not 0 <= now-ctx.front_marker_stamp <= ctx.cfg.get('ground_timeout',1.25) and not 0 <= now-ctx.lane_stamp <= ctx.cfg['sensor_timeout']:
        return ctx.call('motion','stop','straight_front_stale')
    fixed = ctx.action_source == 'sign' and 'straight_steering_raw' in ctx.cfg
    reference = None
    if fixed:
        raw = ctx.cfg['straight_steering_raw']
        scale = ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
        steer = float(raw)/ctx.cfg['steering_raw_limit']*scale/ctx.cfg['steering_sign']
        ctx.lane_source = 'straight_fixed_steer'
    else:
        # Legacy/right-exit straight retains its independent blue reference.
        reference = right_blue_reference(ctx,now)
        steer = right_blue_steering(ctx,reference) if reference is not None else 0.
        ctx.lane_source = 'straight_right_blue' if reference is not None else 'straight_zero_steer'
    s['right_blue'] = reference
    command = ctx.call('obstacle', 'checked_command',
        (s['speed_raw'],steer),now,False)
    if command[0]:
        ctx.reason = ('straight_fixed_steer' if fixed else
                      'straight_align_right_blue' if reference is not None else 'straight_continue')
    return command


def right_blue_steering(ctx, reference):
    x,y=reference['target']
    physical=math.atan2(2*ctx.cfg['wheelbase']*y,max(.0025,x*x+y*y))
    limit=6.0/ctx.cfg['steering_raw_limit']*ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    return max(-limit,min(limit,model_to_command_steering(physical,ctx.cfg)))


def right_center_exit(ctx, now, progress):
    """Allow the normal lane path when a single straight left edge is unavailable."""
    r = ctx.right_lock
    if (progress < ctx.cfg.get('right_lock_min_angle_deg',55) or
            not ctx.call('lane', 'lane_valid', now) or ctx.lane_stamp <= ctx.action_started):
        r['center_votes'] = 0
        r['center_stamp'] = ctx.lane_stamp
        return None
    points = sorted(local(ctx.pose,p) for p in ctx.lane)
    points = [p for p in points if p[0] > .05]
    if len(points) < 2 or points[-1][0]-points[0][0] < .10:
        r['center_votes'] = 0
        r['center_stamp'] = ctx.lane_stamp
        return None
    a,b = points[0],points[-1]
    heading = math.atan2(b[1]-a[1],b[0]-a[0])
    if abs(heading) > math.radians(50):
        r['center_votes'] = 0
        r['center_stamp'] = ctx.lane_stamp
        return None
    if ctx.lane_stamp > r.get('center_stamp',-1):
        r['center_stamp'] = ctx.lane_stamp
        r['center_votes'] = r.get('center_votes',0)+1
    if r.get('center_votes',0) < ctx.cfg['exit_frames']:
        return None
    lateral = a[1]*math.cos(heading)-a[0]*math.sin(heading)
    return dict(points=points,heading=heading,lateral=lateral)


def left_lock_tick(ctx, now):
    """Keep lock until a fresh centered exit is measured, not a model endpoint."""
    if ctx.cfg.get('left_timed_enabled',False):
        from robot.turn.timed_left import tick
        return tick(ctx,now)
    cfg,lock = ctx.cfg,ctx.left_lock
    progress = math.degrees(wrap(ctx.pose[2]-lock['origin'][2]))
    lock['progress_deg'] = progress
    if lock['phase'] == 'ENTRY':
        travelled = local(lock['origin'],ctx.pose)[0]
        lock['entry_travelled_m'] = travelled
        if travelled < cfg.get('left_turn_entry',cfg['turn_entry'])-1e-9:
            ctx.lane_source = 'left_entry'
            command = ctx.call('obstacle','checked_command',(cfg['speed_raw']['action'],0.),now,False)
            if command[0]: ctx.reason = 'left_entry_distance'
            return command
        lock.update(phase='LOCK',lock_started=now)
    ctx.lane_source = 'left_visual_lock'
    if progress >= cfg.get('left_turn_angle_deg',cfg.get('turn_angle_deg',90.))+30.:
        ctx.state = 'FAULT'
        return ctx.call('motion','stop','left_exit_not_found')
    minimum = cfg.get('left_lock_min_angle_deg',30.)
    valid = progress >= minimum and ctx.call('lane','lane_valid',now)
    if valid:
        points = sorted(local(ctx.pose,p) for p in ctx.lane)
        valid = len(points) >= 2 and points[-1][0]-points[0][0] >= .10
        if valid:
            a,b = points[:2]
            heading = math.atan2(b[1]-a[1],b[0]-a[0])
            lock.update(exit_heading_deg=math.degrees(heading),exit_lateral_m=a[1])
            valid = (a[0] > .05 and abs(a[1]) < cfg['exit_lateral_tolerance'] and
                     abs(heading) < math.radians(cfg.get('left_lock_heading_deg',20.)))
    if not valid:
        ctx.exit_count = 0
        ctx.exit_reason = 'left_before_min_turn' if progress < minimum else 'left_exit_not_aligned'
    elif ctx.lane_stamp > ctx.exit_stamp:
        if ctx.lane_stamp-ctx.exit_stamp > cfg['sensor_timeout']:
            ctx.exit_count = 0
        ctx.exit_stamp = ctx.lane_stamp
        ctx.exit_reason = 'before_action' if ctx.lane_stamp <= ctx.action_started else 'ok'
        ctx.exit_count = ctx.exit_count+1 if ctx.exit_reason == 'ok' else 0
    lock['exit_frames'] = ctx.exit_count
    if ctx.exit_count >= cfg['exit_frames']:
        ctx.call('mission','resume_lane')
        return ctx.call('obstacle','checked_command',ctx.call('lane','lane_command',now),now,False)
    steer = model_to_command_steering(cfg['max_steer'],cfg)
    command = ctx.call('obstacle','checked_command',(cfg['speed_raw']['action'],steer),now,False)
    if command[0]: ctx.reason = 'left_wait_exit_lane'
    return command


def right_lock_steering(cfg):
    scale = cfg.get('steering_command_scale_rad',cfg['max_steer'])
    return -min(cfg.get('right_lock_command_rad',.1),scale)


def right_blue_exit_tick(ctx, now):
    cfg,r = ctx.cfg,ctx.right_lock
    ctx.lane_source = 'right_blue_lock'
    r.update(exit_source='blue_junction',blue_local=None,blue_age_s=None,blue_reason='not_seen')
    if ctx.marker is not None:
        point,stamp = ctx.marker
        x,y = local(ctx.pose,point)
        r.update(blue_local=[x,y],blue_age_s=now-stamp)
        if stamp <= r['lock_started']:
            r['blue_reason'] = 'before_lock'
        elif not 0 <= now-stamp <= cfg['sensor_timeout']:
            r['blue_reason'] = 'stale'
        elif ctx.consumed_marker and distance(point,ctx.consumed_marker) < cfg['marker_rearm_distance']:
            r['blue_reason'] = 'consumed_entry_line'
        elif not 0 < x <= 1.10 or abs(y) > cfg['lane_width']:
            r['blue_reason'] = 'outside_forward_corridor'
        else:
            if stamp > r.get('blue_stamp',-1.):
                if (stamp-r.get('blue_stamp',-1.) > cfg['sensor_timeout'] or
                        distance(point,r.get('blue_candidate',point)) > .20):
                    r['blue_votes'] = 0
                r['blue_stamp'],r['blue_candidate'] = stamp,point
                r['blue_votes'] = r.get('blue_votes',0)+1
            r['blue_reason'] = 'confirming'
            if r.get('blue_votes',0) < cfg['exit_frames']:
                command = ctx.call('obstacle','checked_command',
                    (cfg['speed_raw']['action'],right_lock_steering(cfg)),now,False)
                if command[0]: ctx.reason = 'right_confirm_exit_blue'
                return command
            ctx.consumed_marker = point
            ctx.marker,ctx.parking_marker = None,None
            ctx.last_completed_action,ctx.completed_at_pose = 'RIGHT',ctx.pose
            ctx.action_source = 'right_blue'
            ctx.call('mission', 'start_follow', intersection_path(ctx.pose,'STRAIGHT',cfg),'STRAIGHT',now)
            ctx.lane_source = 'straight_search'
            command = ctx.call('obstacle', 'checked_command', (cfg.get('straight_speed_raw',12),0.0),now,False)
            if command[0]: ctx.reason = 'right_blue_begin_straight'
            return command
    r['blue_votes'] = 0
    command = ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['action'],
        right_lock_steering(cfg)),now,False)
    if command[0]: ctx.reason = 'right_wait_new_blue'
    return command


def right_lock_tick(ctx, now):
    if ctx.cfg.get('right_timed_enabled',False):
        from robot.turn.timed_right import tick
        return tick(ctx,now)
    cfg,r = ctx.cfg,ctx.right_lock
    progress = math.degrees(wrap(r['origin'][2]-ctx.pose[2]))
    r['progress_deg'] = progress
    blue_mode = cfg.get('right_exit_on_blue',False)
    if blue_mode and not 0 <= now-ctx.front_marker_stamp <= cfg.get('ground_timeout',1.25):
        return ctx.call('motion', 'stop', 'right_blue_front_stale')
    if r['phase'] == 'ENTRY':
        reverse_entry = cfg.get('right_reverse_entry_m',0.0)
        if reverse_entry > 0:
            if local(r['origin'],ctx.pose)[0] > -reverse_entry:
                return ctx.call('obstacle', 'checked_command', (-cfg['speed_raw']['action'],0.0),now,False)
            r['phase'] = 'LOCK'
            r['lock_started'] = now
            return ctx.call('motion', 'stop', 'right_reverse_entry_complete')
        if local(r['origin'],ctx.pose)[0] < cfg.get('right_turn_entry',cfg['turn_entry']):
            return ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['action'],0.0),now,False)
        r['phase'] = 'LOCK'
        r['lock_started'] = now
    if blue_mode:
        return ctx.call('turn', 'right_blue_exit_tick', now)
    line = ctx.call('lane', 'left_exit_line', now)
    r['left_heading_deg'] = math.degrees(line['heading']) if line else None
    r['left_lateral_m'] = line['lateral'] if line else None
    if cfg.get('right_direct_left_follow',False):
        if progress >= cfg.get('right_lock_max_angle_deg',120):
            ctx.state = 'FAULT'
            ctx.left_reference = None
            return ctx.call('motion', 'stop', 'right_alignment_angle_limit' if r['phase'] == 'ALIGN_LEFT' else 'right_exit_not_found')
        if (r['phase'] == 'LOCK' and line is not None and
                progress >= cfg.get('right_lock_min_angle_deg',30)):
            r['phase'] = 'ALIGN_LEFT'
            r['stable_since'] = None
        if r['phase'] == 'ALIGN_LEFT':
            r['exit_source'] = 'left_boundary_offset'
            ctx.lane_source = 'left_boundary'
            r['alignment_heading_deg'] = math.degrees(line['heading']) if line else None
            r['alignment_lateral_m'] = line['lateral'] if line else None
            if line is None:
                r['stable_since'] = None
                r['stable_s'] = 0
                ctx.exit_count = r['aligned_frames'] = 0
                ctx.left_reference = None
                return ctx.call('motion', 'stop', 'right_left_reference_missing')
            command = ctx.call('lane', 'left_reference_command', line,now)
            if ctx.left_reference is None:
                r['stable_since'] = None
                r['stable_s'] = 0
                ctx.exit_count = r['aligned_frames'] = 0
                return command
            if ctx.left_boundary_stamp > ctx.exit_stamp:
                ctx.exit_stamp = ctx.left_boundary_stamp
                if ctx.left_reference['aligned']:
                    if r['stable_since'] is None: r['stable_since'] = ctx.left_boundary_stamp
                    ctx.exit_count += 1
                else:
                    r['stable_since'] = None
                    ctx.exit_count = 0
            r['aligned_frames'] = ctx.exit_count
            r['stable_s'] = 0 if r['stable_since'] is None else ctx.exit_stamp-r['stable_since']
            if (ctx.exit_count >= cfg['exit_frames'] and
                    r['stable_s'] >= cfg.get('left_reference_stable_s',.5)):
                ctx.call('mission', 'resume_lane')
                ctx.follow_left_boundary = True
            return ctx.call('obstacle', 'checked_command', command,now,False)
        return ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['action'],
            right_lock_steering(cfg)),now,False)
    center = ctx.call('turn', 'right_center_exit', now,progress)
    source = 'left_boundary' if line else 'center' if center else None
    line = line or center
    r['alignment_heading_deg'] = math.degrees(line['heading']) if line else None
    r['alignment_lateral_m'] = line['lateral'] if line else None
    source_stamp = ctx.left_boundary_stamp if source == 'left_boundary' else ctx.lane_stamp
    if r.get('exit_source') != source:
        ctx.exit_count, ctx.exit_stamp = 0,-1.0
    r['exit_source'] = source
    if r['phase'] == 'LOCK':
        if (progress >= cfg.get('right_lock_min_angle_deg',55) and line and
                abs(line['heading']) <= math.radians(50)):
            r['phase'] = 'ALIGN'
        elif progress >= cfg.get('right_lock_max_angle_deg',120):
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', 'right_exit_not_found')
        else:
            # Command-space magnitude; 0.05 maps to raw 11 with scale 0.1.
            lock_steer = right_lock_steering(cfg)
            return ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['action'],lock_steer),now,False)
    if line is None:
        ctx.exit_count = 0
        r['aligned_frames'] = 0
        if (now-r['last_seen'] > cfg['gap_max_seconds'] or
                distance(ctx.pose,r['last_pose']) > cfg['gap_max_distance']):
            return ctx.call('motion', 'stop', 'right_alignment_line_lost')
        if r['lost_steer'] is None:
            r['lost_steer'] = ctx.call('motion', 'current_steering', now)
        return ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['gap'],r['lost_steer']),now,False)
    r.update(last_seen=now,last_pose=ctx.pose,lost_steer=None)
    if source_stamp > ctx.exit_stamp:
        ctx.exit_stamp = source_stamp
        aligned = (abs(line['heading']) <= math.radians(cfg.get('right_lock_heading_deg',15)) and
                   abs(line['lateral']) <= cfg['exit_lateral_tolerance'])
        ctx.exit_count = ctx.exit_count+1 if aligned else 0
    r['aligned_frames'] = ctx.exit_count
    if ctx.exit_count >= cfg['exit_frames']:
        ctx.call('mission', 'resume_lane')
        ctx.right_tail_origin = ctx.pose
        return ctx.call('obstacle', 'checked_command', ctx.call('lane', 'lane_command', now),now,True)
    x,y = min(line['points'],key=lambda p:abs(math.hypot(*p)-cfg.get('action_lookahead',.3)))
    steer = math.atan2(2*cfg['wheelbase']*y,max(.0025,x*x+y*y))
    steer = max(-cfg['max_steer'],min(cfg['max_steer'],steer))
    return ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['action'],steer),now,False)


def direction_exit_reached(ctx, action=None):
    return arc_exit_reached(ctx.pose, ctx.exit_pose,
                            ctx.action if action is None else action, ctx.cfg)


def exit_lane_confirmed(ctx, now):
    if not ctx.call('lane', 'lane_valid', now):
        ctx.exit_count = 0
        ctx.exit_reason = 'stale_or_invalid_lane'
        return False
    if ctx.lane_stamp > ctx.exit_stamp:
        if ctx.lane_stamp - ctx.exit_stamp > ctx.cfg['sensor_timeout']:
            ctx.exit_count = 0
        ctx.exit_stamp = ctx.lane_stamp
        ctx.exit_reason = ('before_action' if ctx.lane_stamp <= ctx.action_started else
            exit_candidate(ctx.pose, ctx.exit_pose, ctx.lane, ctx.action, ctx.cfg))
        ctx.exit_count = ctx.exit_count + 1 if ctx.exit_reason == 'ok' else 0
    return ctx.exit_count >= ctx.cfg['exit_frames']


def handoff_lane_confirmed(ctx, now):
    """Timed maneuvers have no measured endpoint; use the near visual tangent.

    Permit curved exits; reject a crossing or laterally unrelated path. Count
    distinct camera frames, never repeated controller ticks on one image.
    """
    if not ctx.call('lane','lane_valid',now):
        ctx.exit_count=0
        ctx.exit_reason='stale_or_invalid_lane'
        return False
    if ctx.lane_stamp > ctx.exit_stamp:
        if ctx.lane_stamp-ctx.exit_stamp > ctx.cfg['sensor_timeout']:
            ctx.exit_count=0
        ctx.exit_stamp=ctx.lane_stamp
        ctx.exit_reason=exit_candidate(ctx.pose,ctx.pose,ctx.lane,'STRAIGHT',ctx.cfg)
        ctx.exit_count=ctx.exit_count+1 if ctx.exit_reason=='ok' else 0
    return ctx.exit_count >= ctx.cfg['exit_frames']
