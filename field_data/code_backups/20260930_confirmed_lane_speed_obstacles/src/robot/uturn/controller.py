"""uturn module: explicit context input; coordinator applies returned updates."""
from __future__ import division

import math
import copy
from robot.common.geometry import distance
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.motion.tracker import Follower
from robot.turn.planner import intersection_path
from robot.turn.planner import turn_parameters
from robot.camera.alignment import rear_target
from robot.camera.alignment import steering_for_heading
from robot.common.contracts import model_to_command_steering
from robot.uturn.relative import RelativeUturn
from robot.uturn.relative import decode_scene
from robot.uturn.timed import TimedUturn



FIELDS = ('action_started', 'cfg', 'consumed_marker', 'course_clear_frames', 'course_finished',
 'course_marker_stamp', 'course_stop_armed', 'course_stop_pending', 'executor',
 'exit_count', 'exit_pose', 'exit_stamp', 'follow_left_boundary', 'follower',
 'front_blue_lines', 'front_marker_stamp', 'front_markers', 'gap_origin', 'lane_source',
 'left_reference', 'marker', 'parking_marker', 'pending', 'pending_at', 'pose',
 'rear_blue_lines', 'rear_marker_stamp', 'rear_markers', 'reason', 'relative_future',
 'relative_replan_future', 'relative_scene', 'relative_uturn', 'right_tail_origin',
 'startup_started', 'startup_origin', 'startup_blue_point', 'startup_curve_lock',
 'state', 'timed_uturn', 'uturn', 'wait_until')
CALLS = (('lane', 'lane_command'), ('lane', 'lane_valid'), ('turn','handoff_lane_confirmed'), ('mission', 'dispatch'),
 ('mission', 'resume_lane'), ('motion', 'stop'), ('obstacle', 'checked_command'),
 ('turn', 'direction_exit_reached'), ('turn', 'exit_lane_confirmed'),
 ('uturn', 'relative_uturn_tick'), ('uturn', 'uturn_align_first'),
 ('uturn', 'uturn_begin_alignment'), ('uturn', 'uturn_blue_target'),
 ('uturn', 'uturn_fixed_exit_blue'), ('uturn', 'uturn_fixed_reverse'),
 ('uturn', 'uturn_follow'), ('uturn', 'uturn_outer_blue_confirmed'),
 ('uturn', 'uturn_outer_blue_tick'), ('uturn', 'uturn_phase'),
 ('uturn', 'uturn_reverse_rear'), ('uturn', 'uturn_trial_tick'))
OPERATIONS = ('uturn_phase', 'uturn_follow', 'uturn_outer_blue_confirmed', 'uturn_outer_blue_tick',
 'uturn_blue_target', 'uturn_begin_alignment', 'uturn_align_first', 'uturn_reverse_rear',
 'uturn_fixed_exit_blue', 'uturn_fixed_reverse', 'uturn_trial_tick', 'uturn_tick',
 'observe_relative_scene', 'relative_uturn_tick', 'course_blue_stop', 'startup_tick')

def uturn_phase(ctx, phase, now):
    ctx.uturn.update(phase=phase, phase_started=now)


def uturn_follow(ctx, now, second=False):
    cfg = dict(ctx.cfg)
    ctx.follow_left_boundary = False
    ctx.left_reference = None
    ctx.right_tail_origin = None
    if second and not ctx.cfg.get('uturn_reverse_distance_m',0):
        # Complete the residual heading after visual alignment and reverse.
        remaining = (ctx.uturn['target_yaw']-ctx.pose[2]) % (2*math.pi)
        if remaining > math.pi + ctx.cfg['exit_yaw_tolerance']:
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', 'uturn_heading_past_target')
        cfg.update(left_turn_entry=0.0, left_turn_angle_deg=math.degrees(remaining))
    path = intersection_path(ctx.pose,'LEFT',cfg)
    ctx.follower = Follower(path,cfg)
    ctx.exit_pose = path[-1][:3]
    ctx.exit_count, ctx.exit_stamp = 0,-1.0
    if ctx.cfg.get('uturn_reverse_distance_m',0):
        ctx.uturn['left_started']=now
        for key in ('blue_target','blue_candidate','blue_observation_stamp',
                    'blue_votes','blue_seen_stamp','blue_predicted'):
            ctx.uturn.pop(key,None)
    ctx.call('uturn', 'uturn_phase', 'LEFT_SECOND' if second else 'LEFT_FIRST',now)


def uturn_outer_blue_confirmed(ctx, now):
    # White paths through the inner opening are not outer-road evidence.
    u,cfg=ctx.uturn,ctx.cfg
    if ctx.front_marker_stamp<=u['left_started']:
        ctx.exit_count=0
        u['exit_reason']='blue_before_left'
        return False
    if not ctx.call('turn', 'direction_exit_reached', 'LEFT'):
        ctx.exit_count=0
        u['exit_reason']='before_left_arc_exit'
        return False
    line=ctx.call('uturn', 'uturn_blue_target', now)
    if line is None or u.get('blue_predicted',False):
        ctx.exit_count=0
        u['exit_reason']='outer_blue_missing_or_predicted'
        return False
    stamp=u['blue_seen_stamp']
    heading=wrap(line['yaw']-ctx.pose[2])
    aligned=abs(heading)<=math.radians(cfg.get('uturn_align_heading_deg',8))
    if stamp>ctx.exit_stamp:
        if stamp-ctx.exit_stamp>cfg['sensor_timeout']:
            ctx.exit_count=0
        ctx.exit_stamp=stamp
        ctx.exit_count=ctx.exit_count+1 if aligned else 0
    u.update(exit_source='outer_blue',exit_heading_deg=math.degrees(heading),
             exit_frames=ctx.exit_count,exit_reason='ok' if aligned else 'outer_blue_heading')
    return aligned and ctx.exit_count>=cfg['exit_frames']


def uturn_outer_blue_tick(ctx, now):
    u,cfg=ctx.uturn,ctx.cfg
    first=u['phase'] in ('LEFT_FIRST','FIRST_REACQUIRE')
    if not 0<=now-ctx.front_marker_stamp<=cfg.get('ground_timeout',1.25):
        return ctx.call('motion', 'stop', 'uturn_outer_blue_front_stale')
    if ctx.call('uturn', 'uturn_outer_blue_confirmed', now):
        if first:
            return ctx.call('uturn', 'uturn_begin_alignment', now)
        ctx.call('mission', 'resume_lane')
        return ctx.call('obstacle', 'checked_command', ctx.call('lane', 'lane_command', now),now,True)
    if u['phase'] in ('FIRST_REACQUIRE','SECOND_REACQUIRE'):
        if (distance(ctx.pose,ctx.exit_pose)>=cfg['exit_search_distance'] or
                abs(wrap(ctx.pose[2]-ctx.exit_pose[2]))>cfg['exit_yaw_tolerance']):
            return ctx.call('motion', 'stop', 'uturn_outer_blue_not_found')
        return ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['gap'],0.0),now,False)
    speed,physical=ctx.follower.command(ctx.pose,now)
    if ctx.follower.done:
        ctx.call('uturn', 'uturn_phase', 'FIRST_REACQUIRE' if first else 'SECOND_REACQUIRE',now)
        return ctx.call('motion', 'stop', 'uturn_wait_outer_blue')
    return ctx.call('obstacle', 'checked_command', (speed,model_to_command_steering(physical,cfg)),now,False)


def uturn_blue_target(ctx, now):
    cfg,u=ctx.cfg,ctx.uturn
    stamp=ctx.front_marker_stamp
    if not 0<=now-stamp<=cfg['sensor_timeout'] or stamp<=ctx.action_started:
        u['blue_reason']='uturn_blue_camera_stale'
        return None
    if stamp>u.get('blue_observation_stamp',-1):
        u['blue_observation_stamp']=stamp
        candidates=[]
        for line in ctx.front_blue_lines:
            x,y=local(ctx.pose,line['point'])
            heading=wrap(line['yaw']-ctx.pose[2])
            if not (.1<x<1.6 and abs(y)<=cfg['lane_width'] and
                    abs(heading)<math.radians(75) and
                    line['length']>=cfg['blue']['long_min']):
                continue
            if distance(line['point'],u['start_line'])<cfg['marker_rearm_distance']:
                continue
            locked=u.get('blue_target')
            if locked is not None and (distance(line['point'],locked['point'])>.30 or
                    abs(wrap(line['yaw']-locked['yaw']))>math.radians(20)):
                continue
            candidates.append(line)
        line=min(candidates,key=lambda p:local(ctx.pose,p['point'])[0]) if candidates else None
        if line is not None:
            previous=u.get('blue_candidate')
            same=(previous is not None and distance(line['point'],previous['point'])<=.30 and
                  abs(wrap(line['yaw']-previous['yaw']))<=math.radians(20))
            u['blue_votes']=u.get('blue_votes',0)+1 if same else 1
            u['blue_candidate']=line
            if u.get('blue_target') is not None or u['blue_votes']>=2:
                u.update(blue_target=line,blue_seen_pose=ctx.pose,blue_seen_stamp=stamp,
                         blue_seen_x=local(ctx.pose,line['point'])[0])
        else:
            u['blue_votes']=0
    line=u.get('blue_target')
    if line is None:
        u['blue_reason']='uturn_blue_target_unconfirmed'
        return None
    predicted=stamp>u['blue_seen_stamp']
    if predicted and (u['blue_seen_x']>cfg.get('uturn_blue_blind_entry_m',.60) or
                      distance(ctx.pose,u['blue_seen_pose'])>cfg.get('uturn_blue_blind_max_m',.35)):
        u['blue_reason']='uturn_blue_target_lost'
        return None
    u.update(blue_predicted=predicted,blue_reason='ok')
    return line


def uturn_begin_alignment(ctx, now):
    if ctx.cfg.get('uturn_reverse_distance_m',0):
        ctx.call('uturn', 'uturn_phase', 'WAIT_EXIT_BLUE',now)
        ctx.uturn['align_origin']=ctx.pose
        ctx.marker=None
        return ctx.call('motion', 'stop', 'uturn_first_left_complete_wait_blue')
    ctx.call('uturn', 'uturn_phase', 'ALIGN_FIRST',now)
    ctx.uturn.update(stable_since=None,aligned_frames=0,stable_s=0,
                      align_origin=ctx.pose,alignment_source='blue_normal',
                      rear_alignment_latched=False)
    return ctx.call('motion', 'stop', 'uturn_first_left_complete')


def uturn_align_first(ctx, now):
    cfg,u = ctx.cfg,ctx.uturn
    line = ctx.call('uturn', 'uturn_blue_target', now) if not u.get('rear_alignment_latched') else None
    front_missing = (not 0 <= now-ctx.front_marker_stamp <= cfg['sensor_timeout'] or
                     ctx.front_marker_stamp <= u['phase_started'] or not ctx.front_blue_lines)
    if front_missing or u.get('blue_predicted',False):
        u['rear_alignment_latched'] = True
    rear = u.get('rear_alignment_latched',False)
    if rear:
        line = rear_target(ctx.pose,ctx.rear_blue_lines,ctx.rear_marker_stamp,now,cfg,u)
    u['alignment_source'] = 'rear_blue_normal' if rear else 'blue_normal'
    measured_stamp = ctx.rear_marker_stamp if rear else u.get('blue_seen_stamp',-1)
    if line is None:
        u.update(stable_since=None,aligned_frames=0,stable_s=0)
        return ctx.call('motion', 'stop', u.get('blue_reason','uturn_blue_target_unconfirmed'))
    heading = wrap(line['yaw']-ctx.pose[2])
    x,y = local(ctx.pose,line['point'])
    u.update(heading_deg=math.degrees(heading),blue_local=[x,y],
             blue_distance_m=x*math.cos(heading)+y*math.sin(heading))
    aligned = abs(heading) <= math.radians(cfg.get('uturn_align_heading_deg',8))
    if not aligned or measured_stamp-u.get('align_stamp',measured_stamp) > cfg['sensor_timeout']:
        u.update(stable_since=None,aligned_frames=0,stable_s=0)
    if aligned and measured_stamp > u.get('align_stamp',-1):
        u['align_stamp'] = measured_stamp
        u['aligned_frames'] = u.get('aligned_frames',0)+1
        if u.get('stable_since') is None:
            u['stable_since'] = measured_stamp
        u['stable_s'] = measured_stamp-u['stable_since']
    if (aligned and u.get('aligned_frames',0) >= cfg['exit_frames'] and
            u.get('stable_s',0) >= cfg.get('uturn_align_stable_s',.5)):
        u['target_yaw'] = wrap(line['yaw']+math.radians(turn_parameters(cfg,'LEFT')['turn_angle_deg']))
        ctx.call('uturn', 'uturn_phase', 'BRAKE_REVERSE',now)
        ctx.wait_until = now+cfg['cusp_pause']
        return ctx.call('motion', 'stop', 'uturn_blue_heading_aligned')
    if aligned:
        return ctx.call('motion', 'stop', 'uturn_blue_heading_confirming')
    if distance(ctx.pose,u['align_origin']) >= cfg.get('uturn_alignment_max_distance',.5):
        return ctx.call('motion', 'stop', 'uturn_alignment_distance_limit')
    if not rear and u['blue_distance_m'] <= cfg['marker_trigger_x']:
        return ctx.call('motion', 'stop', 'uturn_blue_close_wait_alignment')
    return ctx.call('obstacle', 'checked_command', (cfg.get('uturn_align_speed_raw',12),
                                 steering_for_heading(heading,cfg)),now,False)


def uturn_reverse_rear(ctx, now):
    cfg,u = ctx.cfg,ctx.uturn
    if not 0<=now-ctx.rear_marker_stamp<=cfg.get('ground_timeout',1.25):
        u['rear_votes']=0
        return ctx.call('motion', 'stop', 'uturn_rear_stale')
    if abs(wrap(ctx.pose[2]-u['reverse_origin'][2]))>cfg.get('uturn_reverse_yaw_rad',.15):
        return ctx.call('motion', 'stop', 'uturn_reverse_heading_drift')
    seen=False
    if ctx.rear_marker_stamp>u.get('rear_stamp',u['phase_started']):
        u['rear_stamp']=ctx.rear_marker_stamp
        points=[p for p in ctx.rear_markers if abs(local(ctx.pose,p)[1])<=cfg['lane_width']/2]
        locked=u.get('rear_line')
        if locked is not None:
            points=[p for p in points if distance(p,locked)<=.15]
        point=max(points,key=lambda p:local(ctx.pose,p)[0]) if points else None
        if point is not None:
            same=u.get('rear_candidate') is not None and distance(point,u['rear_candidate'])<=.15
            u['rear_votes']=u.get('rear_votes',0)+1 if same else 1
            u['rear_candidate']=point
            if locked is not None or u['rear_votes']>=2:
                u.update(rear_line=point,rear_seen_pose=ctx.pose,rear_seen_x=local(ctx.pose,point)[0])
                seen=True
        else:
            u['rear_votes']=0
        u['rear_visible']=seen
    locked=u.get('rear_line')
    if locked is not None:
        x,y=local(ctx.pose,locked)
        u.update(rear_line_local=[x,y],front_axle_error_m=cfg['wheelbase']-x)
        if abs(y)>cfg['lane_width']/2:
            return ctx.call('motion', 'stop', 'uturn_rear_target_off_axis')
        if not u.get('rear_visible',False):
            if u['rear_seen_x'] < -cfg.get('uturn_rear_blind_entry_m',.20):
                return ctx.call('motion', 'stop', 'uturn_rear_line_lost')
            if distance(ctx.pose,u['rear_seen_pose'])>cfg.get('uturn_rear_blind_max_m',.60):
                return ctx.call('motion', 'stop', 'uturn_rear_blind_limit')
        if x>=cfg['wheelbase']-cfg.get('uturn_brake_lead_m',.02):
            if x>cfg['wheelbase']+cfg.get('uturn_axle_tolerance_m',.06):
                return ctx.call('motion', 'stop', 'uturn_front_axle_overshoot')
            ctx.consumed_marker=locked
            ctx.marker=None
            ctx.call('uturn', 'uturn_phase', 'BRAKE_FORWARD',now)
            ctx.wait_until=now+cfg['cusp_pause']
            return ctx.call('motion', 'stop', 'uturn_brake_forward')
    if distance(ctx.pose,u['reverse_origin'])>=cfg.get('uturn_reverse_max_distance',1.0):
        return ctx.call('motion', 'stop', 'uturn_reverse_limit_wait_blue')
    return ctx.call('obstacle', 'checked_command', (-cfg.get('uturn_reverse_speed_raw',12),0.0),now,False)


def uturn_fixed_exit_blue(ctx, now):
    u,cfg=ctx.uturn,ctx.cfg
    if not 0 <= now-ctx.front_marker_stamp <= cfg.get('ground_timeout',1.25):
        return ctx.call('motion', 'stop', 'uturn_exit_blue_front_stale')
    if ctx.marker is not None:
        point,stamp=ctx.marker
        x,y=local(ctx.pose,point)
        if (stamp>u['phase_started'] and 0<=now-stamp<=cfg['marker_memory_s'] and
                distance(point,u['start_line'])>=cfg['marker_rearm_distance'] and
                (u.get('blue_target') is None or distance(point,u['blue_target']['point'])<=.30) and
                -.12<=x<=cfg['marker_trigger_x'] and abs(y)<=cfg['lane_width']):
            ctx.consumed_marker=point
            ctx.marker=None
            ctx.call('uturn', 'uturn_phase', 'BRAKE_REVERSE',now)
            ctx.wait_until=now+cfg['cusp_pause']
            return ctx.call('motion', 'stop', 'uturn_exit_blue_triggered')
    if distance(ctx.pose,u['align_origin'])>=cfg.get('straight_search_max_distance',2.2):
        ctx.state='FAULT'
        return ctx.call('motion', 'stop', 'uturn_exit_blue_distance_limit')
    return ctx.call('obstacle', 'checked_command', (cfg['straight_speed_raw'],0.0),now,False)


def uturn_fixed_reverse(ctx, now):
    u,cfg=ctx.uturn,ctx.cfg
    if abs(wrap(ctx.pose[2]-u['reverse_origin'][2]))>cfg.get('uturn_reverse_yaw_rad',.15):
        return ctx.call('motion', 'stop', 'uturn_reverse_heading_drift')
    x,y=local(u['reverse_origin'],ctx.pose)
    u['reverse_travelled_m']=max(0.0,-x)
    u['reverse_target_m']=cfg['uturn_reverse_distance_m']
    if -x>=cfg['uturn_reverse_distance_m']:
        ctx.call('uturn', 'uturn_phase', 'BRAKE_FORWARD',now)
        ctx.wait_until=now+cfg['cusp_pause']
        return ctx.call('motion', 'stop', 'uturn_reverse_distance_complete')
    return ctx.call('obstacle', 'checked_command', (-cfg['speed_raw']['action'],0.0),now,False)


def uturn_trial_tick(ctx, now):
    cfg=ctx.cfg
    if now<ctx.wait_until:
        return ctx.call('motion', 'stop', 'uturn_trial_wait_start')
    if not 0<=now-ctx.front_marker_stamp<=cfg.get('ground_timeout',1.25):
        return ctx.call('motion', 'stop', 'uturn_trial_front_stale')
    try:
        if ctx.timed_uturn is None:ctx.timed_uturn=TimedUturn(cfg)
        task=ctx.timed_uturn
        command=task.command(now)
    except ValueError as exc:
        ctx.state='FAULT'
        return ctx.call('motion', 'stop', 'uturn_trial_error:'+str(exc))
    ctx.uturn.update(task.status())
    if task.done:
        if not cfg.get('uturn_trial_resume_lane',False):
            ctx.state='FINISHED'
            return ctx.call('motion', 'stop', 'uturn_trial_complete_stop')
        if not ctx.call('turn','handoff_lane_confirmed',now):return ctx.call('motion', 'stop', 'uturn_trial_wait_lane')
        ctx.timed_uturn=None
        ctx.call('mission', 'resume_lane')
        if cfg.get('uturn_course_test',False):
            ctx.course_stop_pending=True
            ctx.course_stop_armed=False
            ctx.course_clear_frames=0
            ctx.course_marker_stamp=ctx.front_marker_stamp
        return ctx.call('motion', 'stop', 'uturn_trial_complete_lane')
    result=ctx.call('obstacle', 'checked_command', command,now,False)
    if result==command:ctx.reason='uturn_trial_'+ctx.uturn['phase'].lower()
    return result


def uturn_tick(ctx, now):
    if ctx.cfg.get('uturn_trial_enabled',False):
        return ctx.call('uturn', 'uturn_trial_tick', now)
    if ctx.cfg.get('uturn_relative_enabled',False):
        if now<ctx.wait_until and ctx.relative_uturn is None:
            return ctx.call('motion', 'stop', 'uturn_wait_start')
        return ctx.call('uturn', 'relative_uturn_tick', now)
    u, cfg = ctx.uturn,ctx.cfg
    phase = u['phase']
    if phase in ('WAIT_START','BRAKE_REVERSE','BRAKE_FORWARD'):
        if now < ctx.wait_until:
            return ctx.call('motion', 'stop', 'uturn_'+phase.lower())
        if phase == 'BRAKE_REVERSE':
            ctx.call('uturn', 'uturn_phase', 'REVERSE_STRAIGHT',now)
            u['reverse_origin'] = ctx.pose
            target = u.get('rear_blue_target') if u.get('rear_alignment_latched') else None
            u.update(rear_line=target['point'] if target else None,
                     rear_votes=0,rear_candidate=None,rear_visible=False)
            if target:
                u.update(rear_seen_pose=ctx.pose,rear_seen_x=local(ctx.pose,target['point'])[0])
        else:
            if phase == 'BRAKE_FORWARD' and not cfg.get('uturn_reverse_distance_m',0):
                x=local(ctx.pose,u['rear_line'])[0]
                if abs(x-cfg['wheelbase'])>cfg.get('uturn_axle_tolerance_m',.06):
                    return ctx.call('motion', 'stop', 'uturn_front_axle_not_at_blue')
            result = ctx.call('uturn', 'uturn_follow', now,second=phase == 'BRAKE_FORWARD')
            if result is not None:
                return result
        phase = u['phase']
    if phase == 'REVERSE_STRAIGHT':
        if cfg.get('uturn_reverse_distance_m',0):
            return ctx.call('uturn', 'uturn_fixed_reverse', now)
        return ctx.call('uturn', 'uturn_reverse_rear', now)
    if phase == 'WAIT_EXIT_BLUE':
        return ctx.call('uturn', 'uturn_fixed_exit_blue', now)
    if cfg.get('uturn_reverse_distance_m',0) and phase in (
            'LEFT_FIRST','LEFT_SECOND','FIRST_REACQUIRE','SECOND_REACQUIRE'):
        return ctx.call('uturn', 'uturn_outer_blue_tick', now)
    if phase in ('LEFT_FIRST','LEFT_SECOND') and not (
            0<=now-ctx.front_marker_stamp<=cfg.get('ground_timeout',1.25) or ctx.call('lane', 'lane_valid', now)):
        return ctx.call('motion', 'stop', 'uturn_front_stale')
    if phase == 'LEFT_FIRST':
        if ctx.call('turn', 'direction_exit_reached', 'LEFT') and ctx.call('turn', 'exit_lane_confirmed', now):
            return ctx.call('uturn', 'uturn_begin_alignment', now)
    if phase == 'FIRST_REACQUIRE':
        if ctx.call('turn', 'exit_lane_confirmed', now):
            return ctx.call('uturn', 'uturn_begin_alignment', now)
        if (distance(ctx.pose,ctx.exit_pose)>=cfg['exit_search_distance'] or
                abs(wrap(ctx.pose[2]-ctx.exit_pose[2]))>cfg['exit_yaw_tolerance']):
            return ctx.call('motion', 'stop', 'correct_exit_not_found')
        return ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['gap'],0.0),now,False)
    if phase == 'ALIGN_FIRST':
        return ctx.call('uturn', 'uturn_align_first', now)
    if phase == 'LEFT_SECOND':
        # Do not let the original lane cancel the maneuver at its first arc.
        tail = turn_parameters(cfg,'LEFT')['turn_exit']
        if local(ctx.exit_pose,ctx.pose)[0] >= -tail and ctx.call('turn', 'exit_lane_confirmed', now):
            ctx.call('mission', 'resume_lane')
            return ctx.call('obstacle', 'checked_command', ctx.call('lane', 'lane_command', now),now,True)
    speed,physical = ctx.follower.command(ctx.pose,now)
    command = speed,model_to_command_steering(physical,cfg)
    if ctx.follower.done:
        if phase == 'LEFT_SECOND':
            ctx.call('uturn', 'uturn_phase', 'EXIT_SEARCH',now)
            ctx.state = 'REACQUIRE'
            return ctx.call('motion', 'stop', 'uturn_reacquire')
        ctx.call('uturn', 'uturn_phase', 'FIRST_REACQUIRE',now)
        return ctx.call('motion', 'stop', 'uturn_first_reacquire')
    return ctx.call('obstacle', 'checked_command', command,now,False)


def observe_relative_scene(ctx, data, now):
    scene=decode_scene(data)
    if not 0<=now-scene['stamp']<=ctx.cfg['sensor_timeout']:
        raise ValueError('relative scene stale')
    if ctx.relative_scene and scene['stamp']<=ctx.relative_scene['stamp']:
        return
    ctx.relative_scene=scene
    if ctx.relative_uturn:ctx.relative_uturn.observe(scene)


def relative_uturn_tick(ctx, now):
    if ctx.relative_uturn is None:
        scene=ctx.relative_scene
        if (scene is None or scene['stamp'] < ctx.action_started or
                not 0<=now-scene['stamp']<=ctx.cfg['sensor_timeout']):
            return ctx.call('motion', 'stop', 'relative_uturn_scene_missing')
        ctx.relative_uturn=RelativeUturn(ctx.cfg,scene)
        ctx.relative_future=ctx.executor.submit(ctx.relative_uturn.plan)
        return ctx.call('motion', 'stop', 'relative_uturn_planning')
    task=ctx.relative_uturn
    if ctx.relative_replan_future is not None:
        candidate,future = ctx.relative_replan_future
        if not future.done():
            return ctx.call('motion', 'stop', 'relative_uturn_replanning')
        ctx.relative_replan_future = None
        try:
            future.result()
        except Exception as exc:
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', 'relative_uturn_planner_error:'+str(exc))
        # A worker only mutates its private candidate. A changed reference
        # received while it was searching must never revive the old target.
        if (getattr(task,'reference_invalid',False) or task.reason in
                ('relative_uturn_reference_changed','relative_uturn_replan_reference_invalid')):
            return ctx.call('motion', 'stop', task.reason)
        candidate.observe(task.scene)
        ctx.relative_uturn = task = candidate
    if task.phase=='PLAN':
        if not ctx.relative_future.done():return ctx.call('motion', 'stop', 'relative_uturn_planning')
        try:
            task.accept_plan(ctx.relative_future.result())
        except Exception as exc:
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', 'relative_uturn_planner_error:'+str(exc))
    command=task.command(now)
    ctx.uturn.update(phase='RELATIVE_'+task.phase,goal=task.goal,
                      followed_boundary=task.side,frame=task.frame,
                      measured_pose=task.scene['pose'])
    if (task.phase == 'BLOCKED' and task.reason in
            ('relative_uturn_path_blocked','relative_uturn_tracking_error',
             'relative_uturn_replan_failed') and
            task.replan_attempts < task.replan_limit and
            now-task.last_replan >= task.replan_cooldown and
            0 <= now-task.scene['stamp'] <= ctx.cfg['sensor_timeout']):
        candidate = copy.copy(task)
        ctx.relative_replan_future = (candidate,
            ctx.executor.submit(candidate.replan,now))
        return ctx.call('motion', 'stop', 'relative_uturn_replanning')
    if task.phase=='DONE':
        ctx.relative_uturn=None
        ctx.call('mission', 'resume_lane')
        return ctx.call('motion', 'stop', 'relative_uturn_complete')
    ctx.reason=task.reason
    return ctx.call('obstacle', 'checked_command', command,now,False)


def course_blue_stop(ctx,now):
    if (ctx.front_marker_stamp<=ctx.course_marker_stamp or
            not 0<=now-ctx.front_marker_stamp<=ctx.cfg.get('ground_timeout',1.25)):
        return False
    ctx.course_marker_stamp=ctx.front_marker_stamp
    if not ctx.course_stop_armed:
        ctx.course_clear_frames=ctx.course_clear_frames+1 if not ctx.front_markers else 0
        ctx.course_stop_armed=ctx.course_clear_frames>=3
        return False
    for point in ctx.front_markers:
        x,y=local(ctx.pose,point)
        if 0<x<=ctx.cfg['marker_trigger_x'] and abs(y)<=ctx.cfg['lane_width']:
            ctx.course_finished=True
            ctx.state='FINISHED'
            ctx.marker,ctx.pending=None,None
            return True
    return False


def startup_tick(ctx, now):
    cfg = ctx.cfg
    trim = cfg.get('startup_steering_raw',0)
    ctx.lane_source = 'startup_trim' if trim else 'startup_zero_steer'
    triggered_now = False
    if ctx.startup_blue_point is None and ctx.marker is not None:
        point, stamp = ctx.marker
        x,y = local(ctx.pose,point)
        triggered_now = (stamp > ctx.startup_started and
            0 <= now-stamp <= cfg['marker_memory_s'] and
            -0.12 <= x <= cfg['marker_trigger_x'] and abs(y) <= cfg['lane_width'])
        if triggered_now:
            ctx.startup_blue_point = point
    travelled = local(ctx.startup_origin,ctx.pose)[0]
    if travelled >= max(1.25,cfg.get('straight_distance',1.25)):
        # Startup counts from green release; signed STRAIGHT counts from blue.
        ctx.consumed_marker = ctx.startup_blue_point
        ctx.startup_blue_point = None
        ctx.marker, ctx.parking_marker = None, None
        ctx.state = 'LANE'
        ctx.startup_curve_lock = None
        ctx.gap_origin = None
        ctx.startup_started = None
        return ctx.call('obstacle', 'checked_command', ctx.call('lane', 'lane_command', now),now,True)
    if now-ctx.startup_started >= cfg['action_timeout']:
        ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'startup_distance_timeout')
    if not 0 <= now-ctx.front_marker_stamp <= cfg.get('ground_timeout',1.25):
        return ctx.call('motion', 'stop', 'startup_front_stale')
    steer = (float(trim) / cfg['steering_raw_limit'] / cfg['steering_sign'] *
             cfg.get('steering_command_scale_rad',cfg['max_steer']))
    command = ctx.call('obstacle', 'checked_command',
                       (cfg.get('straight_speed_raw',cfg['speed_raw']['lane']),steer),now,False)
    if command[0]:
        ctx.reason = 'startup_straight_distance'
    return command
