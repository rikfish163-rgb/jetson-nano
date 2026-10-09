"""mission module: explicit context input; coordinator applies returned updates."""
from __future__ import division

import math
from robot.common.geometry import _isfinite
from robot.common.geometry import distance
from robot.common.geometry import inside_slot
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.motion.tracker import Follower
from robot.turn.planner import intersection_path
from robot.common.contracts import model_to_command_steering
from robot.common.executor import _SimpleFuture
from robot.common.executor import _SingleJobExecutor
from robot.parking.forward import ForwardParking
from robot.common.lidar_policy import lidar_guard_disabled_reason
from robot.motion.calibration import speed_gain



FIELDS = ('action', 'action_source', 'action_started', 'route_action_started',
 'right_sign_locked', 'right_sign_rearmed_at',
 'route_sign_last_seen', 'route_sign_rearmed', 'right_completed_count', 'cfg', 'completed_at_pose',
 'consumed_marker', 'course_finished', 'course_stop_pending', 'direction_window', 'estop',
 'executor', 'exit_count', 'exit_pose', 'exit_stamp', 'follow_left_boundary', 'follower',
 'front_marker_stamp', 'future', 'gap_origin', 'last_completed_action', 'left_reference', 'lane_source',
 'marker', 'blue_consumed', 'marker_candidate', 'last_blue_trigger', 'next_direction', 'next_direction_at', 'obstacle_check', 'park_line_side', 'lane_stamp', 'parallel_future',
 'parallel_parking', 'parking_candidates', 'parking_entry', 'parking_sign', 'parking_front_frames', 'parking_route_ready',
 'parking_marker', 'parking_trigger_at', 'parking_trigger_pose', 'pending', 'pending_at',
 'pose', 'pose_stamp', 'reason', 'red', 'left_lock', 'right_lock', 'right_tail_origin', 'sign_count', 'sign_info',
 'sign_label', 'startup_started', 'startup_origin', 'startup_blue_point', 'startup_curve_lock',
 'state', 'straight_search', 'parking_straight_travel', 'timed_uturn', 'timed_parking', 'uturn', 'uturn_exit_straight',
 'uturn_sign_window', 'wait_until', 'timed_bypass', 'timed_bypass_completed', 'timed_bypass_seen', 'lane_recovery',
 'blue_approach', 'blue_prealign', 'front_blue_image_lines', 'blue_clear_since', 'blue_clear_frames', 'front_blue_lines', 'lane_curve_lock', 'lane_preview', 'lane_speed_state')
CALLS = (('lane', 'lane_command'), ('mission', 'dispatch'), ('mission', 'resume_lane'),
 ('mission', 'start_follow'), ('motion', 'follow_path_tick'), ('motion', 'stop'),
 ('obstacle', 'checked_command'), ('obstacle', 'scan_ready'), ('obstacle', 'timed_bypass_tick'),
 ('lane', 'park_line_command'),
 ('parallel_parking', 'parallel_parking_tick'), ('parking', 'auto_parking_tick'),
 ('parking', 'start_parking'), ('parking', 'start_explicit_parking'),
 ('parking', 'explicit_parking_tick'), ('turn', 'exit_lane_confirmed'),
 ('turn', 'straight_search_tick'), ('turn', 'blue_approach_tick'), ('uturn', 'course_blue_stop'),
 ('uturn', 'startup_tick'), ('uturn', 'uturn_phase'), ('uturn', 'uturn_tick'))
OPERATIONS = ('close', 'set_pose', 'start_follow', 'resume_lane', 'dispatch', 'begin_blue_action', 'begin_startup', 'tick')

def initial_state(cfg):
    state = {}
    state['cfg'] = cfg
    state['pose'] = (0.0, 0.0, 0.0)
    state['pose_stamp'] = -1.0
    state['state'] = 'WAIT_GREEN' if cfg['wait_green'] else 'LANE'
    state['startup_started'] = None
    state['startup_origin'] = None
    state['startup_blue_point'] = None
    state['startup_curve_lock'] = None
    state['relative_scene'] = None
    state['relative_uturn'] = None
    state['timed_uturn'] = None
    state['uturn_exit_straight'] = False
    state['timed_parking'] = None
    state['timed_bypass'] = None
    state['timed_bypass_completed'] = False
    state['timed_bypass_seen'] = []
    state['timed_bypass_candidate'] = None
    state['course_stop_pending'] = False
    state['course_stop_armed'] = False
    state['course_clear_frames'] = 0
    state['course_marker_stamp'] = -1.0
    state['course_finished'] = False
    state['relative_future'] = None
    state['relative_replan_future'] = None
    state['parallel_scene'] = None
    state['parallel_parking'] = None
    state['parallel_future'] = None
    state['reason'] = 'startup'
    state['red'] = False
    state['estop'] = False
    state['lane'] = []
    state['lane_target'] = None
    state['lane_curve_lock'] = None
    state['lane_preview'] = None
    state['lane_speed_state'] = None
    state['lane_stamp'] = -1.0
    state['lane_confidence'] = 0.0
    state['lane_boundaries'] = {}
    state['lane_recovery'] = None
    state['scan'] = None
    state['sign_stamp'] = -1.0
    state['sign_label'], state['sign_count'] = '', 0
    state['green_start_window'] = []
    state['direction_window'] = []
    state['uturn_sign_window'] = []
    state['sign_info'] = dict(label='',confidence=0.0,votes=0,stamp=-1.0,decision='not_observed')
    state['pending'] = None
    state['pending_at'] = 0.0
    state['parking_route_ready'] = not cfg['wait_green']
    state['next_direction'], state['next_direction_at'] = None,0.0
    state['route_action_started'] = None
    state['route_sign_last_seen'] = {}
    state['route_sign_rearmed'] = False
    state['right_sign_locked'] = False
    state['right_sign_rearmed_at'] = None
    state['obstacle_check'] = dict(kind='not_checked')
    state['marker'] = None
    state['blue_consumed'] = False
    state['blue_approach'] = None
    state['blue_prealign'] = None
    state['blue_clear_since'] = None
    state['blue_clear_frames'] = 0
    state['marker_candidate'] = None
    state['last_blue_trigger'] = None
    state['parking_straight_travel'] = None
    state['parking_marker'] = None
    state['consumed_marker'] = None
    state['completed_at_pose'] = None
    state['last_completed_action'] = None
    state['slot'] = None
    state['slot_stamp'] = -1.0
    state['slot_locked'] = False
    state['parking_candidates'] = []
    state['parking_candidate_stamp'] = -1.0
    state['parking_front_frames'] = 0
    state['parking_trigger_at'] = -1.0
    state['parking_trigger_pose'] = None
    state['parking_diagnostics'] = []
    state['parking_detection'] = {}
    state['parking_entry'] = None
    state['parking_sign'] = None
    state['parking_lines'], state['parking_lines_stamp'] = [],-1.0
    state['ground_stamp'] = -1.0
    state['ground_stamps'] = {}
    state['wait_until'] = 0.0
    state['follower'] = None
    state['action'] = None
    state['action_source'] = None
    state['action_started'] = 0.0
    state['exit_pose'] = None
    state['exit_count'], state['exit_stamp'] = 0, -1.0
    state['gap_origin'], state['gap_start'] = None, 0.0
    state['gap_steer'] = 0.0
    state['applied_steer'], state['applied_stamp'] = 0.0, -1.0
    state['issued_steer'] = None
    state['left_boundary'], state['left_boundary_stamp'] = [], -1.0
    state['left_fit_diagnostic'] = None
    state['right_lock'] = None
    state['right_completed_count'] = 0
    state['left_lock'] = None
    state['straight_search'] = None
    state['right_tail_origin'] = None
    state['lane_source'] = 'center'
    state['follow_left_boundary'] = False
    state['left_reference'] = None
    state['finished_count'] = 0
    state['finish_pose_stamp'] = -1.0
    state['retry_at'] = 0.0
    state['executor'] = _SingleJobExecutor()
    state['future'] = None
    state['uturn'] = None
    state['front_markers'], state['front_marker_stamp'] = [], -1.0
    state['front_blue_lines'] = []
    state['front_blue_image_lines'] = []
    state['rear_markers'], state['rear_marker_stamp'] = [], -1.0
    state['rear_blue_lines'] = []

    return state

def close(ctx):
    if ctx.timed_parking is not None:
        ctx.timed_parking.close()
    ctx.executor.shutdown(wait=True)


def set_pose(ctx, pose, stamp):
    if stamp >= ctx.pose_stamp and all(_isfinite(v) for v in pose):
        if ctx.last_blue_trigger is not None and 'travelled_m' in ctx.last_blue_trigger:
            ctx.last_blue_trigger['travelled_m'] += distance(ctx.pose,pose)
        if ctx.parking_straight_travel is not None:
            ctx.parking_straight_travel['travelled_m'] += distance(ctx.pose,pose)
        ctx.pose, ctx.pose_stamp = tuple(pose), stamp


def start_follow(ctx, path, action, now):
    # Blue approach/stop and the maneuver belong to one sign-owned action.
    # Its queue clock must survive the maneuver's separate timing reset.
    if (ctx.pending != action and not
            (ctx.action == action and ctx.state in
             ('BLUE_APPROACH', 'BLUE_STOP', 'INTERSECTION_WAIT'))):
        ctx.route_action_started = now
    ctx.lane_speed_state = None
    ctx.lane_preview = None
    ctx.lane_curve_lock = None
    ctx.lane_recovery = None
    ctx.startup_curve_lock = None
    ctx.follower = Follower(path,ctx.cfg,parking=action == 'PARKING',
        full_lock=action=='LEFT' and ctx.cfg.get('left_turn_full_lock',False))
    ctx.follow_left_boundary = False
    ctx.left_reference = None
    if action != 'BYPASS':
        ctx.uturn_exit_straight = False
    ctx.action, ctx.action_started = action, now
    ctx.state = 'PARKING' if action == 'PARKING' else 'MANEUVER'
    if action == 'BYPASS':
        ctx.action_source = 'obstacle'
    else:
        ctx.pending = None
    if action == 'STRAIGHT':
        # Start the S-entry budget at fixed straight, not at blue approach.
        # Keep this checkpoint after straight_search is cleared at handoff.
        ctx.parking_straight_travel = dict(stamp=now,vehicle_pose=list(ctx.pose),
            travelled_m=0.,source='fixed_straight',travel_source=ctx.cfg['pose_mode'])
        # Consume sign state at entry, not at the later lane handoff. Keep the
        # running action itself intact and retain sign_stamp for stale rejection.
        route_started = (ctx.route_action_started if ctx.route_action_started is not None
                         else ctx.action_started)
        if ctx.next_direction_at <= route_started+ctx.cfg['sign_timeout']:
            ctx.next_direction,ctx.next_direction_at = None,0.0
        ctx.pending_at=0.0
        ctx.sign_label,ctx.sign_count='',0
        ctx.direction_window,ctx.uturn_sign_window=[],[]
        ctx.sign_info=dict(label='',confidence=0.,stamp=now,votes=0,
                           decision='consumed_by_straight')
    ctx.exit_pose = path[-1][:3]
    ctx.exit_count, ctx.exit_stamp = 0, -1.0
    ctx.right_tail_origin = None
    ctx.left_lock = (dict(origin=ctx.pose,phase='ENTRY',progress_deg=0.)
                      if action == 'LEFT' and (ctx.cfg.get('left_turn_full_lock',False) or ctx.cfg.get('left_timed_enabled',False)) else None)
    ctx.right_lock = (dict(origin=ctx.pose,phase='ENTRY',last_seen=now,
                            last_pose=ctx.pose,lost_steer=None)
                       if action == 'RIGHT' and (ctx.cfg.get('right_turn_full_lock',False) or ctx.cfg.get('right_timed_enabled',False)) else None)
    ctx.straight_search = (dict(origin=ctx.pose,started=now,phase='STRAIGHT_DISTANCE',
                                speed_raw=ctx.cfg.get('straight_speed_raw',12),
                                timeout_s=ctx.cfg.get('straight_search_timeout',10.0))
                            if action == 'STRAIGHT' else None)


def resume_lane(ctx):
    if ctx.action == 'RIGHT':
        ctx.right_completed_count += 1
    ctx.lane_speed_state = None
    ctx.lane_preview = None
    ctx.lane_curve_lock = None
    ctx.startup_curve_lock = None
    ctx.uturn_exit_straight = (ctx.action == 'UTURN' or
                              (ctx.action == 'BYPASS' and ctx.uturn_exit_straight))
    if ctx.action in ('LEFT', 'RIGHT', 'STRAIGHT', 'UTURN'):
        ctx.parking_route_ready = True
    if ctx.action=='LEFT':
        # Reuse the existing bounded reversal transition at the visual handoff.
        # Same-side steering remains immediate; ordinary lane tuning is intact.
        ctx.lane_recovery=dict(stamp=ctx.pose_stamp)
    if ctx.action == 'BYPASS' and ctx.pending is not None:
        # An obstacle may interrupt lane following after a sign was cached.
        # Preserve that one existing instruction; bypass cannot acquire one.
        pending, pending_at = ctx.pending, ctx.pending_at
    else:
        pending, pending_at = None, 0.0
    route_started = (ctx.route_action_started if ctx.route_action_started is not None
                     else ctx.action_started)
    right_released = ctx.right_sign_rearmed_at if ctx.action == 'RIGHT' else None
    if (ctx.action in ('LEFT','RIGHT','STRAIGHT','UTURN') and pending is None and
            ctx.next_direction is not None and
            (ctx.next_direction_at > route_started+ctx.cfg['sign_timeout'] or
             (right_released is not None and
              ctx.next_direction_at > max(route_started, right_released)))):
        pending,pending_at=ctx.next_direction,ctx.next_direction_at
    ctx.last_completed_action = ctx.action if ctx.action_source != 'blue_default' else None
    ctx.completed_at_pose = ctx.pose
    ctx.state = 'LANE'
    ctx.action, ctx.follower = None,None
    ctx.action_source = None
    ctx.pending,ctx.pending_at = pending,pending_at
    ctx.route_action_started = pending_at if pending is not None else None
    ctx.route_sign_rearmed = False
    ctx.right_sign_rearmed_at = None
    ctx.next_direction,ctx.next_direction_at = None,0.0
    ctx.action_started, ctx.wait_until = 0.0,0.0
    ctx.exit_pose = None
    ctx.marker, ctx.parking_marker = None,None
    ctx.marker_candidate = None
    ctx.blue_approach = None
    ctx.blue_prealign = None
    ctx.blue_clear_since, ctx.blue_clear_frames = None, 0
    ctx.sign_label, ctx.sign_count = '',0
    ctx.direction_window = []
    ctx.uturn_sign_window = []
    ctx.gap_origin = None
    ctx.uturn = None

    ctx.right_lock = None
    ctx.left_lock = None
    ctx.straight_search = None
    ctx.right_tail_origin = None


def dispatch(ctx, now):
    if ctx.action is not None:
        return None
    if (ctx.pending == 'PARKING' and ctx.cfg.get('parking_enabled',True) and
            ctx.cfg.get('parking_mode') == 'forward_center' and
            ctx.cfg.get('parking_entry_style') == 'S'):
        ctx.action_source='sign'
        return ctx.call('parking','start_explicit_parking',now,None,'sign')
    if (ctx.pending == 'PARKING' and ctx.cfg.get('parking_mode') == 'timed_sequence'
            and ctx.cfg.get('parking_enabled',True)):
        from robot.parking.timed_core import TimedParking
        ctx.timed_parking = TimedParking(ctx.cfg,now)
        ctx.state,ctx.action,ctx.action_source = 'TIMED_PARKING','PARKING','sign'
        ctx.action_started = now
        ctx.pending,ctx.pending_at = None,0.
        ctx.marker,ctx.parking_marker,ctx.marker_candidate = None,None,None
        ctx.blue_approach,ctx.blue_prealign = None,None
        ctx.reason = ctx.cfg['parking_slot']+'_WAIT_CAMERA'
        return ctx.call('motion','stop',ctx.reason)
    if ((ctx.marker is not None or ctx.parking_marker is not None) and
            not 0 <= now-ctx.front_marker_stamp <= ctx.cfg.get('ground_timeout',1.25)):
        return ctx.call('motion', 'stop', 'junction_front_stale')
    if ctx.pending == 'PARKING' and ctx.cfg.get('parking_mode') != 'forward_center':
        ctx.action_source = 'sign'
        auto = ctx.cfg['parking_slot'] == 'AUTO'
        parallel = ctx.cfg.get('parking_mode') in ('parallel_reverse','forward_plan')
        if ctx.cfg.get('parking_blue_required',True):
            marker = ctx.marker if auto or parallel else ctx.parking_marker
            if (not marker or marker[1] < ctx.pending_at or
                    now-marker[1] > ctx.cfg['marker_memory_s']):
                return None
            x,y = local(ctx.pose,marker[0])
            if not -0.12 <= x <= ctx.cfg['marker_trigger_x'] or abs(y) > ctx.cfg['lane_width']:
                return None
            ctx.consumed_marker = marker[0]
            ctx.parking_marker = None
        if parallel:
            ctx.marker, ctx.pending = None, None
            ctx.state, ctx.action = 'PARALLEL_PARKING', 'PARKING'
            ctx.action_started = now
            ctx.wait_until = now+ctx.cfg.get('intersection_wait_s',0.0)
            ctx.parallel_parking, ctx.parallel_future = None, None
            return ctx.call('motion', 'stop', 'parallel_parking_blue_stop')
        if auto:
            ctx.marker = None
            ctx.pending = None
            ctx.state, ctx.action = 'PARKING_SCAN','PARKING'
            ctx.parking_trigger_at, ctx.parking_front_frames = now,0
            ctx.parking_trigger_pose = ctx.pose
            ctx.parking_candidates = []
            return ctx.call('motion', 'stop', 'long_blue_stop_observe_bays')
        return ctx.call('parking', 'start_parking', now)
    action = ctx.pending
    fallback = action is None and ctx.cfg.get('blue_default_straight',False)
    if action is None and not fallback:
        if ctx.marker is not None:
            ctx.blue_consumed = True
        ctx.marker = None  # This visible line has spent its one opportunity.
        return None
    if not ctx.marker or not 0 <= now-ctx.marker[1] <= min(ctx.cfg['marker_memory_s'],2.5):
        return None
    x,y = local(ctx.pose,ctx.marker[0])
    if not 0 < x <= ctx.cfg.get('straight_align_distance',1.6) or abs(y) > ctx.cfg['lane_width']:
        return None
    lines = [line for line in ctx.front_blue_lines
             if distance(line['point'],ctx.marker[0]) < .05]
    if not lines:
        return None  # A marker center alone cannot establish body heading.
    yaw = min(lines,key=lambda line:distance(line['point'],ctx.marker[0]))['yaw']
    ctx.last_blue_trigger = dict(stamp=now, observed_stamp=ctx.marker[1],
        observation_age_s=now-ctx.marker[1], point_local=[x,y],
        point_world=list(ctx.marker[0]), action=action or 'STRAIGHT',
        vehicle_pose=list(ctx.pose), travelled_m=0., travel_source=ctx.cfg['pose_mode'],
        source='blue_default' if fallback else 'cached_sign')
    ctx.parking_straight_travel = None
    ctx.consumed_marker = ctx.marker[0]
    ctx.marker = None
    ctx.marker_candidate = None
    ctx.blue_consumed = True
    ctx.action_source = 'blue_default' if fallback else 'sign'
    if fallback:
        action = 'STRAIGHT'
    # Reserve the observed stripe; align the body before counting entry travel.
    if math.cos(yaw-ctx.pose[2]) < 0:
        yaw = wrap(yaw+math.pi)
    ctx.blue_approach = dict(point=ctx.consumed_marker,yaw=yaw,
        observed_stamp=ctx.last_blue_trigger['observed_stamp'],started=now,
        origin=ctx.pose,phase='STOP_LINE' if (action in ('STRAIGHT','UTURN') or (action=='LEFT' and ctx.cfg.get('left_timed_enabled',False)) or (action=='RIGHT' and ctx.cfg.get('right_timed_enabled',False))) else 'ALIGN',
        align_frames=0,align_stamp=-1.,
        image_timed=ctx.cfg.get('blue_timed_enabled',False) and action in ('LEFT','RIGHT','STRAIGHT','UTURN'))
    early=ctx.blue_prealign
    if (ctx.blue_approach['image_timed'] and early is not None and
            early['action']==action and distance(early['point'],ctx.consumed_marker)<=.15 and
            0<=now-early['stamp']<=ctx.cfg['blue_stop_test']['camera_timeout_s']):
        # Keep the alignment votes and clock accumulated before junction confirmation.
        ctx.blue_approach['image_timing']=early['sequence']
        ctx.blue_approach['prealigned_during_confirmation']=True
    ctx.blue_prealign=None
    if (action == 'PARKING' and ctx.parking_sign is not None and
            ctx.pending_at-ctx.cfg['sign_timeout'] <= ctx.parking_sign['stamp'] <= now):
        ctx.blue_approach['parking_anchor'] = ctx.parking_sign['point']
    ctx.action,ctx.pending,ctx.action_started = action,None,now
    if ctx.route_action_started is None:
        ctx.route_action_started = now
    # A confirmed next blue ends straight driving after the completed U-turn.
    ctx.uturn_exit_straight = False
    ctx.follow_left_boundary = False
    ctx.left_reference = None
    ctx.lane_curve_lock = None
    ctx.startup_curve_lock = None
    ctx.state = 'BLUE_APPROACH'
    return ctx.call('turn','blue_approach_tick',now)


def begin_blue_action(ctx, now):
    """Called once after the entry stop and stationary wait."""
    action = ctx.action
    parking_anchor = (ctx.blue_approach or {}).get('parking_anchor')
    ctx.blue_approach = None
    if action == 'PARKING':
        if (ctx.cfg.get('parking_mode') == 'forward_center' and
                ctx.cfg.get('parking_entry_style') in ('S','T')):
            return ctx.call('parking','start_explicit_parking',now,parking_anchor,'blue')
        ctx.parking_entry = ForwardParking(ctx.cfg,ctx.pose,now)
        ctx.parking_entry.sign_anchor = parking_anchor
        if ctx.parking_sign is not None and 0 <= now-ctx.parking_sign['stamp'] <= ctx.cfg['sign_timeout']:
            ctx.parking_entry.sign_anchor = ctx.parking_sign['point']
        ctx.state,ctx.action_started = 'PARKING',now
        return ctx.call('motion','stop','parking_blue_entry')
    if action == 'UTURN':
        ctx.timed_uturn = None
        ctx.state, ctx.action, ctx.action_started = 'UTURN',action,now
        ctx.pending = None
        ctx.uturn = dict(start_line=ctx.consumed_marker,
                          target_yaw=wrap(ctx.pose[2]+math.pi),
                          reverse_origin=None)
        ctx.call('uturn', 'uturn_phase', 'WAIT_START',now)
        ctx.wait_until = now  # The shared BLUE_STOP already waited.
        return ctx.call('motion', 'stop', 'uturn_wait_start')
    path = ([tuple(ctx.pose)+(1,0.)] if ((action=='LEFT' and ctx.cfg.get('left_timed_enabled',False)) or
                (action=='RIGHT' and ctx.cfg.get('right_timed_enabled',False)))
            else intersection_path(ctx.pose,action,ctx.cfg))
    ctx.call('mission', 'start_follow', path,action,now)
    if action == 'STRAIGHT':
        ctx.straight_search['phase'] = 'STRAIGHT_DISTANCE'
        return ctx.call('turn','straight_search_tick',now)
    return 0, 0.0


def begin_startup(ctx, now):
    ctx.lane_speed_state = None
    ctx.lane_curve_lock = None
    ctx.parking_route_ready = False
    if ctx.pending == 'PARKING':
        ctx.pending, ctx.pending_at = None, 0.0
    ctx.park_line_side = None
    ctx.parking_sign = None
    if ctx.cfg.get('startup_follow_lane',False):
        # Command odometry cannot locate the physical bend. Use measured
        # centers from release rather than holding neutral for a distance.
        ctx.state = 'LANE'
        ctx.startup_started = None
        ctx.startup_origin = None
        ctx.startup_blue_point = None
        ctx.startup_curve_lock = None
        ctx.gap_origin = None
        ctx.marker, ctx.parking_marker = None, None
        return
    ctx.state = 'STARTUP_STRAIGHT'
    ctx.startup_started = now
    ctx.startup_origin = ctx.pose
    ctx.startup_blue_point = None
    ctx.startup_curve_lock = None
    ctx.marker, ctx.parking_marker = None, None






def _interrupt_explicit_parking(ctx,reason):
    if (ctx.state == 'PARKING' and ctx.action == 'PARKING' and
            ctx.cfg.get('parking_mode') == 'forward_center' and
            ctx.cfg.get('parking_entry_style') == 'T' and ctx.parking_entry is not None):
        ctx.parking_entry.abort(reason)
        ctx.state='FAULT'


def _parking_distance_guard(ctx, command=None):
    cfg=ctx.cfg
    selected=(cfg.get('parking_mode')=='forward_center' and
              cfg.get('parking_slot') in ('P4','P5') and
              cfg.get('parking_entry_style') in ('S','T'))
    target=(ctx.action=='PARKING' or ctx.pending=='PARKING' or
            ctx.next_direction=='PARKING')
    if not selected or not target:
        return None
    straight=cfg.get('parking_entry_style')=='S'
    trigger=ctx.parking_straight_travel if straight else ctx.last_blue_trigger
    reason=None
    if trigger is None or 'travelled_m' not in trigger:
        if ctx.state=='PARKING':
            reason='parking_straight_origin_missing' if straight else 'parking_blue_origin_missing'
        else:
            return None  # Wait for S fixed straight or T's next blue trigger.
    else:
        limit=min(1.7,cfg.get('parking_blue_max_travel_m',1.7))
        travelled=trigger['travelled_m']
        reserve=(abs(command[0])*speed_gain(cfg,command[0])*.25
                 if command is not None and command[0] else 0.)
        trigger.update(parking_limit_m=limit,remaining_m=limit-travelled,
                       command_reserve_m=reserve)
        if ctx.parking_entry is not None:
            ctx.parking_entry.debug.update(travel_origin='fixed_straight' if straight else 'blue_trigger',
                travel_m=travelled,travel_limit_m=limit,travel_remaining_m=limit-travelled)
        if travelled+reserve>=limit-1e-9:
            reason='parking_straight_distance_limit' if straight else 'parking_blue_distance_limit'
    if reason is None:
        return None
    entry=ctx.parking_entry
    if entry is not None:
        if cfg['parking_entry_style']=='T':
            entry.abort(reason)
        else:
            entry.inner.stopped(reason,fault=True)
            entry.inner.phase='FAULT'
            entry.inner.debug['phase']='FAULT'
    ctx.state='FAULT'
    return ctx.call('motion','stop',reason)


def tick(ctx, now):
    if ctx.estop or ctx.red or ctx.state in ('FAULT','FINISHED','WAIT_GREEN'):
        return _tick(ctx,now)
    stop=_parking_distance_guard(ctx)
    if stop is not None:
        return stop
    command=_tick(ctx,now)
    if ctx.state not in ('FAULT','FINISHED'):
        stop=_parking_distance_guard(ctx,command)
        if stop is not None:
            return stop
    return command


def _tick(ctx, now):
    cfg = ctx.cfg
    if ctx.action is not None or ctx.state not in ('LANE','GAP','WAIT_OBSTACLE'):
        ctx.lane_curve_lock = None
    ctx.obstacle_check = dict(kind='not_checked')
    if ctx.estop:
        _interrupt_explicit_parking(ctx,'emergency_stop')
        if ctx.timed_parking is not None and not ctx.timed_parking.finished:
            ctx.timed_parking.abort('emergency_stop')
            ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'emergency_stop')
    if ctx.state == 'FINISHED':
        if ctx.timed_parking is not None:
            return ctx.call('motion','stop',ctx.timed_parking.reason)
        if (cfg.get('parking_mode')=='forward_center' and
                cfg.get('parking_entry_style') in ('S','T') and ctx.parking_entry is not None):
            return ctx.call('motion','stop',ctx.parking_entry.reason)
        if cfg.get('parking_mode')=='forward_center' and ctx.parking_entry is not None:
            return ctx.call('motion','stop','parking_at_bottom_clearance')
        if ctx.course_finished:return ctx.call('motion', 'stop', 'uturn_course_exit_blue_stop')
        if ctx.timed_uturn is not None:
            return ctx.call('motion', 'stop', 'uturn_trial_complete_stop')
        return ctx.call('motion', 'stop', 'four_wheels_inside_terminal')
    if ctx.state == 'FAULT':
        return ctx.call('motion', 'stop', ctx.reason)
    if ctx.red or ctx.state == 'WAIT_GREEN':
        _interrupt_explicit_parking(ctx,'red_latched' if ctx.red else 'waiting_green')
        if ctx.timed_parking is not None and not ctx.timed_parking.finished:
            ctx.timed_parking.abort('red_latched')
            ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'red_latched' if ctx.red else 'waiting_green')
    if cfg['pose_mode'] == 'odom' and not 0 <= now-ctx.pose_stamp <= cfg['odom_timeout']:
        _interrupt_explicit_parking(ctx,'odom_stale')
        if ctx.timed_parking is not None and not ctx.timed_parking.finished:
            ctx.timed_parking.abort('odom_stale')
            ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'odom_stale')
    # Exceptions apply to the active category, never to a queued next action.
    lidar_disabled = lidar_guard_disabled_reason(
        cfg, ctx.state, ctx.action, ctx.timed_bypass_completed)
    if lidar_disabled is None and not ctx.call('obstacle', 'scan_ready', now):
        _interrupt_explicit_parking(ctx,'scan_missing_or_stale')
        ctx.obstacle_check = dict(kind='unknown',reason='scan_missing_or_stale')
        if ctx.timed_parking is not None and not ctx.timed_parking.finished:
            ctx.timed_parking.abort('scan_missing_or_stale')
            ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'scan_missing_or_stale')
    if ctx.state == 'TIMED_PARKING':
        task = ctx.timed_parking
        command = task.command(now)
        ctx.reason = task.reason
        if task.finished:
            ctx.state = 'FINISHED' if task.reason == cfg['parking_slot']+'_COMPLETE' else 'FAULT'
            return ctx.call('motion','stop',task.reason)
        result = ctx.call('obstacle','checked_command',command,now,False)
        if command[0] and result != command:
            # A fixed-time stage cannot resume after a safety stop with a
            # partially elapsed trajectory. Keep the vehicle stopped.
            task.abort(ctx.reason)
            ctx.state = 'FAULT'
            return ctx.call('motion','stop',task.reason)
        ctx.reason = task.reason
        return result
    if ctx.state=='TIMED_BYPASS':
        return ctx.call('obstacle','timed_bypass_tick',now)
    if ctx.state in ('BLUE_APPROACH','BLUE_STOP'):
        return ctx.call('turn','blue_approach_tick',now)
    if ctx.course_stop_pending and ctx.call('uturn', 'course_blue_stop', now):
        return ctx.call('motion', 'stop', 'uturn_course_exit_blue_stop')
    if ctx.state == 'PARALLEL_PARKING':
        return ctx.call('parallel_parking', 'parallel_parking_tick', now)
    if ctx.state in ('PARKING_SCAN','AUTO_PLANNING'):
        return ctx.call('parking', 'auto_parking_tick', now)
    if ctx.state == 'STARTUP_STRAIGHT':
        return ctx.call('uturn', 'startup_tick', now)
    if ctx.state == 'PARKING' and ctx.parking_entry is not None:
        if (cfg.get('parking_mode')=='forward_center' and
                cfg.get('parking_entry_style') in ('S','T')):
            return ctx.call('parking','explicit_parking_tick',now)
        entry = ctx.parking_entry
        if (cfg.get('parking_mode')=='forward_center' and
                not entry.associate and
                entry.bottom is None and
                entry.phase=='APPROACH'):
            # Single-line approach ends once bay centering starts. Later loss
            # of the pair means straight ahead, never chasing a side again.
            line_command=ctx.call('lane','park_line_command',now)
            ctx.state='PARKING'
            if line_command[0]:
                entry.debug=dict(phase=entry.phase,side=ctx.park_line_side,
                    boundary_age_s=now-ctx.lane_stamp,bottom_seen=False)
                command=(cfg.get('parking_entry_speed_raw',12),line_command[1])
                result=ctx.call('obstacle','checked_command',command,now,allow_bypass=False)
                if result[0]:ctx.reason='parking_follow_single_line'
                return result
        command = ctx.parking_entry.command(now,ctx.pose)
        if cfg.get('parking_mode')=='forward_center':
            ctx.lane_source = ('parking_pair' if entry.reason=='parking_center_between_sides'
                               else 'parking_straight' if entry.reason=='parking_missing_white_straight'
                               else 'parking_stop')
        if ctx.parking_entry.reason is not None:
            ctx.reason = ctx.parking_entry.reason
        if ctx.parking_entry.status is not None:
            ctx.state = ctx.parking_entry.status
        result = ctx.call('obstacle', 'checked_command', command,now,allow_bypass=False)
        if result == command and entry.reason is not None:
            ctx.reason = entry.reason
        return result
    if ctx.state == 'INTERSECTION_WAIT':
        if now < ctx.wait_until:
            return ctx.call('motion', 'stop', 'blue_stop_wait_'+ctx.action.lower())
        ctx.call('mission', 'start_follow', intersection_path(ctx.pose,ctx.action,cfg),ctx.action,now)
    if (ctx.state in ('PLANNING','MANEUVER','PARKING','REACQUIRE','UTURN') and
            not (ctx.state == 'MANEUVER' and ctx.straight_search is not None) and
            not (ctx.state == 'MANEUVER' and ctx.action == 'RIGHT' and
                 cfg.get('right_timed_enabled',False) and ctx.right_lock is not None and
                 ctx.right_lock.get('phase') in ('WAIT_LANE','ALIGN_LANE')) and
            now-ctx.action_started > cfg['action_timeout']):
        ctx.state = 'FAULT'
        return ctx.call('motion', 'stop', 'action_timeout')
    if ctx.state == 'UTURN':
        return ctx.call('uturn', 'uturn_tick', now)
    if ctx.state == 'PLANNING':
        if not ctx.future.done():
            return ctx.call('motion', 'stop', 'planning')
        try:
            path, reason = ctx.future.result()
        except Exception as exc:
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', 'planner_error:'+str(exc))
        if not path:
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', reason)
        if (distance(ctx.pose,path[0]) > cfg['slot_match_distance'] or
                abs(wrap(ctx.pose[2]-path[0][2])) > cfg['exit_yaw_tolerance']):
            ctx.state = 'FAULT'
            return ctx.call('motion', 'stop', 'pose_changed_during_planning')
        ctx.call('mission', 'start_follow', path,ctx.action,now)
    if ctx.state in ('MANEUVER','PARKING'):
        command = ctx.call('motion', 'follow_path_tick', now)
        if command is not None:
            return command
    if ctx.state == 'REACQUIRE':
        yaw_ok = abs(wrap(ctx.pose[2]-ctx.exit_pose[2])) <= cfg['exit_yaw_tolerance']
        if ctx.call('turn', 'exit_lane_confirmed', now):
            ctx.call('mission', 'resume_lane')
        else:
            if distance(ctx.pose,ctx.exit_pose) >= cfg['exit_search_distance'] or not yaw_ok:
                return ctx.call('motion', 'stop', 'correct_exit_not_found')
            return ctx.call('obstacle', 'checked_command', (cfg['speed_raw']['gap'],0.0),now,False)
    if ctx.state == 'WAIT_SLOT':
        return ctx.call('parking', 'start_parking', now)
    # A confirmed sign stays latched until consumed by its action. Expiring
    # it here would reopen the cache before that instruction had executed.
    if ctx.pending or cfg.get('blue_default_straight',False):
        result = ctx.call('mission', 'dispatch', now)
        if result is not None:
            return result
    if ctx.uturn_exit_straight and ctx.action is None:
        # Follow road paint while reserving unconfirmed blue for later dispatch.
        ctx.blue_prealign = None
        command = ctx.call('lane', 'lane_command', now)
        output = ctx.call('obstacle', 'checked_command', command, now, True)
        if command[0] and output == command:
            ctx.reason = 'uturn_exit_straight_wait_blue'
        return output
    from robot.turn.blue_timed import prealign_tick, pending_approach
    result=prealign_tick(ctx,now)
    if result is not None:
        return result
    command=ctx.call('lane','lane_command',now)
    slow_approach=pending_approach(ctx)
    if slow_approach:
        # Slow along the measured lane as soon as a direction sign is latched.
        # This avoids entering the first blue frames at ordinary cruise speed.
        limit=cfg['blue_stop_test']['align_speed_raw']
        command=(math.copysign(min(abs(command[0]),limit),command[0]),command[1])
    output=ctx.call('obstacle','checked_command',command,now,True)
    if slow_approach and command[0] and output==command:
        ctx.reason='direction_sign_slow_approach'
    return output
