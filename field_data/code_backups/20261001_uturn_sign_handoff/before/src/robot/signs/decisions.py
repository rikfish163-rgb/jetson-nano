"""signs module: explicit context input; coordinator applies returned updates."""
from __future__ import division

from robot.common.geometry import _isfinite, local



FIELDS = ('action', 'action_started', 'cfg', 'completed_at_pose', 'course_stop_pending', 'direction_window',
 'green_start_window', 'last_completed_action', 'marker', 'next_direction',
 'next_direction_at', 'park_line_side', 'parking_marker', 'parking_route_ready', 'pending', 'pending_at', 'pose',
 'red', 'sign_count', 'sign_info', 'sign_label', 'sign_stamp', 'state', 'straight_search',
 'uturn_sign_window')
CALLS = (('mission', 'begin_startup'),)
OPERATIONS = ('observe_sign',)

def observe_sign(ctx, label, confidence, stamp, now):
    labels = ('RED','GREEN','LEFT','RIGHT','STRAIGHT','UTURN','PARKING')
    ctx.sign_info = dict(label=label if label in labels else '',
                          confidence=confidence if _isfinite(confidence) else None,
                          stamp=stamp,votes=ctx.sign_count,decision='stale_or_duplicate')
    if stamp <= ctx.sign_stamp or not 0 <= now-stamp <= ctx.cfg['sign_timeout']:
        return
    if stamp-ctx.sign_stamp > ctx.cfg['sign_timeout']:
        ctx.sign_label, ctx.sign_count = '', 0
        ctx.green_start_window = []
        ctx.direction_window = []
        ctx.uturn_sign_window = []
    ctx.sign_stamp = stamp
    if (ctx.cfg.get('uturn_course_test',False) and
            (label in ('LEFT','RIGHT','STRAIGHT','PARKING') or
             (label=='UTURN' and (ctx.state=='WAIT_GREEN' or ctx.course_stop_pending)))):
        ctx.green_start_window=[]
        ctx.sign_info.update(votes=0,decision='uturn_course_sign_ignored')
        return
    if label != 'GREEN' or not ctx.cfg['sign_confidence'] <= confidence <= 1.0:
        ctx.green_start_window = []
    if (label == 'PARKING' and
            (ctx.state in ('WAIT_GREEN', 'STARTUP_STRAIGHT') or not ctx.parking_route_ready)):
        # Green startup and its first lane leg cannot arm parking. Only a
        # completed junction action opens a new, fresh parking vote sequence.
        ctx.sign_label, ctx.sign_count = '', 0
        ctx.direction_window, ctx.uturn_sign_window = [], []
        ctx.sign_info.update(votes=0, decision='parking_wait_route_action')
        return
    bypass_straight = (label == 'STRAIGHT' and ctx.state == 'TIMED_BYPASS'
                       and ctx.action == 'BYPASS' and ctx.pending is None)
    straight_active = (ctx.action == 'STRAIGHT' and ctx.state == 'MANEUVER'
                       and ctx.straight_search is not None)
    if straight_active:
        # The next board can enter view before the mandatory 1.25 m ends.
        # Queue consecutive STRAIGHT votes in the latter part of that segment;
        # a distinct, confirmed blue line is still required to execute them.
        minimum = max(1.25, ctx.cfg.get('straight_distance', 1.25))
        travelled = local(ctx.straight_search['origin'], ctx.pose)[0]
        if (label == 'STRAIGHT' and ctx.cfg['sign_confidence'] <= confidence <= 1.0
                and stamp > ctx.action_started and travelled >= .6*minimum):
            if ctx.next_direction == 'STRAIGHT':
                ctx.sign_info.update(votes=0, decision='next_straight_latched')
                return
            ctx.direction_window.append(('STRAIGHT', stamp))
            required = int(ctx.cfg.get('direction_sign_votes', 2))
            ctx.direction_window = ctx.direction_window[-required:]
            ctx.sign_info.update(votes=len(ctx.direction_window), decision='next_straight_voting')
            if len(ctx.direction_window) == required:
                ctx.next_direction, ctx.next_direction_at = 'STRAIGHT', now
                ctx.sign_info['decision'] = 'next_straight_pending'
            return
        ctx.direction_window = []
    directions=('LEFT','RIGHT','STRAIGHT','UTURN')
    if ctx.action is None and ctx.pending in directions:
        # Before blue commits an action, fresh consecutive observations may
        # correct the cached direction; an active action remains locked.
        if label in directions and ctx.cfg['sign_confidence'] <= confidence <= 1.0:
            if label == ctx.pending:
                ctx.direction_window=[]
                ctx.sign_info.update(votes=0,decision='pending_latched')
                return
            if ctx.direction_window and ctx.direction_window[-1][0] != label:
                ctx.direction_window=[]
            ctx.direction_window.append((label,stamp))
            required=int(ctx.cfg.get('direction_sign_votes',2))
            ctx.direction_window=ctx.direction_window[-required:]
            ctx.sign_info.update(votes=len(ctx.direction_window),decision='direction_correction_voting')
            if len(ctx.direction_window)==required:
                ctx.pending,ctx.pending_at=label,now
                ctx.direction_window=[]
                ctx.sign_label,ctx.sign_count='',0
                ctx.sign_info['decision']='corrected_pending'
            return
        ctx.direction_window=[]
    if (label in ('LEFT','RIGHT','STRAIGHT','UTURN','PARKING') and
            (ctx.pending is not None or ctx.action is not None) and not bypass_straight):
        # Do not carry observations made during one action into the next.
        ctx.sign_label,ctx.sign_count = '',0
        ctx.direction_window,ctx.uturn_sign_window = [],[]
        if not straight_active:
            ctx.next_direction,ctx.next_direction_at = None,0.0
        ctx.sign_info.update(votes=0,decision='action_sign_ignored')
        return
    threshold = ctx.cfg.get('red_sign_confidence',.8) if label == 'RED' else ctx.cfg['sign_confidence']
    valid = label in labels and threshold <= confidence <= 1.0
    if not valid:
        ctx.sign_label, ctx.sign_count = '', 0
        ctx.sign_info.update(votes=0,decision='low_confidence_or_unknown')
        return
    previous_label, previous_count = ctx.sign_label, ctx.sign_count
    ctx.sign_label, ctx.sign_count = label,1
    ctx.sign_info.update(votes=1,decision='voting')
    ctx.sign_info['decision'] = 'ignored_in_current_state'
    if label == 'RED':
        ctx.red = True
        ctx.sign_info['decision'] = 'red_stop'
    elif label == 'GREEN':
        if ctx.state == 'WAIT_GREEN':
            ctx.green_start_window.append(stamp)
            ctx.sign_count = len(ctx.green_start_window)
            ctx.sign_info.update(votes=ctx.sign_count,decision='green_start_voting')
            if ctx.sign_count < ctx.cfg['sign_votes']:
                return
            ctx.green_start_window = []
        ctx.red = False
        ctx.sign_info['decision'] = 'green_release'
        if ctx.state == 'WAIT_GREEN':
            ctx.call('mission', 'begin_startup', now)
    elif (bypass_straight or ctx.state in ('LANE','GAP','WAIT_OBSTACLE','STARTUP_STRAIGHT') or
          (ctx.state == 'WAIT_GREEN' and label == 'UTURN')):
        # Cache a confirmed U-turn before release; tick still waits for GREEN.
        if ctx.pending == 'PARKING':
            ctx.sign_info['decision'] = 'parking_latched'
            return  # armed parking waits for blue; RED/GREEN still handled above
        if label == 'PARKING' and not ctx.cfg.get('parking_enabled',True):
            ctx.sign_info['decision'] = 'parking_disabled'
            return
        if label in directions:
            ctx.sign_count=previous_count+1 if previous_label==label else 1
            ctx.sign_info.update(votes=ctx.sign_count,decision='direction_voting')
            if ctx.sign_count < ctx.cfg.get('direction_sign_votes',2):
                return
        if label == 'PARKING':
            ctx.sign_count = previous_count+1 if previous_label == label else 1
            ctx.sign_info.update(votes=ctx.sign_count,decision='parking_voting')
            if ctx.sign_count < ctx.cfg.get('parking_sign_votes',3):
                return
        # Consecutive junctions may carry the same direction sign. Accept
        # fresh post-action observations; consumed-marker filtering prevents
        # executing again at the previous blue line.
        if (ctx.cfg.get('latch_direction_sign', False) and
                ctx.pending in ('LEFT','RIGHT','STRAIGHT','UTURN')):
            ctx.sign_info['decision'] = 'pending_latched'
            return
        ctx.pending, ctx.pending_at = label, now
        ctx.sign_info['decision'] = 'stored_pending'
        if label == 'PARKING':
            ctx.marker, ctx.parking_marker = None,None
            ctx.park_line_side = None
