"""signs module: explicit context input; coordinator applies returned updates."""
from __future__ import division

from robot.common.geometry import _isfinite



FIELDS = ('action', 'action_started', 'route_action_started', 'route_sign_last_seen',
 'right_sign_locked', 'right_sign_rearmed_at', 'estop',
 'route_sign_rearmed', 'cfg', 'completed_at_pose', 'course_stop_pending', 'direction_window',
 'green_start_window', 'last_completed_action', 'marker', 'next_direction',
 'next_direction_at', 'park_line_side', 'parking_marker', 'parking_route_ready', 'pending', 'pending_at', 'pose',
 'red', 'right_lock', 'sign_count', 'sign_info', 'sign_label', 'sign_stamp', 'state', 'straight_search',
 'uturn_sign_window', 'uturn')
CALLS = (('mission', 'begin_startup'),)
OPERATIONS = ('observe_sign',)


def observe_sign(ctx, label, confidence, stamp, now, right_visible=None):
    labels = ('RED','GREEN','LEFT','RIGHT','STRAIGHT','UTURN','PARKING')
    directions = ('LEFT','RIGHT','STRAIGHT','UTURN')
    route_labels = directions + ('PARKING',)
    previous_stamp = ctx.sign_stamp
    previous_label = ctx.sign_label
    previous_count = ctx.sign_count
    ctx.sign_info = dict(label=label if label in labels else '',
                          confidence=confidence if _isfinite(confidence) else None,
                          stamp=stamp,votes=ctx.sign_count,decision='stale_or_duplicate')
    if stamp <= ctx.sign_stamp or not 0 <= now-stamp <= ctx.cfg['sign_timeout']:
        return
    if stamp-ctx.sign_stamp > ctx.cfg['sign_timeout']:
        ctx.sign_label, ctx.sign_count = '', 0
        previous_label, previous_count = '', 0
        ctx.green_start_window = []
        ctx.direction_window = []
        ctx.uturn_sign_window = []
    ctx.sign_stamp = stamp
    if ctx.estop and label in route_labels:
        ctx.sign_label, ctx.sign_count = '', 0
        ctx.sign_info.update(votes=0,decision='action_sign_ignored')
        return
    # Only fresh image evidence can release a consumed RIGHT. A confidence
    # dip or a camera/callback gap is not evidence of leaving the image.
    # Legacy observations without candidate metadata can report an empty frame.
    right_absent = (right_visible is False or
                    (right_visible is None and label == '' and confidence == 0.))
    released = ctx.right_sign_locked and right_absent
    if released:
        ctx.right_sign_locked = False
        ctx.route_sign_rearmed = True
        if ctx.right_sign_rearmed_at is None:
            ctx.right_sign_rearmed_at = stamp
        ctx.sign_label, ctx.sign_count = '', 0
        previous_label, previous_count = '', 0
        ctx.direction_window, ctx.uturn_sign_window = [], []
        ctx.route_sign_last_seen.pop('RIGHT', None)
    # Other route labels retain their existing visibility-gap policy.
    if label in route_labels and ctx.cfg['sign_confidence'] <= confidence <= 1.0:
        previous_seen = ctx.route_sign_last_seen.get(label, -1.)
        if (label != 'RIGHT' and label == (ctx.action or ctx.pending) and
                (previous_seen < 0. or stamp-previous_seen > ctx.cfg['sign_timeout'])):
            ctx.route_sign_rearmed = True
        ctx.route_sign_last_seen[label] = stamp
    if label == 'PARKING' and not ctx.cfg.get('parking_enabled',True):
        # Keep a disabled parking route distinguishable from a frame where
        # the detector produced no label. This diagnostic also takes
        # precedence over course/action state gates.
        ctx.sign_label, ctx.sign_count = '', 0
        ctx.direction_window, ctx.uturn_sign_window = [],[]
        ctx.sign_info.update(votes=0,decision='parking_disabled')
        return
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
    if (label in route_labels and ctx.action == 'UTURN' and
            ctx.cfg.get('uturn_trial_enabled',False)):
        # While rotating, boards on other legs can occupy the one-entry queue.
        # Only source frames captured after the final steering segment may vote.
        opened = (ctx.uturn or {}).get('next_sign_after')
        if opened is None or stamp <= opened:
            ctx.sign_label,ctx.sign_count = '',0
            ctx.direction_window,ctx.uturn_sign_window = [],[]
            ctx.sign_info.update(votes=0,decision='uturn_wait_final_heading')
            return
    if (label in route_labels and
            (ctx.pending is not None or ctx.action is not None or
             ctx.state == 'STARTUP_STRAIGHT')):
        # A route sign seen after an action has already started can belong to
        # the next junction.  Require the source frame to be newer than the
        # action start and either the existing grace period or image-confirmed
        # RIGHT disappearance before opening this one-entry queue.
        executing_states = ('BLUE_APPROACH','BLUE_STOP','INTERSECTION_WAIT',
                            'PLANNING','MANEUVER','REACQUIRE','UTURN')
        route_started = (ctx.route_action_started if ctx.route_action_started is not None
                         else ctx.action_started)
        right_released = (ctx.action == 'RIGHT' and ctx.right_sign_rearmed_at is not None)
        queue_ready = (ctx.action in directions and ctx.pending is None and
                       ctx.state in executing_states and
                       (right_released or now-route_started > ctx.cfg['sign_timeout']))
        if queue_ready and stamp > route_started:
            threshold = (ctx.cfg.get('red_sign_confidence',.8)
                         if label == 'RED' else ctx.cfg['sign_confidence'])
            if not _isfinite(confidence) or not threshold <= confidence <= 1.0:
                ctx.sign_label,ctx.sign_count = '',0
                ctx.sign_info.update(votes=0,decision='low_confidence_or_unknown')
                return
            if ctx.next_direction is not None:
                ctx.sign_label,ctx.sign_count = '',0
                ctx.direction_window,ctx.uturn_sign_window = [],[]
                ctx.sign_info.update(votes=0,decision='next_direction_occupied')
                return
            if ((label == 'RIGHT' and ctx.right_sign_locked) or
                    (label == ctx.action and label != 'RIGHT' and not ctx.route_sign_rearmed)):
                ctx.sign_label,ctx.sign_count = '',0
                ctx.sign_info.update(votes=0,decision='current_sign_still_visible')
                return
            if previous_stamp <= route_started or previous_label != label:
                count = 1
            else:
                count = previous_count + 1
            votes_required = (ctx.cfg.get('parking_sign_votes',3)
                              if label == 'PARKING' else
                              ctx.cfg.get('direction_sign_votes',2))
            ctx.sign_label,ctx.sign_count = label,count
            decision = 'next_direction_voting'
            ctx.sign_info.update(votes=count,decision=decision)
            if count >= votes_required:
                ctx.next_direction,ctx.next_direction_at = label,now
                if label == 'RIGHT':
                    ctx.right_sign_locked = True
                ctx.sign_info['decision'] = 'stored_next_direction'
            return
        # One confirmed action owns the cache through its blue-line wait and
        # every execution phase. Before the bounded queue opens, observations
        # cannot replace it or vote for the next action; RED/GREEN remain
        # available below. Preserve an already queued next direction.
        ctx.sign_label,ctx.sign_count = '',0
        ctx.direction_window,ctx.uturn_sign_window = [],[]
        ctx.sign_info.update(votes=0,decision=(
            'pending_already_stored' if label == ctx.pending else 'action_sign_ignored'))
        return
    threshold = ctx.cfg.get('red_sign_confidence',.8) if label == 'RED' else ctx.cfg['sign_confidence']
    valid = label in labels and threshold <= confidence <= 1.0
    if not valid:
        ctx.sign_label, ctx.sign_count = '', 0
        ctx.sign_info.update(votes=0,decision=(
            'right_sign_disappeared' if released else 'low_confidence_or_unknown'))
        return
    if label == 'RIGHT' and ctx.right_sign_locked:
        ctx.sign_label, ctx.sign_count = '', 0
        ctx.sign_info.update(votes=0,decision='current_sign_still_visible')
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
    elif (ctx.state in ('LANE','GAP','WAIT_OBSTACLE') or
          (ctx.state == 'WAIT_GREEN' and label == 'UTURN')):
        # Cache a confirmed U-turn before release; tick still waits for GREEN.
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
        ctx.pending, ctx.pending_at = label, now
        ctx.route_action_started = now
        ctx.route_sign_rearmed = False
        ctx.right_sign_rearmed_at = None
        if label == 'RIGHT':
            ctx.right_sign_locked = True
        ctx.sign_info['decision'] = 'stored_pending'
        if label == 'PARKING':
            ctx.marker, ctx.parking_marker = None,None
            ctx.park_line_side = None
