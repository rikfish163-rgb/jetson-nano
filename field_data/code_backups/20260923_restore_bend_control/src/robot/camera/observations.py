"""camera module: explicit context input; coordinator applies returned updates."""
from __future__ import division

import math
from robot.common.geometry import _isfinite
from robot.common.geometry import distance
from robot.common.geometry import local
from robot.common.geometry import world
from robot.common.geometry import wrap



FIELDS = ('cfg', 'action', 'right_lock', 'consumed_marker', 'front_blue_lines', 'front_marker_stamp', 'front_markers',
 'ground_stamp', 'ground_stamps', 'lane', 'lane_boundaries', 'lane_confidence',
 'lane_stamp', 'left_boundary', 'left_boundary_stamp', 'marker', 'blue_consumed', 'marker_candidate', 'pending',
 'parking_candidate_stamp', 'parking_candidates', 'parking_detection', 'parking_entry',
 'parking_front_frames', 'parking_lines', 'parking_lines_stamp', 'parking_marker',
 'parking_trigger_at', 'pose', 'rear_blue_lines', 'rear_marker_stamp', 'rear_markers',
 'slot', 'slot_locked', 'slot_stamp', 'blue_approach', 'blue_clear_since', 'blue_clear_frames')
CALLS = ()
OPERATIONS = ('observe_lane', 'observe_left_boundary', 'observe_ground')

def observe_lane(ctx, points, confidence, stamp, boundaries=None):
    if stamp <= ctx.lane_stamp:
        return
    ctx.lane = [world(ctx.pose, p) for p in points if p[0] > 0 and all(_isfinite(v) for v in p)]
    ctx.lane.sort(key=lambda p: local(ctx.pose,p)[0])
    ctx.lane_stamp, ctx.lane_confidence = stamp, confidence
    ctx.lane_boundaries = dict((side, [world(ctx.pose,p) for p in rows])
                                for side,rows in (boundaries or {}).items())


def observe_left_boundary(ctx, points, stamp):
    if stamp > ctx.left_boundary_stamp:
        ctx.left_boundary = [world(ctx.pose,p) for p in points
                              if p[0] > 0 and all(_isfinite(v) for v in p)]
        ctx.left_boundary_stamp = stamp


def observe_ground(ctx, data, stamp):
    channel = (data.get('source','front'), data.get('part','all'))
    if stamp <= ctx.ground_stamps.get(channel, -1.0):
        return
    ctx.ground_stamps[channel] = stamp
    ctx.ground_stamp = max(ctx.ground_stamp, stamp)
    if channel[0] == 'rear' and channel[1] in ('all','markers'):
        ctx.rear_blue_lines = [dict(point=world(ctx.pose,(m['x'],m['y'])),
            yaw=wrap(ctx.pose[2]+m['yaw']),length=m['length'])
            for m in data.get('blue_lines',[]) if m['x']<0]
        ctx.rear_markers = [world(ctx.pose,(m['x'],m['y']))
            for m in data.get('markers',[]) if m['kind']=='junction' and m['x']<0]
        ctx.rear_marker_stamp = stamp
        if channel[1] == 'markers':
            return  # Rear blue must not dispatch a forward junction action.
    if channel[1] == 'parking_lines':
        if channel[0] == 'front':
            ctx.parking_lines = [[world(ctx.pose,p) for p in line] for line in data['lines']]
            ctx.parking_lines_stamp = stamp
            if ctx.parking_entry is not None:
                ctx.parking_entry.observe(ctx.parking_lines,stamp,ctx.pose)
        return
    if ctx.parking_entry is not None and channel[1] == 'slots':
        return  # the selected white bay stays locked; use its individual edges
    if channel[0] == 'front' and channel[1] in ('all','markers') and stamp > ctx.front_marker_stamp:
        # Keep measured blue orientation independently of junction dispatch.
        ctx.front_blue_lines = [dict(point=world(ctx.pose,(m['x'],m['y'])),
            yaw=wrap(ctx.pose[2]+m['yaw']),length=m['length'])
            for m in data.get('blue_lines',[]) if m['x']>0]
        ctx.front_markers = [world(ctx.pose,(m['x'],m['y']))
                              for m in data.get('markers', [])
                              if m['kind'] == 'junction' and m['x'] > 0]
        ctx.front_marker_stamp = stamp
    markers = []
    for m in data.get('markers', []) if channel[0]=='front' else []:
        if m['kind'] in ('tick','junction') and m['x'] > 0:
            p = world(ctx.pose,(m['x'],m['y']))
            if not ctx.consumed_marker or distance(p,ctx.consumed_marker) >= ctx.cfg['marker_rearm_distance']:
                if (ctx.parking_marker is None or stamp > ctx.parking_marker[1] or
                        distance(p,ctx.pose) < distance(ctx.parking_marker[0],ctx.pose)):
                    ctx.parking_marker = (p,stamp)
        if m['kind'] != 'junction':
            continue  # short scoring ticks never dispatch a turn
        if not 0 < m['x'] <= ctx.cfg.get('straight_align_distance',1.6) or abs(m['y']) > ctx.cfg['lane_width']:
            continue
        if m.get('length',ctx.cfg['blue']['long_min']) < ctx.cfg['blue']['long_min']:
            continue
        measured = [line for line in data.get('blue_lines',[])
                    if math.hypot(line['x']-m['x'],line['y']-m['y']) < .05]
        if measured:
            line = min(measured,key=lambda line:math.hypot(line['x']-m['x'],line['y']-m['y']))
            normal = (line['yaw']+math.pi/2)%math.pi-math.pi/2
            if abs(normal) > math.radians(60):
                continue  # longitudinal blue is alignment paint, not a stop line
            # The finite stripe must reach the car's forward corridor. A long
            # transverse segment wholly in the neighbouring lane is not ours.
            if abs(m['y']/math.cos(normal)) > line['length']/2+ctx.cfg['body_width']/2:
                continue
        p = world(ctx.pose, (m['x'], m['y']))
        markers.append(p)
    if channel[0] == 'front' and channel[1] in ('all','markers'):
        approach = ctx.blue_approach
        if approach is not None and approach['phase'] == 'ALIGN':
            nx,ny = math.cos(approach['yaw']),math.sin(approach['yaw'])
            matches = [p for p in markers if
                abs((p[0]-approach['point'][0])*nx+(p[1]-approach['point'][1])*ny) < .15
                and distance(p,approach['point']) < .40]
            if matches:
                point = min(matches,key=lambda p:distance(p,approach['point']))
                lines = [line for line in ctx.front_blue_lines if distance(line['point'],point) < .05]
                yaw = min(lines,key=lambda line:distance(line['point'],point))['yaw'] if lines else approach['yaw']
                if math.cos(yaw-approach['yaw']) < 0: yaw=wrap(yaw+math.pi)
                if abs(wrap(yaw-approach['yaw'])) < math.radians(25):
                    approach.update(point=point,yaw=yaw,observed_stamp=stamp)
                    ctx.consumed_marker=point
        exit_blue = (ctx.action == 'RIGHT' and ctx.right_lock is not None and
                     ctx.cfg.get('right_exit_on_blue', False))
        if (ctx.pending == 'PARKING' and ctx.cfg.get('parking_mode') != 'forward_center') or exit_blue:
            # Preserve the existing explicit parking and right-exit handlers.
            special = [p for p in markers if not ctx.consumed_marker or
                       distance(p,ctx.consumed_marker) >= ctx.cfg['marker_rearm_distance']]
            if special:
                ctx.marker = (min(special,key=lambda p: distance(p,ctx.pose)),stamp)
        elif ctx.action is not None:
            ctx.blue_consumed = True
            ctx.blue_clear_since,ctx.blue_clear_frames = None,0
            ctx.marker_candidate = None
            ctx.marker = None
        elif not markers:
            ctx.marker_candidate = None
            ctx.marker = None
            if ctx.blue_clear_since is None:
                ctx.blue_clear_since = stamp
            ctx.blue_clear_frames += 1
            if ctx.blue_clear_frames >= 3 and stamp-ctx.blue_clear_since >= .30-1e-9:
                ctx.blue_consumed = False
        elif not ctx.blue_consumed:
            ctx.blue_clear_since,ctx.blue_clear_frames = None,0
            point = min(markers,key=lambda p: distance(p,ctx.pose))
            previous = ctx.marker_candidate
            matched = (previous is not None and 0 < stamp-previous['stamp'] <= .5
                       and distance(point,previous['point']) <= .15)
            count = previous['count']+1 if matched else 1
            since = previous['since'] if matched else stamp
            ctx.marker_candidate = dict(point=point,stamp=stamp,count=count,since=since)
            if count >= 3 and stamp-since >= .15:
                ctx.marker = (point,stamp)
                if ctx.pending is None and not ctx.cfg.get('blue_default_straight',False):
                    ctx.blue_consumed=True
                    ctx.marker=None
        else:
            ctx.blue_clear_since,ctx.blue_clear_frames = None,0
    auto = ctx.cfg['parking_slot'] == 'AUTO'
    profile_name = ('P1' if ctx.slot_locked and ctx.slot['kind'] == 'parallel' else 'P4') if auto else ctx.cfg['parking_slot']
    profile = ctx.cfg['slots'][profile_name]
    candidates = [s for s in data.get('slots', [])
                  if (auto and not ctx.slot_locked) or s['kind'] == profile['kind']]
    candidates.sort(key=lambda s: s['x'])
    if ctx.slot_locked and stamp <= ctx.slot_stamp:
        return  # an older frame from the other camera cannot roll tracking back
    if auto and not ctx.slot_locked:
        # Only current FRONT frames select bays. Rear frames subsequently
        # track the same world-space target, not camera-relative ranks.
        if channel[0] != 'front' or channel[1] == 'markers':
            return
        previous = (ctx.parking_candidates if 0<stamp-ctx.parking_candidate_stamp<=ctx.cfg.get('ground_timeout',1.25) else [])
        ctx.parking_candidates = []
        ctx.parking_detection = dict(input_candidates=len(candidates),position_rejected=0,
                                      yaw_rejected=0,accepted=0,stamp=stamp,
                                      geometry=data.get('slot_diagnostic',{}))
        for s in candidates:
            profile = ctx.cfg['slots']['P1' if s['kind'] == 'parallel' else 'P4']
            side_min = .05 if ctx.cfg.get('parking_mode')=='forward_white' else profile['width']/2
            if s['x'] <= 0 or abs(s['y']) < side_min:
                ctx.parking_detection['position_rejected']+=1
                continue
            offset = 0 if s['kind'] == 'parallel' else (math.pi/2 if s['y'] < 0 else -math.pi/2)
            if ctx.cfg.get('parking_mode') == 'forward_white' and s['kind']=='perpendicular':
                offset = -offset  # forward entry points INTO the bay
            expected = wrap(ctx.pose[2] + offset)
            observed = wrap(ctx.pose[2]+s['yaw'])
            yaw = min((observed,wrap(observed+math.pi)),key=lambda a:abs(wrap(a-expected)))
            yaw_limit = (math.radians(ctx.cfg.get('parking_capture_yaw_deg',45)) if
                         ctx.cfg.get('parking_mode')=='forward_white' else ctx.cfg['parking_yaw_match_tolerance'])
            if abs(wrap(yaw-expected)) > yaw_limit:
                ctx.parking_detection['yaw_rejected']+=1
                continue
            center = world(ctx.pose,(s['x'],s['y']))
            matches=[p for p in previous if p['kind']==profile['kind'] and distance(p['pose'],center)<.08 and
                     abs(wrap(p['pose'][2]-yaw))<math.radians(15)]
            confirmations=1+max([p.get('observations',1) for p in matches] or [0])
            ctx.parking_candidates.append(dict(pose=tuple(center)+(yaw,),
                length=profile['length'],width=profile['width'],kind=profile['kind'],
                observations=min(255,confirmations),
                id='AUTO_%d' % (len(ctx.parking_candidates)+1)))
        ctx.parking_detection['accepted']=len(ctx.parking_candidates)
        ctx.parking_candidate_stamp = stamp
        if stamp >= ctx.parking_trigger_at:
            ctx.parking_front_frames += 1
        return
    if ctx.slot_locked:
        # Reacquire the SAME physical bay; do not re-sort identities during reversing.
        matches = [(distance(world(ctx.pose,(s['x'],s['y'])),ctx.slot['pose']),s) for s in candidates]
        if not matches:
            return
        dist,s = min(matches, key=lambda pair: pair[0])
        if dist > ctx.cfg['slot_match_distance']:
            return
    else:
        if len(candidates) < profile['min_candidates']:
            return
        s = candidates[profile['rank']]
    observed_yaw = wrap(ctx.pose[2]+s['yaw'])
    expected_yaw = ctx.slot['pose'][2] if ctx.slot_locked else wrap(ctx.pose[2]+profile['yaw'])
    yaw = min((observed_yaw,wrap(observed_yaw+math.pi)), key=lambda a: abs(wrap(a-expected_yaw)))
    if abs(wrap(yaw-expected_yaw)) > ctx.cfg['parking_yaw_match_tolerance']:
        return
    center = world(ctx.pose,(s['x'],s['y']))
    if ctx.slot_locked and ctx.cfg['pose_mode'] == 'command_model':
        # Keep the locked bay stationary. Correct ego-pose drift instead
        # of moving the target. The ROS adapter aligns this with history.
        anchor = ctx.slot['pose']
        gain = ctx.cfg['parking_visual_pose_gain']
        limit = ctx.cfg['parking_visual_max_shift']
        alimit = ctx.cfg['parking_visual_max_yaw_shift']
        correction = (max(-limit,min(limit,gain*(anchor[0]-center[0]))),
                      max(-limit,min(limit,gain*(anchor[1]-center[1]))),
                      max(-alimit,min(alimit,gain*wrap(anchor[2]-yaw))))
        ctx.slot_stamp = stamp
        return correction
    if ctx.slot_locked:
        ctx.slot_stamp = stamp
        return
    ctx.slot = dict(pose=tuple(center)+(yaw,), length=profile['length'], width=profile['width'],
                     id=ctx.cfg['parking_slot'], kind=profile['kind'])
    ctx.slot_stamp = stamp
