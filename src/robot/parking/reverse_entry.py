"""Rear-camera adapter for the same measured S parking used by the front."""
import math
from robot.common.geometry import distance, local, wrap, _isfinite
from robot.parking.entry import ExplicitStraightEntry


def reverse_after_turn_enabled(cfg):
    return (cfg.get('parking_enabled', True) and
            cfg.get('parking_mode') == 'forward_center' and
            cfg.get('parking_entry_style') == 'S')


def reverse_after_turn_route(ctx):
    if ctx.parking_completed:
        return False
    if not reverse_after_turn_enabled(ctx.cfg):
        return False
    if any(value not in (None, 'PARKING')
           for value in (ctx.pending, ctx.next_direction)):
        return False
    # Turn exit search still belongs to the maneuver, not lane following.
    return (ctx.action is None and ctx.last_completed_action in ('LEFT','RIGHT') and
            ctx.state in ('LANE', 'GAP', 'WAIT_OBSTACLE'))


class ReverseStraightEntry(ExplicitStraightEntry):
    """Reuse front S in a reflected world, with left mapped to rear-view right.

    Reflect world Y and use heading pi-yaw. Real rear-axle coordinates (x,y)
    then become (-x,y) inside front S, while rear-view coordinates are (-x,-y).
    This maps the front algorithm's preferred left edge to the rear-view right
    edge. Reflection and reverse motion cancel the steering sign changes;
    only speed and the calibrated reverse-steering sign need conversion.
    The virtual front bumper is the real rear bumper.
    """
    camera_source = 'rear'

    @staticmethod
    def reverse_pose(pose):
        return pose[0], -pose[1], wrap(math.pi-pose[2])

    def __init__(self, cfg, pose, now, wait_for_sign=False):
        # Rear P retreat has its own duration; never change the front task config.
        cfg = dict(cfg, parking_p_reverse_s=cfg.get(
            "parking_rear_p_reverse_s", cfg.get("parking_p_reverse_s", 0.)))
        ExplicitStraightEntry.__init__(self, cfg, self.reverse_pose(pose), now)
        self.inner.bumper_offset = cfg['rear_overhang']
        r = cfg.get('rear_bev', {})
        self.inner.side_view_far_m=-r['axle_x']+r['origin_v']/r['pixels_per_m']
        self.inner.camera_steering_feedback=cfg.get('pose_mode')=='command_model'
        if 'bottom_reference_v' in r:
            self.inner.bottom_reference = dict(
                depth_m=-r['axle_x']+(r['origin_v']-r['bottom_reference_v'])/r['pixels_per_m'],
                remaining_m=r['bottom_reference_remaining_m'])
        self.inner.allow_unanchored_side = not wait_for_sign
        # P gates entry below. Once accepted, use the current front geometry
        # policy, including paired far ends and short bottom-fragment tracking.
        self.inner.lock_target_bay = wait_for_sign
        self.inner.independent_bottom = True
        self.armed_at = float(now)
        self.parking_started = None if wait_for_sign else float(now)
        # Searching for rear P is not the trigger. Arm only on its acceptance.
        self.p_reverse_active = False
        self.geometry_after = float(now)
        self.rear_sign = dict(decision='waiting_p' if wait_for_sign else 'direct_visual_entry')
        self.rear_sign_stamp = -1.
        self.ready_at = now+max(.3, cfg.get('cusp_pause', .3))
        self.last_pose = tuple(pose)
        self.travelled = 0.
        self.rear_frames = 0
        self.rear_line_count = 0
        self.rear_observation_stamp = float(now)
        self.blue_candidate = None
        self.blue_stamp = -1.
        self.front_blue_candidate = None
        self.front_blue_stamp = -1.
        self.inner.debug.update(camera_source='rear', direction=-1,
                                phase='GEAR_PAUSE')

    @property
    def sign_anchor(self):
        return None

    @sign_anchor.setter
    def sign_anchor(self, value):
        # A subsequently seen FRONT sign must never retarget the rear task.
        pass

    @property
    def debug(self):
        # Diagnostics use the actual rear-view frame, not the mirrored frame
        # used privately by the unmodified front algorithm.
        result = dict(self.inner.debug)
        pair = self.inner.view
        result.update(camera_source='rear', direction=(1 if self.p_reverse_active and
                      self.p_reverse_phase == 'P_REVERSE' else -1),
                      pair_seen=pair is not None,
                      pair_width_m=pair['width'] if pair is not None else None,
                      right_seen=pair is not None or result.get('left_seen',False),
                      left_seen=pair is not None or result.get('right_seen',False),
                      rear_frames=self.rear_frames, rear_line_count=self.rear_line_count)
        result.update(rear_sign=dict(self.rear_sign),parking_started=self.parking_started)
        if self.parking_started is None and self.inner.status not in ('FAULT','FINISHED'):
            result.update(phase='REVERSE_STRAIGHT_WAIT_P',steering_reference='straight',
                          blue_used=False,bottom_seen=False)
        if result.get('steering_reference'):
            result['steering_reference'] = {'left_side':'right_side','right_side':'left_side'}.get(
                result['steering_reference'],result['steering_reference'])
        for key in ('side_local','bottom_local'):
            if result.get(key) is not None:
                result[key] = [(x,-y) for x,y in result[key]]
        if result.get('pair_local') is not None:
            result['pair_local'] = [[(x,-y) for x,y in side] for side in result['pair_local']]
        if result.get('sign_anchor_local') is not None:
            x,y=result['sign_anchor_local']
            result['sign_anchor_local']=(x,-y)
        if result.get('centerline_local') is not None:
            x,y=result['centerline_local']
            result['centerline_local']=(x,-y)
        for key in ('centerline_heading_error_deg','centerline_lateral_error_m'):
            if result.get(key) is not None:result[key]=-result[key]
        result['target_locked']=self.inner.sign_anchor is not None
        return result

    def observe(self, lines, stamp, pose=None):
        if stamp <= self.rear_observation_stamp or pose is None:
            return
        if stamp-self.rear_observation_stamp > self.cfg['sensor_timeout']:
            self.rear_frames = 0
        self.rear_observation_stamp = stamp
        self.rear_frames += 1
        self.rear_line_count = len(lines)
        if self.parking_started is None or stamp <= self.geometry_after:
            return
        mirrored = [[(x,-y) for x,y in line] for line in lines]
        ExplicitStraightEntry.observe(self, mirrored, stamp, self.reverse_pose(pose))

    @staticmethod
    def same_p_bounds(previous, current):
        """Keep the same image target using the front tracker's box continuity."""
        if previous is None or current is None:
            return True
        if (not isinstance(previous,(list,tuple)) or not isinstance(current,(list,tuple)) or
                len(previous)!=4 or len(current)!=4):return False
        try:
            old=[float(v) for v in previous]
            new=[float(v) for v in current]
        except (ValueError,TypeError):return False
        if not all(_isfinite(v) for v in old+new):return False
        ox,oy,ow,oh=old
        x,y,w,h=new
        if min(ow,oh,w,h)<=0:return False
        ratio=float(w*h)/(ow*oh)
        return (.35<=ratio<=2.8 and
                abs(x+w/2.-ox-ow/2.)<=1.5*max(w,ow) and
                abs(y+h/2.-oy-oh/2.)<=1.5*max(h,oh))

    def observe_sign(self, label, confidence, stamp, now, anchor=None, bounds=None):
        """Trigger once on rear P; refresh its position from newer images."""
        if self.inner.status in ('FAULT','FINISHED'):
            return False
        if (stamp <= max(self.armed_at,self.rear_sign_stamp) or
                not 0<=now-stamp<=self.cfg['sign_timeout']):
            return False
        self.rear_sign_stamp = stamp
        if label!='PARKING' or not self.cfg['sign_confidence']<=confidence<=1.:
            return False
        previous=self.rear_sign
        if not self.same_p_bounds(previous.get('sign_bounds'),bounds):
            previous.update(last_rejected_stamp=stamp,last_rejection='different_rear_p')
            return False
        first=self.parking_started is None
        if first:
            self.parking_started = float(now)
            self.arm_p_reverse()
            self.geometry_after = stamp
        if anchor is not None:
            ExplicitStraightEntry.sign_anchor.fset(self,(anchor[0],-anchor[1]))
        self.rear_sign=dict(label=label,confidence=confidence,stamp=stamp,
            sign_bounds=bounds if bounds is not None else previous.get('sign_bounds'),
            anchor_world=anchor if anchor is not None else previous.get('anchor_world'),
            position_stamp=stamp if anchor is not None else previous.get('position_stamp'),
            position_updates=previous.get('position_updates',0)+(1 if anchor is not None else 0),
            decision=('parking_started' if anchor is not None else 'parking_started_unprojected')
                     if first else
                     ('parking_p_position_updated' if anchor is not None else 'parking_p_observed_unprojected'))
        if first:self.inner.reason = 'parking_reverse_p_seen'
        return True

    def observe_blue(self, lines, stamp, pose, source='rear'):
        """Confirm a nearby transverse blue normal; never treat blue as bottom."""
        if source not in ('front','rear'):return
        front=source=='front'
        last=self.front_blue_stamp if front else self.blue_stamp
        if (self.parking_started is None or self.p_reverse_active or
                (self.inner.associate and self.inner.sign_anchor is None) or
                stamp <= max(self.geometry_after,last)):
            return
        if front:self.front_blue_stamp=stamp
        else:self.blue_stamp=stamp
        candidates = []
        for line in lines:
            x,y = local(pose,line['point'])
            angle = (line['yaw']-pose[2]+math.pi/2)%math.pi-math.pi/2
            depth=x if front else -x
            if (not .05<depth<=1.4 or abs(y)>.45 or
                    line['length']<self.cfg['blue']['long_min'] or
                    abs(angle)>math.radians(35)):
                continue
            candidates.append((abs(x),dict(point=line['point'],
                theta=wrap(math.pi-(pose[2]+angle)),stamp=stamp,frames=1,source=source)))
        if not candidates:
            if front:self.front_blue_candidate=None
            else:self.blue_candidate=None
            return
        candidate = min(candidates,key=lambda row:row[0])[1]
        previous = self.front_blue_candidate if front else self.blue_candidate
        if (previous is not None and 0<stamp-previous['stamp']<=self.cfg['sensor_timeout'] and
                distance(candidate['point'],previous['point'])<=.15 and
                abs(wrap(candidate['theta']-previous['theta']))<=math.radians(8)):
            candidate['frames'] = previous['frames']+1
        if front:self.front_blue_candidate=candidate
        else:self.blue_candidate=candidate

    def blue_steering(self, now, steer):
        front=self.front_blue_candidate
        blue=(front if front is not None and front['frames']>=2 and
              0<=now-front['stamp']<=self.cfg['sensor_timeout'] else self.blue_candidate)
        debug = self.inner.debug
        debug.update(blue_used=False,blue_seen=blue is not None,
                     blue_camera_source=blue['source'] if blue else None,
                     blue_frames=blue['frames'] if blue else 0,
                     blue_reject_reason=None)
        if blue is None or blue['frames']<2 or not 0<=now-blue['stamp']<=self.cfg['sensor_timeout']:
            return steer
        reference = self.inner.view or self.inner.single
        if reference is not None:
            debug['blue_reject_reason'] = 'bay_edges_priority'
            return steer
        # Blue supplies heading only while no target edge is usable. Never
        # replace an edge's lateral/heading correction with another line.
        center = self.inner.pose[:2]
        steer = self.inner.limited_steer(dict(theta=blue['theta'],center=center))
        heading_source='front_blue_heading' if blue['source']=='front' else 'blue_heading'
        debug.update(blue_used=True,
                     blue_heading_error_deg=-math.degrees(wrap(blue['theta']-self.inner.pose[2])),
                     steering_reference=heading_source)
        self.inner.reason = 'parking_reverse_blue_heading'
        return steer

    def abort(self, reason):
        self.inner.status = self.inner.phase = 'FAULT'
        self.inner.reason = reason
        self.inner.debug.update(camera_source='rear', direction=-1,
                                phase='FAULT', reason=reason)
        return 0, 0.

    def command(self, now, pose):
        if self.inner.status in ('FAULT', 'FINISHED'):
            return 0, 0.
        if (self.parking_started is not None and
                now-self.parking_started > self.cfg.get('parking_entry_timeout_s', 35.)):
            return self.abort('parking_reverse_timeout')
        self.travelled += distance(pose, self.last_pose)
        self.last_pose = tuple(pose)
        # Retain travelled distance for diagnostics, without a search stop.
        self.inner.debug.update(travel_m=self.travelled,travel_limit_m=None)
        if now < self.ready_at:
            self.inner.reason = 'parking_reverse_gear_pause'
            return 0, 0.
        if self.parking_started is None:
            self.inner.reason = 'parking_reverse_straight_wait_p'
            self.inner.debug.update(camera_source='rear',direction=-1,
                travel_m=self.travelled,travel_limit_m=None,steering_command_rad=0.)
            return -self.cfg.get('parking_entry_speed_raw',12), 0.
        prelude = self.p_reverse_command(now,self.reverse_pose(pose))
        if prelude is not None:
            # Away from rear P means physically FORWARD before reversing in.
            self.inner.debug['direction'] = 1 if prelude[0] else 0
            return -prelude[0], prelude[1]
        # Reuse the front S wrapper as well as its inner visual controller;
        # keep geometry readiness and no-line fallback in one implementation.
        speed, steer = ExplicitStraightEntry.command(self, now, self.reverse_pose(pose))
        reference=self.inner.view or self.inner.single
        self.inner.debug['rear_vehicle_lateral_m']=None
        if reference is not None:
            unused_heading,lateral=self.inner.errors(reference['theta'],reference['center'],
                pose=self.inner.steering_pose(reference))
            # Positive: vehicle is to the RIGHT of its bay in rear view.
            self.inner.debug['rear_vehicle_lateral_m']=lateral
        if speed:
            steer = self.blue_steering(now,steer)
        self.inner.debug.update(camera_source='rear', direction=-1,
            travel_m=self.travelled,travel_limit_m=None)
        if self.inner.reason == 'parking_follow_left_side':
            self.inner.reason = 'parking_follow_right_side'
        elif self.inner.reason == 'parking_follow_right_side':
            self.inner.reason = 'parking_follow_left_side'
        if self.inner.reason == 'parking_front_stale':
            self.inner.reason = 'parking_rear_stale'
        if self.inner.reason == 'parking_missing_white_straight':
            self.inner.reason = 'parking_reverse_missing_white_straight'
        reverse_sign = self.cfg.get('chassis_calibration', {}).get('reverse_steering_sign', 1)
        self.inner.debug['steering_command_rad'] = steer*reverse_sign
        return -speed, steer*reverse_sign
