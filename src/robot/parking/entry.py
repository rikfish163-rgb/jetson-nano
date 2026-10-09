"""White bay geometry and staged parking; coordinates refer to the rear axle."""
from __future__ import division
import math
from robot.common.geometry import distance
from robot.common.geometry import inside_slot
from robot.common.geometry import local
from robot.common.geometry import world
from robot.common.geometry import wrap
from robot.parking.planner import rank_parking_slots


class TimedRightEntry(object):
    """The explicit P4/P5 ``T`` entry: forward, right full lock, one second.

    The blue-line confirmation is owned by the mission controller.  This task
    starts only after that confirmation, and owns the short fixed-time command
    interval that follows it.  It deliberately has no reverse or follow-up
    stage: completing the one-second command is the terminal parking result.
    """

    def __init__(self, cfg, now):
        self.cfg = cfg
        self.phase = 'READY'
        self.status = None
        self.reason = 'parking_t_entry_right'
        # The blue stop may hold the vehicle for an arbitrary short time
        # before the first control tick.  The one-second interval starts at
        # that first nonzero command, not at task construction.
        self.started = None
        self.last = None
        self.elapsed = 0.0
        self.finished = False
        self.debug = dict(phase=self.phase, style='T', duration_s=1.0,
                          control_gap_s=.5, command_validity_s=.25)

    def observe(self, lines, stamp, pose=None):
        """Accept the common parking-lines callback; T does not use it."""
        return None

    def abort(self, reason):
        self.finished = True
        self.phase = 'FAULT'
        self.status = 'FAULT'
        self.reason = str(reason)
        self.debug['phase'] = self.phase
        return 0, 0.0

    def _command(self):
        speed = float(self.cfg.get('parking_entry_speed_raw', 12))
        raw = -float(self.cfg['steering_raw_limit'])
        scale = self.cfg.get('steering_command_scale_rad', self.cfg['max_steer'])
        return (speed / self.cfg['speed_sign'],
                raw / self.cfg['steering_raw_limit'] * scale /
                self.cfg['steering_sign'])

    def command(self, now):
        """Return one checked-command candidate, or a terminal zero command."""
        if self.finished or self.phase in ('FINISHED', 'FAULT'):
            return 0, 0.0
        now = float(now)
        if self.last is None:
            self.started = self.last = now
            self.phase = 'TIMED_RIGHT'
            self.debug['phase'] = self.phase
            return self._command()
        dt = now - self.last
        if not 0.0 <= dt <= .5:
            return self.abort('parking_t_entry_control_gap')
        self.last = now
        # A command can remain valid for .25 s.  If the scheduler arrives
        # later, count only that valid portion while still allowing bounded
        # recovery up to .5 s; the expired interval remains stopped time.
        self.elapsed += min(dt, .25)
        self.debug.update(elapsed_s=self.elapsed, last_command_at=now)
        if self.elapsed >= 1.0:
            self.finished = True
            self.phase = 'FINISHED'
            self.status = 'FINISHED'
            self.reason = 'parking_t_entry_complete'
            self.debug['phase'] = self.phase
            return 0, 0.0
        return self._command()


class ExplicitStraightEntry(object):
    """P4/P5 ``S`` entry with a hard gate on measured white geometry.

    ``ForwardParking`` contains the calibrated side and bottom-line geometry,
    but its legacy ``forward_center`` fallback follows a lane when no bay line
    is visible.  The explicit S route must wait in that situation so a missing
    parking frame can never turn into blind forward motion.
    """

    def __init__(self, cfg, pose, now, sign_anchor=None):
        # Import lazily because forward.py already imports WhiteParking from
        # this module.  Composition keeps that dependency one-way at runtime.
        from robot.parking.forward import ForwardParking
        self.inner = ForwardParking(cfg, pose, now)
        self.inner.bottom_only = True
        self.cfg = cfg
        # ``parking_sign_association`` is a production-wide default for the
        # legacy forward route.  An explicit S task may have no projected P
        # point (the camera can recognize the sign while projection fails),
        # so association is enabled only for a real anchor supplied at arm
        # time.  Without it, the measured post-sign white geometry remains
        # the target evidence.
        self.inner.associate = sign_anchor is not None
        self.inner.sign_anchor = sign_anchor
        # The sign is the task's observation boundary.  Do not let a white
        # frame captured before the sign arm S, even when it is still inside
        # the ordinary camera freshness window.
        self.inner.lines = []
        self.inner.stamp = float(now) - 1e-9
        self.inner.view = None
        self.inner.single = None
        self.inner.bottom = None
        self.inner.bottom_stamp = -1.0
        self.inner.bottom_source = None
        self.inner.bottom_pose = None
        self.inner.motion_sample = None
        self.inner.motion_ratios = []
        self.inner.motion_scale = 1.0
        self.inner.tracked_view = None
        self.inner.tracked_view_stamp = -1.0
        self.inner.debug['geometry_ready'] = False

    @property
    def sign_anchor(self):
        return self.inner.sign_anchor

    @sign_anchor.setter
    def sign_anchor(self, value):
        # controller_node may receive a valid projection asynchronously after
        # the S task was armed.  Keep the assignment on ForwardParking rather
        # than shadowing it on this composition wrapper.
        self.inner.sign_anchor = value
        self.inner.associate = value is not None

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def observe(self, lines, stamp, pose=None):
        return self.inner.observe(lines, stamp, pose)

    def command(self, now, pose):
        if self.inner.status in ('FINISHED', 'FAULT'):
            return self.inner.command(now, pose)
        sign_side = (self.inner.phase == 'APPROACH' and
                     self.inner.single is not None)
        if self.inner.view is None and self.inner.bottom is None and not sign_side:
            self.inner.reason = 'parking_wait_white_geometry'
            self.inner.debug.update(phase=self.inner.phase, geometry_ready=False,
                bottom_only_search=True,bottom_seen=False,
                bottom_candidate_frames=(self.inner.bottom_candidate or {}).get('frames',0),
                line_count=len(self.inner.lines), single_side_seen=False,
                sign_association=self.inner.associate,
                sign_anchor_local=(local(pose,self.inner.sign_anchor)
                    if self.inner.sign_anchor is not None else None),
                sides_behind_sign_rejected=self.inner.sides_behind_sign_rejected,
                line_age_s=now-self.inner.stamp)
            return 0, 0.0
        command = self.inner.command(now, pose)
        self.inner.debug['geometry_ready'] = True
        return command


def bay_edges(slot, lines, cfg):
    """Associate actual finite white segments to one locked bay, not road lines.

    The slot's +x axis points into the bay. Missing edges remain None; in
    particular, the mouth is never substituted for the bottom white line.
    """
    tol = cfg.get('parking_edge_match_m', .07)
    angle = math.radians(cfg.get('parking_edge_match_deg', 18))
    length, width = slot['length'], slot['width']
    found, scores = dict(left=None, right=None, bottom=None), {}
    for line in lines:
        a, b = [local(slot['pose'], p) for p in line]
        dx, dy = b[0]-a[0], b[1]-a[1]
        if math.hypot(dx,dy) < .12:
            continue
        cx, cy = (a[0]+b[0])/2, (a[1]+b[1])/2
        names = []
        if abs(dy) <= abs(dx)*math.tan(angle) and abs(cx) <= length/2+tol:
            names = [('left', abs(cy-width/2)), ('right', abs(cy+width/2))]
        elif abs(dx) <= abs(dy)*math.tan(angle) and abs(cy) <= width/2+tol:
            names = [('bottom', abs(cx-length/2))]
        for name, error in names:
            if error <= tol and error < scores.get(name, float('inf')):
                found[name], scores[name] = line, error
    return found


def shared_divider(near, far, lines, cfg):
    """Require the SAME observed side segment to bound both adjacent bays."""
    a, b = bay_edges(near, lines, cfg), bay_edges(far, lines, cfg)
    for x in (a['left'], a['right']):
        if x is not None and any(x == y for y in (b['left'], b['right'])):
            return x
    return None


def select_entry(core, now):
    """Select confirmed bays; a lone near bay also needs start-position evidence."""
    cfg = core.cfg
    rows = rank_parking_slots(core.pose,core.parking_candidates,core.scan,cfg)
    core.parking_diagnostics = rows
    rows = sorted((r for r in rows if r['kind']=='perpendicular'),
                  key=lambda r:local(core.pose,r['pose'])[0])
    if not rows:
        core.reason='parking_no_visual_candidates'
        return None
    if not any(r['occupancy']=='FREE' for r in rows):
        core.reason='parking_no_lidar_confirmed_free_bay'
        return None
    ready=[r for r in rows if r.get('observations',0)>=2]
    if not ready:
        core.reason='parking_wait_candidate_confirmation'
        return None
    if len(rows)==1 and len(ready)==1 and ready[0]['occupancy']=='FREE':
        x,y=local(core.pose,ready[0]['pose'])
        angle=abs(wrap(ready[0]['pose'][2]-core.pose[2]))
        radius=cfg['wheelbase']/math.tan(cfg['max_steer'])
        if abs(x-radius*math.sin(angle))<=cfg.get('parking_near_start_tolerance_m',.12):
            return WhiteParking(core,dict(ready[0],relative_bay='near'),None,now)
        core.reason='parking_single_bay_not_at_near_start'
        return None
    for near in rows:
        nx,ny = local(core.pose,near['pose'])
        for far in rows:
            fx,fy = local(core.pose,far['pose'])
            along,across=local(near['pose'],far['pose'])
            if (fx <= nx or ny*fy <= 0 or abs(along)>.10 or
                    abs(wrap(near['pose'][2]-far['pose'][2]))>math.radians(15) or
                    abs(abs(across)-(near['width']+far['width'])/2)>.09):
                continue
            target = near if near['occupancy']=='FREE' else far if far['occupancy']=='FREE' else None
            if target is None:
                continue
            if target.get('observations',0)<2:
                core.reason='parking_wait_candidate_confirmation'
                return None
            divider = shared_divider(near,far,core.parking_lines,cfg)
            if target is far and divider is None:
                core.reason = 'parking_wait_shared_divider'
                return None
            target = dict(target, relative_bay='near' if target is near else 'far')
            return WhiteParking(core,target,divider if target['relative_bay']=='far' else None,now)
    core.reason = 'parking_wait_two_adjacent_bays_or_free_scan'
    return None


class WhiteParking(object):
    """Forward lock -> observed two-side alignment -> measured final advance.

    This object produces command-space steering. Controller retains the sole
    motor output and raw lidar sweep checks. White observations are world-space
    segments captured at their source pose, never refreshed by repeated ticks.
    """
    def __init__(self, core, slot, divider, now):
        # Read the observation once; never retain or mutate the mission owner.
        self.cfg, self.slot, self.pose = core.cfg,slot,core.pose
        self.reason, self.status = None,None
        self.start, self.started = core.pose,now
        self.side = 1 if local(core.pose,slot['pose'])[1]>0 else -1
        self.phase = 'APPROACH_DIVIDER' if divider is not None else 'LOCK'
        self.turn_start = core.pose
        self.divider, self.divider_stamp = divider,now
        self.divider_pose = core.pose
        self.lines, self.stamp = [],-1.0
        self.view = None
        self.bottom = None
        self.bottom_stamp = -1.0
        self.aligned_since, self.last_aligned_stamp = None,-1.0
        self.advance_start, self.advance_at = None,None
        self.final_heading, self.final_center = None,None
        self.debug = dict(phase=self.phase,side='left' if self.side>0 else 'right',
                          relative_bay=slot['relative_bay'],slot_id=slot['id'])
        self.observe(core.parking_lines,core.parking_lines_stamp)

    def observe(self, lines, stamp, pose=None):
        if pose is not None:
            self.pose = tuple(pose)
        if stamp <= self.stamp:
            return
        self.lines, self.stamp = lines,stamp
        cfg, pose = self.cfg,self.pose
        if self.phase == 'APPROACH_DIVIDER' and self.divider is not None:
            # Reassociate the locked divider; never take a different transverse line.
            old = tuple((self.divider[0][i]+self.divider[1][i])/2 for i in (0,1))
            matches = []
            for line in lines:
                mid = tuple((line[0][i]+line[1][i])/2 for i in (0,1))
                a,b = [local(self.start,p) for p in line]
                if distance(mid,old)<.09 and abs(b[0]-a[0])<.3*abs(b[1]-a[1]):
                    matches.append((distance(mid,old),line))
            if matches:
                self.divider = min(matches,key=lambda r:r[0])[1]
                self.divider_stamp,self.divider_pose = stamp,pose
        edges = bay_edges(self.slot,lines,cfg)
        self.debug.update(left_seen=edges['left'] is not None,right_seen=edges['right'] is not None,
                          bottom_seen=edges['bottom'] is not None)
        if edges['bottom'] is not None:
            self.bottom = edges['bottom']
            self.bottom_stamp = stamp
        self.view = None
        if edges['left'] is None or edges['right'] is None:
            self.aligned_since = None
            return
        headings, mids = [],[]
        for line in (edges['left'],edges['right']):
            a,b = line
            theta = math.atan2(b[1]-a[1],b[0]-a[0])
            if abs(wrap(theta-self.slot['pose'][2]))>math.pi/2:
                theta = wrap(theta+math.pi)
            headings.append(theta)
            mids.append(tuple((a[i]+b[i])/2 for i in (0,1)))
        if abs(wrap(headings[0]-headings[1]))>math.radians(10):
            self.aligned_since = None
            return
        theta = headings[0]+wrap(headings[1]-headings[0])/2
        normal = (-math.sin(theta),math.cos(theta))
        width = abs(sum((mids[0][i]-mids[1][i])*normal[i] for i in (0,1)))
        if abs(width-self.slot['width'])>.06:
            self.aligned_since = None
            return
        center = tuple((mids[0][i]+mids[1][i])/2 for i in (0,1))
        self.view = dict(theta=theta,center=center,stamp=stamp,width=width)

    def errors(self, theta, center):
        pose = self.pose
        lateral = -(pose[0]-center[0])*math.sin(theta)+(pose[1]-center[1])*math.cos(theta)
        return wrap(theta-pose[2]),lateral

    def remaining(self, pose=None):
        if self.bottom is None:
            return None
        a,b = [local(self.pose if pose is None else pose,p) for p in self.bottom]
        dx,dy = b[0]-a[0],b[1]-a[1]
        if abs(dy)<.05 or abs(dx/dy)>.35:
            return None
        # Worst corner of the front bumper, not the rear-axle/line-center distance.
        x0 = a[0]-a[1]*dx/dy
        front = self.cfg['wheelbase']+self.cfg['front_overhang']
        clearance = x0-front-abs(dx/dy)*self.cfg['body_width']/2
        return clearance-self.cfg.get('parking_bottom_clearance_m',.04)

    def stopped(self, reason, fault=False):
        self.reason = reason
        if fault:
            self.status = 'FAULT'
        self.debug.update(phase=self.phase,reason=reason)
        return 0,0.0

    def steer(self, theta, center):
        heading,lateral = self.errors(theta,center)
        physical = 2.5*heading-math.atan2(1.5*lateral,.30)
        physical = max(-self.cfg['max_steer'],min(self.cfg['max_steer'],physical))
        return physical/self.cfg['max_steer']*self.cfg.get('steering_command_scale_rad',self.cfg['max_steer'])

    def command(self, now, pose):
        self.pose = tuple(pose)
        cfg,pose = self.cfg,self.pose
        self.debug.update(phase=self.phase,line_age_s=now-self.stamp,bottom_age_s=now-self.bottom_stamp)
        if now-self.started>cfg.get('parking_entry_timeout_s',35):
            return self.stopped('parking_entry_timeout',True)
        if self.phase == 'SETTLE':
            theta,center = (self.view['theta'],self.view['center']) if self.view else (self.final_heading,self.final_center)
            heading,lateral = self.errors(theta,center)
            remaining = self.remaining()
            if (abs(heading)>math.radians(cfg.get('parking_align_heading_deg',6)) or
                    abs(lateral)>cfg.get('parking_align_lateral_m',.035) or remaining is None or
                    not -.025 <= remaining <= .025):
                return self.stopped('parking_settle_pose_changed',True)
            if now-self.settle_at>=.5:
                if not inside_slot(pose,self.slot,cfg):
                    return self.stopped('parking_wheels_not_inside',True)
                self.status = 'FINISHED'
            return self.stopped('parking_settle')
        if not 0 <= now-self.stamp <= cfg.get('ground_timeout',1.25):
            return self.stopped('parking_front_stale')
        if self.phase == 'APPROACH_DIVIDER':
            if (now-self.divider_stamp>3 or distance(pose,self.divider_pose)>.20):
                return self.stopped('parking_divider_lost')
            mid = tuple((self.divider[0][i]+self.divider[1][i])/2 for i in (0,1))
            x = local(pose,mid)[0]
            remaining = x-cfg.get('parking_divider_trigger_x',cfg['wheelbase']+cfg['front_overhang'])
            self.debug['divider_remaining_m'] = remaining
            if remaining <= .015:
                if remaining < -.06:
                    return self.stopped('parking_divider_overshot',True)
                self.phase,self.turn_start = 'LOCK',pose
                return self.stopped('parking_far_start_reached')
            heading = wrap(self.start[2]-pose[2])
            return cfg.get('parking_entry_speed_raw',12),max(-.25,min(.25,heading))*cfg.get('steering_command_scale_rad',cfg['max_steer'])
        progress = self.side*wrap(pose[2]-self.turn_start[2])
        self.debug['turn_progress_deg'] = math.degrees(progress)
        if self.phase in ('LOCK','ALIGN') and progress < -math.radians(10):
            return self.stopped('parking_wrong_turn_direction',True)
        if self.phase == 'LOCK':
            capture = False
            if self.view:
                heading,_ = self.errors(self.view['theta'],self.view['center'])
                capture = progress>=math.radians(30) and abs(heading)<=math.radians(50)
            if capture:
                self.phase = 'ALIGN'
            elif progress>math.radians(110) or distance(pose,self.turn_start)>1.6:
                return self.stopped('parking_no_side_capture',True)
            else:
                return cfg.get('parking_entry_speed_raw',12),self.side*cfg.get('steering_command_scale_rad',cfg['max_steer'])
        if self.phase == 'ALIGN':
            if progress>math.radians(110) or distance(pose,self.turn_start)>1.6:
                return self.stopped('parking_alignment_bound',True)
            if self.view is None:
                return self.stopped('parking_need_two_sides')
            heading,lateral = self.errors(self.view['theta'],self.view['center'])
            self.debug.update(heading_deg=math.degrees(heading),lateral_m=lateral)
            aligned = abs(heading)<=math.radians(cfg.get('parking_align_heading_deg',6)) and abs(lateral)<=cfg.get('parking_align_lateral_m',.035)
            if not aligned:
                self.aligned_since = None
            elif self.view['stamp']>self.last_aligned_stamp:
                self.last_aligned_stamp = self.view['stamp']
                if self.aligned_since is None:
                    self.aligned_since = self.view['stamp']
                if self.view['stamp']-self.aligned_since>=cfg.get('parking_align_stable_s',.5):
                    remaining = self.remaining()
                    if remaining is None or now-self.bottom_stamp>cfg.get('ground_timeout',1.25):
                        return self.stopped('parking_need_fresh_bottom')
                    if remaining>cfg.get('parking_final_max_m',.30):
                        return self.stopped('parking_final_distance_too_long')
                    self.phase,self.advance_start,self.advance_at = 'ADVANCE',pose,now
                    self.final_heading,self.final_center = self.view['theta'],self.view['center']
            remaining = self.remaining()
            self.debug['remaining_m'] = remaining
            if self.phase == 'ALIGN':
                if remaining is not None and remaining<=.015:
                    return self.stopped('parking_bottom_reached_before_alignment')
                if aligned:
                    return self.stopped('parking_confirming_alignment')
                return cfg.get('parking_entry_speed_raw',12),self.steer(self.view['theta'],self.view['center'])
        if self.phase == 'ADVANCE':
            # A source image must stay fresh even when the bottom leaves the view.
            # Its last measured world position can bridge only this bounded finish.
            remaining = self.remaining()
            self.debug.update(remaining_m=remaining,distance_source='live_bottom' if now-self.bottom_stamp<=cfg.get('ground_timeout',1.25) else 'measured_bottom_plus_pose')
            if (remaining is None or distance(pose,self.advance_start)>cfg.get('parking_final_max_m',.30)+.02 or now-self.advance_at>8):
                return self.stopped('parking_final_bound',True)
            theta,center = (self.view['theta'],self.view['center']) if self.view else (self.final_heading,self.final_center)
            heading,lateral = self.errors(theta,center)
            self.debug.update(heading_deg=math.degrees(heading),lateral_m=lateral)
            if abs(heading)>math.radians(10) or abs(lateral)>.05:
                return self.stopped('parking_final_alignment_lost')
            if remaining<=.015:
                if remaining<-.025:
                    return self.stopped('parking_bottom_overshot',True)
                self.phase,self.settle_at = 'SETTLE',now
                return self.stopped('parking_at_bottom_clearance')
            return cfg.get('parking_final_speed_raw',8),self.steer(theta,center)
        return self.stopped('parking_invalid_phase',True)
