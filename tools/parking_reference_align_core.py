"""Measured midpoint/reference-line alignment trial; no ROS or actuator imports."""
from __future__ import division
import math
import numpy as np
from parking_line_stop_core import StopRun


def front_line_gap(line, wheelbase=.26):
    """Signed perpendicular distance from front-axle centre to the observed line."""
    a,b = np.asarray(line,dtype=float)
    delta = b-a
    if not np.all(np.isfinite([a,b])) or np.linalg.norm(delta) < .1:
        raise ValueError('invalid reference line')
    normal = np.array([delta[1],-delta[0]])/np.linalg.norm(delta)
    if normal[0] < 0:
        normal = -normal
    if normal[0] < math.cos(math.radians(45)):
        raise ValueError('reference is not transverse to vehicle')
    return float(np.dot(a-np.array([wheelbase,0.]),normal))


def dual_midline(boundaries):
    """Same-frame measured boundaries only; no inferred missing opposite side."""
    output = dict(valid=False,points=[],reason='need_two_observed_boundaries')
    sides = []
    for name in ('LEFT','RIGHT'):
        p = np.asarray(boundaries.get(name,[]),dtype=float)
        if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.all(np.isfinite(p)):
            return output
        p = p[np.argsort(p[:,0])]
        if np.any(np.diff(p[:,0]) <= 0):
            return output
        sides.append(p)
    left,right = sides
    lo,hi = max(.15,left[0,0],right[0,0]),min(1.3,left[-1,0],right[-1,0])
    if hi-lo < .20:
        output['reason'] = 'short_shared_boundary_span'
        return output
    x = np.linspace(lo,hi,max(5,min(20,int((hi-lo)/.04)+1)))
    y1 = np.interp(x,left[:,0],left[:,1])
    y2 = np.interp(x,right[:,0],right[:,1])
    width = y1-y2
    if np.any(width < .35) or np.any(width > 1.05) or np.ptp(width) > .20:
        output['reason'] = 'invalid_boundary_pair_width'
        return output
    output.update(valid=True,points=[[float(a),float(b)] for a,b in zip(x,(y1+y2)/2.)],
                  reason='measured_two_side_midpoint')
    return output


def midpoint_pose(midline, wheelbase=.26):
    result = dict(valid=False,lateral_m=None,heading_deg=None,curvature=None)
    if not midline or not midline.get('valid'):
        return result
    p = np.asarray(midline['points'],dtype=float)
    # Limit the fit to the closest supported 35 cm: do not aim at the far exit bend.
    p = p[p[:,0] <= p[0,0]+.35+1e-9]
    if len(p) < 4 or np.ptp(p[:,0]) < .20:
        return result
    center = float(np.mean(p[:,0]))
    x = p[:,0]-center
    a,b,c = np.polyfit(x,p[:,1],2)
    if np.max(abs(p[:,1]-(a*x*x+b*x+c))) > .025:
        return result
    # Extrapolate only from a straight fitted segment to the front axle.
    curvature = float(2*a/(1+b*b)**1.5)
    if abs(curvature) > .35:
        return result
    xf = wheelbase-center
    result.update(valid=True,lateral_m=float(a*xf*xf+b*xf+c),
                  heading_deg=math.degrees(math.atan(2*a*xf+b)),curvature=curvature)
    return result


class FixedTerminalLock(object):
    def __init__(self,wheelbase=.26):
        self.wheelbase = wheelbase
        self.locked = False
        self.stamp = None
        self.last_seen = None
        self.gap = None
        self.pending = None
        self.votes = 0
        self.reason = 'searching_unique_row_terminal'

    def observe(self, lines, stamp):
        if self.stamp is not None and stamp <= self.stamp:
            return False
        self.stamp = stamp
        if len(lines) != 1:
            self.reason = 'reference_missing' if not lines else 'reference_ambiguous'
            if not self.locked:
                self.pending,self.votes = None,0
            return False
        try:
            gap = front_line_gap(lines[0],self.wheelbase)
        except (ValueError,TypeError):
            self.reason = 'invalid_reference_geometry'
            if not self.locked:
                self.pending,self.votes = None,0
            return False
        if self.locked:
            dt = max(0.,stamp-self.last_seen)
            if abs(gap-self.gap) > min(.25,.06+.35*dt):
                self.reason = 'reference_identity_jump'
                return False
        else:
            if self.pending is not None and abs(gap-self.pending) <= .12:
                self.votes += 1
            else:
                self.votes = 1
            self.pending = gap
            if self.votes < 3:
                self.reason = 'confirming_unique_row_terminal'
                return False
            self.locked = True
        self.gap,self.last_seen = gap,stamp
        self.reason = 'locked_row_terminal'
        return True


class ReferenceAlignRun(StopRun):
    observe_only = True
    def __init__(self, base, follower, target_gap=.70, creep_speed=12,
                 gap_tolerance=.06,lateral_tolerance=.04,heading_tolerance=4.,
                 stop_margin=.04,stable_seconds=.4,reference_timeout=.6):
        self.__dict__.update(base.__dict__)
        self.follower = follower
        self.target_gap,self.creep_speed = target_gap,creep_speed
        self.gap_tolerance,self.lateral_tolerance = gap_tolerance,lateral_tolerance
        self.heading_tolerance,self.stop_margin = heading_tolerance,stop_margin
        self.stable_seconds,self.reference_timeout = stable_seconds,reference_timeout
        self.reference = FixedTerminalLock()
        self.scene = {}
        self.pose = dict(valid=False,lateral_m=None,heading_deg=None)
        self.current_reference = False
        self.phase = 'APPROACH'
        self.stopped_at = None
        self.verify_since,self.verify_votes = None,0
        self.last_scene_stamp = None
        self.history = []

    def observe_lane(self,observation,now):
        if not self.finished and self.phase == 'APPROACH':
            self.follower.observe(observation,now)

    def observe_scene(self,scene):
        self.scene = scene

    def observe(self,count,stamp,received,both_curved=False):
        if self.finished or (self.stamp is not None and stamp <= self.stamp):
            return
        self.stamp,self.received = stamp,received
        self.line_count,self.both_curved = count,both_curved
        self.last_scene_stamp = stamp
        reference = self.scene.get('fixed_reference',{})
        if reference.get('stamp') != stamp:
            self.current_reference = False
        else:
            self.current_reference = self.reference.observe(reference.get('p1_lines',[]),stamp)
        self.seen = self.reference.locked
        self.pose = midpoint_pose(self.scene.get('dual_midline'))
        qualified = (self.phase == 'HOLD' and self.current_reference and self.pose['valid'] and
                     abs(self.reference.gap-self.target_gap) <= self.gap_tolerance and
                     abs(self.pose['lateral_m']) <= self.lateral_tolerance and
                     abs(self.pose['heading_deg']) <= self.heading_tolerance)
        if qualified:
            self.verify_votes += 1
            if self.verify_since is None:
                self.verify_since = received
        else:
            self.verify_votes,self.verify_since = 0,None
        self.missing_count = self.verify_votes
        self.history.append(dict(stamp=stamp,gap_m=self.reference.gap,
                                 reference_current=self.current_reference,
                                 reference_reason=self.reference.reason,
                                 pose=dict(self.pose),phase=self.phase))
        self.history = self.history[-600:]

    def abort(self,reason):
        self.finished,self.reason = True,reason
        self.phase = 'FINISHED'
        return (0,0)

    def brake(self,now,reason):
        self.phase,self.reason,self.stopped_at = 'HOLD',reason,now
        self.stop_candidate_since = now
        return (0,0)

    def tick(self,now,ros_now):
        if self.finished:
            return (0,0)
        if self.received is None or self.stamp is None or not 0 <= now-self.received <= self.camera_timeout or not 0 <= ros_now-self.stamp <= self.camera_timeout:
            return self.abort('CAMERA_TIMEOUT')
        if self.started is None:
            return (0,0)
        if self.phase == 'HOLD':
            if (now-self.stopped_at >= .5 and self.verify_since is not None and
                    self.verify_votes >= 3 and self.received-self.verify_since >= self.stable_seconds):
                return self.abort('ALIGNMENT_COMPLETE')
            if now-self.stopped_at >= 2.:
                return self.abort('ALIGNMENT_UNCONFIRMED_STOPPED')
            self.reason = 'STOP_VERIFY_POSE'
            return (0,0)
        if now-self.started >= self.max_seconds:
            return self.abort('ALIGNMENT_RUN_TIMEOUT')
        if not self.reference.locked:
            if now-self.started >= self.seek_seconds:
                return self.abort('REFERENCE_NOT_FOUND')
            command = self.follower.command(ros_now)
            self.reason = 'SEARCH_ROW_TERMINAL' if command[0] > 0 else 'WAIT_LANE'
            # Slow down even before the third identity-confirming frame.
            # If already too near, stop rather than overshoot while acquiring.
            if self.reference.votes and self.reference.pending <= self.target_gap+.40:
                if self.reference.pending <= self.target_gap+self.stop_margin:
                    self.reason = 'STOP_CONFIRM_REFERENCE'
                    return (0,0)
                return (min(command[0],self.creep_speed),command[1])
            return command
        if not self.current_reference or ros_now-self.reference.last_seen > self.reference_timeout:
            self.reason = 'STOP_REFERENCE_MISSING'
            if ros_now-self.reference.last_seen > 1.:
                return self.abort('REFERENCE_LOST_STOPPED')
            return (0,0)
        gap = self.reference.gap
        if gap < self.target_gap-self.gap_tolerance:
            return self.abort('REFERENCE_TOO_CLOSE_STOPPED')
        if gap <= self.target_gap+self.stop_margin:
            return self.brake(now,'STOP_AT_REFERENCE_DISTANCE')
        command = self.follower.command(ros_now)
        if command[0] <= 0:
            self.reason = 'WAIT_LANE'
            return (0,0)
        if gap-self.target_gap <= .40:
            self.reason = 'CREEP_ALIGN_REFERENCE'
            return (min(command[0],self.creep_speed),command[1])
        self.reason = 'FOLLOW_MEASURED_MIDLINE' if self.scene.get('dual_midline',{}).get('valid') else 'FOLLOW_LANE_SEEK_TWO_SIDES'
        return command

    def diagnostics(self,now):
        data = self.follower.diagnostics(now)
        data.update(reference_locked=self.reference.locked,gap_m=self.reference.gap,
                    target_gap_m=self.target_gap,reference_reason=self.reference.reason,
                    midpoint_pose=dict(self.pose),stable_frames=self.verify_votes)
        return data

    def format_diagnostics(self,now):
        d = self.diagnostics(now)
        def value(v):return 'UNKNOWN' if v is None else '%.3f' % v
        return ('reference=%s gap_m=%s target_m=%.2f lateral_m=%s heading_deg=%s '
                'dual_pose=%s stable=%d lane_age=%.3fs lane_reason=%s' %
                (self.reference.reason,value(self.reference.gap),self.target_gap,
                 value(self.pose['lateral_m']),value(self.pose['heading_deg']),
                 self.pose['valid'],self.verify_votes,d['lane_age_s'],d['lane_reason']))
