"""Sensor-relative U-turn execution; deliberately independent of command odometry.

Scene and pose must use one fixed local frame. Unknown free space is forbidden.
"""
import math
try:
    string_types=(basestring,)
except NameError:
    string_types=(str,)
from robot.common.contracts import number
from robot.common.contracts import model_to_command_steering
from robot.common.geometry import collision
from robot.common.geometry import distance
from robot.common.geometry import footprint
from robot.common.geometry import wrap
from robot.common.planning import Follower
from robot.uturn.planner import plan_uturn
from robot.uturn.planner import uturn_goal
from robot.uturn.planner import uturn_goal_candidates


try:
    _isfinite = math.isfinite
except AttributeError:
    def _isfinite(value):
        return not math.isnan(value) and not math.isinf(value)


_EPS = 1e-9


def _finite(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError):
        raise ValueError('invalid '+name)
    if not _isfinite(value):
        raise ValueError('invalid '+name)
    return value


def _orientation(a, b, c):
    return ((b[0]-a[0])*(c[1]-a[1]) -
            (b[1]-a[1])*(c[0]-a[0]))


def _on_segment(a, b, p):
    return (abs(_orientation(a,b,p)) <= _EPS and
            min(a[0],b[0])-_EPS <= p[0] <= max(a[0],b[0])+_EPS and
            min(a[1],b[1])-_EPS <= p[1] <= max(a[1],b[1])+_EPS)


def _segments_intersect(a, b, c, d):
    ab_c, ab_d = _orientation(a,b,c), _orientation(a,b,d)
    cd_a, cd_b = _orientation(c,d,a), _orientation(c,d,b)
    crosses_ab = ((ab_c > _EPS and ab_d < -_EPS) or
                  (ab_c < -_EPS and ab_d > _EPS))
    crosses_cd = ((cd_a > _EPS and cd_b < -_EPS) or
                  (cd_a < -_EPS and cd_b > _EPS))
    if crosses_ab and crosses_cd:
        return True
    return (_on_segment(a,b,c) or _on_segment(a,b,d) or
            _on_segment(c,d,a) or _on_segment(c,d,b))


def _validate_convex_polygon(points):
    """Reject duplicate, collinear, concave, or self-crossing rings."""
    if len(set(points)) != len(points):
        raise ValueError('region has duplicate points')
    area2 = sum(a[0]*b[1]-b[0]*a[1]
                for a,b in zip(points,points[1:]+points[:1]))
    if abs(area2) <= _EPS:
        raise ValueError('region has zero area')
    crosses=[]
    for i in range(len(points)):
        crosses.append(_orientation(points[i-2],points[i-1],points[i]))
    if not (all(v > _EPS for v in crosses) or
            all(v < -_EPS for v in crosses)):
        raise ValueError('region must be strictly convex')
    # The turn-sign test rejects the usual bow-tie, but explicit edge checks
    # also reject less obvious self-intersections before polygon_contains is
    # allowed to use the ring.
    count=len(points)
    for i in range(count):
        a,b=points[i],points[(i+1)%count]
        for j in range(i+1,count):
            if i == j or (i+1)%count == j or (j+1)%count == i:
                continue
            if _segments_intersect(a,b,points[j],points[(j+1)%count]):
                raise ValueError('region edges self-intersect')


def _config_limits(cfg):
    """Validate the relative-U-turn tuning surface without changing globals."""
    cfg=cfg or {}
    spacing=_finite(cfg.get('uturn_lane_spacing',.60),'uturn lane spacing')
    if not .05 <= spacing <= 2.0:
        raise ValueError('uturn lane spacing must be in [0.05,2.0] m')
    low=_finite(cfg.get('uturn_goal_forward_min',0.0),'uturn goal forward minimum')
    high=_finite(cfg.get('uturn_goal_forward_max',0.0),'uturn goal forward maximum')
    step=_finite(cfg.get('uturn_goal_forward_step',.10),'uturn goal forward step')
    if not -2.0 <= low <= high <= 2.0 or not .01 <= step <= 1.0:
        raise ValueError('invalid uturn goal forward range')
    tolerance=_finite(cfg.get('uturn_goal_heading_tolerance_deg',10.0),
                      'uturn goal heading tolerance')
    if not 1.0 <= tolerance <= 45.0:
        raise ValueError('uturn goal heading tolerance must be in [1,45] deg')
    tracking=_finite(cfg.get('uturn_tracking_error_max',.15),
                     'uturn tracking error maximum')
    if not .01 <= tracking <= .75:
        raise ValueError('uturn tracking error maximum must be in [.01,.75] m')
    # Recovery is opt-in.  A caller must schedule ``replan`` while the
    # vehicle is stopped; the normal command path never blocks on planning.
    limit=_finite(cfg.get('uturn_replan_limit',0),'uturn replan limit')
    if limit != int(limit) or not 0 <= limit <= 3:
        raise ValueError('uturn replan limit must be an integer in [0,3]')
    cooldown=_finite(cfg.get('uturn_replan_cooldown_s',
                             cfg.get('uturn_replan_cooldown',1.0)),
                     'uturn replan cooldown')
    if not 0.0 <= cooldown <= 10.0:
        raise ValueError('uturn replan cooldown must be in [0,10] s')
    return dict(spacing=spacing,heading_tolerance=math.radians(tolerance),
                tracking_error=tracking, replan_limit=int(limit),
                replan_cooldown=cooldown)

def polygon_contains(polygon, point):
    if not isinstance(point,(list,tuple)) or len(point) != 2:
        return False
    try:
        x,y=(float(point[0]),float(point[1]))
    except (TypeError,ValueError):
        return False
    if not _isfinite(x) or not _isfinite(y):
        return False
    inside=False
    for i in range(len(polygon)):
        a,b=polygon[i],polygon[(i+1)%len(polygon)]
        if _on_segment(a,b,(x,y)):
            return True
        if (a[1]>y)!=(b[1]>y):
            cross=a[0]+(y-a[1])*(b[0]-a[0])/(b[1]-a[1])
            if x<cross:inside=not inside
    return inside


def _compile_allowed(regions):
    """Build a fast union predicate for decoded, strictly convex regions.

    ``decode_scene`` has already checked each ring, so a point is inside a
    ring when it is on the interior side of every edge.  Precomputing the
    edge equations removes the per-call divisions in the generic
    ray-casting predicate.  The same orientation test includes boundary
    points, which is required for a vehicle footprint that touches a white
    line at the edge of the observed drivable area.
    """
    compiled=[]
    rectangles=[]
    for region in regions:
        xs,ys=set(p[0] for p in region),set(p[1] for p in region)
        if len(region)==4 and len(xs)==2 and len(ys)==2:
            rectangles.append((min(xs),max(xs),min(ys),max(ys)))
            continue
        area2=sum(a[0]*b[1]-b[0]*a[1]
                  for a,b in zip(region,region[1:]+region[:1]))
        sign=1.0 if area2>0 else -1.0
        edges=[]
        for a,b in zip(region,region[1:]+region[:1]):
            # cross((b-a),(point-a)) = A*x + B*y + C
            ax,ay=a
            bx,by=b
            edges.append((sign*-(by-ay), sign*(bx-ax),
                          sign*((by-ay)*ax-(bx-ax)*ay)))
        compiled.append(tuple(edges))
    compiled=tuple(compiled)

    def allowed(point):
        # All callers pass points generated from a finite decoded scene and
        # vehicle geometry.  Keep this hot path free of conversion/division;
        # decode_scene remains the trust boundary for external values.
        x,y=point
        for x0,x1,y0,y1 in rectangles:
            if x0-_EPS<=x<=x1+_EPS and y0-_EPS<=y<=y1+_EPS:
                return True
        for edges in compiled:
            inside=True
            for a,b,c in edges:
                if a*x+b*y+c < -_EPS:
                    inside=False
                    break
            if inside:
                return True
        return False
    return allowed


def decode_scene(data):
    if not isinstance(data,dict):
        raise ValueError('invalid scene')
    if 'frame' not in data:
        raise ValueError('invalid scene frame')
    frame=data['frame']
    if not isinstance(frame,string_types) or not frame or len(frame)>100:
        raise ValueError('invalid scene frame')
    side=data.get('followed_boundary')
    if side not in ('LEFT','RIGHT'):raise ValueError('unknown followed boundary')
    def points(rows,limit):
        if not isinstance(rows,list) or not 3<=len(rows)<=limit:
            raise ValueError('invalid polygon')
        result=[]
        for row in rows:
            if not isinstance(row,(list,tuple)) or len(row)!=2:raise ValueError('invalid point')
            p=tuple(number(v) for v in row)
            if max(abs(v) for v in p)>20:raise ValueError('point outside local area')
            result.append(p)
        _validate_convex_polygon(result)
        return result
    regions=data.get('regions')
    if not isinstance(regions,list) or not 1<=len(regions)<=16:
        raise ValueError('unknown drivable area')
    regions=[points(p,64) for p in regions]
    obstacles=data.get('obstacles',[])
    if not isinstance(obstacles,list) or len(obstacles)>2000:raise ValueError('invalid obstacles')
    obs=[]
    for p in obstacles:
        if not isinstance(p,(list,tuple)) or len(p)!=2:raise ValueError('invalid obstacle')
        obstacle=tuple(number(v) for v in p)
        if max(abs(v) for v in obstacle)>20:raise ValueError('obstacle outside local area')
        obs.append(obstacle)
    raw_pose=data.get('pose')
    if not isinstance(raw_pose,(list,tuple)) or len(raw_pose)!=3:
        raise ValueError('invalid measured pose')
    pose=tuple(number(v) for v in raw_pose)
    if max(abs(v) for v in pose[:2])>20:raise ValueError('pose outside local area')
    if not -math.pi <= pose[2] <= math.pi:
        raise ValueError('pose yaw must be in [-pi,pi]')
    if data.get('pose_source') not in ('vision','lidar','fused'):
        raise ValueError('measured pose required')
    if 'stamp' not in data:
        raise ValueError('invalid scene stamp')
    reference = data.get('lane_reference')
    if reference is not None:
        if not isinstance(reference,(list,tuple)) or len(reference)!=3:
            raise ValueError('invalid lane reference')
        reference = tuple(number(v) for v in reference)
        if max(abs(v) for v in reference[:2])>20 or not -math.pi<=reference[2]<=math.pi:
            raise ValueError('invalid lane reference')
    return dict(frame=frame,followed_boundary=side,regions=regions,obstacles=obs,
                pose=pose,pose_source=data['pose_source'],
                lane_reference=reference,
                stamp=number(data['stamp']))


class UturnFollower(Follower):
    """Accurate gear-change points without tightening the final lane tolerance."""
    def __init__(self,path,cfg):
        Follower.__init__(self,path,dict(cfg))
        self.final_position_tolerance=cfg['path_goal_tolerance']
        self.final_heading_tolerance=cfg['path_yaw_tolerance']

    def command(self,pose,now):
        final=self.segment_end==len(self.path)-1
        self.goal_tolerance=(self.final_position_tolerance if final else .03)
        self.cfg['path_yaw_tolerance']=(self.final_heading_tolerance if final else math.radians(3))
        from robot.uturn.calibration import limit
        if self.cfg.get('uturn_calibration') is not None:
            # Let pure pursuit compute its angle before applying the actual
            # gear/side limit (a cusp can switch gear inside command()).
            self.cfg['max_steer']=max(limit(self.cfg,d,s) for d in (1,-1) for s in (1,-1))
        speed,steer=Follower.command(self,pose,now)
        maximum=limit(self.cfg,1 if speed>=0 else -1,steer)
        return speed,max(-maximum,min(maximum,steer))


class RelativeUturn(object):
    def __init__(self,cfg,scene):
        self.cfg=dict(cfg)
        lookahead=_finite(cfg.get('uturn_lookahead',.15),'uturn lookahead')
        if not .05<=lookahead<=.5:
            raise ValueError('uturn lookahead must be in [.05,.5] m')
        self.cfg['action_lookahead']=lookahead
        if 'uturn_speed_raw' in cfg:
            speed=_finite(cfg['uturn_speed_raw'],'uturn raw speed')
            if speed!=int(speed) or not 1<=speed<=30:
                raise ValueError('uturn raw speed must be integer in [1,30]')
            self.cfg['speed_raw']=dict(cfg['speed_raw'],action=int(speed))
        limits=_config_limits(self.cfg)
        self.cfg['uturn_lane_spacing']=limits['spacing']
        # The U-turn goal tolerance is local to this follower.  Do not alter
        # the global lane/turn tolerance used by the ordinary controller.
        self.cfg['path_yaw_tolerance']=limits['heading_tolerance']
        self.goal_heading_tolerance=limits['heading_tolerance']
        self.tracking_error_max=limits['tracking_error']
        self.replan_limit=limits['replan_limit']
        self.replan_cooldown=limits['replan_cooldown']
        self.scene=scene
        self.frame=scene['frame']
        self.side=scene['followed_boundary']
        self._allowed_regions=_compile_allowed(scene['regions'])
        self.start=scene['pose']
        self.reference=scene.get('lane_reference') or self.start
        if (distance(self.start,self.reference)>.10 or
                abs(wrap(self.start[2]-self.reference[2]))>math.radians(10)):
            raise ValueError('uturn approach not aligned with reference lane')
        self.goal=uturn_goal_candidates(self.reference,self.side,self.cfg)[0][0]
        self.target_goal=self.goal
        self.follower=None
        self.phase='PLAN'
        self.reason='relative_uturn_plan'
        self.last_confirm=-1
        self.confirmations=0
        self.replan_attempts=0
        self.last_replan=-float('inf')
        self.reference_invalid=False
        self.validated_stamp=None

    def allowed(self,p):
        return self._allowed_regions(p)

    def plan(self):
        scene=self.scene
        # Capture the region predicate from the same scene snapshot as the
        # obstacles.  ``observe`` may replace the live scene concurrently.
        allowed=_compile_allowed(scene['regions'])
        return plan_uturn(self.start,self.side,self.cfg,allowed,scene['obstacles'],
                          lane_spacing=self.cfg['uturn_lane_spacing'],
                          goal=self.target_goal if scene.get('lane_reference') is not None else None)

    def accept_plan(self,result):
        path,reason=result
        self.reason=reason
        if not path:
            self.phase='BLOCKED'
            return
        self.follower=UturnFollower(path,self.cfg)
        self.validated_stamp=None
        # Keep the selected goal in the original scene frame.  A later
        # recovery plan must not add another lane spacing to the measured
        # pose.  The path endpoint is used only as a follower waypoint.
        candidates=uturn_goal_candidates(self.reference,self.side,self.cfg)
        self.target_goal=min((candidate[0] for candidate in candidates),
                             key=lambda candidate:
                             ((candidate[0]-path[-1][0])**2+
                              (candidate[1]-path[-1][1])**2))
        self.goal=self.target_goal
        self.phase='TRACK'

    def replan(self, now=None):
        """Bounded local replan from the newest measured pose.

        This method is intentionally explicit: an integration layer may call
        it after a path is blocked or tracking error is detected.  It never
        falls back to a timed maneuver and returns False while leaving the
        task stopped.  Attempt count includes failed searches.
        """
        if now is None:
            now=self.scene['stamp']
        now=_finite(now,'replan time')
        if self.phase not in ('TRACK','CONFIRM','BLOCKED'):
            self.reason='relative_uturn_replan_unavailable'
            return False
        if self.reference_invalid:
            # A changed frame or followed boundary invalidates the target;
            # replanning in a new frame could silently select the wrong lane.
            self.reason='relative_uturn_replan_reference_invalid'
            return False
        if now-self.last_replan < self.replan_cooldown:
            self.reason='relative_uturn_replan_cooldown'
            return False
        if self.replan_attempts >= self.replan_limit:
            self.reason='relative_uturn_replan_limit'
            return False
        if not 0 <= now-self.scene['stamp'] <= self.cfg['sensor_timeout']:
            self.reason='relative_uturn_pose_stale'
            return False
        self.replan_attempts += 1
        self.last_replan=now
        # Observe replaces the scene object rather than mutating it, but take
        # an immutable snapshot so an asynchronous planner sees one coherent
        # pose/map while the ROS callback continues receiving observations.
        source=self.scene
        scene=dict(source,
                   pose=tuple(source['pose']),
                   regions=tuple(tuple(tuple(point) for point in region)
                                 for region in source['regions']),
                   obstacles=tuple(tuple(point) for point in source['obstacles']))
        allowed=_compile_allowed(scene['regions'])
        result=plan_uturn(scene['pose'],self.side,self.cfg,allowed,
                          scene['obstacles'],lane_spacing=self.cfg['uturn_lane_spacing'],
                          goal=self.target_goal)
        path,reason=result
        if not path:
            self.phase='BLOCKED'
            self.reason='relative_uturn_replan_failed'
            return False
        self.follower=UturnFollower(path,self.cfg)
        self.validated_stamp=None
        self.goal=self.target_goal
        self.phase='TRACK'
        self.confirmations=0
        self.reason='relative_uturn_replanned'
        return True

    def observe(self,scene):
        if (scene['frame']!=self.frame or scene['followed_boundary']!=self.side or
                scene.get('lane_reference')!=self.scene.get('lane_reference')):
            self.reference_invalid=True
            self.phase='BLOCKED'
            self.reason='relative_uturn_reference_changed'
            return
        if scene['stamp']>self.scene['stamp']:
            self.scene=scene
            self._allowed_regions=_compile_allowed(scene['regions'])

    def command(self,now):
        if self.phase not in ('TRACK','CONFIRM'):return 0,0.0
        if not 0<=now-self.scene['stamp']<=self.cfg['sensor_timeout']:
            self.confirmations=0
            self.reason='relative_uturn_pose_stale'
            return 0,0.0
        pose=self.scene['pose']
        path=self.follower.path[self.follower.index:]
        # Revalidate the remaining swept body when the observed map changes.
        # A frame is immutable. Rechecking the same full path at every command
        # tick wastes Nano CPU; a newer measured scene always revalidates it.
        check = [pose]+path if self.validated_stamp!=self.scene['stamp'] else [pose]
        if any(collision(p,self.scene['obstacles'],self.cfg) or
               not all(self.allowed(q) for q in footprint(p,self.cfg,spacing=.025)) for p in check):
            self.phase='BLOCKED'
            self.reason='relative_uturn_path_blocked'
            return 0,0.0
        self.validated_stamp=self.scene['stamp']
        nearest=min(path,key=lambda p:distance(pose,p))
        if distance(pose,nearest)>self.tracking_error_max:
            self.phase='BLOCKED'
            self.reason='relative_uturn_tracking_error'
            return 0,0.0
        if self.phase=='CONFIRM':
            good=(distance(pose,self.goal)<=self.cfg['path_goal_tolerance'] and
                  abs(wrap(pose[2]-self.goal[2]))<=self.goal_heading_tolerance)
            if self.scene['stamp']>self.last_confirm:
                self.confirmations=self.confirmations+1 if good else 0
                self.last_confirm=self.scene['stamp']
            if self.confirmations>=self.cfg['exit_frames']:
                self.phase='DONE'
                self.reason='relative_uturn_complete'
            return 0,0.0
        speed,steer=self.follower.command(pose,now)
        self.reason='relative_uturn_tracking'
        if self.follower.done:
            self.phase='CONFIRM'
            self.reason='relative_uturn_confirming'
        if self.cfg.get('uturn_calibration') is not None:
            from robot.uturn.calibration import command_angle
            return speed,command_angle(self.cfg,1 if speed>=0 else -1,steer)
        return speed,model_to_command_steering(steer,self.cfg)
