"""Two-image P1 confirmation and image/timed approach, without optical flow."""
from __future__ import division
import math
from collections import deque
from .open_loop_core import finite, OneShot
from ..motion.calibration import validate_speed_points, speed_from_points

SEARCH_DEFAULTS = dict(seek_speed_raw=15, creep_speed_raw=15, wheelbase_m=.26,
                       line_tolerance_m=.02, stop_delay_s=.10,
                       seek_max_m=3., seek_timeout_s=20., camera_timeout_s=.4,
                       settle_seconds=.7, forward_mps_per_raw=.008, forward_speed_table=None)


def front_gap(line,wheelbase):
    a,b=line
    dx,dy=b[0]-a[0],b[1]-a[1]
    return a[0]-a[1]*dx/dy-wheelbase


class ReferenceRun(object):
    def __init__(self, stages, reference_only=False, **options):
        if not isinstance(reference_only,bool): raise ValueError('reference_only must be a boolean')
        self.reference_only=reference_only
        cfg=dict(SEARCH_DEFAULTS)
        if set(options)-set(cfg): raise ValueError('unknown reference-search settings')
        cfg.update(options)
        self.seek=finite(cfg['seek_speed_raw'],'seek_speed_raw',1,30,True)
        self.creep=finite(cfg['creep_speed_raw'],'creep_speed_raw',1,self.seek,True)
        self.wheelbase=finite(cfg['wheelbase_m'],'wheelbase_m',.1,.5)
        self.tolerance=finite(cfg['line_tolerance_m'],'line_tolerance_m',.005,.05)
        self.stop_delay=finite(cfg['stop_delay_s'],'stop_delay_s',0,.5)
        self.max_distance=finite(cfg['seek_max_m'],'seek_max_m',.1,3)
        self.timeout=finite(cfg['seek_timeout_s'],'seek_timeout_s',1,30)
        self.camera_timeout=finite(cfg['camera_timeout_s'],'camera_timeout_s',.1,.5)
        self.settle=finite(cfg['settle_seconds'],'settle_seconds',.3,2)
        self.speed_coefficient=finite(cfg['forward_mps_per_raw'],'forward_mps_per_raw',.0001,.1)
        self.speed_table=cfg['forward_speed_table']
        if self.speed_table is not None:validate_speed_points(self.speed_table)
        self.countdown=stages[0].seconds
        self.motion_task=OneShot(stages[1:])
        self.phase='COUNTDOWN'
        self.reason='NOT_STARTED'
        self.finished=False
        self.deadline=self.last=self.search_started=self.start_distance=None
        self.observation=self.line=self.pending=None
        self.pending_count=0
        self.processed_stamp=-1.
        self.gap=None
        self.velocity=0.
        self.stopped_at=None
        self.distance=0.;self.command_speed=0;self.motion_time=None
        self.history=deque(maxlen=40)
        self.line_gap=self.line_distance=None
        self.gap_source=None

    def abort(self, reason):
        self.finished=True;self.reason=reason
        self.motion_task.abort(reason)
        return (0,0)

    def observe(self, data, received_at, ros_now):
        if not isinstance(data,dict) or data.get('source')!='p1_reference_camera':
            raise ValueError('invalid reference observation source')
        stamp=finite(data.get('stamp'),'reference stamp',0,1e12)
        age=finite(ros_now-stamp,'reference image age',0,self.camera_timeout)
        finite(received_at,'reference receive time',0,1e12)
        if data.get('frame')!='base_link':raise ValueError('reference frame must be base_link')
        if abs(finite(data.get('wheelbase_m'),'observed wheelbase',.1,.5)-self.wheelbase)>.001:
            raise ValueError('reference camera/runner wheelbase mismatch')
        lines=data.get('p1_lines')
        if not isinstance(lines,list) or len(lines)>8: raise ValueError('invalid P1 candidates')
        rows=[]
        for line in lines:
            if (not isinstance(line,(list,tuple)) or len(line)!=2 or
                    any(not isinstance(p,(list,tuple)) or len(p)!=2 for p in line)):
                raise ValueError('invalid P1 segment')
            points=tuple(tuple(finite(v,'P1 line point',-5,5) for v in p) for p in line)
            dx,dy=points[1][0]-points[0][0],points[1][1]-points[0][1]
            if not .28<=math.hypot(dx,dy)<=.44 or abs(dx)>.15*abs(dy):
                raise ValueError('invalid P1 segment geometry')
            rows.append(points)
        if self.observation is not None and stamp<=self.observation['stamp']: return
        self.observation=dict(stamp=stamp,age=age,received_at=received_at,
                              lines=rows,reason=data.get('reason'))

    def advance(self,now):
        if self.motion_time is not None:
            self.distance+=self.command_velocity()*max(0.,now-self.motion_time)
        self.motion_time=now
        if self.history and self.history[-1][0]==now:self.history.pop()
        self.history.append((now,self.distance))

    def command_velocity(self):
        speed=max(0,self.command_speed)
        return speed_from_points(speed,self.speed_table) if self.speed_table is not None else speed*self.speed_coefficient

    def set_command(self,speed,now):
        """The runtime overrides this with the command actually sent by lane following."""
        self.advance(now)
        self.command_speed=speed

    def distance_at(self,when):
        if when<=self.history[0][0]:return self.history[0][1]
        for a,b in zip(self.history,list(self.history)[1:]):
            if a[0]<=when<=b[0]:
                return a[1]+(b[1]-a[1])*(when-a[0])/(b[0]-a[0])
        return self.distance

    def tick(self, now):
        if self.finished: return (0,0)
        if (math.isnan(now) or math.isinf(now) or
                (self.last is not None and not 0<=now-self.last<=.25)):
            return self.abort('REFERENCE_CONTROL_LOOP_GAP')
        self.last=now
        self.advance(now)
        self.velocity=self.command_velocity()
        command=self.tick_reference(now)
        self.set_command(command[0],now)
        return command

    def tick_reference(self,now):
        if self.phase=='MANEUVER':
            return self.tick_maneuver(now)
        if self.deadline is None: self.deadline=now+self.countdown
        if self.phase=='COUNTDOWN':
            if now<self.deadline:
                self.reason='COUNTDOWN';return (0,0)
            self.phase='SEARCH_P1';self.search_started=now
        observed=self.observation
        if observed is None:
            if now-self.search_started>=8: return self.abort('REFERENCE_CAMERA_TIMEOUT')
            self.reason='WAIT_REFERENCE_CAMERA';return (0,0)
        result_gap=max(0.,now-observed['received_at'])
        # observe() checks image age at arrival. A normal processing delay
        # plus a normal result interval must not falsely look like a dropout.
        # Image age is handled by command history; bound the result gap here.
        if result_gap>self.camera_timeout:
            if self.line is not None: return self.abort('REFERENCE_CAMERA_STALE')
            self.pending=None;self.pending_count=0
            if now-self.search_started>=8: return self.abort('REFERENCE_CAMERA_TIMEOUT')
            self.reason='WAIT_REFERENCE_CAMERA';return (0,0)
        self.gap_source='timed_last_seen_line' if self.line is not None else None
        if observed['stamp']>self.processed_stamp:
            self.processed_stamp=observed['stamp']
            lines=observed['lines']
            if len(lines)==1:
                line=lines[0]
                source_distance=self.distance_at(observed['received_at']-observed['age'])
                measured_gap=front_gap(line,self.wheelbase)
                current_gap=measured_gap-(self.distance-source_distance)
                if self.line is None:
                    if (self.pending is not None and
                            abs(current_gap-(self.pending[1]-(self.distance-self.pending[2])))<.04 and
                            max(abs(a[1]-b[1]) for a,b in zip(line,self.pending[0]))<.08):
                        self.pending_count+=1
                    else: self.pending_count=1
                    self.pending=(line,current_gap,self.distance)
                    if self.pending_count>=2:
                        self.line=line;self.start_distance=self.distance;self.search_started=now
                        self.line_gap=measured_gap;self.line_distance=source_distance
                        self.gap_source='line_image'
                elif (abs(current_gap-(self.line_gap-(self.distance-self.line_distance)))<=.15 and
                      max(abs(a[1]-b[1]) for a,b in zip(line,self.line))<=.15):
                    self.line=line
                    self.line_gap=measured_gap;self.line_distance=source_distance
                    self.gap_source='line_image'
            elif self.line is None: self.pending=None;self.pending_count=0
        if self.line is None:
            self.reason='SEARCH_P1_LINE';return (self.seek,0)
        # The road lane controller still steers throughout this approach.
        # Bound the locked-line tracking, not normal lane recentering travel.
        if (now-self.search_started>=self.timeout or
                self.distance-self.start_distance>self.max_distance):
            return self.abort('P1_SEARCH_LIMIT')
        self.gap=self.line_gap-(self.distance-self.line_distance)
        if self.gap < -self.tolerance: return self.abort('FRONT_AXLE_PAST_P1_LINE')
        if self.phase=='STOP_AT_P1':
            self.reason='STOP_AT_P1_LINE'
            if now-self.stopped_at<self.settle: return (0,0)
            if observed['received_at']<=self.stopped_at:return (0,0)
            if abs(self.gap)<=self.tolerance:
                if self.reference_only:return self.abort('COMPLETE')
                self.phase='REFERENCE_ALIGNED';self.reason='REFERENCE_ALIGNED'
                self.deadline=now+.3;return (0,0)
            self.phase='APPROACH_P1'
        if self.phase=='REFERENCE_ALIGNED':
            if abs(self.gap)>self.tolerance:
                self.phase='APPROACH_P1'
            else:
                self.reason='REFERENCE_ALIGNED'
                if now<self.deadline: return (0,0)
                self.phase='MANEUVER'
                return self.tick_maneuver(now)
        # Image age is compensated through the recent command-distance history.
        predicted_gap=self.gap-self.velocity*(self.stop_delay+.05)
        if predicted_gap<=self.tolerance/2:
            # Estimate the short coasting distance after sending zero. A new
            # visible line can still correct this estimate during settling.
            coast=self.velocity*self.stop_delay
            self.distance+=coast;self.gap-=coast
            self.phase='STOP_AT_P1';self.stopped_at=now
            self.reason='STOP_AT_P1_LINE';return (0,0)
        self.phase='APPROACH_P1';self.reason='APPROACH_P1_LINE'
        return (self.creep if self.gap<.3 else self.seek,0)

    def tick_maneuver(self, now):
        command=self.motion_task.tick(now)
        self.reason,self.finished,self.deadline=(self.motion_task.reason,
            self.motion_task.finished,self.motion_task.deadline)
        return command
