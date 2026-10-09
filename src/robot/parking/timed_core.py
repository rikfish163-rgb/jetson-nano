"""Saved P1/P2/P3 approach and timed stages; one central actuator owner."""
from __future__ import division
import math
import json
import numpy as np
from robot.common.contracts import number,boolean
from robot.parallel_parking.open_loop_core import finite

def near_lane_geometry(observation):
    result = dict(valid=False, heading_deg=None, curvature=None)
    if not observation or observation.get('confidence', 0.) < .35:
        return result
    points = np.asarray(observation.get('points', []), dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        return result
    points = points[(points[:,0] >= .15) & (points[:,0] <= 1.1)]
    if len(points) < 3 or np.ptp(points[:,0]) < .20:
        return result
    x = points[:,0]-np.mean(points[:,0])
    a,b,c = np.polyfit(x,points[:,1],2)
    if np.max(abs(points[:,1]-(a*x*x+b*x+c))) > .035:
        return result
    heading = math.degrees(math.atan(b))
    curvature = 2*a/(1+b*b)**1.5
    result.update(valid=abs(heading) <= 60., heading_deg=float(heading),
                  curvature=float(curvature), lateral_m=float(c))
    return result


class StopRun(object):
    def __init__(self, speed=30, lost_seconds=.3, seen_frames=2,
                 seek_seconds=5., max_seconds=20., camera_timeout=.5, steering=-5,
                 lost_frames=3, end_verify_seconds=2.):
        self.speed, self.lost_seconds = speed, lost_seconds
        self.steering = steering
        self.seen_frames = seen_frames
        self.lost_frames = lost_frames
        self.seek_seconds, self.max_seconds = seek_seconds, max_seconds
        self.camera_timeout = camera_timeout
        self.started = None
        self.stamp, self.received = None, None
        self.seen_count, self.missing_count = 0, 0
        self.seen, self.missing_since = False, None
        self.both_curved, self.line_count = False, None
        self.stop_candidate_since = None
        self.end_verify_seconds = end_verify_seconds
        self.finished, self.reason = False, 'READY'

    def start(self, now):
        self.started = now

    def abort(self, reason):
        self.finished, self.reason = True, reason
        return (0, 0)

    def observe(self, count, stamp, received, both_curved=False):
        if self.finished or (self.stamp is not None and stamp <= self.stamp):
            return
        self.stamp, self.received = stamp, received
        self.line_count, self.both_curved = count, both_curved is True
        if self.started is None:
            # Countdown observations can establish visibility, but cannot stop
            # a run that has not started. Blank countdown frames disarm it.
            self.seen_count = self.seen_count+1 if count > 0 else 0
            self.seen = self.seen_count >= self.seen_frames
            return
        # Brake at the first jointly plausible end. Confirmation is performed
        # while stationary, and ambiguous observations must NEVER re-arm motion.
        if self.seen and count == 0 and self.both_curved and self.stop_candidate_since is None:
            self.stop_candidate_since = received
        if count > 0:
            self.seen_count += 1
            self.seen = self.seen or self.seen_count >= self.seen_frames
            self.missing_count, self.missing_since = 0, None
        else:
            self.seen_count = 0
            if self.seen and self.both_curved:
                self.missing_count += 1
                if self.missing_since is None:
                    self.missing_since = received
                if (self.missing_count >= self.lost_frames and
                        received-self.missing_since >= self.lost_seconds):
                    self.abort('BAY_END_CONFIRMED')
            else:
                self.missing_count, self.missing_since = 0, None

    def tick(self, now, ros_now):
        if self.finished:
            return (0, 0)
        if (self.received is None or not 0 <= now-self.received <= self.camera_timeout or
                self.stamp is None or not 0 <= ros_now-self.stamp <= self.camera_timeout):
            return self.abort('CAMERA_TIMEOUT')
        if self.started is None:
            return (0, 0)
        if self.stop_candidate_since is not None:
            if now-self.stop_candidate_since >= self.end_verify_seconds:
                return self.abort('END_UNCONFIRMED_STOPPED')
            self.reason = 'STOP_VERIFY_END'
            return (0, 0)
        if now-self.started >= self.max_seconds:
            return self.abort('MAX_RUN_TIMEOUT')
        if not self.seen and now-self.started >= self.seek_seconds:
            return self.abort('NO_LINES_SEEN')
        self.reason = 'FORWARD_SEEN_LINES' if self.seen else 'FORWARD_SEEK_LINES'
        if self.seen and self.line_count == 0:
            self.reason = 'FORWARD_WAIT_BOTH_CURVES'
        return (self.speed, self.steering)

class ParkingApproachRun(StopRun):
    requires_end_confirmation = True

    def __init__(self, base, end_frames=2, end_seconds=.1):
        self.__dict__.update(base.__dict__)
        self.end_frames,self.end_seconds = end_frames,end_seconds
        self.scene = {}
        self.bay_armed = False
        self.straight_votes = 0
        self.end_votes,self.end_since = 0,None
        # Confirmation is stationary. It must never postpone the first brake.
        self.lost_frames,self.lost_seconds = end_frames,end_seconds

    def observe_scene(self, curve):
        self.scene = curve

    def observe(self, count, stamp, received, both_curved=False):
        if self.finished or (self.stamp is not None and stamp <= self.stamp):
            return
        observation = self.scene.get('lane_observation')
        geometry = near_lane_geometry(observation)
        straight = (geometry['valid'] and abs(geometry['heading_deg']) <= 10. and
                    abs(geometry['curvature']) <= .35 and abs(geometry['lateral_m']) <= .15)
        matching = observation is not None and observation['stamp'] == stamp
        self.straight_votes = self.straight_votes+1 if count > 0 and straight and matching else 0
        # Road alignment is diagnostic only. The lane controller accepts paths
        # outside that ideal geometry, including short paths through a bend.
        # Use the saved visibility history and same-frame boundary bend event.
        StopRun.observe(self,count,stamp,received,both_curved)
        self.bay_armed = self.seen
        self.end_votes,self.end_since = self.missing_count,self.missing_since

    def tick(self, now, ros_now):
        command = StopRun.tick(self,now,ros_now)
        if command[0] > 0:
            if not self.bay_armed:
                self.reason = 'FORWARD_SEEK_LINES'
        return command

class SequenceRun(object):
    """Wrap the saved visual stop; brake, settle, reverse once, then stop."""
    def __init__(self, approach, seconds=(1.5, .2, 3.4, .1, 2.1), reverse_speed=30,
                 parking_pause_seconds=.5, lane_follower=None):
        self.approach = approach
        self.lane_follower = lane_follower
        self.lane_started = False
        self.seconds = tuple(finite(v, 'step%d_seconds' % (i+1), .05, 10.)
                             for i, v in enumerate(seconds))
        if len(self.seconds) != 5:
            raise ValueError('P3 requires exactly five reverse durations')
        self.reverse_speed = finite(reverse_speed, 'reverse_speed', 1, 30, True)
        self.pause = finite(parking_pause_seconds, 'parking_pause_seconds', .3, 3.)
        self.finished, self.reason = False, 'READY'
        self.phase, self.deadline, self.index = 'APPROACH', 0., -1
        self.stages = [('P3_STEP1_REVERSE_STRAIGHT', 0),
                       ('P3_STEP2_REVERSE_LEFT', 22),
                       ('P3_STEP3_REVERSE_RIGHT', -22),
                       ('P3_STEP4_REVERSE_STRAIGHT', 0),
                       ('P3_STEP5_REVERSE_LEFT', 22)]

    def __getattr__(self, name):
        return getattr(self.approach, name)

    def start(self, now):
        self.approach.start(now)

    def observe(self, count, stamp, received, both_curved=False):
        if not self.finished and (self.phase == 'APPROACH' or
                (self.phase == 'HOLD' and getattr(self.approach,'requires_end_confirmation',False))):
            self.approach.observe(count, stamp, received, both_curved)

    def observe_lane(self, observation, now):
        if self.lane_follower is not None and not self.finished and self.phase == 'APPROACH':
            self.lane_follower.observe(observation, now)

    def observe_scene(self, curve):
        observer = getattr(self.approach, 'observe_scene', None)
        if observer is not None and self.phase in ('APPROACH','HOLD') and not self.finished:
            observer(curve)

    def diagnostics(self, now):
        diagnostic = (self.lane_follower.diagnostics(now) if self.lane_follower is not None
                      and hasattr(self.lane_follower, 'diagnostics') else {})
        diagnostic.update(bay_armed=getattr(self.approach,'bay_armed',False),
                          end_votes=getattr(self.approach,'end_votes',0))
        return diagnostic

    def abort(self, reason):
        self.finished, self.reason = True, reason
        self.phase = 'FINISHED'
        self.approach.abort(reason)
        return (0, 0)

    def tick(self, now, ros_now):
        if self.finished:
            return (0, 0)
        if self.phase == 'APPROACH':
            command = self.approach.tick(now, ros_now)
            if self.approach.finished and self.approach.reason != 'BAY_END_CONFIRMED':
                return self.abort(self.approach.reason)
            if self.approach.stop_candidate_since is None:
                self.reason = self.approach.reason
                if self.lane_follower is not None and command[0] > 0:
                    command = self.lane_follower.command(ros_now)
                    if command[0] > 0:
                        self.lane_started = True
                        self.reason = 'P3_LANE_' + self.approach.reason
                    else:
                        self.reason = 'P3_WAIT_LANE'
                return command
            if self.lane_follower is not None and not self.lane_started:
                return self.abort('NO_LANE_BEFORE_PARKING')
            # Brake on the first joint visual event. Validate while stationary;
            # an uncertain end must not drive onward or begin timed reversing.
            self.phase, self.reason = 'HOLD', 'P3_STOP_HOLD'
            self.deadline = now+self.pause
            return (0, 0)
        if self.phase == 'HOLD' and getattr(self.approach,'requires_end_confirmation',False):
            self.approach.tick(now,ros_now)
            if self.approach.finished and self.approach.reason != 'BAY_END_CONFIRMED':
                return self.abort(self.approach.reason)
            if not self.approach.finished:
                self.reason = 'P3_STOP_VERIFY_END'
                return (0,0)
            self.reason = 'P3_STOP_HOLD'
        if now < self.deadline:
            if self.phase == 'HOLD':
                return (0, 0)
            return self.motion_commands[self.index]
        # Start the next stage at the outgoing-command time. Late polling
        # must not silently skip a stage or shorten its configured duration.
        self.index += 1
        if self.index >= len(self.stages):
            return self.abort('P3_COMPLETE')
        self.phase = 'REVERSE'
        self.reason = self.stages[self.index][0]
        self.deadline = now+self.seconds[self.index]
        return self.motion_commands[self.index]



class LaneAdapter(object):
    def __init__(self,cfg,speed):
        # The saved tests reuse this exact production controller with a local speed override.
        from robot.parallel_parking.lane_follow import LaneFollower
        self.follower = LaneFollower(cfg,speed)
        self.stamp = None
    def observe(self,data,now):
        if self.stamp is not None and data['stamp'] <= self.stamp:
            return
        if not 0 <= now-data['stamp'] <= self.follower.cfg['sensor_timeout']:
            return
        self.follower.observe(json.dumps(data,allow_nan=False),now)
        self.stamp = data['stamp']
    def command(self,now):
        command = self.follower.command(now)
        return command if command[0] > 0 else (0,0)
    def diagnostics(self,now):
        return dict(lane_age_s=now-self.stamp if self.stamp is not None else -1.,
                    lane_reason=(self.follower.core.lane_target or {}).get('reason','no_accepted_path'))

def validate_settings(cfg):
    options = cfg.get('parking_timed')
    if not isinstance(options,dict):
        raise ValueError('timed_sequence requires parking_timed settings')
    if cfg.get('parking_slot') not in ('P1','P2','P3'):
        raise ValueError('timed_sequence requires an explicit P1/P2/P3 parking_slot')
    if cfg['speed_raw_limit'] < 30 or cfg['steering_raw_limit'] != 22:
        raise ValueError('saved parking requires raw speed capacity 30 and steering limit 22')
    bounds = (('approach_speed_raw',1,30,True),('seen_frames',1,20,True),
              ('seek_seconds',.5,15.,False),('max_seconds',1.,60.,False),
              ('end_zero_frames',2,6,True),
              ('end_zero_seconds',.05,.5,False),('parking_pause_seconds',.3,3.,False),
              ('end_verify_seconds',.5,5.,False),
              ('min_line_m',.04,.30,False),('acquire_line_m',.10,.50,False),
              ('angle_deg',5.,40.,False),('white_v_min',100,250,True),
              ('curve_min_curvature',.1,3.,False),('curve_min_turn_deg',3.,45.,False))
    for key,low,high,integer in bounds:
        finite(options.get(key),key,low,high,integer)
    if options.get('side') not in ('left','right'):
        raise ValueError('invalid timed parking side')
    if options['max_seconds'] < options['seek_seconds'] or options['end_verify_seconds'] <= options['end_zero_seconds']:
        raise ValueError('invalid timed parking deadlines')
    if options['acquire_line_m'] < options['min_line_m']:
        raise ValueError('invalid timed parking line lengths')
    profiles = options.get('sequences',{})
    if not isinstance(profiles,dict) or set(profiles) != set(('P1','P2','P3')):
        raise ValueError('timed parking requires all three explicit profiles')
    for slot,stages in profiles.items():
        if not isinstance(stages,list) or len(stages) != 5:
            raise ValueError(slot+' requires exactly five stages')
        for stage in stages:
            if not isinstance(stage,dict) or set(stage) != set(('speed','steering','seconds')):
                raise ValueError('invalid parking stage fields')
            speed = finite(stage['speed'],'stage speed',-30,30,True)
            if speed == 0:
                raise ValueError('timed parking stage speed cannot be zero')
            finite(stage['steering'],'stage steering',-22,22,True)
            finite(stage['seconds'],'stage seconds',.05,10.,False)
    return options

def validate_scene(data,cfg,now):
    if not isinstance(data,dict) or data.get('source') != 'timed_parking_front' or data.get('frame') != 'base_link':
        raise ValueError('invalid timed parking scene source/frame')
    stamp = number(data.get('stamp'))
    if not 0 <= now-stamp <= cfg.get('ground_timeout',1.25):
        raise ValueError('timed parking scene stale/future')
    count = number(data.get('lines'))
    if count != int(count) or not 0 <= count <= 128:
        raise ValueError('invalid bay line count')
    boolean(data.get('both_curved'))
    lane = data.get('lane_observation')
    if not isinstance(lane,dict) or lane.get('frame') != cfg['lane_frame'] or number(lane.get('stamp')) != stamp:
        raise ValueError('parking evidence must use the SAME captured lane/image frame')
    confidence = number(lane.get('confidence'))
    if not 0 <= confidence <= 1:
        raise ValueError('invalid paired lane confidence')
    points = lane.get('points')
    if not isinstance(points,list) or len(points) > 1000:
        raise ValueError('invalid paired lane points')
    previous = 0.
    for point in points:
        if not isinstance(point,(list,tuple)) or len(point) != 2:
            raise ValueError('invalid paired lane point')
        x,y = map(number,point)
        if not previous < x <= 8 or abs(y) > 8:
            raise ValueError('unordered/unbounded paired lane')
        previous = x
    return stamp

class TimedParking(SequenceRun):
    def __init__(self,cfg,now,lane_follower=None):
        options = validate_settings(cfg)
        self.cfg,self.slot,self.triggered_at = cfg,cfg['parking_slot'],now
        self.visual_stamp = None
        self.last_lines,self.last_both_curved = None,False
        base = StopRun(speed=options['approach_speed_raw'],steering=0,
                       seen_frames=options['seen_frames'],seek_seconds=options['seek_seconds'],
                       max_seconds=options['max_seconds'],camera_timeout=cfg.get('ground_timeout',1.25),
                       end_verify_seconds=options['end_verify_seconds'])
        approach = ParkingApproachRun(base,options['end_zero_frames'],options['end_zero_seconds'])
        stages = options['sequences'][self.slot]
        follower = lane_follower or LaneAdapter(cfg,options['approach_speed_raw'])
        SequenceRun.__init__(self,approach,tuple(s['seconds'] for s in stages),
                             30,options['parking_pause_seconds'],follower)
        self.motion_commands = [(s['speed'],s['steering']) for s in stages]
        self.stages = [(self.slot+'_STEP%d_' % (i+1)+
                        ('FORWARD' if s['speed']>0 else 'REVERSE')+
                        ('_STRAIGHT' if s['steering']==0 else '_LEFT' if s['steering']>0 else '_RIGHT'),
                        s['steering']) for i,s in enumerate(stages)]
    def abort(self,reason):
        if reason == 'P3_COMPLETE':
            reason = self.slot+'_COMPLETE'
        return SequenceRun.abort(self,reason)
    def close(self):
        if isinstance(self.lane_follower,LaneAdapter):
            self.lane_follower.follower.core.close()
    def observe_visual(self,data,now):
        stamp = validate_scene(data,self.cfg,now)
        if self.finished or stamp < self.triggered_at or (self.visual_stamp is not None and stamp <= self.visual_stamp):
            return
        self.visual_stamp = stamp
        self.last_lines,self.last_both_curved = int(data['lines']),data['both_curved']
        self.observe_lane(data['lane_observation'],now)
        if self.started is None:
            self.start(now)
        self.observe_scene(data)
        self.observe(self.last_lines,stamp,now,self.last_both_curved)
    def command(self,now):
        if self.finished:
            return (0,0.)
        if self.visual_stamp is None:
            self.reason = self.slot+'_WAIT_CAMERA'
            return (0,0.)
        if not 0 <= now-self.visual_stamp <= self.cfg.get('ground_timeout',1.25):
            self.abort('PARKING_CAMERA_TIMEOUT')
            return (0,0.)
        raw_speed,raw_steer = self.tick(now,now)
        if self.reason.startswith('P3_'):
            self.reason = self.slot+'_'+self.reason[3:]
        # Invert the normal encoder so the tested raw commands survive unchanged.
        scale = self.cfg.get('steering_command_scale_rad',self.cfg['max_steer'])
        return (raw_speed/self.cfg['speed_sign'],
                raw_steer/self.cfg['steering_sign']/self.cfg['steering_raw_limit']*scale)
    def status(self,now):
        return dict(slot=self.slot,phase=self.phase,reason=self.reason,
                    step=max(0,self.index+1),lines=self.last_lines,both_curved=self.last_both_curved,
                    bay_armed=self.approach.bay_armed,end_votes=self.approach.end_votes,
                    camera_age_s=now-self.visual_stamp if self.visual_stamp is not None else None,
                    remaining_s=max(0.,self.deadline-now),finished=self.finished,
                    sequence=self.cfg['parking_timed']['sequences'][self.slot])
