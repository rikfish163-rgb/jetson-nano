"""One-shot full-left steering probe, reusing the competition lane controller."""
from robot.master.controller import Controller
from robot.common.contracts import number


class LeftLockController(Controller):
    def __init__(self,cfg,speed_raw=18,max_seconds=12,line_frames=3):
        speed_raw,max_seconds,line_frames = map(number,(speed_raw,max_seconds,line_frames))
        if not 0 < speed_raw <= min(30,cfg['speed_raw_limit']) or speed_raw != int(speed_raw):
            raise ValueError('left lock speed must be an integer in [1,30] within raw limit')
        if not .5 <= max_seconds <= 30 or not 1 <= line_frames <= 10 or line_frames != int(line_frames):
            raise ValueError('left lock requires seconds [0.5,30] and frames [1,10]')
        cfg = dict(cfg,wait_green=False,exit_frames=int(line_frames),bypass_enabled=False)
        Controller.__init__(self,cfg)
        self.state,self.action = 'LEFT_LOCK','LEFT'
        self.lock_speed,self.lock_timeout = int(speed_raw),max_seconds
        self.lock_started = None

    def observe_sign(self,label,confidence,stamp,now):
        # Only traffic-light stop/release is relevant to this isolated probe.
        if label in ('RED','GREEN'):
            Controller.observe_sign(self,label,confidence,stamp,now)

    def observe_ground(self,data,stamp):
        return None

    def tick(self,now):
        if self.state != 'LEFT_LOCK':
            return Controller.tick(self,now)
        self.obstacle_check = dict(kind='not_checked')
        if self.estop:
            return self.stop('emergency_stop')
        if self.lock_started is not None and now-self.lock_started >= self.lock_timeout:
            self.state = 'FAULT'
            return self.stop('left_lock_timeout')
        if self.red:
            return self.stop('red_latched')
        if self.cfg['pose_mode'] == 'odom' and not 0 <= now-self.pose_stamp <= self.cfg['odom_timeout']:
            return self.stop('odom_stale')
        if (self.scan is None or not 0 <= now-self.scan.stamp <= self.cfg['sensor_timeout'] or
                self.scan.valid_rays < self.cfg['lidar']['min_rays']):
            return self.stop('scan_missing_or_stale')
        if self.lock_started is None:
            self.lock_started,self.action_started = now,now
        # Deliberately no planned-90-degree gate in this test. Reuse the forward
        # tangent, lateral offset, freshness and distinct-frame lane checks.
        self.exit_pose = self.pose
        if self.exit_lane_confirmed(now):
            self.resume_lane()
            return self.checked_command(self.lane_command(now),now,False)
        return self.checked_command((self.lock_speed,self.cfg['max_steer']),now,False)
