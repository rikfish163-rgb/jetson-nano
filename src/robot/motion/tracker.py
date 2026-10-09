"""tracking: independently testable geometry; no mission state or ROS."""
from __future__ import division
import math
from robot.common.geometry import distance
from robot.common.geometry import local
from robot.common.geometry import wrap


class Follower:
    def __init__(self, path, cfg, parking=False, full_lock=False):
        self.path, self.cfg, self.parking = path, cfg, parking
        self.full_lock=full_lock
        # A perpendicular reverse arc needs its setup point closely reached;
        # a parallel Hybrid-A* path retains its original lattice tolerance.
        perpendicular = math.radians(70) <= abs(wrap(path[-1][2]-path[0][2])) <= math.radians(110)
        self.goal_tolerance = (min(cfg['path_goal_tolerance'],cfg.get('parking_goal_tolerance',.01))
                               if parking and perpendicular else cfg['path_goal_tolerance'])
        self.index = 0
        self.segment_end = self._segment_end(0)
        self.pause_until = None
        self.done = False

    def _segment_end(self, start):
        d, i = self.path[start][3], start
        while i+1 < len(self.path) and self.path[i+1][3] == d:
            i += 1
        return i

    def command(self, pose, now):
        if self.done:
            return 0, 0.0
        cfg, end = self.cfg, self.path[self.segment_end]
        close = distance(pose, end) <= self.goal_tolerance
        aligned = abs(wrap(pose[2]-end[2])) <= cfg['path_yaw_tolerance']
        if close and aligned:
            if self.segment_end == len(self.path)-1:
                self.done = True
                return 0, 0.0
            if self.pause_until is None:
                self.pause_until = now+cfg['cusp_pause']
            if now < self.pause_until:
                return 0, 0.0
            self.index = self.segment_end+1
            self.segment_end = self._segment_end(self.index)
            self.pause_until = None
        # Only a small forward window; never jump across a crossing/cusp.
        end_index = min(self.segment_end, self.index+20)
        self.index = min(range(self.index, end_index+1), key=lambda i: distance(pose,self.path[i]))
        if self.full_lock:
            # Each point stores the steering for the step arriving at it.
            # Follow entry / full-lock arc / straight exit without preview
            # blending the arc with straight points and reducing its lock.
            target=self.path[min(self.index+1,self.segment_end)]
            return cfg['speed_raw']['action'],target[4]
        ld = cfg['parking_lookahead'] if self.parking else cfg.get('action_lookahead',cfg['lookahead'])
        j, travelled = self.index, 0.0
        while j < self.segment_end and travelled < ld:
            travelled += distance(self.path[j], self.path[j+1]); j += 1
        x,y = local(pose, self.path[j])
        steer = math.atan2(2*cfg['wheelbase']*y, max(0.0025,x*x+y*y))
        steer = max(-cfg['max_steer'], min(cfg['max_steer'], steer))
        raw = cfg['speed_raw']['parking' if self.parking else 'action']
        return self.path[self.index][3]*raw, steer
