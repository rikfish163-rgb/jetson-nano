"""Opt-in front-sector, two-segment obstacle trial (no ROS dependencies)."""
import math


class TimedObstacle(object):
    def __init__(self, half_angle=15.0, distance=0.5, center=0.0, speed=26, turn_s=5.0):
        values = (half_angle, distance, center, speed, turn_s)
        if any(math.isnan(float(v)) or math.isinf(float(v)) for v in values):
            raise ValueError('obstacle parameters must be finite')
        if not (0 < half_angle <= 90 and distance > 0 and 0 < speed <= 100 and turn_s > 0):
            raise ValueError('invalid obstacle sector, distance or speed')
        self.half_angle = math.radians(half_angle)
        self.center = math.radians(center)
        self.distance = distance
        self.speed = int(speed)
        self.turn_s = float(turn_s)
        self.scan = None
        self.started = None
        self.armed = True
        self.fault = False

    def update(self, ranges, angle_min, increment, range_min, range_max, stamp):
        valid = False
        hit = False
        for i, radius in enumerate(ranges):
            angle = angle_min + i * increment - self.center
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) > self.half_angle:
                continue
            # This driver uses inf for absent returns; require at least one
            # finite valid sector return before granting motion.
            if math.isnan(radius) or math.isinf(radius) or not range_min <= radius <= range_max or radius <= 0:
                continue
            valid = True
            hit = hit or radius <= self.distance
        self.scan = (stamp, valid, hit)

    def command(self, now):
        stop = dict(speed_raw=0, steering_raw=0)
        if self.fault:
            return stop
        if self.scan is None or not 0 <= now - self.scan[0] <= .5 or not self.scan[1]:
            if self.started is not None:
                self.fault = True
            return stop
        hit = self.scan[2]
        if self.started is not None:
            elapsed = now - self.started
            if elapsed < 2 * self.turn_s:
                return dict(speed_raw=self.speed, steering_raw=22 if elapsed < self.turn_s else -22)
            self.started = None
            return None
        if not hit:
            self.armed = True
        if hit and self.armed:
            self.armed = False
            self.started = now
            return dict(speed_raw=self.speed, steering_raw=22)
        return None
