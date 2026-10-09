"""Independent, scan-relative left pass controller. Coordinates: x forward, y left."""
from __future__ import division
import math


def clusters(ranges, angle_min, angle_increment, range_min, range_max,
             min_points=4, gap=0.07):
    groups, current = [], []
    for i, distance in enumerate(ranges):
        valid = (not math.isnan(distance) and not math.isinf(distance) and
                 range_min <= distance <= min(range_max, 3.0))
        if valid:
            angle = angle_min + i * angle_increment
            point = (distance * math.cos(angle), distance * math.sin(angle))
            if current and math.hypot(point[0]-current[-1][0], point[1]-current[-1][1]) > gap:
                groups.append(current)
                current = []
            current.append(point)
        elif current:
            groups.append(current)
            current = []
    if current:
        groups.append(current)
    result = []
    for points in groups:
        if len(points) < min_points:
            continue
        x = sum(p[0] for p in points)/len(points)
        y = sum(p[1] for p in points)/len(points)
        radius = max(math.hypot(p[0]-x, p[1]-y) for p in points)
        if .015 <= radius <= .28:
            result.append(dict(x=x, y=y, radius=radius, points=points))
    return result


class LeftPass(object):
    """Uses each new scan for both progress and obstacle clearance; no timed turn."""
    def __init__(self, clearance=.20, body_half=.12, wheelbase=.26,
                 front=.33, rear=.07, speed_raw=8, max_steer_raw=12,
                 max_steer_rad=.46275):
        if not .10 <= clearance <= .60:
            raise ValueError('clearance must be between 0.10 and 0.60 m')
        if not 1 <= speed_raw <= 15 or not 1 <= max_steer_raw <= 22:
            raise ValueError('speed/steering outside bench limits')
        self.clearance, self.body_half = clearance, body_half
        self.wheelbase, self.front, self.rear = wheelbase, front, rear
        self.speed_raw, self.max_steer_raw = speed_raw, max_steer_raw
        self.max_steer_rad = max_steer_rad
        self.phase = 'SEARCH'
        self.target = None
        self.last_reason = 'waiting_for_target'

    def _pick(self, detected):
        if self.target is None:
            candidates = [c for c in detected if .45 <= c['x'] <= 1.6 and
                          abs(c['y']) <= self.body_half+.20]
            return min(candidates, key=lambda c: c['x']) if candidates else None
        candidates = [c for c in detected if
                      math.hypot(c['x']-self.target['x'], c['y']-self.target['y']) < .38]
        return min(candidates, key=lambda c:
                   math.hypot(c['x']-self.target['x'], c['y']-self.target['y'])) if candidates else None

    def step(self, detected, all_points):
        if self.phase == 'DONE':
            return 0, 0, 'passed_target_stop'
        target = self._pick(detected)
        if target is None:
            self.last_reason = 'target_missing_stop' if self.target else 'waiting_for_target'
            return 0, 0, self.last_reason
        self.target = target
        offset = self.body_half + self.clearance + target['radius']
        if target['x'] < -self.rear-target['radius']-.10:
            self.phase = 'DONE'
            self.last_reason = 'passed_target_stop'
            return 0, 0, self.last_reason
        self.phase = 'PASS'
        # Drive to a point left of the measured obstacle; recalculate on every scan.
        alongside = target['x'] <= self.front+target['radius']
        look_x = (.60 if alongside else max(.32, min(.9, target['x']+.12)))
        look_y = max(-.35, min(.65, target['y']+offset))
        curvature = 2.0*look_y/(look_x*look_x+look_y*look_y)
        angle = math.atan(self.wheelbase*curvature)
        steer = int(round(max(-self.max_steer_raw, min(self.max_steer_raw,
                          angle/self.max_steer_rad*22.0))))
        if alongside:
            steer = max(-4, steer)
        # Project the full body along the next short segment. Every scan point
        # remains in the guard, including the selected target.
        steer_angle = steer/22.0*self.max_steer_rad
        k = math.tan(steer_angle)/self.wheelbase
        for distance in (.05, .10, .15, .20):
            if abs(k) < 1e-8:
                px, py, yaw = distance, 0., 0.
            else:
                yaw = distance*k
                px, py = math.sin(yaw)/k, (1-math.cos(yaw))/k
            co, si = math.cos(yaw), math.sin(yaw)
            for x, y in all_points:
                dx, dy = x-px, y-py
                local_x, local_y = co*dx+si*dy, -si*dx+co*dy
                if -self.rear-.03 <= local_x <= self.front+.03 and abs(local_y) <= self.body_half+.03:
                    self.last_reason = 'predicted_collision_stop'
                    return 0, 0, self.last_reason
        # A target already abreast must remain on the right with measured gap.
        if target['x'] <= self.front+target['radius'] and target['x'] >= -self.rear-target['radius']:
            side_gap = -target['y']-target['radius']-self.body_half
            if side_gap < self.clearance-.03:
                self.last_reason = 'side_clearance_stop'
                return 0, 0, self.last_reason
        self.last_reason = 'tracking_left_pass'
        return self.speed_raw, steer, self.last_reason
