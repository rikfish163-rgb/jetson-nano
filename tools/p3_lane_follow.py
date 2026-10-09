"""P3-test-only adapter: original road vision and controller, no extra ROS nodes."""
from __future__ import division
import json
import os
import math
import numpy as np
from parking_lane_curve import LaneCurveVision, both_boundary_curves
from robot.common.config import load_config
from robot.parallel_parking.lane_follow import LaneFollower


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


class P3LaneVision(LaneCurveVision):
    def observe(self, frame, stamp):
        lane = self.lane
        mask = lane.make_metric_bev(lane.detect_white_line(frame))
        observation = dict(stamp=stamp, frame=lane.LANE_PATH_FRAME_ID,
                           confidence=0., points=[])
        if mask is None:
            result = both_boundary_curves({})
            result['lane_observation'] = observation
            return result, None
        tracking = lane.track_metric_lane(mask, source_stamp=stamp)
        boundaries = lane.build_observed_lane_boundaries(tracking, stamp)
        # Use the exact path selector/conversion used by the production camera.
        # Construct plain data without publishing any production ROS topics.
        selected = lane.select_nearest_lane_path_segment(tracking['center_segments'])
        observation.update(
            confidence=max(0., min(1., float(tracking['tracking_confidence']))),
            points=[list(lane.bev_point_to_vehicle_m(p['x'], p['y']))
                    for p in selected])
        result = both_boundary_curves(boundaries, self.min_curvature, self.min_turn_deg)
        result.update(boundaries=boundaries, lane_observation=observation)
        return result, lane.make_lane_tracking_debug(mask, tracking)


class P3LaneFollower(object):
    def __init__(self, root, speed=30):
        # LaneFollower deep-copies this config. Overrides stay in this process.
        self.follower = LaneFollower(load_config(os.path.join(root, 'src/robot/config')), speed)
        self.stamp = None
        self.last_observation = None

    def observe(self, observation, now):
        if observation is None:
            return
        stamp = observation['stamp']
        if self.stamp is not None and stamp <= self.stamp:
            return
        self.follower.observe(json.dumps(observation, allow_nan=False), now)
        self.stamp = stamp
        self.last_observation = observation

    def diagnostics(self, now):
        core = self.follower.core
        age = now-self.stamp if self.stamp is not None else -1.
        if self.stamp is None:
            reason = 'no_lane_received'
        elif age > self.follower.cfg['sensor_timeout']:
            reason = 'lane_stream_stale'
        else:
            target = core.lane_target or {}
            reason = target.get('reason', 'no_accepted_path')
            if not core.lane:
                reason = 'empty_lane_path'
        return dict(lane_age_s=age, lane_points=len(core.lane),
                    lane_confidence=core.lane_confidence, lane_reason=reason)

    def command(self, now):
        speed, steering = self.follower.command(now)
        if speed <= 0:
            return (0, 0)
        return (speed, steering)
