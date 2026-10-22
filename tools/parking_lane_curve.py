"""Measured left/right road-boundary bend guard for the parking-only test."""
from __future__ import division
import os
import imp
from robot.parking.timed_vision import boundary_shape, both_boundary_curves


class LaneCurveVision(object):
    def __init__(self, root, min_curvature=.45, min_turn_deg=10.):
        # Load the existing detector functions only. Its main() and all ROS
        # publishers/vehicle-control nodes are deliberately not started here.
        path = os.path.join(root, 'src/ros/camera/scripts/camera_yihan_web.py')
        self.lane = imp.load_source('parking_stop_boundary_detector', path)
        self.lane.configure_lane_windows(20, .15)
        self.lane.init_undistort_maps()
        self.min_curvature, self.min_turn_deg = min_curvature, min_turn_deg

    def observe(self, frame, stamp):
        lane = self.lane
        mask = lane.make_metric_bev(lane.detect_white_line(frame))
        if mask is None:
            return both_boundary_curves({}), None
        tracking = lane.track_metric_lane(mask, source_stamp=stamp)
        boundaries = lane.build_observed_lane_boundaries(tracking, stamp)
        result = both_boundary_curves(boundaries, self.min_curvature, self.min_turn_deg)
        result['boundaries'] = boundaries
        debug = lane.make_lane_tracking_debug(mask, tracking)
        return result, debug
