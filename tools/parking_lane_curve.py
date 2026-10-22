"""Measured left/right road-boundary bend guard for the parking-only test."""
from __future__ import division
import math
import os
import imp
import numpy as np


def boundary_shape(points, min_curvature=.45, min_turn_deg=10.):
    result = dict(kind='UNKNOWN', curvature=None, turn_deg=None, span_m=0.)
    xy = np.asarray(points, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or not np.all(np.isfinite(xy)):
        return result
    xy = xy[(xy[:, 0] >= .45) & (xy[:, 0] <= 1.5)]
    xy = xy[np.argsort(xy[:, 0])]
    if len(xy) < 5 or len(set(xy[:, 0])) != len(xy):
        return result
    span = float(xy[-1, 0]-xy[0, 0])
    result['span_m'] = span
    if span < .35:
        return result
    center = float(np.mean(xy[:, 0]))
    x = xy[:, 0]-center
    a, b, c = np.polyfit(x, xy[:, 1], 2)
    fit = a*x*x+b*x+c
    residual = float(np.sqrt(np.mean((xy[:, 1]-fit)**2)))
    result['residual_m'] = residual
    if residual > .02 or np.max(abs(xy[:, 1]-fit)) > .045:
        return result
    curvature = float(2*a/(1+b*b)**1.5)
    turn = math.degrees(math.atan(2*a*x[-1]+b)-math.atan(2*a*x[0]+b))
    sag = abs(float(a))*span*span/4.
    curved = abs(curvature) >= min_curvature and abs(turn) >= min_turn_deg and sag >= .012
    result.update(kind='CURVE' if curved else 'STRAIGHT', curvature=curvature,
                  turn_deg=turn, sag_m=sag, coefficients=[float(a), float(b), float(c)],
                  center_x=center, min_x=float(xy[0, 0]), max_x=float(xy[-1, 0]))
    return result


def both_boundary_curves(boundaries, min_curvature=.45, min_turn_deg=10.):
    left = boundary_shape(boundaries.get('LEFT', []), min_curvature, min_turn_deg)
    right = boundary_shape(boundaries.get('RIGHT', []), min_curvature, min_turn_deg)
    result = dict(left=left, right=right, both_curved=False, reason='need_two_curves')
    if left['kind'] != 'CURVE' or right['kind'] != 'CURVE':
        return result
    if left['curvature']*right['curvature'] <= 0:
        result['reason'] = 'opposite_bend_directions'
        return result
    lo, hi = max(left['min_x'], right['min_x']), min(left['max_x'], right['max_x'])
    if hi-lo < .30:
        result['reason'] = 'insufficient_shared_span'
        return result
    xs = np.linspace(lo, hi, 9)
    def at(row):
        a, b, c = row['coefficients']
        x = xs-row['center_x']
        return a*x*x+b*x+c
    width = at(left)-at(right)
    if np.any(width < .35) or np.any(width > 1.05):
        result['reason'] = 'boundary_pair_width_invalid'
        return result
    result.update(both_curved=True, reason='both_boundaries_curve_same_way')
    return result


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
