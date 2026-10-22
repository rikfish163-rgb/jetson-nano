"""Measured-path curvature feedforward with a scalar, frame-based Kalman filter."""
from __future__ import division
import math
import numpy as np


def curvature_preview(points, stamp, previous, timeout):
    points = sorted(p for p in points if .05 < p[0] <= 1.5)
    if len(set(x for x,y in points)) < 4 or points[-1][0]-points[0][0] < .30:
        return None
    if previous is not None and stamp == previous['stamp']:
        return previous
    xy = np.asarray(points, dtype=float)
    if not np.all(np.isfinite(xy)):
        return None
    # Centering x avoids ill-conditioning; evaluate only inside observed span.
    middle = float(np.mean(xy[:,0]))
    x = xy[:,0]-middle
    a,b,c = np.polyfit(x, xy[:,1], 2)
    residual = float(np.sqrt(np.mean((xy[:,1]-(a*x*x+b*x+c))**2)))
    if residual > .02:
        return None
    measured = max(-3., min(3., float(2*a/(1+b*b)**1.5)))
    measurement_variance = .04+(4*residual/(points[-1][0]-points[0][0])**2)**2
    dt = stamp-previous['stamp'] if previous is not None else 0.
    if previous is None or not 0 < dt <= timeout:
        estimate, variance = measured, measurement_variance
    else:
        # Random-walk curvature: each new image adds process uncertainty.
        predicted_variance = previous['variance']+.5*dt
        gain = predicted_variance/(predicted_variance+measurement_variance)
        estimate = previous['curvature']+gain*(measured-previous['curvature'])
        variance = (1-gain)*predicted_variance
    return dict(stamp=stamp, curvature=estimate, variance=variance,
                measured_curvature=measured, residual_m=residual)


def preview_steering(ctx, points, steer):
    if not ctx.cfg.get('lane_curvature_preview', False) or ctx.action is not None:
        ctx.lane_preview = None
        return steer
    ctx.lane_preview = curvature_preview(points, ctx.lane_stamp, ctx.lane_preview,
                                         ctx.cfg['sensor_timeout'])
    if ctx.lane_preview is None:
        return steer
    scale = ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    physical = steer/scale*ctx.cfg['max_steer']
    physical += math.atan(ctx.cfg['wheelbase']*ctx.lane_preview['curvature'])
    physical = max(-ctx.cfg['max_steer'], min(ctx.cfg['max_steer'], physical))
    return physical/ctx.cfg['max_steer']*scale


def lane_speed(ctx, speed):
    if ctx.cfg.get('lane_curvature_preview',False) and (
            ctx.lane_curve_lock is not None or
            ctx.lane_preview is not None and abs(ctx.lane_preview['curvature']) >= .4):
        return min(speed, ctx.cfg.get('lane_curve_speed_raw',12))
    return speed
