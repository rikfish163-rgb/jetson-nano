"""Measured-path curvature feedforward with a scalar, frame-based Kalman filter."""
from __future__ import division
import math
import numpy as np
from robot.common.geometry import local


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
    forward = sorted(p for p in points if p[0] > .05)
    if not forward:
        return steer
    scale = ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    x,y = forward[-1]
    # The arc through the measured far center already includes the bend.
    # Adding its lateral displacement again as a proportional error caused
    # full-lock saturation even when the vehicle was centered on that arc.
    curvature = 2*y/(x*x+y*y)
    desired_curvature = curvature
    if ctx.lane_preview is not None:
        # At curve entry the far center can still be almost straight ahead.
        # Blend the measured curve shape, rather than adding a second angle.
        desired_curvature = .5*(curvature+ctx.lane_preview['curvature'])
    physical = math.atan(ctx.cfg['wheelbase']*desired_curvature)
    near_x,near_y = forward[0]
    if abs(curvature*near_x) < 1:
        expected_y = curvature*near_x*near_x/(1+math.sqrt(1-(curvature*near_x)**2))
        # Correct proximity to the center relative to the same arc, not to a
        # straight line. A centered model circle therefore has zero error.
        error = near_y-expected_y
        if abs(near_y) >= .04 and near_y*error > 0:
            fraction = max(-1.,min(1.,error/ctx.cfg.get('lane_lateral_full_scale_m',.10)))
            physical += ctx.cfg['max_steer']*fraction
    physical = max(-ctx.cfg['max_steer'], min(ctx.cfg['max_steer'], physical))
    return physical/ctx.cfg['max_steer']*scale


def lane_speed(ctx, speed):
    # A brief gap must not accelerate above the configured lane speed.
    speed = min(speed, ctx.cfg['speed_raw']['lane'])
    if not ctx.cfg.get('lane_curvature_preview',False):
        return speed
    slow = min(speed, ctx.cfg.get('lane_curve_speed_raw',12))
    points = sorted(local(ctx.pose,p) for p in ctx.lane)
    points = [p for p in points if p[0] > .05]
    preview = ctx.lane_preview
    # A failed fit, an off-axis exit or a gap cannot establish a straight.
    straight = (ctx.state == 'LANE' and ctx.lane_confidence >= .6 and
        preview is not None and len(points) >= 4 and
        points[-1][0]-points[0][0] >= .30 and
        abs(preview['curvature']) < .25 and abs(preview['measured_curvature']) < .25 and
        abs(points[0][1]) <= .06 and abs(points[-1][1]) <= .10 and
        abs(ctx.gap_steer) <= .15*ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer']) and
        all(abs(math.atan2(b[1]-a[1],b[0]-a[0])) <= math.radians(8)
            for a,b in zip(points,points[1:])))
    if not straight:
        ctx.lane_speed_state = None
        return slow
    evidence = ctx.lane_speed_state
    if evidence is None or not 0 <= ctx.lane_stamp-evidence['stamp'] <= ctx.cfg['sensor_timeout']:
        evidence = dict(stamp=-1.,since=ctx.lane_stamp,frames=0,speed=slow)
        ctx.lane_speed_state = evidence
    if ctx.lane_stamp > evidence['stamp']:
        evidence['stamp'] = ctx.lane_stamp
        evidence['frames'] += 1
        if evidence['frames'] >= 5 and ctx.lane_stamp-evidence['since'] >= .6-1e-9:
            evidence['speed'] = min(speed,evidence['speed']+2)
    return min(speed,evidence['speed'])
