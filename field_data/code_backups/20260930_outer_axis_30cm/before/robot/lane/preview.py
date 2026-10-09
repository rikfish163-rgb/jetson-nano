"""Measured-path curvature feedforward with a scalar, frame-based Kalman filter."""
from __future__ import division
import math
import numpy as np
from robot.common.geometry import local, world


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
    xs = np.linspace(points[0][0],points[-1][0],64)
    reference = [(float(v),float(a*(v-middle)**2+b*(v-middle)+c)) for v in xs]
    model, radius, circle_center = 'quadratic', None, None
    # Fit an actual circular arc when supported by the paint. A quadratic's
    # curvature evaluated at its midpoint varies with slope even on a circle.
    origin = np.mean(xy,axis=0)
    q = xy-origin
    matrix = np.column_stack((2*q[:,0],2*q[:,1],np.ones(len(q))))
    fit,unused,rank,unused_s = np.linalg.lstsq(matrix,np.sum(q*q,axis=1),rcond=-1)
    squared = float(fit[2]+np.dot(fit[:2],fit[:2]))
    if rank == 3 and squared > 0:
        candidate_radius = math.sqrt(squared)
        center = origin+fit[:2]
        arc_residual = float(np.sqrt(np.mean((np.sqrt(np.sum((xy-center)**2,axis=1))-candidate_radius)**2)))
        if (.30 <= candidate_radius <= 6. and arc_residual <= .006 and
                np.all(np.abs(xs-center[0]) < candidate_radius)):
            direction = 1. if center[1] > origin[1] else -1.
            measured = direction/candidate_radius
            radius,model = candidate_radius,'circle'
            circle_center=[float(v) for v in center]
            reference = [(float(v),float(center[1]-direction*math.sqrt(
                squared-(v-center[0])**2))) for v in xs]
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
                measured_curvature=measured, residual_m=residual,
                reference_curve=reference,model=model,radius_m=radius,circle_center=circle_center)


def preview_steering(ctx, points, steer):
    if not ctx.cfg.get('lane_curvature_preview', False) or ctx.action is not None:
        ctx.lane_preview = None
        return steer
    previous=ctx.lane_preview
    pose=getattr(ctx,'pose',(0.,0.,0.))
    ctx.lane_preview = curvature_preview(points, ctx.lane_stamp, previous,
                                         ctx.cfg['sensor_timeout'])
    if ctx.lane_preview is not None and ctx.lane_preview is not previous:
        ctx.lane_preview['reference_pose']=tuple(pose)
    forward = sorted(p for p in points if p[0] > .05)
    if not forward:
        return steer
    scale = ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    x,y = forward[-1]
    if ctx.lane_preview is not None:
        # Target the fitted curve rather than a noisy last paint sample.
        x,y = local(pose,world(ctx.lane_preview.get('reference_pose',pose),
                              ctx.lane_preview['reference_curve'][-1]))
    # The arc through the measured far center already includes the bend.
    # Adding its lateral displacement again as a proportional error caused
    # full-lock saturation even when the vehicle was centered on that arc.
    curvature = 2*y/(x*x+y*y)
    desired_curvature = curvature
    turn_progress = 0.
    if ctx.lane_preview is not None:
        shape = ctx.lane_preview['curvature']
        # Once the measured center continues into the bend, its fitted
        # curvature owns feedforward. A far exit point must not halve it.
        turn_progress = max(0.,min(1.,shape*y/.02)) if abs(shape) >= .35 else 0.
        shape_weight = .5+.5*turn_progress
        desired_curvature = shape_weight*shape+(1-shape_weight)*curvature
    physical = math.atan(ctx.cfg['wheelbase']*desired_curvature)
    if ctx.lane_preview is not None and ctx.lane_preview['model']=='circle':
        fit=ctx.lane_preview
        cx,cy=local(pose,world(fit.get('reference_pose',pose),fit['circle_center']))
        center_distance=math.hypot(cx,cy)
        if center_distance > .05:
            direction=1. if fit['measured_curvature']>0 else -1.
            # True signed error to the center circle. Arc displacement at a
            # near point mixes heading with lateral error and can suppress
            # precisely the inward drift correction we need.
            error=direction*(center_distance-fit['radius_m'])
            heading=math.atan2(-direction*cx,direction*cy)
            lookahead=max(2*ctx.cfg['wheelbase'],
                          min(ctx.cfg.get('lookahead',.8),forward[0][0]))
            feedback=2*error/(lookahead*lookahead)+2*math.sin(heading)/lookahead
            # A supported circle supplies its radius directly. Adding the
            # far-point chord again biases the angle with the same offset
            # already handled by position/heading feedback.
            physical=math.atan(ctx.cfg['wheelbase']*(fit['curvature']+feedback))
            fit.update(center_error_m=error,center_heading_error_rad=heading,
                       center_feedback_curvature=feedback)
            physical=max(-ctx.cfg['max_steer'],min(ctx.cfg['max_steer'],physical))
            return physical/ctx.cfg['max_steer']*scale
    near_x,near_y = forward[0]
    reference_curvature = ctx.lane_preview['curvature'] if ctx.lane_preview is not None else curvature
    if abs(reference_curvature*near_x) < 1:
        expected_y = reference_curvature*near_x*near_x/(1+math.sqrt(1-(reference_curvature*near_x)**2))
        # Correct proximity to the center relative to the same arc, not to a
        # straight line. A centered model circle therefore has zero error.
        error = near_y-expected_y
        if ctx.lane_preview is not None:
            # An inward error can have the opposite sign to near_y while all
            # centers remain on the bend side. Do not discard that feedback
            # or cap it to 20% of the bend angle. Bound the correction range
            # below by two wheelbases to avoid excessive near-point gain.
            lookahead=max(2*ctx.cfg['wheelbase'],
                          min(ctx.cfg.get('lookahead',.8),forward[0][0]))
            correction_curvature=2*error/(lookahead*lookahead)
            physical=math.atan(ctx.cfg['wheelbase']*(desired_curvature+correction_curvature))
            ctx.lane_preview.update(center_error_m=error,center_feedback_curvature=correction_curvature)
        elif abs(near_y) >= .04 and near_y*error > 0:
            # Convert the measured offset to a near-target arc angle. The
            # previous full-lock-per-10cm gain could cancel an approaching
            # bend, or reverse it, even with only 6cm of lateral offset.
            correction_curvature = 2*error/(near_x*near_x+error*error)
            correction = 1.25*math.atan(ctx.cfg['wheelbase']*correction_curvature)
            physical += correction
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
        abs(points[0][1]) <= .04 and abs(points[-1][1]) <= .10 and
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
