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
    # The near paint determines the curve the rear axle is about to follow.
    # A far straight exit must not dilute a still-curved near segment.
    near = xy[:3]
    near_a,near_b,near_c = np.polyfit(near[:,0]-middle,near[:,1],2)
    near_slope = 2*near_a*(near[0,0]-middle)+near_b
    measured = max(-3., min(3., float(2*near_a/(1+near_slope*near_slope)**1.5)))
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


def reference_geometry(points, preview, pose):
    """Signed distance and tangent at the rear axle for one reference curve.

    A sparse or noncircular path uses its local osculating circle (or line),
    giving the same error convention and feedback as a supported full arc.
    Only current measured points determine geometry; no far-point gain switch.
    """
    if preview is not None and preview['model'] == 'circle':
        center = local(pose,world(preview.get('reference_pose',pose),preview['circle_center']))
        radius = preview['radius_m']
        curvature = preview['curvature']
        direction = 1. if preview['measured_curvature'] > 0 else -1.
    else:
        xy = np.asarray(points[:3],dtype=float)
        if len(set(x for x,y in points)) < 2:
            x,y = points[0]
            return 0., y, 0.
        middle = float(np.mean(xy[:,0]))
        # Three isolated samples cannot validate curvature against residuals;
        # use their tangent until the full fit has independent support.
        degree = 2 if preview is not None and len(xy) >= 3 else 1
        coefficients = np.polyfit(xy[:,0]-middle,xy[:,1],degree)
        # Evaluate derivatives inside the observed span, then project along
        # that local circle instead of extrapolating a quadratic to x=0.
        x = points[0][0]
        y = float(np.polyval(coefficients,x-middle))
        slope = float(np.polyval(np.polyder(coefficients),x-middle))
        second = float(np.polyval(np.polyder(coefficients,2),x-middle)) if degree==2 else 0.
        measured = max(-3.,min(3.,second/(1+slope*slope)**1.5))
        curvature = preview['curvature'] if preview is not None else measured
        if abs(measured) < .05:
            heading = math.atan(slope)
            return curvature,(y-slope*x)/math.hypot(1.,slope),heading
        radius = abs(1./measured)
        direction = 1. if measured > 0 else -1.
        center = (x-slope/math.hypot(1.,slope)/measured,
                  y+1./math.hypot(1.,slope)/measured)
    cx,cy = center
    return (curvature,direction*(math.hypot(cx,cy)-radius),
            math.atan2(-direction*cx,direction*cy))


def preview_steering(ctx, points, steer):
    if not ctx.cfg.get('lane_curvature_preview', False) or ctx.action is not None:
        ctx.lane_preview = None
        return steer
    forward = sorted(p for p in points if .05 < p[0] <= 1.5)
    if not forward or not np.all(np.isfinite(np.asarray(forward))):
        return steer
    previous = ctx.lane_preview
    pose = getattr(ctx,'pose',(0.,0.,0.))
    ctx.lane_preview = curvature_preview(forward,ctx.lane_stamp,previous,
                                        ctx.cfg['sensor_timeout'])
    if ctx.lane_preview is not None and ctx.lane_preview is not previous:
        ctx.lane_preview['reference_pose'] = tuple(pose)
    curvature,error,heading = reference_geometry(forward,ctx.lane_preview,pose)
    # Use the near measured reference with a bounded feedback horizon (~0.51 m
    # for this wheelbase). Distant exit paint cannot dilute the bend command.
    lookahead = max(1.95*ctx.cfg['wheelbase'],
                    min(ctx.cfg.get('lookahead',.8),forward[0][0]))
    feedback = 2*error/(lookahead*lookahead)+2*math.sin(heading)/lookahead
    desired = curvature+feedback
    physical = math.atan(ctx.cfg['wheelbase']*desired)
    # Heading correction must be allowed to unwind a bypass or an inward
    # drift. A direction lock can command the opposite of the measured lane.
    if ctx.lane_preview is not None:
        ctx.lane_preview.update(center_error_m=error,center_heading_error_rad=heading,
                                center_feedback_curvature=feedback,
                                desired_curvature=desired)
    physical = max(-ctx.cfg['max_steer'],min(ctx.cfg['max_steer'],physical))
    scale = ctx.cfg.get('steering_command_scale_rad',ctx.cfg['max_steer'])
    # Preserve a small real bend command across the integer chassis protocol.
    # Rounding it to zero kept the wheels straight at the startup handoff.
    quantum = ctx.cfg['max_steer']/ctx.cfg.get('steering_raw_limit',22)
    if abs(curvature) >= .35 and physical*curvature > 0 and abs(physical) < quantum:
        physical = math.copysign(quantum,physical)
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
