"""One Pure Pursuit tracker; measured curvature is used only for lane speed."""
from __future__ import division
import math
import numpy as np
from robot.common.geometry import local, world
from robot.common.contracts import model_to_command_steering
from vehicle_control.pure_pursuit import PurePursuit


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


def preview_steering(ctx, points, steer):
    """Track one smooth observed center path with the historical pursuit law.

    Fit only within observed support. The curvature estimate controls speed;
    it never adds another steering command or extrapolates a rear-axle tangent.
    The same pursuit rule applies to straights, bends and bypass reacquisition.
    """
    if ctx.action is not None:
        ctx.lane_preview = None
        return steer
    forward = sorted(p for p in points if .05 < p[0] <= 1.5)
    if len(forward) < 2 or not np.all(np.isfinite(np.asarray(forward))):
        ctx.lane_preview = None
        return steer
    previous = ctx.lane_preview
    ctx.lane_preview = curvature_preview(forward,ctx.lane_stamp,previous,
                                        ctx.cfg['sensor_timeout'])
    preview = ctx.lane_preview
    pose = getattr(ctx,'pose',(0.,0.,0.))
    if preview is not None:
        if preview is not previous:
            preview['reference_pose'] = tuple(pose)
        reference = [local(pose,world(preview['reference_pose'],p))
                     for p in preview['reference_curve']]
    else:
        # A short segment has no independently supported curvature fit.
        # Interpolate its current vertices, keeping exactly the observed span.
        xs = np.linspace(forward[0][0],forward[-1][0],64)
        ys = np.interp(xs,[p[0] for p in forward],[p[1] for p in forward])
        reference = list(zip(xs,ys))
    # Scale the same geometric horizon with the lane speed. A long high-speed
    # target sees the straight exit early and unwinds before the axle exits.
    # Unconfirmed geometry uses the bend speed; promotion requires lane_speed's
    # sustained straight evidence. Selection stays inside the observed curve.
    maximum_speed = ctx.cfg.get('speed_raw',{}).get('lane',24)
    bend_speed = min(maximum_speed,ctx.cfg.get('lane_curve_speed_raw',16))
    speed_state = getattr(ctx,'lane_speed_state',None)
    planning_speed = speed_state['speed'] if speed_state is not None else bend_speed
    lookahead = max(.35,ctx.cfg['lookahead']*planning_speed/max(1.,maximum_speed))
    result = PurePursuit(ctx.cfg['wheelbase'],lookahead).compute(reference)
    if not result.valid:
        ctx.lane_preview = None
        return steer
    physical = max(-ctx.cfg['max_steer'],min(ctx.cfg['max_steer'],result.steering_angle))
    if preview is None:
        preview = dict(stamp=ctx.lane_stamp,curvature=result.curvature,
                       measured_curvature=result.curvature,model='polyline',
                       radius_m=None,residual_m=None,reference_curve=reference,
                       reference_pose=tuple(pose),variance=0.)
        ctx.lane_preview = preview
    preview.update(tracking_curvature=result.curvature,
                   tracking_target=result.target_point,
                   tracking_angle_rad=physical,lookahead_m=lookahead,controller='pure_pursuit')
    return model_to_command_steering(physical,ctx.cfg)


def lane_speed(ctx, speed):
    # A brief gap must not accelerate above the configured lane speed.
    speed = min(speed, ctx.cfg['speed_raw']['lane'])
    slow = min(speed, ctx.cfg.get('lane_curve_speed_raw',12))
    points = sorted(local(ctx.pose,p) for p in ctx.lane)
    points = [p for p in points if p[0] > .05]
    preview = ctx.lane_preview
    # A failed fit, an off-axis exit or a gap cannot establish a straight.
    straight = (ctx.state == 'LANE' and ctx.lane_confidence >= .6 and
        preview is not None and preview['model'] != 'polyline' and len(points) >= 4 and
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
