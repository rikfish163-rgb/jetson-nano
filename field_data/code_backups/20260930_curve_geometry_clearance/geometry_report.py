"""Reproduce steering geometry from the captured exit center samples."""
import importlib.util
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.lane import preview

OUT = os.path.dirname(__file__)
spec = importlib.util.spec_from_file_location('before_preview', os.path.join(OUT, 'preview.before.py'))
before = importlib.util.module_from_spec(spec)
spec.loader.exec_module(before)

CFG = dict(wheelbase=.26, max_steer=.46275, sensor_timeout=.5,
           lane_curvature_preview=True, steering_command_scale_rad=.03)
FRAMES = [
    ('1790725773', [( .635765944,.043665970),(.706168160,.036870968),
                   (.792309472,.020260097),(.865871022,-.001025559),
                   (.948252874,-.032443421)]),
    ('1790725777', [(.533356244,-.035463384),(.598235605,-.063822720),
                   (.668006627,-.105200546),(.750517055,-.167461792),
                   (.841248840,-.251416435),(.924305472,-.341715865)])]


class Context:
    cfg = CFG
    action = None
    lane_preview = None
    lane_stamp = 1.


results = []
fig, axes = plt.subplots(1, 2, figsize=(10, 4.4), constrained_layout=True)
for ax, (stamp, points) in zip(axes, FRAMES):
    old_ctx, ctx = Context(), Context()
    old_command = before.preview_steering(old_ctx, points, 0.)
    new_command = preview.preview_steering(ctx, points, 0.)
    fit = ctx.lane_preview
    point_angles = [math.atan(.26*2*y/(x*x+y*y)) for x,y in points]
    largest = max(point_angles, key=abs)
    feedforward = math.atan(.26*fit['measured_curvature'])
    row = dict(frame_stamp=stamp, measured_points=points,
               model=fit['model'], radius_m=fit['radius_m'],
               measured_curvature=fit['measured_curvature'],
               geometric_feedforward_deg=math.degrees(feedforward),
               old_raw=round(old_command/.03*22),
               new_raw=round(new_command/.03*22),
               maximum_point_raw=round(largest/.46275*22),
               reference_curve=fit['reference_curve'])
    results.append(row)
    ax.plot([p[1] for p in points], [p[0] for p in points], 'o', color='#1565c0', label='Measured centers')
    curve = fit['reference_curve']
    ax.plot([p[1] for p in curve], [p[0] for p in curve], color='#d32f2f', lw=2.5, label='Fitted center curve')
    ax.set_title('%s\nR = %.2f m; raw %d -> %d' % (stamp, fit['radius_m'], row['old_raw'], row['new_raw']))
    ax.set_xlabel('Lateral position, left positive (m)')
    ax.set_ylabel('Forward position (m)')
    ax.set_xlim(-.4,.1)
    ax.set_ylim(.5,1.)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=.25)
    ax.legend(fontsize=8)
fig.suptitle('Exit steering: measured curve geometry, without vehicle motion')
fig.savefig(os.path.join(OUT, 'exit_geometry.png'), dpi=160)
with open(os.path.join(OUT, 'geometry_results.json'), 'w') as stream:
    json.dump(dict(wheelbase_m=.26, max_steering_deg=math.degrees(.46275),
                   steering_raw_limit=22, frames=results), stream, indent=2)
for row in results:
    print('%s radius=%.3f m geometric_angle=%.2f deg old_raw=%d new_raw=%d maximum_point_raw=%d' % (
        row['frame_stamp'], row['radius_m'], row['geometric_feedforward_deg'],
        row['old_raw'], row['new_raw'], row['maximum_point_raw']))
