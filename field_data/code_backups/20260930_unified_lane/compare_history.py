"""Read-only replay of the user-selected historical camera on one recording."""
from __future__ import print_function, division
import glob
import imp
import json
import math
import os
import sys
import cv2
import numpy as np

root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
run = os.path.join(root, 'field_data/lane_runs/20260930_200948_24256')
camera = imp.load_source('historical_camera', os.path.join(os.path.dirname(__file__),
                         'history/camera_20260911.py'))
with open(os.path.join(run, 'calibration.jsonl')) as handle:
    calibration = json.loads(next(handle))['data']
for key, value in calibration['constants'].items():
    if hasattr(camera, key):
        old = getattr(camera, key)
        if isinstance(old, np.ndarray): value = np.asarray(value, dtype=old.dtype)
        elif isinstance(old, tuple): value = tuple(value)
        setattr(camera, key, value)
camera.init_undistort_maps()
class Clock(object):
    stamp = 0.
    @classmethod
    def time(cls): return cls.stamp
camera.time = Clock
rows = []
for index, path in enumerate(sorted(glob.glob(os.path.join(run, '*_front.jpg')))):
    stamp = int(os.path.basename(path).split('_')[0])/1e9
    t = stamp-1790770209.035818
    if t < 10. or t > 31. or index % 2: continue
    Clock.stamp = stamp
    result = camera.track_metric_lane(camera.make_metric_bev(
        camera.detect_white_line(cv2.imread(path))))
    selected = camera.select_nearest_lane_path_segment(result['center_segments'])
    points = [camera.bev_point_to_vehicle_m(p['x'], p['y']) for p in selected]
    target = min(points, key=lambda p: abs(math.hypot(*p)-.7)) if points else None
    angle = math.atan2(2*.26*target[1], sum(v*v for v in target)) if target else 0.
    raw = int(round(max(-22., min(22., angle/.1*22.))))
    row = dict(t=t, stamp=stamp, mode=result['lane_mode'],
               confidence=result['tracking_confidence'], points=points, raw=raw)
    rows.append(row)
    if 18.5 < t < 25. and index%10==0:
        print(round(t,2),result['lane_mode'],round(row['confidence'],2),len(points),raw)
        sys.stdout.flush()
with open(os.path.join(os.path.dirname(__file__), 'history_replay.json'), 'w') as handle:
    json.dump(rows, handle, indent=2)
print('frames',len(rows),'valid',sum(bool(r['points']) for r in rows))
