"""Compare saved JPEG frames with the before/after camera code, without ROS I/O."""
from __future__ import print_function
import glob
import imp
import json
import os
import sys
import cv2
import numpy as np

ROOT = '/home/nano/robocup_ws'
RUN = ROOT + '/field_data/lane_runs/20260929_232406_4508'
OUT = ROOT + '/field_data/lane_recording_check/right_curve_guard'
SOURCE = '/src/ros/camera/scripts/camera_yihan_web.py'
BACKUP = '/home/nano/robocup_cleanup_backups/20260929/right_curve_guard'

calibration = json.loads(next(iter(open(RUN + '/calibration.jsonl'))))['data']
before = imp.load_source('camera_before_guard_replay', BACKUP + SOURCE)
after = imp.load_source('camera_after_guard_replay', ROOT + SOURCE)
for camera in (before, after):
    for name in ('K', 'D', 'H'):
        setattr(camera, name, np.array(calibration[name], dtype=np.float64))
    camera.init_undistort_maps()

summary = dict(method='Chronological saved JPEG replay; independent tracking histories, same mask and recorded K/D/H. No ROS publishers or vehicle movement.',
               frames=0, unreadable=0, before_no_path=0, after_no_path=0,
               recovered=0, lost=0, known_two_point_frame=None)
stream = open(OUT + '/camera_frames.jsonl', 'w')
for filename in sorted(glob.glob(RUN + '/*_front.jpg')):
    image = cv2.imread(filename)
    if image is None:
        summary['unreadable'] += 1
        continue
    stamp_ns = os.path.basename(filename).split('_')[0]
    stamp = int(stamp_ns) / 1e9
    mask = after.make_metric_bev(after.detect_white_line(image))
    records = []
    for camera in (before, after):
        result = camera.track_metric_lane(mask.copy(), source_stamp=stamp)
        path = camera.select_nearest_lane_path_segment(result['center_segments'])
        records.append(dict(path_points=len(path),
                            path_uv=[[float(p['x']), float(p['y'])] for p in path],
                            lane_mode=result.get('lane_mode')))
    row = dict(file=os.path.basename(filename), source_stamp=stamp,
               before=records[0], after=records[1])
    stream.write(json.dumps(row) + '\n')
    summary['frames'] += 1
    no_before, no_after = [r['path_points'] == 0 for r in records]
    summary['before_no_path'] += no_before
    summary['after_no_path'] += no_after
    summary['recovered'] += no_before and not no_after
    summary['lost'] += no_after and not no_before
    if stamp_ns == '1790695536524874752':
        summary['known_two_point_frame'] = row
    if summary['frames'] % 200 == 0:
        print('processed', summary['frames'], 'recovered', summary['recovered'], 'lost', summary['lost'])
        sys.stdout.flush()
stream.close()
with open(OUT + '/camera_summary.json', 'w') as output:
    json.dump(summary, output, indent=2)
print(json.dumps(summary, indent=2))
assert summary['frames'] == 1029 and summary['unreadable'] == 0
assert summary['known_two_point_frame']['before']['path_points'] == 0
assert summary['known_two_point_frame']['after']['path_points'] > 0
