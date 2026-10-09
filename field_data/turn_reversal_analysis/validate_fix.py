"""Offline evidence: original-time bag replay and captured-paint association."""
from __future__ import print_function
import glob
import imp
import json
import os
import cv2
import rosbag
from cv_bridge import CvBridge

ROOT = '/home/nano/robocup_ws'
OUT = os.path.join(ROOT, 'field_data/turn_reversal_analysis')
old = imp.load_source('before_association', '/home/nano/refactor_backups/turn-association-20260919/camera_yihan_web.py')
new = imp.load_source('after_association', os.path.join(ROOT, 'src/ros/camera/scripts/camera_yihan_web.py'))
summary = {}
for name, lane in [('before', old), ('after', new)]:
    lane.configure_lane_windows(40, .15)
    lane.init_undistort_maps()
    results = []
    bridge = CvBridge()
    with rosbag.Bag(os.path.join(ROOT, 'field_data/lane_record_20260919_025049/run.bag')) as bag:
        for _, msg, t in bag.read_messages(topics=['/debug/metric_bev']):
            mask = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            result = lane.track_metric_lane(mask, source_stamp=msg.header.stamp.to_sec())
            results.append(dict(stamp=msg.header.stamp.to_sec(), mode=result['lane_mode'],
                                confidence=result['tracking_confidence'],
                                reason=result['path_diagnostic']['reason']))
    summary[name] = dict(frames=len(results), valid=sum(r['confidence'] >= .35 for r in results),
                         last62_valid=sum(r['confidence'] >= .35 for r in results[-62:]))
    with open(os.path.join(OUT, name+'_bag_replay.json'), 'w') as f:
        json.dump(results, f)

# These captures are two seconds apart, not a continuous stream. Test the
# geometry association separately; do not pretend they are adjacent frames.
new.configure_lane_windows(40, .15)
capture = os.path.join(ROOT, 'field_data/sign_capture/20260919_203433_14802')
frames = []
for prefix in ('1789821295.', '1789821297.'):
    path = glob.glob(os.path.join(capture, prefix+'*_frame.png'))[0]
    mask = new.make_metric_bev(new.detect_white_line(cv2.imread(path)))
    new.configure_lane_windows(40, .15)
    result = new.track_metric_lane(mask, source_stamp=0.)
    frames.append((mask, result))
first, second = frames[0][1], frames[1][1]
new.boundary_association = (0., first['left_points'], first['right_points'])
left, right = new.associate_boundary_roles(second['left_points'], second['right_points'], .1, second['lane_width_est'])
summary['capture_geometry_only'] = dict(before=second['lane_mode'], after=new.determine_lane_tracking_mode(left, right))
assert summary['capture_geometry_only']['after'] == 'LEFT_ONLY', summary
print(json.dumps(summary, indent=2))
with open(os.path.join(OUT, 'fix_validation.json'), 'w') as f:
    json.dump(summary, f, indent=2)
