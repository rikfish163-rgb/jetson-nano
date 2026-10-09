"""Inspect four saved images on Nano; no ROS node, publisher, or actuator."""
from __future__ import print_function
import glob
import imp
import json
import os
import cv2
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller

root = '/home/nano/robocup_ws'
camera = imp.load_source('center_tracking_audit_camera',root+'/src/ros/camera/scripts/camera_yihan_web.py')
camera.init_undistort_maps()
cfg = load_config(root+'/src/robot/config')
cfg.update(wait_green=False,lidar_enabled=False,lane_curvature_preview=True,
           steering_command_scale_rad=.03,lane_curve_speed_raw=24)
cfg['speed_raw'].update(lane=24,action=24)
rows = []
for prefix in ('1790720918.892','1790720923.893','1790720938.356','1790720960.357'):
    files = glob.glob(root+'/field_data/sign_capture/20260930_062804_26342/'+prefix+'*_frame.png')
    if not files:
        continue
    filename = files[0]
    stamp = float(os.path.basename(filename).split('_')[0])
    mask = camera.make_metric_bev(camera.detect_white_line(cv2.imread(filename)))
    tracked = camera.track_metric_lane(mask,source_stamp=stamp)
    points = [camera.bev_point_to_vehicle_m(p['x'],p['y']) for p in
              camera.select_nearest_lane_path_segment(tracked['center_segments'])]
    core = Controller(cfg)
    core.observe_lane(points,tracked['tracking_confidence'],stamp)
    speed,steer = core.tick(stamp)
    row = dict(stamp=stamp,path=points,confidence=tracked['tracking_confidence'],
        lane_valid=core.lane_valid(stamp),command=encode_command(speed,steer,cfg,0),reason=core.reason)
    rows.append(row)
    print(json.dumps(row))
    core.close()
with open(root+'/field_data/lane_recording_check/center_tracking_raw24/saved_frames.json','w') as stream:
    json.dump(dict(method='Four sparse saved images, reset vehicle pose and no original tracker history; visual/control evidence only, not trajectory replay.',rows=rows),stream,indent=2)
