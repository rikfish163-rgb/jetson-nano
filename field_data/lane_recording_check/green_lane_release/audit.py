"""Replay saved camera frames locally on Nano; no ROS publishers or actuators."""
from __future__ import print_function
import glob
import imp
import json
import os
import cv2
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller

ROOT = '/home/nano/robocup_ws'
OUT = ROOT+'/field_data/lane_recording_check/green_lane_release'
camera = imp.load_source('handoff_camera',ROOT+'/src/ros/camera/scripts/camera_yihan_web.py')
camera.init_undistort_maps()
cfg = load_config(ROOT+'/src/robot/config')
cfg.update(wait_green=False, lidar_enabled=False, parking_enabled=False,
           lane_curvature_preview=True, steering_command_scale_rad=.03)
cores = {}
for lookahead in (.8,.6,.5):
    trial = dict(cfg,lookahead=lookahead)
    cores[str(lookahead)] = Controller(trial)
rows = []
for filename in sorted(glob.glob(ROOT+'/field_data/sign_capture/20260930_053708_21915/*_frame.png')):
    stamp = float(os.path.basename(filename).split('_')[0])
    if not 1790717897 <= stamp <= 1790717932:
        continue
    frame = cv2.imread(filename)
    mask = camera.make_metric_bev(camera.detect_white_line(frame))
    tracked = camera.track_metric_lane(mask,source_stamp=stamp)
    points = [camera.bev_point_to_vehicle_m(p['x'],p['y']) for p in
              camera.select_nearest_lane_path_segment(tracked['center_segments'])]
    row = dict(stamp=stamp,frame=filename,points=points,
               confidence=tracked['tracking_confidence'],commands={})
    for name,core in cores.items():
        core.observe_lane(points,row['confidence'],stamp)
        speed,steer = core.tick(stamp)
        row['commands'][name] = dict(encode_command(speed,steer,core.cfg,0),
            reason=core.reason,target=core.lane_target)
    rows.append(row)
    print(json.dumps(dict(stamp=stamp,points=points,confidence=row['confidence'],
        raw={k:v['steering_raw'] for k,v in row['commands'].items()},
        speed={k:v['speed_raw'] for k,v in row['commands'].items()})))
    cv2.imwrite(OUT+'/'+str(stamp)+'_bev.png',mask)
for core in cores.values():
    core.close()
with open(OUT+'/camera_replay.json','w') as stream:
    json.dump(dict(method='Sparse saved sign-camera frames, current calibration, reset tracking history, fixed pose; not a physical trajectory replay.',rows=rows),stream,indent=2)
