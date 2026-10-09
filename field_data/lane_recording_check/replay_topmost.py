from __future__ import print_function
import glob,os,json,imp,time
import cv2
import numpy as np
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller

root='/home/nano/robocup_ws'
output=root+'/field_data/lane_recording_check/topmost_replay.json'
report=dict(runs=[],frames=0,unreadable=0,with_target=0,no_path=0,mismatches=0)
snapshots=[(d,sorted(glob.glob(d+'/*_front.jpg')))
           for d in sorted(glob.glob(root+'/field_data/lane_runs/*')) if os.path.isdir(d)]
rows=open(root+'/field_data/lane_recording_check/topmost_frames.jsonl','w')
for run,files in snapshots:
    camera=imp.load_source('topmost_camera',root+'/src/ros/camera/scripts/camera_yihan_web.py')
    calibration=os.path.join(run,'calibration.jsonl')
    if os.path.isfile(calibration):
        with open(calibration) as stream:
            first=stream.readline()
        if first:
            data=json.loads(first)['data']
            for key in ('K','D','H'):
                if key in data:setattr(camera,key,np.asarray(data[key],dtype=np.float64))
    camera.init_undistort_maps()
    cfg=load_config(root+'/src/robot/config')
    cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03)
    controller=Controller(cfg)
    counts=dict(run=os.path.basename(run),files=len(files),frames=0,unreadable=0,with_target=0,no_path=0,mismatches=0)
    for filename in files:
        frame=cv2.imread(filename)
        if frame is None:
            counts['unreadable']+=1
            continue
        stamp=int(os.path.basename(filename).split('_')[0])/1e9
        mask=camera.make_metric_bev(camera.detect_white_line(frame))
        tracked=camera.track_metric_lane(mask,source_stamp=stamp)
        points=[camera.bev_point_to_vehicle_m(p['x'],p['y']) for p in
                camera.select_nearest_lane_path_segment(tracked['center_segments'])]
        controller.observe_lane(points,tracked['tracking_confidence'],stamp)
        controller.lane_target=None
        speed,steer=controller.lane_command(stamp)
        command=encode_command(speed,steer,cfg,0)
        counts['frames']+=1
        if points:
            counts['with_target']+=1
            expected=max(points,key=lambda p:p[0])
            expected_raw=int(round(22*max(-1.,min(1.,expected[1]/.30))))
            selection=controller.lane_target
            ok=(selection is not None and tuple(selection['target'])==tuple(expected)
                and command['steering_raw']==expected_raw
                and selection['selection']=='topmost_lateral_offset')
            if not ok:counts['mismatches']+=1
        else:
            counts['no_path']+=1
        rows.write(json.dumps(dict(run=counts['run'],stamp=stamp,points=points,
            selection=controller.lane_target,command=command))+'\n')
    controller.close()
    for key in ('frames','unreadable','with_target','no_path','mismatches'):report[key]+=counts[key]
    report['runs'].append(counts)
    print(json.dumps(counts))
    with open(output,'w') as stream:json.dump(report,stream,indent=2)
rows.close()
print(json.dumps(report,indent=2))
assert report['frames']>0 and report['mismatches']==0,report
