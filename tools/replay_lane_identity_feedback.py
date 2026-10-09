#!/usr/bin/env python2
"""Offline JPEG replay with recorded calibration; creates no ROS nodes."""
from __future__ import print_function,division
import argparse
import glob
import imp
import json
import os
import sys
import time
import cv2
import numpy as np


def replay(source,run,t0):
    camera=imp.load_source('replay_camera',source)
    with open(os.path.join(run,'calibration.jsonl')) as handle:
        calibration=json.loads(next(handle))['data']
    for key,value in calibration['constants'].items():
        if hasattr(camera,key):
            old=getattr(camera,key)
            if isinstance(old,np.ndarray):value=np.asarray(value,dtype=old.dtype)
            elif isinstance(old,tuple):value=tuple(value)
            setattr(camera,key,value)
    camera.init_undistort_maps()
    rows=[]
    start=time.time()
    for index,path in enumerate(sorted(glob.glob(os.path.join(run,'*_front.jpg')))):
        stamp=int(os.path.basename(path).split('_')[0])/1e9
        frame=cv2.imread(path)
        result=camera.track_metric_lane(camera.make_metric_bev(camera.detect_white_line(frame)),
                                       source_stamp=stamp)
        selected=camera.select_nearest_lane_path_segment(result['center_segments'])
        points=sorted(camera.bev_point_to_vehicle_m(p['x'],p['y']) for p in selected)
        y08=None
        if len(points)>=2 and points[0][0]<=.8<=points[-1][0]:
            y08=float(np.interp(.8,[p[0] for p in points],[p[1] for p in points]))
        rows.append(dict(t=stamp-t0,stamp=stamp,points=points,confidence=result['tracking_confidence'],
                         y08=y08,association=result.get('reference_association')))
        if index%200==0:
            print('replayed',index,'elapsed',round(time.time()-start,1))
            sys.stdout.flush()
    jumps=[]
    for a,b in zip(rows,rows[1:]):
        if (14.5<=a['t']<=60 and a['y08'] is not None and b['y08'] is not None
                and b['t']-a['t']<=.5 and abs(b['y08']-a['y08'])>.30):
            jumps.append(dict(t=b['t'],jump_m=b['y08']-a['y08']))
    return dict(frames=len(rows),jumps_over_30cm=jumps,
                reassociations=sum(r['association']=='right_to_left' for r in rows),rows=rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run');parser.add_argument('output')
    parser.add_argument('--source',default='src/ros/camera/scripts/camera_yihan_web.py')
    args=parser.parse_args()
    with open(os.path.join(args.run,'target.jsonl')) as handle:
        t0=next(json.loads(line)['received'] for line in handle
                if json.loads(line)['data']['command']['speed_raw']!=0)
    result=replay(args.source,args.run,t0)
    with open(args.output,'w') as handle:json.dump(result,handle,indent=2)
    print('frames',result['frames'],'jumps > 30 cm',len(result['jumps_over_30cm']),
          'reassociations',result['reassociations'])


if __name__=='__main__':main()
