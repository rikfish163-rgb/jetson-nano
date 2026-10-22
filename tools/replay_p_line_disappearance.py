#!/usr/bin/env python
"""Replay an existing bag without ROS nodes, camera capture or control output."""
from __future__ import print_function
import argparse
import json
import time
import cv2
import numpy as np
import rosbag
from robot.common.config import load_config,read_mapping
from robot.parking.timed_vision import BayLineVision


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('bag')
    parser.add_argument('--start',type=float,required=True)
    parser.add_argument('--end',type=float,required=True)
    parser.add_argument('--preview',default='')
    args=parser.parse_args()
    cfg=load_config('src/robot/config')
    cfg['front_camera']['H']=read_mapping('src/robot/parallel_parking/reference_camera.yaml')['front_camera']['H']
    vision=BayLineVision(cfg,argparse.Namespace(**cfg['parking_timed']))
    cv2.setNumThreads(1)
    images,lanes,signs={},{},[]
    with rosbag.Bag(args.bag) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/front/usb_cam/image_raw/compressed',
                                  '/vision/lane_observation','/competition/sign']):
            if topic.endswith('compressed'):
                stamp=msg.header.stamp.to_sec()
                if args.start <= stamp <= args.end:images[round(stamp,5)]=msg.data
            else:
                data=json.loads(msg.data)
                if topic.endswith('/sign') and data.get('label')=='PARKING':
                    signs.append(dict(stamp=data['stamp'],sign_bounds=data['sign_bounds'],
                                      track_id=(data.get('route_gate') or {}).get('track_id')))
                elif topic.endswith('lane_observation') and args.start <= data['stamp'] <= args.end:
                    lanes[round(data['stamp'],5)]=data
    last=-float('inf')
    preview_saved=False
    for key,lane in sorted(lanes.items()):
        if key not in images or key-last < .19:continue
        last=key
        eligible=[s for s in signs if 0 <= lane['stamp']-s['stamp'] <= .8]
        vision.sign_observation=eligible[-1] if eligible else None
        vision.set_lane_observation(lane)
        started=time.time()
        frame=cv2.imdecode(np.frombuffer(images[key],np.uint8),1)
        count,debug=vision.observe(frame)
        reference=vision.p_reference
        print(json.dumps(dict(stamp=key,lines=count,p_reference=reference,
                              processing_ms=1000*(time.time()-started))))
        if args.preview and reference['locked'] and reference['visible'] and not preview_saved:
            # Show the identified ground stripe in the rectified recorded image.
            c=vision.camera
            points=np.array([[[c['origin_u']-p[1]*c['pixels_per_m']-360,
                               c['origin_v']-p[0]*c['pixels_per_m']] for p in reference['line']]],float)
            points=cv2.perspectiveTransform(points,np.linalg.inv(vision.detector.H))[0]
            rectified=cv2.remap(frame,vision.detector.maps[0],vision.detector.maps[1],cv2.INTER_LINEAR)
            cv2.line(rectified,tuple(np.rint(points[0]).astype(int)),
                     tuple(np.rint(points[1]).astype(int)),(0,255,0),3)
            x,y,w,h=reference['p_bounds']
            box=cv2.undistortPoints(np.array([[[x,y],[x+w,y+h]]],float),
                                   vision.K,vision.D,P=vision.newK).reshape(-1,2)
            cv2.rectangle(rectified,tuple(np.rint(box[0]).astype(int)),
                          tuple(np.rint(box[1]).astype(int)),(0,255,255),2)
            if not cv2.imwrite(args.preview,rectified):raise ValueError('preview write failed')
            preview_saved=True
        if reference['ended']:break


if __name__=='__main__':main()
