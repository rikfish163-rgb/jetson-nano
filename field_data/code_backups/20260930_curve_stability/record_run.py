#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Read-only processed-frame recorder; selection comes from the actual controller."""
from __future__ import print_function
import os
import json
import time
import threading
import Queue
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


def selection_image(event):
    canvas = np.zeros((600,640,3),np.uint8)
    selection = event.get('selection')
    def pixel(point):
        return (int(320-float(point[1])*150), int(570-float(point[0])*250))
    cv2.putText(canvas, 'Actual controller selection (rear axle frame)',(10,22),0,.5,(255,255,255),1)
    cv2.putText(canvas, '%s raw=%s' % (event['state'],event['command']['steering_raw']),
                (10,45),0,.55,(255,255,255),1)
    cv2.circle(canvas,pixel((0,0)),6,(255,255,255),-1)
    if not selection:
        cv2.putText(canvas,'No new lane target: '+event.get('reason',''),(10,75),0,.5,(255,255,255),1)
        return canvas
    points = selection['path']
    for a,b in zip(points,points[1:]):
        cv2.line(canvas,pixel(a),pixel(b),(0,180,0),2)
    for index,p in enumerate(points):
        cv2.circle(canvas,pixel(p),3,(0,255,0),-1)
        cv2.putText(canvas,str(index),pixel(p),0,.35,(255,255,255),1)
    if selection.get('line_x') is not None:
        y = pixel((selection['line_x'],0))[1]
        cv2.line(canvas,(0,y),(639,y),(255,255,0),1)
    if selection.get('intersection') is not None:
        cv2.circle(canvas,pixel(selection['intersection']),8,(255,0,255),2)
    if selection.get('target') is not None:
        cv2.circle(canvas,pixel(selection['target']),10,(0,0,255),2)
    if selection.get('rejected_target') is not None:
        cv2.circle(canvas,pixel(selection['rejected_target']),10,(0,165,255),2)
        cv2.putText(canvas,'orange=rejected left target; right-curve guard',(10,135),0,.5,(0,165,255),1)
    cv2.line(canvas,pixel((0.,0.)),pixel((1.8,0.)),(130,130,130),1)
    cv2.putText(canvas,'red=selected  grey=vehicle centerline',(10,75),0,.5,(255,255,255),1)
    cv2.putText(canvas,selection['selection'],(10,95),0,.5,(255,255,255),1)
    if selection.get('target') is not None:
        target_x=float(selection['target'][0])
        lateral=float(selection['target'][1])
        cv2.putText(canvas,'target x=%.2fm  lateral offset=%.2fm' %
                    (target_x,lateral),(10,115),0,.5,(255,255,255),1)
    return canvas


class Recorder(object):
    def __init__(self):
        root=rospy.get_param('~root','/home/nano/robocup_ws/field_data/lane_runs')
        self.path=os.path.join(root,time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid()))
        os.makedirs(self.path)
        self.queue=Queue.Queue(200)
        self.bridge=CvBridge()
        self.counts={}
        self.sources=dict(front=set(),bev=set(),observation=set())
        self.dropped=0
        self.bytes=0
        self.disabled=None
        self.done=False
        self.limit=int(rospy.get_param('~max_mb',512))*1024*1024
        self.reserve=300*1024*1024
        self.thread=threading.Thread(target=self.worker)
        self.thread.start()
        self.subs=[]
        for key,topic in (('front','/debug/front_raw'),('bev','/debug/lane_tracking')):
            self.subs.append(rospy.Subscriber(topic,Image,
                lambda msg,k=key:self.enqueue(k,msg),queue_size=100,buff_size=8*1024*1024))
        for key,topic in (('target','/competition/lane_target'),('observation','/vision/lane_observation'),
                          ('calibration','/vision/lane_calibration'),('chassis','/base_controller/status')):
            self.subs.append(rospy.Subscriber(topic,String,
                lambda msg,k=key:self.enqueue(k,msg),queue_size=200))
        rospy.on_shutdown(self.close)
        rospy.loginfo('LANE RECORDING: %s (processed frames, max 512 MiB)',self.path)

    def enqueue(self,key,msg):
        if self.disabled or self.done:return
        try:self.queue.put_nowait((key,msg,rospy.Time.now().to_sec()))
        except Queue.Full:
            self.dropped+=1
            rospy.logerr_throttle(2,'LANE RECORDING queue overflow; dropped=%d',self.dropped)

    def summary(self):
        with open(os.path.join(self.path,'summary.json'),'w') as out:
            json.dump(dict(counts=self.counts,dropped_queue=self.dropped,bytes=self.bytes,
                stopped_reason=self.disabled,closed=self.done,
                missing_front=len(self.sources['observation']-self.sources['front']),
                missing_bev=len(self.sources['observation']-self.sources['bev']),
                scope='all received processed frames; not all USB-camera exposures',
                matching='front/bev filename is source_stamp_ns; target log has source_stamp'),out,indent=2)

    def save_image(self,name,frame):
        ok,encoded=cv2.imencode('.jpg',frame,[cv2.IMWRITE_JPEG_QUALITY,85])
        if not ok:raise IOError('JPEG encoding failed')
        data=encoded.tostring()
        with open(os.path.join(self.path,name+'.jpg'),'wb') as out:out.write(data)
        self.bytes+=len(data)

    def worker(self):
        streams={}
        try:
            while not self.done or not self.queue.empty():
                try:key,msg,received=self.queue.get(timeout=.2)
                except Queue.Empty:continue
                if self.disabled:continue
                disk=os.statvfs(self.path)
                if disk.f_bavail*disk.f_frsize<self.reserve or self.bytes>=self.limit:
                    self.disabled='disk_reserve_or_recording_limit'
                    rospy.logerr('LANE RECORDING STOPPED: %s; data kept at %s',self.disabled,self.path)
                    self.summary()
                    continue
                if key in ('front','bev'):
                    name='%d_%s'%(int(round(msg.header.stamp.to_sec()*1e9)),key)
                    self.save_image(name,self.bridge.imgmsg_to_cv2(msg,'bgr8'))
                    self.sources[key].add(int(round(msg.header.stamp.to_sec()*1e9)))
                else:
                    data=json.loads(msg.data)
                    if key=='observation':self.sources[key].add(int(round(data['stamp']*1e9)))
                    if key not in streams:streams[key]=open(os.path.join(self.path,key+'.jsonl'),'a',1)
                    row=json.dumps(dict(received=received,data=data),allow_nan=False)+'\n'
                    streams[key].write(row)
                    self.bytes+=len(row)
                    if key=='target':
                        name='%d_%d_target'%(int(round(data['source_stamp']*1e9)),int(round(data['stamp']*1e9)))
                        self.save_image(name,selection_image(data))
                self.counts[key]=self.counts.get(key,0)+1
                if sum(self.counts.values())%100==0:self.summary()
        except Exception as exc:
            self.disabled='write_error: '+str(exc)
            rospy.logerr('LANE RECORDING FAILED: %s',self.disabled)
        finally:
            for stream in streams.values():stream.close()
            self.summary()

    def close(self):
        self.done=True
        self.thread.join(8)


if __name__=='__main__':
    cv2.setNumThreads(1)
    rospy.init_node('lane_run_recorder')
    recorder=Recorder()
    rospy.spin()
