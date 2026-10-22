#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Subscribe to the existing camera; no actuator publisher, no camera ownership."""
from __future__ import print_function
import argparse
import json
import os
import sys
import threading
import time
import uuid
from common import LABELS


def main():
    p = argparse.ArgumentParser(description='采集现有 ROI 与原图；只订阅，不控制车辆')
    p.add_argument('--root',required=True)
    p.add_argument('--label',choices=LABELS + ['background'],required=True)
    p.add_argument('--split',choices=['train','val','test'],required=True)
    p.add_argument('--topic',default='/front/usb_cam/image_raw')
    p.add_argument('--count',type=int,default=150)
    p.add_argument('--hz',type=float,default=2)
    p.add_argument('--duration',type=float,default=180)
    p.add_argument('--preview',action='store_true')
    p.add_argument('--runtime-scripts',default='/home/nano/robocup_ws/src/ros/signs/scripts')
    args = p.parse_args()
    if not 0 < args.hz <= 20 or args.count < 1 or args.duration <= 0:
        p.error('count/duration must be positive; hz must be in (0,20]')
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    cv2.setNumThreads(1)
    sys.path.insert(0,args.runtime_scripts)
    from sign_string_node import extract_sign_roi
    run = time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
    target = os.path.join(os.path.abspath(args.root),args.split,args.label,run)
    for folder in ('roi','frames','missed'):
        os.makedirs(os.path.join(target,folder))
    with open(os.path.join(target,'session.json'),'w') as f:
        json.dump(vars(args),f,indent=2)
    print('采集目录:',target)
    if args.label == 'background':
        print('非路牌采集：移走所有真实路牌，摆放锥桶、蓝线、衣物等；改变位置与光照。')
        print('只计入颜色筛选实际选中的候选 ROI；没有候选的画面保存在 missed/。')
    else:
        print('请保持只有当前类别的牌入镜，并缓慢改变距离、角度和光照。')
    print('保存的是自动裁剪结果，请之后检查 roi/；裁错的图不能保留当前标签。')
    latest,lock = [None],threading.Lock()
    def callback(msg):
        with lock:
            latest[0] = msg
    rospy.init_node('sign_dataset_capture',anonymous=True)
    crop_pub = rospy.Publisher('/debug/sign_capture_roi', Image, queue_size=1)
    frame_pub = rospy.Publisher('/debug/sign_capture_frame', Image, queue_size=1)
    last_preview = 0
    sub = rospy.Subscriber(args.topic,Image,callback,queue_size=1,buff_size=2**22)
    bridge = CvBridge()
    start,last_warn,last_stamp,saved,missed = time.time(),0,-1,0,0
    paused = False
    rate = rospy.Rate(args.hz)
    try:
        while not rospy.is_shutdown() and saved < args.count and time.time()-start < args.duration:
            rate.sleep()
            with lock:
                msg = latest[0]
            if msg is None or msg.header.stamp.to_sec() <= last_stamp or not 0 <= rospy.Time.now().to_sec()-msg.header.stamp.to_sec() <= 1:
                if time.time()-last_warn > 5:
                    print('等待新鲜相机画面；请确认前摄正在发布',args.topic)
                    last_warn = time.time()
                continue
            last_stamp = msg.header.stamp.to_sec()
            frame = bridge.imgmsg_to_cv2(msg,'bgr8')
            candidate = extract_sign_roi(frame, return_bounds=True)
            roi, bounds = candidate if candidate is not None else (None, None)
            panel = frame.copy()
            if bounds is not None:
                x, y, w, h = bounds
                cv2.rectangle(panel, (x,y), (x+w,y+h), (0,255,255), 2)
            cv2.putText(panel,'%s %d/%d' % (args.label,saved,args.count),
                        (10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
            if time.time() - last_preview >= .2:
                last_preview = time.time()
                display = bridge.cv2_to_imgmsg(panel, 'bgr8')
                display.header = msg.header
                frame_pub.publish(display)
                if roi is not None:
                    display = bridge.cv2_to_imgmsg(roi, 'bgr8')
                    display.header = msg.header
                    crop_pub.publish(display)
            if args.preview:
                panel = frame.copy()
                cv2.putText(panel,'%s %d/%d %s' % (args.label,saved,args.count,'PAUSED' if paused else 'CAPTURING'),
                            (10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
                cv2.imshow('Sign capture - space pause / q quit',panel)
                if roi is not None: cv2.imshow('Actual training ROI',cv2.resize(roi,(224,224)))
                key = cv2.waitKey(1) & 255
                if key == ord('q'): break
                if key == 32: paused = not paused
                if paused: continue
            name = '%.9f' % last_stamp
            if roi is None:
                missed += 1
                if missed % 10 == 1:
                    if not cv2.imwrite(os.path.join(target,'missed',name+'.jpg'),frame):
                        raise IOError('Failed to save missed frame')
                    print('没有候选牌，已保存原图到 missed/；调整牌的位置，不给背景打 P 标签。')
                continue
            if not cv2.imwrite(os.path.join(target,'frames',name+'.jpg'),frame):
                raise IOError('Failed to save frame')
            if not cv2.imwrite(os.path.join(target,'roi',name+'.png'),roi):
                raise IOError('Failed to save ROI')
            saved += 1
            if saved % 10 == 0:
                print('%s/%s 已保存 %d/%d，未检测帧 %d' % (args.split,args.label,saved,args.count,missed))
    finally:
        sub.unregister()
        if args.preview: cv2.destroyAllWindows()
    print('结束：保存 %d，未检测 %d，目录 %s' % (saved,missed,target))
    if saved < args.count:
        print('未达到目标数量；已保存样本保留，可再次采集，不会覆盖。')
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
