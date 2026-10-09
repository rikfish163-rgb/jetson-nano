#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Score collection-compatible sign crops with the model and SOURCE stamps."""
import json
import os
import sys
import threading
import time
import cv2
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot.common.contracts import validate_config
from robot.signs.capture import save_capture, CaptureWriter
from robot.signs.candidate_gate import RouteSignGate


class Node(object):
    def __init__(self):
        cfg = rospy.get_param('~config') if rospy.has_param('~config') else rospy.get_param('/competition/config')
        validate_config(cfg)
        cv2.setNumThreads(1)
        scripts = os.path.join(rospkg.RosPack().get_path('hts_ros_pkg'),'scripts')
        sys.path.insert(0,scripts)
        from sign_classifier_cv import SignClassifier
        from sign_string_node import normalize_model_label, extract_sign_roi
        self.normalize = normalize_model_label
        self.roi = extract_sign_roi
        self.roi_mode = rospy.get_param('~roi_mode','training_roi')
        if self.roi_mode == 'board':
            from board_roi import extract_board_roi
            self.roi = extract_board_roi
        elif self.roi_mode != 'training_roi':
            raise ValueError('unsupported sign roi_mode: '+self.roi_mode)
        self.backend = rospy.get_param('~backend','classifier')
        if self.backend == 'yolo_trt':
            from robot.signs.yolo_detector import YoloDetector
            self.detector = YoloDetector(rospy.get_param('~model'))
            rospy.on_shutdown(self.detector.close)
            rospy.loginfo('Sign backend: YOLOv5s TensorRT, model=%s, full-frame 640',rospy.get_param('~model'))
        elif self.backend == 'classifier':
            self.classifier = SignClassifier(
                rospy.get_param('~model',os.path.join(scripts,'resnet18.onnx')),
                labels_path=rospy.get_param('~labels','') or None)
        else:
            raise ValueError('unsupported sign backend: '+self.backend)
        self.bridge, self.latest, self.lock = CvBridge(), None, threading.Lock()
        self.last = -1.0
        self.timeout = cfg['sign_timeout']
        self.threshold = cfg['sign_confidence']
        self.min_height_ratio = cfg.get('sign_min_height_ratio',.125)
        route_settings = cfg.get('sign_route_gate', {})
        self.route_gate = (RouteSignGate(route_settings, self.threshold, self.min_height_ratio)
                           if route_settings.get('enabled', False) else None)
        workspace = os.path.dirname(os.path.dirname(rospkg.RosPack().get_path('robocup_competition')))
        self.capture_dir = rospy.get_param('~capture_dir', os.path.join(
            workspace, 'field_data', 'sign_capture',
            time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid())))
        rospy.loginfo('Sign inference captures: %s', self.capture_dir)
        self.accept_capture_at = -float('inf')
        self.accept_capture_label = None
        self.capture_writer = None
        if self.capture_dir:
            log_root = os.environ.get('ROS_LOG_DIR', os.path.expanduser('~/.ros/log'))
            session = dict(started_at=time.time(),
                ros_log_dir=os.path.join(log_root, rospy.get_param('/run_id', '')))
            self.capture_writer = CaptureWriter(self.capture_dir,
                on_saved=lambda prefix, info: rospy.loginfo(
                    'sign_capture label=%s top_label=%s confidence=%.4f source_stamp=%.9f files=%s',
                    info['label'], info['top_label'], info['confidence'], info['stamp'], prefix),
                on_error=lambda message: rospy.logwarn_throttle(2.0, '%s', message),
                session=session)
            rospy.on_shutdown(self.capture_writer.close)
        self.reject_capture_count = 0
        self.reject_capture_at = -float('inf')
        self.reject_capture_limit = 60
        self.pub = rospy.Publisher('/competition/sign',String,queue_size=1)
        self.debug_frame = None
        if rospy.get_param('~debug_images',False):
            self.debug_frame = rospy.Publisher('/debug/sign_board_frame',Image,queue_size=1)
            self.debug_crop = rospy.Publisher('/debug/sign_board_crop',Image,queue_size=1)
        rospy.Subscriber(cfg['front_image_topic'],Image,self.callback,queue_size=1,buff_size=2**22)
        self.timer = rospy.Timer(rospy.Duration(1.0/cfg['sign_hz']),self.infer)

    def callback(self,msg):
        with self.lock:
            self.latest = msg

    def infer(self,event):
        with self.lock:
            msg = self.latest
        if msg is None:
            return
        stamp = msg.header.stamp.to_sec()
        if stamp <= self.last or not 0 <= rospy.Time.now().to_sec()-stamp <= self.timeout:
            return
        self.last = stamp
        try:
            started = time.time()
            frame=self.bridge.imgmsg_to_cv2(msg,'bgr8')
            mode=getattr(self,'roi_mode','training_roi')
            diagnostics={}
            detections=[]
            eligible_detections=[]
            route_gate=getattr(self,'route_gate',None)
            crop=None
            scores,top_label,confidence,bounds = {},'',0.0,None
            if getattr(self,'backend','classifier') == 'yolo_trt':
                from robot.signs.yolo_detector import select_detection
                mode='full_frame_yolo';roi_done=started
                detections=sorted(self.detector.detect(frame),
                    key=lambda d:(d['bounds'][2]*d['bounds'][3],d['confidence']),reverse=True)
                eligible_detections=(route_gate.filter(detections,frame.shape)
                                     if route_gate is not None else detections)
                selected=select_detection(eligible_detections,self.threshold)
                if selected is None and detections:
                    # Preserve rejected target pixels and raw prediction for capture evidence.
                    selected=select_detection(detections,self.threshold)
                diagnostics=dict(candidate_count=len(detections),selection='largest_detected_area',
                    regions=[dict(bounds=d['bounds'],reason='#%d %s %.2f' % (i+1,d['label'],d['confidence']))
                             for i,d in enumerate(detections)])
                candidate=None
                if selected is not None:
                    bounds=selected['bounds'];x,y,w,h=bounds
                    crop=frame[y:y+h,x:x+w].copy();candidate=(crop,bounds)
                    scores=dict((self.normalize(k),v) for k,v in selected['scores'].items())
                    top_label=self.normalize(selected['label']);confidence=selected['confidence']
            else:
                candidate=(self.roi(frame,return_bounds=True,diagnostics=diagnostics)
                           if mode=='board' else self.roi(frame,return_bounds=True))
                roi_done=time.time()
                if candidate is not None:
                    crop,bounds=candidate
                    scores=dict(('BACKGROUND' if label=='background' else self.normalize(label),score)
                                for label,score in self.classifier.classify_scores(crop).items())
                    top_label=max(scores,key=scores.get)
                    confidence=scores[top_label]
            label=top_label if confidence>=self.threshold and top_label!='BACKGROUND' else ''
            # Height survives partial clipping at the left/right image edge.
            # Keep all candidates for visibility/rearm even when too small to act on.
            height_ratio = float(bounds[3])/frame.shape[0] if bounds is not None else None
            too_small = (top_label in ('LEFT','RIGHT','STRAIGHT','UTURN','PARKING') and
                         height_ratio is not None and
                         height_ratio < getattr(self,'min_height_ratio',.125))
            if too_small:
                label=''
            info = dict(stamp=stamp,label=label,confidence=confidence,
                scores=scores,top_label=top_label,threshold=self.threshold,
                source='yolov5s' if mode=='full_frame_yolo' else 'classifier',input_mode=mode,startup_only=False,
                range_reason='too_small' if too_small else mode if candidate is not None else 'no_candidate',
                sign_height_ratio=height_ratio,
                sign_min_height_ratio=getattr(self,'min_height_ratio',.125),
                sign_depth_m=None,sign_bounds=bounds,roi_diagnostics=diagnostics,
                roi_ms=1000*(roi_done-started),inference_ms=1000*(time.time()-roi_done),
                source_age_s=rospy.Time.now().to_sec()-stamp)
            if mode=='full_frame_yolo':info['detections']=detections
            if route_gate is not None:
                label,gate_reason,gate_info=route_gate.evaluate(
                    top_label,confidence,bounds,frame.shape,stamp)
                info.update(label=label,range_reason=gate_reason,route_gate=gate_info)
                if mode=='full_frame_yolo':info['route_detections']=eligible_detections
            self.pub.publish(String(data=json.dumps(info,allow_nan=False)))
            show_frame = (getattr(self,'debug_frame',None) is not None and
                          self.debug_frame.get_num_connections() > 0)
            show_crop = (getattr(self,'debug_crop',None) is not None and
                         self.debug_crop.get_num_connections() > 0)
            if show_frame:
                display=frame.copy()
                for region in diagnostics.get('regions',[]):
                    x,y,w,h=region['bounds']
                    reason=region['reason']
                    color=(0,180,0) if reason=='candidate' else (0,100,255)
                    cv2.rectangle(display,(x,y),(x+w,y+h),color,1)
                    cv2.putText(display,reason,(x,max(12,y-3)),cv2.FONT_HERSHEY_SIMPLEX,.35,color,1)
                if bounds is not None:
                    x,y,w,h=bounds
                    cv2.rectangle(display,(x,y),(x+w,y+h),(0,255,255),2)
                # Add a header instead of hiding signs in the original top rows.
                display=np.vstack((np.zeros((104,frame.shape[1],3),dtype=np.uint8),display))
                cv2.putText(display,'%s %.3f %s' % (top_label or 'NONE',confidence,
                            'ACCEPT' if label else 'REJECT'),(8,24),
                            cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),2)
                ranking='  '.join('%s %.2f' % item for item in
                                 sorted(scores.items(),key=lambda item:item[1],reverse=True)[:3])
                cv2.putText(display,ranking or 'NO CANDIDATE',(8,48),
                            cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                cv2.putText(display,'%s age=%.2fs threshold=%.2f' % (
                            rospy.get_name(),rospy.Time.now().to_sec()-stamp,self.threshold),
                            (8,72),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
                rejected=','.join('%s:%s' % item for item in sorted(diagnostics.get('rejected_counts',{}).items()))
                cv2.putText(display,'candidates=%s %s' % (diagnostics.get('candidate_count','?'),rejected),
                            (8,94),cv2.FONT_HERSHEY_SIMPLEX,.35,(255,255,255),1)
                debug=self.bridge.cv2_to_imgmsg(display,'bgr8');debug.header=msg.header
                self.debug_frame.publish(debug)
            if show_crop:
                crop_display=crop if candidate is not None else frame[:32,:32].copy()*0
                debug=self.bridge.cv2_to_imgmsg(crop_display,'bgr8');debug.header=msg.header
                self.debug_crop.publish(debug)
            rejected_capture=(not label and stamp-getattr(self,'reject_capture_at',-float('inf'))>=2.0
                and getattr(self,'reject_capture_count',0)<getattr(self,'reject_capture_limit',60))
            accepted_capture = bool(label) and (
                label != getattr(self, 'accept_capture_label', None) or
                stamp-getattr(self, 'accept_capture_at', -float('inf')) >= 1.0)
            if self.capture_dir and (accepted_capture or rejected_capture):
                try:
                    if rejected_capture:
                        self.reject_capture_at=stamp
                        self.reject_capture_count=getattr(self,'reject_capture_count',0)+1
                    writer = getattr(self, 'capture_writer', None)
                    if writer is not None:
                        queued = writer.submit(frame, crop, info)
                    else:
                        # Support direct/offline invocation without a ROS lifecycle.
                        prefix = save_capture(self.capture_dir,frame,crop,info)
                        rospy.loginfo('sign_capture label=%s confidence=%.4f source_stamp=%.9f files=%s',
                                      label,confidence,stamp,prefix)
                        queued = True
                    if queued and accepted_capture:
                        self.accept_capture_at, self.accept_capture_label = stamp, label
                except Exception as exc:
                    rospy.logwarn_throttle(2.0,'sign capture failed: %s',str(exc))
        except Exception as exc:
            rospy.logwarn_throttle(2.0,'sign inference rejected: %s',str(exc))


if __name__ == '__main__':
    rospy.init_node('competition_sign')
    Node()
    rospy.spin()
