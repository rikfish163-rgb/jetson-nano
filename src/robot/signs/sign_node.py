#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Publish YOLO sign detections with source image timestamps."""
import json
import threading
import time
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot.common.contracts import validate_config
from robot.signs.yolo_detector import YoloDetector, select_detection, LABELS
from robot.signs.candidate_gate import RouteSignGate
from robot.parking.reverse_entry import reverse_after_turn_enabled


LABEL_MAP = dict((label, "PARKING" if label == "park" else label.upper()) for label in LABELS)

def normalize_model_label(label):
    return LABEL_MAP.get(str(label).strip().lower(), "") if label is not None else ""


class Node(object):
    def __init__(self):
        cfg = rospy.get_param('~config') if rospy.has_param('~config') else rospy.get_param('/competition/config')
        validate_config(cfg)
        cv2.setNumThreads(1)
        self.normalize = normalize_model_label
        self.detector = YoloDetector(rospy.get_param('~model'))
        rospy.on_shutdown(self.detector.close)
        rospy.loginfo('Sign backend: YOLOv5s TensorRT, model=%s, full-frame 640',rospy.get_param('~model'))
        self.bridge, self.latest, self.lock = CvBridge(), None, threading.Lock()
        self.last = -1.0
        self.rear_latest, self.rear_last, self.rear_requested = None, -1., False
        self.timeout = cfg['sign_timeout']
        self.threshold = cfg['sign_confidence']
        self.min_height_ratio = cfg.get('sign_min_height_ratio',.125)
        route_settings = cfg.get('sign_route_gate', {})
        self.route_gate = (RouteSignGate(route_settings, self.threshold, self.min_height_ratio)
                           if route_settings.get('enabled', False) else None)
        # Rear P arms on the first confident fresh frame, at any image size
        # or position. Front route filtering and voting remain independent.
        self.pub = rospy.Publisher('/competition/sign',String,queue_size=1)
        self.rear_pub = rospy.Publisher('/competition/rear_sign',String,queue_size=1)
        rospy.Subscriber(cfg['front_image_topic'],Image,self.callback,queue_size=1,buff_size=2**22)
        if reverse_after_turn_enabled(cfg):
            rospy.Subscriber('/rear/usb_cam/image_raw',Image,self.rear_callback,queue_size=1,buff_size=2**22)
            rospy.Subscriber('/competition/status',String,self.status,queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0/cfg['sign_hz']),self.infer)

    def callback(self,msg):
        with self.lock:
            self.latest = msg

    def rear_callback(self,msg):
        with self.lock:
            self.rear_latest = msg

    def status(self,msg):
        try:
            data=json.loads(msg.data)
            requested=(data.get('state')=='PARKING' and
                       (data.get('parking_entry') or {}).get('camera_source')=='rear')
            with self.lock:
                self.rear_requested=requested
        except (ValueError,AttributeError,TypeError):
            return

    def infer(self,event):
        with self.lock:
            rear=getattr(self,'rear_requested',False)
            msg = getattr(self,'rear_latest',None) if rear else self.latest
        if msg is None:
            return
        stamp = msg.header.stamp.to_sec()
        last=getattr(self,'rear_last',-1.) if rear else self.last
        if stamp <= last or not 0 <= rospy.Time.now().to_sec()-stamp <= self.timeout:
            return
        if rear:self.rear_last=stamp
        else:self.last=stamp
        try:
            started = time.time()
            frame=self.bridge.imgmsg_to_cv2(msg,'bgr8')
            route_gate=None if rear else getattr(self,'route_gate',None)
            scores,top_label,confidence,bounds = {},'',0.0,None
            detections=sorted(self.detector.detect(frame),
                key=lambda d:(d['bounds'][2]*d['bounds'][3],d['confidence']),reverse=True)
            if rear:
                detections=[d for d in detections if self.normalize(d['label']) in ('PARKING','RED')]
            eligible_detections=(route_gate.filter(detections,frame.shape)
                                 if route_gate is not None else detections)
            selected=select_detection(eligible_detections,self.threshold)
            if selected is None and detections:
                # Preserve the raw prediction when the route gate rejects it.
                selected=select_detection(detections,self.threshold)
            diagnostics=dict(candidate_count=len(detections),selection='largest_detected_area',
                regions=[dict(bounds=d['bounds'],reason='#%d %s %.2f' % (i+1,d['label'],d['confidence']))
                         for i,d in enumerate(detections)])
            if selected is not None:
                bounds=selected['bounds']
                scores=dict((self.normalize(k),v) for k,v in selected['scores'].items())
                top_label=self.normalize(selected['label']);confidence=selected['confidence']
            label=top_label if confidence>=self.threshold and top_label!='BACKGROUND' else ''
            # Height survives partial clipping at the left/right image edge.
            # Keep all candidates for visibility/rearm even when too small to act on.
            height_ratio = float(bounds[3])/frame.shape[0] if bounds is not None else None
            too_small = (not rear and top_label in ('LEFT','RIGHT','STRAIGHT','UTURN','PARKING') and
                         height_ratio is not None and
                         height_ratio < getattr(self,'min_height_ratio',.125))
            if too_small:
                label=''
            info = dict(stamp=stamp,label=label,confidence=confidence,
                scores=scores,top_label=top_label,threshold=self.threshold,
                source='yolov5s',input_mode='full_frame_yolo',startup_only=False,
                range_reason='too_small' if too_small else 'full_frame_yolo' if selected is not None else 'no_candidate',
                sign_height_ratio=height_ratio,
                sign_min_height_ratio=0. if rear else getattr(self,'min_height_ratio',.125),
                sign_depth_m=None,sign_bounds=bounds,roi_diagnostics=diagnostics,
                roi_ms=0.,inference_ms=1000*(time.time()-started),
                source_age_s=rospy.Time.now().to_sec()-stamp,detections=detections)
            if route_gate is not None:
                label,gate_reason,gate_info=route_gate.evaluate(
                    top_label,confidence,bounds,frame.shape,stamp)
                info.update(label=label,range_reason=gate_reason,route_gate=gate_info)
                info['route_detections']=eligible_detections
            if rear and label not in ('PARKING','RED'):
                label='';info['label']=''
            if rear:
                info.update(route_gate=dict(enabled=False),
                    range_reason='rear_direct' if label else 'low_confidence_or_unknown')
            info['camera_source']='rear' if rear else 'front'
            publisher=self.rear_pub if rear else self.pub
            publisher.publish(String(data=json.dumps(info,allow_nan=False)))
        except Exception as exc:
            rospy.logwarn_throttle(2.0,'sign inference rejected: %s',str(exc))


if __name__ == '__main__':
    rospy.init_node('competition_sign')
    Node()
    rospy.spin()
