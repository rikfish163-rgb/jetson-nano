#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Sensor-only parking producer. Never publishes a chassis/control command."""
import argparse
import json
import os
import threading
import time
from collections import deque
import cv2
import rospy
import rospkg
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot.common.config import read_mapping
from robot.common.contracts import number,decode,validate_config
from robot.parking.timed_core import validate_scene
from robot.parking.timed_vision import BayLineVision,both_boundary_curves,PairBuffer
from robot.parking.target_line import FramePoses,TargetLine,LineCoordinates,p_candidates
from robot.parking.sign_anchor import ParkingSignProjector


def processing_interval(hint, now):
    # Status is a scheduling hint, never a requirement for producing a scene.
    # Lost/delayed telemetry retains the existing active processing rate.
    if hint is None or not 0 <= now-hint['stamp'] <= 1.:
        return .2
    if (hint.get('state') == 'TIMED_PARKING' or
            any(hint.get(key) == 'PARKING'
                for key in ('action', 'pending', 'next_direction'))):
        return .2
    return 1.


def side_processing_active(hint,now):
    if hint is None or not 0 <= now-hint['stamp'] <= 1.:
        return False
    task = hint.get('parking_timed') or {}
    if hint.get('state') == 'TIMED_PARKING':
        return task.get('phase') in (None,'APPROACH')
    return hint.get('pending') == 'PARKING' and hint.get('parking_side_pending') is True


def side_run(hint):
    task = (hint or {}).get('parking_timed') or {}
    if task.get('select_by_lidar') is True and task.get('triggered_at') is not None:
        return number(task['triggered_at'])
    return None


def draw_target(debug,camera,target):
    color = (0,255,0) if target['visible'] is True else (0,140,255)
    if target['line'] is not None:
        pts = [(int(camera['origin_u']-p[1]*camera['pixels_per_m']),
                int(camera['origin_v']-p[0]*camera['pixels_per_m'])) for p in target['line']]
        cv2.line(debug,pts[0],pts[1],color,4)
    cv2.putText(debug,'LOCKED WHITE: '+target['reason'],(15,55),
                cv2.FONT_HERSHEY_SIMPLEX,.5,color,2)


class Node(object):
    def __init__(self):
        self.cfg = rospy.get_param('/competition/config')
        validate_config(self.cfg)
        if self.cfg.get('parking_mode') != 'timed_sequence' and not self.cfg.get('parking_side_lidar_enabled',False):
            raise ValueError('timed parking vision requires timed_sequence mode')
        if self.cfg.get('parking_side_lidar_enabled',False) and self.cfg.get('parking_mode')!='timed_sequence':
            rospy.loginfo('Automatic side parking uses fresh /competition/sign P disappearance; white producer idle')
            return
        self.options = self.cfg['parking_timed']
        cv2.setNumThreads(1)
        root = rospkg.RosPack().get_path('robocup_competition')
        projection = read_mapping(os.path.join(root,'parallel_parking/reference_camera.yaml'))
        camera = dict(self.cfg['front_camera'],H=projection['front_camera']['H'])
        parking_cfg = dict(self.cfg,front_camera=camera)
        self.vision = BayLineVision(parking_cfg,argparse.Namespace(**self.options))
        # Preserve the standalone bearing tracker; this producer uses one
        # farthest-before-P target in the shared production body frame instead.
        self.vision.p_reference_enabled = False
        self.projector = ParkingSignProjector(self.cfg)
        self.coordinates = LineCoordinates(camera,self.cfg['front_camera'])
        self.target = TargetLine(self.cfg)
        self.poses,self.signs = FramePoses(),deque(maxlen=20)
        self.last_target_event = None
        self.bridge,self.lock = CvBridge(),threading.Lock()
        self.buffer = PairBuffer()
        self.mission_hint = None
        self.stop = threading.Event()
        self.output = rospy.Publisher('/competition/parking_timed_scene',String,queue_size=1)
        self.debug = rospy.Publisher('/competition/debug/parking_timed',Image,queue_size=1)
        rospy.Subscriber(self.cfg['front_image_topic'],Image,self.image,queue_size=3,buff_size=2**22)
        rospy.Subscriber(self.cfg['lane_observation_topic'],String,self.lane,queue_size=3)
        rospy.Subscriber('/competition/parking_hint',String,self.status,queue_size=1)
        rospy.Subscriber('/competition/sign',String,self.sign,queue_size=1)
        rospy.on_shutdown(self.stop.set)
        self.worker = threading.Thread(target=self.work)
        self.worker.daemon = True
        self.worker.start()

    def image(self,msg):
        stamp = msg.header.stamp.to_sec()
        with self.lock:
            self.buffer.put('image',stamp,msg)

    def lane(self,msg):
        try:
            data,stamp = decode(msg.data,rospy.Time.now().to_sec(),self.cfg.get('ground_timeout',1.25))
            if data.get('frame') != self.cfg['lane_frame']:
                raise ValueError('lane frame mismatch')
            with self.lock:
                self.buffer.put('lane',stamp,data)
        except (ValueError,KeyError,TypeError) as exc:
            rospy.logwarn_throttle(2,'Timed parking lane rejected: %s',str(exc))

    def status(self,msg):
        try:
            data,stamp = decode(msg.data,rospy.Time.now().to_sec(),1.)
            hint = dict((key,data.get(key)) for key in
                        ('state','action','pending','next_direction','parking_side_pending','parking_timed'))
            hint['stamp'] = stamp
            side_run(hint)
            with self.lock:
                self.poses.update(data.get('pose_samples',[]))
                self.mission_hint = hint
        except (ValueError,KeyError,TypeError):
            return

    def sign(self,msg):
        try:
            data,stamp = decode(msg.data,rospy.Time.now().to_sec(),self.cfg['sign_timeout'])
            if data.get('camera_source','front') != 'front' or not p_candidates(data,self.cfg['sign_confidence']):
                return
            with self.lock:
                if not self.signs or stamp > self.signs[-1]['stamp']:
                    self.signs.append(data)
        except (ValueError,KeyError,TypeError):
            return

    def work(self):
        processed_at = -float('inf')
        while not self.stop.is_set() and not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            with self.lock:
                hint = getattr(self,'mission_hint',None)
                interval = processing_interval(hint,now)
                active = (not self.cfg.get('parking_side_lidar_enabled',False) or
                          self.cfg.get('parking_mode') == 'timed_sequence' or
                          side_processing_active(hint,now))
                pair = (self.buffer.newest(now,self.cfg.get('ground_timeout',1.25))
                        if active and time.time()-processed_at >= interval else None)
                run = side_run(hint) if active else None
                pose = self.poses.at(pair[0].header.stamp.to_sec()) if pair and run is not None else None
                sign_frames = ([(data,self.poses.at(data['stamp'])) for data in reversed(self.signs)
                                if 0 <= now-data['stamp'] <= self.cfg['sign_timeout']]
                              if (pair and run is not None and
                                  (run != self.target.run or self.target.sign_world is None)) else [])
            if pair is None or (run is not None and pair[0].header.stamp.to_sec() < run):
                self.stop.wait(.02)
                continue
            image,lane = pair
            try:
                started = time.time()
                processed_at = started
                stamp = image.header.stamp.to_sec()
                frame = self.bridge.imgmsg_to_cv2(image,'bgr8')
                self.vision.set_lane_observation(lane)
                count,debug = self.vision.observe(frame)
                curves = both_boundary_curves(lane.get('boundaries',{}),
                    self.options['curve_min_curvature'],self.options['curve_min_turn_deg'])
                data = dict(curves,source='timed_parking_front',frame='base_link',stamp=stamp,
                            lines=count,lane_observation=lane,
                            processing_ms=1000.*(time.time()-started))
                if run != self.target.run:
                    self.target.reset(run)
                if run is not None:
                    for sign,source_pose in sign_frames:
                        self.target.acquire_sign(sign,source_pose,pose,self.projector,self.vision.heading)
                        if self.target.sign_world is not None:
                            break
                    lines = [self.coordinates.to_body(line) for line in self.vision.lines]
                    target = self.target.observe(lines,stamp,pose,self.vision.heading)
                    data['target_line'] = target
                    data['stop_mode'] = 'target_white_disappearance'
                    data['lines_evaluated'] = True
                    if self.debug.get_num_connections():
                        drawn = dict(target,line=self.coordinates.to_parking(target['line'])
                                     if target['line'] is not None else None)
                        draw_target(debug,self.vision.camera,drawn)
                    event = (run,target['locked'],target['visible'],target['reason'])
                    if event != self.last_target_event:
                        self.last_target_event = event
                        rospy.loginfo('parking_target_line %s',json.dumps(target,allow_nan=False))
                data['processing_ms'] = 1000.*(time.time()-started)
                validate_scene(data,self.cfg,rospy.Time.now().to_sec())
                self.output.publish(String(data=json.dumps(data,allow_nan=False)))
                if self.debug.get_num_connections():
                    result = self.bridge.cv2_to_imgmsg(debug,'bgr8')
                    result.header = image.header
                    self.debug.publish(result)
            except (ValueError,KeyError,TypeError,cv2.error,CvBridgeError) as exc:
                rospy.logwarn_throttle(2,'Timed parking frame rejected: %s',str(exc))
            # Poll cheaply so a new parking hint raises the rate promptly.
            self.stop.wait(.02)


if __name__ == '__main__':
    rospy.init_node('parking_timed_vision')
    Node()
    rospy.spin()
