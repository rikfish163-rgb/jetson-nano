#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Latest-frame ground processing: publish blue BEFORE optional bay search."""
import json
import threading
import time
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from robot.camera.vision import GroundDetector
from robot.common.contracts import number
from robot.common.contracts import validate_config


class Node(object):
    def __init__(self):
        self.cfg = rospy.get_param('/competition/config')
        validate_config(self.cfg)
        cv2.setNumThreads(1)
        self.detector, self.bridge = GroundDetector(self.cfg), CvBridge()
        self.lock = threading.Lock()
        self.latest = {'front':None,'rear':None}
        self.last = {'front':-1.0,'rear':-1.0}
        self.slot_last = {'front':-1.0,'rear':-1.0}
        self.front_cache = None
        self.slot_at, self.diag_at = -1.0, -1.0
        self.next_slot_source = 'front'
        self.parking_requested = False
        self.parking_selecting = False
        self.uturn_requested = False
        self.uturn_vision = None
        self.uturn_control_side = None
        self.uturn_control_stamp = -1.
        if self.cfg.get('uturn_vision_enabled',False):
            from robot.uturn.vision import VisionUturnScene
            self.uturn_vision = VisionUturnScene(self.cfg)
            self.uturn_scene_pub = rospy.Publisher('/competition/uturn_scene',String,queue_size=1)
            rospy.Subscriber(self.cfg['lane_observation_topic'],String,self.uturn_lane,queue_size=1)
        self.stats = dict(front_frames=0,slot_frames=0,stale_front=0,stale_rear=0,
                          blue_ms=0.0,slot_ms=0.0,blue_age=0.0)
        self.pub = rospy.Publisher('/competition/ground',String,queue_size=4)
        self.diag = rospy.Publisher('/competition/perception_status',String,queue_size=1)
        self.debug = {name:rospy.Publisher('/competition/debug/'+name,Image,queue_size=1)
                      for name in ('front_bev','blue','white','rear_white','rear_blue','parking_bev','parking_white','uturn_blue')}
        rospy.Subscriber(self.cfg['front_image_topic'],Image,self.front,queue_size=1,buff_size=2**22)
        if rospy.get_param('~start_rear',True):
            rospy.Subscriber('/debug/rear_bev',Image,self.rear,queue_size=1,buff_size=2**22)
        rospy.Subscriber('/competition/status',String,self.status,queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1.0/self.cfg['ground_hz']),self.tick)

    def front(self,msg):
        with self.lock:
            self.latest['front'] = msg

    def rear(self,msg):
        with self.lock:
            self.latest['rear'] = msg

    def status(self,msg):
        try:
            data = json.loads(msg.data)
            self.uturn_requested = data.get('pending')=='UTURN' or data.get('action')=='UTURN'
            if self.uturn_vision is not None:
                with self.lock:
                    self.uturn_control_side = {'left_boundary':'LEFT','right_boundary':'RIGHT'}.get(data.get('lane_source'))
                    self.uturn_control_stamp = number(data.get('stamp',-1.))
                    self.uturn_vision.requested = data.get('pending')=='UTURN'
                    self.uturn_vision.set_active(data.get('state')=='UTURN')
            previous_parking_requested = self.parking_requested
            queued_straight_entry = (
                self.cfg.get('parking_mode') == 'forward_center' and
                self.cfg.get('parking_entry_style') == 'S' and
                data.get('next_direction') == 'PARKING')
            self.parking_requested = (data.get('pending') == 'PARKING' or
                queued_straight_entry or
                data.get('state') in ('WAIT_SLOT','PLANNING','PARKING',
                                      'PARALLEL_PARKING','PARKING_SCAN',
                                      'AUTO_PLANNING'))
            self.parking_selecting = data.get('state') in ('PARKING_SCAN','AUTO_PLANNING')
            if self.parking_requested != previous_parking_requested:
                self.next_slot_source = 'front'
        except (ValueError,AttributeError):
            return

    def uturn_lane(self,msg):
        try:
            data = json.loads(msg.data)
            stamp = number(data['stamp'])
            if (data.get('frame') != self.cfg['lane_frame'] or
                    not 0 <= rospy.Time.now().to_sec()-stamp <= self.cfg['sensor_timeout']):
                return
            side = data.get('followed_boundary') if number(data['confidence']) >= self.cfg['lane_min_confidence'] else None
            with self.lock:
                if (self.uturn_control_side is not None and
                        abs(stamp-self.uturn_control_stamp)<=self.cfg['sensor_timeout']):
                    side = self.uturn_control_side
                self.uturn_vision.observe_lane(side,stamp)
        except (ValueError,KeyError,TypeError):
            return

    def fresh_stamp(self,msg,source):
        if msg is None:
            return None
        stamp = number(msg.header.stamp.to_sec())
        if not 0 <= rospy.Time.now().to_sec()-stamp <= self.cfg.get('ground_timeout',1.25):
            self.stats['stale_'+source] += 1
            return None
        return stamp

    def publish(self,data,msg,source,part):
        data.update(stamp=msg.header.stamp.to_sec(),source=source,part=part,frame='base_link')
        self.pub.publish(String(data=json.dumps(data,allow_nan=False)))

    def debug_image(self,name,img,msg):
        if self.debug[name].get_num_connections():
            output = self.bridge.cv2_to_imgmsg(img,'bgr8' if img.ndim == 3 else 'mono8')
            output.header = msg.header
            self.debug[name].publish(output)

    def tick(self,event):
        with self.lock:
            front, rear = self.latest['front'], self.latest['rear']
        try:
            stamp = self.fresh_stamp(front,'front')
            if stamp is not None and stamp > self.last['front']:
                self.last['front'] = stamp
                started = time.time()
                parking = (self.parking_requested and self.cfg.get('parking_enabled',True) and
                           self.cfg.get('parking_mode') != 'timed_sequence')
                blue_reference = (self.uturn_vision is not None and self.uturn_vision.blue_enabled and
                                  self.uturn_requested)
                wide = parking or blue_reference
                full_bev = self.detector.bev(self.bridge.imgmsg_to_cv2(front,'bgr8'),parking=wide)
                offset = 360 if wide else 0
                bev = full_bev[:self.cfg['front_camera']['bev_height'],offset:offset+self.cfg['front_camera']['bev_width']]
                data,blue,white = self.detector.detect(bev,include_slots=False)
                self.publish(data,front,'front','markers')
                if self.uturn_vision is not None and not parking:
                    vision_bev,vision_white,vision_blue=bev,white,blue
                    if blue_reference:
                        vision_bev=full_bev[:self.cfg['front_camera']['bev_height']]
                        hsv=cv2.cvtColor(vision_bev,cv2.COLOR_BGR2HSV)
                        b,w=self.cfg['blue'],self.cfg['white']
                        vision_blue=cv2.inRange(hsv,(b['h_min'],b['s_min'],b['v_min']),(b['h_max'],255,255))
                        vision_white=cv2.inRange(hsv,(0,0,w['v_min']),(179,w['s_max'],255))
                        self.debug_image('uturn_blue',vision_blue,front)
                    with self.lock:
                        scene = self.uturn_vision.update(vision_bev,vision_white,vision_blue,stamp)
                        self.stats['uturn_vision_reason'] = self.uturn_vision.reason
                        self.stats['uturn_vision_inliers'] = self.uturn_vision.motion.inliers
                        if self.uturn_vision.blue_landmarks is not None:
                            self.stats['uturn_blue_reason']=self.uturn_vision.blue_landmarks.reason
                            self.stats['uturn_blue_locked']=self.uturn_vision.blue_landmarks.landmarks is not None
                            self.stats['uturn_blue_correction_stamp']=self.uturn_vision.blue_landmarks.corrected_at
                    if scene is not None:
                        self.uturn_scene_pub.publish(String(data=json.dumps(scene,allow_nan=False)))
                slot_white = white
                if parking:
                    # Keep the whole bay view for either parking executor.
                    hsv = cv2.cvtColor(full_bev,cv2.COLOR_BGR2HSV)
                    w = self.cfg['white']
                    slot_white = cv2.inRange(hsv,(0,0,w['v_min']),(179,w['s_max'],255))
                    if self.cfg.get('parking_mode') in ('forward_white','forward_center'):
                        lines = self.detector.parking_lines(slot_white,u_offset=offset)
                        self.publish(dict(markers=[],slots=[],lines=lines),front,'front','parking_lines')
                    self.debug_image('parking_bev',full_bev,front)
                    self.debug_image('parking_white',slot_white,front)
                self.front_cache = (front,slot_white,offset)
                self.stats.update(front_frames=self.stats['front_frames']+1,
                                  blue_ms=(time.time()-started)*1000,
                                  blue_age=rospy.Time.now().to_sec()-stamp)
                for name,img in (('front_bev',bev),('blue',blue),('white',white)):
                    self.debug_image(name,img,front)
            now = rospy.Time.now().to_sec()
            if self.uturn_requested:
                stamp = self.fresh_stamp(rear,'rear')
                if stamp is not None and stamp > self.last['rear']:
                    self.last['rear'] = stamp
                    bev = self.bridge.imgmsg_to_cv2(rear,'bgr8')
                    data,blue,_ = self.detector.detect(bev,rear=True,include_slots=False)
                    self.publish(data,rear,'rear','markers')
                    self.debug_image('rear_blue',blue,rear)
            # Reverse parallel parking starts with a front frame and, when a
            # current rear frame is available, alternates the 8 Hz ground
            # budget at 4 Hz per camera.  Forward-plan parking stays
            # front-only; if a reverse rear frame is absent, keep publishing
            # the real front measurement rather than making up a rear pose.
            mode = self.cfg.get('parking_mode')
            parallel = mode in ('parallel_reverse', 'forward_plan')
            slot_hz = self.cfg.get('ground_hz',8.0) if parallel else self.cfg.get('slot_hz',2.0)
            if (self.cfg.get('parking_enabled',True) and self.parking_requested and
                    self.cfg.get('parking_mode') not in ('forward_center','timed_sequence') and
                    now-self.slot_at >= 1.0/slot_hz):
                source = None
                if mode == 'forward_plan':
                    source = 'front'
                elif parallel:
                    source = (self.next_slot_source if rear is not None
                              else 'front')
                else:
                    source = (self.next_slot_source if
                              (rear is not None and not self.parking_selecting and
                               self.cfg.get('parking_mode') != 'forward_white')
                              else 'front')
                if source == 'front' and self.front_cache is not None:
                    msg,white,offset = self.front_cache
                elif source == 'rear':
                    msg,white,offset = rear,None,0
                else:
                    msg,white = None,None
                    offset = 0
                stamp = self.fresh_stamp(msg,source) if source is not None else None
                if stamp is not None and stamp > self.slot_last[source]:
                    self.slot_at, self.slot_last[source] = now,stamp
                    started = time.time()
                    if white is None:
                        bev = self.bridge.imgmsg_to_cv2(msg,'bgr8')
                        _,_,white = self.detector.detect(bev,rear=True,include_slots=False)
                        self.debug_image('rear_white',white,msg)
                    slots = self.detector.detect_slots(white,rear=source == 'rear',u_offset=offset)
                    output=dict(markers=[],slots=slots)
                    if mode == 'forward_plan' and source == 'front':
                        # Keep partial bay recovery in the same exposure and
                        # timestamp as slot detection.  The scene builder
                        # ignores these lines until a complete target is
                        # already locked.
                        output['lines'] = self.detector.parking_lines(
                            white, u_offset=offset)
                    if self.cfg.get('parking_mode')=='forward_white' and source=='front':
                        output['slot_diagnostic']=self.detector.slot_diagnostic
                    self.publish(output,msg,source,'slots')
                    self.stats.update(slot_frames=self.stats['slot_frames']+1,
                                      slot_ms=(time.time()-started)*1000)
                    if mode == 'forward_plan':
                        self.next_slot_source = 'front'
                    elif parallel:
                        self.next_slot_source = ('rear' if source == 'front'
                                                 else 'front')
                    else:
                        self.next_slot_source = 'rear' if source == 'front' else 'front'
            if now-self.diag_at >= 1.0:
                self.diag_at = now
                self.diag.publish(String(data=json.dumps(self.stats,allow_nan=False)))
        except Exception as exc:
            rospy.logwarn_throttle(2.0,'ground observation rejected: %s',str(exc))


if __name__ == '__main__':
    rospy.init_node('competition_ground')
    Node()
    rospy.spin()
