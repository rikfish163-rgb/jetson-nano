#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Stationary camera/lidar inspection. Publishes diagnostics and images only."""
from __future__ import division
import json
import threading
import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from robot.lidar.scan import Scan
from robot.common.geometry import world
from robot.camera.vision import GroundDetector
from robot.common.planning import rank_parking_slots
from robot.parking.tracking import BayTracker
from robot.parking.partial_model import PartialBayModel
from robot.parking.partial_debug import draw_partial_diagnostics


def scan_ready(scan, now, timeout=.5):
    """Missing, old, future-dated or invalid scans never assert FREE."""
    ready = (scan is not None and 0 <= now-scan.stamp <= timeout
             and scan.valid_rays >= 30)
    return ready


class Inspector(object):
    def __init__(self):
        cv2.setNumThreads(1)
        with open(rospy.get_param('~config_file')) as stream:
            self.cfg = yaml.safe_load(stream)
        self.cfg['parking_mode'] = 'forward_white'
        self.cfg['lidar']['shape_filter'] = False
        self.detector = GroundDetector(self.cfg)
        self.algorithm = rospy.get_param('~algorithm', 'partial_model')
        if self.algorithm not in ('partial_model', 'legacy'):
            raise ValueError('algorithm must be partial_model or legacy')
        self.partial_model = PartialBayModel(
            self.cfg, lambda u, v: self.detector.metric(u - 360, v),
            1.0 / float(np.linalg.norm(np.subtract(
                self.detector.metric(1, 0), self.detector.metric(0, 0)))))
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = self.scan = None
        self.tracker = BayTracker()
        self.pub = rospy.Publisher('/parking_test/status', String, queue_size=1)
        self.images = {k: rospy.Publisher('/parking_test/'+k, Image, queue_size=1)
                       for k in ('bev', 'white', 'overlay')}
        rospy.Subscriber(rospy.get_param('~image_topic'), Image, self.image,
                         queue_size=1, buff_size=4*1024*1024)
        rospy.Subscriber('/scan', LaserScan, self.lidar, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(.25), self.tick)

    def image(self, msg):
        with self.lock:
            self.frame = msg

    def lidar(self, msg):
        try:
            scan = Scan(msg.ranges, msg.angle_min, msg.angle_increment,
                        msg.range_min, msg.range_max, (0,0,0), self.cfg['lidar'],
                        msg.header.stamp.to_sec())
            with self.lock:
                self.scan = scan
        except ValueError as exc:
            rospy.logwarn_throttle(3, 'parking inspection scan: %s', str(exc))

    def tick(self, unused):
        now = rospy.Time.now().to_sec()
        with self.lock:
            frame, scan = self.frame, self.scan
        stamp = frame.header.stamp.to_sec() if frame is not None else None
        result = dict(stamp=now, mode='stationary_recognition_only',
                      image_age_s=now-stamp if stamp is not None else None,
                      scan_age_s=now-scan.stamp if scan else None,
                      valid_rays=scan.valid_rays if scan else 0,
                      echo_points=len(scan.obstacles) if scan else 0,
                      candidates=[], reason='waiting_camera')
        if stamp is None or not 0 <= now-stamp <= 1.25:
            result['reason'] = 'waiting_camera' if stamp is None else 'stale_camera'
            tracks=self.tracker.snapshot(now,False)
            result.update(tracks=tracks,stable_candidates=[t for t in tracks if t['confirmed']])
            self.pub.publish(json.dumps(result)); return
        try:
            bev = self.detector.bev(self.bridge.imgmsg_to_cv2(frame, 'bgr8'), parking=True)
            hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
            white_cfg = self.cfg['white']
            white = cv2.inRange(hsv, np.array([0,0,white_cfg['v_min']]),
                               np.array([179,white_cfg['s_max'],255]))
            if self.algorithm == 'partial_model':
                detected, diagnostic = self.partial_model.detect(
                    white, self.detector.parking_valid)
                self.detector.slot_diagnostic = diagnostic
            else:
                detected = self.detector.detect_slots(white, u_offset=360)
            slots = [dict(s, id='candidate_%d' % (i+1), pose=[s['x'],s['y'],s['yaw']])
                     for i,s in enumerate(sorted(detected,key=lambda s:s['x']))]
            # Robot is stationary: pair the image with the latest scan after
            # image processing, instead of aging the entry-time scan in a queue.
            with self.lock:
                scan = self.scan
            observed_at = rospy.Time.now().to_sec()
            ready = scan_ready(scan, observed_at)
            result.update(stamp=observed_at, image_age_s=observed_at-stamp,
                          scan_age_s=observed_at-scan.stamp if scan else None,
                          valid_rays=scan.valid_rays if scan else 0,
                          echo_points=len(scan.obstacles) if scan else 0)
            rows = (rank_parking_slots((0,0,0),slots,scan,self.cfg) if ready else
                    [dict(s,occupancy='UNKNOWN',coverage=None) for s in slots])
            result.update(candidates=rows, geometry=self.detector.slot_diagnostic,
                          scan_ready=ready, image_stamp=stamp,
                          reason='no_visual_bay' if not slots else
                          'candidates_classified' if ready else 'waiting_fresh_scan')
            tracks=self.tracker.update(rows,stamp,observed_at)
            # Recheck radar against the smoothed CURRENT geometry. A held track
            # must remain UNKNOWN even if its last observed frame was FREE.
            fresh=[t for t in tracks if t['observed']]
            if ready and fresh:
                classified={t['id']:t for t in rank_parking_slots((0,0,0),fresh,scan,self.cfg)}
                tracks=[classified.get(t['id'],t) for t in tracks]
            else:
                for t in tracks:
                    t.update(occupancy='UNKNOWN',coverage=None)
            result.update(tracks=tracks,stable_candidates=[t for t in tracks if t['confirmed']],
                          tracking_reference='stationary_vehicle',
                          tracking_rules=dict(required_frames=3,window_frames=5,
                                              match_distance_m=.08,match_angle_deg=10,
                                              hold_seconds=1.0,fresh_seconds=.5))
            overlay = bev.copy()
            draw_partial_diagnostics(
                overlay, self.detector.slot_diagnostic,
                lambda u, v: self.detector.metric(u - 360, v))
            c = self.cfg['front_camera']
            def pixel(p):
                return (int(c['origin_u']+360-p[1]*c['pixels_per_m']),
                        int(c['origin_v']-p[0]*c['pixels_per_m']))
            if ready:
                for p in scan.obstacles:
                    cv2.circle(overlay,pixel(p),2,(0,120,255),-1)
            for s in tracks:
                color = {'FREE':(0,220,0),'OCCUPIED':(0,0,255),'UNKNOWN':(0,220,255)}[s['occupancy']]
                pts = [pixel(world(s['pose'],(x*s['length']/2,y*s['width']/2)))
                       for x,y in ((-1,-1),(1,-1),(1,1),(-1,1))]
                if s.get('bottom_inferred') or s.get('geometry_inferred') or not s.get('observed',True):
                    # Dashed inferred rectangle; solid segments are measured.
                    for j in range(4):
                        a,b=np.array(pts[j],float),np.array(pts[(j+1)%4],float)
                        count=max(2,int(np.linalg.norm(b-a)/8))
                        for k in range(0,count,2):
                            q=a+(b-a)*k/count
                            r=a+(b-a)*min(k+1,count)/count
                            cv2.line(overlay,tuple(q.astype(int)),tuple(r.astype(int)),color,2)
                    for a,b in (s.get('observed_sides',[]) if s.get('observed',True) else []):
                        cv2.line(overlay,pixel(a),pixel(b),color,3)
                    for p in (s.get('entrance',[]) if s.get('observed',True) else []):
                        cv2.circle(overlay,pixel(p),5,(255,255,0),-1)
                    if s.get('observed',True):
                        for p in s.get('entrance_dashes',[]):
                            cv2.circle(overlay,pixel(p),4,(255,255,0),-1)
                        if s.get('observed_back'):
                            a,b=s['observed_back']
                            cv2.line(overlay,pixel(a),pixel(b),color,3)
                else:
                    cv2.polylines(overlay,[np.array(pts,np.int32)],True,color,3)
                label=(s['id']+' '+s.get('relative_bay','unassigned').upper()+' '+s['occupancy']+
                       (' HELD' if not s.get('observed',True) else
                        ' CONFIRMED' if s.get('confirmed') else ' TENTATIVE'))
                cv2.putText(overlay,label,pixel(s['pose']),
                            cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
            for key,img in [('bev',bev),('white',white),('overlay',overlay)]:
                if self.images[key].get_num_connections():
                    msg = self.bridge.cv2_to_imgmsg(img,'mono8' if key=='white' else 'bgr8')
                    msg.header = frame.header
                    self.images[key].publish(msg)
        except (ValueError, cv2.error) as exc:
            result.update(reason='image_processing_error', error=str(exc))
            rospy.logwarn_throttle(3, 'parking inspection image: %s', str(exc))
        self.pub.publish(json.dumps(result))
        rospy.loginfo_throttle(3, 'parking_test %s bays=%d rays=%d echoes=%d',
                              result['reason'],len(result['candidates']),
                              result['valid_rays'],result['echo_points'])


if __name__ == '__main__':
    rospy.init_node('parking_inspect')
    Inspector()
    rospy.spin()
