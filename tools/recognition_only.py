#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read-only front RGB recognition. No control publishers or ROS parameters written."""
from __future__ import print_function, division
import argparse
import json
import math
import os
import sys
import threading
import time
try:
    from BaseHTTPServer import HTTPServer, BaseHTTPRequestHandler
except ImportError:
    from http.server import HTTPServer, BaseHTTPRequestHandler
import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src/ros/signs/scripts'))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from sign_classifier_cv import SignClassifier
from sign_string_node import extract_sign_roi, normalize_model_label, SIGN_SEARCH_BOTTOM_RATIO
from robot.camera.vision import GroundDetector


class Recognizer(object):
    def __init__(self, config):
        self.cfg = config
        self.ground = GroundDetector(config)
        self.model = SignClassifier()

    def run(self, frame):
        roi = extract_sign_roi(frame)
        sign = dict(label='', confidence=0., reason='no_candidate_roi')
        if roi is not None:
            _, confidence, label = self.model.classify_sign(roi)
            sign = dict(label=normalize_model_label(label), confidence=float(confidence), reason='classified')
        bev = self.ground.bev(frame)
        detected, mask, _ = self.ground.detect(bev, include_slots=False)
        b = self.cfg['blue']
        ppm = self.cfg['front_camera']['pixels_per_m']
        candidates = []
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:20]:
            rect = cv2.minAreaRect(contour)
            thickness, length = sorted(rect[1])
            thickness, length = thickness/ppm, length/ppm
            box = cv2.boxPoints(rect)
            edges = [box[(i+1)%4]-box[i] for i in range(4)]
            edge = max(edges, key=np.linalg.norm)
            angle = abs(math.atan2(edge[1], edge[0]))
            angle = min(angle, abs(math.pi-angle))
            reasons = []
            if not b['thickness_min'] <= thickness <= b['thickness_max']: reasons.append('thickness')
            if not b['length_min'] <= length <= b['length_max']: reasons.append('length')
            if angle > b['angle_tolerance']: reasons.append('angle')
            kind = 'rejected' if reasons else 'junction' if length >= b['long_min'] else 'tick'
            x, y = self.ground.metric(*rect[0])
            candidates.append(dict(id=len(candidates), x=x, y=y, length_m=length,
                thickness_m=thickness, angle_deg=math.degrees(angle), kind=kind, rejected_by=reasons))
            color = (0, 200, 0) if kind == 'junction' else (0, 140, 255)
            cv2.drawContours(bev, [np.int32(box)], 0, color, 2)
            cv2.putText(bev, '%d:%s' % (len(candidates)-1, kind), tuple(np.int32(rect[0])),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
        front = frame.copy()
        cv2.line(front, (0, int(frame.shape[0]*SIGN_SEARCH_BOTTOM_RATIO)),
                 (frame.shape[1]-1, int(frame.shape[0]*SIGN_SEARCH_BOTTOM_RATIO)), (0, 140, 255), 2)
        cv2.putText(front, '%s %.3f %s' % (sign['label'], sign['confidence'], sign['reason']),
                    (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 255), 1)
        crop = np.zeros_like(front)
        if roi is not None:
            h, w = roi.shape[:2]
            scale = min(front.shape[1]/w, front.shape[0]/h)
            resized = cv2.resize(roi, (max(1,int(w*scale)), max(1,int(h*scale))))
            crop[:resized.shape[0], :resized.shape[1]] = resized
        tile = lambda img: cv2.resize(img, (480, 300))
        panel = np.vstack((np.hstack((tile(front), tile(crop))),
                           np.hstack((tile(bev), tile(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR))))))
        return dict(sign=sign, markers=detected['markers'], blue_candidates=candidates), panel


HTML = u'''<!doctype html><meta charset="utf-8"><title>独立识别</title>
<style>body{background:#eef1f4;color:#172536;font:16px sans-serif;margin:20px}img{max-width:100%}pre{white-space:pre-wrap}</style>
<h2>前摄独立识别（不控制车辆）</h2>
<p>左上：原图 / 橙线以下不搜索路标；右上：送入模型的候选区域；左下：鸟瞰候选；右下：蓝色掩膜。</p>
<p>junction=路口蓝线，tick=短蓝段，rejected=过滤；坐标单位米，以后轴为原点。此脚本不进行方向投票、保存或停车。</p>
<p id="age">等待图像</p><img id="img"><pre id="data"></pre>
<script>setInterval(async()=>{try{let r=await fetch('/state');let s=await r.json();
document.getElementById('age').textContent=s.source_stamp?'源图像年龄 '+(Date.now()/1000-s.source_stamp).toFixed(2)+' 秒（大于2秒视为过期）':'等待图像';
document.getElementById('data').textContent=JSON.stringify(s,null,2);
document.getElementById('img').src='/image?t='+Date.now();}catch(e){document.getElementById('age').textContent='连接中断';}},500);</script>'''.encode('utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/front/usb_cam/image_raw')
    parser.add_argument('--port', type=int, default=8767)
    parser.add_argument('--image', help='Offline image; writes annotated JPEG and JSON, no ROS needed')
    parser.add_argument('--output', default='/tmp/recognition-only')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    with open(os.path.join(ROOT, 'src/robot/config/competition.yaml')) as f:
        config = yaml.safe_load(f)
    detector = Recognizer(config)
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None: raise ValueError('Cannot read input image')
        result, panel = detector.run(frame)
        if not cv2.imwrite(args.output+'.jpg', panel): raise IOError('Cannot write image')
        with open(args.output+'.json', 'w') as f: json.dump(result, f, indent=2, allow_nan=False)
        print(json.dumps(result, allow_nan=False))
        return
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    rospy.init_node('recognition_only', disable_signals=True)
    lock = threading.Lock()
    shared = dict(latest=None, result=dict(status='waiting_for_image'), jpeg=None)
    def receive(msg):
        with lock: shared['latest'] = msg
    subscriber = rospy.Subscriber(args.topic, Image, receive, queue_size=1, buff_size=2**22)
    stop = threading.Event()
    def worker():
        last = None
        bridge = CvBridge()
        while not stop.is_set() and not rospy.is_shutdown():
            with lock: msg = shared['latest']
            if msg is not None and msg.header.stamp != last:
                last = msg.header.stamp
                try:
                    result, panel = detector.run(bridge.imgmsg_to_cv2(msg, 'bgr8'))
                    result.update(source_stamp=msg.header.stamp.to_sec(), processed_at=time.time())
                    ok, encoded = cv2.imencode('.jpg', panel, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    with lock:
                        shared['result'], shared['jpeg'] = result, encoded.tobytes() if ok else None
                    print(json.dumps(result, allow_nan=False))
                except Exception as exc:
                    with lock: shared['result'], shared['jpeg'] = dict(error=str(exc)), None
            stop.wait(.33)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *unused): pass
        def do_GET(self):
            route = self.path.split('?')[0]
            with lock:
                body, kind = ((HTML, 'text/html; charset=utf-8') if route == '/' else
                    (json.dumps(shared['result'], allow_nan=False).encode('utf-8'), 'application/json') if route == '/state' else
                    (shared['jpeg'], 'image/jpeg') if route == '/image' else (None, 'text/plain'))
            body = body if body is not None else b''
            self.send_response(200 if body else 404)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try: self.wfile.write(body)
            except IOError: pass
    server = HTTPServer(('127.0.0.1', args.port), Handler)
    thread = threading.Thread(target=worker)
    thread.daemon = True
    thread.start()
    print('Recognition only: http://127.0.0.1:%d' % args.port)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        stop.set()
        subscriber.unregister()
        server.server_close()


if __name__ == '__main__': main()
