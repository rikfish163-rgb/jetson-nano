#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Read-only local dashboard. Images and status retain their ROS source stamps."""
from __future__ import division
import json
import threading
import time
import cv2
import rospy
from flask import Flask, Response, abort, jsonify
from werkzeug.serving import make_server
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# Debug JPEG work must not create an OpenCV worker pool beside vehicle vision.
cv2.setNumThreads(1)

TOPICS = {
    'sign_board_frame': '/debug/sign_board_frame',
    'sign_board_crop': '/debug/sign_board_crop',
    'sign_capture_roi': '/debug/sign_capture_roi',
    'sign_capture_frame': '/debug/sign_capture_frame',
    'front_raw': '/debug/front_raw',
    'warped_image': '/debug/warped_image',
    'metric_bev': '/debug/metric_bev',
    'lane_tracking': '/debug/lane_tracking',
    'front_camera': '/front/usb_cam/image_raw',
    'blue_mask': '/competition/debug/blue',
    'blue_warped': '/competition/debug/front_bev',
    'rear': '/debug/rear_bev',
}
app = Flask(__name__)
bridge = CvBridge()
frames, encoded_at = {}, {}
subscriptions, requested_at = {}, {}
STREAM_IDLE_SECONDS = 3.0
controller = {}
lock = threading.Lock()


def fresh(stamp, now, timeout=1.5):
    return stamp is not None and 0 <= now-stamp <= timeout


def request_image(name):
    # Only watched topics should make upstream nodes render debug images.
    with lock:
        requested_at[name] = time.time()
        if name not in subscriptions:
            subscriptions[name] = rospy.Subscriber(
                TOPICS[name], Image, make_callback(name), queue_size=1, buff_size=2**22)


def expire_streams(event):
    now = time.time()
    with lock:
        expired = [name for name in subscriptions
                   if now-requested_at.get(name, 0) > STREAM_IDLE_SECONDS]
        removed = [subscriptions.pop(name) for name in expired]
    for subscription in removed:
        subscription.unregister()


def make_callback(name):
    def callback(msg):
        now = time.time()
        with lock:
            if now-requested_at.get(name, 0) > STREAM_IDLE_SECONDS:
                return
            if 0 <= now-encoded_at.get(name, 0) < .2:
                return
            encoded_at[name] = now
        try:
            frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            if ok:
                with lock:
                    frames[name] = (msg.header.stamp.to_sec(), jpeg.tobytes())
        except (CvBridgeError, cv2.error):
            rospy.logwarn_throttle(2, 'viewer image conversion failed: %s', name)
    return callback


def status_callback(msg):
    if len(msg.data) > 131072:
        return
    try:
        data = json.loads(msg.data)
        if not isinstance(data, dict) or not isinstance(data.get('stamp'), (int, float)):
            return
        with lock:
            controller.clear()
            controller.update(data)
    except (ValueError, TypeError):
        return


@app.after_request
def headers(response):
    response.headers['Cache-Control'] = 'no-store'
    response.headers['X-Content-Type-Options'] = 'nosniff'
    response.headers['X-Frame-Options'] = 'DENY'
    return response


@app.route('/status')
def status():
    now = rospy.Time.now().to_sec()
    with lock:
        result = dict(controller)
        streams = {name: dict(topic=topic, fresh=fresh(frames.get(name, (None,))[0], now),
                    age_s=now-frames[name][0] if name in frames else None)
                   for name, topic in TOPICS.items()}
    return jsonify(controller=result, controller_fresh=fresh(result.get('stamp'), now, 1),
                   streams=streams)


@app.route('/frame/<name>')
def frame(name):
    if name not in TOPICS:
        abort(404)
    request_image(name)
    with lock:
        stamp, jpeg = frames.get(name, (None, None))
    if not fresh(stamp, rospy.Time.now().to_sec()):
        abort(503)
    return Response(jpeg, mimetype='image/jpeg')


def generate_stream(name):
    while not rospy.is_shutdown():
        request_image(name)
        with lock:
            stamp, jpeg = frames.get(name, (None, None))
        if fresh(stamp, rospy.Time.now().to_sec()):
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
        time.sleep(.2)


@app.route('/stream/<name>')
def stream(name):
    if name not in TOPICS:
        abort(404)
    return Response(generate_stream(name), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return u"""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>比赛实时调试</title>
<style>
body{margin:24px;background:#15222b;color:#eef4f5;font:16px sans-serif}
header{display:flex;gap:16px;align-items:center;flex-wrap:wrap}
#health{color:#ffbe70}#summary{white-space:pre-wrap;line-height:1.7}
main{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:18px}
article{background:#20323d;padding:14px;border-radius:8px}
h2{font-size:18px;margin:0 0 10px}img{width:100%;min-height:150px;object-fit:contain}
small{display:block;color:#bbccd2}.stale img{opacity:.18}
pre{overflow:auto;max-height:380px;font-size:13px}
</style>
<header><h1>比赛实时调试</h1><strong id="health">等待数据</strong></header>
<p>前摄 / 白线循线 / 正式蓝线检测。状态来自比赛主控；命令估算位姿不是实测里程计。</p>
<div id="summary"></div>
<p>路径俯视：绿色＝白线生成的循线目标；橙色＝当前实际跟踪的名义轨迹。后轴在红点，向上为车头方向；单位米。</p>
<canvas id="path" width="900" height="450" style="width:100%;max-width:900px;background:#0b151b"></canvas>
<main id="views"></main>
<details><summary>控制状态与触发依据</summary><pre id="details"></pre></details>
<script>
const names={front_camera:'前摄原图',metric_bev:'白线鸟瞰',
lane_tracking:'白线跟踪与循线目标',blue_mask:'正式蓝线检测掩膜',
blue_warped:'正式地面感知鸟瞰',rear:'后摄鸟瞰'};
const views=document.getElementById('views');
function drawPath(c,fresh){
 const ctx=document.getElementById('path').getContext('2d');ctx.clearRect(0,0,900,450);
 ctx.strokeStyle='#32434c';ctx.lineWidth=1;
 for(let x=50;x<900;x+=80){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,450);ctx.stroke();}
 for(let y=0;y<450;y+=80){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(900,y);ctx.stroke();}
 ctx.fillStyle='#ff6666';ctx.beginPath();ctx.arc(450,370,5,0,Math.PI*2);ctx.fill();
 if(!fresh)return;
 function line(rows,color){ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();
  (rows||[]).forEach((p,i)=>{const x=450-p[1]*160,y=370-p[0]*160;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}
 if(c.lane_path_fresh)line(c.lane_path_local,'#3bdf80');
 line(c.planned_path_local,'#ffbc59');
}
for(const [key,label] of Object.entries(names)){
 const a=document.createElement('article');a.id=key;a.className='stale';
 const h=document.createElement('h2');h.textContent=label;a.append(h);
 a.append(document.createElement('img'),document.createElement('small'));views.append(a);
}
async function poll(){
 try{
  const r=await fetch('/status',{cache:'no-store'});if(!r.ok)throw Error('status');
  const data=await r.json(),c=data.controller;
  drawPath(c,data.controller_fresh);
  document.getElementById('health').textContent=data.controller_fresh?
   (c.live?'实车命令模式':'影子模式'):'主控未运行或状态过期';
  document.getElementById('summary').textContent=data.controller_fresh?
   '状态：'+c.state+'  动作：'+(c.action||'无')+'  原因：'+c.reason+
   '  待执行：'+(c.pending||'无')+'  下个路口：'+(c.next_direction||'无')+
   '  速度/转向命令：'+JSON.stringify(c.command):'等待正式比赛主控状态';
  document.getElementById('details').textContent=JSON.stringify(c,null,2);
  for(const key of Object.keys(names)){
   const a=document.getElementById(key),s=data.streams[key];
   a.classList.toggle('stale',!s.fresh);
   a.querySelector('small').textContent=(s.fresh?'实时':'未收到或过期')+
      ' · '+(s.age_s===null?'无帧':s.age_s.toFixed(2)+' 秒')+' · '+s.topic;
   a.querySelector('img').src='/frame/'+key+'?t='+Date.now();
  }
 }catch(e){document.getElementById('health').textContent='连接中断';drawPath({},false);
  document.querySelectorAll('article').forEach(a=>{a.classList.add('stale');a.querySelector('img').removeAttribute('src');});
 }finally{setTimeout(poll,500);}
}poll();
</script></html>"""


def main():
    rospy.init_node('competition_viewer')
    expiry_timer = rospy.Timer(rospy.Duration(1), expire_streams)
    rospy.Subscriber('/competition/status', String, status_callback, queue_size=1)
    port = int(rospy.get_param('~port', 8080))
    if not 1024 <= port <= 65535:
        raise ValueError('viewer port must be 1024..65535')
    # Access remotely with SSH forwarding; no command or file-write endpoint.
    server = make_server('127.0.0.1', port, app, threaded=True)
    worker = threading.Thread(target=server.serve_forever)
    worker.daemon = True
    worker.start()
    try:
        rospy.spin()
    finally:
        expiry_timer.shutdown()
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)


if __name__ == '__main__':
    main()
