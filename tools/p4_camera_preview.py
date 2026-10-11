#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Read-only front-camera MJPEG preview. Loopback HTTP; no actuator publishers."""
from __future__ import print_function
import argparse
import json
import threading
import time
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from SocketServer import ThreadingMixIn
from urlparse import urlparse, parse_qs

PAGE = ur'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>P4 前摄实时预览</title><style>
body{margin:0;background:#10151d;color:#e8edf5;font:16px system-ui,sans-serif}
main{max-width:1440px;margin:auto;padding:24px}h1{font-size:25px;margin:0 0 10px}
.muted{color:#b4c1d2}.legend{display:flex;gap:20px;flex-wrap:wrap;margin:18px 0}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:18px}
section{background:#1a2330;border:1px solid #344156;border-radius:12px;padding:14px}
h2{font-size:17px;margin:0 0 12px}img{width:100%;height:auto;display:block;background:#080d13}
#status{padding:12px;background:#18283a;border-radius:8px;margin:14px 0;white-space:pre-wrap}
.green{color:#58ef88}.yellow{color:#ffeb63}.red{color:#ff7777}.blue{color:#61b5ff}
</style><main><h1>P4 前摄实时预览</h1>
<div class="muted">只观察图像，不发送速度或转向命令 · 约3帧/秒</div>
<div id="status">等待相机…</div>
<div class="legend"><span class="green">绿：确认的两条侧线</span><span class="yellow">黄：库底实线</span>
<span class="red">红点：库底中点</span><span class="blue">蓝：接近画面下沿阈值</span></div>
<div class="grid"><section><h2>原始前摄</h2><img src="/stream?view=raw"></section>
<section><h2>前摄识别标注（去畸变）</h2><img src="/stream?view=overlay"></section>
<section><h2>鸟瞰识别</h2><img src="/stream?view=bev"></section></div>
<p class="muted">未确认的库位不会画成已识别标注。库底线需要连接两条侧线的远端，不能把入口线或普通横线当作库底线。</p>
<script>const el=document.querySelector('#status');let wasFresh=false;
function reconnectImages(){document.querySelectorAll('img').forEach(img=>{
const view=new URL(img.src).searchParams.get('view');img.src='/stream?view='+view+'&retry='+Date.now();});}
document.querySelectorAll('img').forEach(img=>{img.onerror=()=>setTimeout(()=>{
img.src=img.src.split('&retry=')[0]+'&retry='+Date.now();},1500);});
async function update(){try{const r=await fetch('/status',{cache:'no-store'});const s=await r.json();
const d=s.scene||{};const fresh=s.age_s!==null&&s.age_s<1;
if(fresh&&!wasFresh)reconnectImages();wasFresh=fresh;
el.textContent=(fresh?'相机实时更新':'等待/图像已过期')+' | 侧线：'+(d.sides_valid?'已确认':'未确认')+
' | 库底线：'+(d.bottom_valid?'已确认':'未确认')+' | 图像年龄：'+(s.age_s===null?'—':s.age_s.toFixed(2)+'秒')+
'\n库底中点：'+(d.bottom_midpoint_m?d.bottom_midpoint_m.map(v=>v.toFixed(3)).join(', ')+'米':'—')+
' | 方向偏差：'+(typeof d.heading_deg==='number'?d.heading_deg.toFixed(1)+'°':'—')+
' | 库底行比例：'+(typeof d.bottom_row==='number'?d.bottom_row.toFixed(3):'—')+
(s.error?'\n'+s.error:'');}catch(e){wasFresh=false;el.textContent='预览连接已断开，正在重试；请保持 SSH 隧道和相机服务运行';}}
update();setInterval(update,700);</script></main></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=18884)
    parser.add_argument('--image-topic', default='/front/usb_cam/image_raw')
    parser.add_argument('--config', default=None)
    chosen = parser.parse_args()
    if not 1024 <= chosen.port <= 65535:
        parser.error('port must be in [1024,65535]')
    import cv2
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from p4_forward_test import arguments
    from p4_forward_vision import BayVision
    cfg_args = ['--config', chosen.config] if chosen.config else []
    vision = BayVision({}, arguments(cfg_args))
    cv2.setNumThreads(1)
    bridge = CvBridge()
    lock = threading.Lock()
    latest = [None]
    received_at = [time.time()]
    state = dict(jpegs={}, scene={}, updated=None, sequence=0, error='')
    rospy.init_node('p4_readonly_preview', anonymous=True, disable_signals=False)
    def image_received(msg):
        with lock:
            latest[0] = msg
            received_at[0] = time.time()
    subscriber = [rospy.Subscriber(chosen.image_topic, Image, image_received,
                                  queue_size=1, buff_size=2**22)]

    def reconnect_camera():
        # rospy does not restore registrations after a ROS master restart.
        # Only recreate this read-only subscriber; never restart the camera/base.
        import rosgraph
        master = rosgraph.Master(rospy.get_name())
        while not rospy.is_shutdown():
            time.sleep(3.)
            with lock:
                stale = time.time()-received_at[0] > 3.
            try:
                master.getPid()
                registrations = master.getSystemState()[1]
                registered = any(topic == subscriber[0].resolved_name and
                                 rospy.get_name() in nodes for topic, nodes in registrations)
                if not stale and registered:
                    continue
                subscriber[0].unregister()
                subscriber[0] = rospy.Subscriber(chosen.image_topic, Image, image_received,
                                                 queue_size=1, buff_size=2**22)
                print('Camera stream/registration lost: restored read-only subscription')
            except Exception as exc:
                with lock:
                    state['error'] = 'Camera reconnect: %s' % exc
    reconnect_thread = threading.Thread(target=reconnect_camera)
    reconnect_thread.daemon = True
    reconnect_thread.start()

    def worker():
        previous = -1.
        while not rospy.is_shutdown():
            with lock:
                msg = latest[0]
            if msg is not None:
                stamp = msg.header.stamp.to_sec()
                if stamp > previous and 0 <= rospy.Time.now().to_sec()-stamp <= .5:
                    previous = stamp
                    try:
                        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
                        unused, bev = vision.observe(frame)
                        frames = dict(raw=frame, overlay=vision.overlay(frame), bev=bev)
                        encoded = {}
                        for key, value in frames.items():
                            ok, jpg = cv2.imencode('.jpg', value, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                            if not ok:
                                raise ValueError('JPEG encode failed')
                            encoded[key] = jpg.tostring()
                        with lock:
                            state.update(jpegs=encoded, scene=vision.scene_data(), updated=time.time(),
                                         sequence=state['sequence']+1, error='')
                    except Exception as exc:
                        with lock:
                            state['error'] = str(exc)
            time.sleep(.30)
    thread = threading.Thread(target=worker)
    thread.daemon = True
    thread.start()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, content, kind):
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        def do_GET(self):
            request = urlparse(self.path)
            if request.path == '/':
                return self.reply(PAGE.encode('utf-8'), 'text/html; charset=utf-8')
            if request.path == '/status':
                with lock:
                    payload = dict(scene=state['scene'], sequence=state['sequence'], error=state['error'],
                                   age_s=None if state['updated'] is None else time.time()-state['updated'])
                return self.reply(json.dumps(payload), 'application/json')
            if request.path in ('/stream', '/snapshot'):
                view = parse_qs(request.query).get('view', ['overlay'])[0]
                if view not in ('raw', 'overlay', 'bev'):
                    self.send_error(400, 'unknown view')
                    return
                if request.path == '/snapshot':
                    with lock:
                        data = state['jpegs'].get(view)
                    if data is None:
                        self.send_error(503, 'waiting for camera')
                        return
                    return self.reply(data, 'image/jpeg')
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=p4frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last = -1
                try:
                    while not rospy.is_shutdown():
                        with lock:
                            data, sequence = state['jpegs'].get(view), state['sequence']
                        if data is not None and sequence != last:
                            self.wfile.write('--p4frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(data))
                            self.wfile.write(data)
                            self.wfile.write('\r\n')
                            self.wfile.flush()
                            last = sequence
                        time.sleep(.05)
                except (IOError, OSError):
                    return
                return
            self.send_error(404)

    class Server(ThreadingMixIn, HTTPServer):
        daemon_threads = True
        allow_reuse_address = True
    server = Server(('127.0.0.1', chosen.port), Handler)
    server.timeout = .5
    print('Read-only P4 preview on 127.0.0.1:%d' % chosen.port)
    try:
        while not rospy.is_shutdown():
            server.handle_request()
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
