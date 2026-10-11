#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Read-only LaserScan web viewer; Python 2/3, no Flask/OpenCV required."""
from __future__ import print_function
import json
import math
import random
import threading
import time
try:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    from SocketServer import ThreadingMixIn
except ImportError:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn

import rospy
from sensor_msgs.msg import LaserScan

# Rectangles keep vehicle axes; only their origin moves to the line midpoint.
BOXES = [('1', (-0.7, 0.0, -0.5, -0.1)),
         ('2', (-1.4, -0.7, -0.5, -0.1))]


def fit_segment(points):
    """Orthogonal least squares and endpoints of the observed point segment."""
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    xx = sum((p[0]-cx)**2 for p in points)
    yy = sum((p[1]-cy)**2 for p in points)
    xy = sum((p[0]-cx)*(p[1]-cy) for p in points)
    angle = 0.5 * math.atan2(2*xy, xx-yy)
    ux, uy = math.cos(angle), math.sin(angle)
    projections = [(p[0]-cx)*ux + (p[1]-cy)*uy for p in points]
    lo, hi = min(projections), max(projections)
    ends = [[cx+v*ux, cy+v*uy] for v in (lo, hi)]
    residual = math.sqrt(sum((-(p[0]-cx)*uy+(p[1]-cy)*ux)**2
                             for p in points) / len(points))
    return dict(endpoints=ends, midpoint=[(ends[0][i]+ends[1][i])/2 for i in (0, 1)],
                length=hi-lo, rms=residual, count=len(points))


def detect_line(points, radius=1.0, tolerance=0.02, min_points=8,
                min_length=0.12, max_gap=0.10):
    """Bounded deterministic RANSAC, splitting disconnected collinear objects."""
    roi = [p for p in points if p[0] > 0 and p[1] < 0
           and p[0]*p[0]+p[1]*p[1] <= radius*radius]
    if len(roi) < min_points:
        return None
    rng = random.Random(0)
    best, best_score = None, (0, 0)
    for _ in range(160):
        a, b = rng.sample(roi, 2)
        dx, dy = b[0]-a[0], b[1]-a[1]
        length = math.hypot(dx, dy)
        if length < min_length/2:
            continue
        ux, uy = dx/length, dy/length
        hits = sorted(((p[0]*ux+p[1]*uy, p) for p in roi
                       if abs((p[0]-a[0])*uy-(p[1]-a[1])*ux) <= tolerance),
                      key=lambda row: row[0])
        groups = []
        for projection, point in hits:
            if not groups or projection-previous > max_gap:
                groups.append([])
            groups[-1].append(point)
            previous = projection
        for group in groups:
            if len(group) < min_points:
                continue
            segment = fit_segment(group)
            # Require every retained point to remain within the final fit band.
            e0, e1 = segment['endpoints']
            span = segment['length']
            if span < min_length:
                continue
            kept = [p for p in group if abs((p[0]-e0[0])*(e1[1]-e0[1]) -
                    (p[1]-e0[1])*(e1[0]-e0[0]))/span <= tolerance]
            if len(kept) < min_points:
                continue
            segment = fit_segment(kept)
            score = (segment['count'], segment['length'])
            if segment['length'] >= min_length and segment['rms'] <= tolerance and score > best_score:
                best, best_score = segment, score
    return best


def measure(points):
    line = detect_line(points)
    boxes = []
    if line is not None:
        mx, my = line['midpoint']
        for name, bounds in BOXES:
            x0, x1, y0, y1 = bounds
            # Half-open X intervals assign the shared x=-0.7 edge to box 1.
            count = sum(1 for x, y in points if x0 <= x-mx < x1
                        and y0 <= y-my <= y1)
            if name == '1':
                count += sum(1 for x, y in points if x-mx == x1 and y0 <= y-my <= y1)
            boxes.append(dict(name=name, relative=list(bounds),
                              bounds=[mx+x0, mx+x1, my+y0, my+y1], count=count))
    return dict(line=line, boxes=boxes,
                difference=boxes[0]['count']-boxes[1]['count'] if boxes else None)


PAGE = u'''<!doctype html><meta charset="utf-8">
<title>雷达直线与区域点数</title>
<style>body{background:#101820;color:#eee;font:16px sans-serif;text-align:center;margin:16px}
canvas{width:min(94vw,760px);height:auto;background:#071019}input{width:65px}
.panel{max-width:900px;margin:12px auto;padding:12px;background:#1b2935;border-radius:8px;line-height:1.7}
#details{white-space:pre-line;text-align:left} .muted{color:#a9bac8}</style>
<h2>雷达直线中点与双框统计</h2>
<label>显示半径（米） <input id="range" type="number" min="0.5" max="50" step="0.5" value="2"></label>
<p id="status">等待 /scan 数据…</p><canvas id="plot" width="800" height="800"></canvas>
<div class="panel" id="details">等待检测</div>
<div class="panel muted">雷达中心为红点；上方 +X（车前），左侧 +Y（车左），单位：米。<br>
黄色扇区：右前方、距雷达 ≤ 1 米的检测范围；黄色线段：检测结果；白点：可见线段端点的中点。<br>
矩形仅平移原点，不随直线旋转。框 1（蓝）：X [-0.7, 0]，Y [-0.3, -0.1]；
框 2（紫）：X [-1.4, -0.7]，Y [-0.3, -0.1]。共同边界归框 1。<br>
每帧独立检测：优先选择支持点最多的连续直线段；至少 8 点、长度 ≥ 0.12 米、拟合容差 0.02 米、点间距 ≤ 0.10 米。<br>
中点是检测范围内可见线段的中点，不一定是完整物体中点。未检出或数据超时不输出有效比较。</div>
<script>
const c=document.getElementById('plot'),g=c.getContext('2d'),s=document.getElementById('status'),details=document.getElementById('details');
const fmt=p=>'('+p[0].toFixed(3)+', '+p[1].toFixed(3)+')';
function draw(d){
 const r=Math.max(.5,Number(document.getElementById('range').value)||2),k=360/r;
 const pix=p=>[400-p[1]*k,400-p[0]*k];
 const line=(a,b)=>{g.beginPath();g.moveTo(...pix(a));g.lineTo(...pix(b));g.stroke()};
 g.clearRect(0,0,800,800);g.font='14px sans-serif';
 g.fillStyle='#eac54b18';g.beginPath();g.moveTo(400,400);g.arc(400,400,k,-Math.PI/2,0);g.closePath();g.fill();
 g.strokeStyle='#354554';g.fillStyle='#95a4b0';g.lineWidth=1;
 for(let i=1;i<=4;i++){g.beginPath();g.arc(400,400,360*i/4,0,Math.PI*2);g.stroke();g.fillText((r*i/4).toFixed(2)+' m',405,400-360*i/4+15)}
 line([-r,0],[r,0]);line([0,-r],[0,r]);
 g.fillStyle='#42edbb';for(const p of d.points){if(Math.hypot(p[0],p[1])<=r)g.fillRect(...pix(p).map(v=>v-1.5),3,3)}
 const fresh=d.age!==null&&d.age<=2;
 g.fillStyle='#ff625e';g.beginPath();g.arc(400,400,5,0,Math.PI*2);g.fill();
 s.textContent=d.age===null?'等待雷达数据':(fresh?'实时接收':'数据已中断，统计无效')+' | '+d.points.length+' 点 | '+d.age.toFixed(1)+' 秒前 | '+d.frame;
 s.style.color=fresh?'#42edbb':'#ffbb66';
 if(!fresh){details.textContent='等待新鲜雷达数据，暂不比较。';return}
 if(!d.line){details.textContent='右前方 1 米内未检测到满足条件的直线，暂不生成矩形或比较点数。';return}
 const l=d.line;g.strokeStyle='#ffda59';g.lineWidth=3;line(...l.endpoints);
 g.fillStyle='white';g.beginPath();g.arc(...pix(l.midpoint),5,0,Math.PI*2);g.fill();g.fillText('M '+fmt(l.midpoint),...pix(l.midpoint).map(v=>v+12));
 for(let i=0;i<d.boxes.length;i++){
  const b=d.boxes[i],[x0,x1,y0,y1]=b.bounds,[px,py]=pix([x1,y1]);
  g.strokeStyle=i?'#d999ff':'#65bdff';g.fillStyle=i?'#d999ff18':'#65bdff18';
  g.fillRect(px,py,(y1-y0)*k,(x1-x0)*k);g.strokeRect(px,py,(y1-y0)*k,(x1-x0)*k);
  g.fillStyle=g.strokeStyle;g.fillText('框 '+b.name+': '+b.count+' 点',px+5,py+20);
 }
 let compare=d.difference===0?'两框点数相同':(d.difference>0?'框 1 更多':'框 2 更多')+'，相差 '+Math.abs(d.difference)+' 点';
 details.textContent='直线中点 M（雷达坐标）：'+fmt(l.midpoint)+' m\\n端点：'+fmt(l.endpoints[0])+' → '+fmt(l.endpoints[1])+ ' m\\n线段长度：'+l.length.toFixed(3)+' m | 支持点：'+l.count+' | 拟合 RMS：'+(l.rms*1000).toFixed(1)+' mm\\n'+d.boxes.map(b=>'框 '+b.name+'：'+b.count+' 点 | 雷达坐标 X ['+b.bounds[0].toFixed(3)+', '+b.bounds[1].toFixed(3)+']，Y ['+b.bounds[2].toFixed(3)+', '+b.bounds[3].toFixed(3)+']').join('\\n')+'\\n比较：'+compare;
}
async function tick(){const ctl=new AbortController(),timer=setTimeout(()=>ctl.abort(),3000);try{const r=await fetch('/scan.json',{cache:'no-store',signal:ctl.signal});if(!r.ok)throw Error(r.status);draw(await r.json())}catch(e){s.textContent='连接中断，正在重试；统计无效';s.style.color='#ffbb66';g.clearRect(0,0,800,800);details.textContent='等待恢复连接，暂不比较。'}finally{clearTimeout(timer);setTimeout(tick,100)}}tick();
</script>'''.encode('utf-8')


class Viewer(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {'points': [], 'frame': '', 'stamp': None}
        self.received = None

    def observe(self, scan):
        points = []
        for i, value in enumerate(scan.ranges):
            if math.isnan(value) or math.isinf(value):
                continue
            if value <= 0 or not scan.range_min <= value <= scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            points.append([value * math.cos(angle), value * math.sin(angle)])
        measurements = measure(points)
        with self.lock:
            self.data = {'points': points, 'frame': scan.header.frame_id,
                         'stamp': scan.header.stamp.to_sec()}
            self.data.update(measurements)
            self.received = time.time()

    def snapshot(self):
        with self.lock:
            result = dict(self.data)
            result['age'] = None if self.received is None else max(0, time.time() - self.received)
        return json.dumps(result, allow_nan=False).encode('utf-8')


class Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def main():
    rospy.init_node('lidar_web_test')
    viewer = Viewer()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/':
                body, mime = PAGE, 'text/html; charset=utf-8'
            elif path == '/scan.json':
                body, mime = viewer.snapshot(), 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (IOError, OSError):
                pass

        def log_message(self, *args):
            pass

    port = int(rospy.get_param('~port', 8090))
    server = Server(('0.0.0.0', port), Handler)
    server.timeout = 0.5
    rospy.Subscriber(rospy.get_param('~scan_topic', '/scan'), LaserScan,
                     viewer.observe, queue_size=1)
    rospy.loginfo('Lidar viewer: http://<Nano LAN IP>:%d/', port)
    try:
        while not rospy.is_shutdown():
            server.handle_request()
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
