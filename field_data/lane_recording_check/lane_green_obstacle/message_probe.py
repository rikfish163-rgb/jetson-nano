#!/usr/bin/env python2
"""Isolated ROS transport probe. No camera, lidar driver, bridge or base node."""
from __future__ import division, print_function
import os
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11329'
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME',None)
import json
import math
import socket
import subprocess
import time
import imp

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE,'../../..'))
sock = socket.socket()
try:
    if sock.connect_ex(('127.0.0.1',11329)) == 0:
        raise RuntimeError('probe port already occupied')
finally:
    sock.close()
log = open(os.path.join(HERE,'ros_probe.log'),'w')
master = subprocess.Popen(['roscore','-p','11329'],stdout=log,stderr=log)
try:
    import rospy
    from std_msgs.msg import String
    from sensor_msgs.msg import LaserScan
    from ackermann_msgs.msg import AckermannDriveStamped
    from robot.common.config import load_config
    time.sleep(2.)
    rospy.init_node('lane_message_probe',disable_signals=True)
    cfg = load_config(os.path.join(ROOT,'src/robot/config'))
    cfg.update(wait_green=True,lidar_enabled=True,parking_enabled=False,
               lane_curvature_preview=True,steering_command_scale_rad=.03,
               lookahead=.8,timed_bypass_enabled=False)
    rospy.set_param('/competition/config',cfg)
    rospy.set_param('~live',False)
    rospy.set_param('~enabled',True)
    adapter = imp.load_source('adapter_probe',os.path.join(ROOT,'src/robot/master/controller_node.py'))
    node = adapter.Node()
    # Its publisher was registered in shadow mode. Exercise enabled-cache logic
    # while keeping every command on /competition/control_preview.
    rospy.get_param_cached('~enabled',False)
    node.live = True
    commands = []
    def receive(msg):
        data = json.loads(msg.data)
        commands.append((time.time(),dict(data),node.core.reason))
        ack = AckermannDriveStamped()
        ack.header.stamp = rospy.Time.now()
        ack.drive.speed = data['speed_raw']
        ack.drive.steering_angle = data['steering_raw']
        node.applied_cb(ack)
    rospy.Subscriber('/competition/control_preview',String,receive,queue_size=100)
    class SlowStatus(object):
        def publish(self,msg): time.sleep(.35)
    node.status = SlowStatus()
    lane = rospy.Publisher(cfg['lane_observation_topic'],String,queue_size=1)
    sign = rospy.Publisher('/competition/sign',String,queue_size=1)
    ground = rospy.Publisher('/competition/ground',String,queue_size=1)
    scan_pub = rospy.Publisher(cfg['scan_topic'],LaserScan,queue_size=1)
    time.sleep(.5)
    started = time.time()
    due = dict(lane=0.,sign=0.,ground=0.,scan=0.)
    cache_disabled = False
    while time.time()-started < 17.:
        elapsed = time.time()-started
        now = rospy.Time.now().to_sec()
        if elapsed >= due['lane']:
            due['lane'] = elapsed+1./12
            confidence = .327 if 11.5 < elapsed < 11.67 else .9
            lane.publish(String(data=json.dumps(dict(stamp=now,frame=cfg['lane_frame'],
                confidence=confidence,points=[[.5,0.],[.7,0.],[.9,0.],[1.1,0.]],boundaries={}))))
        if elapsed >= due['sign'] and elapsed > .6:
            due['sign'] = elapsed+.4
            sign.publish(String(data=json.dumps(dict(stamp=now,label='GREEN',confidence=.99))))
        if elapsed >= due['ground']:
            due['ground'] = elapsed+.2
            ground.publish(String(data=json.dumps(dict(stamp=now,frame=cfg['lane_frame'],source='front',part='markers',markers=[],slots=[]))))
        if elapsed >= due['scan']:
            due['scan'] = elapsed+.125
            scan = LaserScan()
            scan.header.stamp = rospy.Time.from_sec(now-.1)
            scan.header.frame_id = cfg['scan_frame']
            scan.angle_min = -math.pi
            scan.angle_increment = 2*math.pi/3000
            scan.range_min,scan.range_max = .05,6.
            scan.time_increment = .1/2999
            scan.ranges = [float('inf')]*3000
            if 3. < elapsed < 3.8: scan.ranges[1500] = .2
            if 12. < elapsed < 13.: scan.ranges = [float('nan')]*3000
            scan_pub.publish(scan)
        disabled = 14. < elapsed < 14.7
        if disabled != cache_disabled:
            rospy.set_param('~enabled',not disabled)
            cache_disabled = disabled
        time.sleep(.01)
    node.timer.shutdown()
    node.telemetry_timer.shutdown()
    intervals = sorted(b[0]-a[0] for a,b in zip(commands,commands[1:]))
    origin = started
    windows = {}
    for name,lo,hi in [('before_green',0.,.4),('driving',2.,2.8),('obstacle',3.2,3.7),
                       ('lane_after_startup',10.5,11.4),('invalid_scan',12.2,12.8),
                       ('disabled',14.15,14.65),('resumed',15.,16.5)]:
        rows = [row for row in commands if lo <= row[0]-origin <= hi]
        windows[name] = dict(count=len(rows),speeds=sorted(set(row[1]['speed_raw'] for row in rows)),
                             reasons=sorted(set(row[2] for row in rows)))
    result = dict(commands=len(commands),max_interval_s=max(intervals),
                  p95_interval_s=intervals[int(.95*len(intervals))],
                  intervals_above_bridge_timeout=sum(t>.25 for t in intervals),
                  windows=windows,final_state=node.core.state,
                  output_topic=node.output.resolved_name,telemetry_delay_s=.35)
    with open(os.path.join(HERE,'message_probe.json'),'w') as out: json.dump(result,out,indent=2)
    print(json.dumps(result,indent=2))
    assert result['output_topic'] == '/competition/control_preview'
    assert result['intervals_above_bridge_timeout'] == 0
    for name in ('before_green','obstacle','invalid_scan','disabled'):
        assert windows[name]['speeds'] == [0],(name,windows[name])
    for name in ('driving','lane_after_startup','resumed'):
        assert windows[name]['speeds'] and min(windows[name]['speeds']) > 0,(name,windows[name])
    assert result['final_state'] == 'LANE'
finally:
    try: rospy.signal_shutdown('probe complete')
    except NameError: pass
    master.terminate()
    master.wait()
    log.close()
