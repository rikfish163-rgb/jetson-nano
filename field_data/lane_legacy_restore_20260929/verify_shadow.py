#!/usr/bin/env python2
"""Start only vision and disabled main on an isolated localhost ROS master."""
from __future__ import print_function
import json
import os
import signal
import socket
import subprocess
import time
import xmlrpclib

ROOT = '/home/nano/robocup_ws'
HERE = os.path.dirname(os.path.abspath(__file__))
processes, logs = [], []
sock = socket.socket()
sock.bind(('127.0.0.1', 0))
port = sock.getsockname()[1]
sock.close()
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)


def launch(name, args):
    stream = open(os.path.join(HERE, name+'.log'), 'w')
    logs.append(stream)
    process = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT,
                               preexec_fn=os.setsid)
    processes.append(process)
    return process


try:
    master = launch('shadow_master', ['roscore', '-p', str(port)])
    server = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    deadline = time.time()+10
    while True:
        try:
            assert server.getPid('/legacy_verify')[0] == 1
            break
        except Exception:
            if time.time() > deadline:
                raise RuntimeError('isolated master did not start')
            time.sleep(.1)
    import rospy
    import numpy as np
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from robot.common.config import load_config
    rospy.init_node('legacy_restore_verify', disable_signals=True)
    rospy.set_param('/competition/config', load_config(ROOT+'/src/robot/config'))
    received = {}
    def capture(key):
        def callback(msg):
            received[key] = json.loads(msg.data)
        return callback
    subscriptions = [rospy.Subscriber(topic, String, capture(key), queue_size=1)
        for key,topic in [('calibration','/vision/lane_calibration'),
                          ('observation','/vision/lane_observation'),
                          ('status','/competition/status')]]
    camera = launch('shadow_camera', ['rosrun','yihan_ros_pkg','camera_yihan_web.py',
        '__name:=legacy_shadow_camera','_image_topic:=/legacy_verify/image',
        '_processing_hz:=12','_window_height:=40','_min_path_span:=0.15'])
    main = launch('shadow_main', ['rosrun','robocup_competition','main.py',
        '__name:=legacy_shadow_main','_live:=false','_enabled:=false'])
    publisher = rospy.Publisher('/legacy_verify/image', Image, queue_size=1)
    bridge = CvBridge()
    deadline = time.time()+15
    while time.time() < deadline:
        if camera.poll() is not None or main.poll() is not None:
            raise RuntimeError('shadow node exited; inspect shadow logs')
        msg = bridge.cv2_to_imgmsg(np.zeros((360,640,3),np.uint8), encoding='bgr8')
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)
        if set(received) == set(('calibration','observation','status')):
            break
        time.sleep(.1)
    assert set(received) == set(('calibration','observation','status')), sorted(received)
    assert received['observation']['points'] == []
    assert received['calibration']['source_file'].endswith('/camera_yihan_web.py')
    publishers = dict(server.getSystemState('/legacy_verify')[2][0])
    assert '/control/cmd' not in publishers, publishers.get('/control/cmd')
    assert '/ackermann_cmd' not in publishers, publishers.get('/ackermann_cmd')
    result = dict(passed=True, isolated_master_port=port,
        observation_received=True, status_received=True,
        camera_source_sha256=received['calibration']['source_sha256'],
        actuator_publishers=[], enabled=False, live=False)
    with open(os.path.join(HERE,'shadow_result.json'),'w') as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps(result,sort_keys=True))
finally:
    for process in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
    deadline = time.time()+5
    while any(p.poll() is None for p in processes) and time.time()<deadline:
        time.sleep(.1)
    for process in processes:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
    for stream in logs:
        stream.close()
