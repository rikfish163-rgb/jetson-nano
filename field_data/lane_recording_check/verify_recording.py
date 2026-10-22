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
    import cv2
    import glob
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from robot.common.config import load_config
    rospy.init_node('legacy_restore_verify', disable_signals=True)
    cfg=load_config(ROOT+'/src/robot/config')
    cfg.update(lidar_enabled=False,wait_green=False,steering_command_scale_rad=.03)
    rospy.set_param('/competition/config',cfg)
    received = {}
    def capture(key):
        def callback(msg):
            received[key] = json.loads(msg.data)
        return callback
    subscriptions = [rospy.Subscriber(topic, String, capture(key), queue_size=1)
        for key,topic in [('calibration','/vision/lane_calibration'),
                          ('observation','/vision/lane_observation'),
                          ('target','/competition/lane_target'),
                          ('status','/competition/status')]]
    recorder = launch('recorder', ['rosrun','robocup_competition','record_run.py',
        '_root:='+HERE+'/runs'])
    camera = launch('shadow_camera', ['rosrun','yihan_ros_pkg','camera_yihan_web.py',
        '__name:=legacy_shadow_camera','_image_topic:=/legacy_verify/image',
        '_processing_hz:=12','_window_height:=40','_min_path_span:=0.15'])
    main = launch('shadow_main', ['rosrun','robocup_competition','main.py',
        '__name:=legacy_shadow_main','_live:=false','_enabled:=false'])
    publisher = rospy.Publisher('/legacy_verify/image', Image, queue_size=1)
    bridge = CvBridge()
    frame = cv2.imread(glob.glob(ROOT + '/field_data/sign_capture/20260929_191543_23760/1790680576.*_frame.png')[0])
    deadline = time.time()+15
    while time.time() < deadline:
        if camera.poll() is not None or main.poll() is not None:
            raise RuntimeError('shadow node exited; inspect shadow logs')
        msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = rospy.Time.now()
        publisher.publish(msg)
        if (set(received) == set(('calibration','observation','status','target')) and
                received['target'].get('selection') is not None):
            break
        time.sleep(.1)
    assert set(received) == set(('calibration','observation','status','target')), sorted(received)
    assert received['target']['selection'] is not None, received['target']
    time.sleep(1)
    os.killpg(recorder.pid,signal.SIGINT)
    recorder.wait()
    folder=sorted(glob.glob(HERE+'/runs/*'))[-1]
    summary=json.load(open(folder+'/summary.json'))
    assert summary['dropped_queue']==0 and summary['stopped_reason'] is None,summary
    for key in ('front','bev','target','observation','calibration'):
        assert summary['counts'].get(key,0)>0,summary
    assert glob.glob(folder+'/*_target.jpg')
    assert glob.glob(folder+'/*_front.jpg')
    assert glob.glob(folder+'/*_bev.jpg')
    print('RECORDING VERIFIED: '+json.dumps(summary,sort_keys=True))
    observation = received['observation']
    assert observation['diagnostic']['dashed_left']
    assert observation['confidence'] > .35
    assert len(observation['points']) >= 3
    assert all(p[1] < 0 for p in observation['points'])
    assert received['calibration']['source_file'].endswith('/camera_yihan_web.py')
    publishers = dict(server.getSystemState('/legacy_verify')[2][0])
    assert '/control/cmd' not in publishers, publishers.get('/control/cmd')
    assert '/ackermann_cmd' not in publishers, publishers.get('/ackermann_cmd')
    result = dict(passed=True, isolated_master_port=port,
        observation_received=True, status_received=True,
        camera_source_sha256=received['calibration']['source_sha256'],
        actuator_publishers=[], enabled=False, live=False,
        observation=observation)
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
