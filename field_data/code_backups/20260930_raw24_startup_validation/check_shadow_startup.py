from __future__ import print_function
import os
import socket
import subprocess
import time
import xmlrpclib
import yaml

from robot.common.contracts import validate_config

with open('/tmp/lane_raw24_startup_params.yaml') as stream:
    params = yaml.safe_load(stream)
cfg = {}
prefix = '/competition/config/'
for key, value in params.items():
    if not key.startswith(prefix):
        continue
    parts = key[len(prefix):].split('/')
    current = cfg
    for part in parts[:-1]:
        current = current.setdefault(part, {})
    current[parts[-1]] = value
validate_config(cfg)
print('Actual launch configuration validated: lane=%s curve=%s straight=%s action=%s' % (
    cfg['speed_raw']['lane'], cfg['lane_curve_speed_raw'],
    cfg['straight_speed_raw'], cfg['speed_raw']['action']))

sock = socket.socket()
sock.bind(('127.0.0.1', 0))
port = sock.getsockname()[1]
sock.close()
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
log = open('/tmp/lane_raw24_shadow_master.log', 'w')
master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log, stderr=log)
node = None
try:
    proxy = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
    for attempt in range(50):
        try:
            if proxy.getPid('/raw24_startup_check')[0] == 1:
                break
        except Exception:
            time.sleep(.2)
    else:
        raise RuntimeError('isolated master did not start')
    import rospy
    import rosgraph
    from robot.master.controller_node import Node
    rospy.init_node('raw24_startup_check', disable_signals=True)
    rospy.set_param('/competition/config', cfg)
    rospy.set_param('~live', False)
    rospy.set_param('~enabled', False)
    node = Node()
    time.sleep(.25)
    publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
    assert not node.live
    assert '/control/cmd' not in publishers
    assert '/ackermann_cmd' not in publishers
    assert '/competition/control_preview' in publishers
    print('Controller initialized and timers ran in shadow mode; no vehicle command publishers.')
finally:
    if node is not None:
        node.timer.shutdown()
        node.telemetry_timer.shutdown()
    try:
        rospy.signal_shutdown('startup check completed')
    except NameError:
        pass
    master.terminate()
    master.wait()
    log.close()
