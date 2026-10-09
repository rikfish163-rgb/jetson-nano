#!/usr/bin/env python2
"""Stationary integration: private ROS master, no base driver or hardware nodes.

Requires a sourced Nano ROS environment. Nonzero samples stay on this private
graph and have only this verifier as subscriber; no chassis is started.
"""
from __future__ import print_function
import json
import os
import signal
import socket
import subprocess
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 11329


def main():
    probe = socket.socket()
    probe.bind(('127.0.0.1', PORT))
    probe.close()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % PORT,
                      ROS_HOSTNAME='127.0.0.1', ROS_HOME='/tmp/competition-interface-ros',
                      PYTHONDONTWRITEBYTECODE='1')
    import rosgraph
    import rospy
    from std_msgs.msg import String
    from ackermann_msgs.msg import AckermannDriveStamped
    children = []

    def launch(args, name):
        with open('/tmp/competition-interface-'+name+'.log', 'w') as log:
            child = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                     preexec_fn=os.setsid)
        children.append(child)
        return child

    def stop(child):
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            deadline = time.time()+5
            while child.poll() is None and time.time() < deadline:
                time.sleep(.05)
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGTERM)
            child.wait()

    def rejects(child):
        deadline = time.time()+10
        while child.poll() is None and time.time() < deadline:
            time.sleep(.05)
        assert child.poll() not in (None, 0), 'duplicate publisher was accepted'

    try:
        launch(['roscore', '-p', str(PORT)], 'master')
        master = rosgraph.Master('/interface_verifier')
        for _ in range(100):
            try:
                master.getPid()
                break
            except Exception:
                time.sleep(.1)
        rospy.init_node('interface_verifier', disable_signals=True)
        main_path = os.path.join(ROOT, 'src/robot/master/main.py')
        bridge_path = os.path.join(ROOT, 'src/ros/manual/scripts/testloop.py')
        central = launch(['python2', main_path], 'main')
        command = json.loads(rospy.wait_for_message('/control/cmd', String, timeout=15).data)
        assert command['speed_raw'] == 0 and 'stamp' in command
        rejects(launch(['python2', main_path, '__name:=duplicate_main'], 'duplicate-main'))
        bridge = launch(['python2', bridge_path, '_require_stamp:=true'], 'bridge')
        output = rospy.wait_for_message('/ackermann_cmd', AckermannDriveStamped, timeout=15)
        assert output.drive.speed == 0
        rejects(launch(['python2', bridge_path, '__name:=duplicate_bridge'], 'duplicate-bridge'))
        subscribers = dict(master.getSystemState()[1])
        assert set(subscribers.get('/ackermann_cmd', [])) <= set(['/main_node', '/interface_verifier'])
        print('PASS stamped main -> bridge; both duplicate owners rejected; no base subscriber')
        received = []
        sub = rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped,
                               lambda msg: received.append(msg.drive.speed), queue_size=10)
        joystick = rospy.Publisher('/joystick/control_cmd', String, queue_size=1)
        deadline = time.time()+3
        while joystick.get_num_connections() < 2 and time.time() < deadline:
            time.sleep(.05)
        for seq in range(5):
            joystick.publish(String(data=json.dumps(dict(version=1, seq=seq, enabled=True,
                stamp=rospy.Time.now().to_sec(), speed_raw=7, steering_raw=0))))
            time.sleep(.05)
        time.sleep(.1)
        assert 7 in received, received
        time.sleep(.5)
        assert received[-1] == 0, received[-5:]
        status = json.loads(rospy.wait_for_message('/competition/status', String, timeout=5).data)
        assert status['estop'] is True
        print('PASS manual takeover, automatic latch, and disconnected manual stop')
        sub.unregister()
        stop(bridge)
        stop(central)
        launch(['python2', main_path, '__name:=shadow_main', '_live:=false'], 'shadow')
        rospy.wait_for_message('/competition/control_preview', String, timeout=15)
        pubs = dict(master.getSystemState()[0])
        assert not pubs.get('/control/cmd') and not pubs.get('/ackermann_cmd')
        print('PASS shadow mode registers no real command publisher')
    finally:
        for child in reversed(children):
            stop(child)


if __name__ == '__main__':
    main()
