#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Explicit supervised RAW probe, refuses to compete with another publisher."""
from __future__ import print_function
import argparse
import json
import time
import rosgraph
import rospy
from std_msgs.msg import String

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--speed',type=int,required=True)
parser.add_argument('--steering',type=int,default=0)
parser.add_argument('--seconds',type=float,default=2.0)
parser.add_argument('--confirm-wheels-safe',action='store_true',required=True)
args = parser.parse_args()
if abs(args.speed)>30 or abs(args.steering)>22 or not 0<args.seconds<=5:
    parser.error('probe limits: raw speed +/-30, steering +/-22, duration (0,5]')
rospy.init_node('competition_bench_probe',anonymous=True)
publishers,_,_ = rosgraph.Master(rospy.get_name()).getSystemState()
if dict(publishers).get('/control/cmd'):
    raise SystemExit('REFUSED: another /control/cmd publisher exists; stop it first')
actuator_sources = dict(publishers).get('/ackermann_cmd',[])
if actuator_sources != ['/ackermann_control_bridge']:
    raise SystemExit('REFUSED: /ackermann_cmd must have only /ackermann_control_bridge; stop keyboard/trial publishers')
pub = rospy.Publisher('/control/cmd',String,queue_size=1)
seq = [0]
def send(speed,steer):
    pub.publish(String(data=json.dumps(dict(version=1,seq=seq[0],speed_raw=speed,steering_raw=steer))))
    seq[0] = (seq[0]+1)%256
rospy.on_shutdown(lambda:send(0,0))
try:
    start = time.time()
    while pub.get_num_connections()==0 and time.time()-start<2 and not rospy.is_shutdown():
        time.sleep(0.05)
    if pub.get_num_connections()==0:
        raise SystemExit('no bridge subscriber')
    start = time.time()
    while time.time()-start<args.seconds and not rospy.is_shutdown():
        send(args.speed,args.steering); time.sleep(0.05)
finally:
    for _ in range(4):
        send(0,0); time.sleep(0.05)
