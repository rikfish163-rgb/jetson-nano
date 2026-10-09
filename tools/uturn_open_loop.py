#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Timed U-turn experiment; no camera, blue line, sign or pose feedback."""
from __future__ import print_function
import argparse
import json
import math
import os
import sys
import time

ROOT=os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
import yaml
from robot.uturn.planner import plan_uturn
from robot.uturn.calibration import command_angle
from robot.uturn.calibration import limit
from robot.common.contracts import encode_command
from robot.common.geometry import bicycle
from robot.common.geometry import footprint


def right_turn_segment6(segments):
    if len(segments)<7 or segments[5]['speed']>=0 or segments[5]['steering']!=0:
        raise ValueError('segment 6 must be straight reverse followed by another segment')
    result=[dict(s) for s in segments]
    result[5]['steering']=-22
    return result


def replay_segments(segments,cfg,allowed):
    """Recheck the entire edited timed sequence; old endpoint is no longer valid."""
    pose=(0.,0.,0.)
    table=cfg['uturn_calibration']
    for s in segments:
        gear=1 if s['speed']>0 else -1
        fraction=s['steering']/float(cfg['steering_raw_limit'])/cfg['steering_sign']
        if gear<0:fraction*=table.get('reverse_steering_sign',1)
        angle=fraction*limit(cfg,gear,fraction)
        gain=table['forward_mps_per_raw' if gear>0 else 'reverse_mps_per_raw']
        travel=s['speed']*gain*s['seconds']
        count=max(1,int(math.ceil(abs(travel)/.025)))
        origin=pose
        for i in range(count+1):
            pose=bicycle(origin,travel*i/count,angle,cfg['wheelbase'])
            if not all(allowed(p) for p in footprint(pose,cfg,spacing=.025)):
                raise ValueError('edited sequence exceeds configured corridor')
    return pose


def extend_last_turn(path,cfg,speed,seconds,allowed):
    if not 0<=seconds<=1:raise ValueError('last-turn-extra must be 0..1 seconds')
    if seconds==0:return list(path)
    if not path or path[-1][3]!=1 or abs(path[-1][4])<1e-9:
        raise ValueError('last segment must be a forward turn for extension')
    distance=speed*cfg['uturn_calibration']['forward_mps_per_raw']*seconds
    count=max(1,int(math.ceil(distance/.025)))
    result=list(path)
    for i in range(1,count+1):
        pose=bicycle(path[-1][:3],distance*i/count,path[-1][4],cfg['wheelbase'])
        if not all(allowed(p) for p in footprint(pose,cfg,spacing=.025)):
            raise ValueError('tail extension exceeds configured corridor')
        result.append(tuple(pose)+(1,path[-1][4]))
    return result


def schedule(path,cfg,speed):
    """Convert each constant-curvature primitive to command and duration."""
    result=[]
    for a,b in zip(path,path[1:]):
        gear,steer=b[3],b[4]
        chord=math.hypot(b[0]-a[0],b[1]-a[1])
        curvature=abs(math.tan(steer)/cfg['wheelbase'])
        length=2*math.asin(min(1.,chord*curvature/2))/curvature if curvature>1e-9 else chord
        gain=cfg['uturn_calibration']['forward_mps_per_raw' if gear>0 else 'reverse_mps_per_raw']
        raw=encode_command(gear*speed,command_angle(cfg,gear,steer),cfg,0)['steering_raw']
        if raw not in (-22,0,22):raise ValueError('open-loop requires zero or full lock')
        seconds=length/(speed*gain)
        if seconds<=1e-9:continue
        if result and (result[-1]['speed'],result[-1]['steering'])==(gear*speed,raw):
            result[-1]['seconds']+=seconds
        else:result.append(dict(speed=gear*speed,steering=raw,seconds=seconds))
    if not result or sum(s['seconds'] for s in result)>60:
        raise ValueError('empty plan or motion exceeds 60 seconds')
    return result


def run_segments(segments,pause,hold):
    previous=0
    for i,segment in enumerate(segments):
        if previous*segment['speed']<=0:
            hold(0,segment['steering'],pause)
        print('segment %d/%d: %s'%(i+1,len(segments),segment))
        sys.stdout.flush()
        hold(segment['speed'],segment['steering'],segment['seconds'])
        previous=segment['speed']


def execute(segments,pause,evidence=None):
    import rospy
    import rosgraph
    from std_msgs.msg import String
    rospy.init_node('uturn_open_loop',anonymous=True,disable_signals=True)
    master=rosgraph.Master(rospy.get_name())
    def check_sources(own=False):
        publishers=dict(master.getSystemState()[0])
        allowed=[rospy.get_name()] if own else []
        if any(n not in allowed for n in publishers.get('/control/cmd',[])):
            raise RuntimeError('stop other /control/cmd publishers first')
        if publishers.get('/ackermann_cmd',[])!=['/ackermann_control_bridge']:
            raise RuntimeError('require only ackermann_control_bridge on /ackermann_cmd')
    check_sources()
    records=[]
    def record_status(msg):
        if len(records)<10000:
            records.append(dict(received_at=time.time(),data=msg.data))
    subscriber=rospy.Subscriber('/base_controller/status',String,record_status,queue_size=100)
    pub=rospy.Publisher('/control/cmd',String,queue_size=1)
    seq=[0]
    def send(speed,steer):
        pub.publish(String(data=json.dumps(dict(version=1,seq=seq[0],speed_raw=speed,steering_raw=steer))))
        seq[0]=(seq[0]+1)%256
    # Wall clock independent of ROS simulation time, including Python 2 Nano.
    clock=lambda:os.times()[4]
    def hold(speed,steer,seconds):
        deadline=clock()+seconds
        while clock()<deadline and not rospy.is_shutdown():
            if not pub.get_num_connections():raise RuntimeError('bridge disconnected')
            send(speed,steer)
            time.sleep(min(.05,max(0.,deadline-clock())))
        if rospy.is_shutdown():raise RuntimeError('ROS shutdown')
    try:
        deadline=clock()+2
        while not pub.get_num_connections() and clock()<deadline:time.sleep(.05)
        if not pub.get_num_connections():raise RuntimeError('no bridge subscriber')
        hold(0,0,1.)
        check_sources(True)
        run_segments(segments,pause,hold)
    finally:
        for unused in range(6):send(0,0);time.sleep(.05)
        subscriber.unregister()
        if evidence:
            with open(evidence,'w') as output:json.dump(records,output,indent=2)


def select_segments(segments,stop_after):
    if stop_after is None:return segments
    if not 1<=stop_after<=len(segments):raise ValueError('stop-after outside segment count')
    return segments[:stop_after]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shift',choices=('left','right'),default='left',help='relative to initial vehicle heading')
    parser.add_argument('--speed',type=int,default=26)
    parser.add_argument('--pause',type=float,default=.7)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--stop-after',type=int,help='stop after N segments for measured diagnosis')
    parser.add_argument('--last-turn-extra',type=float,default=0.,help='add 0..1 seconds to final forward turn only')
    parser.add_argument('--segment6-right',action='store_true',help='replace straight reverse in segment 6 with RAW -22')
    args=parser.parse_args()
    if not 1<=args.speed<=30 or not .3<=args.pause<=3:parser.error('speed 1..30, pause .3..3 seconds')
    if args.segment6_right and args.shift!='left':parser.error('segment6-right is for the left U-turn trial')
    config=os.path.join(ROOT,'src/robot/config')
    with open(os.path.join(config,'competition.yaml')) as f:cfg=yaml.safe_load(f)
    with open(os.path.join(config,'uturn_vision.yaml')) as f:cfg.update(yaml.safe_load(f))
    cfg['steering_command_scale_rad']=.1
    # Offline planning happens before connecting to ROS; allow more search time.
    cfg['planner_timeout']=30.
    cfg['planner_full_lock_only']=True
    cfg['planner_turn_direction']=1 if args.shift=='left' else -1
    spacing=cfg['uturn_lane_spacing']
    margin=cfg['uturn_boundary_margin_m']
    low,high=(-spacing-.3+margin,.3-margin) if args.shift=='right' else (-.3+margin,spacing+.3-margin)
    def allowed(p):
        return cfg['uturn_corridor_min_x_m']<=p[0]<=cfg['uturn_corridor_max_x_m'] and low<=p[1]<=high
    path,reason=plan_uturn((0,0,0),'LEFT' if args.shift=='right' else 'RIGHT',cfg,allowed)
    if not path:raise RuntimeError('planning failed: '+reason)
    nominal_endpoint=path[-1][:3]
    path=extend_last_turn(path,cfg,args.speed,args.last_turn_extra,allowed)
    segments=schedule(path,cfg,args.speed)
    full_endpoint=path[-1][:3]
    if args.segment6_right:
        segments=right_turn_segment6(segments)
        full_endpoint=replay_segments(segments,cfg,allowed)
    segments=select_segments(segments,args.stop_after)
    report=dict(version=4,shift=args.shift,turn_direction=args.shift,segments=segments,full_plan_predicted_endpoint=full_endpoint,
        segment6_right=args.segment6_right,
        stop_after=args.stop_after,last_turn_extra=args.last_turn_extra,
        nominal_plan_endpoint=nominal_endpoint,
        motion_seconds=sum(s['seconds'] for s in segments),pause_seconds=args.pause,
        calibration=cfg['uturn_calibration'],feedback=False)
    print(json.dumps(report,indent=2))
    if not args.execute:
        print('DRY RUN: no ROS publisher. Add --execute to move.');return
    folder=os.path.join(ROOT,'field_data','uturn_open_loop')
    if not os.path.isdir(folder):os.makedirs(folder)
    record_path=os.path.join(folder,'%s_%d.json'%(time.strftime('%Y%m%d_%H%M%S'),os.getpid()))
    with open(record_path,'w') as f:
        json.dump(report,f,indent=2)
    print('Base output record: '+record_path+'.status.json')
    execute(segments,args.pause,record_path+'.status.json')


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:sys.exit(130)
    except (ValueError,RuntimeError) as exc:print(str(exc));sys.exit(1)
