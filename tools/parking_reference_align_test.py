#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Align to the observed two-side midpoint and a fixed parking-row terminal line."""
from __future__ import print_function
import argparse
import json
import os
import sys
import time
import cv2
import numpy as np
import parking_line_stop_test as common
from open_loop_core import finite
from p3_lane_follow import P3LaneVision,P3LaneFollower
from parking_reference_align_core import dual_midline,ReferenceAlignRun
from robot.parallel_parking.reference_vision import ReferenceVision


class MidlineVision(P3LaneVision):
    def __init__(self,*args):
        P3LaneVision.__init__(self,*args)
        self.pixel_origin = np.asarray(self.lane.bev_point_to_vehicle_m(0,0))
        basis = np.column_stack((np.asarray(self.lane.bev_point_to_vehicle_m(1,0))-self.pixel_origin,
                                 np.asarray(self.lane.bev_point_to_vehicle_m(0,1))-self.pixel_origin))
        self.pixel_inverse = np.linalg.inv(basis)

    def observe(self,frame,stamp):
        result,debug = P3LaneVision.observe(self,frame,stamp)
        middle = dual_midline(result.get('boundaries',{}))
        result['dual_midline'] = middle
        if middle['valid']:
            result['lane_observation']['points'] = middle['points']
            if debug is not None:
                pixels = [tuple(map(int,np.dot(self.pixel_inverse,np.array([x,y])-self.pixel_origin)))
                          for x,y in middle['points']]
                for a,b in zip(pixels,pixels[1:]):
                    cv2.line(debug,a,b,(255,0,255),3)
                cv2.putText(debug,'MEASURED TWO-SIDE MIDPOINT',(12,55),
                            cv2.FONT_HERSHEY_SIMPLEX,.5,(255,0,255),1)
        return result,debug


class TerminalVision(object):
    def __init__(self,cfg,args):
        self.vision = ReferenceVision(cfg)
        self.stamp = None
        self.result = None
    def set_lane_observation(self,observation):
        self.stamp = observation['stamp']
    def observe(self,frame):
        self.result,debug = self.vision.observe(frame,self.stamp)
        return len(self.result['p1_lines']),debug
    def scene_data(self):
        return dict(fixed_reference=self.result)


def make_follower(args):
    follower = P3LaneFollower(common.ROOT,args.speed)
    # Instance-local copy only. Aim nearer than the production driving horizon
    # so the exit bend cannot pull the alignment target beyond the terminal line.
    follower.follower.cfg['lookahead'] = args.align_lookahead_m
    return follower


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-gap-m',type=float,default=.70,
                        help='front-axle perpendicular gap to fixed row-end line; default 0.70 m')
    parser.add_argument('--creep-speed',type=int,default=12)
    parser.add_argument('--align-lookahead-m',type=float,default=.65)
    parser.add_argument('--gap-tolerance-m',type=float,default=.06)
    parser.add_argument('--lateral-tolerance-m',type=float,default=.04)
    parser.add_argument('--heading-tolerance-deg',type=float,default=4.)
    parser.add_argument('--stop-margin-m',type=float,default=.04)
    parser.add_argument('--stable-seconds',type=float,default=.4)
    parser.add_argument('--record-dir',default=os.path.join(common.ROOT,'field_data/parking_reference_alignment'))
    cli = sys.argv[1:] if argv is None else list(argv)
    args = common.arguments(['--steering','0','--speed','16','--seek-seconds','15',
                             '--max-seconds','30']+cli,parser=parser)
    if args.steering != 0 or args.side != 'right':
        raise ValueError('test uses automatic lane steering and the right-hand parking row')
    for name,lo,hi,integer in (
            ('target_gap_m',.55,1.4,False),('creep_speed',1,args.speed,True),
            ('align_lookahead_m',.45,1.,False),
            ('gap_tolerance_m',.02,.10,False),('lateral_tolerance_m',.01,.10,False),
            ('heading_tolerance_deg',1.,10.,False),('stop_margin_m',0.,.10,False),
            ('stable_seconds',.2,1.,False)):
        finite(getattr(args,name),name,lo,hi,integer)
    if args.stop_margin_m > args.gap_tolerance_m:
        raise ValueError('stop-margin-m must not exceed gap-tolerance-m')
    return args


def save_result(args,task,error):
    if task is None or not args.execute:
        return
    if not os.path.isdir(args.record_dir):
        os.makedirs(args.record_dir)
    path = os.path.join(args.record_dir,'trial_%d_%d.json' % (int(time.time()),os.getpid()))
    data = dict(version=1,reference='fixed_right_row_terminal',
                result=task.reason,error=error,target_gap_m=args.target_gap_m,
                speed=args.speed,creep_speed=args.creep_speed,
                align_lookahead_m=args.align_lookahead_m,
                final_gap_m=task.reference.gap,final_pose=task.pose,
                note='camera estimates; physical ruler measurements required for calibration',
                observations=task.history)
    with open(path,'w') as stream:
        json.dump(data,stream,indent=2,allow_nan=False)
    print('TRIAL RECORD: '+path)


def main(argv=None):
    args,task,error = None,[None],None
    try:
        args = arguments(argv)
        print('ALIGNMENT ONLY: right parking-row terminal line, front axle gap %.2f m.' % args.target_gap_m)
        print('Same-frame observed left/right midpoint; heading/lateral correction; no steering trim.')
        print('Search speed %d; creep speed %d. Stop does NOT depend on line disappearance.' % (args.speed,args.creep_speed))
        print('Stationary success: gap +/-%.2fm, lateral +/-%.2fm, heading +/-%.1fdeg for %.2fs.' %
              (args.gap_tolerance_m,args.lateral_tolerance_m,args.heading_tolerance_deg,args.stable_seconds))
        print('Reference missing/ambiguous after lock -> zero speed; never switch to another bay divider.')
        print('No reverse parking or automatic obstacle avoidance. SPACE / x / q / Ctrl+C = STOP.')
        if not args.execute and not args.observe:
            print('PREVIEW ONLY: no ROS node or vehicle commands.')
            return 0
        def factory(base):
            follower = make_follower(args)
            task[0] = ReferenceAlignRun(base,follower,args.target_gap_m,args.creep_speed,
                args.gap_tolerance_m,args.lateral_tolerance_m,args.heading_tolerance_deg,
                args.stop_margin_m,args.stable_seconds)
            return task[0]
        return common.run(args,task_factory=factory,node_name='parking_reference_align_test',
                          status_topic='/parking_reference_align/status',
                          lane_vision_factory=MidlineVision,line_vision_factory=TerminalVision,
                          lane_first=True,success_reasons=('ALIGNMENT_COMPLETE',))
    except (ValueError,RuntimeError,IOError,KeyboardInterrupt) as exc:
        error = str(exc)
        print('STOP / NOT STARTED: '+error,file=sys.stderr)
        return 1
    finally:
        if args is not None:
            if error is not None and task[0] is not None and not task[0].finished:
                task[0].abort(error)
            try:
                save_result(args,task[0],error)
            except (ValueError,IOError,OSError) as exc:
                print('Record save failed: '+str(exc),file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())
