#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""P2 trial: saved P3 lane/visual stop, one forward stage, then four reverse stages."""
from __future__ import print_function
import argparse
import sys
import parking_line_stop_test as common
from open_loop_core import finite
from p3_parking_test import P3Run
from p3_lane_follow import P3LaneVision,P3LaneFollower
from p3_approach import P3LineVision,P3ApproachRun


class P2Run(P3Run):
    def __init__(self,approach,seconds=(1.,.2,3.4,.1,2.1),reverse_speed=30,
                 parking_pause_seconds=.5,lane_follower=None,step1_speed=30):
        P3Run.__init__(self,approach,seconds,reverse_speed,parking_pause_seconds,lane_follower)
        self.step1_speed = finite(step1_speed,'step1_speed',1,30,True)
        self.stages = [('P2_STEP1_FORWARD_STRAIGHT',0),
                       ('P2_STEP2_REVERSE_LEFT',22),
                       ('P2_STEP3_REVERSE_RIGHT',-22),
                       ('P2_STEP4_REVERSE_STRAIGHT',0),
                       ('P2_STEP5_REVERSE_LEFT',22)]

    def abort(self,reason):
        return P3Run.abort(self,'P2_COMPLETE' if reason == 'P3_COMPLETE' else reason)

    def tick(self,now,ros_now):
        command = P3Run.tick(self,now,ros_now)
        if self.reason.startswith('P3_'):
            self.reason = 'P2_'+self.reason[3:]
        # Preserve all approach/confirmation/fault/stop transitions from P3.
        # Only replace the actual first timed stage's moving command.
        if not self.finished and self.phase == 'REVERSE' and self.index == 0 and command[0] < 0:
            return (self.step1_speed,0)
        return command


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for i,seconds in enumerate((1.,.2,3.4,.1,2.1)):
        parser.add_argument('--step%d-seconds' % (i+1),type=float,default=seconds,
                            help='step %d duration; default %.1f seconds' % (i+1,seconds))
    parser.add_argument('--step1-speed',type=int,default=30,
                        help='positive raw FORWARD speed for step 1; default 30')
    parser.add_argument('--reverse-speed',type=int,default=30,
                        help='positive magnitude; steps 2-5 emit negative speed')
    parser.add_argument('--parking-pause-seconds',type=float,default=.5)
    parser.add_argument('--end-zero-frames',type=int,default=2)
    parser.add_argument('--end-zero-seconds',type=float,default=.1)
    cli = sys.argv[1:] if argv is None else list(argv)
    args = common.arguments(['--steering','0','--seek-seconds','15','--max-seconds','30']+cli,parser=parser)
    if args.steering != 0:
        raise ValueError('P2 automatic lane steering has no fixed steering compensation')
    finite(args.end_zero_frames,'end_zero_frames',2,6,True)
    finite(args.end_zero_seconds,'end_zero_seconds',.05,.5)
    P2Run(None,tuple(getattr(args,'step%d_seconds' % i) for i in range(1,6)),
          args.reverse_speed,args.parking_pause_seconds,step1_speed=args.step1_speed)
    return args


def main(argv=None):
    try:
        args = arguments(argv)
        print('P2 APPROACH: same automatic lane and visual stop as P3; speed=%d side=%s.' % (args.speed,args.side))
        print('Seen bay lines then ZERO lines AND BOTH boundaries curved -> BRAKE.')
        print('Confirm while stopped: %d fresh frames / %.2fs; hold >=%.2fs before stages.' %
              (args.end_zero_frames,args.end_zero_seconds,args.parking_pause_seconds))
        print('Ambiguous end stays stopped; lane/camera/base/ownership gates remain active.')
        for i,steering in enumerate((0,22,-22,0,22)):
            print('STEP%d speed=%d steering=%+d seconds=%.3f' %
                  (i+1,args.step1_speed if i==0 else -args.reverse_speed,steering,
                   getattr(args,'step%d_seconds' % (i+1))))
        print('One shot then STOP. SPACE / x / q / Ctrl+C = STOP.')
        print('Manual P2 motion trial; no slot-ID verification or automatic obstacle avoidance.')
        if not args.execute and not args.observe:
            print('PREVIEW ONLY: no ROS node or vehicle commands.')
            return 0
        def factory(base):
            approach = P3ApproachRun(base,args.end_zero_frames,args.end_zero_seconds)
            return P2Run(approach,tuple(getattr(args,'step%d_seconds' % i) for i in range(1,6)),
                         args.reverse_speed,args.parking_pause_seconds,
                         P3LaneFollower(common.ROOT,args.speed),args.step1_speed)
        return common.run(args,task_factory=factory,node_name='p2_parking_test',
                          status_topic='/parking_p2/status',lane_vision_factory=P3LaneVision,
                          line_vision_factory=P3LineVision,lane_first=True,
                          success_reasons=('P2_COMPLETE',))
    except (ValueError,RuntimeError,IOError,KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: '+str(exc),file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
