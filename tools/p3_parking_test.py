#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""P3 trial: automatic road lane approach, visual stop, then five reverse stages."""
from __future__ import print_function
import argparse
import sys
import parking_line_stop_test as approach_test
from open_loop_core import finite
from p3_lane_follow import P3LaneVision, P3LaneFollower
from p3_approach import P3LineVision, P3ApproachRun

class P3Run(object):
    """Wrap the saved visual stop; brake, settle, reverse once, then stop."""
    def __init__(self, approach, seconds=(1.5, .2, 3.4, .1, 2.1), reverse_speed=30,
                 parking_pause_seconds=.5, lane_follower=None):
        self.approach = approach
        self.lane_follower = lane_follower
        self.lane_started = False
        self.seconds = tuple(finite(v, 'step%d_seconds' % (i+1), .05, 10.)
                             for i, v in enumerate(seconds))
        if len(self.seconds) != 5:
            raise ValueError('P3 requires exactly five reverse durations')
        self.reverse_speed = finite(reverse_speed, 'reverse_speed', 1, 30, True)
        self.pause = finite(parking_pause_seconds, 'parking_pause_seconds', .3, 3.)
        self.finished, self.reason = False, 'READY'
        self.phase, self.deadline, self.index = 'APPROACH', 0., -1
        self.stages = [('P3_STEP1_REVERSE_STRAIGHT', 0),
                       ('P3_STEP2_REVERSE_LEFT', 22),
                       ('P3_STEP3_REVERSE_RIGHT', -22),
                       ('P3_STEP4_REVERSE_STRAIGHT', 0),
                       ('P3_STEP5_REVERSE_LEFT', 22)]

    def __getattr__(self, name):
        return getattr(self.approach, name)

    def start(self, now):
        self.approach.start(now)

    def observe(self, count, stamp, received, both_curved=False):
        if not self.finished and (self.phase == 'APPROACH' or
                (self.phase == 'HOLD' and getattr(self.approach,'requires_end_confirmation',False))):
            self.approach.observe(count, stamp, received, both_curved)

    def observe_lane(self, observation, now):
        if self.lane_follower is not None and not self.finished and self.phase == 'APPROACH':
            self.lane_follower.observe(observation, now)

    def observe_scene(self, curve):
        observer = getattr(self.approach, 'observe_scene', None)
        if observer is not None and self.phase in ('APPROACH','HOLD') and not self.finished:
            observer(curve)

    def diagnostics(self, now):
        diagnostic = (self.lane_follower.diagnostics(now) if self.lane_follower is not None
                      and hasattr(self.lane_follower, 'diagnostics') else {})
        diagnostic.update(bay_armed=getattr(self.approach,'bay_armed',False),
                          end_votes=getattr(self.approach,'end_votes',0))
        return diagnostic

    def abort(self, reason):
        self.finished, self.reason = True, reason
        self.phase = 'FINISHED'
        self.approach.abort(reason)
        return (0, 0)

    def tick(self, now, ros_now):
        if self.finished:
            return (0, 0)
        if self.phase == 'APPROACH':
            command = self.approach.tick(now, ros_now)
            if self.approach.finished and self.approach.reason != 'BAY_END_CONFIRMED':
                return self.abort(self.approach.reason)
            if self.approach.stop_candidate_since is None:
                self.reason = self.approach.reason
                if self.lane_follower is not None and command[0] > 0:
                    command = self.lane_follower.command(ros_now)
                    if command[0] > 0:
                        self.lane_started = True
                        self.reason = 'P3_LANE_' + self.approach.reason
                    else:
                        self.reason = 'P3_WAIT_LANE'
                return command
            if self.lane_follower is not None and not self.lane_started:
                return self.abort('NO_LANE_BEFORE_PARKING')
            # Brake on the first joint visual event. Validate while stationary;
            # an uncertain end must not drive onward or begin timed reversing.
            self.phase, self.reason = 'HOLD', 'P3_STOP_HOLD'
            self.deadline = now+self.pause
            return (0, 0)
        if self.phase == 'HOLD' and getattr(self.approach,'requires_end_confirmation',False):
            self.approach.tick(now,ros_now)
            if self.approach.finished and self.approach.reason != 'BAY_END_CONFIRMED':
                return self.abort(self.approach.reason)
            if not self.approach.finished:
                self.reason = 'P3_STOP_VERIFY_END'
                return (0,0)
            self.reason = 'P3_STOP_HOLD'
        if now < self.deadline:
            if self.phase == 'HOLD':
                return (0, 0)
            return (-self.reverse_speed, self.stages[self.index][1])
        # Start the next stage at the outgoing-command time. Late polling
        # must not silently skip a stage or shorten its configured duration.
        self.index += 1
        if self.index >= len(self.stages):
            return self.abort('P3_COMPLETE')
        self.phase = 'REVERSE'
        self.reason = self.stages[self.index][0]
        self.deadline = now+self.seconds[self.index]
        return (-self.reverse_speed, self.stages[self.index][1])


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--step1-seconds', type=float, default=1.5,
                        help='reverse speed -30, steering 0; default 1.5 seconds')
    parser.add_argument('--step2-seconds', type=float, default=.2,
                        help='reverse speed -30, steering +22; default 0.2 seconds')
    parser.add_argument('--step3-seconds', type=float, default=3.4,
                        help='reverse speed -30, steering -22; default 3.4 seconds')
    parser.add_argument('--step4-seconds', type=float, default=.1,
                        help='reverse speed -30, steering 0; default 0.1 seconds')
    parser.add_argument('--step5-seconds', type=float, default=2.1,
                        help='reverse speed -30, steering +22; default 2.1 seconds')
    parser.add_argument('--reverse-speed', type=int, default=30,
                        help='positive magnitude, emitted reverse speed is negative')
    parser.add_argument('--parking-pause-seconds', type=float, default=.5,
                        help='zero-speed hold between visual forward stop and reverse')
    parser.add_argument('--end-zero-frames', type=int, default=2,
                        help='stationary end confirmation frames after immediate brake')
    parser.add_argument('--end-zero-seconds', type=float, default=.1,
                        help='stationary end confirmation duration after immediate brake')
    # The shared straight-line test defaults to -3; P3 never uses that trim.
    cli = sys.argv[1:] if argv is None else list(argv)
    # A bend start can take longer than the straight-only test's 5 s seek.
    # Explicit user flags remain last and retain precedence.
    args = approach_test.arguments(['--steering', '0', '--seek-seconds', '15',
                                    '--max-seconds', '30'] + cli, parser=parser)
    if args.steering != 0:
        raise ValueError('P3 auto lane uses controller steering directly; remove --steering')
    finite(args.end_zero_frames, 'end_zero_frames', 2, 6, True)
    finite(args.end_zero_seconds, 'end_zero_seconds', .05, .5)
    P3Run(None, (args.step1_seconds, args.step2_seconds, args.step3_seconds, args.step4_seconds, args.step5_seconds),
          args.reverse_speed, args.parking_pause_seconds)
    return args


def main(argv=None):
    try:
        args = arguments(argv)
        print('P3 APPROACH: AUTO LANE, speed=%d side=%s.' % (args.speed, args.side))
        print('Steering follows the production lane controller directly; no steering compensation.')
        print('Seek bay lines for up to %.1fs after forward starts; total approach limit %.1fs.' %
              (args.seek_seconds,args.max_seconds))
        print('Missing/unreliable lane commands zero speed; it does not start reverse parking.')
        print('After bay lines have been seen: FIRST ZERO bay lines AND BOTH boundaries curved -> BRAKE.')
        print('Confirm the joint end WHILE STOPPED for %d fresh frames / %.2fs before reverse.' %
              (args.end_zero_frames,args.end_zero_seconds))
        print('Ambiguous end: remain stopped; no automatic forward resume or reverse.')
        print('Hold zero for at least %.2fs; reverse only after stationary end confirmation:' %
              args.parking_pause_seconds)
        for i, (seconds, steering) in enumerate(zip(
                (args.step1_seconds, args.step2_seconds, args.step3_seconds, args.step4_seconds, args.step5_seconds), (0, 22, -22, 0, 22))):
            print('STEP%d speed=%d steering=%+d seconds=%.3f' %
                  (i+1, -args.reverse_speed, steering, seconds))
        print('Then STOP. Runs once. SPACE / x / q / Ctrl+C = STOP.')
        print('No automatic obstacle avoidance or measured final parking pose.')
        if not args.execute and not args.observe:
            print('PREVIEW ONLY: no ROS node or vehicle commands.')
            return 0
        def make_task(approach):
            approach = P3ApproachRun(approach,args.end_zero_frames,args.end_zero_seconds)
            return P3Run(approach, (args.step1_seconds, args.step2_seconds, args.step3_seconds, args.step4_seconds, args.step5_seconds),
                         args.reverse_speed, args.parking_pause_seconds,
                         lane_follower=P3LaneFollower(approach_test.ROOT, args.speed))
        return approach_test.run(args, task_factory=make_task,
                                 node_name='p3_parking_test', status_topic='/parking_p3/status',
                                 lane_vision_factory=P3LaneVision,
                                 line_vision_factory=P3LineVision, lane_first=True)
    except (ValueError, RuntimeError, IOError, KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
