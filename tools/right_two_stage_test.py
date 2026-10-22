#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""One-shot right-turn test from a manually positioned front axle on blue."""
from __future__ import print_function
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot', 'parallel_parking'))
from open_loop_core import Stage, finite
from parallel_open_loop_test import execute

PROFILE = dict(node_name='right_two_stage_test',
               command_topic='/right/two_stage_cmd',
               status_topic='/right/two_stage_status',
               bridge_node='/right_test_bridge',
               actuator_launch='right_test_actuators.launch')


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='execute once; default is offline preview')
    parser.add_argument('--countdown', type=float, default=3.0,
                        help='stopped countdown before motion, 2..10 seconds')
    parser.add_argument('--reverse-speed', type=int, default=30,
                        help='positive magnitude; emitted speed is negative, 1..30')
    parser.add_argument('--reverse-steering', type=int, default=0,
                        help='raw steering, -22..22')
    parser.add_argument('--reverse-seconds', type=float, default=1.0)
    parser.add_argument('--forward-speed', type=int, default=30,
                        help='positive raw speed, 1..30')
    parser.add_argument('--forward-steering', type=int, default=-22,
                        help='raw steering, -22..22; negative is right')
    parser.add_argument('--forward-seconds', type=float, default=3.0)
    return parser.parse_args(argv)


def make_stages(args):
    countdown = finite(args.countdown, 'countdown', 2, 10)
    reverse = finite(args.reverse_speed, 'reverse_speed', 1, 30, True)
    forward = finite(args.forward_speed, 'forward_speed', 1, 30, True)
    reverse_steer = finite(args.reverse_steering, 'reverse_steering', -22, 22, True)
    forward_steer = finite(args.forward_steering, 'forward_steering', -22, 22, True)
    reverse_s = finite(args.reverse_seconds, 'reverse_seconds', .05, 10)
    forward_s = finite(args.forward_seconds, 'forward_seconds', .05, 10)
    return [
        Stage('COUNTDOWN', countdown, 0, 0),
        Stage('REVERSE_STRAIGHT', reverse_s, -reverse, reverse_steer),
        Stage('FORWARD_RIGHT', forward_s, forward, forward_steer),
        Stage('FINAL_STOP', .7, 0, 0),
    ]


def main(argv=None):
    try:
        args = arguments(argv)
        stages = make_stages(args)
        print('Place front axle on the blue line and align the car manually.')
        print('TWO motion stages, no inter-stage pause, then stop; runs once.')
        print('Open loop: no sign/blue recognition, lane following or obstacle detection.')
        print('Durations time outgoing commands; they are not wheel-motion measurements.')
        print('%-20s %8s %10s %13s' %
              ('Stage', 'seconds', 'speed_raw', 'steering_raw'))
        for row in stages:
            print('%-20s %8.3f %10d %13d' % tuple(row))
        sys.stdout.flush()
        if not args.execute:
            print('PREVIEW ONLY: no ROS node or vehicle commands.')
            return 0
        return execute(stages, auto_start=True, **PROFILE)
    except (ValueError, RuntimeError, IOError, KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
