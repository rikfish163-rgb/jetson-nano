#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Forward RAW 30: straight 0.5 s, full right 0.5 s, then stop once."""
from __future__ import print_function
import argparse
import sys
from right_two_stage_test import Stage, PROFILE as RIGHT_PROFILE, execute, finite


PROFILE = dict(RIGHT_PROFILE, node_name='forward_right_test')


def make_stages(straight_seconds=.5, right_seconds=.5):
    straight_seconds = finite(straight_seconds, 'straight_seconds', .05, 10)
    right_seconds = finite(right_seconds, 'right_seconds', .05, 10)
    return [Stage('COUNTDOWN', 3.0, 0, 0),
            Stage('FORWARD_STRAIGHT', straight_seconds, 30, 0),
            Stage('FORWARD_FULL_RIGHT', right_seconds, 30, -22),
            Stage('FINAL_STOP', .7, 0, 0)]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='run once after a stopped 3-second countdown')
    parser.add_argument('--straight-seconds', type=float, default=.5,
                        help='straight duration, 0.05..10 seconds')
    parser.add_argument('--right-seconds', type=float, default=.5,
                        help='full-right duration, 0.05..10 seconds')
    args = parser.parse_args(argv)
    try:
        stages = make_stages(args.straight_seconds, args.right_seconds)
    except ValueError as exc:
        parser.error(str(exc))
    print('Straight %.3f s -> full right %.3f s -> stop; both stages RAW 30.' %
          (args.straight_seconds, args.right_seconds))
    print('No pause between motion stages. Right full lock is RAW -22.')
    print('Open loop; no green-sign wait, camera correction or obstacle detection.')
    print('Durations time outgoing commands, not measured wheel motion.')
    for row in stages:
        print('%-20s seconds=%.3f speed=%d steering=%d' % tuple(row))
    sys.stdout.flush()
    if not args.execute:
        print('PREVIEW ONLY: no ROS node or vehicle commands.')
        return 0
    try:
        return execute(stages, auto_start=True, **PROFILE)
    except (ValueError, RuntimeError, IOError, KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
