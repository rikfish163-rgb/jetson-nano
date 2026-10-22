#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""One-shot four-stage U-turn tuning from a manually placed blue-line start."""
from __future__ import print_function
import argparse
import copy
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
PACKAGE = os.path.join(ROOT, 'src', 'robot')
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(PACKAGE, 'parallel_parking'))
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.uturn.timed import TimedUturn
from open_loop_core import Stage, finite
from parallel_open_loop_test import execute

PROFILE = dict(node_name='uturn_four_stage_test',
               command_topic='/uturn/four_stage_cmd',
               status_topic='/uturn/four_stage_status',
               bridge_node='/uturn_test_bridge',
               actuator_launch='uturn_test_actuators.launch')


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=os.path.join(PACKAGE, 'config', 'maneuvers.yaml'),
                        help='maneuver YAML; read at every invocation')
    parser.add_argument('--execute', action='store_true',
                        help='run once after countdown; otherwise preview only')
    parser.add_argument('--countdown', type=float, default=3.0, help='stopped countdown, 2..10 seconds')
    parser.add_argument('--pause', type=float, help='start/gear-change/settle pause, 0.3..3 seconds')
    for i in range(1, 5):
        parser.add_argument('--s%d-speed' % i, type=int, help='signed RAW speed, +/-1..30')
        parser.add_argument('--s%d-steering' % i, type=int, help='RAW steering: -22, 0, or 22')
        parser.add_argument('--s%d-seconds' % i, type=float, help='duration, greater than 0 and at most 10')
    return parser.parse_args(argv)


def make_stages(args):
    cfg = load_config(os.path.join(PACKAGE, 'config'), maneuver_path=os.path.abspath(args.config))
    rows = copy.deepcopy(cfg.get('uturn_trial_sequence'))
    if not isinstance(rows, list) or len(rows) != 4:
        raise ValueError('uturn_trial_sequence must contain exactly four stages')
    for i, row in enumerate(rows, 1):
        if not isinstance(row, dict):
            raise ValueError('each stage must be a mapping')
        if set(row) != set(('speed', 'steering', 'seconds')):
            raise ValueError('each stage must contain only speed, steering, seconds')
        for key in ('speed', 'steering', 'seconds'):
            value = getattr(args, 's%d_%s' % (i, key))
            if value is not None:
                row[key] = value
    countdown = finite(args.countdown, 'countdown', 2, 10)
    cfg['uturn_trial_sequence'] = rows
    # The operator already placed the front axle on the line.
    cfg['uturn_trial_entry_m'] = 0.0
    cfg['uturn_trial_exit_max_s'] = 0.0
    if args.pause is not None:
        cfg['uturn_trial_pause_s'] = args.pause
    # Same raw command encoding as the normal vehicle launcher.
    cfg['steering_command_scale_rad'] = .03
    task = TimedUturn(cfg)
    stages = [Stage('COUNTDOWN', countdown, 0, 0)]
    for name, seconds, command in task.steps:
        raw = encode_command(command[0], command[1], cfg, 0)
        stages.append(Stage(name, seconds, raw['speed_raw'], raw['steering_raw']))
    return stages


def main(argv=None):
    try:
        args = arguments(argv)
        stages = make_stages(args)
        print('Start position: front axle on blue line, vehicle aligned by operator.')
        print('Runs FOUR stages once, then stops. No extra entry, exit correction or lane resume.')
        print('Open-loop test: no sign/blue-line recognition or lidar obstacle avoidance.')
        print('Config: ' + os.path.abspath(args.config))
        print('Command-line overrides apply to this run only; YAML is not changed.')
        print('%-20s %8s %10s %13s' % ('Stage', 'seconds', 'speed_raw', 'steering_raw'))
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
