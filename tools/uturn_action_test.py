#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""One-shot U-turn from a manually aligned front axle; independent tuning YAML."""
from __future__ import print_function
import argparse
import copy
import os
import sys
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
PACKAGE = os.path.join(ROOT, 'src', 'robot')
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(0, os.path.join(PACKAGE, 'parallel_parking'))
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.uturn.timed import TimedUturn
from open_loop_core import Stage, finite
from parallel_open_loop_test import execute

PROFILE = dict(node_name='uturn_action_test',
               command_topic='/uturn/four_stage_cmd',
               status_topic='/uturn/action_test_status',
               bridge_node='/uturn_test_bridge',
               actuator_launch='uturn_test_actuators.launch')
DEFAULT_CONFIG = os.path.join(ROOT, 'tools', 'uturn_action_test.yaml')
ALLOWED = set(('uturn_trial_sequence', 'uturn_trial_pause_s', 'countdown_s'))


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=DEFAULT_CONFIG,
                        help='independent YAML, reloaded on every run')
    parser.add_argument('--execute', action='store_true',
                        help='run once after countdown; default is offline preview')
    parser.add_argument('--countdown', type=float, help='countdown seconds, 2..10')
    parser.add_argument('--pause', type=float, help='gear/settle pause seconds, 0.3..3')
    parser.add_argument('--stop-after', type=int, help='stop after movement segment N')
    for i in range(1, 21):
        parser.add_argument('--s%d-speed' % i, type=int, help='signed RAW speed, +/-1..30')
        parser.add_argument('--s%d-steering' % i, type=int, help='RAW steering, -22..22')
        parser.add_argument('--s%d-seconds' % i, type=float, help='duration, >0 and <=10')
    return parser.parse_args(argv)


def make_stages(args):
    with open(args.config) as stream:
        tuning = yaml.safe_load(stream)
    if not isinstance(tuning, dict) or set(tuning) - ALLOWED:
        raise ValueError('test YAML allows only: '+', '.join(sorted(ALLOWED)))
    rows = copy.deepcopy(tuning.get('uturn_trial_sequence'))
    if not isinstance(rows, list) or not 1 <= len(rows) <= 20:
        raise ValueError('uturn_trial_sequence must contain 1..20 movement segments')
    for i in range(1, 21):
        if i <= len(rows):
            if not isinstance(rows[i-1], dict) or set(rows[i-1]) != set(('speed','steering','seconds')):
                raise ValueError('each segment must contain exactly speed, steering, seconds')
        for key in ('speed', 'steering', 'seconds'):
            value = getattr(args, 's%d_%s' % (i, key))
            if value is not None:
                if i > len(rows):
                    raise ValueError('override refers to missing segment %d' % i)
                rows[i-1][key] = value
    countdown = finite(args.countdown if args.countdown is not None else tuning.get('countdown_s', 3.0),
                       'countdown', 2, 10)
    cfg = load_config(os.path.join(PACKAGE, 'config'))
    # Apply independent test values AFTER production configuration.
    cfg['uturn_trial_sequence'] = rows
    cfg['uturn_trial_pause_s'] = args.pause if args.pause is not None else tuning.get('uturn_trial_pause_s', .3)
    cfg['uturn_trial_entry_m'] = 0.0
    cfg['uturn_trial_exit_max_s'] = 0.0
    cfg['steering_command_scale_rad'] = .03
    task = TimedUturn(cfg)
    steps = list(task.steps)
    if args.stop_after is not None:
        if not 1 <= args.stop_after <= len(rows):
            raise ValueError('stop-after must refer to an existing movement segment')
        stop_name = 'SEGMENT_%d' % args.stop_after
        index = next(i for i, step in enumerate(steps) if step[0] == stop_name)
        steps = steps[:index+1]+[('SETTLE', cfg['uturn_trial_pause_s'], (0, 0.0))]
    stages = [Stage('COUNTDOWN', countdown, 0, 0)]
    for name, seconds, command in steps:
        raw = encode_command(command[0], command[1], cfg, 0)
        stages.append(Stage(name, seconds, raw['speed_raw'], raw['steering_raw']))
    return stages


def main(argv=None):
    try:
        args = arguments(argv)
        stages = make_stages(args)
        print('Start: front axle aligned with blue line; vehicle aligned by operator.')
        print('Runs once, then stops. No additional entry, exit correction or lane resume.')
        print('Open-loop action only: no camera/sign/blue detection or lidar avoidance.')
        print('Independent config: '+os.path.abspath(args.config))
        print('Command-line overrides are temporary; YAML is not modified.')
        print('%-20s %8s %10s %13s' % ('Stage', 'seconds', 'speed_raw', 'steering_raw'))
        for row in stages:
            print('%-20s %8.3f %10d %13d' % tuple(row))
        sys.stdout.flush()
        if not args.execute:
            print('PREVIEW ONLY: no ROS node or vehicle commands.')
            return 0
        return execute(stages, auto_start=True, **PROFILE)
    except (ValueError, RuntimeError, IOError, yaml.YAMLError, KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
