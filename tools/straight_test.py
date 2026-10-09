#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Supervised, bounded zero-steering test using the existing RAW probe."""
from __future__ import print_function

import argparse
import math
import os
import subprocess
import time


def probe_command(root, speed, seconds):
    return ['python', os.path.join(root, 'src/robot/motion/bench_command.py'),
            '--speed', str(speed), '--steering', '0', '--seconds', str(seconds),
            '--confirm-wheels-safe']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=int, default=26, help='forward RAW speed, 1..30')
    parser.add_argument('--seconds', type=float, default=2.0, help='duration, up to 5 seconds')
    parser.add_argument('--confirm-clear', action='store_true', help='confirm clear ground test area')
    args = parser.parse_args()
    if not 1 <= args.speed <= 30:
        parser.error('speed must be 1..30')
    if math.isnan(args.seconds) or not 0 < args.seconds <= 5:
        parser.error('seconds must be greater than 0 and at most 5')
    if not args.confirm_clear:
        parser.error('ground motion requires --confirm-clear')

    root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    directory = os.path.join(root, 'field_data', 'straight_%s_%s' %
                             (time.strftime('%Y%m%d_%H%M%S'), os.getpid()))
    os.makedirs(directory)
    status_path = os.path.join(directory, 'base_status.yaml')
    recorder = None
    print('Forward RAW=%d, steering RAW=0, duration=%.2fs. Ctrl+C stops.' %
          (args.speed, args.seconds))
    print('Base-controller evidence: ' + status_path)
    try:
        with open(status_path, 'w') as status, open(os.path.join(directory, 'recorder.log'), 'w') as errors:
            try:
                recorder = subprocess.Popen(['rostopic', 'echo', '/base_controller/status'],
                                            stdout=status, stderr=errors)
                time.sleep(1)
                if recorder.poll() is not None:
                    raise RuntimeError('status recorder exited; check recorder.log and ROS setup')
                # The existing probe rejects competing command publishers and
                # sends zero speed in finally, including on Ctrl+C.
                result = subprocess.call(probe_command(root, args.speed, args.seconds))
                time.sleep(.5)
                return result
            finally:
                if recorder is not None and recorder.poll() is None:
                    recorder.terminate()
                    recorder.wait()
    except KeyboardInterrupt:
        print('Interrupted; probe sends zero speed. Check vehicle has stopped.')
        return 130
    except (OSError, RuntimeError) as exc:
        print('Test not completed: ' + str(exc))
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
