#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Bounded chassis calibration using the existing command probe. Python 2/3."""
from __future__ import print_function
import argparse
import json
import math
import os
import subprocess
import sys
import time

CASES = {'forward_zero': (1, 0), 'reverse_zero': (-1, 0),
         'forward_left': (1, 22), 'forward_right': (1, -22),
         'reverse_left': (-1, 22), 'reverse_right': (-1, -22)}


def positive(value):
    value = float(value)
    if math.isnan(value) or math.isinf(value) or value <= 0:
        raise argparse.ArgumentTypeError('value must be finite and positive')
    return value


def command(root, case, speed, seconds):
    if case not in CASES or not 1 <= speed <= 30 or not 0 < seconds <= 5:
        raise ValueError('case/speed/duration outside probe limits')
    direction, steer = CASES[case]
    return ['python', os.path.join(root, 'src/robot/motion/bench_command.py'),
            '--speed', str(direction*speed), '--steering', str(steer),
            '--seconds', str(seconds), '--confirm-wheels-safe']


def radius_from_chord(chord, sagitta):
    if not 0 < sagitta <= chord/2:
        raise ValueError('use a minor arc: sagitta must be >0 and <= chord/2')
    return chord*chord/(8*sagitta)+sagitta/2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation')
    run = sub.add_parser('run', help='print command, or execute one bounded motion')
    run.add_argument('case', choices=sorted(CASES))
    run.add_argument('--speed', type=int, default=26)
    run.add_argument('--seconds', type=positive, default=3.)
    run.add_argument('--execute', action='store_true')
    measure = sub.add_parser('measure', help='calculate from manually measured rear-axle path')
    measure.add_argument('--wheelbase-cm', type=positive, default=26.)
    measure.add_argument('--diameter-cm', type=positive)
    measure.add_argument('--chord-cm', type=positive)
    measure.add_argument('--sagitta-cm', type=positive)
    measure.add_argument('--heading-change-deg', type=positive)
    measure.add_argument('--distance-cm', type=positive)
    measure.add_argument('--elapsed-s', type=positive)
    measure.add_argument('--speed-raw', type=positive, default=26.)
    args = parser.parse_args(argv)
    root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    if args.operation == 'measure':
        if args.diameter_cm and (args.chord_cm or args.sagitta_cm or args.heading_change_deg):
            parser.error('use diameter OR chord measurements')
        if args.sagitta_cm and args.heading_change_deg:
            parser.error('use sagitta OR heading change')
        if bool(args.chord_cm) != bool(args.sagitta_cm or args.heading_change_deg):
            parser.error('chord needs either sagitta or heading change')
        if args.heading_change_deg and not 10 <= args.heading_change_deg <= 170:
            parser.error('heading change must be 10..170 degrees')
        if bool(args.distance_cm) != bool(args.elapsed_s):
            parser.error('distance and elapsed time must be supplied together')
        result = {}
        try:
            radius = (args.diameter_cm/2 if args.diameter_cm else
                      args.chord_cm/(2*math.sin(math.radians(args.heading_change_deg)/2))
                      if args.heading_change_deg else
                      radius_from_chord(args.chord_cm,args.sagitta_cm) if args.chord_cm else None)
        except ValueError as exc:
            parser.error(str(exc))
        if radius:
            result.update(rear_axle_radius_m=radius/100.,
                equivalent_model_steer_deg=math.degrees(math.atan(args.wheelbase_cm/radius)))
        if args.distance_cm:
            speed=args.distance_cm/100./args.elapsed_s
            result.update(average_speed_mps=speed, effective_mps_per_raw=speed/args.speed_raw)
        if not result: parser.error('provide radius measurements or distance+elapsed time')
        print(json.dumps(result,indent=2,sort_keys=True))
        return 0
    if args.operation != 'run': parser.error('choose run or measure')
    try:
        probe = command(root,args.case,args.speed,args.seconds)
    except ValueError as exc:
        parser.error(str(exc))
    print('case=%s speed_raw=%d steering_raw=%d duration=%.2fs' %
          (args.case,CASES[args.case][0]*args.speed,CASES[args.case][1],args.seconds))
    if not args.execute:
        print('DRY RUN: no ROS nodes or publishers started. Add --execute for this single test.')
        return 0
    folder = os.path.join(root,'field_data','calibration',
        '%s_%s_%d' % (time.strftime('%Y%m%d_%H%M%S'),args.case,os.getpid()))
    os.makedirs(folder)
    record = dict(case=args.case,speed_raw=CASES[args.case][0]*args.speed,
                  steering_raw=CASES[args.case][1],requested_seconds=args.seconds,
                  started_at=time.time(),returncode=None)
    recorder = None
    print('Evidence: '+folder)
    try:
        with open(os.path.join(folder,'base_status.yaml'),'w') as output:
            with open(os.path.join(folder,'recorder.log'),'w') as errors:
                recorder=subprocess.Popen(['rostopic','echo','/base_controller/status'],
                                           stdout=output,stderr=errors)
                time.sleep(.5)
                if recorder.poll() is not None: raise RuntimeError('status recorder failed')
                record['returncode']=subprocess.call(probe)
                time.sleep(.3)
        return record['returncode']
    except KeyboardInterrupt:
        record['interrupted']=True
        return 130
    except (OSError,RuntimeError) as exc:
        record['error']=str(exc)
        print(str(exc))
        return 1
    finally:
        if recorder is not None and recorder.poll() is None:
            recorder.terminate()
            recorder.wait()
        record['finished_at']=time.time()
        with open(os.path.join(folder,'run.json'),'w') as output:
            json.dump(record,output,indent=2)


if __name__ == '__main__':
    sys.exit(main())
