#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Interactive one-shot test. Default is OFFLINE preview, not vehicle motion."""
from __future__ import print_function
import argparse
import json
import os
import sys
import time
from open_loop_core import (make_plan, make_slot_plan, slot_distance, SLOT_REFERENCE, SLOT_MOTION,
                            OneShot, require_ownership, require_base)


def monotonic_clock():
    if hasattr(time, 'monotonic'):
        return time.monotonic
    # Python 2.7 / Ubuntu 18.04: use CLOCK_MONOTONIC, never ROS/sim/wall time.
    import ctypes
    class Timespec(ctypes.Structure):
        _fields_ = [('seconds', ctypes.c_long), ('nanoseconds', ctypes.c_long)]
    library = ctypes.CDLL('librt.so.1', use_errno=True)
    clock = library.clock_gettime
    clock.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]
    clock.restype = ctypes.c_int
    def read():
        value = Timespec()
        if clock(1, ctypes.byref(value)) != 0:
            raise RuntimeError('CLOCK_MONOTONIC failed')
        return value.seconds + value.nanoseconds/1e9
    read()
    return read


def arguments(argv):
    parser = argparse.ArgumentParser(description='One-shot OPEN LOOP parking test; no obstacle avoidance.')
    parser.add_argument('--config', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                        'open_loop_test.yaml'))
    parser.add_argument('--execute', action='store_true',
                        help='enable ROS output; manual mode waits for g, --slot starts countdown automatically')
    parser.add_argument('--slot', type=str.upper, choices=('P1', 'P2', 'P3'),
                        help='position from P1 front line, then reverse left 1.6s/right 1.4s; auto countdown')
    start = parser.add_mutually_exclusive_group()
    start.add_argument('--auto-start', action='store_true', help='start countdown after base handshake')
    start.add_argument('--wait-g', action='store_true', help='wait for g, including in --slot mode')
    parser.add_argument('--position-speed', type=int, help='raw speed magnitude for reference positioning')
    parser.add_argument('--seek-speed', type=int, help='lane-following speed while seeking P1 front line')
    parser.add_argument('--creep-speed', type=int, help='lane-following speed near P1 front line, <= seek speed')
    parser.add_argument('--line-tolerance-m', type=float, help='front-axle line alignment tolerance')
    parser.add_argument('--reference-only', action='store_true',
                        help='with --slot, stop after P1 front-axle alignment; do not enter bay')
    for flag, key in (('--reference-offset-m', 'reference_offset_m'),
                      ('--slot-spacing-m', 'slot_spacing_m'),
                      ('--forward-mps-per-raw', 'forward_mps_per_raw'),
                      ('--reverse-mps-per-raw', 'reverse_mps_per_raw')):
        parser.add_argument(flag, dest=key, type=float, help='slot reference/model override (metres/seconds)')
    parser.add_argument('--side', choices=('right', 'left'), help='bay side relative to the vehicle')
    parser.add_argument('--forward-only', dest='forward_only', action='store_true', default=None,
                        help='stop after forward preparation; do not reverse')
    for flag, key, kind, help_text in (
            ('--forward-speed', 'forward_speed_raw', int, 'positive forward speed raw, 1..30'),
            ('--forward-seconds', 'forward_seconds', float, 'straight forward preparation seconds, 0..5; 0 skips'),
            ('--speed', 'speed_raw', int, 'positive reverse speed raw, 1..30'),
            ('--first-steer', 'first_steering_raw', int, 'first steering magnitude raw, 1..22'),
            ('--first-seconds', 'first_seconds', float, 'first reverse turn seconds, 0.05..5'),
            ('--second-steer', 'second_steering_raw', int, 'countersteering magnitude raw, 1..22'),
            ('--second-seconds', 'second_seconds', float, 'countersteering seconds; 0 disables'),
            ('--repeat-bay-steer', 'repeat_bay_steering_raw', int,
             'second turn-toward-bay magnitude raw, 1..22'),
            ('--repeat-bay-seconds', 'repeat_bay_seconds', float,
             'second reverse turn toward bay; 0 disables'),
            ('--repeat-counter-steer', 'repeat_counter_steering_raw', int,
             'second countersteering magnitude raw, 1..22'),
            ('--repeat-counter-seconds', 'repeat_counter_seconds', float,
             'second reverse countersteer; 0 disables'),
            ('--straight-seconds', 'straight_seconds', float, 'straight reverse seconds; 0 disables'),
            ('--pause-seconds', 'pause_seconds', float, 'stopped pause between stages, 0.3..3'),
            ('--countdown-seconds', 'countdown_seconds', float, 'stopped countdown, 2..10')):
        parser.add_argument(flag, dest=key, type=kind, help=help_text)
    args = parser.parse_args(argv)
    import yaml
    with open(args.config) as stream:
        values = yaml.safe_load(stream)
    if not isinstance(values, dict):
        raise ValueError('config must be a YAML mapping')
    reference = values.pop('slot_reference', {})
    if not isinstance(reference, dict) or set(reference) - set(SLOT_REFERENCE):
        raise ValueError('slot_reference must contain only reference/model parameters')
    reference = dict(SLOT_REFERENCE, **reference)
    slot_motion=values.pop('slot_motion',{})
    if not isinstance(slot_motion,dict) or set(slot_motion)-set(SLOT_MOTION):
        raise ValueError('slot_motion must contain only speed_raw and position_speed_raw')
    slot_motion=dict(SLOT_MOTION,**slot_motion)
    search=values.pop('reference_search', {})
    extra = ('slot', 'auto_start', 'wait_g', 'position_speed', 'seek_speed',
             'creep_speed', 'line_tolerance_m', 'reference_only') + tuple(SLOT_REFERENCE)
    for key, value in vars(args).items():
        if key not in ('config', 'execute') + extra and value is not None:
            values[key] = value
    if args.slot:
        incompatible = ('side', 'forward_only', 'forward_seconds', 'first_steering_raw',
                        'second_steering_raw', 'repeat_bay_steering_raw', 'repeat_bay_seconds',
                        'repeat_counter_steering_raw', 'repeat_counter_seconds', 'straight_seconds')
        if any(getattr(args, key) is not None for key in incompatible):
            raise ValueError('--slot cannot be combined with manual side/forward/extra-stage options')
        # Validate the legacy config too; misspelled keys must never be ignored.
        make_plan(values)
        reference.update((key, getattr(args, key)) for key in SLOT_REFERENCE
                         if getattr(args, key) is not None)
        sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        from robot.common.config import load_config
        from robot.motion import calibration as chassis
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'config'))
        chassis.validate(cfg)
        position_speed=args.position_speed if args.position_speed is not None else (
            args.forward_speed_raw if args.forward_speed_raw is not None else slot_motion['position_speed_raw'])
        # Preserve explicit coefficient overrides; otherwise use the shared measured model.
        if args.forward_mps_per_raw is None:
            reference['forward_mps_per_raw']=chassis.speed_gain(cfg,position_speed)
        if args.reverse_mps_per_raw is None:
            reference['reverse_mps_per_raw']=chassis.speed_gain(cfg,-position_speed)
        args.reference_distance_m = slot_distance(args.slot, reference['reference_offset_m'],
                                                  reference['slot_spacing_m'])
        stages=make_slot_plan(args.slot,
            speed_raw=args.speed_raw if args.speed_raw is not None else slot_motion['speed_raw'],
            position_speed_raw=position_speed,
            first_seconds=args.first_seconds if args.first_seconds is not None else 1.6,
            second_seconds=args.second_seconds if args.second_seconds is not None else 1.4,
            pause_seconds=values.get('pause_seconds', .7),
            countdown_seconds=values.get('countdown_seconds', 3.), **reference)
        from robot.parallel_parking.reference_core import ReferenceRun
        if not isinstance(search,dict): raise ValueError('reference_search must be a mapping')
        search=dict(search)
        search['forward_mps_per_raw']=reference['forward_mps_per_raw']
        if args.forward_mps_per_raw is None:
            points=cfg.get('chassis_calibration',{}).get('speed_points',{}).get('forward')
            if points is not None:search['forward_speed_table']=points
        for flag,key in (('seek_speed','seek_speed_raw'),('creep_speed','creep_speed_raw'),
                         ('line_tolerance_m','line_tolerance_m')):
            if getattr(args,flag) is not None: search[key]=getattr(args,flag)
        if args.reference_only:search['reference_only']=True
        ReferenceRun(stages,**search)  # Validate before ROS imports, including preview.
        args.reference_search=search
        return args, stages
    if (args.reference_only or any(getattr(args,key) is not None for key in ('position_speed','seek_speed','creep_speed','line_tolerance_m'))
            or any(getattr(args, key) is not None for key in SLOT_REFERENCE)):
        raise ValueError('reference/model options require --slot')
    return args, make_plan(values)


def print_plan(stages, args=None):
    print('No lidar, no automatic collision/parking detection; slot entry uses measured camera reference.')
    if args is not None and args.slot:
        print('REFERENCE: follow road lane while seeking P1; stop front axle on line, THEN position for selected slot.')
        print('P1 is confirmed in 2 consecutive images; no optical flow. Blind approach uses timed distance estimate.')
        if args.reference_only:print('REFERENCE ONLY: stop on P1 line; timed positioning/reverse stages below will NOT run.')
        print('Slot %s: signed position distance = %+.3f m (positive = forward).' %
              (args.slot, args.reference_distance_m))
        print('Distance is TIME ESTIMATED using the shared calibrated speed model, NOT measured odometry.')
        print('Left/right reverse stages are continuous; no stopped pause between them.')
    else:
        print('Forward preparation starts at the marked front-axle/bay-rear-line position.')
        print('Forward time is UNCALIBRATED. Use --forward-only to measure it first.')
    print('Positive steering = left; negative steering = right. Verify on raised wheels first.')
    print('Stage                         seconds    speed_raw    steering_raw')
    for row in stages:
        print('%-29s %7.2f %12d %15d' % tuple(row))
    print('Runs ONCE, then stops. No automatic repeat or resume.')


def execute(stages, auto_start=False, reference_search=None, on_command=None,
            node_name='parallel_open_loop_test',
            command_topic='/parallel_parking/open_loop_cmd',
            status_topic='/parallel_parking/open_loop_status',
            bridge_node='/parallel_open_loop_bridge',
            actuator_launch='parallel_open_loop_actuators.launch'):
    # No ROS imports/publishers at all on the default preview path.
    if not sys.stdin.isatty():
        raise ValueError('execute requires a foreground interactive SSH terminal')
    import select
    import signal
    import termios
    import tty
    import threading
    import rospy
    import rosgraph
    from std_msgs.msg import String, Bool

    clock = monotonic_clock()
    rospy.init_node(node_name, anonymous=True, disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise ValueError('real-hardware test rejects /use_sim_time')
    if rospy.get_param(bridge_node+'/control_topic', '') != command_topic:
        raise ValueError('start the dedicated '+actuator_launch+' first')
    master = rosgraph.Master(rospy.get_name())
    def ownership():
        pubs, subs, _ = master.getSystemState()
        require_ownership(dict(pubs), dict(subs), rospy.get_name(),
                          command=command_topic, bridge=bridge_node)
        if reference_search is not None and set(dict(pubs).get('/parallel_parking/p1_reference', [])) != {'/parallel_p1_reference'}:
            raise ValueError('start parallel_slot_test.launch: unique P1 reference camera publisher required')
        if reference_search is not None and len(set(dict(pubs).get('/vision/lane_observation', [])))!=1:
            raise ValueError('start parallel_slot_test.launch: unique road lane observation publisher required')
    ownership()
    abort_event = threading.Event()
    reason = ['interrupted']
    base = [None, -1.0]
    lock = threading.Lock()
    sequence = [0]
    publisher = rospy.Publisher(command_topic, String, queue_size=1)
    status_pub = rospy.Publisher(status_topic, String, queue_size=1)
    def publish(speed=0, steering=0):
        publisher.publish(String(data=json.dumps(dict(version=1, seq=sequence[0],
            stamp=rospy.Time.now().to_sec(), speed_raw=int(speed), steering_raw=int(steering)))))
        sequence[0] = (sequence[0]+1) % 256
        if on_command is not None:on_command(int(speed),int(steering),clock())
    def on_base(message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError('status is not a mapping')
            with lock:
                base[:] = [data, clock()]
        except (ValueError, TypeError):
            reason[0] = 'invalid base status'
            abort_event.set()
    def on_estop(message):
        if message.data:
            reason[0] = 'emergency stop topic'
            abort_event.set()
    rospy.Subscriber('/base_controller/status', String, on_base, queue_size=1)
    rospy.Subscriber('/competition/estop', Bool, on_estop, queue_size=1)
    def stop_signal(signum, frame):
        reason[0] = 'process signal %d' % signum
        abort_event.set()
    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, stop_signal)
    settings = termios.tcgetattr(sys.stdin)
    if reference_search is None:
        task=OneShot(stages)
    else:
        from robot.parallel_parking.reference_core import ReferenceRun
        from robot.parallel_parking.lane_follow import LaneFollower
        from robot.common.config import load_config
        task=ReferenceRun(stages,**reference_search)
        config_dir=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),'config')
        lane=LaneFollower(load_config(config_dir),task.seek)
        def on_reference(message):
            try:
                data=json.loads(message.data)
                with lock: task.observe(data,clock(),rospy.Time.now().to_sec())
            except (ValueError,TypeError,KeyError) as exc:
                reason[0]='invalid P1 reference: '+str(exc)
                abort_event.set()
        rospy.Subscriber('/parallel_parking/p1_reference',String,on_reference,queue_size=1)
        def on_lane(message):
            try:
                with lock:lane.observe(message.data,rospy.Time.now().to_sec())
            except (ValueError,TypeError,KeyError) as exc:
                reason[0]='invalid lane observation: '+str(exc)
                abort_event.set()
        rospy.Subscriber('/vision/lane_observation',String,on_lane,queue_size=1)
    started = auto_start
    try:
        # Handshake sends ONLY zero commands. Never arm on missing status.
        deadline = clock()+8
        while True:
            if abort_event.is_set() or rospy.is_shutdown():
                raise ValueError(reason[0])
            publish()
            with lock:
                snapshot = tuple(base)
            try:
                require_base(snapshot[0], snapshot[1], clock())
                if publisher.get_num_connections() < 1:
                    raise ValueError('bridge not connected')
                break
            except ValueError as exc:
                if clock() >= deadline:
                    raise ValueError('not armed: '+str(exc))
            time.sleep(0.05)
        tty.setcbreak(sys.stdin.fileno())
        print('READY: %s. SPACE / x / q / Ctrl+C = STOP and exit.' %
              ('automatic countdown' if auto_start else 'press g to start countdown'))
        sys.stdout.flush()
        last_graph = clock()
        armed_deadline = clock()+30
        last_phase = None
        last_gap_print = -1.
        while not rospy.is_shutdown():
            if abort_event.is_set():
                raise ValueError(reason[0])
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                key = os.read(sys.stdin.fileno(), 1)
                if key in (b'', b' ', b'x', b'X', b'q', b'Q', b'\x03', b'\x1b'):
                    raise ValueError('operator stop')
                if key in (b'g', b'G') and not started:
                    started = True
            if clock()-last_graph >= 0.5:
                ownership()
                last_graph = clock()
            if abort_event.is_set():
                raise ValueError(reason[0])
            with lock:
                snapshot = tuple(base)
            now = clock()
            require_base(snapshot[0], snapshot[1], now)
            if publisher.get_num_connections() != 1:
                raise ValueError('bridge connection lost or unexpected subscriber')
            if not started:
                if now >= armed_deadline:
                    raise ValueError('g was not pressed within 30 seconds')
                command, phase = (0, 0), 'WAIT_G'
            else:
                with lock:
                    command=task.tick(now)
                    phase=task.reason
                    if reference_search is not None and task.phase!='MANEUVER' and command[0]>0:
                        lane_command=lane.command(rospy.Time.now().to_sec())
                        if lane_command[0]>0:
                            command=(command[0],lane_command[1])
                        elif task.line is not None:
                            command=task.abort('LANE_TRACKING_LOST')
                            phase=task.reason
                        else:
                            command=(0,0)
                            phase='WAIT_ROAD_LANE'
            if reference_search is not None:
                with lock:task.set_command(command[0],now)
            publish(*command)
            status_pub.publish(String(data=json.dumps(dict(phase=phase, speed_raw=command[0],
                steering_raw=command[1], remaining_s=max(0, task.deadline-now) if task.deadline else 0,
                front_axle_gap_m=getattr(task,'gap',None),
                front_axle_gap_source=getattr(task,'gap_source',None)))))
            if phase != last_phase:
                print('%s  speed=%d  steering=%d' % (phase, command[0], command[1]))
                sys.stdout.flush()
                last_phase = phase
            gap=getattr(task,'gap',None)
            if gap is not None and getattr(task,'phase',None)!='MANEUVER' and now-last_gap_print>=.5:
                print('P1 front-axle remaining: %.1f cm  speed_raw=%d steering_raw=%d source=%s' %
                      (gap*100.,command[0],command[1],getattr(task,'gap_source',None)))
                sys.stdout.flush()
                last_gap_print=now
            if task.finished:
                if task.reason != 'COMPLETE':
                    raise ValueError(task.reason)
                return 0
            time.sleep(0.05)
        raise ValueError('ROS shutdown')
    finally:
        task.abort('STOPPED')
        # Existing bridge/base watchdogs remain active if this process disappears.
        try:
            for _ in range(10):
                try:
                    publish()
                except Exception:
                    break
                time.sleep(0.05)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(argv=None):
    try:
        args, stages = arguments(argv)
        print_plan(stages, args)
        if not args.execute:
            print('PREVIEW ONLY: no ROS node and no vehicle commands. Add --execute for a field test.')
            return 0
        return execute(stages, auto_start=args.auto_start or bool(args.slot and not args.wait_g),
                       reference_search=getattr(args,'reference_search',None))
    except (ValueError, RuntimeError, KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
