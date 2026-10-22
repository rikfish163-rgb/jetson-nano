#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Three timed stages with independent signed RAW speed and steering."""
from __future__ import print_function
import argparse
import json
import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot', 'parallel_parking'))
from open_loop_core import Stage, finite, require_base, require_ownership
from parallel_open_loop_test import monotonic_clock

PROFILE = dict(node_name='left_straight_test',
               command_topic='/left/straight_test_cmd',
               status_topic='/left/straight_test_status',
               bridge_node='/left_straight_test_bridge',
               actuator_launch='left_straight_test_actuators.launch')


def make_stages(left_seconds=1., straight_seconds=1., entry_seconds=0.,
                entry_speed=30, entry_steer=0, turn_speed=30, turn_steer=22,
                straight_speed=30, straight_steer=0):
    left_seconds = finite(left_seconds, 'left_seconds', .05, 10)
    straight_seconds = finite(straight_seconds, 'straight_seconds', .05, 10)
    entry_seconds = finite(entry_seconds, 'entry_seconds', 0, 10)
    commands = []
    for name, speed, steer in (('entry', entry_speed, entry_steer),
                               ('turn', turn_speed, turn_steer),
                               ('straight', straight_speed, straight_steer)):
        commands.append((finite(speed, name+'_speed', -30, 30, integer=True),
                         finite(steer, name+'_steer', -22, 22, integer=True)))
    stages = [Stage('COUNTDOWN', 3., 0, 0)]
    if entry_seconds:
        stages.append(Stage('ENTRY', entry_seconds, *commands[0]))
    return stages + [Stage('TURN', left_seconds, *commands[1]),
            Stage('STRAIGHT', straight_seconds, *commands[2]),
            Stage('FINAL_STOP', .7, 0, 0)]


def make_hold_stages(entry_seconds=0.):
    stages = make_stages(entry_seconds=entry_seconds)
    return stages[:-3]+[Stage('HOLD_LEFT', float('inf'), 30, 22)]


def make_reverse_right_stages(seconds=1.):
    seconds = finite(seconds, 'reverse_right_seconds', .05, 10)
    return [Stage('COUNTDOWN', 3., 0, 0),
            Stage('REVERSE_RIGHT', seconds, -30, -22),
            Stage('FINAL_STOP', .7, 0, 0)]


class OwnershipWatch(object):
    """ROS master RPC must never stall the 20 Hz vehicle command loop."""
    def __init__(self, query, clock):
        self.query, self.clock = query, clock
        self.last_ok = clock()
        self.error = None
        self.lock = threading.Lock()
        self.closed = threading.Event()
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon = True

    def start(self):
        self.thread.start()

    def run(self):
        while not self.closed.is_set():
            try:
                self.query()
                with self.lock:
                    self.last_ok = self.clock()
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                return
            self.closed.wait(.5)

    def check(self):
        with self.lock:
            error, last_ok = self.error, self.last_ok
        if error is not None:
            raise ValueError('ownership check failed: '+error)
        if not 0 <= self.clock()-last_ok <= 2.:
            raise ValueError('ownership check stale')

    def close(self):
        self.closed.set()
        if self.thread.is_alive():
            self.thread.join(.1)


class OutputTimedRun(object):
    """Credit only intervals bounded by matching base serial-output reports.

    Feedback has no device timestamp: these are host receipt intervals, not
    encoder measurements. Unknown intervals over .25 seconds earn no credit.
    """
    def __init__(self, stages):
        self.stages = stages
        self.index = 0
        self.started = None
        self.last_match = None
        self.output_seconds = 0.
        self.completed = []
        self.finished = False

    def observe(self, now, speed, steering):
        stage = self.stages[self.index]
        if self.started is None or self.finished or not (stage.speed or stage.steering):
            return
        if (speed, steering) != (stage.speed, stage.steering):
            self.last_match = None
            return
        if self.last_match is not None:
            dt = now-self.last_match
            if 0 <= dt <= .25:
                self.output_seconds += dt
        self.last_match = now

    def tick(self, now):
        if self.finished:
            return 0, 0
        if self.started is None:
            self.started = now
        stage = self.stages[self.index]
        active = stage.speed or stage.steering
        elapsed = self.output_seconds if active else now-self.started
        if active and now-self.started > stage.seconds+2.:
            raise ValueError('%s incomplete: confirmed %.3fs / requested %.3fs' %
                             (stage.name, self.output_seconds, stage.seconds))
        if elapsed >= stage.seconds:
            if active:
                self.completed.append((stage.name, elapsed, stage.seconds))
            self.index += 1
            if self.index == len(self.stages):
                self.index -= 1
                self.finished = True
                return 0, 0
            self.started, self.last_match, self.output_seconds = now, None, 0.
            stage = self.stages[self.index]
        return stage.speed, stage.steering


def execute(stages, record=None):
    """Dedicated executor; leaves production and other experiment runners alone."""
    if not sys.stdin.isatty():
        raise ValueError('execute requires a foreground interactive terminal')
    import select
    import signal
    import termios
    import tty
    import rospy
    import rosgraph
    from std_msgs.msg import String, Bool

    clock = monotonic_clock()
    rospy.init_node(PROFILE['node_name'], anonymous=True, disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise ValueError('real-hardware test rejects /use_sim_time')
    bridge, topic = PROFILE['bridge_node'], PROFILE['command_topic']
    if rospy.get_param(bridge+'/control_topic', '') != topic:
        raise ValueError('start '+PROFILE['actuator_launch']+' first')
    master = rosgraph.Master(rospy.get_name())
    def ownership():
        pubs, subs, _ = master.getSystemState()
        require_ownership(dict(pubs), dict(subs), rospy.get_name(),
                          command=topic, bridge=bridge)
    ownership()
    watch = OwnershipWatch(ownership, clock)
    task, lock = OutputTimedRun(stages), threading.Lock()
    hold = stages[-1].name == 'HOLD_LEFT'
    start_utc = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
    stop_reason = 'COMPLETE'
    base, error, sequence = [None, -1.], [], [0]
    stopped = threading.Event()
    publisher = rospy.Publisher(topic, String, queue_size=1)
    def publish(speed=0, steering=0):
        publisher.publish(String(data=json.dumps(dict(version=1, seq=sequence[0],
            stamp=rospy.Time.now().to_sec(), speed_raw=speed, steering_raw=steering))))
        sequence[0] = (sequence[0]+1) % 256
    def on_base(message):
        try:
            data, now = json.loads(message.data), clock()
            if not isinstance(data, dict):
                raise ValueError('invalid base status')
            with lock:
                previous = base[0]
                base[:] = [data, now]
                valid = (data.get('serial_open') is True and data.get('serial_bytes') == 11
                         and data.get('command_received') is True
                         and data.get('timed_out') is False)
                task.observe(now, data.get('speed_raw') if valid else None,
                             data.get('steering_raw') if valid else None)
            keys = ('speed_raw', 'steering_raw', 'timed_out', 'serial_bytes')
            if previous is None or any(previous.get(k) != data.get(k) for k in keys):
                rospy.loginfo('BASE_OUTPUT mono=%.6f %s', now, json.dumps(data))
        except (ValueError, TypeError) as exc:
            error.append(str(exc))
            stopped.set()
    def estop(message):
        if message.data:
            error.append('emergency stop')
            stopped.set()
    rospy.Subscriber('/base_controller/status', String, on_base, queue_size=1)
    rospy.Subscriber('/competition/estop', Bool, estop, queue_size=1)
    def stop_signal(signum, frame):
        error.append('operator stop' if signum == signal.SIGINT else 'signal %d' % signum)
        stopped.set()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop_signal)
    settings = termios.tcgetattr(sys.stdin)
    try:
        deadline = clock()+8.
        while True:
            if stopped.is_set() or rospy.is_shutdown():
                raise ValueError('interrupted before start')
            publish()
            try:
                with lock:
                    require_base(base[0], base[1], clock())
                if publisher.get_num_connections() != 1:
                    raise ValueError('bridge not connected')
                break
            except ValueError:
                if clock() >= deadline:
                    raise ValueError('base/bridge handshake failed')
            time.sleep(.05)
        tty.setcbreak(sys.stdin.fileno())
        if hold:
            rospy.loginfo('HOLD_LEFT_READY speed=30 steering=22; SPACE/q/Ctrl+C stops and records total')
        else:
            rospy.loginfo('OUTPUT_TIMED_READY plan=%s; SPACE/q/Ctrl+C stops',
                          json.dumps([s._asdict() for s in stages]))
        # Start after handshake; its blocking XMLRPC work stays off this loop.
        watch.last_ok = clock()
        watch.start()
        last_tick, phase, logged = clock(), None, 0
        last_total = clock()
        while not rospy.is_shutdown():
            if stopped.is_set():
                raise ValueError(error[0] if error else 'operator/emergency stop')
            if select.select([sys.stdin], [], [], 0)[0]:
                if os.read(sys.stdin.fileno(), 1) in (b'', b' ', b'x', b'X', b'q', b'Q', b'\x03', b'\x1b'):
                    raise ValueError('operator stop')
            watch.check()
            now = clock()
            if not 0 <= now-last_tick <= .5:
                raise ValueError('control loop gap: unfinished, not COMPLETE')
            last_tick = now
            with lock:
                require_base(base[0], base[1], now)
                command = task.tick(now)
            if publisher.get_num_connections() != 1:
                raise ValueError('bridge connection changed')
            if stopped.is_set():
                raise ValueError(error[0] if error else 'operator/emergency stop')
            publish(*command)
            if hold and now-last_total >= .5:
                with lock:
                    total = task.output_seconds if stages[task.index].name == 'HOLD_LEFT' else 0.
                rospy.loginfo('LEFT_HOLD_TOTAL confirmed_output_s=%.3f', total)
                last_total = now
            while logged < len(task.completed):
                name, actual, requested = task.completed[logged]
                rospy.loginfo('STAGE_COMPLETE %s confirmed_output_s=%.6f requested_s=%.6f',
                              name, actual, requested)
                logged += 1
            if task.index != phase:
                phase = task.index
                rospy.loginfo('STAGE_START %s mono=%.6f speed=%d steering=%d',
                              stages[phase].name, now, command[0], command[1])
            if task.finished:
                rospy.loginfo('COMPLETE: %d motion stage(s) timed using base serial-output status',
                              len(task.completed))
                return 0
            time.sleep(.05)
        raise ValueError('ROS shutdown')
    except Exception as exc:
        stop_reason = str(exc)
        if hold and stop_reason == 'operator stop':
            rospy.loginfo('Operator stopped continuous left test')
            return 0
        rospy.logerr('INCOMPLETE: %s', stop_reason)
        raise
    finally:
        stop_requested = clock()
        try:
            for _ in range(10):
                try:
                    publish()
                except Exception:
                    break
                time.sleep(.05)
        finally:
            try:
                watch.close()
                if hold:
                    with lock:
                        task.finished = True
                        total = task.output_seconds if stages[task.index].name == 'HOLD_LEFT' else 0.
                        data, received = base
                        stop_confirmed = bool(data and received >= stop_requested and
                            clock()-received <= .25 and data.get('speed_raw') == 0 and
                            data.get('steering_raw') == 0 and data.get('serial_bytes') == 11 and
                            data.get('serial_open') is True)
                    result = dict(start_utc=start_utc,
                        end_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                        speed_raw=30, steering_raw=22, confirmed_output_s=round(total, 6),
                        stop_reason=stop_reason, stop_output_confirmed=stop_confirmed,
                        timing_basis='matching_base_status_receipt_intervals')
                    rospy.loginfo('LEFT_HOLD_RESULT %s', json.dumps(result, sort_keys=True))
                    if record is not None:
                        record.write(json.dumps(result, sort_keys=True)+'\n')
                        record.flush()
                        os.fsync(record.fileno())
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true',
                        help='run once after a stopped three-second countdown')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--hold-left', action='store_true',
                        help='hold RAW 30 / left +22 until operator stop; record elapsed output')
    mode.add_argument('--reverse-right-seconds', type=float,
                      help='RAW -30 / right -22 for 0.05..10 seconds, then stop')
    parser.add_argument('--record-file', default=os.path.join(ROOT, 'logs', 'left_hold_times.jsonl'),
                        help='append continuous-left results to this JSONL file')
    parser.add_argument('--left-seconds', '--turn-seconds', dest='left_seconds', type=float, default=1.,
                        help='middle-stage duration, 0.05..10 seconds; direction set by --turn-steer')
    parser.add_argument('--entry-seconds', type=float, default=0.,
                        help='first-stage duration, 0..10 seconds; 0 skips this stage')
    parser.add_argument('--straight-seconds', type=float, default=1.,
                        help='last-stage duration, 0.05..10 seconds')
    for stage in ('entry', 'turn', 'straight'):
        parser.add_argument('--'+stage+'-speed', type=int,
                            help='signed RAW speed -30..30; positive forward, negative reverse')
        parser.add_argument('--'+stage+'-steer', type=int,
                            help='signed RAW steering -22..22; positive left, negative right, 0 centered')
    args = parser.parse_args(argv)
    try:
        commands = dict((stage+'_'+kind, getattr(args, stage+'_'+kind))
                        for stage in ('entry', 'turn', 'straight') for kind in ('speed', 'steer')
                        if getattr(args, stage+'_'+kind) is not None)
        if commands and (args.hold_left or args.reverse_right_seconds is not None):
            raise ValueError('signed stage options belong to the three-stage mode; omit fixed-mode flags')
        if args.reverse_right_seconds is not None:
            if args.entry_seconds != 0 or args.left_seconds != 1 or args.straight_seconds != 1:
                raise ValueError('reverse-right cannot be combined with forward-stage overrides')
            stages = make_reverse_right_stages(args.reverse_right_seconds)
        else:
            stages = (make_hold_stages(args.entry_seconds) if args.hold_left else
                      make_stages(args.left_seconds, args.straight_seconds, args.entry_seconds, **commands))
    except ValueError as exc:
        parser.error(str(exc))
    if args.reverse_right_seconds is not None:
        print('REVERSE RIGHT: speed -30 / steering -22 for %.3fs -> stop.' % args.reverse_right_seconds)
    elif args.hold_left:
        print('RAW 30: hold left +22 until SPACE / q / Ctrl+C. No straight stage.')
        print('Results append to: '+os.path.abspath(args.record_file))
    else:
        print('Timed sequence: ENTRY -> TURN -> STRAIGHT -> STOP; each stage has its own timer.')
        print('Speed: + forward / - reverse / 0 stop. Steering: + left / - right / 0 centered.')
    print('Three-second stopped countdown; no pause between motion stages.')
    print('Open loop; no green sign, camera correction or automatic avoidance.')
    print('Durations count matching base serial-output feedback; not measured wheel motion.')
    for row in stages:
        print('%-20s %5.2fs speed=%d steering=%d' % tuple(row))
    sys.stdout.flush()
    if not args.execute:
        print('PREVIEW ONLY: no ROS node or vehicle commands.')
        return 0
    try:
        if args.hold_left:
            directory = os.path.dirname(os.path.abspath(args.record_file))
            if not os.path.isdir(directory):
                os.makedirs(directory)
            with open(args.record_file, 'a') as record:
                return execute(stages, record=record)
        return execute(stages)
    except (ValueError, RuntimeError, IOError, KeyboardInterrupt) as exc:
        print('STOP / NOT STARTED: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
