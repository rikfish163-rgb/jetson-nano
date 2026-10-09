"""Pure unit tests and offline CLI checks; never imports rospy or drives hardware."""
from __future__ import division
import os
import sys
import unittest
import subprocess
import types

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE = os.path.join(os.path.dirname(HERE), 'parallel_parking')
if not os.path.isfile(os.path.join(MODULE, 'open_loop_core.py')):
    MODULE = HERE  # Also runnable in the local staging directory.
sys.path.insert(0, MODULE)
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
from open_loop_core import DEFAULTS, OneShot, make_plan, make_slot_plan, require_base, require_ownership
from parallel_open_loop_test import monotonic_clock
import parallel_open_loop_test as app


class SimulatedRuntimeTests(unittest.TestCase):
    """Replace ALL ROS/terminal adapters; exercise real runner without hardware."""
    def run_fake(self, fault=None, auto_start=False, slot=None, use_reference=False, observer=None):
        instant = [0.0]
        commands = []
        callbacks = {}
        restored = []
        status_phases=[]
        keys = [] if auto_start else [b'g']
        saved_modules = {}
        def module(name, **attrs):
            obj = types.ModuleType(name)
            obj.__dict__.update(attrs)
            saved_modules[name] = sys.modules.get(name)
            sys.modules[name] = obj
            return obj
        class Message(object):
            def __init__(self, data=None): self.data = data
        class Publisher(object):
            def __init__(self, topic, *args, **kwargs): self.topic = topic
            def get_num_connections(self): return 1
            def publish(self, message):
                if self.topic == '/parallel_parking/open_loop_status':
                    import json
                    status_phases.append(json.loads(message.data)['phase'])
                if self.topic != '/parallel_parking/open_loop_cmd': return
                import json
                value = json.loads(message.data)
                commands.append((instant[0], value['speed_raw'], value['steering_raw']))
                if fault == 'stale' and instant[0] >= 2.1: return
                if '/base_controller/status' in callbacks:
                    callbacks['/base_controller/status'](Message(json.dumps(dict(
                        serial_open=True, serial_bytes=11, command_received=True, timed_out=False))))
                if '/parallel_parking/p1_reference' in callbacks and fault!='reference_missing':
                    x=min(.64,max(0.,(instant[0]-2.2)*.3))
                    callbacks['/parallel_parking/p1_reference'](Message(json.dumps(dict(
                        source='p1_reference_camera',frame='base_link',stamp=1000+instant[0],
                        wheelbase_m=.26,
                        p1_lines=[] if x>.3 or fault=='reference_unseen' else [[(.9-x,-.22),(.9-x,-.58)]]))))
                if '/vision/lane_observation' in callbacks and not (fault=='lane_lost' and instant[0]>=2.3):
                    callbacks['/vision/lane_observation'](Message(json.dumps(dict(
                        stamp=1000+instant[0],frame='base_link',confidence=.95,
                        points=[[.2,-.06],[.6,-.06],[1.,-.06]]))))
        class Master(object):
            def __init__(self, name): pass
            def getSystemState(self):
                pubs, subs = OpenLoopTests('test_default_only_first_turn_and_stop').graph()
                if use_reference:
                    pubs['/parallel_parking/p1_reference']=['/parallel_p1_reference']
                    pubs['/vision/lane_observation']=['/parallel_lane']
                if fault == 'competing' and instant[0] > 2.1:
                    pubs['/control/cmd'] = ['/unexpected']
                return list(pubs.items()), list(subs.items()), []
        class Terminal(object):
            def isatty(self): return True
            def fileno(self): return 42
        def sleep(seconds):
            instant[0] += seconds
        def read(fd, count):
            return keys.pop(0) if keys else b' '
        def select_fn(*args):
            active = (bool(keys) or (fault == 'operator' and instant[0] >= 2.1) or
                      (fault=='reference_unseen' and instant[0]>=4.))
            return ([sys.stdin] if active else [], [], [])
        def subscribe(topic, kind, callback, **kwargs): callbacks[topic] = callback
        params = {'/parallel_open_loop_bridge/control_topic': '/parallel_parking/open_loop_cmd'}
        old_stdin, old_clock, old_time, old_os = sys.stdin, app.monotonic_clock, app.time, app.os
        try:
            module('rospy', init_node=lambda *a, **k: None,
                   get_param=lambda key, default=None: params.get(key, default),
                   get_name=lambda: '/test', Publisher=Publisher, Subscriber=subscribe,
                   is_shutdown=lambda: False,
                   Time=type('Clock', (), {'now': staticmethod(lambda: type('Stamp', (), {
                       'to_sec': lambda self: 1000+instant[0]})())}))
            module('rosgraph', Master=Master)
            messages = module('std_msgs.msg', String=Message, Bool=Message)
            module('std_msgs', msg=messages)
            module('select', select=select_fn)
            module('termios', tcgetattr=lambda stream: 'settings', TCSADRAIN=0,
                   tcsetattr=lambda *a: restored.append(True))
            module('tty', setcbreak=lambda fd: None)
            module('signal', SIGTERM=15, SIGINT=2, SIGHUP=1, signal=lambda *a: None)
            sys.stdin = Terminal()
            app.monotonic_clock = lambda: lambda: instant[0]
            app.time = type('FakeTime', (), {'sleep': staticmethod(sleep)})
            app.os = type('FakeOS', (), {'read': staticmethod(read),'path':old_os.path})
            plan = make_plan(dict(countdown_seconds=2, first_seconds=1, speed_raw=26,
                                  forward_seconds=0))
            if slot:
                plan = make_slot_plan(slot, countdown_seconds=2, speed_raw=26,
                                      position_speed_raw=26)
            search={} if use_reference else None
            if fault:
                with self.assertRaises(ValueError): app.execute(plan,auto_start=auto_start,reference_search=search)
            else:
                self.assertEqual(app.execute(plan, auto_start=auto_start,reference_search=search,
                                             on_command=observer), 0)
        finally:
            sys.stdin, app.monotonic_clock, app.time, app.os = old_stdin, old_clock, old_time, old_os
            for name, previous in saved_modules.items():
                if previous is None: sys.modules.pop(name, None)
                else: sys.modules[name] = previous
        self.assertTrue(restored)
        if fault=='reference_missing':
            self.assertTrue(all(speed==steer==0 for _,speed,steer in commands))
        elif fault in ('lane_lost','reference_unseen'):
            self.assertTrue(all(speed>=0 for _,speed,steer in commands))
        else:
            self.assertTrue(any(speed == -26 and steer == -22 for _, speed, steer in commands))
        if use_reference and not fault:
            self.assertLess(status_phases.index('REFERENCE_ALIGNED'),
                            status_phases.index('POSITION_FROM_REFERENCE'))
            self.assertLess(status_phases.index('POSITION_FROM_REFERENCE'),
                            status_phases.index('REVERSE_LEFT_LOCK'))
        self.assertTrue(all(speed == steer == 0 for _, speed, steer in commands[-10:]))
        return commands

    def test_runner_finishes_and_sends_stop(self): self.run_fake()
    def test_operator_stop_exits_and_sends_stop(self): self.run_fake('operator')
    def test_status_loss_exits_and_sends_stop(self): self.run_fake('stale')
    def test_new_controller_exits_and_sends_stop(self): self.run_fake('competing')
    def test_auto_start_runs_without_g_and_sends_stop(self): self.run_fake(auto_start=True)

    def test_command_observer_receives_every_published_command_and_timestamp(self):
        observed=[]
        commands=self.run_fake(observer=lambda speed,steering,now:observed.append((now,speed,steering)))
        self.assertEqual(observed,commands)

    def test_reference_camera_alignment_is_used_by_real_runner_before_slot_motion(self):
        commands=self.run_fake(auto_start=True,slot='P1',use_reference=True)
        road=[(speed,steer) for _,speed,steer in commands if speed==15]
        self.assertTrue(road)
        self.assertTrue(all(steer<0 for speed,steer in road),
                        'road approach must steer right when lane center is right')

    def test_reference_camera_missing_never_moves_or_reverses(self):
        self.run_fake(fault='reference_missing',auto_start=True,slot='P1',use_reference=True)

    def test_unseen_p1_continues_lane_following_without_zero_steering(self):
        commands=self.run_fake(fault='reference_unseen',auto_start=True,slot='P1',use_reference=True)
        road=[(speed,steer) for t,speed,steer in commands if 2.1<=t<4.]
        self.assertTrue(road)
        self.assertTrue(all(speed==15 and steer<0 for speed,steer in road))

    def test_lane_stream_loss_stops_before_parking_maneuver(self):
        commands=self.run_fake(fault='lane_lost',auto_start=True,slot='P1',use_reference=True)
        self.assertTrue(any(speed==15 and steer<0 for _,speed,steer in commands))

    def test_slot_runner_positions_then_continuously_switches_left_to_right(self):
        commands = self.run_fake(auto_start=True, slot='P1')
        phases = []
        for _, speed, steer in commands:
            command = (speed, steer)
            if not phases or phases[-1] != command:
                phases.append(command)
        self.assertEqual(phases, [(0, 0), (26, 0), (0, 0),
                                 (-26, 22), (-26, -22), (0, 0)])


class SlotPlanTests(unittest.TestCase):
    def test_each_slot_moves_from_same_reference_then_left_right(self):
        for slot, distance in [('P1', .85), ('P2', .15), ('P3', -.55)]:
            plan = make_slot_plan(slot, speed_raw=26, position_speed_raw=26)
            moving = [s for s in plan if s.speed]
            expected_speed = 26 if distance > 0 else -26
            coefficient = .008 if distance > 0 else .007
            self.assertEqual((moving[0].speed, moving[0].steering), (expected_speed, 0))
            self.assertAlmostEqual(moving[0].seconds, abs(distance)/(26*coefficient))
            self.assertEqual([(s.seconds, s.speed, s.steering) for s in moving[1:]],
                             [(1.6, -26, 22), (1.4, -26, -22)])
            left = next(i for i, s in enumerate(plan) if s.name == 'REVERSE_LEFT_LOCK')
            self.assertEqual(plan[left+1].name, 'REVERSE_RIGHT_LOCK')
            self.assertEqual((plan[-1].speed, plan[-1].steering), (0, 0))

    def test_default_slot_state_machine_uses_speed_30_for_position_and_both_turns(self):
        for slot,distance in [('P1',.85),('P2',.15),('P3',-.55)]:
            self.assertAlmostEqual(app.slot_distance(slot),distance)
            stages=make_slot_plan(slot)
            moving=[stage for stage in stages if stage.speed]
            self.assertEqual((moving[0].speed,moving[0].steering),(30 if distance>0 else -30,0))
            self.assertAlmostEqual(moving[0].seconds,abs(distance)/(30*(.008 if distance>0 else .007)))
            self.assertEqual([(s.seconds,s.speed,s.steering) for s in moving[1:]],
                             [(1.6,-30,22),(1.4,-30,-22)])

    def test_zero_offset_skips_positioning(self):
        plan = make_slot_plan('P1', reference_offset_m=0)
        self.assertFalse(any(s.name == 'POSITION_FROM_REFERENCE' for s in plan))

    def test_distance_and_time_are_configurable(self):
        plan = make_slot_plan('P2', reference_offset_m=.4, slot_spacing_m=.8,
                             reverse_mps_per_raw=.01, position_speed_raw=20)
        self.assertAlmostEqual(plan[1].seconds, 2.)
        self.assertEqual(plan[1].speed, -20)

    def test_invalid_or_excessive_distance_time_rejected(self):
        for values in [dict(slot='P4'), dict(slot='AUTO'), dict(speed_raw=0),
                       dict(position_speed_raw=True), dict(reference_offset_m=float('nan')),
                       dict(slot_spacing_m=0), dict(forward_mps_per_raw=0),
                       dict(reverse_mps_per_raw=float('inf')),
                       dict(reverse_mps_per_raw=.0001), dict(first_seconds=0),
                       dict(second_seconds=0)]:
            args = dict(slot='P1')
            args.update(values)
            if 'reverse_mps_per_raw' in values:
                args['slot'] = 'P3'
            with self.assertRaises(ValueError): make_slot_plan(**args)


class OpenLoopTests(unittest.TestCase):
    def graph(self):
        return ({'/ackermann_cmd': ['/parallel_open_loop_bridge'],
                 '/parallel_parking/open_loop_cmd': ['/test'],
                 '/base_controller/status': ['/base_controller']},
                {'/parallel_parking/open_loop_cmd': ['/parallel_open_loop_bridge'],
                 '/ackermann_cmd': ['/base_controller']})

    def healthy(self):
        return dict(serial_open=True, command_received=True, timed_out=False, serial_bytes=11)

    def test_default_only_first_turn_and_stop(self):
        plan = make_plan()
        self.assertEqual([s.name for s in plan], ['COUNTDOWN', 'FORWARD_PREPARE',
                         'PAUSE_BEFORE_REVERSE', 'REVERSE_INTO_BAY', 'FINAL_STOP'])
        self.assertEqual((plan[1].speed, plan[1].steering), (12, 0))
        self.assertEqual((plan[3].speed, plan[3].steering), (-12, -22))
        self.assertEqual(plan[3].seconds, 0.5)

    def test_two_right_left_cycles_and_straight(self):
        plan = make_plan(dict(speed_raw=26, second_seconds=1,
                              repeat_bay_steering_raw=18, repeat_bay_seconds=.8,
                              repeat_counter_steering_raw=16, repeat_counter_seconds=.7,
                              straight_seconds=0.2))
        moving = [s for s in plan if s.speed < 0]
        self.assertEqual([(s.speed, s.steering) for s in moving],
                         [(-26, -22), (-26, 22), (-26, -18), (-26, 16), (-26, 0)])
        self.assertTrue(all(s.speed == s.steering == 0 for s in plan if s.name == 'PAUSE'))

    def test_left_side_mirrors(self):
        plan = make_plan(dict(side='left', second_seconds=1,
                              repeat_bay_seconds=1, repeat_counter_seconds=1))
        self.assertEqual([s.steering for s in plan if s.speed < 0], [22, -22, 22, -22])

    def test_forward_only_never_reverses_even_with_reverse_durations(self):
        plan = make_plan(dict(forward_only=True, forward_speed_raw=18,
                             second_seconds=1, repeat_bay_seconds=1,
                             repeat_counter_seconds=1, straight_seconds=1))
        self.assertEqual([s.name for s in plan], ['COUNTDOWN', 'FORWARD_PREPARE', 'FINAL_STOP'])
        task = OneShot(plan)
        commands = [task.tick(i * .05) for i in range(120)]
        self.assertTrue(task.finished)
        self.assertIn((18, 0), commands)
        self.assertTrue(all(speed >= 0 and steer == 0 for speed, steer in commands))

    def test_forward_to_reverse_has_full_stationary_pause(self):
        task = OneShot(make_plan(dict(countdown_seconds=2, forward_seconds=.5,
                                     pause_seconds=.7, forward_speed_raw=18)))
        samples = [(i * .05, task.tick(i * .05)) for i in range(110)]
        forward_end = max(t for t, cmd in samples if cmd[0] > 0)
        reverse_start = min(t for t, cmd in samples if cmd[0] < 0)
        self.assertGreaterEqual(reverse_start - forward_end, .7)
        self.assertTrue(all(cmd == (0, 0) for t, cmd in samples
                            if forward_end < t < reverse_start))

    def test_zero_forward_preserves_original_start(self):
        plan = make_plan(dict(forward_seconds=0))
        self.assertEqual([s.name for s in plan], ['COUNTDOWN', 'REVERSE_INTO_BAY', 'FINAL_STOP'])

    def test_forward_parameters_and_stage_duration_limits(self):
        changes = [dict(forward_speed_raw=0), dict(forward_speed_raw=-12),
                   dict(forward_speed_raw=31), dict(forward_speed_raw=True),
                   dict(forward_seconds=-1), dict(forward_seconds=6),
                   dict(forward_seconds=float('nan')), dict(forward_seconds=float('inf')),
                   dict(forward_only='false'), dict(forward_only=True, forward_seconds=0),
                   dict(repeat_bay_seconds=-1), dict(repeat_bay_seconds=6),
                   dict(repeat_counter_seconds=-1), dict(repeat_counter_seconds=6),
                   dict(repeat_bay_steering_raw=0), dict(repeat_bay_steering_raw=23),
                   dict(repeat_counter_steering_raw=0), dict(repeat_counter_steering_raw=23),
                   dict(repeat_bay_seconds=1, second_seconds=0),
                   dict(repeat_counter_seconds=1, repeat_bay_seconds=0)]
        for change in changes:
            with self.assertRaises(ValueError):
                make_plan(change)

    def test_invalid_values_never_make_plan(self):
        changes = [dict(speed_raw=0), dict(speed_raw=-26), dict(speed_raw=31),
                   dict(speed_raw=True), dict(first_steering_raw=23), dict(first_steering_raw=1.2),
                   dict(first_seconds=0), dict(first_seconds=-1), dict(first_seconds=float('nan')),
                   dict(first_seconds=float('inf')), dict(first_seconds=6), dict(second_seconds=-1),
                   dict(straight_seconds=9), dict(pause_seconds=0), dict(countdown_seconds=0),
                   dict(side='RIGHT'), dict(first_second=1), dict(speed_raw='26')]
        for change in changes:
            with self.assertRaises(ValueError):
                make_plan(change)

    def test_total_over_twelve_seconds_runs_once_then_stops(self):
        plan = make_plan(dict(forward_seconds=5, first_seconds=5,
                              second_seconds=5, straight_seconds=5))
        self.assertEqual(sum(s.seconds for s in plan if s.speed), 20)
        task = OneShot(plan)
        commands = [task.tick(i * .05) for i in range(600)]
        self.assertTrue(task.finished)
        self.assertEqual(task.reason, 'COMPLETE')
        for command in [(12, 0), (-12, -22), (-12, 22), (-12, 0)]:
            self.assertIn(command, commands)
        self.assertEqual(task.tick(100), (0, 0))

    def test_defaults_not_mutated(self):
        before = dict(DEFAULTS)
        make_plan(dict(speed_raw=26, side='left'))
        self.assertEqual(DEFAULTS, before)

    def test_once_and_no_restart(self):
        task = OneShot(make_plan(dict(countdown_seconds=2, first_seconds=.1)))
        commands = [task.tick(i*.05) for i in range(100)]
        self.assertTrue(task.finished)
        self.assertEqual(task.reason, 'COMPLETE')
        self.assertIn((-12, -22), commands)
        self.assertEqual(task.tick(100), (0, 0))
        self.assertEqual(task.reason, 'COMPLETE')

    def test_countdown_is_stationary(self):
        task = OneShot(make_plan())
        for i in range(60):
            self.assertEqual(task.tick(i*.05), (0, 0))

    def test_long_loop_gap_aborts_instead_of_resuming(self):
        task = OneShot(make_plan())
        task.tick(0)
        self.assertEqual(task.tick(.3), (0, 0))
        self.assertEqual(task.reason, 'CONTROL_LOOP_GAP')
        self.assertEqual(task.tick(.35), (0, 0))
        self.assertTrue(task.finished)

    def test_backwards_clock_and_nonfinite_abort(self):
        for when in (-1, float('nan'), float('inf')):
            task = OneShot(make_plan())
            task.tick(0)
            self.assertEqual(task.tick(when), (0, 0))
            self.assertTrue(task.finished)

    def test_operator_abort_latches(self):
        task = OneShot(make_plan())
        task.tick(0)
        task.abort('operator')
        self.assertEqual(task.tick(.05), (0, 0))
        self.assertEqual(task.reason, 'operator')

    def test_ownership_allows_only_dedicated_bridge(self):
        pubs, subs = self.graph()
        require_ownership(pubs, subs, '/test')
        pubs['/ackermann_cmd'].append('/rogue')
        with self.assertRaises(ValueError):
            require_ownership(pubs, subs, '/test')

    def test_autonomy_and_teleop_block_test(self):
        for topic in ('/control/cmd', '/keyboard/control_cmd', '/joystick/control_cmd'):
            pubs, subs = self.graph()
            pubs[topic] = ['/other']
            with self.assertRaises(ValueError):
                require_ownership(pubs, subs, '/test')

    def test_other_test_missing_bridge_or_base_rejected(self):
        pubs, subs = self.graph()
        pubs['/parallel_parking/open_loop_cmd'].append('/other')
        with self.assertRaises(ValueError):
            require_ownership(pubs, subs, '/test')
        for topic in ('/parallel_parking/open_loop_cmd', '/ackermann_cmd'):
            pubs, subs = self.graph()
            subs[topic] = []
            with self.assertRaises(ValueError):
                require_ownership(pubs, subs, '/test')

    def test_ambiguous_status_publisher_rejected(self):
        pubs, subs = self.graph()
        pubs['/base_controller/status'].append('/other')
        with self.assertRaises(ValueError):
            require_ownership(pubs, subs, '/test')

    def test_base_health_and_stale_checks(self):
        require_base(self.healthy(), 1, 1.1)
        for status, received, now in ((None, 1, 1), (self.healthy(), 1, 2), (self.healthy(), 2, 1)):
            with self.assertRaises(ValueError):
                require_base(status, received, now)
        for key, value in [('serial_open', False), ('command_received', False),
                           ('timed_out', True), ('serial_bytes', 0)]:
            status = self.healthy()
            status[key] = value
            with self.assertRaises(ValueError):
                require_base(status, 1, 1.1)

    def test_monotonic_clock_available(self):
        clock = monotonic_clock()
        before = clock()
        self.assertGreaterEqual(clock(), before)

    def cli(self, args):
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_MASTER_URI='http://127.0.0.1:1')
        proc = subprocess.Popen([sys.executable, '-B', os.path.join(MODULE, 'parallel_open_loop_test.py')]+args,
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        out, err = proc.communicate()
        return proc.returncode, out+err

    def test_preview_needs_no_ros_master(self):
        code, text = self.cli(['--speed', '26', '--first-seconds', '0.4'])
        self.assertEqual(code, 0)
        self.assertIn(b'PREVIEW ONLY', text)
        self.assertIn(b'REVERSE_INTO_BAY', text)

    def test_execute_without_tty_is_rejected_before_ros(self):
        code, text = self.cli(['--execute'])
        self.assertNotEqual(code, 0)
        self.assertIn(b'foreground interactive', text)

    def test_forward_only_cli_preview_needs_no_ros(self):
        code, text = self.cli(['--forward-only', '--forward-speed', '18', '--forward-seconds', '0.7'])
        self.assertEqual(code, 0)
        self.assertIn(b'FORWARD_PREPARE', text)
        self.assertNotIn(b'REVERSE_INTO_BAY', text)

    def test_bad_parameter_rejected_before_ros(self):
        code, text = self.cli(['--execute', '--first-seconds', 'nan'])
        self.assertNotEqual(code, 0)
        self.assertIn(b'first_seconds', text)

    def test_slot_cli_preview_includes_signed_distance_and_continuous_turns(self):
        for slot, distance in [('P1', b'+0.850'), ('P2', b'+0.150'), ('P3', b'-0.550')]:
            code, text = self.cli(['--slot', slot, '--speed', '26', '--position-speed', '26'])
            self.assertEqual(code, 0, text)
            self.assertIn(distance, text)
            self.assertIn(b'REVERSE_LEFT_LOCK', text)
            self.assertIn(b'REVERSE_RIGHT_LOCK', text)
            self.assertIn(b'PREVIEW ONLY', text)

    def test_slot_defaults_and_blind_model_share_requested_speed_coefficient(self):
        args,stages=app.arguments(['--slot','P1','--reference-only','--forward-mps-per-raw','.01'])
        self.assertEqual(args.reference_search['seek_speed_raw'],15)
        self.assertEqual(args.reference_search['creep_speed_raw'],15)
        self.assertEqual(args.reference_search['forward_mps_per_raw'],.01)

    def test_slot_cli_default_state_machine_is_30_and_line_search_remains_15(self):
        for slot,distance in [('P1',.85),('P2',.15),('P3',-.55)]:
            args,stages=app.arguments(['--slot',slot])
            self.assertAlmostEqual(args.reference_distance_m,distance)
            self.assertEqual(args.reference_search['seek_speed_raw'],15)
            self.assertEqual(args.reference_search['creep_speed_raw'],15)
            self.assertTrue(all(abs(s.speed)==30 for s in stages if s.speed))

    def test_explicit_slot_speed_overrides_are_preserved(self):
        args,stages=app.arguments(['--slot','P1','--speed','26','--position-speed','20'])
        self.assertEqual(stages[1].speed,20)
        self.assertTrue(all(s.speed==-26 for s in stages if s.name.startswith('REVERSE_')))

    def test_slot_positioning_uses_shared_calibration_at_selected_raw_speed(self):
        from robot.common.config import load_config
        from robot.motion.calibration import speed_gain
        cfg=load_config(os.path.join(os.path.dirname(MODULE),'config'))
        for slot,distance,raw in [('P1',.85,15),('P2',.15,20),('P3',.55,-30)]:
            args,stages=app.arguments(['--slot',slot,'--position-speed',str(abs(raw))])
            self.assertAlmostEqual(stages[1].seconds,distance/(abs(raw)*speed_gain(cfg,raw)))

    def test_slot_rejects_manual_conflicts_or_bad_calibration_before_ros(self):
        for flags in (['--side', 'right'], ['--forward-only'], ['--forward-seconds', '1'],
                      ['--reverse-mps-per-raw', 'nan'], ['--position-speed', '0'],
                      ['--first-steer', '20'], ['--second-seconds', '0']):
            code, text = self.cli(['--slot', 'P3', '--execute'] + flags)
            self.assertNotEqual(code, 0)
            self.assertNotIn(b'foreground interactive', text)

    def test_slot_execute_without_tty_is_rejected(self):
        code, text = self.cli(['--slot', 'P1', '--execute'])
        self.assertNotEqual(code, 0)
        self.assertIn(b'foreground interactive', text)


if __name__ == '__main__':
    unittest.main()
