"""Recover brief scheduler delays without counting expired drive commands."""
import math
import os
import unittest

import test_blue_timed_production as production
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller
from robot.turn.blue_stop_test import BlueStopTest


class BlueTimedControlGapTests(unittest.TestCase):
    def make(self, action='UTURN'):
        cfg = load_config(os.path.join(production.ROOT, 'config'))
        cfg.update(blue_timed_enabled=True,
                   blue_stop_test=dict(production.SETTINGS, forward_seconds=1.),
                   lidar_enabled=False, wait_green=False, intersection_wait_s=.5,
                   steering_command_scale_rad=.03)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.action = action
        c.state = 'BLUE_APPROACH'
        c.blue_approach = dict(image_timed=True, phase='STOP_LINE', point=(1, 0),
                              yaw=0, observed_stamp=1, started=1)
        return c

    def frame(self, c, t, row, angle=0):
        camera = c.cfg['front_camera']
        c.front_marker_stamp = t
        c.front_blue_image_lines = [] if row is None else [dict(
            x=(camera['origin_v']-row*(camera['bev_height']-1))/camera['pixels_per_m'],
            y=0, yaw=math.radians(angle), length=.6)]
        return c.execute('turn', 'blue_approach_tick', t).value

    def trigger(self, c):
        for t in (1., 1.125, 1.25):
            self.frame(c, t, .7)
        self.assertEqual(c.blue_approach['image_timing']['trigger'], 1.25)

    def test_recorded_gap_completes_stop_then_starts_uturn(self):
        c = self.make()
        trigger = 1791269489.108837
        last = 1791269490.035848
        recovered = 1791269490.457125
        c.blue_approach['image_timing'] = dict(trigger=trigger, last=last,
            start=trigger, aligned=True, debug=dict(elapsed_s=last-trigger))
        c.blue_approach['started'] = c.action_started = trigger
        c.next_direction = 'RIGHT'
        self.assertEqual(self.frame(c, recovered, None), (0, 0))
        self.assertEqual(c.state, 'BLUE_STOP')
        sequence = c.blue_approach['image_timing']
        self.assertAlmostEqual(sequence['debug']['elapsed_s'], last-trigger+.25)
        self.assertAlmostEqual(sequence['debug']['paused_s'], recovered-last-.25)
        self.assertEqual(sequence['trigger'], trigger)
        dispatch = c.blue_approach['stop_until']+.01
        c.front_marker_stamp = dispatch
        self.assertEqual(c.tick(dispatch), (0, 0))
        self.assertEqual(c.state, 'UTURN')
        self.assertEqual(c.reason, 'uturn_wait_start')
        c.front_marker_stamp = dispatch+.1
        self.assertEqual(c.tick(dispatch+.1), (0, 0))
        self.assertEqual(c.uturn['phase'], 'GEAR_PAUSE_1')
        for i in range(2, 12):
            c.front_marker_stamp = dispatch+i*.1
            command = c.tick(dispatch+i*.1)
            if c.uturn['phase'] == 'SEGMENT_1':
                break
        raw = encode_command(command[0], command[1], c.cfg, 0)
        self.assertEqual((raw['speed_raw'], raw['steering_raw']), (30, 0))
        self.assertEqual(c.uturn['phase'], 'SEGMENT_1')
        self.assertEqual(c.next_direction, 'RIGHT')

    def test_expired_time_does_not_finish_any_approach_early(self):
        for action in ('LEFT', 'RIGHT', 'STRAIGHT', 'UTURN'):
            c = self.make(action)
            self.trigger(c)
            for t in (1.5, 1.75, 1.875):
                self.frame(c, t, None)
            self.assertGreater(self.frame(c, 2.25, None)[0], 0)
            self.assertEqual(c.state, 'BLUE_APPROACH')
            debug = c.blue_approach['image_timing']['debug']
            self.assertEqual(debug['elapsed_s'], .875)
            self.assertEqual(debug['paused_s'], .125)
            self.assertEqual(debug['control_gap_s'], .375)
            self.assertEqual(self.frame(c, 2.375, None), (0, 0))
            self.assertEqual(c.state, 'BLUE_STOP')

    def test_repeated_delays_count_each_command_at_most_quarter_second(self):
        c = self.make()
        self.trigger(c)
        for t, elapsed in ((1.625, .25), (2., .5), (2.375, .75)):
            self.assertGreater(self.frame(c, t, None)[0], 0)
            self.assertEqual(c.blue_approach['image_timing']['debug']['elapsed_s'], elapsed)
        self.assertEqual(self.frame(c, 2.75, None), (0, 0))
        self.assertEqual(c.state, 'BLUE_STOP')
        self.assertEqual(c.blue_approach['image_timing']['debug']['paused_s'], .5)

    def test_half_second_delay_can_recover(self):
        c = self.make()
        self.trigger(c)
        self.assertGreater(self.frame(c, 1.75, None)[0], 0)
        self.assertEqual(c.state, 'BLUE_APPROACH')
        debug = c.blue_approach['image_timing']['debug']
        self.assertEqual(debug['elapsed_s'], .25)
        self.assertEqual(debug['paused_s'], .25)

    def test_quarter_second_commands_count_the_full_interval(self):
        c = self.make()
        self.trigger(c)
        for t in (1.5, 1.75, 2.):
            self.assertGreater(self.frame(c, t, None)[0], 0)
        self.assertEqual(self.frame(c, 2.25, None), (0, 0))
        self.assertEqual(c.state, 'BLUE_STOP')
        self.assertEqual(c.blue_approach['image_timing']['debug']['paused_s'], 0)

    def test_alignment_recovers_brief_gap_without_triggering_early(self):
        c = self.make()
        self.frame(c, 1., .3, -20)
        self.assertGreater(self.frame(c, 1.421277, .4, -10)[0], 0)
        self.assertEqual(c.state, 'BLUE_APPROACH')
        self.assertNotIn('trigger', c.blue_approach['image_timing'])

    def test_long_gap_or_backward_clock_still_latches_fault(self):
        for timed in (False, True):
            for gap in (.500001, -.01):
                c = self.make()
                if timed:
                    self.trigger(c)
                    last = 1.25
                else:
                    self.frame(c, 1., .3)
                    last = 1.
                now = last+gap
                self.assertEqual(self.frame(c, now, None if timed else .4), (0, 0))
                self.assertEqual(c.state, 'FAULT')
                self.assertEqual(c.reason, 'blue_timed_control_gap')
                c.front_marker_stamp = now+.1
                self.assertEqual(c.tick(now+.1), (0, 0))
                self.assertEqual(c.state, 'FAULT')

    def test_short_gap_cannot_resume_beyond_the_ground_camera_budget(self):
        c = self.make()
        self.trigger(c)
        c.front_marker_stamp = 1.625-c.cfg['ground_timeout']-.01
        self.assertEqual(c.execute('turn', 'blue_approach_tick', 1.625).value, (0, 0))
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.reason, 'blue_timed_sensor_lost')

    def test_test_entry_and_production_recover_the_same_expired_time(self):
        c = self.make()
        test = BlueStopTest(c.cfg, c.cfg['blue_stop_test'])
        self.addCleanup(test.close)
        test.scan_ready = lambda now: True
        test.checked_command = lambda command, now, allow_bypass: command
        for t, row in ((1., .7), (1.125, .7), (1.25, .7),
                       (1.5, None), (1.875, None), (2.125, None), (2.375, None)):
            output = self.frame(c, t, row)
            test.observe_ground(dict(source='front', part='markers',
                                     blue_lines=c.front_blue_image_lines), t)
            self.assertEqual(output, test.tick(t))
            self.assertEqual(c.blue_approach['image_timing']['debug'], test.test_debug)
        self.assertEqual(c.state, 'BLUE_STOP')
        self.assertEqual(test.test_result, 'blue_test_complete')

    def test_recheck_short_gap_does_not_start_forward_time_while_stopped(self):
        c = self.make()
        c.cfg['blue_stop_test']['align_recheck_s'] = 1.
        self.assertEqual(self.frame(c, 1., .7, 18), (0, 0))
        for t in (1.375, 1.5):
            self.assertEqual(self.frame(c, t, .7, 0), (0, 0))
        self.assertGreater(self.frame(c, 1.625, .7, 0)[0], 0)
        sequence = c.blue_approach['image_timing']
        self.assertEqual(sequence['trigger'], 1.625)
        self.assertEqual(sequence['debug']['elapsed_s'], 0)
        self.assertEqual(sequence['debug']['paused_s'], 0)

    def test_obstacle_guard_still_blocks_recovered_command(self):
        c = self.make()
        self.trigger(c)
        c.checked_command = lambda *args: (0, 0)
        self.assertEqual(self.frame(c, 1.625, None), (0, 0))
        self.assertEqual(c.state, 'FAULT')
        self.assertTrue(c.reason.startswith('blue_timed_guard:'))

    def test_estop_red_and_missing_scan_still_take_priority(self):
        for protection, reason in (('estop', 'emergency_stop'),
                                   ('red', 'red_latched'),
                                   ('scan', 'scan_missing_or_stale')):
            c = self.make()
            self.trigger(c)
            c.front_marker_stamp = 1.625
            if protection == 'scan':
                c.cfg['lidar_enabled'] = True
            else:
                setattr(c, protection, True)
            self.assertEqual(c.tick(1.625), (0, 0))
            self.assertEqual(c.reason, reason)
            self.assertEqual(c.state, 'FAULT')


if __name__ == '__main__':
    unittest.main()
