"""Production blue approach shares the normal ground-camera freshness budget."""
import os
import unittest

import robot
from robot.common.config import load_config
from robot.master.controller import Controller
from robot.turn.blue_stop_test import BlueStopTest


class BlueCameraBudgetTests(unittest.TestCase):
    def core(self):
        cfg = load_config(os.path.join(os.path.dirname(robot.__file__), 'config'))
        cfg.update(wait_green=False, lidar_enabled=False, intersection_wait_s=.2,
                   steering_command_scale_rad=.03)
        cfg['blue_stop_test']['forward_seconds'] = 1.
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.action, c.state = 'RIGHT', 'BLUE_APPROACH'
        c.blue_approach = dict(image_timed=True, phase='STOP_LINE', point=(1., 0.),
                              yaw=0., observed_stamp=1., started=1.)
        return c

    def frame(self, c, now, row, stamp=None):
        cam = c.cfg['front_camera']
        c.front_marker_stamp = now if stamp is None else stamp
        c.front_blue_image_lines = [] if row is None else [dict(
            x=(cam['origin_v']-row*(cam['bev_height']-1))/cam['pixels_per_m'],
            y=0., yaw=0., length=.8)]
        return c.execute('turn', 'blue_approach_tick', now).value

    def test_recorded_right_approach_camera_gap_recovers(self):
        c = self.core()
        sequence = dict(last=1791298012.421764, start=1791298011.024661,
                        processed=1791298011.8351789, align_started=1791298011.024661,
                        seen_blue=True, aligned=True, align_frames=7,
                        confirmations=0, heading_deg=1.5326296533233679,
                        in_trigger_band=False, previous_row=.6906354409475595)
        c.blue_approach['image_timing'] = sequence
        now = 1791298012.540199
        self.assertGreater(self.frame(c, now, .6906, stamp=1791298011.8351789)[0], 0)
        self.assertEqual(c.state, 'BLUE_APPROACH')
        self.assertEqual(c.reason, 'right_blue_approach')
        self.assertEqual(c.action, 'RIGHT')
        self.assertNotIn('fault', c.blue_approach['image_timing'])
        for delta in (.1, .2, .3):
            self.assertGreater(self.frame(c, now+delta, .72)[0], 0)
        self.assertIn('trigger', c.blue_approach['image_timing'])
        for i in range(4, 18):
            t = now+i*.1
            self.frame(c, t, None)
            if c.state == 'BLUE_STOP':
                break
        self.assertEqual(c.state, 'BLUE_STOP', c.reason)
        t = c.blue_approach['stop_until']+.01
        c.front_marker_stamp = t
        c.tick(t)
        self.assertEqual(c.state, 'MANEUVER')
        c.front_marker_stamp = t+.1
        self.assertLess(c.tick(t+.1)[0], 0)
        self.assertEqual(c.right_lock['phase'], 'TIMED_REVERSE')

    def test_brief_camera_delay_does_not_interrupt_forward_timer(self):
        c = self.core()
        for t in (1., 1.1, 1.2):
            self.frame(c, t, .72)
        trigger = c.blue_approach['image_timing']['trigger']
        for i in range(3, 8):
            self.frame(c, 1.+i*.1, None, stamp=1.2)
        self.frame(c, 1.79, None, stamp=1.2)
        self.assertGreater(self.frame(c, 1.9, None, stamp=1.2)[0], 0)
        self.assertGreater(self.frame(c, 2., None, stamp=1.2)[0], 0)
        self.assertGreater(self.frame(c, 2.1, None)[0], 0)
        sequence = c.blue_approach['image_timing']
        self.assertEqual(sequence['trigger'], trigger)
        self.assertAlmostEqual(sequence['paused_s'], 0.)
        self.assertAlmostEqual(sequence['debug']['elapsed_s'], .9)
        self.assertEqual(self.frame(c, 2.21, None), (0, 0.))
        self.assertEqual(c.state, 'BLUE_STOP')

    def test_camera_loss_beyond_ground_timeout_still_latches_fault(self):
        c = self.core()
        self.frame(c, 1., .5)
        for i in range(1, 13):
            self.frame(c, 1.+i*.1, .5, stamp=1.)
        self.assertEqual(self.frame(c, 2.26, .5, stamp=1.), (0, 0.))
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.reason, 'blue_timed_sensor_lost')
        c.front_marker_stamp = 2.3
        self.assertEqual(c.tick(2.3), (0, 0.))

    def test_emergency_stop_during_brief_camera_delay_is_not_recoverable(self):
        c = self.core()
        self.frame(c, 1., .5)
        for i in range(1, 8):
            self.frame(c, 1.+i*.1, .5, stamp=1.)
        self.assertEqual(c.state, 'BLUE_APPROACH')
        c.estop = True
        self.assertEqual(c.tick(1.8), (0, 0.))
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.reason, 'emergency_stop')

    def test_blue_stop_dispatches_right_within_ground_camera_budget(self):
        c = self.core()
        c.state = 'BLUE_STOP'
        c.blue_approach.update(phase='STOP', stop_until=1.5,
                              image_timing=dict(done=True, last=1.6))
        c.front_marker_stamp = 1.
        self.assertEqual(c.tick(1.7), (0, 0.))
        self.assertEqual(c.state, 'MANEUVER')

    def test_camera_budget_cannot_hide_a_long_control_gap(self):
        c = self.core()
        self.frame(c, 1., .5)
        for i in range(1, 8):
            self.frame(c, 1.+i*.1, .5, stamp=1.)
        self.assertEqual(c.reason, 'right_blue_align')
        self.assertEqual(self.frame(c, 2.21, .5, stamp=1.), (0, 0.))
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.reason, 'blue_timed_control_gap')

    def test_delayed_duplicate_frame_cannot_supply_extra_trigger_votes(self):
        c = self.core()
        for t in (1., 1.1):
            self.frame(c, t, .72)
        self.assertEqual(c.blue_approach['image_timing']['confirmations'], 2)
        for i in range(2, 9):
            self.frame(c, 1.+i*.1, .72, stamp=1.1)
        self.assertEqual(c.blue_approach['image_timing']['confirmations'], 2)
        self.assertNotIn('trigger', c.blue_approach['image_timing'])
        self.frame(c, 1.9, .72)
        self.assertAlmostEqual(c.blue_approach['image_timing']['trigger'], 1.9)

    def test_red_and_lidar_protection_still_block_brief_camera_delay(self):
        for protection in ('red', 'scan'):
            c = self.core()
            self.frame(c, 1., .5)
            for i in range(1, 8):
                self.frame(c, 1.+i*.1, .5, stamp=1.)
            self.assertEqual(c.reason, 'right_blue_align')
            if protection == 'red':
                c.red = True
            else:
                c.cfg['lidar_enabled'] = True
            c.front_marker_stamp = 1.8
            self.assertEqual(c.tick(1.8), (0, 0.))
            self.assertEqual(c.state, 'FAULT')
            c.red = False
            c.cfg['lidar_enabled'] = False
            c.front_marker_stamp = 1.9
            self.assertEqual(c.tick(1.9), (0, 0.))

    def test_production_uses_the_configured_ground_camera_budget(self):
        c = self.core()
        c.cfg['ground_timeout'] = .9
        self.frame(c, 1., .5)
        for i in range(1, 9):
            self.assertGreater(self.frame(c, 1.+i*.1, .5, stamp=1.)[0], 0)
        self.assertEqual(self.frame(c, 1.91, .5, stamp=1.), (0, 0.))
        self.assertEqual(c.reason, 'blue_timed_sensor_lost')

    def test_production_does_not_change_standalone_camera_timeout(self):
        c = self.core()
        self.frame(c, 1., .5)
        for i in range(1, 8):
            command = self.frame(c, 1.+i*.1, .5, stamp=1.)
        self.assertGreater(command[0], 0)
        self.assertEqual(c.cfg['blue_stop_test']['camera_timeout_s'], .6)
        test = BlueStopTest(c.cfg, c.cfg['blue_stop_test'])
        self.addCleanup(test.close)
        test.scan_ready = lambda now: True
        test.checked_command = lambda command, now, allow_bypass: command
        test.observe_ground(dict(source='front', part='markers',
                                 blue_lines=c.front_blue_image_lines), 1.)
        self.assertGreater(test.tick(1.)[0], 0)
        self.assertEqual(test.tick(1.7), (0, 0.))
        self.assertEqual(test.test_result, 'blue_test_sensor_lost')


if __name__ == '__main__':
    unittest.main()
