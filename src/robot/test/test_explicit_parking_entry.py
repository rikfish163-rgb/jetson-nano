"""Parking-module hooks for explicit P4/P5 S and T entry styles."""
import copy
import unittest

from test_core import CONFIG
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class ExplicitParkingEntryTests(unittest.TestCase):
    def core(self, style, slot='P4'):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(parking_mode='forward_center', parking_slot=slot,
                   parking_entry_style=style, lidar_enabled=True,
                   parking_entry_speed_raw=12,
                   steering_command_scale_rad=.03)
        core = Controller(cfg)
        core.last_blue_trigger = dict(stamp=9., vehicle_pose=core.pose, travelled_m=0.)
        core.parking_straight_travel = dict(stamp=9., vehicle_pose=core.pose, travelled_m=0.)
        self.addCleanup(core.close)
        core.scan_ready = lambda now: True
        core.checked_command = lambda command, now, allow_bypass: command
        return core

    def start(self, core, style, trigger):
        result = core.execute('parking', 'start_explicit_parking',
                              10.0, trigger=trigger).value
        self.assertEqual(result, (0, 0.0))
        return core

    def test_s_waits_for_real_bay_geometry(self):
        core = self.start(self.core('S'), 'S', 'sign')
        self.assertEqual(core.execute('parking', 'explicit_parking_tick',
                                      10.1).value, (0, 0.0))
        self.assertEqual(core.state, 'PARKING')
        self.assertEqual(core.reason, 'parking_wait_white_geometry')

    def test_parking_lidar_disabled_allows_s_and_t_without_scan(self):
        for style, slot in (('S', 'P4'), ('T', 'P4'), ('S', 'P5'), ('T', 'P5')):
            core = self.core(style, slot)
            core.cfg['parking_lidar_enabled'] = False
            core.scan_ready = lambda now: False
            # Use the actual command guard rather than the fixture stub.
            core._runtime.overrides.pop('checked_command')
            self.start(core, style, 'sign' if style == 'S' else 'blue')
            if style == 'S':
                core.parking_entry.observe(
                    [[(.40, -.19), (1.00, -.19)],
                     [(.40, .19), (1.00, .19)]], 10.1, core.pose)
            command = core.tick(10.1)
            raw = encode_command(command[0], command[1], core.cfg, 0)
            self.assertEqual(raw['speed_raw'], 12)
            self.assertEqual(raw['steering_raw'], 0 if style == 'S' else -22)
            self.assertEqual(core.state, 'PARKING')
            self.assertEqual(core.obstacle_check['kind'], 'disabled')

    def test_parking_lidar_disabled_keeps_white_geometry_and_estop_gates(self):
        core = self.core('S')
        core.cfg['parking_lidar_enabled'] = False
        core.scan_ready = lambda now: False
        self.start(core, 'S', 'sign')
        self.assertEqual(core.tick(10.1), (0, 0.0))
        self.assertEqual(core.reason, 'parking_wait_white_geometry')
        core.parking_entry.observe(
            [[(.40, -.19), (1.00, -.19)],
             [(.40, .19), (1.00, .19)]], 10.2, core.pose)
        core.estop = True
        self.assertEqual(core.tick(10.2), (0, 0.0))
        self.assertEqual(core.reason, 'emergency_stop')

    def test_parking_lidar_disabled_keeps_preceding_straight_scan_gate(self):
        core = self.core('S')
        core.cfg['parking_lidar_enabled'] = False
        core.scan_ready = lambda now: False
        core.state, core.action, core.action_started = 'MANEUVER', 'STRAIGHT', 10.0
        self.assertEqual(core.tick(10.1), (0, 0.0))
        self.assertEqual(core.reason, 'scan_missing_or_stale')

    def test_lidar_enabled_still_requires_scan_in_s_and_t(self):
        for style in ('S', 'T'):
            core = self.core(style)
            core.cfg['parking_lidar_enabled'] = True
            core.scan_ready = lambda now: False
            self.start(core, style, 'sign' if style == 'S' else 'blue')
            self.assertEqual(core.execute(
                'parking', 'explicit_parking_tick', 10.1).value, (0, 0.0))
            self.assertEqual(core.reason, 'parking_scan_missing_or_stale'
                             if style == 'S' else 'parking_t_entry_scan_stale')

    def test_s_uses_measured_side_lines_once_visible(self):
        core = self.start(self.core('S'), 'S', 'sign')
        lines = [[(.40, -.19), (1.00, -.19)],
                 [(.40, .19), (1.00, .19)]]
        core.parking_entry.observe(lines, 10.1, core.pose)
        command = core.execute('parking', 'explicit_parking_tick',
                               10.1).value
        raw = encode_command(command[0], command[1], core.cfg, 0)
        self.assertEqual(raw['speed_raw'], 12)
        self.assertEqual(raw['steering_raw'], 0)
        self.assertEqual(core.state, 'PARKING')

    def test_s_can_align_on_a_fresh_single_side_associated_with_p(self):
        core = self.core('S')
        core.cfg.update(parking_sign_association=True,
                        parking_entry_speed_raw=16)
        core.parking_sign = dict(point=(.8108, .3417), stamp=10.)
        self.start(core, 'S', 'sign')
        # Parking exposure from 2026-10-09: the car is to the right of P,
        # with only the near bay side recovered at about ten degrees.
        core.parking_entry.observe([[(.5525, .0875), (.8108, .13335)]],
                                   10.1, core.pose)
        self.assertIsNone(core.parking_entry.view)
        self.assertIsNotNone(core.parking_entry.single)
        command = core.execute('parking', 'explicit_parking_tick', 10.1).value
        self.assertEqual(command[0], 16)
        self.assertGreater(command[1], 0.)
        self.assertEqual(core.reason, 'parking_follow_sign_side')
        self.assertLessEqual(abs(encode_command(command[0], command[1],
                                                core.cfg, 0)['steering_raw']), 6)
        self.assertEqual(core.execute('parking', 'explicit_parking_tick',
                                      12.).value, (0, 0.))
        self.assertEqual(core.reason, 'parking_front_stale')

    def test_s_unassociated_single_side_still_waits(self):
        core = self.start(self.core('S'), 'S', 'sign')
        core.parking_entry.observe([[(.5525, .0875), (.8108, .13335)]],
                                   10.1, core.pose)
        self.assertIsNone(core.parking_entry.single)
        self.assertEqual(core.execute('parking', 'explicit_parking_tick',
                                      10.1).value, (0, 0.))
        self.assertEqual(core.reason, 'parking_wait_white_geometry')

    def test_s_discards_white_lines_captured_before_the_sign(self):
        core = self.core('S')
        old_lines = [[(.40, -.19), (1.00, -.19)],
                     [(.40, .19), (1.00, .19)]]
        core.parking_lines, core.parking_lines_stamp = old_lines, 9.9
        self.start(core, 'S', 'sign')
        self.assertEqual(core.execute('parking', 'explicit_parking_tick',
                                      10.1).value, (0, 0.0))
        core.parking_entry.observe(old_lines, 10.1, core.pose)
        command = core.execute('parking', 'explicit_parking_tick',
                               10.1).value
        self.assertGreater(command[0], 0)

    def test_s_production_association_without_anchor_uses_fresh_geometry(self):
        core = self.core('S')
        core.cfg.update(parking_sign_association=True,
                        parking_entry_speed_raw=16,
                        parking_bottom_clearance_m=.09)
        self.start(core, 'S', 'sign')
        lines = [[(.40, -.19), (1.00, -.19)],
                 [(.40, .19), (1.00, .19)],
                 [(1.00, -.19), (1.00, .19)]]
        core.parking_entry.observe(lines, 10.1, core.pose)
        command = core.execute('parking', 'explicit_parking_tick',
                               10.1).value
        raw = encode_command(command[0], command[1], core.cfg, 0)
        self.assertEqual(raw['speed_raw'], 16)
        self.assertEqual(raw['steering_raw'], 0)
        self.assertIsNotNone(core.parking_entry.bottom)

        stop_x = (1. - core.cfg['wheelbase'] - core.cfg['front_overhang'] -
                  core.cfg['parking_bottom_clearance_m'] + .001)
        core.pose = (stop_x, 0., 0.)
        shifted = [[(x - stop_x, y) for x, y in line] for line in lines]
        core.parking_entry.observe(shifted, 10.2, core.pose)
        self.assertEqual(core.execute('parking', 'explicit_parking_tick',
                                      10.2).value, (0, 0.0))
        self.assertEqual(core.state, 'FINISHED')

    def test_s_valid_anchor_keeps_association_and_missing_lines_stops(self):
        core = self.core('S')
        core.cfg['parking_sign_association'] = True
        core.parking_sign = dict(point=(1.0, 0.0), stamp=10.0)
        self.start(core, 'S', 'sign')
        self.assertTrue(core.parking_entry.associate)
        self.assertEqual(core.parking_entry.sign_anchor, (1.0, 0.0))
        self.assertEqual(core.execute('parking', 'explicit_parking_tick',
                                      10.1).value, (0, 0.0))
        self.assertEqual(core.reason, 'parking_wait_white_geometry')

        # An asynchronous projector result must update the wrapped task,
        # rather than create a shadow attribute on ExplicitStraightEntry.
        core.parking_entry.sign_anchor = (1.1, 0.0)
        self.assertEqual(core.parking_entry.inner.sign_anchor, (1.1, 0.0))
        self.assertTrue(core.parking_entry.inner.associate)

        stale = self.core('S')
        stale.cfg['parking_sign_association'] = True
        stale.parking_sign = dict(point=(1.0, 0.0), stamp=8.0)
        self.start(stale, 'S', 'sign')
        self.assertFalse(stale.parking_entry.associate)

    def test_t_cannot_start_from_the_sign_without_blue_trigger(self):
        core = self.core('T')
        core.execute('parking', 'start_explicit_parking', 10.0,
                     trigger='sign')
        self.assertEqual(core.state, 'FAULT')
        self.assertEqual(core.reason, 'parking_explicit_wrong_trigger')

    def test_t_blue_trigger_runs_right_lock_then_finishes(self):
        core = self.start(self.core('T'), 'T', 'blue')
        command = core.execute('parking', 'explicit_parking_tick',
                               10.0).value
        raw = encode_command(command[0], command[1], core.cfg, 0)
        self.assertEqual((raw['speed_raw'], raw['steering_raw']), (12, -22))
        for now in (10.2, 10.4, 10.6, 10.8):
            self.assertNotEqual(core.execute('parking',
                              'explicit_parking_tick', now).value, (0, 0.0))
        self.assertEqual(core.execute('parking', 'explicit_parking_tick',
                                      11.0).value, (0, 0.0))
        self.assertEqual(core.state, 'FINISHED')
        self.assertEqual(core.reason, 'parking_t_entry_complete')

    def test_t_control_gap_faults_and_stops(self):
        core = self.start(self.core('T'), 'T', 'blue')
        core.execute('parking', 'explicit_parking_tick', 10.0)
        self.assertEqual(core.execute('parking', 'explicit_parking_tick',
                                      10.501).value, (0, 0.0))
        self.assertEqual(core.state, 'FAULT')
        self.assertEqual(core.reason, 'parking_t_entry_control_gap')


if __name__ == '__main__':
    unittest.main()
