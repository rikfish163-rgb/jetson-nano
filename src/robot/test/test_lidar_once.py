"""One completed straight-road bypass consumes only that lidar category."""
import copy
import unittest

from test_core import CONFIG
from test_continuous_obstacles import _SyntheticScan
from robot.master.controller import Controller
from robot.common.contracts import validate_config


class LidarOnceTests(unittest.TestCase):
    def core(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, lidar_enabled=True,
                   straight_lidar_once=True, timed_bypass_enabled=True)
        core = Controller(cfg)
        core.last_blue_trigger = dict(stamp=.5, vehicle_pose=core.pose, travelled_m=0.)
        core.parking_straight_travel = dict(stamp=.5, vehicle_pose=core.pose, travelled_m=0.)
        self.addCleanup(core.close)
        core.observe_lane([(.5, 0), (.7, 0), (.9, 0)], .99, 1.)
        return core

    def test_first_bypass_and_incomplete_bypass_keep_lidar(self):
        core = self.core()
        for state, action in (('LANE', None), ('TIMED_BYPASS', 'BYPASS')):
            core.state, core.action = state, action
            core.scan = None
            self.assertEqual(core.tick(1.), (0, 0.))
            self.assertEqual(core.reason, 'scan_missing_or_stale')
        self.assertFalse(core.timed_bypass_completed)

    def test_completed_bypass_disables_lane_and_straight_guard(self):
        core = self.core()
        core.timed_bypass_completed = True
        for state, action in (('LANE', None), ('BLUE_APPROACH', 'STRAIGHT'),
                              ('MANEUVER', 'STRAIGHT')):
            core.state, core.action = state, action
            for scan in (None, _SyntheticScan(1., [(.2, 0)])):
                core.scan = scan
                self.assertEqual(core.checked_command((12, 0.), 1., True),
                                 (12, 0.))
                self.assertEqual(core.obstacle_check['reason'],
                                 'straight_lidar_consumed')
        core.state, core.action, core.scan = 'LANE', None, None
        self.assertGreater(core.tick(1.)[0], 0)

    def test_successful_handoff_consumes_category_but_waiting_does_not(self):
        core = self.core()
        core.state, core.action = 'TIMED_BYPASS', 'BYPASS'
        core.timed_bypass = dict(phase='REACQUIRE', elapsed_s=0., last=1.,
                                trigger_world=(-1., 0.))
        core.scan = _SyntheticScan(1.)
        core.lane_tracking_ready = lambda now: False
        self.assertEqual(core.execute('obstacle', 'timed_bypass_tick', 1.).value,
                         (0, 0.))
        self.assertFalse(core.timed_bypass_completed)
        core._runtime.overrides.pop('lane_tracking_ready')
        for stamp in (1.1, 1.2, 1.3):
            core.scan = _SyntheticScan(stamp)
            core.observe_lane([(.5, 0), (.7, 0), (.9, 0)], .99, stamp)
            core.execute('obstacle', 'timed_bypass_tick', stamp)
        self.assertTrue(core.timed_bypass_completed)
        self.assertIsNone(core.timed_bypass)
        self.assertEqual(core.state, 'LANE')
        core.scan = None
        core.observe_lane([(.5, 0), (.7, 0), (.9, 0)], .99, 1.4)
        self.assertGreater(core.tick(1.4)[0], 0)

    def test_straight_tick_after_completion_skips_scan_but_not_motion_owner(self):
        core = self.core()
        core.timed_bypass_completed = True
        core.state, core.action = 'MANEUVER', 'STRAIGHT'
        core.straight_search = dict(phase='STRAIGHT_DISTANCE')
        core.straight_search_tick = lambda now: (12, 0.)
        core.scan = None
        self.assertEqual(core.tick(1.), (12, 0.))

    def test_completed_bypass_cannot_trigger_again_even_for_new_target(self):
        core = self.core()
        core.timed_bypass_completed = True
        for stamp in (1., 1.2, 1.4):
            core.observe_lane([(.5, 0), (.7, 0), (.9, 0)], .99, stamp)
            core.scan = _SyntheticScan(stamp, [(.49, 0)])
            self.assertIsNone(core.execute(
                'obstacle', 'begin_timed_bypass', stamp).value)
        self.assertIsNone(core.timed_bypass)
        self.assertEqual(core.state, 'LANE')

    def test_completed_bypass_keeps_other_actions_and_restored_parking_guard(self):
        core = self.core()
        core.timed_bypass_completed = True
        core.cfg.update(parking_mode='forward_center', parking_slot='P4',
                        parking_entry_style='S', parking_lidar_enabled=True)
        for state, action in (('MANEUVER', 'LEFT'), ('MANEUVER', 'RIGHT'),
                              ('UTURN', 'UTURN'), ('PARKING', 'PARKING'),
                              ('BLUE_APPROACH', 'PARKING')):
            core.state, core.action = state, action
            core.scan = None
            self.assertEqual(core.checked_command((12, 0.), 1., False), (0, 0.))
            self.assertEqual(core.tick(1.), (0, 0.))
            self.assertEqual(core.reason, 'scan_missing_or_stale')

    def test_completion_keeps_estop_and_red_and_new_run_rearms(self):
        core = self.core()
        core.timed_bypass_completed = True
        core.estop = True
        self.assertEqual(core.tick(1.), (0, 0.))
        self.assertEqual(core.reason, 'emergency_stop')
        core.estop, core.red = False, True
        self.assertEqual(core.tick(1.), (0, 0.))
        self.assertEqual(core.reason, 'red_latched')
        fresh = self.core()
        self.assertFalse(fresh.timed_bypass_completed)
        self.assertEqual(fresh.tick(1.), (0, 0.))
        self.assertEqual(fresh.reason, 'scan_missing_or_stale')

    def test_once_setting_can_be_disabled(self):
        core = self.core()
        core.cfg['straight_lidar_once'] = False
        core.timed_bypass_completed = True
        self.assertEqual(core.tick(1.), (0, 0.))
        self.assertEqual(core.reason, 'scan_missing_or_stale')

    def test_switches_reject_strings_and_integers(self):
        for key in ('straight_lidar_once', 'parking_lidar_enabled'):
            for value in ('false', 0):
                cfg = copy.deepcopy(CONFIG)
                cfg[key] = value
                with self.assertRaises(ValueError):
                    validate_config(cfg)
