"""Replay an upcoming UTURN visible before the previous straight has ended."""
import math
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller


class StraightUturnHandoffTests(unittest.TestCase):
    def straight(self, x=1.10):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, sign_ttl=0,
                   intersection_wait_s=1., straight_speed_raw=20)
        core = Controller(cfg)
        self.addCleanup(core.close)
        core.start_follow([(0, 0, 0, 1, 0), (1.25, 0, 0, 1, 0)], 'STRAIGHT', 10.)
        core.consumed_marker = (.35, 0.)
        core.blue_consumed = True
        core.set_pose((x, 0, 0), 18.)
        return core

    def ground(self, core, stamp, x=None, yaw=0., lane=True):
        core.observe_lane([(.4, 0), (.6, 0), (.8, 0)] if lane else [], .9 if lane else .1146, stamp)
        core.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[] if x is None else [dict(kind='junction', x=x, y=0., length=.6)],
            blue_lines=[] if x is None else [dict(x=x, y=0., yaw=yaw, length=.6)]), stamp)

    def votes(self, core, label='UTURN', start=18.):
        core.observe_sign(label, .8637654, start, start)
        core.observe_sign(label, .8109708, start+.2, start+.2)

    def test_detected_uturn_is_queued_before_075_and_survives_blank_frames(self):
        core = self.straight(.6985)
        self.votes(core)
        self.assertEqual(core.next_direction, 'UTURN')
        self.assertEqual(core.sign_info['decision'], 'next_uturn_pending')
        self.assertEqual(core.action, 'STRAIGHT')
        self.assertIsNone(core.pending)
        core.observe_sign('', 0., 18.3, 18.3)
        core.observe_sign('LEFT', .99, 18.4, 18.4)
        self.assertEqual(core.next_direction, 'UTURN')

    def test_uturn_blue_cannot_interrupt_125_then_normal_lane_dispatches_next_action(self):
        core = self.straight(.6985)
        self.votes(core)
        # Recorded geometry: line .613 m ahead when straight progress is .699 m.
        for stamp in (18.3, 18.5, 18.7):
            self.ground(core, stamp, .613, lane=False)
            self.assertEqual(core.tick(stamp)[0], 20)
            self.assertEqual(core.action, 'STRAIGHT')
            self.assertEqual(core.state, 'MANEUVER')
        core.set_pose((1.249, 0., 0.), 18.8)
        self.ground(core, 18.8)
        self.assertEqual(core.tick(18.8)[0], 20)
        self.assertEqual(core.action, 'STRAIGHT')
        core.set_pose((1.25, 0., 0.), 18.9)
        self.ground(core, 18.9)
        self.assertGreater(core.tick(18.9)[0], 0)
        self.assertEqual(core.state, 'LANE')
        self.assertIsNone(core.action)
        self.assertEqual(core.pending, 'UTURN')
        for stamp in (19., 19.2, 19.4):
            self.ground(core, stamp, .65)
            core.tick(stamp)
        self.assertEqual(core.action, 'UTURN')
        self.assertEqual(core.blue_approach['phase'], 'STOP_LINE')
        self.assertEqual(core.blue_approach['stop_reference'], 'front_axle')

    def test_uturn_without_confirmed_new_transverse_blue_cannot_interrupt_straight(self):
        for x, yaw in ((None, 0.), (.01, 0.), (.613, math.pi/2)):
            core = self.straight(.6985)
            core.consumed_marker = (.6985+x, 0.) if x == .01 else (.35, 0.)
            self.votes(core)
            for stamp in (18.3, 18.5, 18.7):
                self.ground(core, stamp, x, yaw, lane=False)
                self.assertEqual(core.tick(stamp)[0], 20)
            self.assertEqual(core.action, 'STRAIGHT')
            self.assertIsNone(core.marker)

    def test_queued_uturn_survives_completed_straight_lane_handoff(self):
        core = self.straight()
        self.votes(core)
        core.set_pose((1.25, 0, 0), 18.3)
        for stamp in (18.3, 18.5, 18.7):
            self.ground(core, stamp)
            core.tick(stamp)
        self.assertEqual(core.state, 'LANE')
        self.assertEqual(core.pending, 'UTURN')
        for stamp in (18.8, 19., 19.2):
            self.ground(core, stamp, .65, lane=False)
            core.tick(stamp)
        self.assertEqual(core.action, 'UTURN')
        self.assertEqual(core.state, 'BLUE_APPROACH')

    def test_mixed_direction_votes_cannot_confirm_the_wrong_next_action(self):
        core = self.straight()
        core.observe_sign('STRAIGHT', .99, 18., 18.)
        core.observe_sign('UTURN', .99, 18.2, 18.2)
        self.assertIsNone(core.next_direction)
        core.observe_sign('UTURN', .99, 18.4, 18.4)
        self.assertEqual(core.next_direction, 'UTURN')

    def test_red_blocks_uturn_blue_handoff(self):
        core = self.straight(.6985)
        self.votes(core)
        for stamp in (18.3, 18.5, 18.7):
            self.ground(core, stamp, .613)
        core.observe_sign('RED', .99, 18.8, 18.8)
        self.assertEqual(core.tick(18.8)[0], 0)
        self.assertEqual(core.action, 'STRAIGHT')

    def test_green_startup_still_requires_125_even_with_uturn_and_blue(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=True, lidar_enabled=False, straight_speed_raw=20)
        core = Controller(cfg)
        self.addCleanup(core.close)
        for i in range(int(cfg['sign_votes'])):
            stamp = 1.+i*.1
            core.observe_sign('GREEN', .99, stamp, stamp)
        self.assertEqual(core.state, 'STARTUP_STRAIGHT')
        core.set_pose((.7, 0., 0.), 2.)
        self.votes(core, start=2.)
        for stamp in (2.3, 2.5, 2.7):
            self.ground(core, stamp, .613)
            self.assertGreater(core.tick(stamp)[0], 0)
            self.assertEqual(core.state, 'STARTUP_STRAIGHT')
        self.assertIsNone(core.action)
        core.set_pose((1.25, 0., 0.), 2.8)
        self.ground(core, 2.8)
        core.tick(2.8)
        self.assertEqual(core.state, 'LANE')
        self.assertEqual(core.pending, 'UTURN')


if __name__ == '__main__':
    unittest.main()
