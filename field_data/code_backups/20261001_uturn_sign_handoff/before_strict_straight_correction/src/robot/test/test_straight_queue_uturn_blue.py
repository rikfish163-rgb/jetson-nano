"""Replay the missed second STRAIGHT and stop UTURN at the front axle."""
import math
import os
import unittest

from robot.common.config import load_config
from robot.common.geometry import local
from robot.master.controller import Controller


class StraightQueueUturnBlueTests(unittest.TestCase):
    def core(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False, sign_ttl=0,
                   intersection_wait_s=1., straight_speed_raw=20)
        core = Controller(cfg)
        self.addCleanup(core.close)
        return core

    def straight(self, x=1.10, stamp=18.):
        core = self.core()
        core.start_follow([(0, 0, 0, 1, 0), (1.25, 0, 0, 1, 0)], 'STRAIGHT', 10.)
        core.consumed_marker = (.35, 0.)
        core.blue_consumed = True
        core.set_pose((x, 0, 0), stamp)
        return core

    def ground(self, core, stamp, x=None, yaw=0., lane=True):
        core.observe_lane([(.4, 0), (.6, 0), (.8, 0)] if lane else [], .9 if lane else .1146, stamp)
        core.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[] if x is None else [dict(kind='junction', x=x, y=0., length=.6)],
            blue_lines=[] if x is None else [dict(x=x, y=0., yaw=yaw, length=.6)]), stamp)

    def votes(self, core, start=18.):
        core.observe_sign('STRAIGHT', .8415427, start, start)
        core.observe_sign('STRAIGHT', .9305515, start+.2, start+.2)

    def test_late_second_straight_is_queued_without_interrupting_current_action(self):
        core = self.straight()
        self.votes(core)
        self.assertEqual(core.next_direction, 'STRAIGHT')
        self.assertEqual(core.action, 'STRAIGHT')
        self.assertIsNone(core.pending)
        self.assertEqual(core.sign_info['decision'], 'next_straight_pending')
        # One wrong LEFT frame and blank frames cannot replace confirmed STRAIGHT.
        core.observe_sign('LEFT', .82, 18.3, 18.3)
        core.observe_sign('', 0., 18.4, 18.4)
        self.assertEqual(core.next_direction, 'STRAIGHT')

    def test_same_early_board_duplicate_or_interrupted_votes_do_not_queue(self):
        core = self.straight(.2, 11.)
        self.votes(core, 11.)
        self.assertIsNone(core.next_direction)
        core.set_pose((1.10, 0, 0), 18.)
        core.observe_sign('STRAIGHT', .99, 18., 18.)
        core.observe_sign('STRAIGHT', .99, 18., 18.1)
        self.assertIsNone(core.next_direction)
        core.observe_sign('', 0., 18.2, 18.2)
        core.observe_sign('STRAIGHT', .99, 18.3, 18.3)
        self.assertIsNone(core.next_direction)

    def test_confirmed_next_blue_hands_over_at_125_without_waiting_for_white(self):
        core = self.straight()
        self.votes(core)
        for stamp in (18.3, 18.5, 18.7):
            self.ground(core, stamp, .8, lane=False)
        self.assertIsNotNone(core.marker)
        self.assertEqual(core.action, 'STRAIGHT')
        self.assertEqual(core.tick(18.7)[0], 20)
        core.set_pose((1.25, 0, 0), 18.8)
        self.ground(core, 18.8, .65, lane=False)
        self.assertGreater(core.tick(18.8)[0], 0)
        self.assertEqual(core.state, 'BLUE_APPROACH')
        self.assertEqual(core.action, 'STRAIGHT')
        self.assertEqual(core.last_blue_trigger['action'], 'STRAIGHT')
        self.assertAlmostEqual(core.last_blue_trigger['point_world'][0], 1.90)

    def test_old_blue_or_longitudinal_paint_cannot_dispatch_queued_straight(self):
        for x, yaw in ((.01, 0.), (.8, math.pi/2)):
            core = self.straight()
            core.consumed_marker = (1.10+x, 0.) if x == .01 else (.35, 0.)
            self.votes(core)
            for stamp in (18.3, 18.5, 18.7):
                self.ground(core, stamp, x, yaw, lane=False)
            self.assertIsNone(core.marker)
            core.set_pose((1.25, 0, 0), 18.8)
            self.assertEqual(core.tick(18.8)[0], 0)
            self.assertEqual(core.state, 'MANEUVER')

    def test_next_straight_survives_white_handoff_until_its_blue_line(self):
        core = self.straight()
        self.votes(core)
        core.set_pose((1.25, 0, 0), 18.3)
        for stamp in (18.3, 18.5, 18.7):
            self.ground(core, stamp)
            core.tick(stamp)
        self.assertEqual(core.state, 'LANE')
        self.assertIsNone(core.action)
        self.assertEqual(core.pending, 'STRAIGHT')
        for stamp in (18.8, 18.9, 19.):
            self.ground(core, stamp, .65, lane=False)
            core.tick(stamp)
        self.assertEqual(core.state, 'BLUE_APPROACH')
        self.assertEqual(core.action, 'STRAIGHT')
        self.assertGreater(core.tick(19.01)[0], 0)

    def arm_uturn(self, x, yaw=0.):
        core = self.core()
        core.pending, core.pending_at = 'UTURN', .9
        for stamp in (1., 1.2, 1.4):
            self.ground(core, stamp, x, yaw)
            core.tick(stamp)
        self.assertEqual(core.action, 'UTURN')
        self.assertEqual(core.blue_approach['phase'], 'STOP_LINE')
        return core

    def test_uturn_stop_depends_on_line_range_and_front_axle_not_bumper(self):
        for x in (.65, 1.10):
            core = self.arm_uturn(x)
            stop_x = x-core.cfg['wheelbase']
            core.set_pose((stop_x-.03, 0, 0), 1.5)
            self.ground(core, 1.5)
            self.assertGreater(core.tick(1.5)[0], 0)
            core.set_pose((stop_x, 0, 0), 1.6)
            self.ground(core, 1.6)
            self.assertEqual(core.tick(1.6), (0, 0.))
            self.assertEqual(core.state, 'BLUE_STOP')
            self.assertEqual(core.blue_approach['stop_reference'], 'front_axle')
            self.assertAlmostEqual(local(core.pose, core.blue_approach['point'])[0], core.cfg['wheelbase'])
            self.ground(core, 2.5)
            self.assertEqual(core.tick(2.5), (0, 0.))
            self.ground(core, 2.61)
            self.assertEqual(core.tick(2.61), (0, 0.))
            self.assertEqual(core.state, 'UTURN')

    def test_uturn_does_not_start_with_front_axle_heading_misaligned(self):
        core = self.arm_uturn(.9, math.radians(15))
        core.set_pose((.9-core.cfg['wheelbase'], 0, 0), 1.5)
        self.ground(core, 1.5)
        core.tick(1.5)
        self.ground(core, 2.6)
        self.assertEqual(core.tick(2.6)[0], 0)
        self.assertNotEqual(core.state, 'UTURN')
        self.assertEqual(core.reason, 'uturn_blue_heading_not_aligned')

    def test_uturn_visual_updates_move_target_plane_and_stale_stream_stops(self):
        core = self.arm_uturn(.9)
        core.set_pose((.1, 0, 0), 1.5)
        # New visual geometry corrects the target from world x=.9 to x=1.0.
        self.ground(core, 1.5, .9)
        self.assertGreater(core.tick(1.5)[0], 0)
        self.assertAlmostEqual(core.blue_approach['point'][0], 1.)
        core.set_pose((.9-core.cfg['wheelbase'], 0, 0), 1.6)
        self.ground(core, 1.6)
        self.assertEqual(core.tick(1.6)[0], 20)
        core.set_pose((1.-core.cfg['wheelbase'], 0, 0), 1.7)
        self.ground(core, 1.7)
        self.assertEqual(core.tick(1.7)[0], 0)
        self.assertEqual(core.state, 'BLUE_STOP')
        self.assertEqual(core.tick(3.1)[0], 0)
        self.assertEqual(core.reason, 'blue_stop_front_stale')

    def test_red_overrides_queued_blue_handoff(self):
        core = self.straight()
        self.votes(core)
        for stamp in (18.3, 18.5, 18.7):
            self.ground(core, stamp, .8)
        core.set_pose((1.25, 0, 0), 18.8)
        core.observe_sign('RED', .99, 18.8, 18.8)
        self.assertEqual(core.tick(18.8)[0], 0)
        self.assertEqual(core.action, 'STRAIGHT')
        self.assertEqual(core.state, 'MANEUVER')


if __name__ == '__main__':
    unittest.main()
