"""Green startup follows lane centers until a later junction action completes."""
import os
import unittest

from robot.common.config import load_config
from robot.master.controller import Controller


class StartupParkingGateTests(unittest.TestCase):
    def core(self, wait_green=True):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=wait_green, lidar_enabled=False,
                   parking_enabled=True, parking_mode='forward_plan',
                   straight_distance=1.25, straight_speed_raw=20,
                   lane_curve_speed_raw=30, steering_command_scale_rad=.03)
        cfg['speed_raw']['lane'] = 30
        core = Controller(cfg)
        self.addCleanup(core.close)
        return core

    def vote(self, core, label, start, count=3):
        for i in range(count):
            stamp = start + i*.1
            core.observe_sign(label, .99, stamp, stamp)

    def release(self, core):
        self.vote(core, 'GREEN', 1., int(core.cfg['sign_votes']))
        self.assertEqual(core.state, 'STARTUP_STRAIGHT')

    def lane(self, core, stamp):
        core.observe_lane([(.5, 0), (.7, -.04), (.9, -.12), (1.1, -.24)],
                          .9, stamp,
                          {'LEFT':[(.5, .3), (.7, .3), (.9, .3), (1.1, .3)]})
        core.observe_ground(dict(source='front', part='markers', markers=[], slots=[]), stamp)

    def handoff(self, core, stamp=2.):
        core.set_pose((1.25, 0, 0), stamp)
        self.lane(core, stamp)
        command = core.tick(stamp)
        self.assertEqual(core.state, 'LANE')
        return command

    def test_recorded_parking_votes_cannot_replace_startup_or_center_handoff(self):
        core = self.core()
        self.release(core)
        self.vote(core, 'PARKING', 1.2)
        self.assertIsNone(core.pending)
        self.assertEqual(core.sign_info['votes'], 0)
        self.assertEqual(core.sign_info['decision'], 'parking_wait_route_action')
        self.lane(core, 1.5)
        self.assertEqual(core.tick(1.5), (20, 0.))
        speed, steer = self.handoff(core)
        self.assertEqual(speed, 30)
        self.assertLess(steer, 0.)
        self.assertEqual(core.lane_source, 'center')
        self.vote(core, 'PARKING', 2.1, 8)
        self.assertIsNone(core.pending)
        self.assertEqual(core.sign_info['votes'], 0)

    def test_green_clears_a_preexisting_parking_pending(self):
        core = self.core()
        core.pending, core.pending_at = 'PARKING', .5
        core.park_line_side = 'LEFT'
        self.release(core)
        self.assertIsNone(core.pending)
        self.assertEqual(core.pending_at, 0.)
        self.assertIsNone(core.park_line_side)
        self.handoff(core)
        self.assertEqual(core.lane_source, 'center')

    def test_only_completed_junction_actions_enable_fresh_parking_votes(self):
        for action in ('LEFT', 'RIGHT', 'STRAIGHT', 'UTURN'):
            core = self.core()
            self.release(core)
            self.handoff(core)
            core.state, core.action, core.action_source = 'MANEUVER', action, 'sign'
            self.vote(core, 'PARKING', 2.1)
            self.assertIsNone(core.pending)
            core.resume_lane()
            self.vote(core, 'PARKING', 3., 2)
            self.assertIsNone(core.pending)
            core.observe_sign('PARKING', .99, 3.2, 3.2)
            self.assertEqual(core.pending, 'PARKING')

    def test_bypass_does_not_enable_parking_but_keeps_completed_route_permission(self):
        core = self.core()
        self.release(core)
        self.handoff(core)
        core.action, core.action_source = 'BYPASS', 'obstacle'
        core.resume_lane()
        self.vote(core, 'PARKING', 3.)
        self.assertIsNone(core.pending)
        core.action, core.action_source = 'LEFT', 'sign'
        core.resume_lane()
        core.action, core.action_source = 'BYPASS', 'obstacle'
        core.resume_lane()
        self.vote(core, 'PARKING', 4.)
        self.assertEqual(core.pending, 'PARKING')

    def test_waiting_parking_sign_keeps_center_lane_until_blue_trigger(self):
        core = self.core(wait_green=False)
        self.vote(core, 'PARKING', 1.)
        self.assertEqual(core.pending, 'PARKING')
        self.lane(core, 1.3)
        speed, steer = core.tick(1.3)
        self.assertEqual(speed, 30)
        self.assertLess(steer, 0.)
        self.assertEqual(core.lane_source, 'center')
        self.assertIsNone(core.action)
        core.marker, core.front_marker_stamp = ((.1, 0), 1.4), 1.4
        self.assertEqual(core.dispatch(1.4), (0, 0.))
        self.assertEqual(core.state, 'PARALLEL_PARKING')
        self.assertEqual(core.action, 'PARKING')

    def test_new_green_start_resets_previous_route_permission(self):
        core = self.core(wait_green=False)
        core.action, core.action_source = 'UTURN', 'sign'
        core.resume_lane()
        core.state = 'WAIT_GREEN'
        self.release(core)
        self.handoff(core)
        self.vote(core, 'PARKING', 3.)
        self.assertIsNone(core.pending)

    def test_startup_route_sign_and_red_green_priority_are_preserved(self):
        core = self.core()
        self.vote(core, 'UTURN', .5, 2)
        self.assertEqual(core.pending, 'UTURN')
        self.release(core)
        self.vote(core, 'PARKING', 1.2)
        self.assertEqual(core.pending, 'UTURN')
        self.lane(core, 1.5)
        core.observe_sign('RED', .99, 1.5, 1.5)
        self.assertEqual(core.tick(1.5)[0], 0)
        core.observe_sign('GREEN', .99, 1.6, 1.6)
        self.assertEqual(core.tick(1.6), (20, 0.))


if __name__ == '__main__':
    unittest.main()
