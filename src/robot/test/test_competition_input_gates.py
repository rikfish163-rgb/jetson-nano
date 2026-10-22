"""Competition regressions: unreliable paths cannot authorize motion/handoff."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from robot.parallel_parking.planner import ParallelParking


class CompetitionInputGateTests(unittest.TestCase):
    def core(self, parking_mode=None):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False)
        if parking_mode is not None:
            cfg['parking_mode'] = parking_mode
        c = Controller(cfg)
        self.addCleanup(c.close)
        return c


    def test_straight_distance_waits_for_distinct_exit_frames(self):
        c = self.core()
        c.start_follow([(0, 0, 0, 1), (1.25, 0, 0, 1)], 'STRAIGHT', 1.)
        c.set_pose((1.3, 0, 0), 2.)
        c.observe_lane([(.3, 0), (.6, 0)], .99, 2.)
        for now in (2., 2.01, 2.02):
            self.assertEqual(c.straight_search_tick(now), (0, 0.))
            self.assertEqual(c.action, 'STRAIGHT')
        for now in (2.1, 2.2):
            c.observe_lane([(.3, 0), (.6, 0)], .99, now)
            c.straight_search_tick(now)
        self.assertIsNone(c.action)
        self.assertEqual(c.last_completed_action, 'STRAIGHT')

    def test_straight_rejects_crossing_or_neighbour_lane(self):
        for points in ([], [(.3, .3), (.6, .3)], [(.3, 0), (.31, .4)]):
            c = self.core()
            c.start_follow([(0, 0, 0, 1), (1.25, 0, 0, 1)], 'STRAIGHT', 1.)
            c.set_pose((1.3, 0, 0), 2.)
            for now in (2., 2.1, 2.2):
                c.observe_lane(points, .99, now)
                self.assertEqual(c.straight_search_tick(now), (0, 0.))
            self.assertEqual(c.action, 'STRAIGHT')

    def test_completed_bypass_still_requires_fresh_scan(self):
        c = self.core()
        c.cfg['lidar_enabled'] = True
        c.cfg['straight_lidar_once'] = False
        c.timed_bypass_completed = True
        c.observe_lane([(.3, 0), (.6, 0)], .99, 1.)
        self.assertEqual(c.tick(1.), (0, 0.))
        self.assertEqual(c.reason, 'scan_missing_or_stale')

    def test_straight_missing_exit_has_bounded_wait(self):
        c = self.core()
        c.start_follow([(0, 0, 0, 1), (1.25, 0, 0, 1)], 'STRAIGHT', 1.)
        c.set_pose((1.3, 0, 0), 2.)
        c.observe_lane([], 0., 2.)
        self.assertEqual(c.straight_search_tick(2.), (0, 0.))
        deadline = 1.+c.straight_search['timeout_s']
        self.assertEqual(c.straight_search_tick(deadline), (0, 0.))
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.reason, 'straight_exit_timeout')

    def test_unconfirmed_side_parking_scene_cannot_start_planner(self):
        c = self.core('parallel_reverse')
        c.action, c.state, c.action_started = 'PARKING', 'PARALLEL_PARKING', 1.
        c.parallel_scene = dict(stamp=1.1, ready=False, occupancy='UNKNOWN')
        self.assertEqual(c.parallel_parking_tick(1.1), (0, 0.))
        self.assertIsNone(c.parallel_future)
        self.assertIsNone(c.parallel_parking)
        self.assertEqual(c.reason, 'parallel_parking_scene_unconfirmed')

    def test_unknown_scene_cannot_confirm_low_level_parking_completion(self):
        c = self.core('parallel_reverse')
        raw = dict(frame='measured_bay', pose_source='vision', stamp=10.,
                   pose=[-.13,.45,0.], ready=False, occupancy='UNKNOWN',
                   regions=[[[-2.,-2.],[2.,-2.],[2.,2.],[-2.,2.]]],
                   obstacles=[], slot=dict(id='P1',pose=[0.,.45,0.],
                                           length=.70,width=.36))
        task = ParallelParking(c.cfg,raw)
        task.phase = 'CONFIRM'
        task.started_at = 10.
        for stamp in (10.1,10.2,10.3):
            task.observe(dict(raw,stamp=stamp))
            self.assertEqual(task.command(stamp),(0,0.))
        self.assertEqual(task.phase,'CONFIRM')
        self.assertEqual(task.confirmations,0)
        self.assertEqual(task.reason,'parallel_parking_scene_unconfirmed')


if __name__ == '__main__':
    unittest.main()
