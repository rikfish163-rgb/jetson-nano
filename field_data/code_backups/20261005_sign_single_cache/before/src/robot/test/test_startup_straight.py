import copy
import os
import sys
import unittest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.master.controller import Controller


class StartupStraightTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT, 'config', 'competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg.update(wait_green=True, lidar_enabled=False)
        cfg['speed_raw']['lane'] = 26
        cfg['straight_speed_raw'] = 26
        self.c = Controller(copy.deepcopy(cfg))
        self.addCleanup(self.c.close)
        for t in (1., 1.05, 1.1):
            self.c.observe_sign('GREEN', .99, t, t)

    def ground(self, t, x=None, kind='junction'):
        self.c.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[] if x is None else [dict(kind=kind, x=x, y=0)]), t)

    def test_green_starts_zero_steer_even_with_curved_or_missing_lane(self):
        self.ground(1.2)
        self.assertEqual(self.c.state, 'STARTUP_STRAIGHT')
        self.assertEqual(self.c.tick(1.2), (26, 0.0))
        self.c.observe_lane([(.3,.2),(.5,.3),(.7,.4)], .9, 1.3)
        self.ground(1.3)
        self.assertEqual(self.c.tick(1.3), (26, 0.0))

    def test_blue_during_startup_is_consumed_without_restarting_distance(self):
        self.ground(1.2, .8)
        self.assertEqual(self.c.tick(1.2), (26, 0.0))
        self.ground(1.3, .3, 'short')
        self.assertEqual(self.c.tick(1.3), (26, 0.0))
        self.assertIsNone(self.c.pending)
        self.c.set_pose((1.66,0,0),1.4)
        self.ground(1.4, .3)
        self.c.observe_lane([(.3,0),(.5,0),(.7,0)], .9, 1.4)
        self.assertEqual(self.c.tick(1.4), (26, 0.0))
        self.assertEqual(self.c.state, 'LANE')
        self.assertIsNone(self.c.action)
        self.assertIsNone(self.c.pending)
        self.ground(1.6)
        self.c.tick(1.6)
        self.assertEqual(self.c.state, 'LANE')

    def test_missing_lane_is_handled_by_lane_after_startup_distance(self):
        self.ground(1.2,.3)
        self.assertEqual(self.c.tick(1.2), (26,0))
        self.c.set_pose((1.25,0,0),1.3)
        self.ground(1.3)
        self.assertEqual(self.c.tick(1.3), (0,0))
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.startup_started)

    def test_recorded_curved_lane_takes_over_at_distance(self):
        self.ground(1.15,.3)
        self.c.tick(1.15)
        self.c.set_pose((1.25,0,0),1.2)
        self.ground(1.2,.01)
        self.c.observe_lane([(.49,.027),(.62,.028),(.75,.011),
                             (.88,-.025),(1.01,-.079)],.83,1.2)
        self.assertGreater(self.c.tick(1.2)[1],0)
        self.assertEqual(self.c.reason,'tracking_lane')
        self.assertEqual(self.c.state,'LANE')
        self.c.set_pose((1.9,0,0),1.4)
        self.ground(1.4)
        self.c.observe_lane([(.3,0),(.6,0)],.9,1.4)
        self.c.tick(1.4)
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.startup_started)

    def test_blue_before_minimum_is_remembered_but_does_not_end_straight(self):
        self.c.set_pose((.9,0,0),1.2)
        self.ground(1.2,.1)
        self.c.observe_lane([(.3,0),(.6,0)],.9,1.2)
        self.assertEqual(self.c.tick(1.2),(26,0))
        self.assertEqual(self.c.state,'STARTUP_STRAIGHT')
        self.c.set_pose((1.,0,0),1.3)
        self.ground(1.3,.1)
        self.c.tick(1.3)
        self.assertEqual(self.c.startup_origin,(0.,0,0))
        self.c.set_pose((1.25,0,0),10.)
        self.ground(10.)
        self.c.observe_lane([(.3,0),(.6,0)],.9,10.)
        self.c.tick(10.)
        self.assertEqual(self.c.state,'LANE')

    def test_later_sign_does_not_reuse_startup_blue(self):
        self.c.set_pose((1.67,0,0),1.2)
        self.ground(1.2,.01)
        self.c.tick(1.2)
        self.c.set_pose((3.,0,0),1.4)
        self.ground(1.4)
        self.c.observe_sign('STRAIGHT',.99,1.35,1.35)
        self.c.observe_sign('STRAIGHT',.99,1.4,1.4)
        self.c.observe_lane([(.3,0),(.6,0)],.9,1.4)
        self.c.tick(1.4)
        self.assertEqual(self.c.state,'LANE')
        self.assertEqual(self.c.pending,'STRAIGHT')
        self.assertIsNone(self.c.straight_search)

    def test_new_startup_clears_previous_blue(self):
        self.ground(1.2,.1)
        self.c.tick(1.2)
        self.c.state='WAIT_GREEN'
        self.c.observe_sign('GREEN',.99,2.,2.)
        self.assertIsNone(self.c.startup_blue_point)
        self.c.set_pose((1.7,0,0),2.1)
        self.ground(2.1)
        self.c.observe_lane([(.3,0),(.6,0)],.9,2.1)
        self.c.tick(2.1)
        self.assertEqual(self.c.state,'LANE')

    def test_startup_preserves_pending_uturn_for_next_blue(self):
        self.c.observe_sign('UTURN', .8, 1.15, 1.15)
        self.c.observe_sign('UTURN', .8, 1.2, 1.2)
        self.assertEqual(self.c.pending, 'UTURN')
        self.ground(1.3, .8)
        self.assertEqual(self.c.tick(1.3), (26, 0.0))
        self.c.observe_sign('LEFT', .9, 1.4, 1.4)
        self.assertEqual(self.c.pending, 'UTURN')
        self.c.set_pose((1.66,0,0),1.5)
        self.ground(1.5, .3)
        self.assertEqual(self.c.tick(1.5), (0, 0))
        self.c.set_pose((2.92,0,0),1.6)
        self.ground(1.6)
        self.assertEqual(self.c.tick(1.6), (0, 0))
        self.assertEqual(self.c.state, 'LANE')
        self.assertEqual(self.c.pending, 'UTURN')
        self.assertIsNone(self.c.action)
        from blue_test_helpers import enter_blue_action, clear_blue
        clear_blue(self.c,2.1)
        self.c.set_pose((3.3,0,0),2.2)
        enter_blue_action(self.c,'UTURN',2.6)
        self.assertEqual(self.c.state, 'UTURN')
        self.assertEqual(self.c.action, 'UTURN')
        self.assertIsNone(self.c.pending)

    def test_missing_front_and_timeout_stop(self):
        self.assertEqual(self.c.tick(1.2), (0,0))
        self.ground(1.2)
        self.assertEqual(self.c.tick(3), (0,0))
        t = 1.1+self.c.cfg['action_timeout']+.1
        self.ground(t)
        self.assertEqual(self.c.tick(t), (0,0))
        self.assertEqual(self.c.state,'FAULT')

    def test_red_and_estop_keep_priority(self):
        self.ground(1.2)
        self.c.red = True
        self.assertEqual(self.c.tick(1.2), (0,0))
        self.c.red = False
        self.c.estop = True
        self.assertEqual(self.c.tick(1.2), (0,0))

    def test_green_distance_alone_hands_off(self):
        self.c.observe_lane([(.3,.2),(.5,.3)],.99,1.2)
        self.ground(1.2,.3)
        self.assertEqual(self.c.tick(1.2),(26,0))
        self.assertEqual(self.c.state,'STARTUP_STRAIGHT')
        self.c.set_pose((1.66,0,0),1.3)
        self.c.marker=None
        self.ground(1.3)
        self.c.observe_lane([(.3,0),(.6,0)],.99,1.3)
        self.assertEqual(self.c.tick(1.3),(26,0))
        self.assertEqual(self.c.state,'LANE')


if __name__ == '__main__':
    unittest.main()
