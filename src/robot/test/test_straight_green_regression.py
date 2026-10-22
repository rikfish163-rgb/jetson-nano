import unittest
from test_direction_single_frame import DirectionSingleFrameTests


class StraightGreenRegressionTests(unittest.TestCase):
    core = DirectionSingleFrameTests.__dict__['core']

    def ground(self, c, t, yaw=None):
        c.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[dict(kind='junction', x=.3, y=0)],
            blue_lines=[] if yaw is None else
                [dict(x=.3, y=0, yaw=yaw, length=.8)]), t)

    def start(self, c, yaw=None):
        c.observe_sign('STRAIGHT', .99, 1., 1.)
        self.ground(c, 1.1, yaw)
        return c.tick(1.1)

    def test_blue_without_lane_or_angle_starts_straight_not_old_full_lock(self):
        c = self.core()
        c.gap_steer = -.1
        self.assertEqual(self.start(c), (c.cfg['straight_speed_raw'], 0.))
        self.assertEqual(c.action, 'STRAIGHT')

    def test_visible_lane_cannot_end_straight_before_1_25m(self):
        c = self.core()
        c.observe_lane([(.3,-.03),(.6,-.03)], .99, 1.1)
        speed,steer = self.start(c, .3)
        self.assertGreater(speed, 0)
        self.assertEqual(steer, 0)
        self.assertEqual(c.state, 'MANEUVER')
        c.set_pose((1.24,0,0),1.2)
        c.observe_lane([(.3,-.03),(.6,-.03)], .99, 1.2)
        self.assertEqual(c.tick(1.2)[1],0)
        self.assertEqual(c.state,'MANEUVER')
        c.set_pose((1.25,0,0),1.3)
        c.observe_lane([(.3,-.03),(.6,-.03)], .99, 1.3)
        self.assertLess(c.tick(1.3)[1],0)
        self.assertEqual(c.state,'LANE')

    def test_missing_lane_reacquires_without_waiting_one_second(self):
        c = self.core()
        self.start(c, .3)
        self.assertEqual(c.tick(1.2), (c.cfg['straight_speed_raw'], 0.))
        self.ground(c, 1.3, .3)
        c.set_pose((1.25,0,0),1.3)
        c.observe_lane([(.3,-.03),(.6,-.03)], .99, 1.3)
        self.assertLess(c.tick(1.3)[1], 0)
        self.assertEqual(c.state, 'LANE')

    def test_distance_starts_at_this_trigger_not_vehicle_start(self):
        c=self.core()
        c.set_pose((5.,2.,0.),1.)
        self.start(c)
        for pose in ((5.,4.,0.),(4.,2.,0.),(6.24,2.,0.)):
            c.set_pose(pose,1.2)
            c.observe_lane([(.3,.1),(.6,.1)],.99,1.2)
            self.assertEqual(c.tick(1.2)[1],0.)
            self.assertEqual(c.state,'MANEUVER')
        c.set_pose((6.25,2.,0.),1.3)
        c.observe_lane([(.3,.1),(.6,.1)],.99,1.3)
        c.tick(1.3)
        self.assertEqual(c.state,'LANE')

    def test_missing_lane_past_distance_uses_normal_lane_behavior(self):
        c = self.core()
        self.start(c, -.3)
        c.tick(2.2)
        c.set_pose((3.,0.,0.), 3.)
        for t in (3., 10., c.cfg['action_timeout'] + 5.):
            self.assertEqual(c.tick(t), (0, 0.))
            self.assertIsNone(c.action)
            self.assertIsNone(c.straight_search)

    def test_red_and_estop_still_stop_straight(self):
        for flag in ('red', 'estop'):
            c = self.core()
            self.start(c)
            setattr(c, flag, True)
            self.assertEqual(c.tick(2.), (0, 0.))

    def test_default_green_releases_on_first_fresh_valid_frame(self):
        c = self.core(); c.state = 'WAIT_GREEN'
        c.observe_sign('GREEN', .99, 1., 10.)
        self.assertEqual(c.state, 'WAIT_GREEN')
        c.observe_sign('GREEN', .79, 10., 10.)
        self.assertEqual(c.state, 'WAIT_GREEN')
        c.observe_sign('GREEN', .99, 10., 10.1)
        self.assertEqual(c.state, 'WAIT_GREEN')
        c.observe_sign('GREEN', .8, 10.2, 10.2)
        self.assertEqual(c.state, 'STARTUP_STRAIGHT')
        self.assertEqual(c.sign_info['votes'], 1)

    def test_configured_green_vote_count_is_respected(self):
        c = self.core(); c.state = 'WAIT_GREEN'; c.cfg['sign_votes'] = 2
        c.observe_sign('GREEN', .9, 1., 1.)
        self.assertEqual(c.state, 'WAIT_GREEN')
        c.observe_sign('GREEN', .9, 1.1, 1.1)
        self.assertEqual(c.state, 'STARTUP_STRAIGHT')


if __name__ == '__main__':
    unittest.main()
