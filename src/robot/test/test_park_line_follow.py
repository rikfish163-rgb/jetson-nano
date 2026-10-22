import unittest
from test_direction_single_frame import DirectionSingleFrameTests


class ParkLineTests(DirectionSingleFrameTests):
    def test_park_tracks_raw_line_and_locks_side(self):
        c = self.core()
        for t in (.8,.9,1.):c.observe_sign('PARKING', .95, t, t)
        points = [(x, 0.) for x in (.3, .5, .7)]
        right = [(x, -.3) for x in (.3, .5, .7)]
        c.observe_lane(points, 1., 1., {'RIGHT': right})
        speed, steer = c.lane_command(1.)
        self.assertGreater(speed, 0)
        self.assertLess(steer, 0)
        self.assertEqual(c.lane_source, 'park_line_right')
        self.assertEqual(c.left_reference['offset_m'], 0.)
        c.observe_lane(points, 1., 1.1, {'LEFT': points})
        self.assertEqual(c.lane_command(1.1), (0, 0.))
        self.assertEqual(c.reason, 'park_line_missing')

    def test_park_missing_or_stale_line_stops(self):
        c = self.core()
        for t in (.8,.9,1.):c.observe_sign('PARKING', .95, t, t)
        c.observe_lane([(x, 0.) for x in (.3, .5, .7)], 1., 1.)
        self.assertEqual(c.lane_command(1.), (0, 0.))
        c.observe_lane([], 1., 1.1, {'LEFT': [(x, .2) for x in (.3,.5,.7)]})
        self.assertEqual(c.lane_command(3.), (0, 0.))

    def test_normal_lane_still_uses_center(self):
        c = self.core()
        c.observe_lane([(x, 0.) for x in (.3,.5,.7)], 1., 1.,
                       {'LEFT': [(x, .3) for x in (.3,.5,.7)]})
        speed, steer = c.lane_command(1.)
        self.assertGreater(speed, 0)
        self.assertEqual(steer, 0.)


if __name__ == '__main__':
    unittest.main()
