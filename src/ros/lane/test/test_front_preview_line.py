import math
import unittest
from vehicle_control.pure_pursuit import PurePursuit


class FrontPreviewLineTests(unittest.TestCase):
    def compute(self, path):
        return PurePursuit(.26, .85, front_offset=.33).compute(path)

    def test_intersection_then_nearest_existing_point(self):
        # x=1.18 crosses the segment at (1.18,-.28), nearest vertex is 1.2.
        r = self.compute([(.6,-.7),(1.,-.1),(1.2,-.3),(1.4,-.5)])
        self.assertEqual(r.target_index,2)
        self.assertEqual(r.target_point,(1.2,-.3))
        self.assertAlmostEqual(r.steering_angle,math.atan(.26*2*(-.3)/(1.2**2+.3**2)))

    def test_lateral_distance_does_not_trigger_early_target(self):
        r = self.compute([(.6,.8),(.9,.7),(1.16,.6),(1.3,.5)])
        self.assertEqual(r.target_point,(1.16,.6))

    def test_short_path_uses_nearest_point_to_line(self):
        self.assertEqual(self.compute([(.5,.1),(.8,.2),(1.,.3)]).target_point,(1.,.3))

    def test_all_points_beyond_line(self):
        self.assertEqual(self.compute([(1.3,.1),(1.5,.2)]).target_point,(1.3,.1))

    def test_multiple_crossings_use_first_in_path_order(self):
        self.assertEqual(self.compute([(.5,0),(1.3,.1),(.9,.8),(1.4,1.)]).target_index,1)

    def test_segment_on_line_and_duplicate_points(self):
        self.assertEqual(self.compute([(1.18,.1),(1.18,.1),(1.18,.3)]).target_index,0)

    def test_invalid_front_offset(self):
        for value in (-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                PurePursuit(.26,.85,front_offset=value)


if __name__ == '__main__':
    unittest.main()
