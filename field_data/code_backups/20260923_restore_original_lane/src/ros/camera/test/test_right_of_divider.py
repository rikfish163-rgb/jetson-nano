"""The inner route lies right of the dashed divider, not between divider and outside edge."""
import imp
import os
import unittest


class RightOfDividerTests(unittest.TestCase):
    def setUp(self):
        script = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py')
        self.lane = imp.load_source('right_of_divider_test', script)
        self.lane.configure_lane_windows(40, .15)

    def boundary(self, x):
        return [dict(x=float(x), y=float(380-40*i),
                     usable=True, observed=True, geometry_ok=True, score=1.)
                for i in range(self.lane.NUM_WINDOWS)]

    def test_divider_right_of_outer_edge_becomes_left_reference(self):
        outer = self.boundary(109)
        divider = self.boundary(352)
        left, right = self.lane.select_right_of_divider(outer, divider, 240., 1.)
        self.assertEqual(self.lane.determine_lane_tracking_mode(left, right), 'LEFT_ONLY')
        self.assertEqual(left[0]['x'], 352.)
        self.assertEqual(self.lane.count_reliable_boundary_points(right), 0)

    def test_outer_edge_alone_is_not_a_lane_reference(self):
        left, right = self.lane.select_right_of_divider(self.boundary(92),
            [None]*self.lane.NUM_WINDOWS, 240., 1.)
        self.assertEqual(self.lane.determine_lane_tracking_mode(left, right), 'NO_LANE')

    def test_divider_keeps_left_role_after_crossing_image_center(self):
        empty = [None]*self.lane.NUM_WINDOWS
        for index, chosen in enumerate((352, 320, 285, 250, 215, 180, 145)):
            # The old detector assigns image-half roles. The divider moves
            # from the right half to the left half during the same bend.
            left_raw = self.boundary(chosen) if chosen < 240 else self.boundary(chosen-240)
            right_raw = self.boundary(chosen) if chosen >= 240 else empty
            left, right = self.lane.select_right_of_divider(
                left_raw, right_raw, 240., 1.+.1*index)
            self.assertEqual(self.lane.determine_lane_tracking_mode(left, right), 'LEFT_ONLY')
            self.assertEqual(left[0]['x'], chosen)

    def test_wrong_line_cannot_replace_recent_divider(self):
        empty = [None]*self.lane.NUM_WINDOWS
        self.lane.select_right_of_divider(self.boundary(109),
            self.boundary(352), 240., 1.)
        left, right = self.lane.select_right_of_divider(
            self.boundary(145), empty, 240., 1.1)
        self.assertEqual(self.lane.determine_lane_tracking_mode(left, right), 'NO_LANE')
        left, right = self.lane.select_right_of_divider(
            self.boundary(150), empty, 240., 1.2)
        self.assertEqual(self.lane.determine_lane_tracking_mode(left, right), 'NO_LANE')

    def test_inner_solid_alone_is_not_mistaken_for_divider(self):
        left, right = self.lane.select_right_of_divider(
            [None]*self.lane.NUM_WINDOWS, self.boundary(430), 240., 1.)
        self.assertEqual(self.lane.determine_lane_tracking_mode(left, right), 'NO_LANE')

    def test_two_stripes_without_recent_identity_are_ambiguous(self):
        for left_x, right_x in ((123, 377), (150, 380)):
            self.lane.right_divider_reference = None
            left, right = self.lane.select_right_of_divider(
                self.boundary(left_x), self.boundary(right_x), 240., 1.)
            self.assertEqual(self.lane.determine_lane_tracking_mode(left, right), 'NO_LANE')


if __name__ == '__main__':
    unittest.main()
