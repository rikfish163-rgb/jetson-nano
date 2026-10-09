"""Bend direction comes from curvature, never image half or lateral offset."""
import imp
import os
import unittest


class BendMemoryTests(unittest.TestCase):
    def setUp(self):
        self.lane = imp.load_source('bend_memory_test', os.path.join(
            os.path.dirname(__file__), '..', 'scripts', 'camera_yihan_web.py'))
        self.lane.configure_lane_windows(40, .15)

    def boundary(self, curve, shift=0):
        points = []
        for i in range(self.lane.NUM_WINDOWS):
            v = 380-40*i
            x = (600-v)/400.
            points.append(dict(x=240-400*(shift+curve*(x-.5)**2), y=v,
                               usable=True, observed=True, geometry_ok=True, score=1.))
        return points

    def test_curvature_is_independent_of_lateral_location(self):
        for shift in (-.3, .3):
            self.assertEqual(self.lane.boundary_bend(self.boundary(-.3, shift)), -1)
            self.assertEqual(self.lane.boundary_bend(self.boundary(.3, shift)), 1)
            self.assertEqual(self.lane.boundary_bend(self.boundary(0, shift)), 0)

    def test_confirmation_gap_exit_and_opposite_bend(self):
        lane = self.lane
        right = self.boundary(-.3)
        left = self.boundary(.3)
        empty = [None]*lane.NUM_WINDOWS
        for t in (1., 1.1):
            self.assertEqual(lane.update_bend_memory(right, empty, t)['direction'], 0)
        self.assertEqual(lane.update_bend_memory(right, empty, 1.2)['direction'], -1)
        self.assertEqual(lane.update_bend_memory(empty, empty, 1.3)['direction'], -1)
        self.assertEqual(lane.update_bend_memory(left, empty, 1.4)['direction'], -1)
        lane.update_bend_memory(left, empty, 1.5)
        self.assertEqual(lane.update_bend_memory(left, empty, 1.6)['direction'], 1)
        for i in range(5):
            state = lane.update_bend_memory(self.boundary(0), empty, 1.7+i*.1)
        self.assertEqual(state['direction'], 0)
        for t in (3., 3.1, 3.2):
            lane.update_bend_memory(right, empty, t)
        self.assertEqual(lane.update_bend_memory(empty, empty, 4.3)['direction'], 0)

    def test_conflicting_boundaries_and_duplicate_frames_do_not_confirm(self):
        lane = self.lane
        for t in (1., 1.1, 1.2):
            self.assertEqual(lane.update_bend_memory(self.boundary(-.3),
                self.boundary(.3), t)['direction'], 0)
        for _ in range(5):
            self.assertEqual(lane.update_bend_memory(self.boundary(-.3),
                [None]*lane.NUM_WINDOWS, 2.)['direction'], 0)

    def test_confirmed_bend_preserves_role_during_larger_image_motion(self):
        lane = self.lane
        previous = self.boundary(-.3)
        empty = [dict(p, x=None, usable=False, observed=False) for p in previous]
        for t in (1., 1.1, 1.2):
            lane.update_bend_memory(previous, empty, t)
        lane.boundary_association = (1.2, previous, empty)
        moved = [dict(p, x=p['x']+95) for p in previous]
        lane.update_bend_memory(empty, moved, 1.3)
        left, right = lane.associate_boundary_roles(empty, moved, 1.3, 240.)
        self.assertEqual(lane.determine_lane_tracking_mode(left, right), 'LEFT_ONLY')
        # An unrelated stripe cannot be relabeled merely because it curves right.
        unrelated = [dict(p, x=p['x']+180) for p in previous]
        left, right = lane.associate_boundary_roles(empty, unrelated, 1.3, 240.)
        self.assertEqual(lane.determine_lane_tracking_mode(left, right), 'RIGHT_ONLY')
        # Opposite curvature does not receive the confirmed-right-bend allowance.
        lane.update_bend_memory(empty, self.boundary(.3), 1.4)
        left, right = lane.associate_boundary_roles(empty, moved, 1.4, 240.)
        self.assertEqual(lane.determine_lane_tracking_mode(left, right), 'RIGHT_ONLY')


if __name__ == '__main__':
    unittest.main()
