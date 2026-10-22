"""Rear visual reference stopping, offline; no ROS or actuator commands."""
import copy
import math
import unittest

from test_core import CONFIG
from robot.parking.reverse_entry import ReverseStraightEntry


class RearBottomReferenceTests(unittest.TestCase):
    def acquire_with_p_anchor(self, anchor):
        cfg = copy.deepcopy(CONFIG)
        cfg['parking_p_reverse_s'] = 0.
        entry = ReverseStraightEntry(cfg, (0., 0., 0.), 10., wait_for_sign=True)
        self.assertTrue(entry.observe_sign('PARKING', .99, 10.1, 10.1,
                                          anchor=(-.9, 0.)))
        # Establish the target bay before testing a drifting P projection.
        entry.observe([[(-.3, y), (-.8, y)] for y in (-.19, .19)],
                      10.15, (0., 0., 0.))
        self.assertIsNotNone(entry.inner.view)
        self.assertTrue(entry.observe_sign('PARKING', .99, 10.16, 10.16,
                                          anchor=anchor))
        line = [(-.8, -.19), (-.8, .19)]
        entry.observe([line], 10.2, (0., 0., 0.))
        entry.observe([line], 10.3, (0., 0., 0.))
        return entry

    def test_far_p_depth_does_not_reject_visible_rear_bottom(self):
        entry = self.acquire_with_p_anchor((-2.61, 0.))
        self.assertIsNotNone(entry.bottom)
        self.assertEqual(entry.bottom_source, 'observed_bottom')

    def test_p_depth_behind_vehicle_does_not_reject_visible_rear_bottom(self):
        entry = self.acquire_with_p_anchor((.109, 0.))
        self.assertIsNotNone(entry.bottom)
        self.assertEqual(entry.bottom_source, 'observed_bottom')

    def entry(self):
        cfg = copy.deepcopy(CONFIG)
        cfg['parking_p_reverse_s'] = 0.
        cfg['rear_bev'].update(bottom_reference_v=318.,
                              bottom_reference_remaining_m=.20)
        entry = ReverseStraightEntry(cfg, (0., 0., 0.), 10.)
        r = cfg['rear_bev']
        depth = -r['axle_x'] + (r['origin_v']-318.)/r['pixels_per_m']
        line = [(-depth, -.19), (-depth, .19)]
        entry.observe([line], 10.1, (0., 0., 0.))
        entry.observe([line], 10.2, (0., 0., 0.))
        self.assertIsNotNone(entry.bottom)
        return entry, depth

    def test_reference_frame_means_twenty_centimetres_remaining(self):
        entry, depth = self.entry()
        entry.command(10.4, (0., 0., 0.))
        self.assertAlmostEqual(entry.remaining(), .20)
        self.assertAlmostEqual(entry.debug['remaining_m'], .20)

    def test_missing_frame_keeps_last_distance_and_stops_at_twenty(self):
        entry, depth = self.entry()
        entry.observe([], 10.5, (-.04, 0., 0.))
        self.assertLess(entry.command(10.6, (-.199, 0., 0.))[0], 0)
        self.assertAlmostEqual(entry.debug['remaining_m'], .001)
        self.assertEqual(entry.command(10.7, (-.20, 0., 0.)), (0, 0.))
        self.assertEqual(entry.reason, 'parking_rear_visual_reference_reached')

    def test_new_measurement_corrects_overestimated_travel(self):
        entry, depth = self.entry()
        pose = (-.10, 0., 0.)
        # Model moved 10 cm; the new image measures only 4 cm of progress.
        measured = pose[0]-(depth-.04)
        entry.observe([[(measured, -.19), (measured, .19)]], 10.5, pose)
        entry.command(10.6, pose)
        self.assertAlmostEqual(entry.remaining(), .16)
        self.assertEqual(entry.debug['distance_source'], 'live_bottom_reference')

    def test_moving_away_adds_distance_instead_of_consuming_it(self):
        entry, depth = self.entry()
        entry.observe([], 10.5, (0., 0., 0.))
        entry.command(10.6, (.05, 0., 0.))
        self.assertAlmostEqual(entry.remaining(), .25)

    def test_far_loss_does_not_trigger_old_fifteen_centimetre_stop(self):
        entry, depth = self.entry()
        # Start 40 cm farther away while retaining the same observed stripe.
        entry.inner.bottom = [(x-.40, y) for x, y in entry.inner.bottom]
        entry.observe([], 10.5, (0., 0., 0.))
        self.assertLess(entry.command(10.6, (-.20, 0., 0.))[0], 0)
        self.assertAlmostEqual(entry.remaining(), .40)

    def test_lost_bottom_uses_full_net_distance_despite_recorded_scale(self):
        entry, depth = self.entry()
        entry.command(10.4, (0., 0., 0.))
        entry.inner.motion_scale = .2243250626042474
        entry.observe([], 10.5, (0., 0., 0.))
        self.assertLess(entry.command(10.6, (-.199, 0., 0.))[0], 0)
        self.assertAlmostEqual(entry.remaining(), .001)
        self.assertEqual(entry.command(10.7, (-.20, 0., 0.)), (0, 0.))

    def test_new_image_does_not_learn_an_extra_distance_scale(self):
        entry, depth = self.entry()
        pose = (-.10, 0., 0.)
        measured = pose[0]-(depth-.04)
        entry.observe([[(measured, -.19), (measured, .19)]], 10.5, pose)
        self.assertAlmostEqual(entry.inner.motion_scale, 1.)
        entry.observe([], 10.6, pose)
        entry.command(10.7, (-.15, 0., 0.))
        self.assertAlmostEqual(entry.remaining(), .11)

    def test_farther_stripe_cannot_restart_the_remaining_distance(self):
        entry, depth = self.entry()
        original_stamp = entry.bottom_stamp
        other = [(-(depth+.30), -.19), (-(depth+.30), .19)]
        for stamp in (10.5, 10.6, 10.7):
            entry.observe([other], stamp, (0., 0., 0.))
        self.assertEqual(entry.bottom_stamp, original_stamp)
        self.assertAlmostEqual(entry.remaining(), .20)
        self.assertGreater(entry.inner.bottom_depth_jump_rejected, 0)

    def test_net_distance_counts_travel_through_a_turn_like_front(self):
        entry, depth = self.entry()
        entry.observe([], 10.5, (0., 0., 0.))
        # Equivalent to the front counter: signed chord length at each pose
        # update, rather than displacement projected onto the original heading.
        points = [(-.03, 0., 0.), (-.06, -.03, math.pi/2),
                  (-.06, -.07, math.pi/2)]
        travelled = .03+math.hypot(.03,.03)+.04
        for i, pose in enumerate(points):
            entry.command(10.6+i*.1, pose)
        self.assertAlmostEqual(entry.remaining(), .20-travelled)

    def test_repeated_control_ticks_do_not_count_the_same_pose_twice(self):
        entry, depth = self.entry()
        entry.observe([], 10.5, (0., 0., 0.))
        for stamp in (10.6, 10.7, 10.8):
            entry.command(stamp, (-.05, 0., 0.))
        self.assertAlmostEqual(entry.remaining(), .15)


if __name__ == '__main__':
    unittest.main()
