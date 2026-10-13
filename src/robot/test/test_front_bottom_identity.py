"""A front S bottom must not jump to paint behind the locked stripe."""
import copy
import unittest
from test_core import CONFIG
from robot.parking.entry import ExplicitStraightEntry


class FrontBottomIdentityTests(unittest.TestCase):
    def entry(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=0., parking_s_blue_distance_m=0.)
        p = ExplicitStraightEntry(cfg, (0., 0., 0.), 1.)
        for t in (1.1, 1.2):
            p.observe([[ (.55, -.19), (.55, .19)]], t, (0., 0., 0.))
        self.assertIsNotNone(p.bottom)
        return p

    def test_farther_stripe_cannot_reacquire_or_reset_loss(self):
        p = self.entry()
        old = list(p.bottom)
        for t in (1.3, 1.4, 1.5):
            p.observe([[ (.88, -.19), (.88, .19)]], t, (0., 0., 0.))
        self.assertEqual(p.bottom, old)
        self.assertEqual(p.bottom_stamp, 1.2)
        self.assertIsNotNone(p.bottom_loss_origin)
        self.assertEqual(p.command(1.6, (.151, 0., 0.)), (0, 0.))
        self.assertEqual(p.reason, 'parking_bottom_lost_15cm')

    def test_pair_cannot_replace_bottom_even_with_small_farther_jump(self):
        p = self.entry()
        old = list(p.bottom)
        p.observe([[ (.2, -.19), (.63, -.19)],
                   [(.2, .19), (.63, .19)],
                   [(.63, -.19), (.63, .19)]], 1.3, (0., 0., 0.))
        self.assertEqual(p.bottom, old)
        self.assertEqual(p.bottom_stamp, 1.2)

    def test_same_line_survives_retreat_and_forward_approach(self):
        p = self.entry()
        p.observe([[ (.55, -.19), (.55, .19)]], 1.3, (-.1, 0., 0.))
        self.assertEqual(p.bottom_stamp, 1.3)
        far = p.remaining()
        p.observe([[ (.55, -.19), (.55, .19)]], 1.4, (0., 0., 0.))
        self.assertEqual(p.bottom_stamp, 1.4)
        self.assertLess(p.remaining(), far)

    def test_forward_remaining_budget_cannot_grow_on_yaw_change(self):
        p = self.entry()
        p.inner.pose = (.1, 0., 0.)
        before = p.remaining()
        p.inner.pose = (.1, 0., .15)
        self.assertLessEqual(p.remaining(), before)


if __name__ == '__main__':
    unittest.main()
