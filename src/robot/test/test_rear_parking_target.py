"""Rear P must identify one bay before any paint can steer or stop the car."""
import copy
import unittest

from test_core import CONFIG
from robot.common.geometry import local
from robot.parking.reverse_entry import ReverseStraightEntry


class RearParkingTargetTests(unittest.TestCase):
    def entry(self, anchor=(-.9,.24)):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_slot='P5',parking_entry_style='S',parking_entry_speed_raw=16)
        entry=ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)
        entry.observe_sign('PARKING',.9648,10.1,10.2,anchor=anchor)
        return entry

    def rails(self, center, lo=.3, hi=.85):
        return [[(-lo,y),(-hi,y)] for y in (center-.19,center+.19)]

    def test_p_selects_its_offset_bay_not_nearer_neighbor(self):
        entry=self.entry()
        entry.observe(self.rails(-.14,lo=.2)+self.rails(.24),10.3,(0.,0.,0.))
        entry.command(10.6,(0.,0.,0.))
        self.assertTrue(entry.inner.associate)
        self.assertAlmostEqual(local(entry.inner.pose,entry.inner.view['center'])[1],.24)
        self.assertAlmostEqual(entry.debug['sign_anchor_local'][1],-.24)

    def test_missing_p_position_does_not_adopt_any_bay(self):
        entry=self.entry(anchor=None)
        for stamp in (10.3,10.4):
            entry.observe(self.rails(0.)+[[(-.8,-.19),(-.8,.19)]],stamp,(0.,0.,0.))
        self.assertEqual(entry.command(10.6,(0.,0.,0.)),(-16,0.))
        self.assertIsNone(entry.bottom)
        self.assertIsNone(entry.inner.view)
        self.assertTrue(entry.observe_sign('PARKING',.98,10.7,10.7,anchor=(-.9,.24)))

    def test_target_does_not_change_after_gap_or_second_p(self):
        entry=self.entry()
        entry.observe(self.rails(.24),10.3,(0.,0.,0.))
        entry.command(10.6,(0.,0.,0.))
        self.assertFalse(entry.observe_sign('PARKING',.99,10.7,10.7,anchor=(-.9,-.14)))
        entry.observe(self.rails(-.14),13.,(0.,0.,0.))
        self.assertIsNone(entry.inner.view)
        # Adjacent bays share an edge. It may still guide the original bay,
        # but must never turn into the neighbour's center at -.14.
        self.assertIsNotNone(entry.inner.single)
        self.assertAlmostEqual(local(entry.inner.pose,entry.inner.single['center'])[1],.24)
        entry.observe(self.rails(-.14)[:1],13.1,(0.,0.,0.))
        self.assertIsNone(entry.inner.single)
        self.assertEqual(entry.inner.sign_anchor,(-.9,-.24))

    def test_bottom_must_belong_to_p_bay_and_need_two_frames(self):
        entry=self.entry()
        wrong=[[(-.8,-.33),(-.8,.05)]]
        target=[[(-.8,.05),(-.8,.43)]]
        for stamp in (10.3,10.4):
            entry.observe(self.rails(.24)+wrong,stamp,(0.,0.,0.))
        self.assertIsNone(entry.bottom)
        entry.observe(self.rails(.24)+target,10.5,(0.,0.,0.))
        self.assertIsNone(entry.bottom)
        entry.observe(self.rails(.24)+target,10.6,(0.,0.,0.))
        self.assertIsNotNone(entry.bottom)
        saved=entry.bottom
        entry.observe(wrong+[[(-.8,-.6),(-.8,.6)]],10.7,(0.,0.,0.))
        self.assertEqual(entry.bottom,saved)

    def test_sides_beyond_bottom_cannot_replace_target(self):
        entry=self.entry()
        for stamp in (10.3,10.4):
            entry.observe(self.rails(.24)+[[(-.8,.05),(-.8,.43)]],stamp,(0.,0.,0.))
        entry.observe(self.rails(.24,lo=.85,hi=1.2),10.5,(0.,0.,0.))
        self.assertIsNone(entry.inner.view)
        self.assertIsNone(entry.inner.single)

    def test_bay_mouth_in_front_of_p_is_not_the_bottom(self):
        entry=self.entry()
        for stamp in (10.3,10.4):
            entry.observe(self.rails(.24)+[[(-.45,.05),(-.45,.43)]],stamp,(0.,0.,0.))
        self.assertIsNone(entry.bottom)

    def test_one_side_of_p_bay_can_correct_without_blue_or_pair(self):
        entry=self.entry()
        entry.observe(self.rails(.24)[1:],10.3,(0.,0.,0.))
        speed,steer=entry.command(10.6,(0.,0.,0.))
        self.assertEqual(speed,-16)
        self.assertGreater(steer,0.)
        self.assertEqual(entry.reason,'parking_follow_right_side')
        self.assertIsNone(entry.inner.view)


if __name__=='__main__':unittest.main()
