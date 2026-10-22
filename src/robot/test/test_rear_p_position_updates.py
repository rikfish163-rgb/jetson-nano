"""Rear P triggers once, then refreshes the same target without replacing rails."""
import copy
import unittest

from test_core import CONFIG
from robot.parking.reverse_entry import ReverseStraightEntry


class RearPPositionUpdatesTests(unittest.TestCase):
    def entry(self, prelude=0.):
        cfg=copy.deepcopy(CONFIG)
        cfg['parking_p_reverse_s']=prelude
        return ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)

    def test_small_initial_box_is_corrected_when_same_p_grows(self):
        entry=self.entry()
        self.assertTrue(entry.observe_sign('PARKING',.95,10.1,10.1,
            anchor=(-2.56,-.51),bounds=[447,175,38,38]))
        self.assertTrue(entry.observe_sign('PARKING',.97,10.4,10.4,
            anchor=(-.85,0.),bounds=[437,165,58,58]))
        self.assertEqual(entry.inner.sign_anchor,(-.85,0.))
        self.assertEqual(entry.parking_started,10.1)
        self.assertEqual(entry.rear_sign['position_updates'],2)
        lines=[[(-.30,-.19),(-.85,-.19)],
               [(-.30,.19),(-.85,.19)],
               [(-.85,-.19),(-.85,.19)]]
        entry.observe(lines,10.5,(0.,0.,0.))
        entry.command(10.6,(0.,0.,0.))
        self.assertTrue(entry.debug['pair_seen'])
        self.assertEqual(entry.debug['steering_reference'],'both_sides')

    def test_refresh_keeps_prelude_and_geometry_boundary(self):
        entry=self.entry(prelude=3.)
        entry.observe_sign('PARKING',.95,10.1,10.1,
            anchor=(-1.,0.),bounds=[200,100,80,80])
        entry.command(10.5,(0.,0.,0.))
        entry.command(10.9,(0.,0.,0.))
        before=(entry.parking_started,entry.geometry_after,entry.p_reverse_phase,
                entry.p_reverse_until,entry.p_reverse_elapsed)
        self.assertTrue(entry.observe_sign('PARKING',.97,11.,11.,
            anchor=(-.85,0.),bounds=[195,95,90,90]))
        self.assertEqual(before,(entry.parking_started,entry.geometry_after,
            entry.p_reverse_phase,entry.p_reverse_until,entry.p_reverse_elapsed))

    def test_different_p_box_cannot_replace_target(self):
        entry=self.entry()
        entry.observe_sign('PARKING',.95,10.1,10.1,
            anchor=(-1.,0.),bounds=[447,175,38,38])
        self.assertFalse(entry.observe_sign('PARKING',.99,10.4,10.4,
            anchor=(-.8,.5),bounds=[50,175,38,38]))
        self.assertEqual(entry.inner.sign_anchor,(-1.,0.))

    def test_existing_measured_pair_stays_the_steering_reference(self):
        entry=self.entry()
        entry.observe_sign('PARKING',.95,10.1,10.1,
            anchor=(-.9,0.),bounds=[200,100,80,80])
        lines=[[(-.3,-.19),(-.9,-.19)],[(-.3,.19),(-.9,.19)]]
        entry.observe(lines,10.2,(0.,0.,0.))
        measured_pair=entry.inner.tracked_view
        self.assertIsNotNone(measured_pair)
        self.assertTrue(entry.observe_sign('PARKING',.97,10.3,10.3,
            anchor=(-1.5,.5),bounds=[195,95,90,90]))
        self.assertIs(entry.inner.tracked_view,measured_pair)
        entry.observe(lines,10.4,(0.,0.,0.))
        entry.command(10.5,(0.,0.,0.))
        self.assertEqual(entry.debug['steering_reference'],'both_sides')


if __name__=='__main__':unittest.main()
