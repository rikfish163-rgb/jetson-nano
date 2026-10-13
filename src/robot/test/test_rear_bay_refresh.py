"""Recorded rear bay and near-stripe regressions; never starts actuators."""
import json
import os
import unittest
from test_core import PACKAGE_DIR
from robot.common.config import load_config
from robot.parking.reverse_entry import ReverseStraightEntry


class RearBayRefreshTests(unittest.TestCase):
    def entry(self,anchor=(-.9,0.)):
        cfg=load_config(os.path.join(PACKAGE_DIR,'config'))
        cfg.update(parking_p_reverse_s=0.,parking_rear_p_reverse_s=0.,steering_command_scale_rad=.03)
        e=ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)
        e.observe_sign('PARKING',.99,10.1,10.1,anchor=anchor)
        return e

    def rails(self,lo=.3,hi=.8):
        return [[(-lo,y),(-hi,y)] for y in (-.19,.19)]

    def test_recorded_both_sides_and_shared_bottom_refresh_every_frame(self):
        with open(os.path.join(os.path.dirname(__file__),'fixtures','rear_bay_stationary.json')) as f:
            data=json.load(f)
        e=self.entry(tuple(data['anchor']))
        for i,lines in enumerate(data['frames']):
            stamp=10.2+.2*i
            e.observe(lines,stamp,(0.,0.,0.))
            speed,steer=e.command(stamp+.3,(0.,0.,0.))
            self.assertTrue(e.debug['pair_seen'])
            self.assertEqual(e.debug['steering_reference'],'both_sides')
            self.assertEqual(e.bottom_stamp,stamp)
            self.assertEqual(e.debug['distance_source'],'live_bottom_reference')
            self.assertEqual(speed,-16)
            self.assertGreater(steer,0.)

    def test_unassociated_cross_stripe_cannot_start_a_stop_distance(self):
        e=self.entry(anchor=(-3.,0.))
        for stamp in (10.2,10.4,10.6):
            e.observe([[(-.7,-.19),(-.7,.19)]],stamp,(0.,0.,0.))
        self.assertIsNone(e.bottom)
        self.assertLess(e.command(11.,(-.8,0.,0.))[0],0)
        self.assertNotEqual(e.status,'FINISHED')

    def test_near_mouth_does_not_cut_off_visible_pair_or_create_bottom(self):
        e=self.entry()
        for stamp in (10.2,10.4):
            e.observe(self.rails()+[[(-.45,-.19),(-.45,.19)]],stamp,(0.,0.,0.))
            self.assertIsNotNone(e.view)
            self.assertIsNone(e.bottom)

    def test_shared_bottom_updates_without_accepting_adjacent_only_piece(self):
        e=self.entry()
        e.observe(self.rails()+[[(-.8,-.19),(-.8,.19)]],10.2,(0.,0.,0.))
        self.assertIsNotNone(e.bottom)
        e.observe([[(-.8,-.6),(-.8,.6)]],10.4,(0.,0.,0.))
        self.assertEqual(e.bottom_stamp,10.4)
        e.observe([[(-.8,.25),(-.8,.6)]],10.6,(0.,0.,0.))
        self.assertEqual(e.bottom_stamp,10.4)


if __name__=='__main__':unittest.main()
