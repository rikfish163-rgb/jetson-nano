"""Rear bottom acquisition uses the front S fallback with fresh bay geometry."""
import os
import unittest

from test_core import PACKAGE_DIR
from robot.common.config import load_config
from robot.common.geometry import world
from robot.parking.entry import ExplicitStraightEntry
from robot.parking.reverse_entry import ReverseStraightEntry


class RearBottomAcquisitionTests(unittest.TestCase):
    def cfg(self):
        cfg=load_config(os.path.join(PACKAGE_DIR,'config'))
        cfg.update(parking_p_reverse_s=0.,parking_rear_p_reverse_s=0.,
                   pose_mode='command_model')
        return cfg

    def entry(self):
        e=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.,wait_for_sign=True)
        e.observe_sign('PARKING',.99,10.1,10.1,anchor=(-1.05,0.))
        return e

    def rails(self, depth=1.05, center=0.):
        return [[(-.3,y),(-depth,y)] for y in (center-.19,center+.19)]

    def bottom(self, depth=.85, center=0., half=.19):
        return [(-depth,center-half),(-depth,center+half)]

    def test_same_fragmented_far_end_as_front_s_confirms_in_two_frames(self):
        lines=self.rails()+[self.bottom()]
        front=ExplicitStraightEntry(self.cfg(),(0.,0.,0.),10.,(1.05,0.))
        rear=self.entry()
        for stamp in (10.2,10.4):
            front.observe([[(-x,y) for x,y in line] for line in lines],stamp,(0.,0.,0.))
            rear.observe(lines,stamp,(0.,0.,0.))
            if stamp==10.2:self.assertIsNone(rear.bottom)
        self.assertIsNotNone(front.bottom)
        self.assertIsNotNone(rear.bottom)
        self.assertEqual(rear.bottom_source,'observed_bottom')

    def test_current_single_edge_replaces_old_pair_for_bottom_acquisition(self):
        e=self.entry()
        e.observe(self.rails(),10.2,(0.,0.,0.))
        # Same camera corridor, but accumulated command pose has drifted.
        pose=(-.2,.3,.15)
        lines=self.rails(.65)[:1]+[self.bottom(.65)]
        for stamp in (12.,12.2):
            e.observe([[world(pose,p) for p in line] for line in lines],stamp,pose)
        self.assertIsNone(e.view)
        self.assertIsNotNone(e.single)
        self.assertIsNotNone(e.bottom)
        self.assertAlmostEqual(e.remaining(),.65-.255+.20)

    def test_p_associated_long_single_edge_can_initialize_bottom(self):
        e=self.entry()
        for stamp in (10.2,10.4):
            e.observe(self.rails()[:1]+[self.bottom()],stamp,(0.,0.,0.))
        self.assertIsNone(e.view)
        self.assertIsNotNone(e.bottom)

    def test_command_pose_jump_does_not_reset_consecutive_camera_confirmation(self):
        e=self.entry()
        lines=self.rails()+[self.bottom()]
        e.observe(lines,10.2,(0.,0.,0.))
        pose=(-.2,.2,.2)
        e.observe([[world(pose,p) for p in line] for line in lines],10.4,pose)
        self.assertIsNotNone(e.bottom)

    def test_front_width_tolerance_accepts_slightly_extended_bottom(self):
        e=self.entry()
        for stamp in (10.2,10.4):
            e.observe(self.rails()+[self.bottom(half=.285)],stamp,(0.,0.,0.))
        self.assertIsNotNone(e.bottom)

    def test_p_alone_does_not_promote_unrelated_cross_line(self):
        e=self.entry()
        for stamp in (10.2,10.4):
            e.observe([self.bottom()],stamp,(0.,0.,0.))
        self.assertIsNone(e.bottom)

    def test_mouth_and_neighbor_remain_rejected(self):
        for bottom in (self.bottom(.4),self.bottom(center=.38)):
            e=self.entry()
            for stamp in (10.2,10.4):
                e.observe(self.rails()+[bottom],stamp,(0.,0.,0.))
            self.assertIsNone(e.bottom)

    def test_duplicate_frame_does_not_confirm_bottom(self):
        e=self.entry()
        for stamp in (10.2,10.2):
            e.observe(self.rails()+[self.bottom()],stamp,(0.,0.,0.))
        self.assertIsNone(e.bottom)

    def test_acquired_bottom_refreshes_to_visual_stop(self):
        e=self.entry()
        for stamp in (10.2,10.4):
            e.observe(self.rails()+[self.bottom()],stamp,(0.,0.,0.))
        self.assertIsNotNone(e.bottom)
        # Actual image progress must work even without any model movement.
        for i in range(1,10):
            stamp=10.4+i*.2
            e.observe([self.bottom(.85-i*.08)],stamp,(0.,0.,0.))
            self.assertEqual(e.bottom_stamp,stamp)
            self.assertLess(e.command(stamp+.01,(0.,0.,0.))[0],0)
        e.observe([self.bottom(.055)],12.4,(0.,0.,0.))
        self.assertEqual(e.command(12.41,(0.,0.,0.)),(0,0.))
        self.assertEqual(e.reason,'parking_rear_visual_reference_reached')


if __name__=='__main__':unittest.main()
