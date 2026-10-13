"""Rear field regressions: cropped rails, image feedback and sign paint."""
import copy
import os
import unittest
import cv2
import numpy as np
from test_core import CONFIG, PACKAGE_DIR
from robot.parking.reverse_entry import ReverseStraightEntry
from robot.common.geometry import world
from robot.camera.vision import GroundDetector


class RearParkingStabilityTests(unittest.TestCase):
    def entry(self, anchor=(-2.4,.06)):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=0.,parking_entry_speed_raw=16,
                   steering_command_scale_rad=.03,pose_mode='command_model')
        e=ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)
        e.observe_sign('PARKING',.95,10.1,10.1,anchor=anchor)
        return e

    def rails(self):
        return [[(-.35,y),(-1.04,y)] for y in (-.13,.25)]

    def test_far_p_does_not_reject_both_rails_cropped_at_bev_far_border(self):
        e=self.entry();e.observe(self.rails(),10.2,(0.,0.,0.))
        speed,steer=e.command(10.5,(0.,0.,0.))
        self.assertIsNotNone(e.view)
        self.assertEqual(e.debug['steering_reference'],'both_sides')
        self.assertEqual(speed,-16)
        self.assertGreater(steer,0.)
        self.assertIsNone(e.bottom)

    def test_far_p_does_not_accept_an_arbitrary_short_near_rectangle(self):
        e=self.entry()
        lines=[[(-.35,y),(-.65,y)] for y in (-.13,.25)]
        e.observe(lines,10.2,(0.,0.,0.))
        self.assertIsNone(e.view)
        self.assertIsNone(e.single)

    def test_command_model_yaw_cannot_reverse_steer_without_a_new_image(self):
        e=self.entry(anchor=(-1.1,.06))
        e.observe(self.rails(),10.2,(0.,0.,0.))
        first=e.command(10.5,(0.,0.,0.))[1]
        # The log changed model yaw by ~18 degrees inside one image delay.
        second=e.command(10.6,(-.02,0.,-.32))[1]
        self.assertGreater(first,0.)
        self.assertAlmostEqual(first,second)
        self.assertEqual(e.debug['steering_pose_source'],'rear_image')

    def test_sign_white_border_is_masked_in_bev_but_ground_paint_remains(self):
        from robot.camera.rear_sign_mask import RearSignMask
        path=os.path.join(PACKAGE_DIR,'..','ros','camera','calibration','rear_bev.yaml')
        mask=RearSignMask(path)
        white=np.full((400,480),255,np.uint8)
        bounds=[356,90,168,180]  # current raw rear frame; board crosses the horizon
        result=mask.apply(white,bounds)
        self.assertLess(np.count_nonzero(result),np.count_nonzero(white))
        self.assertGreater(np.count_nonzero(result),white.size*.5)
        self.assertEqual(result[280,270],255)  # ground bottom below the board
        self.assertEqual(result[60,180],0)  # projected elevated board border
        np.testing.assert_array_equal(mask.apply(white,[0,0,-1,30]),white)

    def test_identical_new_camera_pair_survives_model_yaw_drift(self):
        e=self.entry(anchor=(-1.1,.06))
        e.observe(self.rails(),10.2,(0.,0.,0.))
        first=e.command(10.5,(0.,0.,0.))[1]
        pose=(-.02,0.,-.32)
        e.observe([[world(pose,p) for p in side] for side in self.rails()],10.6,pose)
        self.assertIsNotNone(e.view)
        self.assertAlmostEqual(e.command(10.7,pose)[1],first)

    def test_new_image_center_change_updates_steer_and_keeps_both_sides(self):
        e=self.entry(anchor=(-1.1,.02))
        rails=[[(-.35,y),(-1.04,y)] for y in (-.17,.21)]
        e.observe(rails,10.2,(0.,0.,0.))
        first=e.command(10.5,(0.,0.,0.))[1]
        shifted=[[(x,y-.04) for x,y in line] for line in rails]
        e.observe(shifted,10.6,(0.,0.,0.))
        second=e.command(10.7,(0.,0.,0.))[1]
        self.assertEqual(e.debug['steering_reference'],'both_sides')
        self.assertGreater(first,0.)
        self.assertLess(second,0.)

    def test_rear_hough_duplicates_cannot_starve_a_second_side(self):
        detector=GroundDetector(copy.deepcopy(CONFIG))
        old=cv2.HoughLinesP
        # Old first-80 truncation saw only one thick rail's repeated pixels.
        rows=[[[164+i%4,30,164+i%4,290]] for i in range(90)]
        rows.append([[316,30,316,290]])
        cv2.HoughLinesP=lambda *a,**k:np.array(rows,dtype=np.int32)
        try:lines=detector.parking_lines(np.zeros((400,480),np.uint8),rear=True)
        finally:cv2.HoughLinesP=old
        self.assertTrue(any(p[1]<-.17 for line in lines for p in line))
        self.assertTrue(any(p[1]>.17 for line in lines for p in line))
        self.assertLess(len(lines),10)


if __name__=='__main__':unittest.main()
