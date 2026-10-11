"""Real rear calibration and production pixels-to-steering, without actuators."""
import math
import os
import unittest

import cv2
import numpy as np
import yaml

from robot.common.config import load_config
from robot.common.geometry import bicycle
from robot.camera.vision import GroundDetector
from robot.parking.reverse_entry import ReverseStraightEntry


class RearBevDirectionTests(unittest.TestCase):
    def setUp(self):
        root=os.path.dirname(os.path.dirname(__file__))
        self.cfg=load_config(os.path.join(root,'config'))
        self.cfg.update(parking_slot='P4',parking_entry_style='S',
                        steering_command_scale_rad=.03)
        self.detector=GroundDetector(self.cfg)
        path=os.path.join(root,'..','ros','camera','calibration','rear_bev.yaml')
        with open(path) as stream:
            self.cal=yaml.safe_load(stream)

    def test_real_homography_raw_right_remains_body_left(self):
        # An upright rear camera's image-right is vehicle-left (+body Y).
        # The deployed homography mirrors horizontal pixels before metric().
        points=np.array([[[280.,350.],[380.,350.]]],dtype=np.float32)
        bev=cv2.perspectiveTransform(points,np.array(
            self.cal['homography_rectified_to_bev'],dtype=float))[0]
        self.assertLess(bev[1][0],bev[0][0])
        a,b=[self.detector.metric(u,v,rear=True) for u,v in bev]
        self.assertGreater(b[1],a[1])
        self.assertLess(a[1],0.)
        self.assertGreater(b[1],0.)

    def test_measured_ground_board_maps_to_rear_axle_frame(self):
        # 2026-10-11 capture: 20 mm squares, 8 horizontal x 11 depth corners.
        # Nearest inner row is 285 mm behind the lens ground projection;
        # the centred lens is 50 mm behind the rear axle. Raw image-right
        # is body-left. These are measured pixels, not inverse-H fixtures.
        raw=np.array([[[472.66525,392.95432]],[[275.05731,395.17633]],
                      [[430.16876,302.88577]],[[305.28677,303.38293]]],
                     dtype=np.float64)
        rectified=cv2.undistortPoints(raw,
            np.array(self.cal['camera_matrix']),
            np.array(self.cal['distortion_coefficients']),
            P=np.array(self.cal['rectified_camera_matrix']))
        bev=cv2.perspectiveTransform(rectified,
            np.array(self.cal['homography_rectified_to_bev'])).reshape(-1,2)
        expected=[(-.335,.07),(-.335,-.07),(-.535,.07),(-.535,-.07)]
        for pixel,want in zip(bev,expected):
            actual=self.detector.metric(pixel[0],pixel[1],rear=True)
            for measured,target in zip(actual,want):
                self.assertAlmostEqual(measured,target,delta=.004)

    def command(self,offset=0.,angle=0.,single=False):
        r=self.cfg['rear_bev'];mask=np.zeros((400,480),np.uint8)
        for side in ((.19,) if single else (-.19,.19)):
            pixels=[]
            for depth in (.3,.8):
                x,y=-depth,offset+side-depth*math.tan(angle)
                # Fixed measured BEV convention, independent of tested y_sign.
                pixels.append((int(round(r['origin_u']-y*r['pixels_per_m'])),
                    int(round(r['origin_v']+(x-r['axle_x'])*r['pixels_per_m']))))
            cv2.line(mask,pixels[0],pixels[1],255,3)
        lines=self.detector.parking_lines(mask,min_length_m=.04,rear=True)
        entry=ReverseStraightEntry(self.cfg,(0.,0.,0.),10.)
        entry.observe(lines,10.2,(0.,0.,0.))
        speed,steer=entry.command(10.5,(0.,0.,0.))
        self.assertLess(speed,0)
        return entry,steer

    def test_both_pixel_offsets_steer_rear_towards_target(self):
        for offset in (-.06,.06):
            entry,steer=self.command(offset=offset)
            self.assertTrue(entry.debug['pair_seen'])
            moved=bicycle((0.,0.,0.),-.1,steer,self.cfg['wheelbase'])
            self.assertGreater(moved[1]*offset,0.)

    def test_large_errors_use_full_chassis_range_in_front_and_rear(self):
        from robot.parking.entry import ExplicitStraightEntry
        scale=self.cfg['steering_command_scale_rad']
        raw_limit=self.cfg['steering_raw_limit']
        for offset in (-.18,.18):
            entry,steer=self.command(offset=offset)
            self.assertAlmostEqual(abs(steer)*raw_limit/scale,raw_limit)
            moved=bicycle((0.,0.,0.),-.1,steer,self.cfg['wheelbase'])
            self.assertGreater(moved[1]*offset,0.)
        front=ExplicitStraightEntry(self.cfg,(0.,0.,0.),10.)
        steer=front.inner.limited_steer(dict(theta=0.,center=(.6,.18)))
        self.assertAlmostEqual(abs(steer)*raw_limit/scale,raw_limit)

    def test_both_pixel_headings_turn_towards_bay(self):
        for angle in (-.10,.10):
            entry,steer=self.command(angle=angle)
            moved=bicycle((0.,0.,0.),-.1,steer,self.cfg['wheelbase'])
            self.assertGreater(moved[2]*angle,0.)

    def test_rear_view_right_side_is_the_preferred_side(self):
        entry,steer=self.command(offset=.05,single=True)
        self.assertEqual(entry.reason,'parking_follow_right_side')
        self.assertGreater(steer,0.)
        self.assertTrue(entry.debug['right_seen'])

    def test_front_pixel_mapping_unchanged(self):
        self.assertGreater(self.detector.metric(200,200)[1],
                           self.detector.metric(280,200)[1])

    def test_rear_p_projection_uses_real_rear_intrinsics_and_body_axes(self):
        self.cfg['parking_p_reverse_s']=0.  # Isolate post-retreat geometry.
        from robot.parking.sign_anchor import ParkingSignProjector
        projector=ParkingSignProjector(self.cfg,self.cal)
        left=projector.position([220,180,80,80])
        right=projector.position([360,180,80,80])
        # Raw rear-image right is body-left, as in the independent BEV test.
        self.assertLess(left[0],0.)
        self.assertLess(right[0],0.)
        self.assertLess(left[1],0.)
        self.assertGreater(right[1],0.)
        self.assertGreater(right[1],left[1])
        entry=ReverseStraightEntry(self.cfg,(0.,0.,0.),10.,wait_for_sign=True)
        entry.observe_sign('PARKING',.99,10.1,10.2,anchor=right)
        lines=[[(-.3,y),(-1.05,y)] for y in
               (right[1]-.57,right[1]-.19,right[1]+.19)]
        entry.observe(lines,10.3,(0.,0.,0.))
        unused_speed,steer=entry.command(10.6,(0.,0.,0.))
        self.assertTrue(entry.debug['pair_seen'])
        moved=bicycle((0.,0.,0.),-.1,steer,self.cfg['wheelbase'])
        self.assertGreater(moved[1]*right[1],0.)
        for bounds in (None,[0,100,80,80],[600,100,80,80],[200,100,8,8]):
            self.assertIsNone(projector.position(bounds))

    def test_rear_projection_has_no_height_floor_after_p_trigger(self):
        from robot.parking.sign_anchor import ParkingSignProjector
        rear=ParkingSignProjector(self.cfg,self.cal)
        self.assertIsNotNone(rear.position([360,180,80,8]))
        self.assertIsNone(rear.position([360,180,80,0]))
        front=ParkingSignProjector(self.cfg)
        self.assertIsNone(front.position([360,180,80,8]))

    def test_production_front_and_rear_share_five_cm_stop_and_speed(self):
        self.cfg['parking_p_reverse_s']=0.  # Timed retreat has separate coverage.
        from robot.parking.entry import ExplicitStraightEntry
        front=ExplicitStraightEntry(self.cfg,(0.,0.,0.),10.)
        rear=ReverseStraightEntry(self.cfg,(0.,0.,0.),10.)
        self.assertEqual(self.cfg['parking_entry_speed_raw'],16)
        self.assertEqual(self.cfg['parking_final_speed_raw'],12)
        self.assertAlmostEqual(self.cfg['parking_bottom_clearance_m'],.05)
        lines=[[(.3,-.19),(.8,-.19)],[(.3,.19),(.8,.19)],
               [(.8,-.19),(.8,.19)]]
        front.observe(lines,10.2,(0.,0.,0.))
        rear.observe([[(-x,y) for x,y in side] for side in lines],10.1,(0.,0.,0.))
        rear.observe([[(-x,y) for x,y in side] for side in lines],10.2,(0.,0.,0.))
        self.assertEqual(front.command(10.5,(0.,0.,0.))[0],16)
        self.assertEqual(rear.command(10.5,(0.,0.,0.))[0],-16)
        for entry,direction,bumper in ((front,1,self.cfg['wheelbase']+self.cfg['front_overhang']),
                                       (rear,-1,self.cfg['rear_overhang'])):
            for stamp,clearance in ((10.6,.20),(10.7,.051),(10.8,.049)):
                pose=(direction*(.8-bumper-clearance),0.,0.)
                entry.observe([[(direction*.8,-.19),(direction*.8,.19)]],stamp,pose)
                speed,steer=entry.command(stamp,pose)
                self.assertEqual(speed,direction*12 if clearance>.05 else 0)
                self.assertAlmostEqual(entry.debug['bumper_clearance_m'],clearance)
            self.assertEqual(entry.status,'FINISHED')


if __name__=='__main__':unittest.main()
