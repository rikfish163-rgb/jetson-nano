"""Forward parking stops at bumper clearance, never just on bottom detection."""
import math
import unittest
import test_direction_single_frame as fixtures
from robot.common.geometry import world
from robot.parking.forward import ForwardParking


class ForwardBottomClearanceTests(unittest.TestCase):
    def parking(self, pose=(0.,0.,0.), bottom_x=1.4):
        c=fixtures.DirectionSingleFrameTests.__dict__['core'](self)
        c.cfg.update(parking_sign_association=True,parking_entry_speed_raw=12,
                     parking_final_speed_raw=8,parking_bottom_clearance_m=.04)
        p=ForwardParking(c.cfg,pose,1.)
        p.sign_anchor=world(pose,(bottom_x+.1,0.))
        lines=[[(.5,-.19),(bottom_x,-.19)],
               [(.5,.19),(bottom_x,.19)],
               [(bottom_x,-.19),(bottom_x,.19)]]
        p.observe([[world(pose,q) for q in line] for line in lines],1.,pose)
        self.assertIsNotNone(p.bottom)
        return c,p

    def test_far_bottom_keeps_centering(self):
        c,p=self.parking()
        self.assertEqual(p.command(1.,c.pose)[0],12)
        self.assertIsNone(p.status)
        self.assertEqual(p.reason,'parking_center_between_sides')
        self.assertAlmostEqual(p.debug['remaining_m'],1.03)

    def test_p_near_cross_does_not_discard_visible_sides(self):
        c,p=self.parking(bottom_x=1.)
        self.assertEqual(p.bottom_source,'p_near_cross')
        self.assertIsNotNone(p.view)
        self.assertEqual(p.command(1.,c.pose)[0],12)
        self.assertEqual(p.reason,'parking_center_between_sides')

    def test_final_speed_then_front_clearance_with_line_out_of_view(self):
        c,p=self.parking()
        for now,x,speed in ((2.,.75,8),(3.,1.02,8),(4.,1.031,0)):
            pose=(x,0.,0.)
            p.observe([],now,pose)  # Fresh image; paint is retained in world coordinates.
            self.assertEqual(p.command(now,pose)[0],speed)
            if speed:self.assertIsNone(p.status)
        self.assertEqual(p.status,'FINISHED')
        self.assertEqual(p.reason,'parking_at_bottom_clearance')
        self.assertAlmostEqual(p.debug['bumper_clearance_m'],.039)
        self.assertEqual(p.debug['distance_source'],'measured_bottom_plus_pose')
        self.assertEqual(p.command(20.,(1.,0.,0.)),(0,0.))

    def test_slanted_bottom_uses_nearest_front_corner(self):
        c,p=self.parking()
        p.bottom=[(1.38,-.2),(1.42,.2)]
        # At rear-axle x=1.02 the center has .05 m; closest corner only .038 m.
        pose=(1.02,0.,0.)
        p.observe([],2.,pose)
        self.assertEqual(p.command(2.,pose),(0,0.))
        self.assertEqual(p.status,'FINISHED')
        self.assertAlmostEqual(p.debug['bumper_clearance_m'],.038)

    def test_world_pose_rotation_preserves_clearance(self):
        origin=(2.,-3.,math.pi/3.)
        c,p=self.parking(origin)
        self.assertEqual(p.command(1.,origin)[0],12)
        pose=tuple(world(origin,(1.031,0.)))+(origin[2],)
        p.observe([],2.,pose)
        self.assertEqual(p.command(2.,pose),(0,0.))
        self.assertAlmostEqual(p.debug['bumper_clearance_m'],.039)

    def test_stale_camera_does_not_report_finished(self):
        c,p=self.parking()
        self.assertEqual(p.command(4.,(1.031,0.,0.)),(0,0.))
        self.assertIsNone(p.status)
        self.assertEqual(p.reason,'parking_front_stale')

    def test_recorded_far_bottom_is_still_over_one_metre_from_stop(self):
        c,p=self.parking()
        p.bottom=[(1.3935568369,.24427090099),(1.4286533213,-.08878237820)]
        self.assertEqual(p.command(1.,c.pose)[0],12)
        self.assertIsNone(p.status)
        self.assertGreater(p.debug['remaining_m'],1.)

    def test_new_measurement_updates_bottom_before_final_approach(self):
        c,p=self.parking(bottom_x=1.)
        p.observe([[(.9,-.19),(.9,.19)]],1.1,c.pose)
        self.assertEqual(p.command(1.1,c.pose)[0],12)
        self.assertAlmostEqual(p.debug['remaining_m'],.53)
        self.assertEqual(p.debug['distance_source'],'live_bottom')

    def test_bottom_no_longer_transverse_does_not_report_finished(self):
        c,p=self.parking()
        pose=(.7,0.,math.pi/2.)
        p.observe([],2.,pose)
        self.assertEqual(p.command(2.,pose),(0,0.))
        self.assertIsNone(p.status)
        self.assertEqual(p.reason,'parking_bottom_geometry_invalid')


if __name__=='__main__':unittest.main()
