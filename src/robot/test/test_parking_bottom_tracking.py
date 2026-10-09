"""Keep measuring the selected bottom after the cached P anchor drifts."""
import unittest
import test_parking_sign_anchor as fixtures
from robot.common.geometry import world


class ParkingBottomTrackingTests(unittest.TestCase):
    def test_confirmed_pair_keeps_centering_when_cached_p_drifts(self):
        c,p=fixtures.ParkingAnchorTests.__dict__['parking'](self,anchor=(1.,0.))
        rails=[[(.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)]]
        p.observe(rails,1.,c.pose);p.command(1.,c.pose)
        p.sign_anchor=(.7,-.3)
        pose=(0.,-.06,0.)  # Car is right of its confirmed bay center.
        p.observe(rails,1.2,pose)
        speed,steer=p.command(1.2,pose)
        self.assertIsNotNone(p.view)
        self.assertEqual(p.reason,'parking_center_between_sides')
        self.assertGreater(steer,0.)

    def test_confirmed_pair_rejects_adjacent_bay_and_lines_past_bottom(self):
        c,p=fixtures.ParkingAnchorTests.__dict__['parking'](self,anchor=(1.,0.))
        rails=[[(.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)]]
        p.observe(rails+[[(1.,-.19),(1.,.19)]],1.,c.pose)
        p.command(1.,c.pose)
        for lines in ([[ (.4,.21),(1.,.21)],[(.4,.59),(1.,.59)]],
                      [[(1.2,-.19),(1.5,-.19)],[(1.2,.19),(1.5,.19)]]):
            p.observe(lines,p.stamp+.1,c.pose)
            self.assertIsNone(p.view)
            self.assertEqual(p.command(p.stamp,c.pose)[1],0.)

    def test_production_parking_slows_then_stops_five_cm_earlier(self):
        import os
        from robot.common.config import load_config
        from robot.parking.forward import ForwardParking
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        self.assertEqual(cfg['parking_entry_speed_raw'],16)
        self.assertEqual(cfg['parking_final_speed_raw'],12)
        self.assertAlmostEqual(cfg['parking_bottom_clearance_m'],.09)
        p=ForwardParking(cfg,(0.,0.,0.),1.);p.sign_anchor=(1.,0.)
        p.observe([[ (.85,-.3),(.85,.3)]],1.,(0.,0.,0.))
        self.assertEqual(p.command(1.,(0.,0.,0.))[0],16)
        p.observe([],1.2,(.20,0.,0.))
        self.assertEqual(p.command(1.2,(.20,0.,0.))[0],12)
        p.observe([],1.4,(.42,0.,0.))
        self.assertGreater(p.command(1.4,(.42,0.,0.))[0],0)
        p.observe([],1.6,(.431,0.,0.))
        self.assertEqual(p.command(1.6,(.431,0.,0.)),(0,0.))
        self.assertAlmostEqual(p.debug['bumper_clearance_m'],.089)

    def parking(self):
        c,p=fixtures.ParkingAnchorTests.__dict__['parking'](self,anchor=(1.,0.))
        p.cfg.update(parking_entry_speed_raw=16,parking_final_speed_raw=16)
        p.observe([[ (.85,-.3),(.85,.3)]],1.,c.pose)
        self.assertIsNotNone(p.bottom)
        p.command(1.,c.pose)
        return c,p

    def test_visible_bottom_updates_despite_speed_model_overestimating_travel(self):
        c,p=self.parking()
        # Model advances 0.6 m, real image only 0.3 m. Cached P falls behind
        # the visible bottom; it must no longer veto an established line track.
        for i in range(1,11):
            pose=(.06*i,0.,0.)
            line=[(.85-.03*i,-.3),(.85-.03*i,.3)]
            p.observe([[world(pose,q) for q in line]],1.+i*.2,pose)
            self.assertGreater(p.command(1.+i*.2,pose)[0],0)
            self.assertIsNone(p.status)
            self.assertAlmostEqual(p.bottom_stamp,1.+i*.2)
        self.assertAlmostEqual(p.debug['bumper_clearance_m'],.22)
        self.assertEqual(p.bottom_source,'tracked_bottom')
        self.assertEqual(p.debug['distance_source'],'live_bottom')
        self.assertAlmostEqual(p.motion_scale,.5)
        # No paint in the near blind region: consume visually calibrated travel,
        # not the model's exaggerated displacement.
        p.observe([],3.2,(.9,0.,0.))
        self.assertGreater(p.command(3.2,(.9,0.,0.))[0],0)
        self.assertAlmostEqual(p.debug['bumper_clearance_m'],.07)
        p.observe([],3.4,(.97,0.,0.))
        self.assertEqual(p.command(3.4,(.97,0.,0.)),(0,0.))

    def test_tracking_does_not_switch_to_background_line_or_adjacent_bay(self):
        c,p=self.parking()
        for lines in ([[ (1.3,-.4),(1.3,.4)]],
                      [[(.85,.2),(.85,.7)]],
                      [[(.7,-.3),(1.,.3)]]):
            p.observe(lines,p.stamp+.1,c.pose)
            self.assertEqual(p.bottom_stamp,1.)

    def test_tracking_accepts_reversed_endpoints_and_keeps_centering(self):
        c,p=self.parking()
        p.sign_anchor=(.7,0.)  # stale P is already behind the measured bottom
        p.observe([[ (.82,.3),(.82,-.3)]],1.2,c.pose)
        self.assertEqual(p.bottom_stamp,1.2)
        self.assertGreater(p.command(1.2,c.pose)[0],0)

    def test_finish_requires_arrival_even_after_p_anchor_drifts(self):
        c,p=self.parking()
        for i in range(1,8):
            pose=(i*.1,0.,0.)
            x=.85-i*.07
            p.observe([[world(pose,q) for q in ((x,-.3),(x,.3))]],1.+i*.2,pose)
            command=p.command(1.+i*.2,pose)
            if i<7:self.assertGreater(command[0],0)
        self.assertEqual(command,(0,0.))
        self.assertEqual(p.status,'FINISHED')

    def test_yaw_change_cannot_calibrate_translation_scale(self):
        c,p=self.parking()
        p.remember_bottom([(.85,-.3),(.85,.3)],1.1,'tracked_bottom')
        p.pose=(.1,0.,.2)
        p.remember_bottom([world(p.pose,q) for q in ((.8,-.3),(.8,.3))],1.3,'tracked_bottom')
        self.assertEqual(p.motion_scale,1.)
        self.assertEqual(p.motion_ratios,[])

    def test_new_association_resets_previous_motion_calibration(self):
        c,p=self.parking()
        p.motion_scale=.4;p.motion_ratios=[.4]
        p.remember_bottom([(.8,-.3),(.8,.3)],1.2,'p_near_cross')
        self.assertEqual(p.motion_scale,1.)
        self.assertEqual(p.motion_ratios,[])


if __name__=='__main__':unittest.main()
