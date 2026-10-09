"""P-board association: an unrelated, nearer rectangle must never finish parking."""
import unittest
from blue_test_helpers import enter_blue_action
import test_direction_single_frame as fixtures
from robot.parking.forward import ForwardParking
from robot.common.contracts import encode_command


class ParkingAnchorTests(unittest.TestCase):
    def parking(self, anchor=(1.4, .1)):
        c=fixtures.DirectionSingleFrameTests.__dict__['core'](self)
        c.cfg.update(parking_sign_association=True,parking_entry_speed_raw=12,
                     parking_mode='forward_center',steering_command_scale_rad=.1)
        p=ForwardParking(c.cfg,c.pose,1.)
        p.sign_anchor=anchor
        return c,p

    def test_p_bay_wins_over_nearer_closed_rectangle(self):
        c,p=self.parking()
        wrong=[[(.2,-.19),(.6,-.19)],[(.2,.19),(.6,.19)],[(.6,-.19),(.6,.19)]]
        target=[[(.7,-.09),(1.4,-.09)],[(.7,.29),(1.4,.29)]]
        p.observe(wrong+target,1.,c.pose)
        self.assertIsNone(p.bottom)
        self.assertAlmostEqual(p.view['center'][0],1.05)
        speed,steer=p.command(1.,c.pose)
        self.assertEqual(speed,12)
        self.assertLessEqual(abs(encode_command(speed,steer,c.cfg,0)['steering_raw']),6)
        p.observe(target+[[(1.4,-.09),(1.4,.29)]],1.1,c.pose)
        self.assertEqual(p.command(1.1,c.pose)[0],12)
        p.observe([],1.2,(1.031,0.,0.))
        self.assertEqual(p.command(1.2,(1.031,0.,0.)),(0,0.))

    def test_wrong_side_pair_and_missing_sign_cannot_stop(self):
        for anchor in ((1.,-.7),None):
            c,p=self.parking(anchor)
            p.observe([[ (.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)],
                       [(1.,-.19),(1.,.19)]],1.,c.pose)
            self.assertIsNone(p.bottom)
            self.assertEqual(p.command(1.,c.pose),(12,0.))

    def test_associated_single_then_pair_loss_goes_straight(self):
        c,p=self.parking((1.,0.))
        lines=[[(.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)]]
        p.observe(lines[:1],1.,c.pose)
        self.assertEqual(p.command(1.,c.pose)[0],12)
        self.assertEqual(p.reason,'parking_follow_sign_side')
        p.observe(lines,1.1,c.pose);p.command(1.1,c.pose)
        p.observe(lines[:1],1.2,c.pose)
        self.assertEqual(p.command(1.2,c.pose),(12,0.))

    def test_mission_does_not_follow_unrelated_raw_boundary(self):
        c,p=self.parking(None)
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        enter_blue_action(c,'PARKING',1.2)
        c.observe_lane([],0.,1.2,{'LEFT':[(.3,.18),(.5,.18),(.7,.18)]})
        c.tick(1.2)
        c.observe_ground(dict(source='front',part='parking_lines',slots=[],markers=[],lines=[]),1.3)
        self.assertEqual(c.tick(1.3),(12,0.))

    def test_off_center_p_bay_is_not_replaced_by_centered_lane(self):
        c,p=self.parking((1.,-.42))
        p.observe([[ (.4,-.61),(1.,-.61)],[(.4,-.23),(1.,-.23)]],1.,c.pose)
        self.assertIsNotNone(p.view)
        self.assertAlmostEqual(p.view['center'][1],-.42)

    def test_projection_uses_height_and_rejects_clipped_bounds(self):
        from robot.parking.sign_anchor import ParkingSignProjector
        c,p=self.parking()
        projector=ParkingSignProjector(c.cfg)
        position=projector.position([551,71,71,84])
        self.assertGreater(position[0],.5)
        self.assertLess(position[0],1.2)
        self.assertLess(position[1],-.2)
        for bounds in ([620,54,20,111],[10,10,0,30],[10,10,float('nan'),30]):
            self.assertIsNone(projector.position(bounds))

    def test_bottom_near_p_stops_without_simultaneous_sides(self):
        c,p=self.parking((.85,-.16))
        # Recorded geometry: broad bottom paint, one side off-camera.
        line=[(.69,-.34),(.765,.36)]
        p.observe([line],1.,c.pose)
        self.assertEqual(p.command(1.,c.pose)[0],12)
        self.assertIsNone(p.status)
        # Nearest bumper corner reaches the 4 cm clearance at x=.34357.
        pose=(.344,0.,0.)
        p.observe([],1.1,pose)
        self.assertEqual(p.command(1.1,pose),(0,0.))
        self.assertEqual(p.status,'FINISHED')
        self.assertEqual(p.debug['bottom_source'],'p_near_cross')
        self.assertEqual(p.command(1.2,pose),(0,0.))

    def test_entrance_adjacent_bay_and_behind_p_cross_do_not_stop(self):
        for line in ([[(.3,-.4),(.3,.4)]],
                     [[(.72,.1),(.72,.5)]],
                     [[(.9625,-.0575),(.86,-.5825)]],
                     [[(1.05,-.4),(1.05,.4)]]):
            c,p=self.parking((.85,-.16))
            p.observe(line,1.,c.pose)
            self.assertIsNone(p.bottom)
            self.assertEqual(p.command(1.,c.pose),(12,0.))

    def test_mission_finishes_from_bottom_without_side_pair(self):
        c,p=self.parking((.85,-.16))
        c.parking_sign=dict(point=p.sign_anchor,stamp=1.2)
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        enter_blue_action(c,'PARKING',1.2)
        c.tick(1.2)
        c.observe_ground(dict(source='front',part='parking_lines',slots=[],markers=[],
            lines=[[ (.69,-.34),(.765,.36)]]),1.3)
        self.assertGreater(c.tick(1.3)[0],0)
        self.assertEqual(c.state,'PARKING')
        c.set_pose((.344,0.,0.),1.4)
        c.observe_ground(dict(source='front',part='parking_lines',lines=[]),1.4)
        self.assertEqual(c.tick(1.4),(0,0.))
        self.assertEqual(c.state,'FINISHED')
        self.assertEqual(c.reason,'parking_at_bottom_clearance')
        self.assertEqual(c.tick(2.),(0,0.))

    def test_recorded_line_behind_p_cannot_supply_missing_right_side(self):
        c,p=self.parking((.89575,-.40071))
        lines=[[(.62417,-.08064),(1.28198,-.18369)],
               [(1.09058,-.55426),(1.34573,-.60601)]]
        p.observe(lines,1.,c.pose)
        self.assertIsNone(p.view)
        speed,steer=p.command(1.,c.pose)
        self.assertEqual(speed,12)
        self.assertEqual(p.reason,'parking_follow_sign_side')
        self.assertLess(steer,0)
        self.assertEqual(p.phase,'APPROACH')
        self.assertEqual(p.debug['sides_behind_sign_rejected'],1)

    def test_front_pair_wins_over_background_pair_even_if_background_is_parallel(self):
        c,p=self.parking((.9,-.4))
        front=[[(.3,-.2),(.8,-.2)],[(.3,-.6),(.8,-.6)]]
        back=[[(1.0,-.2),(1.3,-.2)],[(1.0,-.6),(1.3,-.6)]]
        p.observe(back+front,1.,c.pose)
        self.assertIsNotNone(p.view)
        for side in p.view['sides']:
            self.assertLess(max(q[0] for q in side),.9)

    def test_only_background_lines_means_straight_not_fake_pair(self):
        c,p=self.parking((.9,-.4))
        p.observe([[(1.,-.2),(1.3,-.2)],[(1.,-.6),(1.3,-.6)]],1.,c.pose)
        self.assertIsNone(p.view)
        self.assertIsNone(p.single)
        self.assertEqual(p.command(1.,c.pose),(12,0.))

    def test_shared_bottom_does_not_finish_while_car_is_in_adjacent_bay(self):
        c,p=self.parking((.95,-.4))
        sides=[[(.4,-.2),(.85,-.2)],[(.4,-.6),(.85,-.6)]]
        for lines in ([[ (.85,-.8),(.85,.4)]],sides+[[ (.85,-.6),(.85,-.2)]]):
            p.observe(lines,p.stamp+2.,c.pose)
            self.assertIsNone(p.bottom)
            self.assertEqual(p.command(p.stamp,c.pose)[0],12)

if __name__=='__main__':unittest.main()
