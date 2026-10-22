"""Front S parking counts signed travel after its front axle crosses blue."""
import copy
import math
import unittest
from test_core import CONFIG
from blue_test_helpers import enter_blue_action, observe_direction
from robot.master.controller import Controller
from robot.common.geometry import world


class BlueNetDistanceTests(unittest.TestCase):
    def core(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(wait_green=False,lidar_enabled=False,parking_lidar_enabled=False,
                   parking_mode='forward_center',parking_entry_style='S',
                   parking_s_blue_distance_m=1.40,parking_p_reverse_s=0.)
        c=Controller(cfg);self.addCleanup(c.close)
        observe_direction(c,'STRAIGHT',.99,.6)
        enter_blue_action(c,'STRAIGHT',1.)
        return c

    def crossing(self,c):
        return c.last_blue_trigger['point_world'][0]-c.cfg['wheelbase']

    def arm(self,c,now):
        c.execute('parking','start_explicit_parking',now,trigger='sign')

    def test_starts_at_front_axle_crossing_and_interpolates_crossing_step(self):
        c=self.core();x=self.crossing(c)
        c.set_pose((x-.01,0.,0.),1.1)
        self.assertFalse(c.parking_straight_travel['crossed'])
        self.assertEqual(c.parking_straight_travel['travelled_m'],0.)
        c.set_pose((x+.03,0.,0.),1.2)
        self.assertTrue(c.parking_straight_travel['crossed'])
        self.assertAlmostEqual(c.parking_straight_travel['travelled_m'],.03)

    def test_reverse_subtracts_and_p_handoff_preserves_origin(self):
        c=self.core();x=self.crossing(c)
        c.set_pose((x+.5,0.,0.),1.1)
        self.arm(c,1.1)
        c.set_pose((x+.2,0.,0.),1.2)
        self.assertAlmostEqual(c.parking_straight_travel['travelled_m'],.2)
        c.set_pose((x+1.399,0.,0.),1.3)
        self.assertGreater(c.tick(1.3)[0],0)
        c.set_pose((x+1.40,0.,0.),1.4)
        self.assertEqual(c.tick(1.4),(0,0.))
        self.assertEqual(c.state,'FINISHED')
        self.assertEqual(c.reason,'parking_blue_net_distance_reached')
        self.assertEqual(c.tick(1.5),(0,0.))
        self.assertEqual(c.reason,'parking_blue_net_distance_reached')

    def test_reverse_back_across_blue_does_not_reset_counter(self):
        c=self.core();x=self.crossing(c)
        for t,offset in ((1.1,.2),(1.2,-.1),(1.3,.1)):
            c.set_pose((x+offset,0.,0.),t)
            self.assertAlmostEqual(c.parking_straight_travel['travelled_m'],offset)

    def test_curved_travel_uses_signed_path_length_and_ignores_old_pose(self):
        c=self.core();x=self.crossing(c)
        c.set_pose((x,0.,0.),1.1)
        c.set_pose((x+.3,.4,math.pi/2),1.2)
        self.assertAlmostEqual(c.parking_straight_travel['travelled_m'],.5)
        c.set_pose((x+.3,.3,math.pi/2),1.3)
        c.set_pose((x+10.,0.,0.),1.2)
        self.assertAlmostEqual(c.parking_straight_travel['travelled_m'],.4)

    def test_bottom_visible_or_lost_cannot_finish_distance_entry_early(self):
        c=self.core();x=self.crossing(c)
        c.set_pose((x+.1,0.,0.),1.1);self.arm(c,1.1)
        p=c.parking_entry
        bottom=[[world(c.pose,(.34,-.19)),world(c.pose,(.34,.19))]]
        for t in (1.2,1.3):p.observe(bottom,t,c.pose)
        self.assertGreater(c.tick(1.3)[0],0)
        p.observe([],1.4,c.pose)
        c.set_pose((x+.3,0.,0.),1.5)
        self.assertGreater(c.tick(1.5)[0],0)
        self.assertEqual(c.state,'PARKING')

    def test_missing_blue_origin_stops_instead_of_using_p_as_zero(self):
        c=self.core();c.parking_straight_travel=None
        self.arm(c,1.1)
        self.assertEqual(c.tick(1.2),(0,0.))
        self.assertEqual(c.reason,'parking_blue_origin_missing')

    def test_actual_three_second_p_retreat_subtracts_from_same_counter(self):
        c=self.core();x=self.crossing(c)
        c.set_pose((x+.6,0.,0.),1.1)
        origin_stamp=c.parking_straight_travel['stamp']
        c.cfg['parking_p_reverse_s']=3.
        self.arm(c,1.1)
        reverse_distance=0.
        saw_reverse=False
        for i in range(80):
            now=1.1+i*.05
            speed,steer=c.tick(now)
            if speed<0:
                saw_reverse=True
                reverse_distance+=.005
                c.set_pose((c.pose[0]-.005,0.,0.),now+.05)
            if not c.parking_entry.p_reverse_active:break
        self.assertTrue(saw_reverse)
        self.assertFalse(c.parking_entry.p_reverse_active)
        self.assertAlmostEqual(c.parking_entry.p_reverse_elapsed,3.)
        self.assertAlmostEqual(c.parking_straight_travel['travelled_m'],.6-reverse_distance)
        self.assertEqual(c.parking_straight_travel['stamp'],origin_stamp)

    def test_production_distance_and_rear_bottom_stop_are_independent(self):
        import os
        from robot.common.config import load_config
        from robot.common.contracts import validate_config
        from robot.parking.reverse_entry import ReverseStraightEntry
        from robot.parking.entry import ExplicitStraightEntry
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        self.assertEqual(cfg['parking_s_blue_distance_m'],1.40)
        validate_config(cfg)
        self.assertFalse(ExplicitStraightEntry(cfg,(0.,0.,0.),1.).bottom_stop_enabled)
        self.assertTrue(ReverseStraightEntry(cfg,(0.,0.,0.),1.).bottom_stop_enabled)
        for bad in (-1.,float('nan'),float('inf')):
            cfg['parking_s_blue_distance_m']=bad
            with self.assertRaises(ValueError):validate_config(cfg)


if __name__=='__main__':unittest.main()
