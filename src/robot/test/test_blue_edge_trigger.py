"""Blue event lifetime, independent of global map coordinates."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from blue_test_helpers import enter_blue_action, clear_blue, observe_direction


class BlueEdgeTests(unittest.TestCase):
    def setUp(self):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        cfg.update(wait_green=False,lidar_enabled=False,blue_default_straight=False,sign_ttl=0)
        cfg['blue_align_duration_s']=.1
        self.c=Controller(cfg);self.addCleanup(self.c.close)
        observe_direction(self.c,'STRAIGHT',.99,1.)

    def frame(self,t,x=None,kind='junction'):
        self.c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[] if x is None else [dict(kind=kind,x=x,y=0)],
            blue_lines=[] if x is None or kind!='junction' else [dict(x=x,y=0,yaw=0,length=.6)]),t)

    def confirmed(self,t,x):
        for stamp in (t-.2,t-.1,t):
            self.frame(stamp,x);self.c.dispatch(stamp)

    def test_far_confirmed_line_approaches_and_ignores_old_map_position(self):
        self.c.pending='LEFT';self.c.consumed_marker=(1.2,0)
        self.confirmed(1.4,1.2)
        self.assertEqual(self.c.action,'LEFT')
        self.assertEqual(self.c.state,'BLUE_APPROACH')

    def test_aligned_target_survives_near_blind_region(self):
        self.c.pending='LEFT';self.confirmed(1.4,.58)
        for t in (1.5,1.6):self.frame(t,.58);self.c.tick(t)
        self.c.set_pose((.36,0,0),2.)
        self.frame(2.);self.c.tick(2.)
        self.assertEqual(self.c.state,'BLUE_STOP')

    def test_lane_gap_holds_previous_turn_until_blue_takes_heading_control(self):
        self.c.observe_lane([(.5,.1),(.7,.1)],.9,1.)
        before=self.c.tick(1.)[1]
        for t in (1.1,1.2,1.3):
            self.c.observe_lane([],0.,t);self.frame(t,.9)
            self.assertEqual(self.c.tick(t)[1],before if t<1.3 else 0.)
        self.assertEqual(self.c.state,'BLUE_APPROACH')

    def test_single_frame_does_not_arm_projection(self):
        self.frame(1.1,.58)
        self.c.set_pose((.25,0,0),2.);self.frame(2.);self.c.dispatch(2.)
        self.assertIsNone(self.c.action)

    def test_confirmed_aligned_target_advances_after_leaving_image(self):
        self.confirmed(1.4,1.2)
        self.frame(2.)
        self.assertGreater(self.c.tick(2.)[0],0)
        self.assertEqual(self.c.reason,'straight_approach_stop_line')
        self.assertEqual(self.c.blue_approach['phase'],'STOP_LINE')
        self.assertEqual(self.c.action,'STRAIGHT')

    def test_continuously_visible_line_has_only_one_chance(self):
        enter_blue_action(self.c,'STRAIGHT',1.5);self.c.resume_lane()
        observe_direction(self.c,'RIGHT',.99,1.6)
        self.confirmed(2.,.8);self.assertIsNone(self.c.action)
        self.frame(2.1)
        self.confirmed(2.5,.8);self.assertIsNone(self.c.action)
        clear_blue(self.c,3.)
        self.confirmed(3.4,.8);self.assertEqual(self.c.action,'RIGHT')

    def test_short_ticks_and_sign_alone_do_not_trigger(self):
        for t in (1.1,1.2,1.3):
            self.frame(t,.3,'tick');self.c.dispatch(t)
        self.assertIsNone(self.c.action)
        self.assertEqual(self.c.pending,'STRAIGHT')

    def test_line_during_action_is_not_queued(self):
        enter_blue_action(self.c,'STRAIGHT',1.5)
        self.frame(1.6);self.frame(1.7,.9)
        self.c.resume_lane();observe_direction(self.c,'LEFT',.99,1.8)
        self.confirmed(2.2,.9)
        self.assertIsNone(self.c.action)

    def test_rear_updates_do_not_rearm_front_line(self):
        enter_blue_action(self.c,'STRAIGHT',1.5);self.c.resume_lane()
        for t in (1.6,1.8,2.):
            self.c.observe_ground(dict(source='rear',part='markers',slots=[],markers=[]),t)
        observe_direction(self.c,'STRAIGHT',.99,2.1)
        self.confirmed(2.5,.9);self.assertIsNone(self.c.action)

    def test_line_without_pending_does_not_wait_for_later_sign(self):
        self.c.pending=None;self.confirmed(1.4,.8)
        observe_direction(self.c,'STRAIGHT',.99,1.5)
        self.confirmed(1.9,.8);self.assertIsNone(self.c.action)


if __name__=='__main__':unittest.main()
