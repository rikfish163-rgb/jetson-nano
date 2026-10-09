"""Accept the next route only after the final U-turn steering segment."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller


class UturnSignWindowTests(unittest.TestCase):
    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False)
        c=Controller(cfg);self.addCleanup(c.close)
        c.action='UTURN'
        c.execute('mission','begin_blue_action',0.)
        return c

    def tick(self,c,t):
        c.front_marker_stamp=t
        c.observe_lane([(.3,0.),(.5,0.),(.7,0.)],.95,t)
        return c.tick(t)

    def final_window(self,c):
        for i in range(300):
            t=i*.05;self.tick(c,t)
            if c.uturn and c.uturn.get('next_sign_after') is not None:return t
        self.fail('final U-turn sign window did not open')

    def test_recorded_early_straight_does_not_block_late_right(self):
        c=self.core()
        for i in range(241):
            t=i*.05;self.tick(c,t)
            if i in (40,47):c.observe_sign('STRAIGHT',.886,t,t)
            if i in (80,87):c.observe_sign('RIGHT',.941,t,t,right_visible=True)
            if i in (40,47,80,87):self.assertIsNone(c.next_direction)
        for t in (12.1,12.3):
            self.tick(c,t);c.observe_sign('RIGHT',.96,t,t,right_visible=True)
        self.assertEqual(c.next_direction,'RIGHT')
        for i in range(1,61):
            self.tick(c,12.3+i*.05)
            if c.action is None:break
        self.assertIsNone(c.action)
        self.assertEqual(c.pending,'RIGHT')

    def test_delayed_early_frame_cannot_vote_in_final_window(self):
        c=self.core();t=self.final_window(c)
        c.observe_sign('STRAIGHT',.99,t-.1,t+.01)
        self.assertEqual(c.sign_info['decision'],'uturn_wait_final_heading')
        self.assertIsNone(c.next_direction)
        c.observe_sign('STRAIGHT',.99,t+.02,t+.02)
        self.assertIsNone(c.next_direction)
        c.observe_sign('STRAIGHT',.99,t+.12,t+.12)
        self.assertEqual(c.next_direction,'STRAIGHT')

    def test_final_window_accepts_actual_route_without_right_preference(self):
        for label in ('LEFT','STRAIGHT','RIGHT'):
            c=self.core();t=self.final_window(c)
            for offset in (.02,.12):c.observe_sign(label,.95,t+offset,t+offset)
            self.assertEqual(c.next_direction,label)

    def test_red_remains_active_while_route_voting_is_closed(self):
        c=self.core();self.tick(c,0.)
        c.observe_sign('RED',.99,.1,.1)
        self.assertTrue(c.red)
        self.assertIsNone(c.next_direction)

    def test_window_follows_configured_last_turn_instead_of_segment_number(self):
        for tail,expected in (([], 'SETTLE'),
                ([dict(speed=-30,steering=0,seconds=.2)],'GEAR_PAUSE_4')):
            c=self.core()
            c.cfg['uturn_trial_sequence']=[dict(speed=30,steering=22,seconds=.1),
                dict(speed=-30,steering=-22,seconds=.1),
                dict(speed=30,steering=22,seconds=.1)]+tail
            self.final_window(c)
            self.assertEqual(c.uturn['phase'],expected)


if __name__=='__main__':unittest.main()
