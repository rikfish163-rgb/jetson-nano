"""Acquired road centers release exit/search modes back to normal lane control."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller


class NormalLaneHandoffTests(unittest.TestCase):
    def core(self, action):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03)
        c=Controller(cfg);self.addCleanup(c.close)
        c.action=action
        c.execute('mission','begin_blue_action',0.)
        return c

    def observe(self,c,t):
        c.observe_lane([(.3,0.),(.5,0.),(.7,0.),(.9,0.),(1.1,0.)],.95,t)
        c.front_marker_stamp=t

    def test_right_releases_on_fresh_center_without_left_boundary(self):
        c=self.core('RIGHT');c.right_lock.update(phase='WAIT_LANE',exit_started=10.)
        for t in (10.1,10.2,10.3):
            self.observe(c,t);command=c.tick(t)
        self.assertIsNone(c.action)
        self.assertIsNone(c.right_lock)
        self.assertEqual(c.state,'LANE')
        self.assertFalse(c.follow_left_boundary)
        self.assertFalse(c.right_handoff_slow)
        self.assertGreater(command[0],20)

    def test_repeated_or_pre_completion_frames_cannot_release_right(self):
        c=self.core('RIGHT');c.right_lock.update(phase='WAIT_LANE',exit_started=10.)
        self.observe(c,9.9);c.tick(10.)
        self.observe(c,10.1)
        for t in (10.1,10.2,10.3):c.tick(t)
        self.assertEqual(c.action,'RIGHT')

    def test_right_preserves_cached_second_sign_at_normal_handoff(self):
        c=self.core('RIGHT');c.right_lock.update(phase='WAIT_LANE',exit_started=10.)
        c.next_direction='RIGHT';c.next_direction_at=10.05
        c.right_sign_rearmed_at=9.
        for t in (10.1,10.2,10.3):self.observe(c,t);c.tick(t)
        self.assertEqual(c.pending,'RIGHT')
        self.assertIsNone(c.action)

    def test_uturn_completion_uses_normal_centers_without_waiting_for_blue(self):
        c=self.core('UTURN')
        c.execute('mission','resume_lane')
        self.assertFalse(c.uturn_exit_straight)
        self.observe(c,1.)
        self.assertGreater(c.tick(1.)[0],0)
        self.assertEqual(c.lane_source,'center')

    def test_straight_releases_before_distance_limit_when_lane_is_confirmed(self):
        c=self.core('STRAIGHT');c.action_source='sign'
        for t in (.1,.2,.3):self.observe(c,t);c.tick(t)
        self.assertIsNone(c.straight_search)
        self.assertIsNone(c.action)
        self.assertEqual(c.lane_source,'center')


if __name__=='__main__':unittest.main()
