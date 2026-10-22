"""Requested fixed bypass and moving lane handoff, without sweep vetoes."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class RequestedBypassCrawlTests(unittest.TestCase):
    PATH = [(.55,.03),(.70,.03),(.85,.03),(1.,.03)]

    def core(self, phase='RIGHT', elapsed=4.):
        cfg = load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False, lidar_enabled=True, timed_bypass_enabled=True,
                   steering_command_scale_rad=.03, lane_curvature_preview=True)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.state, c.action = 'TIMED_BYPASS', 'BYPASS'
        c.timed_bypass = dict(phase=phase,elapsed_s=elapsed,last=1.,
                              trigger_world=(-.5,0),trigger_point=(.8,0))
        return c

    def tick(self, c, now, path=None):
        c.observe_lane(self.PATH if path is None else path, .9, now)
        c.scan = _SyntheticScan(now, obstacles=[(.20,0),(.45,-.18)])
        return c.execute('obstacle','timed_bypass_tick',now).value

    def test_left_switches_at_one_point_five_despite_predicted_collision(self):
        c = self.core('LEFT',1.49)
        self.assertEqual(self.tick(c,1.02),(26,-.03))
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')

    def test_single_candidate_does_not_stop_right_or_lock_reacquire(self):
        c = self.core()
        self.assertEqual(self.tick(c,1.05),(26,-.03))
        self.assertEqual(self.tick(c,1.15,[]),(26,-.03))
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')

    def test_three_consistent_frames_handoff_without_zero_speed(self):
        c = self.core()
        for now in (1.05,1.15,1.25):
            self.assertGreater(self.tick(c,now)[0],0)
        self.assertIsNone(c.timed_bypass)

    def test_six_second_limit_crawls_on_trackable_unconfirmed_lane(self):
        c = self.core(elapsed=5.99)
        speed, steer = self.tick(c,1.02)
        self.assertEqual(speed,12)
        self.assertEqual(c.timed_bypass['phase'],'REACQUIRE')
        self.assertEqual(c.state,'TIMED_BYPASS')

    def test_six_second_limit_still_stops_on_actual_lane_loss(self):
        c = self.core(elapsed=5.99)
        self.assertEqual(self.tick(c,1.02,[])[0],0)

    def test_sweep_does_not_veto_forward_or_reverse_commands(self):
        c = self.core()
        c.timed_bypass = None
        c.state,c.action = 'MANEUVER','RIGHT'
        c.scan = _SyntheticScan(1.,obstacles=[(.20,0)],classification='UNKNOWN')
        for command in ((20,.03),(-30,0.)):
            self.assertEqual(c.execute('obstacle','checked_command',command,1.,False).value,command)

    def test_left_reference_does_not_stop_for_predicted_boundary_crossing(self):
        c = self.core()
        c.timed_bypass = None
        c.state,c.action = 'MANEUVER','RIGHT'
        c.right_lock = dict(phase='ALIGN_LANE')
        line = dict(points=[(.2,-.15),(.4,-.15),(.6,-.15)],heading=0.,
                    lateral=-.15,offset_m=.30,boundary_distance_m=.15,side='LEFT')
        self.assertGreater(c.execute('lane','left_reference_command',line,1.).value[0],0)

    def test_explicit_stop_and_lane_loss_are_preserved(self):
        c = self.core()
        self.assertEqual(c.execute('motion','stop','emergency_stop').value[0],0)
        c.timed_bypass=None
        c.state,c.action='LANE',None
        c.observe_lane([],0.,1.)
        self.assertEqual(c.execute('lane','lane_command',1.).value[0],0)

    def test_no_lane_during_arc_does_not_shorten_four_second_minimum(self):
        c = self.core(elapsed=3.5)
        for now in (1.1,1.2,1.3,1.4):
            self.assertEqual(self.tick(c,now,[]),(26,-.03))
            self.assertEqual(c.timed_bypass['phase'],'RIGHT')

    def test_lidar_dropout_does_not_interrupt_active_fixed_bypass(self):
        c = self.core('LEFT',1.49)
        c.scan = None
        self.assertEqual(c.tick(1.02),(26,-.03))
        self.assertEqual(c.state,'TIMED_BYPASS')

    def test_left_wait_crawls_without_discarding_pending_handoff(self):
        c = self.core()
        c.timed_bypass = None
        c.state,c.action = 'MANEUVER','LEFT'
        c.left_lock = dict(phase='WAIT_LANE')
        c.observe_lane(self.PATH,.9,1.)
        c._runtime.overrides['handoff_lane_confirmed'] = lambda now: False
        self.assertEqual(c.execute('turn','left_lock_tick',1.).value[0],12)
        self.assertEqual(c.state,'MANEUVER')
        self.assertEqual(c.left_lock['phase'],'WAIT_LANE')

    def test_camera_lane_available_but_lidar_missing_uses_crawl(self):
        c = self.core()
        c.state,c.action,c.timed_bypass = 'LANE',None,None
        c.scan = None
        self.assertEqual(c.execute('obstacle','checked_command',(30,.01),1.,False).value,(12,.01))


if __name__ == '__main__':
    unittest.main()
