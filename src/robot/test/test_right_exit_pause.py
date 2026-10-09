"""Final reverse extends to 2.5 s, then stop for 2 s before lane handoff."""
import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import validate_config
from robot.master.controller import Controller


class RightExitPauseTests(unittest.TestCase):
    def core(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False, lidar_enabled=False,
                   straight_speed_raw=30, lane_curve_speed_raw=30,
                   right_timed_reverse_s=.1, right_timed_turn_s=.2)
        cfg['speed_raw']['lane'] = 30
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.action = 'RIGHT'
        c.execute('mission', 'begin_blue_action', 0.)
        return c

    def tick(self, c, now, lane=True):
        c.front_marker_stamp = now
        if lane:
            c.observe_lane([(.25, 0.), (.45, 0.), (.65, 0.)], .99, now)
        return c.tick(now)

    def reach_pause(self, c):
        reverse_start = None
        for i in range(81):
            now = i*.05
            command = self.tick(c, now)
            phase = c.right_lock['phase'] if c.right_lock else None
            if phase == 'TIMED_EXIT_REVERSE':
                reverse_start = c.right_lock['timed_started']
                self.assertEqual(command[0], -30)
            if phase == 'EXIT_SETTLE':
                self.assertIsNotNone(reverse_start)
                self.assertGreaterEqual(now-reverse_start, 2.5)
                self.assertLess(now-reverse_start, 2.56)
                self.assertEqual(command, (0, 0.))
                return now
        self.fail('final reverse never reached its stationary pause')

    def test_production_duration_and_pause_config(self):
        c = self.core()
        self.assertEqual(c.cfg['right_timed_exit_reverse_s'], 2.5)
        self.assertEqual(c.cfg['right_timed_exit_stop_s'], 2.)
        for value in (-1., 0., float('nan'), 11.):
            cfg = dict(c.cfg, right_timed_exit_stop_s=value)
            with self.assertRaises(ValueError):
                validate_config(cfg)

    def test_fresh_aligned_lane_cannot_shorten_two_second_stop(self):
        c = self.core()
        start = self.reach_pause(c)
        for offset in [i*.05 for i in range(1, 40)] + [1.999]:
            self.assertEqual(self.tick(c, start+offset), (0, 0.))
            self.assertEqual(c.right_lock['phase'], 'EXIT_SETTLE')
            self.assertEqual(c.state, 'MANEUVER')
        self.assertEqual(self.tick(c, start+2.), (0, 0.))
        self.assertEqual(c.right_lock['phase'], 'WAIT_LANE')
        self.assertGreater(self.tick(c, start+2.05)[0], 0)
        self.assertEqual(c.right_lock['phase'], 'ALIGN_LANE')

    def test_pause_images_do_not_count_as_post_pause_lane_confirmation(self):
        c = self.core()
        start = self.reach_pause(c)
        for i in range(1, 41):
            self.tick(c, start+i*.05)
        self.assertEqual(self.tick(c, start+2.05, lane=False), (30, 0.))
        self.assertEqual(c.reason, 'right_timed_exit_search_straight')
        self.assertEqual(c.exit_count,0)

    def exit_search(self,c):
        c.right_lock.update(phase='WAIT_LANE',exit_started=1.)
        c.consumed_marker=(-1.,0.)

    def blue(self,c,t,x=.65,yaw=0.):
        c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind='junction',x=x,y=0.,length=.8)],
            blue_lines=[dict(x=x,y=0.,yaw=yaw,length=.8)]),t)
        return c.tick(t)

    def test_missing_lane_keeps_configured_speed_past_old_action_timeout_then_recovers(self):
        c=self.core();self.exit_search(c)
        for t in (1.1,41.,60.):
            self.assertEqual(self.tick(c,t,lane=False),(30,0.))
            self.assertEqual(c.state,'MANEUVER')
        for t in (60.1,60.3,60.7):self.tick(c,t)
        self.assertEqual(c.state,'MANEUVER')
        self.assertEqual(c.reason,'right_timed_exit_align')

    def test_unusable_lane_points_keep_straight_instead_of_last_steering(self):
        for points,confidence in (([],.9),([(.3,.1)],.9),
                ([(.3,0),(.5,0),(.7,0)],.1),
                ([(.3,.9),(.5,1.5),(.7,2.1)],.9)):
            c=self.core();self.exit_search(c)
            c.issued_steer=-.03
            c.observe_lane(points,confidence,2.)
            self.assertEqual(self.tick(c,2.,lane=False),(30,0.))

    def test_confirmed_new_blue_dispatches_queued_direction_without_lane(self):
        for action in ('STRAIGHT','RIGHT'):
            c=self.core();self.exit_search(c)
            c.next_direction=action;c.next_direction_at=2.
            for t in (3.,3.1):
                self.assertEqual(self.blue(c,t),(30,0.))
                self.assertEqual(c.action,'RIGHT')
            self.blue(c,3.2)
            self.assertEqual(c.action,action)
            self.assertEqual(c.state,'BLUE_APPROACH')
            self.assertEqual(c.last_blue_trigger['action'],action)
            self.assertIsNone(c.right_lock)

    def test_next_straight_runs_through_blue_trigger_and_stop(self):
        c=self.core();self.exit_search(c)
        c.next_direction='STRAIGHT';c.next_direction_at=2.
        for t in (3.,3.1,3.2):self.blue(c,t)
        camera=c.cfg['front_camera']
        row=c.cfg['blue_stop_test']['trigger_row_ratio']+.01
        x=(camera['origin_v']-row*(camera['bev_height']-1.))/camera['pixels_per_m']
        states=[]
        for i in range(1,41):
            self.blue(c,3.2+i*.1,x=x)
            states.append(c.state)
            if c.state=='MANEUVER':break
        self.assertIn('BLUE_STOP',states)
        self.assertEqual(c.state,'MANEUVER')
        self.assertEqual(c.action,'STRAIGHT')

    def test_duplicate_or_stale_blue_cannot_supply_confirmation_votes(self):
        c=self.core();self.exit_search(c)
        c.next_direction='STRAIGHT';c.next_direction_at=2.
        self.blue(c,3.)
        for t in (3.1,3.2,5.):
            c.tick(t)
            self.assertEqual(c.action,'RIGHT')
        self.assertIsNone(c.last_blue_trigger)

    def test_old_longitudinal_or_unassigned_blue_cannot_handoff(self):
        import math
        for old,yaw,queued in ((True,0.,True),(False,math.pi/2,True),(False,0.,False)):
            c=self.core();self.exit_search(c)
            if old:c.consumed_marker=(.65,0.)
            if queued:c.next_direction='STRAIGHT';c.next_direction_at=2.
            for t in (3.,3.1,3.2):self.blue(c,t,yaw=yaw)
            self.assertEqual(c.action,'RIGHT')
            self.assertIsNone(c.last_blue_trigger)

    def test_search_still_obeys_explicit_stop_and_obstacle_guard(self):
        c=self.core();self.exit_search(c)
        c.checked_command=lambda *args:(0,0.)
        self.assertEqual(self.tick(c,2.,lane=False),(0,0.))
        c=self.core();self.exit_search(c);c.estop=True
        self.assertEqual(self.tick(c,2.,lane=False),(0,0.))

    def test_second_right_can_be_cached_during_pause(self):
        c = self.core()
        start = self.reach_pause(c)
        for offset in (.1, .2):
            now = start+offset
            c.observe_sign('RIGHT', .95, now, now)
            self.assertEqual(self.tick(c, now), (0, 0.))
        self.assertEqual(c.next_direction, 'RIGHT')
        for i in range(5, 57):
            self.tick(c, start+i*.05)
        self.assertEqual(c.state, 'MANEUVER')
        self.assertEqual(c.next_direction, 'RIGHT')

    def test_aligned_lane_keeps_correcting_until_next_blue(self):
        c=self.core();self.exit_search(c)
        c.next_direction='STRAIGHT';c.next_direction_at=2.
        for t in (3.,3.2,3.6,4.,40.,60.):
            self.assertEqual(self.tick(c,t)[0],30)
            self.assertEqual(c.action,'RIGHT')
            self.assertEqual(c.reason,'right_timed_exit_align')
        c.observe_lane([(.25,.075),(.45,.135),(.65,.195)],.99,60.1)
        speed,steer=self.tick(c,60.1,lane=False)
        self.assertEqual(speed,30)
        self.assertGreater(steer,0.)
        for t in (60.2,60.3,60.4):self.blue(c,t)
        self.assertEqual(c.state,'BLUE_APPROACH')
        self.assertEqual(c.action,'STRAIGHT')

    def test_parking_sign_can_handoff_without_a_junction_blue(self):
        c=self.core();self.exit_search(c)
        c.cfg.update(parking_mode='timed_sequence',parking_slot='P3')
        c.next_direction='PARKING';c.next_direction_at=2.
        self.tick(c,3.)
        self.assertEqual(c.state,'TIMED_PARKING')
        self.assertEqual(c.action,'PARKING')

    def test_estop_during_pause_cannot_resume_motion(self):
        c = self.core()
        start = self.reach_pause(c)
        c.estop = True
        for i in range(1, 50):
            self.assertEqual(self.tick(c, start+i*.05), (0, 0.))
        self.assertEqual(c.state, 'FAULT')


if __name__ == '__main__':
    unittest.main()
