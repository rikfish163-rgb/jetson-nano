"""Production RIGHT timing, blue-stop dispatch and interruption tests; no ROS."""
import copy
import os
import unittest
from test_core import CONFIG, PACKAGE_DIR
from robot.master.controller import Controller
from robot.common.contracts import encode_command, validate_config
from robot.common.config import load_config


class TimedRightTests(unittest.TestCase):
    def test_production_reverse_and_right_turn_timing(self):
        cfg=load_config(os.path.join(PACKAGE_DIR,'config'))
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03)
        c=Controller(cfg)
        self.addCleanup(c.close)
        c.action='RIGHT'
        c.execute('mission','begin_blue_action',0.)
        for t in [i*.05 for i in range(22)]+[1.099]:
            self.assertEqual(self.command(c,t),(-30,0))
        self.assertEqual(self.command(c,1.1),(30,-22))
        self.assertEqual(c.right_lock['phase'],'TIMED_TURN')
        for t in [1.1+i*.1 for i in range(1,47)]+[5.799]:
            self.assertEqual(self.command(c,t),(30,-22))
        self.assertEqual(self.command(c,5.801),(-30,0))
        self.assertEqual(c.right_lock['phase'],'TIMED_EXIT_REVERSE')

    def make(self, start=True):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(right_timed_enabled=True, right_timed_reverse_s=1.,
                   right_timed_turn_s=2.0, right_timed_exit_reverse_s=1.0,
                   right_timed_exit_reverse_speed_raw=-30, right_timed_exit_reverse_steering_raw=0,
                   right_timed_reverse_speed_raw=-30,
                   right_timed_reverse_steering_raw=0, right_timed_turn_speed_raw=30,
                   right_timed_turn_steering_raw=-22, right_exit_on_blue=True,
                   right_reverse_entry_m=.25, right_turn_full_lock=False,
                   wait_green=False, lidar_enabled=False,
                   steering_command_scale_rad=.03)
        c=Controller(cfg)
        self.addCleanup(c.close)
        if start:
            c.action='RIGHT'
            c.execute('mission','begin_blue_action',0)
        return c

    def command(self,c,t):
        c.front_marker_stamp=t
        speed,steer=c.tick(t)
        raw=encode_command(speed,steer,c.cfg,0)
        return raw['speed_raw'],raw['steering_raw']

    def test_exact_sequence_via_main_tick(self):
        c=self.make()
        for i in range(80):
            t=i/20.
            expected=(-30,0) if t<1 or t>=3 else (30,-22)
            self.assertEqual(self.command(c,t),expected)
            if t>=3:self.assertEqual(c.right_lock['phase'],'TIMED_EXIT_REVERSE')
        self.assertEqual(self.command(c,4.0),(0,0))
        self.assertEqual(c.right_lock['phase'],'EXIT_SETTLE')
        for i in range(81,120):
            self.assertEqual(self.command(c,i/20.),(0,0))
            self.assertEqual(c.right_lock['phase'],'EXIT_SETTLE')
        c.lane_command=lambda now:(30,0.)
        self.assertEqual(self.command(c,6.0),(30,0))
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.right_lock)
        self.assertIsNone(c.action)
        self.assertIsNone(c.straight_search)

    def test_blue_stop_waits_then_dispatches_timed_right(self):
        c=self.make(False)
        c.cfg['blue_stop_test']=dict(camera_timeout_s=.6)
        c.action='RIGHT';c.state='BLUE_STOP'
        c.blue_approach=dict(image_timed=True,phase='STOP',stop_until=.5)
        self.assertEqual(self.command(c,.49),(0,0))
        self.assertIsNone(c.right_lock)
        self.assertEqual(self.command(c,.5),(0,0))
        self.assertEqual(c.state,'MANEUVER')
        self.assertEqual(self.command(c,.55),(-30,0))

    def test_new_blue_does_not_end_timed_turn(self):
        c=self.make()
        for i in range(31):
            t=i/20.
            c.marker=((.6,0),t)
            self.assertEqual(self.command(c,t),(-30,0) if t<1 else (30,-22))
        self.assertEqual(c.action,'RIGHT')
        self.assertIsNone(c.straight_search)

    def test_completed_right_returns_to_normal_lane_without_confirmation(self):
        c=self.make()
        c.right_lock.update(phase='WAIT_LANE',exit_started=.9)
        c.lane_command=lambda now:(30,.006)
        def forbidden(*args):raise AssertionError('extra exit confirmation')
        c.handoff_lane_confirmed=forbidden
        c.front_marker_stamp=1.
        self.assertEqual(c.tick(1.),(30,.006))
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.right_lock)
        self.assertIsNone(c.action)

    def test_no_lane_after_completion_stays_stopped_under_normal_lane_rules(self):
        c=self.make()
        c.right_lock.update(phase='WAIT_LANE',exit_started=.9)
        self.assertEqual(self.command(c,1.),(0,0))
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.right_lock)

    def test_gap_fault_never_restarts(self):
        c=self.make()
        self.command(c,0)
        self.assertEqual(self.command(c,2.01),(0,0))
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(self.command(c,2.1),(0,0))

    def test_external_protection_stop_latches(self):
        c=self.make()
        self.command(c,0)
        c.stop('scan_missing_or_stale')
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(self.command(c,.1),(0,0))

    def test_front_timeout_stops(self):
        c=self.make()
        self.command(c,0)
        c.front_marker_stamp=-10
        self.assertEqual(c.tick(.1),(0,0))
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(c.reason,'right_timed_front_stale')

    def test_guard_block_faults(self):
        c=self.make()
        c.checked_command=lambda *args:(0,0)
        self.assertEqual(self.command(c,0),(0,0))
        self.assertEqual(c.state,'FAULT')

    def test_geometric_fallback_uses_front_axle(self):
        c=self.make()
        c.blue_approach=dict(phase='STOP_LINE',point=(c.cfg['wheelbase'],0),
                            yaw=0,observed_stamp=1,started=1)
        c.front_marker_stamp=1
        c.execute('turn','blue_approach_tick',1)
        self.assertEqual(c.blue_approach['stop_reference'],'front_axle')
        self.assertEqual(c.state,'BLUE_STOP')


    def test_third_stage_protection_stop_latches(self):
        c=self.make()
        for i in range(61):self.command(c,i/20.)
        self.assertEqual(c.right_lock['phase'],'TIMED_EXIT_REVERSE')
        c.stop('scan_missing_or_stale')
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(self.command(c,3.1),(0,0))

    def test_third_stage_gap_never_resumes(self):
        c=self.make()
        for i in range(61):self.command(c,i/20.)
        self.assertEqual(self.command(c,5.01),(0,0))
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(self.command(c,5.1),(0,0))

    def test_third_stage_obstacle_stops(self):
        c=self.make()
        for i in range(61):self.command(c,i/20.)
        c.checked_command=lambda *args:(0,0)
        self.assertEqual(self.command(c,3.1),(0,0))
        self.assertEqual(c.state,'FAULT')

    def test_production_config_and_validation(self):
        cfg=load_config(os.path.join(PACKAGE_DIR,'config'))
        validate_config(cfg)
        self.assertTrue(cfg['right_timed_enabled'])
        self.assertEqual(cfg['right_timed_turn_s'],4.7)
        self.assertEqual(cfg['right_timed_exit_reverse_s'],2.5)
        self.assertEqual(cfg['right_timed_exit_stop_s'],2.0)
        self.assertEqual(cfg['right_timed_exit_reverse_speed_raw'],-30)
        self.assertEqual(cfg['right_timed_exit_reverse_steering_raw'],0)
        for key,value in [('right_timed_reverse_speed_raw',30),
                          ('right_timed_turn_speed_raw',-30),
                          ('right_timed_turn_speed_raw',31),
                          ('right_timed_turn_steering_raw',-23),
                          ('right_timed_reverse_s',0),
                          ('right_timed_turn_s',float('nan')),
                          ('right_timed_exit_reverse_s',0),
                          ('right_timed_exit_reverse_speed_raw',30),
                          ('right_timed_exit_reverse_steering_raw',23)]:
            bad=copy.deepcopy(cfg);bad[key]=value
            with self.assertRaises(ValueError):
                validate_config(bad)


if __name__=='__main__':
    unittest.main()
