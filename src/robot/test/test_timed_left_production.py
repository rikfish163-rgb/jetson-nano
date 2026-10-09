"""Production timing, exact RAW output and stop/blue-line contracts; no ROS."""
import copy
import unittest
from test_core import CONFIG
from robot.master.controller import Controller
from robot.common.contracts import encode_command


class TimedLeftTests(unittest.TestCase):
    def make(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(left_timed_enabled=True,left_timed_entry_s=1.5,left_timed_turn_s=3.5,
                   wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03)
        c=Controller(cfg)
        self.addCleanup(c.close)
        c.start_follow([(0,0,0,1,0)],'LEFT',0)
        return c

    def command(self,c,t):
        speed,steer=c.execute('turn','left_lock_tick',t).value
        return encode_command(speed,steer,c.cfg,0)

    def test_exact_stages_and_wait_for_lane(self):
        c=self.make()
        for t,raw in [(0,0),(.25,0),(.5,0),(.75,0),(1,0),(1.25,0),(1.5,22),
                      (1.75,22),(2,22),(2.25,22),(2.5,22),(2.75,22),(3,22),
                      (3.25,22),(3.5,22),(3.75,22),(4,22),(4.25,22),(4.5,22),(4.75,22)]:
            out=self.command(c,t)
            self.assertEqual((out['speed_raw'],out['steering_raw']),(30,raw))
        self.assertEqual(c.execute('turn','left_lock_tick',5).value,(0,0))
        self.assertEqual(c.left_lock['phase'],'WAIT_LANE')
        self.assertEqual(c.execute('turn','left_lock_tick',5.1).value,(0,0))
        self.assertEqual(c.state,'MANEUVER')

    def test_stop_latches_fault_during_motion(self):
        c=self.make();c.execute('turn','left_lock_tick',0).value
        c.stop('scan_missing_or_stale')
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(c.tick(.1),(0,0))

    def test_long_control_gap_aborts(self):
        c=self.make();c.execute('turn','left_lock_tick',0).value
        self.assertEqual(c.execute('turn','left_lock_tick',.51).value,(0,0))
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(c.reason,'left_timed_control_gap')
        self.assertEqual(c.tick(.6),(0,0))

    def test_backward_control_clock_aborts(self):
        c=self.make();self.command(c,1.)
        self.assertEqual(c.execute('turn','left_lock_tick',.9).value,(0,0))
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(c.reason,'left_timed_control_gap')

    def test_recorded_scheduler_delay_continues_entry(self):
        c=self.make()
        started=1791261519.768466
        self.command(c,started)
        out=self.command(c,1791261520.063441)
        self.assertEqual((out['speed_raw'],out['steering_raw']),(30,0))
        self.assertEqual(c.state,'MANEUVER')
        self.assertEqual(c.left_lock['phase'],'TIMED_ENTRY')

    def test_expired_command_time_does_not_shorten_entry(self):
        c=self.make()
        for now in (0.,.4,.65,.9,1.15,1.4,1.5):
            out=self.command(c,now)
            self.assertEqual((out['speed_raw'],out['steering_raw']),(30,0))
        self.assertAlmostEqual(c.left_lock['timed_started'],.15)
        self.assertEqual(self.command(c,1.66)['steering_raw'],22)

    def test_expired_command_time_does_not_shorten_turn(self):
        c=self.make()
        for now in (0.,.25,.5,.75,1.,1.25,1.5):
            self.command(c,now)
        for now in (1.9,2.15,2.4,2.65,2.9,3.15,3.4,3.65,3.9,4.15,4.4,4.65,4.9,5.):
            out=self.command(c,now)
            self.assertEqual((out['speed_raw'],out['steering_raw']),(30,22))
        self.assertAlmostEqual(c.left_lock['timed_started'],1.65)
        self.assertEqual(c.execute('turn','left_lock_tick',5.16).value,(0,0))
        self.assertEqual(c.left_lock['phase'],'WAIT_LANE')

    def test_left_axle_reference_without_extra_advance(self):
        c=self.make()
        c.blue_approach=dict(phase='STOP_LINE',point=(c.cfg['wheelbase'],0),yaw=0,
                            observed_stamp=1,started=1)
        c.front_marker_stamp=1
        c.execute('turn','blue_approach_tick',1).value
        self.assertEqual(c.blue_approach['stop_reference'],'front_axle')
        self.assertEqual(c.blue_approach['phase'],'STOP')
        self.assertEqual(c.state,'BLUE_STOP')

    def test_confirmation_returns_to_lane(self):
        c=self.make()
        c.left_lock['phase']='WAIT_LANE'
        c.handoff_lane_confirmed=lambda now:True
        self.assertEqual(c.execute('turn','left_lock_tick',1).value,(0,0))
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.left_lock)


if __name__=='__main__':unittest.main()
