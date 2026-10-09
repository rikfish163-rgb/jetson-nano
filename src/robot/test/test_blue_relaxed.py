"""Logged second-line boundary and bounded stopped recheck; no hardware."""
import unittest
import test_blue_timed_production as production
from robot.turn.blue_stop_test import BlueStopTest


class BlueRelaxedTests(unittest.TestCase):
    lines=production.BlueTimedProductionTests.lines
    frame=production.BlueTimedProductionTests.frame

    def make(self,action='STRAIGHT'):
        c=production.BlueTimedProductionTests.make(self,action)
        c.cfg['blue_stop_test'].update(align_speed_raw=30,align_tolerance_deg=10,
            heading_tolerance_deg=15,trigger_row_ratio=.7,forward_seconds=1,
            align_recheck_s=1)
        return c

    def test_logged_11_degree_boundary_can_confirm_and_finish(self):
        for action in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            c=self.make(action)
            self.frame(c,1,.466,-20.556)
            self.frame(c,1.2,.560,-17.038)
            self.assertEqual(self.frame(c,1.4,.726,-11.497),(20,0))
            self.assertNotIn('trigger',c.blue_approach['image_timing'])
            self.frame(c,1.5,.75,-9)
            self.frame(c,1.6,.78,-5.26)
            self.assertEqual(c.blue_approach['image_timing']['trigger'],1.6)
            for i in range(1,11):self.frame(c,1.6+i*.1,None)
            self.assertEqual(c.state,'BLUE_STOP')

    def test_excess_angle_holds_zero_until_three_fresh_good_frames(self):
        c=self.make()
        self.assertEqual(self.frame(c,1,.71,-18),(0,0))
        self.assertEqual(c.reason,'straight_blue_recheck')
        self.assertNotEqual(c.state,'FAULT')
        for t in (1.1,1.2):
            self.assertEqual(self.frame(c,t,.72,-12),(0,0))
        # A repeated image must not release the stopped vehicle.
        c.execute('turn','blue_approach_tick',1.25)
        self.assertNotIn('trigger',c.blue_approach['image_timing'])
        self.assertEqual(self.frame(c,1.3,.73,-8),(20,0))
        self.assertEqual(c.blue_approach['image_timing']['trigger'],1.3)

    def test_recheck_times_out_even_if_last_frame_becomes_good(self):
        c=self.make();self.frame(c,1,.71,18)
        for i in range(1,10):self.frame(c,1+i*.1,.71,18)
        self.assertEqual(self.frame(c,2,.71,0),(0,0))
        self.assertEqual(c.state,'FAULT')
        self.assertIn('alignment_recheck_timeout',c.reason)
        self.assertEqual(c.tick(2.1),(0,0))

    def test_frames_captured_before_pause_cannot_confirm_recheck(self):
        c=self.make()
        c.front_marker_stamp=.8
        c.front_blue_image_lines=self.lines(c,.71,18)
        c.execute('turn','blue_approach_tick',1)
        c.front_marker_stamp=.9
        c.front_blue_image_lines=self.lines(c,.72,0)
        self.assertEqual(c.execute('turn','blue_approach_tick',1.1).value,(0,0))
        self.assertEqual(c.blue_approach['image_timing']['align_frames'],0)
        self.assertNotIn('trigger',c.blue_approach['image_timing'])

    def test_sensor_and_guard_stops_still_latch(self):
        c=self.make();self.frame(c,1,.71,18)
        c.stop('scan_missing_or_stale')
        self.assertEqual(c.state,'FAULT')
        c=self.make();self.frame(c,1,.71,18)
        for t in (1.2,1.4,1.6):c.execute('turn','blue_approach_tick',t)
        c.execute('turn','blue_approach_tick',1.7)
        self.assertEqual(c.state,'FAULT')
        self.assertIn('sensor_lost',c.reason)
        c=self.make();self.frame(c,1,.71,18)
        self.assertEqual(self.frame(c,1.1,.91,0),(0,0))
        self.assertIn('past_trigger_band',c.reason)

    def test_recheck_and_normal_test_outputs_are_identical(self):
        c=self.make()
        test=BlueStopTest(c.cfg,c.cfg['blue_stop_test']);self.addCleanup(test.close)
        test.scan_ready=lambda now:True
        test.checked_command=lambda command,now,allow_bypass:command
        for t,row,angle in [(1,.71,18),(1.1,.72,12),(1.2,.73,8),(1.3,.74,5)]:
            test.observe_ground(dict(source='front',part='markers',blue_lines=self.lines(c,row,angle)),t)
            self.assertEqual(self.frame(c,t,row,angle),test.tick(t))
        self.assertFalse(test.test_done)
        self.assertEqual(test.test_trigger,1.3)


if __name__=='__main__':unittest.main()
