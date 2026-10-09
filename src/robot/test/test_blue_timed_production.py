"""Shared calibrated blue approach for four actions; parking stays isolated."""
import copy
import math
import os
import unittest
import yaml
from test_core import CONFIG
from robot.master.controller import Controller
from robot.common.contracts import encode_command
from robot.turn.blue_stop_test import BlueStopTest

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with open(os.path.join(ROOT,'turn','blue_stop_test.yaml')) as stream:
    SETTINGS=yaml.safe_load(stream)
# Fixed regression scenario; field tuning must not change its expected timings.
SETTINGS.update(speed_raw=20,align_speed_raw=20,trigger_row_ratio=.65,
                trigger_band_ratio=.2,forward_seconds=1.9,confirm_frames=3,
                align_tolerance_deg=5,heading_tolerance_deg=10,
                align_confirm_frames=3,align_recheck_s=0)


class BlueTimedProductionTests(unittest.TestCase):
    def make(self,action):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(blue_timed_enabled=True,blue_stop_test=dict(SETTINGS),
                   lidar_enabled=False,wait_green=False,intersection_wait_s=.5,
                   steering_command_scale_rad=.03)
        c=Controller(cfg);self.addCleanup(c.close)
        c.action=action;c.state='BLUE_APPROACH'
        c.blue_approach=dict(image_timed=True,phase='STOP_LINE',point=(1,0),yaw=0,
                            observed_stamp=1,started=1)
        return c

    def lines(self,c,row,angle=0):
        if row is None:return []
        cam=c.cfg['front_camera']
        return [dict(x=(cam['origin_v']-row*(cam['bev_height']-1))/cam['pixels_per_m'],
                     y=0,yaw=math.radians(angle),length=.6)]

    def frame(self,c,t,row,angle=0):
        c.front_marker_stamp=t
        c.front_blue_image_lines=self.lines(c,row,angle)
        return c.execute('turn','blue_approach_tick',t).value

    def test_all_four_stop_after_calibrated_time_then_dispatch(self):
        for action in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            c=self.make(action)
            self.frame(c,.8,.2);self.frame(c,.9,.25)
            self.frame(c,1,.3)
            for t in (1.1,1.2,1.3):
                command=self.frame(c,t,.7)
                raw=encode_command(command[0],command[1],c.cfg,0)
                self.assertEqual((raw['speed_raw'],raw['steering_raw']),(20,0))
            self.assertAlmostEqual(c.blue_approach['image_timing']['trigger'],1.3)
            for i in range(1,19):self.frame(c,1.3+.1*i,None)
            self.assertEqual(self.frame(c,3.21,None),(0,0))
            self.assertEqual(c.state,'BLUE_STOP')
            calls=[]
            c.begin_blue_action=lambda now:calls.append(now) or (0,0)
            self.frame(c,3.5,None);self.assertFalse(calls)
            self.frame(c,3.72,None);self.assertEqual(len(calls),1)

    def test_same_image_cannot_supply_three_votes(self):
        c=self.make('LEFT')
        for t in (.7,.8,.9):self.frame(c,t,.3)
        self.frame(c,1,.7)
        c.execute('turn','blue_approach_tick',1.1)
        self.assertEqual(c.blue_approach['image_timing']['confirmations'],1)

    def test_protection_stop_latches_fault(self):
        c=self.make('RIGHT');self.frame(c,1,.3)
        c.stop('scan_missing_or_stale')
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(c.tick(1.1),(0,0))

    def test_all_four_correct_skew_before_trigger(self):
        for action in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            for angle in (-19.98,19.98):
                c=self.make(action)
                speed,steer=self.frame(c,1,.4745,angle)
                self.assertGreater(speed,0)
                self.assertGreater(steer*angle,0)
                for t,row in [(1.1,.5),(1.2,.55),(1.3,.6)]:self.frame(c,t,row,2)
                self.assertTrue(c.blue_approach['image_timing']['aligned'])
                for t in (1.4,1.5,1.6):self.frame(c,t,.7,2)
                self.assertEqual(c.blue_approach['image_timing']['trigger'],1.6)

    def test_unresolved_recorded_skew_stops_without_dispatch(self):
        c=self.make('STRAIGHT');self.frame(c,1,.4745,-19.98)
        for t in (1.1,1.2,1.3):self.frame(c,t,.6,-15)
        self.assertEqual(self.frame(c,1.4,.697,-11.84),(0,0))
        self.assertEqual(c.state,'FAULT')
        self.assertIn('alignment_distance_insufficient',c.reason)
        self.assertNotIn('trigger',c.blue_approach['image_timing'])

    def test_parking_does_not_use_image_timing_even_if_flag_present(self):
        c=self.make('PARKING')
        c.blue_approach.update(phase='STOP',stop_until=2)
        c.front_marker_stamp=1
        self.assertEqual(c.execute('turn','blue_approach_tick',1).value,(0,0))
        self.assertNotIn('image_timing',c.blue_approach)
        self.assertNotEqual(c.state,'FAULT')

    def test_test_entry_and_production_share_identical_outputs(self):
        c=self.make('STRAIGHT')
        test=BlueStopTest(copy.deepcopy(c.cfg),dict(SETTINGS));self.addCleanup(test.close)
        test.scan_ready=lambda now:True
        test.checked_command=lambda command,now,allow_bypass:command
        frames=[(.6,.1,-20),(.7,.15,-10),(.8,.2,2),(.9,.25,2),(1,.3,2),
                (1.1,.7,2),(1.2,.7,2),(1.3,.7,2)]
        for t,row,angle in frames+[(1.3+.1*i,None,0) for i in range(1,21)]:
            test.observe_ground(dict(source='front',part='markers',blue_lines=self.lines(c,row,angle)),t)
            self.assertEqual(self.frame(c,t,row,angle),test.tick(t))


if __name__=='__main__':unittest.main()
