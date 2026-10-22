"""Frame evidence for speed changes, startup distance and bypass admission."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class ConfirmedSpeedTests(unittest.TestCase):
    STRAIGHT = [(.5,0),(.7,0),(.9,0),(1.1,0)]
    BEND = [(.5,0),(.7,-.04),(.9,-.12),(1.1,-.24)]

    def core(self, **overrides):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '../config'))
        cfg.update(wait_green=False,lidar_enabled=False,lane_curvature_preview=True,
                   lane_curve_speed_raw=12,steering_command_scale_rad=.03)
        cfg.update(overrides)
        cfg['speed_raw'].update(lane=24,action=12)
        c = Controller(cfg)
        self.addCleanup(c.close)
        return c

    def observe(self,c,path,stamp,confidence=.9):
        c.observe_lane(path,confidence,stamp)
        return c.lane_command(stamp)

    def test_first_straight_and_repeated_ticks_cannot_accelerate(self):
        c=self.core()
        self.assertEqual(self.observe(c,self.STRAIGHT,1.)[0],12)
        for now in (1.1,1.2,1.3,1.4):
            self.assertEqual(c.lane_command(now)[0],12)

    def test_persistent_straight_accelerates_but_bend_and_gap_reduce_immediately(self):
        c=self.core()
        speeds=[self.observe(c,self.STRAIGHT,1.+i*.1)[0] for i in range(20)]
        self.assertEqual(speeds[-1],24)
        self.assertTrue(all(b-a<=2 for a,b in zip(speeds,speeds[1:])))
        self.assertEqual(self.observe(c,self.BEND,3.)[0],12)
        self.assertEqual(self.observe(c,[],3.1)[0],12)
        self.assertEqual(self.observe(c,self.STRAIGHT,3.2)[0],12)

    def test_sparse_or_off_axis_exit_stays_slow(self):
        c=self.core()
        for i in range(12):
            self.assertEqual(self.observe(c,[(.5,.1),(.8,.1),(1.1,.1)],1.+i*.1)[0],12)

    def test_startup_keeps_distance_gate_but_latches_early_lane_slowdown(self):
        c=self.core(straight_speed_raw=24,straight_distance=1.25,startup_follow_lane=False)
        c.execute('mission','begin_startup',1.)
        c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.)
        self.assertEqual(c.tick(1.),(24,0))
        c.set_pose((.4,0,0),1.1)
        c.observe_lane(self.BEND,.9,1.1)
        self.assertEqual(c.tick(1.1),(12,0))
        c.set_pose((1.24,0,0),1.2)
        c.observe_lane([],0.,1.2)
        self.assertEqual(c.tick(1.2),(12,0))
        self.assertEqual(c.state,'STARTUP_STRAIGHT')
        c.set_pose((1.25,0,0),1.3)
        c.observe_lane(self.BEND,.9,1.3)
        speed,steer=c.tick(1.3)
        self.assertEqual((c.state,speed),('LANE',12))
        self.assertLess(steer,0)

    def test_unreliable_lane_cannot_admit_timed_bypass(self):
        c=self.core(lidar_enabled=True,timed_bypass_enabled=True)
        for i in range(4):
            now=1.+i*.2
            c.observe_lane(self.STRAIGHT,.109,now)
            c.scan=_SyntheticScan(now,obstacles=[(.48,-.184)])
            self.assertIsNone(c.execute('obstacle','begin_timed_bypass',now).value)
        self.assertIsNone(c.timed_bypass)

    def test_three_distinct_scans_required_for_target(self):
        c=self.core(lidar_enabled=True,timed_bypass_enabled=True)
        c.observe_lane(self.STRAIGHT,.9,1.)
        c.scan=_SyntheticScan(1.,obstacles=[(.6,0)])
        for now in (1.,1.05,1.1):
            self.assertIsNone(c.execute('obstacle','begin_timed_bypass',now).value)
        for now in (1.2,1.4):
            c.observe_lane(self.STRAIGHT,.9,now)
            c.scan=_SyntheticScan(now,obstacles=[(.6,0)])
            result=c.execute('obstacle','begin_timed_bypass',now).value
        self.assertIsNotNone(result)
        self.assertEqual(c.state,'TIMED_BYPASS')


if __name__ == '__main__':
    unittest.main()
