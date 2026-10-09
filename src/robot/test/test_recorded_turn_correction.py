"""Observed right-bend paths must not lose their turn to offset feedback."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class RecordedTurnCorrectionTests(unittest.TestCase):
    # Current centers reprocessed from the user's 072716 run, not invented rays.
    ENTRY = [(.666766,.062190),(.692062,.067932),(.760680,.078659),
             (.834641,.082164),(.908530,.077262),(.989439,.062362)]
    BEND = [(.654550,.064216),(.726701,.059630),(.816881,.046910),
            (.900019,.028365),(.984991,.002743),(1.005889,-.004580)]

    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False,lane_curvature_preview=True,
                   steering_command_scale_rad=.03,lane_curve_speed_raw=16)
        cfg['speed_raw']['lane']=24
        c=Controller(cfg)
        self.addCleanup(c.close)
        return c

    def command(self,c,path,stamp):
        c.observe_lane(path,.8,stamp)
        speed,steer=c.lane_command(stamp)
        return speed,encode_command(speed,steer,c.cfg,0)['steering_raw']

    def test_right_entry_has_effective_right_command(self):
        speed,raw=self.command(self.core(),self.ENTRY,1.)
        self.assertEqual(speed,16)
        self.assertLessEqual(raw,-4)

    def test_right_bend_feedback_does_not_reverse_steering(self):
        speed,raw=self.command(self.core(),self.BEND,1.)
        self.assertEqual(speed,16)
        self.assertLess(raw,0)

    def test_left_mirror_uses_equal_opposite_angle(self):
        for path in (self.ENTRY,self.BEND):
            right=self.command(self.core(),path,1.)[1]
            left=self.command(self.core(),[(x,-y) for x,y in path],1.)[1]
            self.assertEqual(left,-right)

    def test_straight_with_center_offset_can_still_correct_towards_center(self):
        c=self.core()
        path=[(.5,.08),(.7,.08),(.9,.08),(1.1,.08)]
        self.assertGreater(self.command(c,path,1.)[1],0)

    def test_bypass_left_and_right_use_action24_instead_of_curve16(self):
        from test_continuous_obstacles import _SyntheticScan
        c=self.core()
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        c.cfg['speed_raw']['action']=24
        c.action='BYPASS'
        c.state='TIMED_BYPASS'
        for phase,elapsed,expected in (('LEFT',0.,22),('RIGHT',0.,-22)):
            c.timed_bypass=dict(phase=phase,elapsed_s=elapsed,last=1.)
            c.scan=_SyntheticScan(1.1)
            command=c.execute('obstacle','timed_bypass_tick',1.1).value
            raw=encode_command(command[0],command[1],c.cfg,0)
            self.assertEqual((raw['speed_raw'],raw['steering_raw']),(24,expected))


if __name__ == '__main__':
    unittest.main()
