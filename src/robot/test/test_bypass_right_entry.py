"""Do not enter a right arc that immediately returns into the side obstacle."""
import os
import unittest
from robot.common.config import load_config
from robot.common.geometry import bicycle
from robot.common.contracts import command_to_model_steering
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class BypassRightEntryTests(unittest.TestCase):
    POINT = (.45,-.18)

    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=True,timed_bypass_enabled=True,
                   steering_command_scale_rad=.03,timed_bypass_left_s=2.,
                   timed_bypass_right_s=6.,timed_bypass_speed_raw=24)
        cfg['speed_raw']['action']=24
        c=Controller(cfg);self.addCleanup(c.close)
        c.state='TIMED_BYPASS';c.action='BYPASS'
        c.timed_bypass=dict(phase='LEFT',last=1.,elapsed_s=1.99)
        return c

    def test_clear_imminent_right_sweep_is_not_enough_to_end_left(self):
        c=self.core();c.scan=_SyntheticScan(1.05,obstacles=[self.POINT])
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.05).value,(24,.03))
        self.assertEqual(c.timed_bypass['phase'],'LEFT')
        self.assertEqual(c.reason,'bypass_extend_left_for_clearance')

    def test_clear_right_entry_preserves_nominal_two_second_switch(self):
        c=self.core();c.scan=_SyntheticScan(1.05)
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.05).value,(24,-.03))
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')
        self.assertEqual(c.timed_bypass['elapsed_s'],0.)

    def test_production_left_switches_at_one_point_five_seconds_when_clear(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        c=self.core()
        for key in ('timed_bypass_left_s','timed_bypass_right_s','timed_bypass_speed_raw'):
            c.cfg[key]=cfg[key]
        c.timed_bypass.update(elapsed_s=1.29,last=1.)
        for now in (1.1,1.2):
            c.scan=_SyntheticScan(now)
            self.assertEqual(c.execute('obstacle','timed_bypass_tick',now).value,(26,.03))
        c.scan=_SyntheticScan(1.21)
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.21).value,(26,-.03))
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')
        self.assertEqual(c.cfg['timed_bypass_right_s'],4.)

    def test_left_extension_releases_right_when_measured_obstacle_clears(self):
        c=self.core();released=False
        for i in range(1,31):
            now=1.+i*.05
            c.scan=_SyntheticScan(now,obstacles=[self.POINT])
            speed,steer=c.execute('obstacle','timed_bypass_tick',now).value
            self.assertEqual(speed,24)
            if c.timed_bypass['phase']=='RIGHT':
                self.assertEqual(steer,-.03)
                released=True
                break
            c.set_pose(bicycle(c.pose,speed*c.cfg['raw_to_mps']['forward']*.05,
                command_to_model_steering(steer,c.cfg),c.cfg['wheelbase']),now)
        self.assertTrue(released)

    def test_left_extension_still_checks_imminent_obstacle_and_coverage(self):
        for obstacles,classification in [([self.POINT,(.20,0)],'FREE'),([self.POINT],'UNKNOWN')]:
            c=self.core();c.scan=_SyntheticScan(1.05,obstacles=obstacles,classification=classification)
            self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.05).value[0],0)

    def test_blocked_right_entry_has_bounded_left_extension(self):
        c=self.core();c.timed_bypass['elapsed_s']=3.49
        c.scan=_SyntheticScan(1.05,obstacles=[self.POINT])
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.05).value[0],0)
        self.assertEqual(c.reason,'bypass_right_entry_blocked')


if __name__=='__main__':
    unittest.main()
