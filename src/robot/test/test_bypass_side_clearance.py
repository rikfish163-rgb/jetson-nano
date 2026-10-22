"""Blocked timed steering may only continue on a fully checked safe sweep."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class BypassSideClearanceTests(unittest.TestCase):
    POINT=(.225952906,-.156749123)

    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=True,timed_bypass_enabled=True,
                   steering_command_scale_rad=.03,timed_bypass_speed_raw=24)
        cfg['speed_raw']['action']=24
        c=Controller(cfg);self.addCleanup(c.close)
        c.state='TIMED_BYPASS';c.action='BYPASS'
        c.timed_bypass=dict(phase='RIGHT',last=1.,elapsed_s=1.515)
        return c

    def test_recorded_side_return_uses_safe_sweep_at_action_speed(self):
        c=self.core();c.scan=_SyntheticScan(1.05,obstacles=[self.POINT])
        speed,steer=c.execute('obstacle','timed_bypass_tick',1.05).value
        self.assertEqual(speed,24)
        self.assertGreater(steer,-.03)
        self.assertEqual(c.reason,'bypass_side_clearance')
        before=c.timed_bypass['elapsed_s']
        c.scan=_SyntheticScan(1.1,obstacles=[self.POINT])
        c.execute('obstacle','timed_bypass_tick',1.1)
        self.assertEqual(c.timed_bypass['elapsed_s'],before)

    def test_actual_body_collision_or_unknown_space_still_stops(self):
        for obstacles,kind in [([(.20,0)],'FREE'),([self.POINT],'UNKNOWN')]:
            c=self.core();c.scan=_SyntheticScan(1.05,obstacles=obstacles,classification=kind)
            self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.05).value[0],0)

    def test_safe_clearance_cannot_extend_motion_without_bound(self):
        c=self.core();c.scan=_SyntheticScan(1.05,obstacles=[self.POINT])
        c.execute('obstacle','timed_bypass_tick',1.05)
        c.scan=_SyntheticScan(2.6,obstacles=[self.POINT])
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',2.6).value[0],0)

    def test_recorded_point_clears_while_vehicle_advances(self):
        from robot.common.geometry import bicycle
        from robot.common.contracts import command_to_model_steering
        c=self.core();resumed=False
        for i in range(1,30):
            now=1.+i*.05
            c.scan=_SyntheticScan(now,obstacles=[self.POINT])
            speed,steer=c.execute('obstacle','timed_bypass_tick',now).value
            self.assertEqual(speed,24)
            if steer==-.03:
                resumed=True
                break
            c.set_pose(bicycle(c.pose,speed*.008*.05,
                command_to_model_steering(steer,c.cfg),c.cfg['wheelbase']),now)
        self.assertTrue(resumed)

    def test_new_clear_scan_resumes_original_right_phase(self):
        c=self.core();c.scan=_SyntheticScan(1.05,obstacles=[self.POINT])
        c.execute('obstacle','timed_bypass_tick',1.05)
        c.scan=_SyntheticScan(1.1)
        self.assertEqual(c.execute('obstacle','timed_bypass_tick',1.1).value,(24,-.03))


if __name__=='__main__':
    unittest.main()
