"""A stable measured lane can end a bypass right turn before its time cap."""
import os
import unittest
from robot.common.config import load_config
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class BypassVisualExitTests(unittest.TestCase):
    CENTER=[(.55,.03),(.70,.03),(.85,.03),(1.,.03)]
    RECORDED=[(.654871,-.161688),(.760287,-.129812),(.858497,-.101381),
              (.937567,-.079377),(.999604,-.062668),(1.103328,-.035819)]

    def core(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=True,timed_bypass_enabled=True,
                   lane_curvature_preview=True,lane_curve_speed_raw=16,
                   steering_command_scale_rad=.03,timed_bypass_right_s=6.)
        cfg['speed_raw'].update(action=24,lane=24)
        c=Controller(cfg);self.addCleanup(c.close)
        c.state='TIMED_BYPASS';c.action='BYPASS';c.issued_steer=-.03
        c.timed_bypass=dict(phase='RIGHT',last=1.,elapsed_s=3.,
            trigger_world=(-.5,0),trigger_point=(.8,0))
        return c

    def tick(self,c,path,stamp,confidence=.8):
        c.observe_lane(path,confidence,stamp)
        c.scan=_SyntheticScan(stamp)
        return c.execute('obstacle','timed_bypass_tick',stamp).value

    def confirm(self,c,path):
        for stamp in (1.1,1.2,1.3):
            command=self.tick(c,path,stamp)
        return command

    def test_stable_current_lane_takes_over_before_six_seconds(self):
        c=self.core();speed,steer=self.confirm(c,self.CENTER)
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.timed_bypass)
        self.assertTrue(c.timed_bypass_completed)
        self.assertEqual(speed,16)
        self.assertGreater(steer,-.03)

    def test_recorded_four_second_view_is_eligible_for_handoff(self):
        c=self.core()
        for stamp in (1.1,1.2,1.3):
            speed,steer=self.tick(c,self.RECORDED,stamp,.389641)
        self.assertIsNone(c.timed_bypass)
        self.assertEqual(speed,16)

    def test_repeated_source_image_does_not_confirm_exit(self):
        c=self.core()
        c.observe_lane(self.CENTER,.8,1.1)
        for now in (1.1,1.15,1.2,1.25,1.3):
            c.scan=_SyntheticScan(now)
            c.execute('obstacle','timed_bypass_tick',now)
        self.assertIsNotNone(c.timed_bypass)

    def test_obstacle_still_ahead_or_too_early_cannot_end_turn(self):
        for elapsed,trigger in [(3.,(.2,0)),(1.,(-.5,0))]:
            c=self.core();c.timed_bypass.update(elapsed_s=elapsed,trigger_world=trigger)
            self.confirm(c,self.CENTER)
            self.assertEqual(c.timed_bypass['phase'],'RIGHT')

    def test_off_axis_or_low_confidence_lane_cannot_end_turn(self):
        for path,confidence in [([(x,.3) for x,y in self.CENTER],.8),
                                ([(.5,0),(.6,.12),(.7,.24),(.8,.36)],.8),
                                (self.CENTER,.2)]:
            c=self.core()
            for stamp in (1.1,1.2,1.3): self.tick(c,path,stamp,confidence)
            self.assertIsNotNone(c.timed_bypass)

    def test_six_second_cap_keeps_existing_partial_lane_handoff(self):
        c=self.core();c.timed_bypass.update(elapsed_s=5.95)
        speed,steer=self.tick(c,[(.6,.03),(.7,.02)],1.05,.329595)
        self.assertIsNone(c.timed_bypass)
        self.assertEqual(speed,16)


if __name__=='__main__':
    unittest.main()
