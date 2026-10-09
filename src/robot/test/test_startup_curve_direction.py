"""Green-start handoff follows the legacy target path without a bend override."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class StartupCurveDirectionTests(unittest.TestCase):
    def core(self):
        root=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg=load_config(os.path.join(root,'config'))
        cfg.update(wait_green=True,lidar_enabled=False,steering_command_scale_rad=.1)
        cfg['speed_raw']['lane']=20
        c=Controller(cfg)
        self.addCleanup(c.close)
        for t in (1.,1.05,1.1):
            c.observe_sign('GREEN',.99,t,t)
        return c

    def handoff(self,c,path,boundaries):
        c.set_pose((1.25,0.,0.),1.3)
        c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.3)
        c.observe_lane(path,.85,1.3,boundaries)
        return c.tick(1.3)

    def test_both_turn_directions_follow_path_at_handoff(self):
        xs=(.35,.5,.65,.8,.95)
        for sign in (-1,1):
            c=self.core()
            path=[(x,sign*.16) for x in xs]
            # Boundary curvature cannot reverse or stop the requested path.
            boundaries={'LEFT':[(x,.3-sign*.48*(x-.35)**2) for x in xs],
                        'RIGHT':[(x,-.3-sign*.48*(x-.35)**2) for x in xs]}
            speed,steer=self.handoff(c,path,boundaries)
            self.assertEqual(c.state,'LANE')
            self.assertIsNone(c.startup_curve_lock)
            self.assertEqual(speed,20)
            self.assertGreater(sign*encode_command(speed,steer,c.cfg,0)['steering_raw'],0)

    def test_centered_path_keeps_zero_steering(self):
        c=self.core()
        self.assertEqual(self.handoff(c,[(.5,0),(.8,0)],{}),(20,0))
        self.assertIsNone(c.startup_curve_lock)

    def test_missing_path_stops_at_handoff(self):
        c=self.core()
        self.assertEqual(self.handoff(c,[],{}),(0,0))
        self.assertIsNone(c.startup_curve_lock)

    def test_startup_raw_trim_ignores_lane_and_ends_at_handoff(self):
        c=self.core()
        c.cfg['startup_steering_raw']=-2
        c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.2)
        c.observe_lane([(.4,.2),(.8,.3)],.9,1.2)
        speed,steer=c.tick(1.2)
        self.assertEqual(speed,20)
        self.assertEqual(encode_command(speed,steer,c.cfg,0)['steering_raw'],-2)
        self.assertEqual(c.state,'STARTUP_STRAIGHT')
        self.assertEqual(self.handoff(c,[(.5,0),(.8,0)],{}),(20,0))

    def test_startup_trim_does_not_override_stale_sensor_stop(self):
        c=self.core()
        c.cfg['startup_steering_raw']=-2
        self.assertEqual(c.tick(2.),(0,0))

    def test_startup_trim_encoding_respects_sign_and_scale(self):
        for sign in (-1,1):
            c=self.core()
            c.cfg.update(startup_steering_raw=-2,steering_sign=sign,
                         steering_command_scale_rad=.2)
            c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),1.2)
            speed,steer=c.tick(1.2)
            self.assertEqual(encode_command(speed,steer,c.cfg,0)['steering_raw'],-2)


if __name__=='__main__':unittest.main()
