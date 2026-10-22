"""Exercise the real adapter with delayed control ticks and RAW feedback."""
from __future__ import division
import unittest
from collections import deque
import test_ros_adapter as fixture


@unittest.skipUnless(fixture._rospy is not None and fixture.imp is not None, 'requires ROS')
class MotionFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.env = fixture.RosAdapterTests('test_applied_raw_over_contract_clears_command')
        self.env.setUpClass(); self.env.setUp()
        self.addCleanup(self.env.doCleanups)
        self.n = self.env.node
        self.n.cfg['pose_mode'] = 'command_model'
        self.n.seq,self.n.live,self.n.last_tick = 0,False,10.
        self.n.history = deque(maxlen=100)
        self.n.output = type('Sink',(),{'publish':lambda self,msg:None})()
        self.n.core.tick = lambda now:(0,0.)

    def applied(self, stamp, speed=20, steer=0):
        self.env.clock.value = stamp
        msg = fixture._AppliedMessage(speed, steer)
        msg.header = fixture._Header(stamp,'base_link')
        self.n.applied_cb(msg)

    def tick(self, now):
        self.env.clock.value = now
        self.n.tick(None)
        self.assertFalse(self.env.fake_rospy.errors)

    def test_busy_tick_does_not_drop_distance(self):
        self.applied(10.)
        self.tick(10.2)
        self.assertAlmostEqual(self.n.core.pose[0], .032)

    def test_stop_feedback_keeps_motion_before_stop(self):
        self.applied(10.)
        self.applied(10.1,0)
        self.tick(10.2)
        self.assertAlmostEqual(self.n.core.pose[0], .016)

    def test_timeout_counts_only_the_valid_held_interval(self):
        self.applied(10.)
        self.tick(10.4)
        self.assertAlmostEqual(self.n.core.pose[0], .04)

    def test_model_mapping_does_not_change_at_uturn_handoff(self):
        from robot.common.config import load_config
        import os
        self.n.cfg = load_config(os.path.join(fixture.PACKAGE_DIR,'config'))
        self.n.cfg.update(steering_command_scale_rad=.03, max_steer=.2)
        self.n.core.cfg = self.n.cfg
        self.n.core.state = 'UTURN'
        self.applied(10.,26,22)
        uturn_angle = self.n.applied[1]
        self.n.core.state = 'MANEUVER'
        self.applied(10.1,26,22)
        self.assertAlmostEqual(self.n.applied[1],uturn_angle)

    def test_real_straight_handoff_with_sparse_control_ticks(self):
        from robot.common.config import load_config
        from robot.master.controller import Controller
        from robot.turn.planner import intersection_path
        from robot.motion.calibration import speed_gain
        import os
        cfg=load_config(os.path.join(fixture.PACKAGE_DIR,'config'))
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03,
                   straight_speed_raw=20)
        n=self.n
        n.cfg=cfg;n.core=Controller(cfg)
        self.addCleanup(n.core.close)
        n.core.start_follow(intersection_path((0.,0.,0.),'STRAIGHT',cfg),'STRAIGHT',10.)
        # Feedback continues at 20 Hz; the controller wakes only every 0.2 s.
        for i in range(160):
            now=10.+i*.05
            self.applied(now)
            if i%4:continue
            n.core.front_marker_stamp=now
            n.core.observe_lane([(.5,0.),(.8,0.)],.99,now)
            self.tick(now)
            if n.core.state=='LANE':break
            self.assertEqual(n.core.state,'MANEUVER')
        velocity=20*speed_gain(cfg,20)
        self.assertEqual(n.core.state,'LANE')
        self.assertGreaterEqual(n.core.pose[0],1.25)
        self.assertLess(n.core.pose[0],1.25+.2*velocity+1e-9)
        self.assertLess(now-10.,1.25/velocity+.2+1e-9)


if __name__ == '__main__': unittest.main()
