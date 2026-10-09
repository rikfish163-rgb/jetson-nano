"""Timed UTURN regressions with current lane limits; never starts ROS nodes."""
from __future__ import division
import math
import os
import unittest

from robot.common.config import load_config
from robot.common.geometry import bicycle, distance, wrap, world
from robot.master.controller import Controller
from robot.uturn.timed import TimedUturn
from robot.uturn.calibration import raw_angle, speed_gain, command_angle, trial_active
from robot.common.contracts import encode_command, validate_config


class CurrentModelTests(unittest.TestCase):
    def setUp(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.cfg = load_config(os.path.join(root, 'config'))
        self.cfg.update(wait_green=False, lidar_enabled=False,
                        steering_command_scale_rad=.03, max_steer=.2)
        self.c = Controller(self.cfg)
        self.addCleanup(self.c.close)

    def finish(self, yaw):
        c = self.c
        c.state, c.action, c.action_started = 'UTURN', 'UTURN', 1.
        c.wait_until = 0.
        c.uturn = dict(trial_last_yaw=yaw, trial_turn_rad=yaw)
        c.set_pose((0., 0., yaw), 10.)
        c.timed_uturn = TimedUturn(self.cfg)
        c.timed_uturn.index = len(c.timed_uturn.steps)

    def lane(self, now, heading=0., lateral=0.):
        c = self.c
        c.front_marker_stamp = now
        c.observe_lane([(.5, lateral), (.8, lateral+.3*math.tan(heading))], .99, now)

    def test_ninety_degree_turn_cannot_handoff_to_a_crossing_lane(self):
        self.finish(math.pi/2)
        for i in range(4):
            now = 10.+i*.1
            self.lane(now)
            command = self.c.tick(now)
            self.assertEqual(command, (26, .03))
            self.assertEqual(self.c.state, 'UTURN')

    def test_exit_needs_new_aligned_frames_without_a_stop_pulse(self):
        self.finish(math.pi)
        self.lane(10., heading=math.radians(45))
        command=self.c.tick(10.)
        self.assertEqual(command[0],26)
        self.assertGreater(command[1],0.)
        for i in range(1, 4):
            now = 10.+i*.1
            self.lane(now)
            command = self.c.tick(now)
            self.assertGreater(command[0], 0)
            if i < 3:
                self.assertEqual(self.c.state, 'UTURN')
                self.assertEqual(command, (26, 0.))
                self.c.tick(now+.01)  # Same camera image cannot add a vote.
                self.assertEqual(self.c.state, 'UTURN')
                self.assertGreater(self.c.timed_uturn.elapsed, 0.)
        self.assertEqual(self.c.state, 'LANE')

    def test_exit_can_correct_the_other_direction_after_overshoot(self):
        self.finish(math.radians(205))
        self.lane(10.,heading=math.radians(-35))
        command=self.c.tick(10.)
        self.assertEqual(command[0],26)
        self.assertLess(command[1],0.)
        self.assertEqual(self.c.state,'UTURN')
        self.assertEqual(self.c.action,'UTURN')

    def test_exit_correction_is_bounded_and_stale_camera_stops(self):
        self.finish(math.pi/2)
        self.lane(10.)
        self.c.tick(10.)
        elapsed = self.c.timed_uturn.elapsed
        self.assertEqual(self.c.tick(11.3), (0, 0.))
        self.assertEqual(self.c.timed_uturn.elapsed, elapsed)
        for i in range(55):
            now = 11.4+i*.1
            self.lane(now)
            self.c.tick(now)
            if self.c.state == 'FAULT':
                break
        self.assertEqual(self.c.state, 'FAULT')
        self.assertEqual(self.c.reason, 'uturn_trial_exit_not_aligned')

    def test_preview_stops_at_reverse_segment_endpoint(self):
        c = self.c
        c.cfg['lidar_enabled'] = True
        c.state, c.action = 'UTURN', 'UTURN'
        c.timed_uturn = TimedUturn(self.cfg)
        c.timed_uturn.index = next(i for i, step in enumerate(c.timed_uturn.steps)
                                  if step[0] == 'SEGMENT_6')
        paths = []
        c._runtime.overrides['scan_ready'] = lambda now: True
        c._runtime.overrides['sweep_clear'] = lambda path, *args, **kwargs: paths.append(path) or True
        self.assertEqual(c.checked_command((-26, -.03), 1., False), (-26, -.03))
        endpoint = paths[-1][-1]
        expected = bicycle(c.pose, -26*.0092692308*.8298755159,
                           -math.atan(.26/1.3608228346), .26)
        self.assertLess(distance(endpoint, expected), 1e-7)
        self.assertLess(abs(wrap(endpoint[2]-expected[2])), 1e-7)
        # The same wire command in normal lane mode keeps the shared .2 model.
        c.state,c.action='LANE',None
        c.checked_command((-26,-.03),1.,False)
        expected=bicycle(c.pose,-c.cfg['obstacle_stop_distance'],-.2,.26)
        self.assertLess(distance(paths[-1][-1],expected),1e-7)
        self.assertLess(abs(wrap(paths[-1][-1][2]-expected[2])),1e-7)

    def test_calibration_is_independent_of_lane_limit_and_raw_roundtrips(self):
        for reverse_sign in (-1, 1):
            self.cfg['uturn_calibration']['reverse_steering_sign'] = reverse_sign
            for speed in (-26, 26):
                for raw in (-22, -11, 0, 11, 22):
                    angle = raw_angle(self.cfg, speed, raw)
                    command = command_angle(self.cfg, 1 if speed > 0 else -1, angle)
                    self.assertEqual(encode_command(speed,command,self.cfg,0)['steering_raw'],raw)
                    changed = dict(self.cfg,max_steer=.46275)
                    self.assertEqual(raw_angle(changed,speed,raw),angle)
        self.assertAlmostEqual(26*speed_gain(self.cfg,26),.25,places=7)
        self.assertTrue(trial_active(self.cfg,'UTURN'))
        for state in ('LANE','MANEUVER','TIMED_BYPASS','PARKING'):
            self.assertFalse(trial_active(self.cfg,state))

    def test_blocked_exit_pauses_time_and_keeps_collision_protection(self):
        self.finish(math.pi/2)
        c=self.c
        c.cfg['lidar_enabled']=True
        c._runtime.overrides['scan_ready']=lambda now: True
        # Real raw-return sweep with a point in the current body.
        class BodyScan(object):
            stamp=10.
            shape_filter=False
            obstacles=[(.2,0.)]
            def evidence(self,point):return {}
            def classify(self,point):return 'FREE'
        c.scan=BodyScan()
        c.scan.obstacles=[world(c.pose,(.2,0.))]
        self.lane(10.)
        self.assertEqual(c.tick(10.),(0,0.))
        self.assertEqual(c.reason,'lidar_obstacle_in_sweep')
        for i in range(1,6):
            now=10.+i*.1;self.lane(now)
            self.assertEqual(c.tick(now),(0,0.))
        self.assertEqual(c.timed_uturn.elapsed,0.)
        c.scan.obstacles=[]
        self.lane(10.6)
        self.assertEqual(c.tick(10.6),(26,.03))
        self.assertEqual(c.timed_uturn.elapsed,0.)

    def test_invalid_exit_limits_are_rejected(self):
        for key,value in (('uturn_trial_exit_max_s',float('nan')),
                          ('uturn_trial_exit_min_turn_deg',181),
                          ('uturn_trial_exit_heading_deg',0)):
            cfg=dict(self.cfg);cfg[key]=value
            with self.assertRaises(ValueError):validate_config(cfg)


if __name__ == '__main__':
    unittest.main()
