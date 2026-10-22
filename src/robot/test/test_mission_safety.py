"""Competition policy regressions; no ROS master or actuator required."""
import os
import sys
import unittest
import math
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.master.controller import Controller
from robot.common.planning import intersection_path
from robot.common.contracts import encode_command
from robot.common.contracts import command_to_model_steering
from robot.common.geometry import bicycle


class MissionSafetyTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
            cfg = yaml.safe_load(stream)
        cfg.update(wait_green=False, lidar_enabled=False, sign_ttl=0,
                   right_turn_full_lock=True, right_exit_on_blue=True)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def front(self, stamp, markers=None):
        self.c.observe_ground(dict(source='front', part='markers',
                                   markers=markers or [], slots=[]), stamp)

    def test_straight_sign_alone_never_drives_without_lane(self):
        for stamp in (1, 1.1):
            self.c.observe_sign('STRAIGHT', .99, stamp, stamp)
        self.front(1.2)
        self.assertEqual(self.c.tick(1.2), (0, 0))
        self.assertEqual(self.c.pending, 'STRAIGHT')
        self.assertIsNone(self.c.action)

    def test_right_blue_wait_has_finite_deadline(self):
        self.c.start_follow(intersection_path(self.c.pose, 'RIGHT', self.c.cfg), 'RIGHT', 1)
        self.c.right_lock['phase'] = 'LOCK'
        now = 2 + self.c.cfg['action_timeout']
        self.front(now)
        self.assertEqual(self.c.tick(now), (0, 0))
        self.assertEqual(self.c.state, 'FAULT')
        self.assertEqual(self.c.reason, 'action_timeout')

    def test_cached_blue_cannot_trigger_after_front_stream_dies(self):
        self.c.pending = 'LEFT'
        self.front(1, [dict(kind='junction', x=.3, y=0)])
        self.c.tick(1 + self.c.cfg['ground_timeout'] + .01)
        self.assertIsNone(self.c.action)

    def test_exit_votes_do_not_accumulate_across_long_frame_gap(self):
        self.c.start_follow(intersection_path(self.c.pose, 'LEFT', self.c.cfg), 'LEFT', 0)
        self.c.set_pose(self.c.exit_pose, 1)
        for stamp in (1, 1.1, 3):
            self.c.observe_lane([(.3, 0), (.5, 0), (.7, 0)], .9, stamp)
            result = self.c.exit_lane_confirmed(stamp)
        self.assertFalse(result)

    def test_camera_loss_is_not_a_paint_gap(self):
        self.c.observe_lane([(.3,0),(.5,0),(.7,0)], .9, 1)
        self.front(1)
        self.assertGreater(self.c.tick(1)[0], 0)
        self.assertEqual(self.c.tick(2), (0,0))
        self.assertEqual(self.c.reason, 'lane_stream_stale')

    def test_left_path_cannot_continue_when_all_front_processing_stops(self):
        self.c.start_follow(intersection_path(self.c.pose, 'LEFT', self.c.cfg), 'LEFT', 1)
        self.c.observe_lane([], 0, 1)
        self.front(1)
        self.assertGreater(self.c.tick(1)[0], 0)
        self.assertEqual(self.c.tick(3), (0,0))
        self.assertEqual(self.c.reason, 'maneuver_front_stale')

    def test_planned_model_angle_is_not_amplified_by_command_scale(self):
        self.c.cfg['steering_command_scale_rad'] = .1
        self.c.start_follow(intersection_path(self.c.pose,'LEFT',self.c.cfg),'LEFT',1)
        self.front(1)
        speed, angle = self.c.follower.command(self.c.pose,1)
        command = self.c.tick(1)
        raw = encode_command(command[0],command[1],self.c.cfg,0)['steering_raw']
        expected = int(round(angle/self.c.cfg['max_steer']*self.c.cfg['steering_raw_limit']))
        self.assertEqual(raw,expected)

    def test_collision_sweep_uses_encoded_steering_model(self):
        self.c.cfg.update(lidar_enabled=True,steering_command_scale_rad=.1)
        self.c.scan = type('Scan', (), dict(stamp=1,valid_rays=10000))()
        captured = []
        def clear(path, unknown, report=False):
            captured.extend(path)
            return True
        self.c.sweep_clear = clear
        self.c.checked_command((20,-.1),1,False)
        expected = bicycle(self.c.pose,self.c.cfg['obstacle_stop_distance'],
                           -self.c.cfg['max_steer'],self.c.cfg['wheelbase'])
        self.assertAlmostEqual(captured[-1][2],expected[2])

    def test_turns_handoff_at_three_speeds_through_raw_model(self):
        # This checks command/model consistency, not measured vehicle stability.
        for raw_speed in (12,20,28):
            for action in ('LEFT','RIGHT'):
                cfg = dict(self.c.cfg,right_turn_full_lock=False,
                           steering_command_scale_rad=.1,right_exit_on_blue=False)
                cfg['speed_raw'] = dict(cfg['speed_raw'],action=raw_speed)
                c = Controller(cfg)
                self.addCleanup(c.close)
                c.start_follow(intersection_path(c.pose,action,cfg),action,0)
                for i in range(1000):
                    now = (i+1)*.05
                    at_exit = c.direction_exit_reached()
                    c.observe_lane([(.3,0),(.5,0),(.7,0)] if at_exit else [],
                                   .9 if at_exit else 0,now)
                    c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),now)
                    speed,command = c.tick(now)
                    if c.state == 'LANE':
                        break
                    physical = command_to_model_steering(command,cfg)
                    gain = cfg['raw_to_mps']['forward' if speed>=0 else 'reverse']
                    c.set_pose(bicycle(c.pose,speed*gain*.05,physical,cfg['wheelbase']),now)
                self.assertEqual(c.state,'LANE',(raw_speed,action,c.reason))
                target = math.pi/2 if action=='LEFT' else -math.pi/2
                self.assertLess(abs(c.pose[2]-target),cfg['exit_yaw_tolerance'])


if __name__ == '__main__':
    unittest.main()
