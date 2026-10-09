"""Current steering model: maneuver commands and visual handoffs, no hardware."""
from __future__ import division
import math
import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller
from robot.turn.planner import intersection_path


class CurrentTurnTests(unittest.TestCase):
    def core(self, action, speed=20):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load_config(os.path.join(root, 'config'))
        # Explicit legacy blue-exit RIGHT; timed production has its own suite.
        cfg['right_timed_enabled'] = False
        cfg.update(wait_green=False, lidar_enabled=False, max_steer=.2,
                   steering_command_scale_rad=.03, left_turn_full_lock=True,
                   left_turn_entry=.12, right_turn_full_lock=True,
                   right_exit_on_blue=True, right_reverse_entry_m=.25,
                   straight_speed_raw=20, straight_distance=1.25)
        cfg['speed_raw'] = dict(cfg['speed_raw'], action=speed, lane=30)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.start_follow(intersection_path(c.pose, action, cfg), action, 1.)
        return c

    def front(self, c, stamp, markers=None):
        c.observe_ground(dict(source='front', part='markers',
                              markers=markers or [], slots=[]), stamp)

    def raw(self, c, stamp):
        speed, steer = c.tick(stamp)
        return encode_command(speed, steer, c.cfg, 0)

    def test_launcher_enables_current_left_profile(self):
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
        with open(os.path.join(root, 'tools/sign_detector_20260916/run_yolo_vehicle.sh')) as f:
            self.assertIn('left_turn_full_lock:=true', f.read())

    def test_left_entry_then_full_lock_at_selected_action_speed(self):
        for speed in (20, 24):
            c = self.core('LEFT', speed)
            self.front(c, 1.1)
            c.set_pose((.119, 0., 0.), 1.1)
            cmd = self.raw(c, 1.1)
            self.assertEqual((cmd['speed_raw'], cmd['steering_raw']), (speed, 0))
            c.set_pose((.12, 0., 0.), 1.2)
            self.front(c, 1.2)
            cmd = self.raw(c, 1.2)
            self.assertEqual((cmd['speed_raw'], cmd['steering_raw']), (speed, 22))

    def test_left_does_not_straighten_at_estimated_arc_end_without_exit(self):
        c = self.core('LEFT')
        c.set_pose(c.exit_pose, 2.)
        self.front(c, 2.)
        cmd = self.raw(c, 2.)
        self.assertEqual((cmd['speed_raw'], cmd['steering_raw']), (20, 22))
        self.assertEqual(c.action, 'LEFT')

    def test_left_visual_exit_can_handoff_before_large_model_arc_endpoint(self):
        c = self.core('LEFT')
        c.set_pose((.4, .3, math.radians(40)), 2.)
        self.assertFalse(c.direction_exit_reached())
        for t in (2., 2.1, 2.2):
            self.front(c, t)
            c.observe_lane([(.35, .03), (.55, .03), (.75, .03)], .9, t)
            cmd = self.raw(c, t)
            if t < 2.2:
                self.assertEqual(cmd['steering_raw'], 22)
                self.assertEqual(c.action, 'LEFT')
        self.assertEqual(c.state, 'LANE')
        self.assertIsNone(c.action)
        self.assertGreater(cmd['speed_raw'], 0)

    def test_left_rejects_entry_lane_crossing_lane_and_outside_path(self):
        cases = [(5, [(.35, .03), (.55, .03), (.75, .03)]),
                 (40, [(.35, 0.), (.55, .2), (.75, .4)]),
                 (40, [(.35, .25), (.55, .25), (.75, .25)])]
        for angle, points in cases:
            c = self.core('LEFT')
            c.set_pose((.4, .3, math.radians(angle)), 2.)
            for t in (2., 2.1, 2.2):
                self.front(c, t)
                c.observe_lane(points, .9, t)
                self.assertEqual(self.raw(c, t)['steering_raw'], 22)
            self.assertEqual(c.action, 'LEFT')

    def test_left_counts_distinct_images_and_resets_after_camera_gap(self):
        c = self.core('LEFT')
        c.set_pose((.4, .3, math.radians(40)), 2.)
        points = [(.35, .03), (.55, .03), (.75, .03)]
        c.observe_lane(points, .9, 2.)
        for t in (2., 2.05, 2.1):
            self.front(c, t)
            self.assertEqual(self.raw(c, t)['steering_raw'], 22)
        for t in (3., 3.1):
            self.front(c, t)
            c.observe_lane(points, .9, t)
            self.assertEqual(self.raw(c, t)['steering_raw'], 22)
        self.assertEqual(c.action, 'LEFT')

    def test_left_without_exit_stops_at_angle_limit(self):
        c = self.core('LEFT')
        c.set_pose((.4, .3, math.radians(125)), 2.)
        self.front(c, 2.)
        self.assertEqual(self.raw(c, 2.)['speed_raw'], 0)
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.reason, 'left_exit_not_found')

    def right_lock(self, c):
        c.set_pose((-.25, 0., 0.), 1.1)
        self.front(c, 1.1)
        self.raw(c, 1.1)  # existing reverse-to-forward braking tick
        self.front(c, 1.2)
        self.raw(c, 1.2)

    def test_right_uses_bounded_current_command_scale(self):
        c = self.core('RIGHT', 24)
        self.right_lock(c)
        self.front(c, 2.)
        speed, steer = c.tick(2.)
        self.assertEqual((speed, steer), (24, -.03))
        self.assertEqual(encode_command(speed, steer, c.cfg, 0)['steering_raw'], -22)

    def test_right_new_blue_requires_distinct_frames_then_straight(self):
        c = self.core('RIGHT')
        self.right_lock(c)
        marker = [dict(kind='junction', x=.8, y=.1)]
        self.front(c, 2., marker)
        for t in (2., 2.02, 2.04):
            self.assertEqual(self.raw(c, t)['steering_raw'], -22)
            self.assertEqual(c.action, 'RIGHT')
        self.front(c, 2.1, marker)
        self.assertEqual(self.raw(c, 2.1)['steering_raw'], -22)
        self.front(c, 2.2, marker)
        cmd = self.raw(c, 2.2)
        self.assertEqual((cmd['speed_raw'], cmd['steering_raw']), (20, 0))
        self.assertEqual(c.action, 'STRAIGHT')
        origin = c.pose
        for t, distance in ((2.3, 1.249), (2.4, 1.25), (2.5, 1.25), (2.6, 1.25)):
            c.set_pose((origin[0]+distance, origin[1], origin[2]), t)
            self.front(c, t)
            c.observe_lane([(.35, 0.), (.55, 0.), (.75, 0.)], .9, t)
            self.raw(c, t)
            if t == 2.3:
                self.assertEqual(c.action, 'STRAIGHT')
        self.assertEqual(c.state, 'LANE')

    def test_right_blue_confirmation_resets_across_image_gap(self):
        c = self.core('RIGHT')
        self.right_lock(c)
        marker = [dict(kind='junction', x=.8, y=.1)]
        for t in (2., 2.1, 3., 3.1):
            self.front(c, t, marker)
            self.assertEqual(self.raw(c, t)['steering_raw'], -22)
            self.assertEqual(c.action, 'RIGHT')

    def test_both_turns_keep_estop_red_and_front_loss_stops(self):
        for action in ('LEFT', 'RIGHT'):
            c = self.core(action)
            self.front(c, 2.)
            c.estop = True
            self.assertEqual(self.raw(c, 2.)['speed_raw'], 0)
            c.estop = False
            c.red = True
            self.assertEqual(self.raw(c, 2.)['speed_raw'], 0)
            c.red = False
            self.assertEqual(self.raw(c, 4.)['speed_raw'], 0)


if __name__ == '__main__':
    unittest.main()
