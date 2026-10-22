"""Fresh visual geometry, command units, and bounded loss (no hardware)."""
import math
import os
import unittest

from robot.common.config import load_config
from robot.common.contracts import encode_command, model_to_command_steering
from robot.master.controller import Controller


class VisualPathFeedbackTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(os.path.join(os.path.dirname(__file__), '..', 'config'))
        cfg.update(lidar_enabled=False, wait_green=False, blue_default_straight=False,
                   lane_path_feedback=True, lookahead=.55, steering_command_scale_rad=.1)
        cfg['speed_raw']['lane'] = 24
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def observe(self, points, t=1., curve=None, boundaries=None):
        self.c.observe_lane(points, .9 if points else 0., t, boundaries, curve)
        return self.c.lane_command(t)

    def test_physical_angle_uses_same_mapping_as_planned_paths(self):
        speed, steer = self.observe([(.3, .03), (.8, .03)])
        d = self.c.lane_feedback
        x, y = d['target_m']
        expected = math.atan2(2*self.c.cfg['wheelbase']*y, x*x+y*y)
        self.assertAlmostEqual(steer, model_to_command_steering(expected, self.c.cfg))
        self.assertAlmostEqual(d['command_steer'], steer)

    def test_fresh_path_can_reduce_turn_without_steering_memory_override(self):
        curve = dict(direction='RIGHT', entry_side='LEFT', entry_stamp=1.)
        self.observe([(.3, -.25), (.7, -.25)], curve=curve)
        speed, steer = self.observe([(.3, -.01), (.7, -.01)], 1.1, curve)
        self.assertGreater(steer, -.02)
        self.assertEqual(self.c.lane_source, 'visual_path')

    def test_unknown_label_does_not_discard_fresh_geometry(self):
        self.observe([(.3, -.2), (.8, -.2)], curve=dict(
            direction='RIGHT', entry_side='LEFT', entry_stamp=1.))
        self.assertEqual(self.observe([(.3, 0), (.8, 0)], 1.1)[1], 0.)

    def test_curve_label_does_not_override_opposite_valid_path(self):
        command = self.observe([(.3, .02), (.8, .02)], curve=dict(
            direction='RIGHT', entry_side='LEFT', entry_stamp=1.))
        self.assertGreater(command[1], 0.)

    def test_interpolated_target_does_not_jump_between_sparse_points(self):
        from robot.lane.path_feedback import circle_target
        path = [(.3, -.03), (.8, -.08)]
        a, b = circle_target(path, .549), circle_target(path, .551)
        self.assertLess(math.hypot(a[0]-b[0], a[1]-b[1]), .003)
        self.assertAlmostEqual(math.hypot(*a), .549)

    def test_mirrored_curves_produce_mirrored_commands_and_slowdown(self):
        path = [(.2+i*.05, -.8*(.2+i*.05)**2) for i in range(12)]
        a = self.observe(path)
        b = self.observe([(x, -y) for x, y in path], 1.1)
        self.assertAlmostEqual(a[1], -b[1])
        self.assertEqual(a[0], b[0])
        self.assertLess(a[0], 24)

    def test_full_lock_is_available_when_path_requires_it(self):
        command = self.observe([(.2, -.3), (.4, -.45), (.6, -.5)])
        self.assertEqual(encode_command(command[0], command[1], self.c.cfg, 0)['steering_raw'], -22)

    def test_hold_expires_despite_new_empty_frames_and_recovers(self):
        command = self.observe([(.3, -.1), (.8, -.1)])
        self.assertEqual(self.observe([], 1.1)[1], command[1])
        self.assertEqual(self.observe([], 1.4), (0, 0.))
        self.assertEqual(self.observe([], 1.5), (0, 0.))
        self.assertEqual(self.observe([(.3, 0), (.8, 0)], 1.6)[1], 0.)

    def test_stale_stream_never_keeps_driving(self):
        self.observe([(.3, -.1), (.8, -.1)])
        self.assertEqual(self.c.lane_command(2.), (0, 0.))

    def test_target_respects_observed_boundaries_and_body_margin(self):
        edges = {'LEFT': [(.2, .3), (1., .3)], 'RIGHT': [(.2, -.3), (1., -.3)]}
        self.observe([(.3, -.29), (.8, -.29)], boundaries=edges)
        d = self.c.lane_feedback
        self.assertGreaterEqual(d['target_m'][1], -.3+self.c.cfg['body_width']/2)
        self.assertGreater(d['corridor_adjusted_points'], 0)

    def test_crossed_boundary_pair_is_not_a_drivable_corridor(self):
        edges = {'LEFT': [(.2, -.3), (1., -.3)], 'RIGHT': [(.2, .3), (1., .3)]}
        self.assertEqual(self.observe([(.3, 0), (.8, 0)], boundaries=edges), (0, 0.))

    def test_short_path_is_not_extrapolated_beyond_observations(self):
        self.observe([(.1, -.01), (.3, -.03)])
        self.assertEqual(self.c.lane_feedback['target_m'], (.3, -.03))

    def test_duplicate_frame_does_not_renew_loss_budget(self):
        self.observe([(.3, -.1), (.8, -.1)])
        self.c.lane_command(1.3)
        self.assertEqual(self.observe([], 1.4), (0, 0.))

    def test_loss_distance_budget_is_independent_of_time_budget(self):
        self.observe([(.3, -.1), (.8, -.1)])
        self.c.pose = (.13, 0., 0.)
        self.assertEqual(self.observe([], 1.1), (0, 0.))

    def test_selected_maneuver_keeps_its_existing_controller(self):
        self.c.action = 'LEFT'
        command = self.observe([(.3, .1), (.8, .1)])
        self.assertIsNone(self.c.lane_feedback)
        self.assertGreater(command[1], 0)

    def test_ideal_bicycle_tracks_both_turns_with_offset_and_heading_error(self):
        from robot.common.geometry import bicycle, local
        from robot.common.contracts import command_to_model_steering
        from robot.lane.path_feedback import track_path
        # Model-only acceptance, not evidence of real actuator calibration.
        for side in (-1, 1):
            radius = .65
            path = [(radius*math.sin(i*.005), side*radius*(1-math.cos(i*.005)))
                    for i in range(750)]
            pose = (0., -side*.06, -side*.05)
            max_error = 0.
            for step in range(160):
                nearest = min(range(len(path)), key=lambda i:
                    math.hypot(path[i][0]-pose[0], path[i][1]-pose[1]))
                observed, previous_x = [], -1.
                for p in path[nearest:]:
                    p = local(pose, p)
                    if p[0] < previous_x:
                        break  # the camera cannot follow the back of the arc
                    previous_x = p[0]
                    if .30 <= p[0] <= .95:
                        observed.append(p)
                result, reason = track_path(observed, {}, self.c.cfg)
                self.assertIsNone(reason)
                angle = command_to_model_steering(result['command_steer'], self.c.cfg)
                pose = bicycle(pose, .008, angle, self.c.cfg['wheelbase'])
                error = abs(math.hypot(pose[0], pose[1]-side*radius)-radius)
                max_error = max(max_error, error)
            self.assertLess(max_error, .08)
            self.assertLess(error, .025)

    def test_moving_pose_transforms_target_into_current_vehicle_frame(self):
        self.observe([(.3, -.03), (.8, -.08)])
        self.c.pose = (.05, 0., 0.)
        self.c.lane_command(1.1)
        x, y = self.c.lane_feedback['path_m'][0]
        self.assertAlmostEqual(x, .25)
        self.assertAlmostEqual(y, -.03)

    def test_status_contains_the_exact_used_path_and_target(self):
        from robot.master.telemetry import controller_status
        command = self.observe([(.3, -.03), (.8, -.08)])
        status = controller_status(self.c, self.c.cfg, 1., False, True, command, 1)
        self.assertEqual(status['lane_feedback'], self.c.lane_feedback)
        self.assertEqual(status['lane_feedback']['command_steer'], command[1])

    def test_resume_after_maneuver_does_not_reuse_old_visual_hold(self):
        self.observe([(.3, -.03), (.8, -.08)])
        self.c.action = 'LEFT'
        self.c.resume_lane()
        self.assertIsNone(self.c.lane_feedback)
        self.assertEqual(self.observe([], 1.1), (0, 0.))


if __name__ == '__main__':
    unittest.main()
