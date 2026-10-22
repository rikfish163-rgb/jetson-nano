#!/usr/bin/env python2
"""Small deterministic tests for the offline competition simulator."""
from __future__ import division

import os
import sys
import unittest
import math
import yaml


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, 'tools')
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)
import competition_sim as sim


class CompetitionSimulationTests(unittest.TestCase):

    def obstacle_profile(self):
        with open(os.path.join(ROOT, 'src/robot/obstacle/config.yaml')) as stream:
            cfg = yaml.safe_load(stream)
        return dict((key, cfg['timed_bypass_' + key]) for key in
                    ('trigger_distance_m', 'settle_s', 'left_s', 'right_s'))

    def test_metric_and_configured_model_provenance_are_separate(self):
        course = sim.load_course()
        measured = course['provenance']['measured']
        configured = course['provenance']['configured_model']
        self.assertEqual(measured['arena_width_m'], 6.0)
        self.assertEqual(measured['lane_width_m'], .60)
        self.assertEqual(measured['turn_radii_m'], [.60, 1.20, 1.80])
        self.assertEqual(measured['slots']['P1']['length_m'], .70)
        self.assertEqual(measured['slots']['P4']['width_m'], .38)
        self.assertEqual(configured['wheelbase_m'], .26)
        self.assertEqual(configured['max_steer_rad'], .46275)
        self.assertNotIn('wheelbase_m', measured)
        self.assertIn('road_geometry', measured)
        self.assertNotIn('outer_corner_radius_m',
                         course['provenance']['inferred'].get('road_geometry', {}))
        self.assertNotIn('central_openings', measured['road_geometry'])

    def test_launch_startup_and_route_parking_profiles(self):
        cfg = sim.load_sim_config()
        self.assertTrue(cfg['wait_green'])
        self.assertEqual(cfg['sign_ttl'], 0)
        self.assertEqual(cfg['lane_hz'], 12)
        self.assertEqual(cfg['lane_window_height'], 40)
        self.assertEqual(cfg['lane_min_span'], .15)
        self.assertTrue(cfg['lane_curvature_preview'])
        self.assertEqual(cfg['lane_curve_speed_raw'], 12)
        for key, value in self.obstacle_profile().items():
            self.assertAlmostEqual(cfg['timed_bypass_' + key], value)
        self.assertAlmostEqual(cfg['parallel_parking_turn_radius_m'], .65)
        perpendicular = sim.CompetitionSimulation('image1_left', cfg=cfg)
        parallel = sim.CompetitionSimulation('image2', cfg=cfg)
        try:
            self.assertEqual(perpendicular.cfg['parking_mode'], 'forward_plan')
            self.assertEqual(parallel.cfg['parking_mode'], 'forward_plan')
            self.assertEqual(parallel.cfg['parking_slot'], 'P1')
        finally:
            perpendicular.close()
            parallel.close()

    def test_screenshot_route_topology_and_terminal_bay(self):
        course = sim.load_course()
        left = course['routes']['image1_left']
        right = course['routes']['image1_right']
        third = course['routes']['image2']
        self.assertEqual(left['start'], [-2.10, -1.00, 1.5707963267948966])
        self.assertEqual(right['start'], [-2.10, -1.00, 1.5707963267948966])
        self.assertEqual(third['start'], [-0.80, -2.70, 0.0])
        self.assertEqual(third['events'][0]['id'], 'A')
        self.assertEqual(third['events'][0]['action'], 'STRAIGHT')
        self.assertEqual(third['events'][-1]['slot'], 'P1')
        self.assertEqual(third['events'][-1]['target'], [.70, 1.62, 0.0])
        self.assertEqual(left['events'][-1]['target'], [.20, .83, 1.5707963267948966])
        self.assertEqual(right['events'][-1]['target'], [-.20, .83, 1.5707963267948966])
        bays = course['provenance']['inferred']['road_geometry']['parking_bay_regions']
        self.assertEqual(bays['P1'], {'xmin': .35, 'xmax': 1.05,
                                      'ymin': 1.44, 'ymax': 1.80})
        self.assertEqual(bays['P4'], {'xmin': .01, 'xmax': .39,
                                      'ymin': .605, 'ymax': 1.055})

    def test_rounded_road_rejects_square_corner(self):
        course = sim.load_course()
        cfg = sim.load_sim_config()
        runner = sim.CompetitionSimulation('image1_left', course=course, cfg=cfg)
        try:
            geometry = sim._road_geometry(course)
            self.assertFalse(runner._inside_outer((-2.7, -2.7), 3.0, geometry))
            self.assertTrue(runner._inside_outer((-2.7, -1.2), 3.0, geometry))
        finally:
            runner.close()

    def test_known_cylinder_is_ray_cast_into_scan(self):
        course = sim.load_course()
        course['obstacles'] = [dict(id='test-cylinder', center=[-1.5, -1.0],
                                    radius_m=.10)]
        runner = sim.CompetitionSimulation('image1_left', course=course)
        try:
            ranges = runner._scan_ranges()
            self.assertLess(min(ranges), .51)
            self.assertGreaterEqual(min(ranges), .05)
        finally:
            runner.close()

    def test_lane_observation_does_not_fabricate_after_route_end(self):
        runner = sim.CompetitionSimulation('image1_left')
        try:
            runner.nearest_index = len(runner.route.path) - 1
            runner.controller.set_pose(runner.route.path[-1][:3], 1.0)
            points, confidence = runner._lane_observation()
            self.assertEqual(points, [])
            self.assertEqual(confidence, 0.0)
        finally:
            runner.close()

    def test_parking_ground_truth_slots_and_lines_use_measured_sizes(self):
        for route_name, slot, expected in (
                ('image1_left', 'P4', (.45, .38)),
                ('image1_right', 'P5', (.45, .38)),
                ('image2', 'P1', (.70, .36))):
            runner = sim.CompetitionSimulation(route_name)
            try:
                event = runner._parking_event()
                rows = runner._parking_slot_observations(event)
                target = [row for row in rows if row['id'] == slot][0]
                target_world = sim.world(runner.controller.pose,
                                         (target['x'], target['y']))
                target_pose = event['target_pose']
                self.assertAlmostEqual(target_world[0], target_pose[0], places=8)
                self.assertAlmostEqual(target_world[1], target_pose[1], places=8)
                self.assertEqual((target['length'], target['width']), expected)
                lengths = [math.hypot(b[0] - a[0], b[1] - a[1])
                           for a, b in runner._parking_lines()]
                self.assertAlmostEqual(lengths[0], expected[0], places=8)
                self.assertAlmostEqual(lengths[1], expected[0], places=8)
                self.assertAlmostEqual(lengths[2], expected[1], places=8)
            finally:
                runner.close()

    def test_controller_tick_uses_bicycle_truth_and_localizes_timeout(self):
        runner = sim.CompetitionSimulation('image1_left')
        result = runner.run(max_time=1.0)
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(result['failure']['reason'], 'simulation_timeout')
        self.assertEqual(len(result['trajectory']), 20)
        first = result['trajectory'][0]
        self.assertEqual(first['command_raw'], 20.0)
        self.assertAlmostEqual(first['pose'][1], -.992, places=8)
        self.assertEqual(first['physical_steer'], 0.0)
        self.assertIn('route_progress_m', result['failure'])
        self.assertIn('pose', result['failure'])
        self.assertEqual(result['metric_provenance']['configured_model']['wheelbase_m'], .26)
        self.assertEqual(result['model']['forward_command_raw'], 20)
        self.assertAlmostEqual(result['model']['raw_forward_mps'], .16)
        self.assertTrue(result['source_hashes']['stable'])
        self.assertFalse(result['sensor_provenance']['sensor_cadence_emulated'])
        self.assertIn('not full sensor-link equivalence',
                      result['sensor_provenance']['cadence'])
        self.assertEqual(result['launch_profile']['ground_hz'], 8.0)
        self.assertEqual(result['launch_profile']['obstacle_timed_bypass'],
                         self.obstacle_profile())
        self.assertEqual(result['launch_profile']['parallel_parking'],
                         {'turn_radius_m': .65, 'confirm_frames': 3,
                          'slot_size_tolerance_m': .05})
        for name in ('lane_controller', 'obstacle_controller', 'geometry',
                     'master_controller', 'turn_controller', 'course',
                     'simulator', 'parking_scene', 'parallel_parking_controller',
                     'parallel_parking_planner', 'parallel_parking_config',
                     'parking_planner', 'camera_observations', 'config_loader'):
            self.assertIn(name, result['source_hashes']['before'])

    def test_short_run_is_repeatable(self):
        a = sim.CompetitionSimulation('image1_right').run(max_time=.75)
        b = sim.CompetitionSimulation('image1_right').run(max_time=.75)
        self.assertEqual(a['status'], b['status'])
        self.assertEqual(a['failure'], b['failure'])
        self.assertEqual(a['trajectory'], b['trajectory'])


if __name__ == '__main__':
    unittest.main()
