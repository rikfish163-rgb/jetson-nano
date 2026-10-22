import os
import sys
import unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.master.diagnostics import graph_issues
from robot.master.diagnostics import sensor_issues


class DiagnosticTests(unittest.TestCase):
    def test_empty_graph_is_not_healthy(self):
        self.assertTrue(graph_issues({}, {}, {}, True))

    def test_duplicate_actuator_owner_is_reported_in_shadow(self):
        issues = graph_issues({'/ackermann_cmd':['a','b']}, {}, {}, False)
        self.assertIn('multiple control owners: /ackermann_cmd', issues)

    def test_publishers_do_not_prove_fresh_sensor_data(self):
        self.assertTrue(sensor_issues({}, {}))
        ages = dict(lane=.1, front_ground=.1, scan=.1)
        self.assertEqual(sensor_issues({'source_ages': ages}, {}), [])
        ages['lane'] = 10.0
        self.assertIn('sensor stale or missing: lane', sensor_issues({'source_ages': ages}, {}))

    def test_reverse_phase_requires_rear_stream(self):
        status = dict(state='UTURN', source_ages=dict(lane=.1, front_ground=.1, scan=.1))
        self.assertIn('sensor stale or missing: rear_ground', sensor_issues(status, {}))

    def test_scoped_lidar_exceptions_match_runtime_readiness(self):
        cfg = dict(lidar_enabled=True, straight_lidar_once=True,
                   parking_mode='forward_center', parking_slot='P4',
                   parking_entry_style='S', parking_lidar_enabled=False)
        status = dict(state='MANEUVER', action='STRAIGHT',
                      source_ages=dict(lane=.1, front_ground=.1))
        self.assertIn('sensor stale or missing: scan', sensor_issues(status, cfg))
        status['timed_bypass_completed'] = True
        self.assertEqual(sensor_issues(status, cfg), [])
        status.update(state='PARKING', action='PARKING')
        self.assertEqual(sensor_issues(status, cfg), [])
        cfg['parking_lidar_enabled'] = True
        self.assertIn('sensor stale or missing: scan', sensor_issues(status, cfg))
        # The device is still required by the launch graph for other categories.
        self.assertIn('missing publisher: /scan', graph_issues({}, {}, cfg, False))
