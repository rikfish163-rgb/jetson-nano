"""Record the unchanged broad test selection against the final implementation."""
import json
import os
import sys
import unittest

ROOT = '/home/nano/robocup_ws'
BACKUP = os.path.join(ROOT, 'field_data/code_backups/20261005_sign_single_cache')
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(1, os.path.join(ROOT, 'src/robot/test'))
names = (
    'test_action_sign_lock', 'test_direction_single_frame',
    'test_straight_uturn_handoff', 'test_straight_queue_uturn_blue',
    'test_requested_maneuvers', 'test_startup_straight', 'test_direction_handoff',
    'test_startup_parking_gate', 'test_blue_stop_line', 'test_blue_edge_trigger',
    'test_mission_integration', 'test_mission_blue_sequence',
    'test_sign_callback_scheduling')
suite = unittest.defaultTestLoader.loadTestsFromNames(names)
with open(os.path.join(BACKUP, 'final_integration.txt'), 'w') as log:
    result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
data = dict(tests=result.testsRun,
            failures=[dict(test=t.id(), traceback=trace) for t, trace in result.failures],
            errors=[dict(test=t.id(), traceback=trace) for t, trace in result.errors])
with open(os.path.join(BACKUP, 'final_integration.json'), 'w') as out:
    json.dump(data, out, indent=2)
baseline = json.load(open(os.path.join(BACKUP, 'baseline_integration.json')))
previous = set(item['test'] for kind in ('failures', 'errors') for item in baseline[kind])
remaining = set(item['test'] for kind in ('failures', 'errors') for item in data[kind])
introduced = sorted(remaining-previous)
comparison = dict(baseline_tests=baseline['tests'], final_tests=data['tests'],
                  baseline_failures=len(baseline['failures']), baseline_errors=len(baseline['errors']),
                  final_failures=len(data['failures']), final_errors=len(data['errors']),
                  new_failing_tests=introduced)
with open(os.path.join(BACKUP, 'comparison.json'), 'w') as out:
    json.dump(comparison, out, indent=2)
print(json.dumps(comparison, sort_keys=True))
if introduced:
    sys.exit(1)
