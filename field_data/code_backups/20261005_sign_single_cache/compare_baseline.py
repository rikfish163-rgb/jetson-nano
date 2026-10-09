"""Replay saved production and test sources without changing the live workspace."""
import json
import os
import sys
import types
import unittest

ROOT = '/home/nano/robocup_ws'
BACKUP = os.path.join(ROOT, 'field_data/code_backups/20261005_sign_single_cache')
sys.path.insert(0, os.path.join(ROOT, 'src'))
sys.path.insert(1, os.path.join(ROOT, 'src/robot/test'))


def saved_module(name, relative_path):
    module = types.ModuleType(name)
    module.__file__ = os.path.join(ROOT, relative_path)
    sys.modules[name] = module
    parent, _, child = name.rpartition('.')
    if parent:
        package = __import__(parent, fromlist=[child])
        setattr(package, child, module)
    source = open(os.path.join(BACKUP, 'before', relative_path), 'rb').read()
    exec compile(source, module.__file__, 'exec') in module.__dict__


saved_module('robot.master.state_machine', 'src/robot/master/state_machine.py')
saved_module('robot.signs.decisions', 'src/robot/signs/decisions.py')
saved_module('robot.camera.observations', 'src/robot/camera/observations.py')
saved_tests = (
    'test_action_sign_lock', 'test_direction_single_frame',
    'test_straight_uturn_handoff', 'test_straight_queue_uturn_blue',
    'test_requested_maneuvers', 'test_startup_straight')
for name in saved_tests:
    saved_module(name, 'src/robot/test/'+name+'.py')
names = saved_tests + (
    'test_direction_handoff', 'test_startup_parking_gate', 'test_blue_stop_line',
    'test_blue_edge_trigger', 'test_mission_integration',
    'test_mission_blue_sequence', 'test_sign_callback_scheduling')
suite = unittest.defaultTestLoader.loadTestsFromNames(names)
with open(os.path.join(BACKUP, 'baseline_integration.txt'), 'w') as log:
    result = unittest.TextTestRunner(stream=log, verbosity=2).run(suite)
data = dict(tests=result.testsRun,
            failures=[dict(test=t.id(), traceback=trace) for t, trace in result.failures],
            errors=[dict(test=t.id(), traceback=trace) for t, trace in result.errors])
with open(os.path.join(BACKUP, 'baseline_integration.json'), 'w') as out:
    json.dump(data, out, indent=2)
print('Saved baseline: %d tests, %d failures, %d errors' %
      (result.testsRun, len(result.failures), len(result.errors)))
