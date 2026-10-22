#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Team entrypoint: source ownership, config checks, tests and offline replay.

This tool never launches ROS nodes or publishes actuator commands.
"""
from __future__ import print_function
import argparse
import fnmatch
import json
import os
import subprocess
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PACKAGE = os.path.join(ROOT, 'src', 'robot')
CONFIG = os.path.join(PACKAGE, 'config')
sys.path.insert(0, os.path.dirname(PACKAGE))
from robot.common.config import read_mapping
from robot.common.config import load_config

SUITES = ('src/robot/test', 'src/ros/main/test',
          'src/ros/camera/test', 'src/ros/lane/test',
          'src/ros/manual/test', 'src/ros/lidar/test',
          'src/ros/signs/scripts', 'tools', 'tools/sign_retrain_20260907')
EXTENSIONS = ('.py', '.cpp', '.cc', '.c', '.h', '.hpp', '.launch', '.yaml',
              '.yml', '.xml', '.msg', '.srv', '.sh')


def catalog():
    return read_mapping(os.path.join(CONFIG, 'workspace_modules.yaml'))


def owner_of(path, spec):
    for row in spec['rules']:
        if fnmatch.fnmatchcase(path, row['pattern']):
            return row['owner']
    raise ValueError('source has no owner: '+path)


def source_files():
    for prefix in ('src', 'tools'):
        for folder, dirs, files in os.walk(os.path.join(ROOT, prefix)):
            dirs[:] = sorted(d for d in dirs if d not in ('.git', 'build', 'devel', '__pycache__'))
            for name in sorted(files):
                if name.endswith(EXTENSIONS) or name == 'CMakeLists.txt':
                    yield os.path.relpath(os.path.join(folder, name), ROOT)


def cases(suite):
    for case in suite:
        if isinstance(case, unittest.TestSuite):
            for nested in cases(case):
                yield nested
        else:
            yield case


def worker(owner, directory, report):
    spec = catalog()
    discovered = unittest.defaultTestLoader.discover(os.path.join(ROOT, directory))
    selected = {}
    for case in cases(discovered):
        test_module = case.id().split('.')[0]
        path = directory+'/'+test_module+'.py'
        # Import failures belong to this suite and must never disappear.
        failed_import = case.__class__.__name__ in ('ModuleImportFailure', '_FailedTest')
        if failed_import or owner_of(path, spec) == owner:
            selected[case.id()] = case
    result = unittest.TextTestRunner(verbosity=2).run(unittest.TestSuite(selected[k] for k in sorted(selected)))
    row = dict(module=owner, suite=directory, tests=result.testsRun,
               failures=[dict(test=t.id(), traceback=s) for t, s in result.failures],
               errors=[dict(test=t.id(), traceback=s) for t, s in result.errors],
               skipped=[dict(test=t.id(), reason=s) for t, s in result.skipped],
               passed=result.wasSuccessful())
    with open(report, 'w') as stream:
        json.dump(row, stream, indent=2)
    return 0 if result.wasSuccessful() else 1


def run_tests(owner, report_dir):
    if not os.path.isdir(report_dir):
        os.makedirs(report_dir)
    spec, failed, reports = catalog(), False, []
    for directory in SUITES:
        path = os.path.join(ROOT, directory)
        files = [directory+'/'+name for name in os.listdir(path)
                 if name.startswith('test_') and name.endswith('.py')]
        if not any(owner_of(name, spec) == owner for name in files):
            continue
        report = os.path.join(report_dir, directory.replace('/', '-')+'.json')
        command = [sys.executable, os.path.abspath(__file__), '_worker', owner,
                   '--suite', directory, '--report', report]
        environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
        failed = subprocess.call(command, env=environment) != 0 or failed
        with open(report) as stream:
            reports.append(json.load(stream))
    if not reports:
        raise ValueError('no tests assigned to module '+owner)
    summary = dict(module=owner, tests=sum(row['tests'] for row in reports),
                   passed=not failed, suites=reports)
    with open(os.path.join(report_dir, 'summary.json'), 'w') as stream:
        json.dump(summary, stream, indent=2)
    print('MODULE_TEST_GATE', owner, 'FAIL' if failed else 'PASS', summary['tests'])
    return 1 if failed else 0


def json_value(value):
    if isinstance(value, dict):
        return dict((k, json_value(v)) for k, v in value.items())
    if isinstance(value, (tuple, list)):
        return [json_value(v) for v in value]
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    try:
        if isinstance(value, unicode):
            return value
    except NameError:
        pass
    return dict(resource_type=type(value).__name__)


def replay(path):
    from robot.master.runtime import ModuleRuntime
    from robot.master.state_machine import initial_state
    from robot.lidar.scan import Scan
    from robot.common.contracts import validate_config
    with open(path) as stream:
        fixture = json.load(stream)
    cfg = load_config(CONFIG)
    validate_config(cfg)
    state = initial_state(cfg)
    # JSON state may contain observations and action progress, never executable resources.
    resources = ('cfg', 'executor', 'future', 'relative_future', 'parallel_future',
                 'relative_replan_future', 'follower', 'parking_entry', 'relative_uturn',
                 'parallel_parking', 'timed_uturn', 'scan')
    try:
        given = fixture.get('state', {})
        if any(key in resources for key in given):
            raise ValueError('use observations instead of replacing runtime resources')
        state.update(given)
        if 'scan' in fixture:
            scan = fixture['scan']
            state['scan'] = Scan(scan['ranges'], scan['angle_min'], scan['increment'],
                                 scan['range_min'], scan['range_max'], state['pose'],
                                 cfg['lidar'], scan['stamp'])
        runtime, rows = ModuleRuntime(), []
        if fixture['module'] == 'lidar':
            if state['scan'] is None:
                raise ValueError('lidar replay requires a scan')
            return dict(obstacle_frame='snapshot_world', cluster_frame='lidar',
                        sensor_pose=state['scan'].pose, valid_rays=state['scan'].valid_rays,
                        obstacles=state['scan'].obstacles,
                        clusters=state['scan'].cluster_diagnostics)
        if fixture['module'] == 'camera' and fixture.get('operation') == 'detect_image':
            import cv2
            from robot.camera.vision import GroundDetector
            frame = cv2.imread(fixture['image'])
            if frame is None:
                raise ValueError('image cannot be read')
            detector = GroundDetector(cfg)
            return json_value(detector.detect(detector.bev(frame)))
        steps = fixture.get('steps', [fixture])
        for step in steps:
            module = step.get('module', fixture['module'])
            result = runtime.execute(module, step['operation'], state, *step.get('args', []))
            state.update(result.updates)
            rows.append(dict(module=module, operation=step['operation'],
                             value=json_value(result.value), updates=json_value(result.updates)))
        return dict(steps=rows, state=state['state'], reason=state['reason'])
    finally:
        state['executor'].shutdown(wait=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('list', 'check', 'test', 'replay', '_worker'))
    parser.add_argument('target', nargs='?')
    parser.add_argument('--report-dir', default='/tmp/nano-module-tests')
    parser.add_argument('--suite', choices=SUITES)
    parser.add_argument('--report')
    args = parser.parse_args(argv)
    spec = catalog()
    if args.command in ('test', '_worker', 'list') and args.target and args.target not in spec['modules']:
        parser.error('unknown module: '+args.target)
    if args.command == 'replay':
        if not args.target:
            parser.error('replay requires a JSON fixture')
        print(json.dumps(replay(args.target), indent=2, allow_nan=False))
        return 0
    if args.command == '_worker':
        return worker(args.target, args.suite, args.report)
    if args.command == 'test':
        if not args.target:
            parser.error('test requires a module name')
        return run_tests(args.target, args.report_dir)
    files = list(source_files())
    assignments = dict((path, owner_of(path, spec)) for path in files)
    if args.command == 'check':
        from robot.common.contracts import validate_config
        validate_config(load_config(CONFIG))
        print('PASS %d source/config files assigned; module overrides valid' % len(files))
        return 0
    if args.target:
        print(json.dumps(dict(module=args.target, **spec['modules'][args.target]), ensure_ascii=False, indent=2))
        for path in sorted(path for path in assignments if assignments[path] == args.target):
            print(path)
    else:
        for name in sorted(spec['modules']):
            print(name, sum(v == name for v in assignments.values()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
