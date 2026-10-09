#!/usr/bin/env python2
"""No-node test gate. Failures are never filtered or converted into skips.

Run on Nano after sourcing ROS. Imported TestCases are counted once by test id.
Each package runs separately because legacy tests install fake ROS modules.
"""
from __future__ import print_function
import argparse
import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
SUITES = {'competition': 'src/robot/test',
          'main': 'src/ros/main/test', 'manual': 'src/ros/manual/test',
          'vision': 'src/ros/camera/test'}


def cases(suite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            for case in cases(item):
                yield case
        else:
            yield item


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', choices=sorted(SUITES))
    parser.add_argument('--report', required=True, help='JSON result file')
    args = parser.parse_args()
    discovered = unittest.defaultTestLoader.discover(os.path.join(ROOT, SUITES[args.suite]))
    unique, duplicates = {}, []
    for case in cases(discovered):
        name = case.id()
        if name in unique:
            duplicates.append(name)
        else:
            unique[name] = case
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.TestSuite(unique[name] for name in sorted(unique)))
    report = dict(suite=args.suite, python=sys.version, tests=result.testsRun,
                  failures=[dict(test=t.id(), traceback=trace) for t, trace in result.failures],
                  errors=[dict(test=t.id(), traceback=trace) for t, trace in result.errors],
                  skipped=[dict(test=t.id(), reason=why) for t, why in result.skipped],
                  duplicate_discovery_ids=duplicates, passed=result.wasSuccessful())
    with open(args.report, 'w') as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
    print('RELEASE_TEST_GATE', 'PASS' if result.wasSuccessful() else 'FAIL')
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    sys.exit(main())
