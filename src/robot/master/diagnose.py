#!/usr/bin/env python
"""Read-only graph and status check; never publishes commands or changes params."""
from __future__ import print_function
import argparse
import json
import sys
import rosgraph
import rospy
from std_msgs.msg import String
from robot.master.diagnostics import graph_issues
from robot.master.diagnostics import sensor_issues


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live', action='store_true', help='require the real control and actuator chain')
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('competition_readonly_check', anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    publishers, _, _ = master.getSystemState()
    cfg = rospy.get_param('/competition/config', {})
    issues = graph_issues(dict(publishers), dict(master.getTopicTypes()), cfg, args.live)
    try:
        message = rospy.wait_for_message('/competition/status', String, timeout=2)
        status = json.loads(message.data)
        if not 0 <= rospy.Time.now().to_sec()-status['stamp'] <= 1:
            issues.append('controller status stale')
        if status.get('state') == 'FAULT' or status.get('estop'):
            issues.append('controller fault or estop: '+str(status.get('reason')))
        if status.get('live') != args.live:
            issues.append('requested mode differs from running controller')
        issues.extend(sensor_issues(status, cfg))
        print(json.dumps(status, indent=2, sort_keys=True))
    except (rospy.ROSException, ValueError, TypeError, KeyError):
        issues.append('controller status unavailable or invalid')
    for issue in issues:
        print('FAIL:', issue)
    print('READINESS', 'FAIL' if issues else 'PASS; physical motion not verified')
    return 1 if issues else 0


if __name__ == '__main__':
    sys.exit(main())
