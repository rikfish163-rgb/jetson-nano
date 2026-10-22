#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Single central-control entry. Drivers and teleoperation run separately."""
import imp
import os
import rospy


def load_competition_config(default_path):
    """Keep launch overrides intact; direct startup publishes disabled heartbeat."""
    if not rospy.has_param('/competition/config'):
        import yaml
        path = rospy.get_param('~config_file', default_path)
        with open(path) as stream:
            cfg = yaml.safe_load(stream)
        if not isinstance(cfg, dict):
            raise ValueError('competition config must be a mapping')
        cfg.update(wait_green=True, sign_ttl=0)
        rospy.set_param('/competition/config', cfg)
    for name, value in (('~live', True), ('~enabled', False)):
        if not rospy.has_param(name):
            rospy.set_param(name, value)


def main():
    rospy.init_node('main_node')
    backend = rospy.get_param('~controller_backend', 'competition')
    if backend == 'legacy':
        import rospkg
        import sys
        sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('main'), 'scripts'))
        from legacy_controller import run
        run()
    elif backend == 'competition':
        import rospkg
        root = rospkg.RosPack().get_path('robocup_competition')
        load_competition_config(os.path.join(root, 'config', 'competition.yaml'))
        # Load the existing adapter in this process; never start a second main.
        adapter = imp.load_source('competition_main_adapter',
                                  os.path.join(root, 'master', 'controller_node.py'))
        adapter.Node()
        rospy.spin()
    else:
        raise ValueError('unknown controller_backend: %s' % backend)


if __name__ == '__main__':
    main()
