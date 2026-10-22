#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Single central-control entry. Drivers and teleoperation run separately."""
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
    import rospkg
    root = rospkg.RosPack().get_path('robocup_competition')
    load_competition_config(os.path.join(root, 'config', 'competition.yaml'))
    from robot.master.controller_node import Node
    Node()
    rospy.spin()


if __name__ == '__main__':
    main()
