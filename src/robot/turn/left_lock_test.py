#!/usr/bin/env python
"""Use the existing ROS adapter with an isolated, one-shot full-left test core."""
import rospy
from robot.master import controller_node as adapter
from robot.turn.left_lock import LeftLockController


def make_core(cfg):
    return LeftLockController(cfg,rospy.get_param('~left_lock_speed_raw',18),
                              rospy.get_param('~left_lock_max_seconds',12),
                              rospy.get_param('~left_lock_line_frames',3))


if __name__ == '__main__':
    rospy.init_node('competition_controller')
    adapter.Controller = make_core  # process-local; the normal entry is unchanged
    adapter.Node()
    rospy.spin()
