#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""One-shot LEFT test starting with the front axle on the blue stripe.
No sign/blue approach is simulated. Reuses the production LEFT controller.
"""
from __future__ import division
import copy
from robot.master.controller import Controller
from robot.common.contracts import number
from robot.turn.planner import intersection_path


class LeftBlueTestController(Controller):
    def __init__(self, cfg, settings):
        cfg = copy.deepcopy(cfg)
        defaults = dict(speed_raw=20, entry_m=.12, start_delay_s=3.,
                        max_seconds=12., lock_seconds=0.)
        defaults.update(settings)
        limits = dict(speed_raw=(1,min(30,cfg['speed_raw_limit'])), entry_m=(0,.8),
                      start_delay_s=(0,30), max_seconds=(.5,30), lock_seconds=(0,20))
        for key, bounds in limits.items():
            value = number(defaults[key])
            if not bounds[0] <= value <= bounds[1]:
                raise ValueError('invalid left test '+key)
            defaults[key] = value
        if defaults['speed_raw'] != int(defaults['speed_raw']):
            raise ValueError('test speed_raw must be an integer')
        cfg.update(wait_green=False, parking_enabled=False, lidar_enabled=True,
                   timed_bypass_enabled=False, bypass_enabled=False,
                   left_turn_full_lock=True, left_turn_entry=defaults['entry_m'])
        cfg['speed_raw']['action'] = int(defaults['speed_raw'])
        Controller.__init__(self,cfg)
        self.test_settings = defaults
        self.test_ready_at = None
        self.test_started_at = None
        self.test_finished = False
        self.test_result = None
        self.state = 'LEFT_TEST_WAIT'

    def observe_sign(self,*args):
        pass  # Isolated test has no traffic-sign dispatch.

    def finish_test(self, reason):
        self.test_finished = True
        self.test_result = reason
        self.state = 'FINISHED'
        return self.stop(reason)

    def tick(self, now):
        if self.test_finished:
            return self.stop(self.test_result)
        if self.estop:
            return self.finish_test('left_test_estop')
        if self.test_started_at is None:
            if not self.scan_ready(now) or not self.lane_valid(now):
                self.test_ready_at = None
                return self.stop('left_test_wait_sensors')
            if self.test_ready_at is None:
                self.test_ready_at = now
            if now-self.test_ready_at < self.test_settings['start_delay_s']:
                return self.stop('left_test_countdown')
            self.test_started_at = now
            self.start_follow(intersection_path(self.pose,'LEFT',self.cfg),'LEFT',now)
        if now-self.test_started_at >= self.test_settings['max_seconds']:
            return self.finish_test('left_test_timeout')
        duration = self.test_settings['lock_seconds']
        lock = self.left_lock
        if duration and lock is not None and lock['phase'] == 'LOCK':
            if now-lock['lock_started'] >= duration:
                return self.finish_test('left_test_timed_complete')
        # Timed mode suppresses only the visual early exit, retaining all guards.
        old_frames = self.cfg['exit_frames']
        if duration:
            self.cfg['exit_frames'] = 1000000000
        try:
            command = Controller.tick(self,now)
        finally:
            self.cfg['exit_frames'] = old_frames
        if self.action != 'LEFT' and self.state == 'LANE':
            return self.finish_test('left_test_visual_complete')
        if self.state == 'FAULT' or command[0] == 0:
            return self.finish_test('left_test_stopped:'+self.reason)
        return command


def main():
    import rospy
    from robot.master import controller_node as adapter
    rospy.init_node('competition_controller')
    adapter.Controller = lambda cfg: LeftBlueTestController(cfg,rospy.get_param('~left_blue_test'))
    node = adapter.Node()
    rospy.loginfo('LEFT test: front axle on blue line; enable explicitly; one run then STOP')
    rospy.spin()


if __name__ == '__main__':
    main()
