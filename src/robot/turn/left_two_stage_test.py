#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""One-shot raw 30/0 for 1.5 s, then raw 30/+22 for 2.5 s.
Start with the front axle on the blue stripe. No sign or lane dispatch.
"""
import copy
from robot.master.controller import Controller


class TwoStageLeft(Controller):
    def __init__(self, cfg):
        cfg = copy.deepcopy(cfg)
        if cfg['speed_raw_limit'] < 30 or cfg['steering_raw_limit'] != 22:
            raise ValueError('test requires speed limit >=30 and steering limit 22')
        cfg.update(wait_green=False, parking_enabled=False, lidar_enabled=True,
                   timed_bypass_enabled=False, bypass_enabled=False)
        Controller.__init__(self,cfg)
        self.state = 'LEFT_TEST_WAIT'
        self.test_start = None
        self.test_last = None
        self.test_stage = 0
        self.test_done = False
        self.test_reason = 'left_test_wait_lidar'

    def observe_sign(self,*args):
        pass

    def finish(self, reason):
        self.test_done = True
        self.test_reason = reason
        self.state = 'FINISHED'
        return self.stop(reason)

    def tick(self, now):
        if self.test_done:
            return self.stop(self.test_reason)
        if self.estop:
            return self.finish('left_test_estop')
        if not self.scan_ready(now):
            if self.test_start is not None:
                return self.finish('left_test_lidar_lost')
            return self.stop('left_test_wait_lidar')
        if self.test_last is not None and not 0 <= now-self.test_last <= .25:
            return self.finish('left_test_control_gap')
        self.test_last = now
        if self.test_start is None:
            self.test_start = now
            self.test_stage = 1
        elapsed = now-self.test_start
        if self.test_stage == 1 and elapsed >= 2.2:
            self.test_stage = 2
            self.test_start = now
            elapsed = 0.
        if self.test_stage == 2 and elapsed >= 3.6:
            return self.finish('left_test_complete')
        self.state = 'LEFT_TEST_STAGE_%d' % self.test_stage
        # Encode exact protocol values even with non-default direction signs.
        scale = self.cfg.get('steering_command_scale_rad',self.cfg['max_steer'])
        steer = 0. if self.test_stage == 1 else scale/self.cfg['steering_sign']
        command = (30./self.cfg['speed_sign'],steer)
        result = self.checked_command(command,now,False)
        if result != command:
            return self.finish('left_test_guard:'+self.reason)
        self.reason = self.state.lower()
        return result


def main():
    import rospy
    from robot.master import controller_node as adapter
    rospy.init_node('competition_controller')
    adapter.Controller = TwoStageLeft
    node = adapter.Node()
    rospy.loginfo('One-shot LEFT: raw 30/0 1.5s -> raw 30/+22 2.5s -> STOP; enable explicitly')
    rospy.spin()


if __name__ == '__main__':
    main()
