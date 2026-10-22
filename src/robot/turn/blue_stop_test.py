#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Image-row trigger + fixed-time straight motion; one shot, no ROS in core."""
from __future__ import division
import copy
import math
from robot.common.contracts import number
from robot.master.controller import Controller


from robot.turn.blue_timed import validate_settings, row_candidate, advance, approach_command


class BlueStopTest(Controller):
    def __init__(self,cfg,settings):
        cfg=copy.deepcopy(cfg)
        self_settings=validate_settings(settings,cfg)
        cfg.update(wait_green=False,parking_enabled=False,lidar_enabled=True,
                   timed_bypass_enabled=False,bypass_enabled=False)
        Controller.__init__(self,cfg)
        self.test_settings=self_settings
        self.state='BLUE_TEST_WAIT'
        self.test_camera_stamp=-1.
        self.test_lines=[]
        self.test_processed_stamp=-1.
        self.test_confirmations=0
        self.test_previous_row=None
        self.test_start=None
        self.test_trigger=None
        self.test_last=None
        self.test_done=False
        self.test_result=None
        self.test_debug={}
        self.test_sequence={}

    def observe_sign(self,*args):pass

    def observe_ground(self,data,stamp):
        if data.get('source')!='front' or data.get('part','all') not in ('all','markers'):
            return
        if stamp<=self.test_camera_stamp:return
        self.test_camera_stamp=stamp
        self.test_lines=list(data.get('blue_lines',[]))

    def finish_test(self,reason):
        self.test_done=True
        self.test_result=reason
        self.state='FINISHED'
        return self.stop(reason)

    def row_candidate(self):
        return row_candidate(self.test_lines,self.cfg)

    def tick(self,now):
        if self.test_done:return self.stop(self.test_result)
        if self.estop:return self.finish_test('blue_test_estop')
        ready=(self.scan_ready(now) and 0<=now-self.test_camera_stamp<=self.test_settings['camera_timeout_s'])
        if not ready:
            if self.test_start is not None:return self.finish_test('blue_test_sensor_lost')
            return self.stop('blue_test_wait_camera_lidar')
        p=self.test_settings
        result=advance(self.test_sequence,now,self.test_camera_stamp,self.test_lines,self.cfg,p)
        self.test_start=self.test_sequence.get('start')
        self.test_trigger=self.test_sequence.get('trigger')
        self.test_confirmations=self.test_sequence.get('confirmations',0)
        self.test_debug=self.test_sequence.get('debug',{})
        if result=='done':return self.finish_test('blue_test_complete')
        if result not in ('align','approach','timed','recheck'):return self.finish_test('blue_test_'+result)
        self.state=dict(align='BLUE_TEST_ALIGN',approach='BLUE_TEST_APPROACH',
                        timed='BLUE_TEST_TIMED_FORWARD',recheck='BLUE_TEST_RECHECK')[result]
        command=approach_command(self.test_sequence,result,self.cfg,p)
        result=self.checked_command(command,now,False)
        if result!=command:return self.finish_test('blue_test_guard:'+self.reason)
        self.reason=self.state.lower()
        return result


def main():
    import rospy
    from robot.master import controller_node as adapter
    rospy.init_node('competition_controller')
    adapter.Controller=lambda cfg:BlueStopTest(cfg,rospy.get_param('~blue_stop_test'))
    node=adapter.Node()
    def report(event):
        with node.lock:
            c=node.core
            state,reason,debug=c.state,c.reason,dict(c.test_debug)
        rospy.loginfo('blue_stop_test state=%s reason=%s image=%s',state,reason,debug)
    timer=rospy.Timer(rospy.Duration(.5),report)
    rospy.spin()


if __name__=='__main__':main()
