"""Sensor scheduling and real LaserScan conversion; no ROS master needed."""
import math
import json
import unittest
from collections import deque
import rospy
from sensor_msgs.msg import LaserScan
from test_timed_parking import config
from robot.lidar.side_bay_node import selection_request,scan_observation
from robot.lidar.side_bays import measure
from robot.parking.timed_vision_node import side_processing_active,side_run
from robot.parking.target_line import FramePoses
from robot.master.controller_node import Node as ControllerNode
from robot.master.telemetry import parking_schedule_hint


class Box(object):
    def __init__(self,**values):self.__dict__.update(values)


class SideBaySensorTests(unittest.TestCase):
    def scan(self):
        msg=LaserScan();msg.header.stamp=rospy.Time.from_sec(2.)
        msg.header.frame_id='laser_link'
        msg.angle_min=-math.pi;msg.angle_increment=2*math.pi/1440
        msg.range_min=.01;msg.range_max=12.
        msg.ranges=[float('inf')]*1440
        for i in range(1440):
            angle=msg.angle_min+i*msg.angle_increment
            if -.9<angle<-.2:msg.ranges[i]=.4/math.cos(angle)
        return msg

    def test_scan_uses_exact_shared_geometry(self):
        msg=self.scan();cfg=config()
        data=scan_observation(msg,cfg,1.,2.1)
        points=[]
        for i,r in enumerate(msg.ranges):
            if math.isinf(r):continue
            a=msg.angle_min+i*msg.angle_increment
            points.append((r*math.cos(a),r*math.sin(a)))
        expected=measure(points)
        self.assertTrue(data['valid'])
        self.assertEqual(data['boxes'],expected['boxes'])
        self.assertEqual(data['line'],expected['line'])
        self.assertEqual(data['counts'],{'1':0,'2':0})

    def test_invalid_empty_wrong_frame_and_pre_stop_scans_cannot_be_valid(self):
        cfg=config()
        for kind in ('empty','frame','old','pre_stop','increment'):
            msg=self.scan();request,now=1.,2.1
            if kind=='empty':msg.ranges=[float('inf')]*1440
            if kind=='frame':msg.header.frame_id='base_link'
            if kind=='old':now=10.
            if kind=='pre_stop':request=3.
            if kind=='increment':msg.angle_increment=0.
            self.assertFalse(scan_observation(msg,cfg,request,now)['valid'])

    def test_processing_only_enabled_while_stationary_selection(self):
        hint=dict(stamp=2.,enabled=True,state='TIMED_PARKING',parking_timed=dict(
                  phase='SELECT_SLOT',select_by_lidar=True,selection=dict(started=1.5)))
        self.assertEqual(selection_request(hint,2.1),1.5)
        for phase in ('APPROACH','HOLD','REVERSE','FINISHED'):
            hint['parking_timed']['phase']=phase
            self.assertIsNone(selection_request(hint,2.1))
        hint['parking_timed']['phase']='SELECT_SLOT'
        hint['enabled']=False
        self.assertIsNone(selection_request(hint,2.1))
        hint['enabled']=True
        self.assertIsNone(selection_request(hint,4.))

    def test_side_vision_sleeps_without_confirmed_lane_p(self):
        self.assertFalse(side_processing_active(None,2.))
        hint=dict(stamp=2.,state='LANE',pending=None,parking_side_pending=False)
        self.assertFalse(side_processing_active(hint,2.1))
        hint.update(state='MANEUVER',pending='PARKING')
        self.assertFalse(side_processing_active(hint,2.1))
        hint.update(state='LANE',parking_side_pending=True)
        self.assertTrue(side_processing_active(hint,2.1))
        hint.update(state='TIMED_PARKING',pending=None,parking_side_pending=False)
        self.assertTrue(side_processing_active(hint,2.1))
        self.assertFalse(side_processing_active(hint,4.))

    def test_busy_full_diagnostics_cannot_block_white_line_activation(self):
        core=Box(state='TIMED_PARKING',action='PARKING',pending=None,
                 timed_parking=Box(phase='APPROACH',select_by_lidar=True,selection=None))
        node=ControllerNode.__new__(ControllerNode)
        node.parking_hint_snapshot=parking_schedule_hint(core,10.,True)
        received=[]
        node.parking_hint_pub=Box(publish=lambda m:received.append(json.loads(m.data)))
        node.control_events=deque()
        node.lock=Box(acquire=lambda blocking:False)
        node.telemetry(None)
        self.assertEqual(len(received),1)
        self.assertTrue(side_processing_active(received[0],10.1))
        self.assertIsNone(selection_request(received[0],10.1))

    def test_light_hint_carries_bounded_image_time_pose_samples_and_task_identity(self):
        core=Box(state='TIMED_PARKING',action='PARKING',pending=None,
                 pose=(2.,0.,.1),cfg={'pose_mode':'command_model'},
                 timed_parking=Box(phase='APPROACH',select_by_lidar=True,
                                  selection=None,triggered_at=9.))
        history=[(5.+i*.05,(i*.01,0.,0.)) for i in range(100)]
        hint=parking_schedule_hint(core,10.,True,history)
        self.assertLessEqual(len(hint['pose_samples']),31)
        self.assertEqual(side_run(hint),9.)
        poses=FramePoses();poses.update(hint['pose_samples'])
        self.assertIsNotNone(poses.at(9.4))
        for a,b in zip(poses.at(10.),core.pose):self.assertAlmostEqual(a,b)
        core.timed_parking=None
        self.assertNotIn('pose_samples',parking_schedule_hint(core,10.,True,history))
        self.assertIsNone(side_run(parking_schedule_hint(core,10.,True,history)))

    def test_same_control_state_refreshes_hint_and_only_stop_phase_enables_lidar(self):
        node=ControllerNode.__new__(ControllerNode)
        task=Box(phase='APPROACH',select_by_lidar=True,selection=None)
        node.core=Box(state='TIMED_PARKING',reason='AUTO_WAIT_CAMERA',action='PARKING',
                      pending=None,timed_parking=task,timed_bypass=None)
        node.last_control_event=(True,'TIMED_PARKING','AUTO_WAIT_CAMERA',None,True)
        node.queue_control_event(10.,True,(0,0.))
        self.assertEqual(node.parking_hint_snapshot['stamp'],10.)
        self.assertIsNone(selection_request(node.parking_hint_snapshot,10.1))
        task.phase='SELECT_SLOT';task.selection=Box(started=10.1)
        node.queue_control_event(10.2,True,(0,0.))
        self.assertEqual(node.parking_hint_snapshot['stamp'],10.2)
        self.assertEqual(selection_request(node.parking_hint_snapshot,10.3),10.1)


if __name__=='__main__':unittest.main()
