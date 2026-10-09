"""Saved P1/P2/P3 production dispatch, timing and stop interlocks; no actuators."""
import copy
import json
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command,validate_config
from robot.master.controller import Controller
from robot.master.telemetry import controller_status
from robot.parking.timed_core import TimedParking,validate_scene
from robot.parking.timed_vision import PairBuffer,both_boundary_curves

ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class Follower(object):
    def observe(self,data,now): pass
    def command(self,now): return (30,7)

def config(slot='P3'):
    cfg=load_config(os.path.join(ROOT,'config'))
    cfg.update(parking_mode='timed_sequence',parking_slot=slot,
               wait_green=False,lidar_enabled=False)
    return cfg

def scene(t,count=2,curved=False):
    return dict(stamp=t,source='timed_parking_front',frame='base_link',lines=count,
                both_curved=curved,lane_observation=dict(stamp=t,frame='base_link',
                confidence=.9,points=[[.3,0.],[.5,0.],[.7,0.],[.9,0.]]))

def raw(task,t):
    command=encode_command(*task.command(t),cfg=task.cfg,seq=0)
    return command['speed_raw'],command['steering_raw']

class SavedParkingTests(unittest.TestCase):
    def test_first_camera_frame_can_arrive_after_long_wait(self):
        for slot in ('P1','P2','P3'):
            task=self.task(slot)
            for t in (2.1,10.,60.):
                self.assertEqual(raw(task,t),(0,0))
                self.assertFalse(task.finished)
                self.assertEqual(task.reason,slot+'_WAIT_CAMERA')
            task.observe_visual(scene(60.1),60.1)
            self.assertEqual(raw(task,60.1),(30,7))
            self.assertFalse(task.finished)

    def test_recorded_camera_gap_within_ground_budget_keeps_approaching(self):
        task=self.task()
        task.observe_visual(scene(1791299585.6322367),1791299585.6322367)
        self.assertEqual(raw(task,1791299586.462516),(30,7))
        self.assertFalse(task.finished)

    def test_scene_with_recorded_age_is_accepted_without_a_parking_timeout_option(self):
        cfg=config()
        cfg['parking_timed'].pop('camera_timeout',None)
        validate_config(cfg)
        stamp=1791299585.6322367
        self.assertEqual(validate_scene(scene(stamp),cfg,1791299586.462516),stamp)

    def task(self,slot='P3'):
        task=TimedParking(config(slot),0.,Follower())
        self.addCleanup(task.close)
        return task
    def hold(self,task):
        for t in (.1,.2):
            task.observe_visual(scene(t),t)
            self.assertEqual(raw(task,t),(30,7))
        task.observe_visual(scene(.3,0,True),.3)
        self.assertEqual(raw(task,.3),(0,0))
        self.assertEqual(task.phase,'HOLD')
        task.observe_visual(scene(.45,0,True),.45)
        self.assertEqual(raw(task,.45),(0,0))

    def test_all_profiles_complete_with_exact_saved_outputs_and_durations(self):
        first={'P1':(30,0,4.),'P2':(30,0,1.2),'P3':(-30,0,1.5)}
        tail=[(-30,22,.2),(-30,-22,3.4),(-30,0,.1),(-30,22,2.1)]
        for slot in ('P1','P2','P3'):
            task=self.task(slot)
            self.hold(task)
            t=.81
            for i,(speed,steer,duration) in enumerate([first[slot]]+tail):
                task.observe_visual(scene(t,1),t)
                self.assertEqual(raw(task,t),(speed,steer))
                self.assertEqual(task.index,i)
                deadline=task.deadline
                while t+.2 < deadline:
                    t+=.2
                    task.observe_visual(scene(t,1),t)
                    self.assertEqual(raw(task,t),(speed,steer))
                t=deadline-.001
                task.observe_visual(scene(t,1),t)
                self.assertEqual(raw(task,t),(speed,steer))
                t=deadline+.001
            task.observe_visual(scene(t,1),t)
            self.assertEqual(raw(task,t),(0,0))
            self.assertEqual(task.reason,slot+'_COMPLETE')
            self.assertTrue(task.finished)
            self.assertEqual(raw(task,t+50),(0,0))

    def test_blank_or_single_boundary_curve_does_not_stop_approach(self):
        task=self.task()
        for t,count in ((.1,2),(.2,2),(.3,0),(.4,1),(.5,0)):
            task.observe_visual(scene(t,count,False),t)
            self.assertEqual(raw(task,t),(30,7))
        self.assertEqual(task.phase,'APPROACH')

    def test_first_end_brakes_and_unconfirmed_end_never_restarts(self):
        task=self.task()
        for t in (.1,.2):task.observe_visual(scene(t),t);raw(task,t)
        task.observe_visual(scene(.3,0,True),.3)
        self.assertEqual(raw(task,.3),(0,0))
        for i in range(1,23):
            t=.3+i*.1
            task.observe_visual(scene(t,1,True),t)
            self.assertEqual(raw(task,t),(0,0))
        self.assertEqual(task.reason,'END_UNCONFIRMED_STOPPED')

    def test_camera_loss_during_fixed_stage_latches_zero(self):
        task=self.task();self.hold(task)
        task.observe_visual(scene(.81),.81)
        self.assertEqual(raw(task,.81),(-30,0))
        expired=.81+task.cfg['ground_timeout']+.01
        self.assertEqual(raw(task,expired),(0,0))
        self.assertEqual(task.reason,'PARKING_CAMERA_TIMEOUT')
        task.observe_visual(scene(expired+.1),expired+.1)
        self.assertEqual(raw(task,expired+.1),(0,0))

    def test_delayed_tick_starts_next_stage_at_actual_command(self):
        task=self.task();self.hold(task)
        task.observe_visual(scene(4),4)
        self.assertEqual(raw(task,4),(-30,0))
        self.assertAlmostEqual(task.deadline,5.5)
        task.observe_visual(scene(8),8)
        self.assertEqual(raw(task,8),(-30,22))
        self.assertAlmostEqual(task.deadline,8.2)

    def test_scene_rejects_neighboring_frames_future_and_duplicates(self):
        cfg=config()
        data=scene(1);data['lane_observation']['stamp']=.99
        with self.assertRaises(ValueError):validate_scene(data,cfg,1)
        with self.assertRaises(ValueError):validate_scene(scene(2),cfg,1)
        with self.assertRaises(ValueError):validate_scene(scene(1),cfg,1+cfg['ground_timeout']+.01)
        task=self.task()
        task.observe_visual(scene(.1),.1)
        task.observe_visual(scene(.1,0,True),.1)
        self.assertEqual(task.last_lines,2)

    def test_config_requires_manual_slot_and_complete_profiles(self):
        for slot in ('P1','P2','P3'):validate_config(config(slot))
        for slot in ('AUTO','P4','P5'):
            with self.assertRaises(ValueError):validate_config(config(slot))
        cfg=config();cfg['parking_timed']['sequences']['P1'].pop()
        with self.assertRaises(ValueError):validate_config(cfg)

class IntegrationTests(unittest.TestCase):
    def core(self,slot='P3'):
        core=Controller(config(slot));self.addCleanup(core.close)
        return core
    def start(self,core):
        for t in (.1,.2,.3):core.observe_sign('PARKING',.99,t,t)
        self.assertEqual(core.pending,'PARKING')
        self.assertEqual(core.dispatch(.3),(0,0))
        self.assertEqual(core.state,'TIMED_PARKING')
        core.scan_ready=lambda now:True
        core.sweep_clear=lambda path,unknown,report=False:True

    def test_sign_dispatch_without_blue_and_fresh_lane_uses_no_trim(self):
        for slot in ('P1','P2','P3'):
            c=self.core(slot);self.start(c)
            self.assertIsNone(c.marker)
            c.observe_timed_parking(scene(.4),.4)
            encoded=encode_command(*c.tick(.4),cfg=c.cfg,seq=0)
            self.assertEqual((encoded['speed_raw'],encoded['steering_raw']),(30,0))
            c.observe_timed_parking(scene(.5),.5);c.tick(.5)
            c.observe_timed_parking(scene(.6,0,True),.6)
            self.assertEqual(c.tick(.6),(0,0))
            c.observe_timed_parking(scene(.75,0,True),.75);c.tick(.75)
            c.observe_timed_parking(scene(1.11),1.11)
            encoded=encode_command(*c.tick(1.11),cfg=c.cfg,seq=0)
            self.assertEqual(encoded['speed_raw'],-30 if slot=='P3' else 30)
            json.dumps(controller_status(c,c.cfg,1.11,True,True,c.tick(1.11),0),allow_nan=False)

    def test_wait_green_and_route_gate_remain(self):
        c=self.core();c.state='WAIT_GREEN';c.parking_route_ready=False
        for t in (.1,.2,.3):c.observe_sign('PARKING',.99,t,t)
        self.assertIsNone(c.pending)
        c.state='LANE'
        for t in (.4,.5,.6):c.observe_sign('PARKING',.99,t,t)
        self.assertIsNone(c.pending)
        c.parking_route_ready=True
        for t in (.7,.8,.9):c.observe_sign('PARKING',.99,t,t)
        self.assertEqual(c.pending,'PARKING')

    def test_existing_direction_is_not_overridden_by_parking(self):
        c=self.core();c.pending='RIGHT'
        for t in (.1,.2,.3):c.observe_sign('PARKING',.99,t,t)
        self.assertEqual(c.pending,'RIGHT')

    def test_estop_red_missing_scan_and_obstacle_abort_latch(self):
        for fault in ('estop','red','scan','obstacle'):
            c=self.core();self.start(c)
            c.observe_timed_parking(scene(.4),.4)
            if fault in ('estop','red'):setattr(c,fault,True)
            elif fault=='scan':c.scan_ready=lambda now:False
            else:c.checked_command=lambda command,now,allow_bypass:(0,0.)
            self.assertEqual(c.tick(.4),(0,0))
            self.assertEqual(c.state,'FAULT')
            self.assertTrue(c.timed_parking.finished)
            c.estop,c.red=False,False;c.scan_ready=lambda now:True
            self.assertEqual(c.tick(.5),(0,0))

    def test_normal_lidar_guard_keeps_clear_stage_and_rejects_near_obstacle(self):
        import math
        from robot.lidar.scan import Scan
        c=self.core();c.pending='PARKING';c.dispatch(.3)
        c.cfg['lidar_enabled']=True
        c.scan=Scan([float('inf')]*360,-math.pi,math.pi/180,.05,6,c.pose,c.cfg['lidar'],.4)
        c.observe_timed_parking(scene(.4),.4)
        self.assertGreater(c.tick(.4)[0],0)
        rays=[float('inf')]*360
        for i in range(175,186):rays[i]=.2
        c.scan=Scan(rays,-math.pi,math.pi/180,.05,6,c.pose,c.cfg['lidar'],.5)
        c.observe_timed_parking(scene(.5),.5)
        self.assertEqual(c.tick(.5),(0,0))
        self.assertEqual(c.state,'FAULT')

class AdapterTests(unittest.TestCase):
    def test_vision_produces_scene_before_parking_status_arrives(self):
        from robot.parking import timed_vision_node as module
        from test_sign_training_roi import Value
        import threading
        old_rospy=module.rospy
        self.addCleanup(setattr,module,'rospy',old_rospy)
        warnings=[]
        module.rospy=Value(Time=Value(now=lambda:Value(to_sec=lambda:1.1)),
            is_shutdown=lambda:False,logwarn_throttle=lambda *a:warnings.append(a))
        class OneCycle(object):
            stopped=False
            def is_set(self):return self.stopped
            def wait(self,seconds):self.stopped=True
        node=module.Node.__new__(module.Node)
        node.cfg=config();node.options=node.cfg['parking_timed']
        node.lock=threading.Lock();node.stop=OneCycle();node.buffer=PairBuffer()
        node.active=False;node.active_stamp=-1.
        frame=object()
        image=Value(header=Value(stamp=Value(to_sec=lambda:1.)))
        node.buffer.put('image',1.,image)
        node.buffer.put('lane',1.,scene(1.)['lane_observation'])
        node.bridge=Value(imgmsg_to_cv2=lambda *args:frame)
        node.vision=Value(set_lane_observation=lambda lane:None,observe=lambda frame:(2,None))
        outputs=[]
        def publish(msg):
            outputs.append(json.loads(msg.data));node.stop.stopped=True
        node.output=Value(publish=publish)
        node.debug=Value(get_num_connections=lambda:0)
        node.work()
        self.assertEqual(len(outputs),1)
        self.assertEqual(outputs[0]['stamp'],1.)
        self.assertEqual(validate_scene(outputs[0],node.cfg,1.1),1.)
        self.assertFalse(warnings)

    def test_parking_scene_callback_accepts_the_shared_camera_budget(self):
        from test_ros_adapter import RosAdapterTests,_Clock,_FakeRospy
        import threading
        RosAdapterTests.setUpClass()
        adapter=RosAdapterTests.adapter
        adapter.rospy=_FakeRospy(_Clock(1.830279))
        node=adapter.Node.__new__(adapter.Node)
        node.cfg=config();node.core=Controller(node.cfg);self.addCleanup(node.core.close)
        node.core.pending='PARKING';node.core.dispatch(.3)
        node.lock=threading.RLock()
        node.timed_parking_scene(type('Message',(),dict(data=json.dumps(scene(1.))))())
        self.assertEqual(node.core.timed_parking.visual_stamp,1.)
        self.assertFalse(node.core.timed_parking.finished)

    def test_disabled_output_aborts_without_consuming_any_more_stages(self):
        from test_ros_adapter import RosAdapterTests,_Clock,_FakeRospy
        import threading
        from collections import deque
        try:RosAdapterTests.setUpClass()
        except unittest.SkipTest as exc:raise exc
        adapter=RosAdapterTests.adapter
        clock=_Clock(.4);fake=_FakeRospy(clock);adapter.rospy=fake
        node=adapter.Node.__new__(adapter.Node)
        node.cfg=config();node.core=Controller(node.cfg);self.addCleanup(node.core.close)
        node.core.pending='PARKING';node.core.dispatch(.3)
        node.lock=threading.RLock();node.history=deque(maxlen=100)
        node.last_tick=.3;node.live=True;node.seq=0
        outputs=[]
        node.output=type('Output',(),dict(publish=lambda obj,msg:outputs.append(json.loads(msg.data))))()
        node.tick(None)
        self.assertTrue(node.core.timed_parking.finished)
        self.assertEqual(node.core.reason,'automatic_output_disabled')
        self.assertEqual(outputs[-1]['speed_raw'],0)
        fake.params['~enabled']=True;clock.value=.5
        node.tick(None)
        self.assertEqual(outputs[-1]['speed_raw'],0)
        self.assertEqual(node.core.state,'FAULT')

class PairTests(unittest.TestCase):
    def test_exact_capture_pair_only_and_bounded_history(self):
        b=PairBuffer(3);b.put('image',1,'image');b.put('lane',1.01,'lane')
        self.assertIsNone(b.newest(1.1,.8))
        b.put('lane',1,'matching')
        self.assertEqual(b.newest(1.1,.8),('image','matching'))
        self.assertIsNone(b.newest(1.1,.8))
        for i in range(10):b.put('image',i,i)
        self.assertEqual(len(b.images),3)
        b.put('lane',9,9)
        self.assertIsNone(b.newest(8,.8))
        self.assertIsNone(b.newest(10,.8))

    def test_both_actual_boundaries_required_in_same_bend(self):
        xs=[.45+.1*i for i in range(11)]
        left=[[x,.3+.3*(x-.95)**2] for x in xs]
        right=[[x,-.3+.3*(x-.95)**2] for x in xs]
        self.assertTrue(both_boundary_curves(dict(LEFT=left,RIGHT=right))['both_curved'])
        self.assertFalse(both_boundary_curves(dict(LEFT=left))['both_curved'])

if __name__=='__main__':unittest.main()
