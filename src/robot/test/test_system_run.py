from __future__ import division
import copy
import math
import os
import sys
import unittest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from robot.lidar.scan import Scan
from robot.common.geometry import bicycle
from robot.common.geometry import wrap


class SystemRunTests(unittest.TestCase):
    def test_bypass_can_start_early_in_an_authorized_clear_sweep(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg['bypass_enabled'],cfg['wait_green'] = True,False
        cfg['bypass_zones'] = [[-0.5,3.8,-0.5,1.0]]
        c = Controller(cfg)
        self.addCleanup(c.close)
        rays = []
        for i in range(720):
            a = -math.pi+i*2*math.pi/720
            along = 1.1*math.cos(a)
            cross = abs(1.1*math.sin(a))
            rays.append(along-math.sqrt(0.1**2-cross**2) if along > 0 and cross <= 0.1 else float('inf'))
        c.scan = Scan(rays,-math.pi,2*math.pi/720,.05,6,c.pose,cfg['lidar'],1.0)
        self.assertEqual(c.checked_command((25,0.0),1.0,True),(0,0.0))
        self.assertEqual(c.action,'BYPASS',c.reason)
        self.assertEqual(c.state,'MANEUVER')
        c.observe_lane([],0,1.05)
        raw,steer = c.tick(1.05)
        self.assertGreater(raw,0,c.reason)

    def test_locked_bay_corrects_model_drift_instead_of_moving_goal(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg['parking_slot'] = 'P5'
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.slot = dict(pose=(0.7,-0.55,math.pi/2),length=0.45,width=0.38,kind='perpendicular',id='P5')
        c.slot_locked = True
        correction = c.observe_ground(dict(markers=[],slots=[dict(x=0.8,y=-0.55,yaw=math.pi/2,kind='perpendicular')]),1.0)
        self.assertLess(correction[0],0)
        self.assertLessEqual(abs(correction[0]),cfg['parking_visual_max_shift'])
        self.assertEqual(c.slot['pose'],(0.7,-0.55,math.pi/2))

    def test_stale_green_cannot_release_but_fresh_single_frame_can(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:cfg=yaml.safe_load(f)
        cfg['wait_green']=True
        c=Controller(cfg);self.addCleanup(c.close)
        c.observe_sign('GREEN',.99,1,5)
        self.assertEqual(c.state,'WAIT_GREEN')
        c.observe_sign('GREEN',.6,5,5)
        self.assertEqual(c.state,'STARTUP_STRAIGHT')

    def test_exit_votes_reset_when_lane_becomes_stale(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg['wait_green'] = False
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.state,c.exit_pose,c.action_started = 'REACQUIRE',(0,0,0),.9
        c.scan = Scan([float('inf')]*360,-math.pi,2*math.pi/360,.05,6,c.pose,cfg['lidar'],1.0)
        c.observe_lane([(0.2,0),(0.5,0)],0.99,1.0)
        c.tick(1.0)
        self.assertEqual(c.exit_count,1)
        c.scan.stamp = 2.0
        c.tick(2.0)
        self.assertEqual(c.exit_count,0)

    def run_turn_without_white(self,action):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg['right_turn_full_lock'] = False  # generic path follower; lock has its own visual tests
        c = Controller(cfg)
        self.addCleanup(c.close)
        for i in range(3):
            c.observe_sign('GREEN',0.99,0.1+i*0.1,0.1+i*0.1)
        for i in range(3):
            c.observe_sign(action,0.99,0.5+i*0.1,0.5+i*0.1)
        c.observe_ground(dict(markers=[dict(kind='junction',x=0.30,y=0.0)],slots=[]),0.8)
        start = c.pose
        moved_without_white = False
        for i in range(550):
            now = 0.9+i*0.05
            c.scan = Scan([float('inf')]*360,-math.pi,2*math.pi/360,.05,6,c.pose,cfg['lidar'],now)
            c.observe_ground(dict(source='front',part='markers',markers=
                [dict(kind='junction',x=.3,y=0)] if action == 'STRAIGHT' and c.pose[0]-start[0] > .72 else [],slots=[]),now)
            if c.state == 'REACQUIRE':
                c.observe_lane([(0.2,0),(0.5,0),(0.8,0)],0.95,now)
            if action == 'STRAIGHT':
                # Fresh blank frames while crossing the gap; exit appears ahead.
                visible = c.pose[0]-start[0] > .20
                c.observe_lane([(0.2,0),(0.5,0),(0.8,0)] if visible else [],0.95 if visible else 0.0,now)
            raw,steer = c.tick(now)
            if c.state == 'MANEUVER' and raw:
                moved_without_white = True
            gain = cfg['raw_to_mps']['forward' if raw >= 0 else 'reverse']
            c.set_pose(bicycle(c.pose,raw*gain*0.05,steer,cfg['wheelbase']),now)
            if c.state == 'LANE' and c.completed_at_pose is not None:
                break
        self.assertTrue(moved_without_white)
        self.assertEqual(c.state,'LANE',c.reason)
        expected = {'STRAIGHT':0,'LEFT':math.pi/2,'RIGHT':-math.pi/2}[action]
        self.assertLess(abs(wrap(c.pose[2]-expected)),cfg['exit_yaw_tolerance'])
        self.assertNotEqual(start,c.pose)

    def test_straight_without_intersection_white(self):
        self.run_turn_without_white('STRAIGHT')

    def test_left_without_intersection_white(self):
        self.run_turn_without_white('LEFT')

    def test_right_without_intersection_white(self):
        self.run_turn_without_white('RIGHT')


if __name__ == '__main__':
    unittest.main()
