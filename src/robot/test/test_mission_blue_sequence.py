"""Synthetic observations/poses test mission ordering, not real driving accuracy."""
import math
import os
import unittest
from robot.common.config import load_config
from robot.common.geometry import world
from robot.master.controller import Controller
from blue_test_helpers import enter_blue_action, clear_blue


class MissionBlueSequenceTests(unittest.TestCase):
    def frame(self,c,now):
        c.observe_ground(dict(source='front',part='markers',slots=[],markers=[]),now)
        c.observe_lane([(.5,0),(.7,0),(.9,0)],.9,now)

    def finish_straight(self,c,now):
        pose=tuple(world(c.pose,(1.26,0)))+(c.pose[2],)
        c.set_pose(pose,now)
        for t in (now,now+.1,now+.2):
            self.frame(c,t);c.tick(t)
        self.assertEqual(c.state,'LANE',c.reason)
        return now+.2

    def run_sequence(self,actions):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        cfg.update(wait_green=True,lidar_enabled=False,blue_default_straight=False,
                   sign_ttl=0,right_exit_on_blue=True,right_reverse_entry_m=.25)
        c=Controller(cfg);self.addCleanup(c.close)
        c.observe_sign('GREEN',.99,1.,1.)
        self.assertEqual(c.state,'STARTUP_STRAIGHT')
        c.set_pose((1.26,0,0),2.);self.frame(c,2.);c.tick(2.)
        self.assertEqual(c.state,'LANE')
        now=3.
        for action in actions:
            clear_blue(c,now)
            for t in (now+.1,now+.2,now+.3):
                c.observe_sign(action,.99,t,t)
            self.assertEqual(c.pending,action)
            if action=='PARKING':
                c.parking_sign=dict(point=world(c.pose,(.85,0)),stamp=now+.3)
            self.frame(c,now+.3);c.tick(now+.3)
            self.assertIsNone(c.action)  # Sign alone cannot take over.
            enter_blue_action(c,action,now+.8)
            now+=1.
            if action=='STRAIGHT':
                now=self.finish_straight(c,now)
            elif action=='LEFT':
                exit_pose=c.exit_pose
                c.set_pose(tuple(world(exit_pose,(.01,0)))+(exit_pose[2],),now)
                for t in (now,now+.1,now+.2):
                    self.frame(c,t);c.tick(t)
                self.assertEqual(c.state,'LANE',c.reason)
                now+=.2
            elif action=='RIGHT':
                self.frame(c,now)
                self.assertLess(c.tick(now)[0],0)
                origin=c.pose
                c.set_pose(tuple(world(origin,(-.26,0)))+(origin[2],),now+.1)
                self.frame(c,now+.1);c.tick(now+.1)
                c.set_pose(c.pose[:2]+(origin[2]-math.pi/2,),now+.2)
                c.observe_ground(dict(source='front',part='markers',slots=[],
                    markers=[dict(kind='junction',x=.8,y=0,length=.6)],
                    blue_lines=[dict(x=.8,y=0,yaw=0,length=.6)]),now+.2)
                c.tick(now+.2)
                self.assertEqual(c.action,'STRAIGHT')  # Existing right-exit continuation.
                now=self.finish_straight(c,now+.3)
            elif action=='UTURN':
                for i in range(800):
                    self.frame(c,now);c.tick(now)
                    if c.state=='LANE':break
                    now+=.05
                self.assertEqual(c.state,'LANE',c.reason)
            else:
                c.observe_ground(dict(source='front',part='parking_lines',lines=[
                    [(.4,-.19),(.85,-.19)],[(.4,.19),(.85,.19)],
                    [(.85,-.19),(.85,.19)]]),now)
                self.assertGreater(c.tick(now)[0],0)
                self.assertEqual(c.state,'PARKING')
                pose=tuple(world(c.pose,(.481,0.)))+(c.pose[2],)
                c.set_pose(pose,now+.1)
                c.observe_ground(dict(source='front',part='parking_lines',lines=[]),now+.1)
                self.assertEqual(c.tick(now+.1),(0,0.))
                self.assertEqual(c.state,'FINISHED')
            now+=1.
        self.assertEqual(c.state,'FINISHED')

    def test_repeated_straight_uturn_left_then_park(self):
        self.run_sequence(('STRAIGHT','STRAIGHT','UTURN','LEFT','PARKING'))

    def test_different_order_right_straight_left_then_park(self):
        self.run_sequence(('RIGHT','STRAIGHT','LEFT','PARKING'))


if __name__=='__main__':unittest.main()
