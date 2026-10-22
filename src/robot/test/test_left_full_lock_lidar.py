"""No hardware: production left-lock steering and lidar delivery jitter."""
import math
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command, model_to_command_steering
from robot.common.geometry import bicycle
from robot.master.controller import Controller
from robot.turn.planner import intersection_path, turn_parameters
from robot.motion.tracker import Follower


class LeftFullLockLidarTests(unittest.TestCase):
    def config(self, full_lock=None):
        cfg=load_config(os.path.join(os.path.dirname(os.path.dirname(__file__)),'config'))
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.1,
                   right_turn_radius=.55)
        if full_lock is not None:cfg['left_turn_full_lock']=full_lock
        return cfg

    def test_production_left_uses_configured_radius_and_path_tracking(self):
        cfg=self.config()
        self.assertFalse(cfg['left_turn_full_lock'])
        self.assertAlmostEqual(turn_parameters(cfg,'LEFT')['effective_radius'],.65)
        c=Controller(cfg);self.addCleanup(c.close)
        c.start_follow(intersection_path(c.pose,'LEFT',cfg),'LEFT',1.)
        self.assertFalse(c.follower.full_lock)
        arc=[q for q in c.follower.path if q[4]>0]
        self.assertTrue(arc)
        for q in arc:
            self.assertAlmostEqual(cfg['wheelbase']/math.tan(q[4]),.65)

    def test_opt_in_left_arc_plans_full_lock_radius(self):
        cfg=self.config(True)
        self.assertTrue(cfg.get('left_turn_full_lock',False))
        p=turn_parameters(cfg,'LEFT')
        self.assertAlmostEqual(p['effective_radius'],cfg['wheelbase']/math.tan(cfg['max_steer']))
        arc=[q for q in intersection_path((0,0,0),'LEFT',cfg) if q[4]>0]
        self.assertTrue(arc)
        for q in arc:
            raw=encode_command(28,model_to_command_steering(q[4],cfg),cfg,0)
            self.assertEqual(raw['steering_raw'],22)

    def test_controller_wires_full_lock_only_for_left(self):
        cfg=self.config(True);c=Controller(cfg);self.addCleanup(c.close)
        c.start_follow(intersection_path(c.pose,'LEFT',cfg),'LEFT',1.)
        arc=next(q for q in c.follower.path if q[4]>0)
        c.set_pose(arc[:3],1.1)
        c.observe_ground(dict(source='front',part='markers',markers=[]),1.1)
        speed,steer=c.tick(1.1)
        self.assertEqual(encode_command(speed,steer,cfg,0)['steering_raw'],22)

    def test_disabled_left_lock_preserves_old_radius(self):
        cfg=self.config();cfg['left_turn_full_lock']=False
        self.assertAlmostEqual(turn_parameters(cfg,'LEFT')['effective_radius'],.65)
        self.assertAlmostEqual(turn_parameters(cfg,'RIGHT')['effective_radius'],.55)

    def test_full_lock_entry_arc_exit_model_finishes_without_reverse_steering(self):
        cfg=self.config(True);path=intersection_path((0,0,0),'LEFT',cfg)
        follower=Follower(path,cfg,full_lock=True)
        pose=(0.,0.,0.);arc_count=0;tail=False
        for i in range(1000):
            speed,physical=follower.command(pose,i*.05)
            if follower.done:break
            self.assertIn(round(physical,6),(0.,round(cfg['max_steer'],6)))
            if physical>0:arc_count+=1
            elif arc_count:tail=True
            pose=bicycle(pose,speed*.008*.05,physical,cfg['wheelbase'])
        self.assertTrue(follower.done)
        self.assertGreater(arc_count,20)
        self.assertTrue(tail)
        self.assertAlmostEqual(pose[2],math.pi/2,delta=.05)

    def test_lidar_tolerates_recorded_jitter_but_not_long_outage(self):
        cfg=self.config();c=Controller(cfg);self.addCleanup(c.close)
        c.scan=type('Scan',(),dict(stamp=2.,completed_stamp=2.1,valid_rays=1440))()
        self.assertTrue(c.scan_ready(2.65))  # recorded completion age .55 s
        self.assertFalse(c.scan_ready(2.91))
        self.assertFalse(c.scan_ready(2.0))
        c.scan.valid_rays=0
        self.assertFalse(c.scan_ready(2.2))
        self.assertEqual(cfg['sensor_timeout'],.5)

    def test_left_handoff_reuses_recovery_for_opposite_steering_only(self):
        for y in (-.15,.15):
            cfg=self.config();c=Controller(cfg);self.addCleanup(c.close)
            c.set_pose((0.,0.,0.),1.)
            c.action='LEFT';c.action_source='sign'
            c.observe_applied_steering(.1,1.)
            c.observe_lane([(.65,y),(.75,y+.05)],.9,1.)
            c.resume_lane()
            speed,steer=c.lane_command(1.)
            self.assertEqual(encode_command(speed,steer,cfg,0)['steering_raw'],22)
            if y<0:
                self.assertIsNotNone(c.lane_recovery)
                c.observe_lane([(.65,y),(.75,y+.05)],.9,1.6)
                self.assertLess(c.lane_command(1.6)[1],0)

    def test_startup_does_not_zero_speed_for_short_scan_delay(self):
        cfg=self.config();cfg['lidar_enabled']=True
        c=Controller(cfg);self.addCleanup(c.close)
        from robot.lidar.scan import Scan
        c.begin_startup(1.)
        c.scan=Scan([float('inf')]*360,-math.pi,math.pi/180,.05,6,c.pose,cfg['lidar'],2.)
        c.scan.completed_stamp=2.1
        c.observe_ground(dict(source='front',part='markers',markers=[]),2.65)
        self.assertGreater(c.tick(2.65)[0],0)
        c.observe_ground(dict(source='front',part='markers',markers=[]),2.91)
        self.assertEqual(c.tick(2.91),(0,0.))
        self.assertEqual(c.reason,'scan_missing_or_stale')


if __name__=='__main__':unittest.main()
