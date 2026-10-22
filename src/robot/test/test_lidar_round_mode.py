"""No hardware: optional lidar and polar point-cloud round-object filtering."""
from __future__ import division
import math
import os
import sys
import unittest
import yaml
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.master.controller import Controller
from robot.lidar.scan import Scan
from robot.common.planning import intersection_path


class LidarModeTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            self.cfg = yaml.safe_load(f)
        self.cfg.update(wait_green=False,lidar_enabled=False)
        self.c = Controller(self.cfg)
        self.addCleanup(self.c.close)

    def scan(self,kind):
        rays = []
        for i in range(1440):
            a = -math.pi+i*math.pi/720
            if kind == 'wall':
                r = .4/math.cos(a) if abs(a)<.7 else float('inf')
            elif kind == 'circle':
                # Circle centered 0.48 m ahead, radius 0.08 m.
                d = .08**2-(.48*math.sin(a))**2
                r = .48*math.cos(a)-math.sqrt(d) if d>=0 and math.cos(a)>0 else float('inf')
            else:
                r = float('nan')
            rays.append(r)
        extrinsic = dict(self.cfg['lidar'],shape_filter=True,shape_mode='round_only')
        return Scan(rays,-math.pi,math.pi/720,.05,6,(0,0,0),extrinsic,1)

    def test_disabled_lane_never_requires_scan(self):
        self.c.observe_lane([(.55,0),(.7,0)],.8,1)
        self.assertEqual(self.c.tick(1),(20,0))
        self.assertEqual(self.c.obstacle_check['kind'],'disabled')

    def test_disabled_turn_never_requires_scan(self):
        self.cfg.update(right_turn_full_lock=False,right_turn_entry=0)
        self.c.start_follow(intersection_path(self.c.pose,'RIGHT',self.cfg),'RIGHT',1)
        self.c.observe_lane([],0,1)
        speed,steer = self.c.tick(1)
        self.assertGreater(speed,0)
        self.assertLess(steer,0)

    def test_disabled_does_not_bypass_green_or_estop(self):
        self.c.state = 'WAIT_GREEN'
        self.assertEqual(self.c.tick(1),(0,0))
        self.c.state,self.c.estop = 'LANE',True
        self.assertEqual(self.c.tick(1),(0,0))

    def test_disabled_cannot_claim_parking_bay_clear(self):
        self.c.state,self.c.action = 'PARKING_SCAN','PARKING'
        self.assertEqual(self.c.tick(1),(0,0))
        self.assertEqual(self.c.reason,'scan_missing_or_stale')
        self.assertEqual(self.c.start_parking(1),(0,0))

    def test_circle_detected_but_raw_scan_preserved(self):
        scan = self.scan('circle')
        self.assertEqual(len(scan.round_clusters),1)
        self.assertAlmostEqual(scan.round_clusters[0]['radius_m'],.08,places=4)
        self.assertTrue(scan.motion_obstacles)
        self.assertTrue(scan.obstacles)

    def test_wall_not_classified_as_round_and_raw_occupancy_remains(self):
        scan = self.scan('wall')
        self.assertEqual(scan.round_clusters,[])
        self.assertEqual(scan.motion_obstacles,[])
        self.assertEqual(scan.classify((.4,0)),'OCCUPIED')

    def test_enabled_motion_ignores_wall_but_stops_for_circle(self):
        self.cfg['lidar_enabled'] = True
        self.c.scan = self.scan('wall')
        self.assertEqual(self.c.checked_command((20,0),1,False),(20,0))
        self.c.scan = self.scan('circle')
        self.assertEqual(self.c.checked_command((20,0),1,False),(0,0))

    def test_parking_keeps_wall_collision_check(self):
        self.cfg['lidar_enabled'] = True
        self.c.scan = self.scan('wall')
        self.c.action,self.c.state = 'PARKING','PARKING'
        self.assertEqual(self.c.checked_command((16,0),1,False),(0,0))

    def test_enabled_invalid_scan_still_stops(self):
        self.cfg['lidar_enabled'] = True
        self.c.scan = self.scan('invalid')
        self.assertEqual(self.c.tick(1),(0,0))
        self.assertEqual(self.c.reason,'scan_missing_or_stale')


if __name__ == '__main__':
    unittest.main()
