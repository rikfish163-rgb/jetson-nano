"""Shape discrimination from synthetic 2D point clusters, not semantic proof."""
from __future__ import division
import math
import os
import sys
import unittest
import yaml
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0,os.path.join(ROOT,'src'))
from robot.common import geometry as g
from robot.lidar import scan as lidar


class ShapeTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            self.cfg = yaml.safe_load(f)['lidar']
        self.cfg['shape_mode'] = 'round_only' # Explicit regression for retained legacy mode.

    def arc(self,r=.08,n=35,span=140):
        return [(.6+r*math.cos(math.radians(180-span/2+span*i/(n-1))),
                 r*math.sin(math.radians(180-span/2+span*i/(n-1)))) for i in range(n)]

    def classify(self,points):
        return lidar.classify_scan_cluster(points,self.cfg)

    def test_cone_or_cylinder_section_is_round_candidate_only(self):
        row = self.classify(self.arc())
        self.assertEqual(row['kind'],'round_candidate')
        self.assertAlmostEqual(row['radius_m'],.08,places=5)
        self.assertFalse(row['semantic_verified'])

    def test_small_flat_board_is_not_round(self):
        points = [(.4+.001*math.sin(i),-.10+.2*i/30) for i in range(31)]
        self.assertEqual(self.classify(points)['kind'],'flat_board_candidate')

    def test_large_flat_surface_is_wall_candidate(self):
        points = [(.4,-.6+1.2*i/50) for i in range(51)]
        self.assertEqual(self.classify(points)['kind'],'wall_candidate')

    def test_sign_thin_post_is_not_target(self):
        self.assertEqual(self.classify(self.arc(.012))['kind'],'thin_post_candidate')

    def test_box_corner_is_not_round(self):
        points = [(.5+abs(t),t) for t in [i*.005 for i in range(-20,21)]]
        self.assertEqual(self.classify(points)['kind'],'corner_candidate')

    def test_shallow_arc_cannot_confirm_round(self):
        self.assertNotEqual(self.classify(self.arc(span=15))['kind'],'round_candidate')

    def test_sparse_and_duplicate_points_are_unknown(self):
        for points in ([(.4,0)]*12,self.arc(n=3)):
            self.assertEqual(self.classify(points)['kind'],'unknown')

    def test_noisy_circle_still_matches(self):
        points = [(x+.001*math.sin(i*2),y+.001*math.cos(i*3)) for i,(x,y) in enumerate(self.arc())]
        self.assertEqual(self.classify(points)['kind'],'round_candidate')

    def test_geometry_equivalent_support_cannot_be_claimed_as_semantic_cone(self):
        row = self.classify(self.arc(.08))
        self.assertEqual(row['kind'],'round_candidate')
        self.assertFalse(row['semantic_verified'])

    def test_cluster_order_reversal_does_not_change_shape(self):
        self.assertEqual(self.classify(list(reversed(self.arc())))['kind'],'round_candidate')

    def circle_scan(self,bearing=0,offset=None):
        ranges = []
        for i in range(1440):
            angle = -math.pi+i*math.pi/720-bearing
            d = .08**2-(.48*math.sin(angle))**2
            ranges.append(.48*math.cos(angle)-math.sqrt(d) if d>=0 and math.cos(angle)>0 else float('inf'))
        return lidar.Scan(ranges,-math.pi,math.pi/720,.05,6,(1,2,.3),
                      dict(self.cfg,shape_filter=True,**(offset or {})),1)

    def test_scan_seam_merges_same_round_object_once(self):
        scan = self.circle_scan(math.pi)
        self.assertEqual(len(scan.round_clusters),1)
        self.assertAlmostEqual(scan.round_clusters[0]['radius_m'],.08,places=4)

    def test_invalid_endpoint_does_not_merge_unrelated_clusters(self):
        rays = [0]+[.5]*8+[float('inf')]*342+[.5]*8+[0]
        rows = []
        lidar.round_scan_clusters(rays,-math.pi,2*math.pi/360,.05,6,self.cfg,rows)
        self.assertEqual(len(rows),2)

    def test_classification_diagnostics_are_finite_json(self):
        import json
        for points in (self.arc(),[(.4,0)]*10,[(float('nan'),0)]*10):
            json.dumps(self.classify(points),allow_nan=False)

    def test_scan_pose_and_extrinsic_used_for_target_obstacles(self):
        scan = self.circle_scan(offset=dict(x=.1,y=.2,yaw=.3))
        expected = g.world(scan.pose,(.48,0))
        self.assertAlmostEqual(scan.motion_obstacles[0][0],expected[0],places=4)
        self.assertAlmostEqual(scan.motion_obstacles[0][1],expected[1],places=4)

    def test_invalid_geometry_is_rejected(self):
        for amin,inc,rmin,rmax,stamp in ((0,0,.05,6,1),(float('nan'),.01,.05,6,1),
                                       (0,.01,6,.05,1),(0,.01,.05,6,float('nan'))):
            with self.assertRaises(ValueError):
                lidar.Scan([1,1],amin,inc,rmin,rmax,(0,0,0),self.cfg,stamp)

    def test_filtered_scan_diagnostics_include_ignored_board(self):
        rays = [.4/math.cos(-math.pi+i*math.pi/720) if abs(-math.pi+i*math.pi/720)<.2
                else float('inf') for i in range(1440)]
        scan = lidar.Scan(rays,-math.pi,math.pi/720,.05,6,(0,0,0),dict(self.cfg,shape_filter=True),1)
        self.assertEqual(scan.cluster_diagnostics[0]['kind'],'flat_board_candidate')
        self.assertFalse(scan.motion_obstacles)
        self.assertTrue(scan.obstacles)

    def test_excessive_clusters_are_rejected_before_fitting(self):
        rays = ([.5]*6+[float('inf')])*129
        with self.assertRaises(ValueError):
            lidar.round_scan_clusters(rays,-math.pi,2*math.pi/len(rays),.05,6,self.cfg)


class ScanAdapterTests(unittest.TestCase):
    def setUp(self):
        import test_ros_adapter as fixture
        if fixture._rospy is None or fixture.imp is None:
            self.skipTest('requires ROS')
        self.fixture = fixture
        self.env = fixture.RosAdapterTests('test_applied_raw_over_contract_clears_command')
        self.env.setUpClass()
        self.env.setUp()
        self.addCleanup(self.env.doCleanups)
        self.n = self.env.node
        from collections import deque
        self.n.history = deque([(9.8,(0,0,0)),(10,(.1,0,0))])

    def message(self,stamp=9.9):
        from sensor_msgs.msg import LaserScan
        m = LaserScan()
        m.header.stamp = self.fixture._rospy.Time.from_sec(stamp)
        m.header.frame_id = 'laser_link'
        m.angle_min,m.angle_increment = -math.pi,math.pi/180
        m.range_min,m.range_max = .05,6
        m.ranges = [float('inf')]*360
        return m

    def test_scan_fit_does_not_hold_control_lock(self):
        original = self.env.adapter.Scan
        self.addCleanup(setattr,self.env.adapter,'Scan',original)
        def checked(*args):
            self.assertFalse(self.n.lock._is_owned())
            return original(*args)
        self.env.adapter.Scan = checked
        self.n.scan(self.message())
        self.assertAlmostEqual(self.n.core.scan.stamp,9.9)

    def test_old_scan_cannot_overwrite_new_scan(self):
        self.n.scan(self.message(10))
        self.n.scan(self.message(9.8))
        self.assertEqual(self.n.core.scan.stamp,10)

    def test_acquisition_start_is_not_completion_time(self):
        msg=self.message(9.4)
        msg.time_increment=.3/359
        self.n.scan(msg)
        self.assertIsNotNone(self.n.core.scan)
        self.assertAlmostEqual(self.n.core.scan.stamp,9.4)
        self.assertAlmostEqual(self.n.core.scan.completed_stamp,9.7)
        self.assertTrue(self.n.core.scan_ready(10.))
        self.assertFalse(self.n.core.scan_ready(10.3))

    def test_future_last_ray_is_rejected(self):
        msg=self.message(9.9);msg.time_increment=.3/359
        self.n.scan(msg)
        self.assertIsNone(self.n.core.scan)

    def test_duplicate_scan_skips_fitting(self):
        self.n.scan(self.message())
        original=self.env.adapter.Scan
        self.addCleanup(setattr,self.env.adapter,'Scan',original)
        def fail(*args):raise AssertionError('duplicate was fitted')
        self.env.adapter.Scan=fail
        self.n.scan(self.message())

    def test_future_or_bad_geometry_does_not_replace_scan(self):
        self.n.scan(self.message(9.9))
        self.n.scan(self.message(11))
        msg = self.message(10)
        msg.range_max = .01
        self.n.scan(msg)
        self.assertAlmostEqual(self.n.core.scan.stamp,9.9)


if __name__ == '__main__':
    unittest.main()
