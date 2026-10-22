"""User-labelled real scans + synthetic invariance; no ROS or actuators."""
from __future__ import division
import copy
import json
import math
import os
import unittest
from test_auto_parking import CONFIG
from robot.common import geometry as g
from robot.lidar import scan as lidar
from robot.master.controller import Controller
from robot.common.contracts import validate_config

class LineRejectTests(unittest.TestCase):
    def setUp(self):
        self.cfg = copy.deepcopy(CONFIG)
        self.cfg['lidar'].update(shape_filter=True,shape_mode='line_reject')
        with open(os.path.join(os.path.dirname(__file__),'fixtures','lidar_user_shapes.json')) as f:
            self.frames = json.load(f)['frames']

    def scan(self,index,pose=(0,0,0)):
        f = self.frames[index]
        return lidar.Scan([float('inf') if r is None else r for r in f['ranges']],
                      f['angle_min'],f['angle_increment'],f['range_min'],f['range_max'],
                      pose,self.cfg['lidar'],f['stamp'])

    def box_points(self,points,box):
        return [p for p in points if g.in_box(p,box)]

    def test_image1_box_preserves_all_24_raw_obstacle_points(self):
        s = self.scan(0)
        box = (.50,.60,-.03,.07)
        raw = self.box_points(s.obstacles,box)
        kept = self.box_points(s.motion_obstacles,box)
        self.assertEqual(len(raw),24)
        self.assertEqual(set(raw),set(kept))

    def test_image2_two_sign_lines_removed_irregular_cluster_kept(self):
        s = self.scan(1)
        for box,n in (((.30,.36,-.70,-.48),28),((.56,.73,-.83,-.78),24)):
            self.assertEqual(len(self.box_points(s.obstacles,box)),n)
            self.assertEqual(self.box_points(s.motion_obstacles,box),[])
        box = (1.00,1.08,-.58,-.50)
        raw = self.box_points(s.obstacles,box)
        self.assertEqual(len(raw),7)
        self.assertEqual(set(raw),set(self.box_points(s.motion_obstacles,box)))

    def test_real_scan_diagnostics_explain_policy(self):
        rows = self.scan(1).cluster_diagnostics
        near = [r for r in rows if 'center' in r and g.in_box(r['center'],(1,1.08,-.58,-.50))]
        self.assertEqual(len(near),1)
        self.assertEqual(near[0]['kind'],'obstacle_candidate')
        self.assertTrue(near[0]['motion_target'])
        self.assertFalse(near[0]['semantic_verified'])

    def test_lines_at_any_orientation_are_ignored(self):
        for angle in (0,.3,1.57,2.8):
            points = [g.world((.7,-.3,angle),(i*.005,.0007*math.sin(i))) for i in range(31)]
            r = lidar.classify_scan_cluster(points,self.cfg['lidar'])
            self.assertEqual(r['kind'],'flat_board_candidate')
            self.assertFalse(r['motion_target'])

    def test_irregular_clusters_do_not_need_circle_fit(self):
        points = [(.5+abs(i)*.005,i*.005) for i in range(-6,7)]
        for angle in (0,.8,2.1):
            r = lidar.classify_scan_cluster([g.world((1,1,angle),p) for p in points],self.cfg['lidar'])
            self.assertEqual(r['kind'],'obstacle_candidate')
            self.assertTrue(r['motion_target'])

    def test_sparse_or_degenerate_does_not_become_obstacle(self):
        for points in ([(.4,0)],[(.4,0)]*10,[(.4+i*.001,i*.001) for i in range(5)]):
            self.assertFalse(lidar.classify_scan_cluster(points,self.cfg['lidar'])['motion_target'])

    def test_raw_geometry_transforms_with_pose(self):
        s = self.scan(0,pose=(1,2,.5))
        for p in s.motion_obstacles:
            self.assertIn(p,s.obstacles)

    def test_default_config_selects_line_reject(self):
        self.assertEqual(CONFIG['lidar']['shape_mode'],'line_reject')

    def test_user_shapes_with_rotation_and_small_measurement_jitter(self):
        cases = ((0,(.50,.60,-.03,.07),True),
                 (1,(.30,.36,-.70,-.48),False),
                 (1,(.56,.73,-.83,-.78),False),
                 (1,(1.00,1.08,-.58,-.50),True))
        for frame,box,target in cases:
            points = self.box_points(self.scan(frame).obstacles,box)
            for k in range(10):
                moved = [g.world((1,-2,k*.3),(p[0]+.0005*math.sin(i+k),
                                             p[1]+.0005*math.cos(2*i+k))) for i,p in enumerate(points)]
                row = lidar.classify_scan_cluster(moved,self.cfg['lidar'])
                self.assertEqual(row['motion_target'],target)

    def test_obstacle_command_reports_raw_evidence(self):
        c = Controller(self.cfg)
        self.addCleanup(c.close)
        c.scan = self.scan(0)
        self.assertFalse(c.sweep_clear([(.35,0,0)],False,True))
        self.assertEqual(c.obstacle_check['mode'],'line_reject')
        self.assertEqual(c.obstacle_check['evidence_type'],'raw_cluster_points')

    def test_sign_remains_collision_for_parking_only(self):
        s = self.scan(1)
        c = Controller(self.cfg)
        self.addCleanup(c.close)
        c.scan = s
        path = [(.325,-.57,0)]
        self.assertTrue(c.sweep_clear(path,require_coverage=False))
        c.action = 'PARKING'
        self.assertFalse(c.sweep_clear(path,require_coverage=False))

    def test_invalid_line_policy_config_rejected(self):
        for overrides in ({'shape_mode':'typo'},{'line_max_ratio':0},
                          {'line_inlier_ratio':1.1},{'compact_min_span':.7,'compact_max_span':.5}):
            cfg = copy.deepcopy(self.cfg)
            cfg['lidar'].update(overrides)
            with self.assertRaises(ValueError):
                validate_config(cfg)

    def labelled_short_shapes(self):
        with open(os.path.join(os.path.dirname(__file__),'fixtures','lidar_short_board.json')) as f:
            return json.load(f)

    def test_measured_B_is_board_not_obstacle(self):
        points = self.labelled_short_shapes()['B']
        self.assertEqual(len(points),14)
        row = lidar.classify_scan_cluster(points,self.cfg['lidar'])
        self.assertEqual(row['kind'],'flat_board_candidate')
        self.assertFalse(row['motion_target'])

    def test_measured_C_still_obstacle(self):
        row = lidar.classify_scan_cluster(self.labelled_short_shapes()['C'],self.cfg['lidar'])
        self.assertEqual(row['kind'],'obstacle_candidate')
        self.assertTrue(row['motion_target'])

    def test_resolved_short_lines_do_not_need_eight_cm(self):
        for length in (.02,.04,.074,.12):
            for angle in (0,.7,1.57):
                points = [g.world((.8,-.3,angle),(length*i/14,0)) for i in range(15)]
                self.assertEqual(lidar.classify_scan_cluster(points,self.cfg['lidar'])['kind'],'flat_board_candidate')

    def test_unresolved_tiny_line_is_unknown_not_obstacle(self):
        points = [(.5+i*.0004,0) for i in range(15)]
        row = lidar.classify_scan_cluster(points,self.cfg['lidar'])
        self.assertEqual(row['kind'],'unknown')
        self.assertFalse(row['motion_target'])

    def test_borderline_straightness_is_unknown_not_automatic_obstacle(self):
        points = [(.5+i*.003,.0024*(1 if i%2 else -1)) for i in range(25)]
        row = lidar.classify_scan_cluster(points,self.cfg['lidar'])
        self.assertEqual(row['kind'],'unknown')
        self.assertFalse(row['motion_target'])

    def test_B_rotation_order_and_distance_do_not_change_class(self):
        points = self.labelled_short_shapes()['B']
        for pose in ((0,0,0),(1,-1,.7),(-2,3,1.57)):
            moved = [g.world(pose,p) for p in points]
            for variant in (moved,list(reversed(moved)),moved[::2]+moved[1::2]):
                self.assertEqual(lidar.classify_scan_cluster(variant,self.cfg['lidar'])['kind'],'flat_board_candidate')

    def test_smooth_curve_has_positive_segment_direction_evidence(self):
        points = [(.6-.1*math.cos(math.radians(-30+i*2)),.1*math.sin(math.radians(-30+i*2))) for i in range(31)]
        row = lidar.classify_scan_cluster(points,self.cfg['lidar'])
        self.assertEqual(row['kind'],'obstacle_candidate')
        self.assertEqual(row['reason'],'supported_curved_shape')

    def test_old_length_parameter_no_longer_controls_class(self):
        for length in (.01,.08,1):
            cfg = dict(self.cfg['lidar'],line_min_width=length)
            row = lidar.classify_scan_cluster(self.labelled_short_shapes()['B'],cfg)
            self.assertEqual(row['kind'],'flat_board_candidate')

    def test_shape_threshold_overlap_is_rejected(self):
        for changes in ({'obstacle_min_ratio':.01},{'line_max_bend_deg':30,'obstacle_min_bend_deg':20}):
            cfg = copy.deepcopy(self.cfg)
            cfg['lidar'].update(changes)
            with self.assertRaises(ValueError):
                validate_config(cfg)
