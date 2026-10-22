"""Regressions for queued directions and evidence behind a lidar stop; no ROS nodes."""
from __future__ import division
import json
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


class RouteObstacleRegressionTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg.update(wait_green=False,sign_ttl=0,intersection_wait_s=1.0)
        self.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def vote(self,label,start=1):
        if label == 'PARKING':
            votes = self.c.cfg['parking_sign_votes']
        elif label in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            votes = self.c.cfg['direction_sign_votes']
        else:
            votes = self.c.cfg['sign_votes']
        for i in range(votes):
            t = start+i*.1
            self.c.observe_sign(label,.99,t,t)

    def scan(self,ranges):
        return Scan(ranges,-math.pi,math.pi/180,.05,6,self.c.pose,self.c.cfg['lidar'],1)

    def test_right_seen_during_left_needs_new_votes_after_handoff(self):
        c = self.c
        c.start_follow(intersection_path(c.pose,'LEFT',c.cfg),'LEFT',0)
        self.vote('RIGHT')
        self.assertEqual(c.action,'LEFT')
        self.assertIsNone(c.pending)
        self.assertIsNone(c.next_direction)
        c.resume_lane()
        self.assertIsNone(c.pending)
        self.vote('RIGHT',1.5)
        self.assertEqual(c.pending,'RIGHT')
        self.assertIsNone(c.next_direction)
        self.assertIsNone(c.dispatch(2))
        # A real blue junction needs the same three-frame marker evidence as
        # the camera path; a marker-only fixture must not start a turn.
        command = None
        for stamp in (2.0,2.1,2.2):
            c.observe_ground(dict(source='front',part='markers',
                markers=[dict(kind='junction',x=.3,y=0.,length=.6)],
                blue_lines=[dict(x=.3,y=0.,yaw=0.,length=.6)],slots=[]),stamp)
            command = c.dispatch(stamp)
        self.assertEqual(command,(0,0.0))
        self.assertEqual(c.action,'RIGHT')

    def test_wait_and_reacquire_can_queue_a_fresh_direction(self):
        for state in ('INTERSECTION_WAIT','REACQUIRE'):
            self.c.state,self.c.action = state,'LEFT'
            self.c.next_direction = None
            if state == 'REACQUIRE':
                # The preceding board must leave the image before a second
                # RIGHT can represent a distinct instruction.
                self.c.observe_sign('',0.,3.,3.)
            self.vote('RIGHT',2 if state == 'INTERSECTION_WAIT' else 4)
            self.assertEqual(self.c.next_direction,'RIGHT')
            self.assertEqual(self.c.state,state)
            self.assertEqual(self.c.action,'LEFT')

    def test_one_next_route_sign_is_cached_but_red_still_stops(self):
        c = self.c
        c.state,c.action = 'MANEUVER','LEFT'
        self.vote('LEFT')
        self.assertIsNone(c.next_direction)
        self.vote('RIGHT',2)
        self.vote('STRAIGHT',3)
        self.assertEqual(c.next_direction,'RIGHT')
        self.vote('RED',4)
        self.assertTrue(c.red)
        self.assertEqual(c.next_direction,'RIGHT')

    def test_sign_diagnostics_distinguish_low_confidence_votes_and_queue(self):
        c = self.c
        c.observe_sign('RIGHT',.4,1,1)
        self.assertEqual(c.sign_info['decision'],'low_confidence_or_unknown')
        c.state,c.action = 'MANEUVER','LEFT'
        c.observe_sign('RIGHT',.99,2,2)
        self.assertEqual(c.sign_info['decision'],'next_direction_voting')
        c.observe_sign('RIGHT',.99,2.1,2.1)
        self.assertEqual(c.sign_info['decision'],'stored_next_direction')
        self.assertEqual(c.sign_info['votes'],2)
        self.assertEqual(c.sign_info['confidence'],.99)
        json.dumps(c.sign_info,allow_nan=False)
        c.observe_sign('RIGHT',.99,2.2,2.2)
        self.assertEqual(c.sign_info['decision'],'next_direction_occupied')

    def test_out_of_range_sign_confidence_cannot_arm_a_direction(self):
        for i in range(3):
            self.c.observe_sign('RIGHT',1.2,1+i*.1,1+i*.1)
        self.assertIsNone(self.c.pending)
        self.c.observe_sign('RIGHT',float('nan'),2,2)
        self.assertIsNone(self.c.pending)
        json.dumps(self.c.sign_info,allow_nan=False)

    def test_uturn_rejects_below_090_but_parking_keeps_global_threshold(self):
        c = self.c
        for i in range(c.cfg['direction_sign_votes']):
            stamp = 1.6+i*.1
            c.observe_sign('UTURN',.599,stamp,stamp)
        self.assertIsNone(c.pending)
        self.assertEqual(c.sign_count,0)
        for i in range(c.cfg['direction_sign_votes']):
            stamp = 2+i*.1
            c.observe_sign('UTURN',.95,stamp,stamp)
        self.assertEqual(c.pending,'UTURN')
        c.pending = None
        for i in range(c.cfg['parking_sign_votes']):
            stamp = 3+i*.1
            c.observe_sign('PARKING',.81,stamp,stamp)
        self.assertEqual(c.pending,'PARKING')

    def test_queued_uturn_rejects_below_090(self):
        c = self.c
        c.state,c.action = 'MANEUVER','LEFT'
        for i in range(c.cfg['direction_sign_votes']):
            stamp = 1.6+i*.1
            c.observe_sign('UTURN',.599,stamp,stamp)
        self.assertIsNone(c.next_direction)
        for i in range(c.cfg['direction_sign_votes']):
            stamp = 2+i*.1
            c.observe_sign('UTURN',.95,stamp,stamp)
        self.assertEqual(c.next_direction,'UTURN')

    def test_known_echo_still_stops_and_reports_support_not_fake_probability(self):
        ranges = [float('inf')]*360
        ranges[180] = .5
        self.c.scan = self.scan(ranges)
        self.assertEqual(self.c.checked_command((20,0),1,False),(0,0.0))
        self.assertEqual(self.c.reason,'lidar_obstacle_in_sweep')
        report = self.c.obstacle_check
        self.assertEqual(report['kind'],'obstacle')
        self.assertEqual(report['support_points'],1)
        self.assertEqual(report['support_radius_m'],.06)
        self.assertNotIn('confidence',report)
        json.dumps(report,allow_nan=False)

    def test_invalid_scan_still_stops_with_separate_unknown_reason(self):
        self.c.scan = self.scan([float('nan')]*360)
        self.assertEqual(self.c.checked_command((20,0),1,False),(0,0.0))
        self.assertEqual(self.c.reason,'scan_missing_or_stale')
        self.assertEqual(self.c.obstacle_check['kind'],'unknown')
        self.assertEqual(self.c.obstacle_check['valid_rays'],0)
        json.dumps(self.c.obstacle_check,allow_nan=False)

    def test_clear_scan_resumes_and_clears_old_stop_details(self):
        self.c.scan = self.scan([float('nan')]*360)
        self.c.checked_command((20,0),1,False)
        self.c.scan = self.scan([float('inf')]*360)
        self.assertEqual(self.c.checked_command((20,0),1,False),(20,0))
        self.assertEqual(self.c.obstacle_check['kind'],'clear')
        self.assertNotIn('point_local',self.c.obstacle_check)

    def test_occlusion_has_different_evidence_from_invalid_ray(self):
        ranges = [float('inf')]*360
        ranges[180] = .7
        scan = self.scan(ranges)
        evidence = scan.evidence((1,0))
        self.assertEqual(evidence['classification'],'UNKNOWN')
        self.assertEqual(evidence['cause'],'occluded')
        self.assertAlmostEqual(evidence['range_m'],.7)
        self.assertEqual(scan.evidence((.7,0))['cause'],'echo_at_query')
        self.assertEqual(scan.evidence((.5,0))['cause'],'clear_ray')


if __name__ == '__main__':
    unittest.main()
