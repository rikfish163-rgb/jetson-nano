"""Front white-line parking regressions. No ROS master or motor publisher."""
from __future__ import division
import copy
import json
import math
import os
import sys
import unittest
import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.camera.vision import GroundDetector
from robot.common.contracts import ground
from robot.common.contracts import encode_command
from robot.common.contracts import validate_config
from robot.parking.entry import bay_edges
from robot.parking.entry import shared_divider
from robot.parking.entry import WhiteParking
from robot.master.controller import Controller
from robot.common.geometry import bicycle
from robot.common.geometry import inside_slot
from robot.common.geometry import world

with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
    CONFIG = yaml.safe_load(stream)


class ParkingGeometryTest(unittest.TestCase):
    def test_extracts_metric_segments_from_white_mask(self):
        d = GroundDetector(CONFIG)
        mask = np.zeros((600, 480), np.uint8)
        cv2.line(mask, (164, 200), (164, 370), 255, 4)
        cv2.line(mask, (316, 200), (316, 370), 255, 4)
        lines = d.parking_lines(mask)
        self.assertGreaterEqual(len(lines), 2)
        self.assertTrue(any(abs(line[0][1] - .19) < .02 for line in lines))

    def test_parallel_sides_without_bottom_still_support_alignment(self):
        slot = dict(pose=(.5, 0, 0), length=.45, width=.38)
        lines = [[(.3, .19), (.7, .19)], [(.3, -.19), (.7, -.19)]]
        edges = bay_edges(slot, lines, CONFIG)
        self.assertIsNotNone(edges['left'])
        self.assertIsNotNone(edges['right'])
        self.assertIsNone(edges['bottom'])

    def test_bottom_must_be_far_end_not_mouth_or_another_bay(self):
        slot = dict(pose=(.5, 0, 0), length=.45, width=.38)
        lines = [[(.275, -.19), (.275, .19)], [(1.0, -.19), (1.0, .19)]]
        self.assertIsNone(bay_edges(slot, lines, CONFIG)['bottom'])
        lines.append([(.725, -.19), (.725, .19)])
        self.assertIsNotNone(bay_edges(slot, lines, CONFIG)['bottom'])

    def test_far_bay_requires_observed_shared_white_divider(self):
        near = dict(pose=(.5, -.55, -math.pi/2), length=.45, width=.38)
        far = dict(pose=(.88, -.55, -math.pi/2), length=.45, width=.38)
        self.assertIsNone(shared_divider(near, far, [], CONFIG))
        line = [( .69, -.32), (.69, -.78)]
        self.assertIsNotNone(shared_divider(near, far, [line], CONFIG))
        self.assertIsNone(shared_divider(near, far, [[(.3,-.32),(.3,-.78)]], CONFIG))

    def test_line_message_rejects_invalid_endpoints(self):
        data = dict(stamp=1, source='front', part='parking_lines', frame='base_link',
                    markers=[], slots=[], lines=[[[.5,.2],[.8,.2]]])
        self.assertEqual(ground(json.dumps(data), 1, .5)[0]['lines'], data['lines'])
        for lines in ([[[float('nan'),0],[1,0]]], [[[9,0],[1,0]]], [[[.5,0]]], 'bad'):
            data['lines'] = lines
            with self.assertRaises(ValueError):
                ground(json.dumps(data), 1, .5)

    def test_wide_canvas_keeps_metric_origin_for_both_sides(self):
        d = GroundDetector(CONFIG)
        for side in (-1,1):
            mask = np.zeros((600,1200),np.uint8)
            def pixel(x,y):
                return int(round(600-400*y)),int(round(600-400*x))
            for x in (.31,.69,1.07):
                cv2.line(mask,pixel(x,side*.325),pixel(x,side*.775),255,4)
            cv2.line(mask,pixel(.31,side*.775),pixel(1.07,side*.775),255,4)
            slots = [s for s in d.detect_slots(mask,u_offset=360) if s['kind']=='perpendicular']
            self.assertEqual(len(slots),2)
            for slot in slots:
                self.assertAlmostEqual(slot['y'],side*.55,delta=.025)


class ClearScan(object):
    stamp = 10.0
    valid_rays = 360
    shape_filter = False
    obstacles = []
    def coverage(self, points):
        return 1.0
    def classify(self, point):
        return 'FREE'
    def evidence(self, point):
        return dict(classification='OCCUPIED',cause='test_fixture')


class WhiteParkingTest(unittest.TestCase):
    def core(self, side=-1):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(parking_mode='forward_white', parking_slot='AUTO', wait_green=False,
                   parking_observe_s=1.0, steering_command_scale_rad=.1)
        c = Controller(cfg)
        self.addCleanup(c.close)
        c.scan = ClearScan()
        c.scan.obstacles = []
        c.pending, c.pending_at = 'PARKING', 9.0
        c.observe_ground(dict(source='front',part='markers',slots=[],
                             markers=[dict(kind='junction',x=.36,y=0)]),10.0)
        c.dispatch(10.0)
        self.bays = [dict(kind='perpendicular',x=x,y=side*.55,yaw=math.pi/2)
                     for x in (.50,.88)]
        self.side = side
        return c

    def observations(self, c, now, lines=None):
        c.scan.stamp = now
        if lines is None:
            lines = [[(.69,self.side*.32),(.69,self.side*.78)]]
        c.observe_ground(dict(source='front',part='slots',markers=[],slots=self.bays),now)
        c.observe_ground(dict(source='front',part='parking_lines',markers=[],slots=[],lines=lines),now)

    def selected(self, side=-1, far=False):
        c = self.core(side)
        if far:
            c.scan.obstacles = [(.5,side*.55)]
        self.observations(c,10.5)
        self.assertEqual(c.tick(10.5),(0,0))
        self.observations(c,11.0)
        c.tick(11.0)
        return c

    def test_near_starts_forward_full_lock_on_correct_side(self):
        for side in (-1,1):
            c = self.selected(side)
            self.assertIsNotNone(c.parking_entry)
            self.assertEqual(c.parking_entry.phase,'LOCK')
            cmd = c.tick(11.05)
            self.assertGreater(cmd[0],0)
            self.assertEqual(encode_command(cmd[0],cmd[1],c.cfg,0)['steering_raw'],side*22)
            self.assertIsNone(c.future)

    def test_far_waits_for_actual_divider_and_does_not_lock_at_blue(self):
        c = self.core()
        c.scan.obstacles = [(.5,-.55)]
        self.observations(c,10.5,[])
        self.observations(c,11.0,[])
        self.assertEqual(c.tick(11.0),(0,0))
        self.assertIsNone(c.parking_entry)
        self.observations(c,11.1)
        c.tick(11.1)
        self.assertEqual(c.parking_entry.phase,'APPROACH_DIVIDER')

    def test_unknown_and_stale_scan_never_start_entry(self):
        c = self.core()
        c.scan.coverage = lambda p: .1
        self.observations(c,10.5)
        self.observations(c,11.0)
        self.assertEqual(c.tick(11.0),(0,0))
        self.assertIsNone(c.parking_entry)
        c.scan.stamp = 1.0
        self.assertEqual(c.tick(11.0),(0,0))

    def test_one_side_cannot_finish_alignment(self):
        c = self.selected()
        e = c.parking_entry
        e.phase = 'ALIGN'
        c.set_pose((.5,-.1,-math.pi/2),11.1)
        line = [world(e.slot['pose'],(x,.19)) for x in (-.2,.2)]
        e.observe([line],11.1, pose=c.pose)
        self.assertEqual(e.command(11.1, pose=c.pose),(0,0))
        self.assertEqual(e.phase,'ALIGN')

    def test_bottom_distance_is_from_bumper_and_advance_is_bounded(self):
        c = self.selected()
        e = c.parking_entry
        e.phase = 'ALIGN'
        c.set_pose((.5,-.25,-math.pi/2),11.1)
        lines = [[world(e.slot['pose'],(x,y)) for x in (-.225,.225)] for y in (-.19,.19)]
        lines.append([world(e.slot['pose'],(.225,y)) for y in (-.19,.19)])
        for t in (11.1,11.3,11.7):
            e.observe(lines,t, pose=c.pose)
            e.command(t, pose=c.pose)
        self.assertEqual(e.phase,'ADVANCE')
        self.assertAlmostEqual(e.debug['remaining_m'],.155,places=3)
        # Front view becomes occluded: measured target is retained for a bounded move.
        c.set_pose((.5,-.405,-math.pi/2),12.0)
        self.assertEqual(e.command(12.0, pose=c.pose),(0,0))
        self.assertEqual(e.phase,'SETTLE')

    def test_divider_reached_transitions_to_lock_without_reselecting(self):
        c = self.selected(far=True)
        e = c.parking_entry
        self.assertEqual(e.phase,'APPROACH_DIVIDER')
        c.set_pose((.36,0,0),11.1)
        e.observe([[(.69,-.32),(.69,-.78)]],11.1, pose=c.pose)
        self.assertEqual(e.command(11.1, pose=c.pose),(0,0))
        self.assertEqual(e.phase,'LOCK')
        self.assertGreater(e.command(11.15, pose=c.pose)[0],0)
        self.assertEqual(e.slot['relative_bay'],'far')

    def test_alignment_steers_toward_center_and_cannot_confirm_duplicate_frame(self):
        c = self.selected()
        e = c.parking_entry
        e.phase = 'ALIGN'
        c.set_pose((.46,-.2,-math.pi/2),11.1)
        lines = [[world(e.slot['pose'],(x,y)) for x in (-.225,.225)] for y in (-.19,.19)]
        e.observe(lines,11.1, pose=c.pose)
        self.assertGreater(e.command(11.1, pose=c.pose)[1],0)  # left toward the center at x=.50
        c.set_pose((.5,-.2,-math.pi/2),11.2)
        e.command(11.2, pose=c.pose)
        e.command(11.9, pose=c.pose)
        self.assertEqual(e.phase,'ALIGN')

    def test_heading_from_other_bay_does_not_end_lock(self):
        c = self.selected()
        e = c.parking_entry
        e.observe([[(2,0),(2,.4)],[(2.38,0),(2.38,.4)]],11.1, pose=c.pose)
        e.command(11.1, pose=c.pose)
        self.assertEqual(e.phase,'LOCK')

    def test_lock_transitions_using_observed_sides_after_turn_progress(self):
        c = self.selected()
        e = c.parking_entry
        c.set_pose((.4,-.15,-math.pi/3),11.1)
        lines = [[world(e.slot['pose'],(x,y)) for x in (-.225,.225)] for y in (-.19,.19)]
        e.observe(lines,11.1, pose=c.pose)
        self.assertGreater(e.command(11.1, pose=c.pose)[0],0)
        self.assertEqual(e.phase,'ALIGN')

    def test_full_lock_sweep_uses_physical_angle_and_raw_lidar(self):
        c = self.selected()
        c.scan.obstacles = [(.4,-.1)]
        c.scan.shape_filter = True  # parking must not ignore raw returns
        self.assertEqual(c.tick(11.05),(0,0))
        self.assertEqual(c.obstacle_check['kind'],'obstacle')

    def test_final_advance_stops_if_side_reference_heading_changes(self):
        c = self.selected()
        e = c.parking_entry
        e.phase,e.advance_start,e.advance_at = 'ADVANCE',(.5,-.25,-math.pi/2),11.0
        e.final_heading,e.final_center = -math.pi/2,(.5,-.55)
        e.bottom = [( .31,-.775),(.69,-.775)]
        c.set_pose((.5,-.25,-math.pi/2+.22),11.1)
        e.observe([],11.1, pose=c.pose)
        self.assertEqual(e.command(11.1, pose=c.pose),(0,0))
        self.assertEqual(e.reason,'parking_final_alignment_lost')
        self.assertFalse(hasattr(e, 'core'))

    def test_vision_loss_in_turn_and_red_stop_are_not_bypassed(self):
        c = self.selected()
        self.assertEqual(c.parking_entry.command(14.0, pose=c.pose),(0,0))
        c.red = True
        self.assertEqual(c.tick(11.1),(0,0))

    def test_wrong_direction_feedback_stops(self):
        c = self.selected()
        c.set_pose((0,0,.25),11.1)
        self.assertEqual(c.tick(11.1),(0,0))
        self.assertEqual(c.reason,'parking_wrong_turn_direction')
        self.assertEqual(c.state,'FAULT')

    def test_forward_mode_requires_auto_and_lidar(self):
        c = self.core()
        validate_config(c.cfg)
        for key,value in (('parking_slot','P4'),('lidar_enabled',False),
                          ('parking_bottom_clearance_m',-.01),('parking_mode','typo')):
            cfg = dict(c.cfg)
            cfg[key] = value
            with self.assertRaises(ValueError):
                validate_config(cfg)

    def test_forward_entry_model_both_sides_and_bays_reaches_inside_terminal(self):
        for side in (-1,1):
            for far in (False,True):
                c = self.core(side)
                cfg = c.cfg
                radius = cfg['wheelbase']/math.tan(cfg['max_steer'])
                slot = dict(pose=(radius+(.38 if far else 0),side*.70,side*math.pi/2),
                            kind='perpendicular',length=.45,width=.38,id='fixture',
                            relative_bay='far' if far else 'near')
                divider = [[radius+.19,side*.475],[radius+.19,side*.925]] if far else None
                e = WhiteParking(c,slot,divider,11.0)
                c.parking_entry,c.slot,c.state,c.action = e,slot,'PARKING','PARKING'
                lines = [[world(slot['pose'],(x,y)) for x in (-.225,.225)] for y in (-.19,.19)]
                lines.append([world(slot['pose'],(.225,y)) for y in (-.19,.19)])
                if divider:
                    lines.append(divider)
                for i in range(701):
                    now = 11+i*.05
                    c.scan.stamp = now
                    e.observe(lines,now, pose=c.pose)
                    speed,steer = c.tick(now)
                    self.assertGreaterEqual(speed,0)  # this entire maneuver is forward
                    physical = steer/cfg['steering_command_scale_rad']*cfg['max_steer']
                    c.set_pose(bicycle(c.pose,speed*cfg['raw_to_mps']['forward']*.05,
                                       physical,cfg['wheelbase']),now)
                    if c.state in ('FAULT','FINISHED'):
                        break
                self.assertEqual(c.state,'FINISHED',(side,far,c.reason,e.debug))
                self.assertTrue(inside_slot(c.pose,slot,cfg))


if __name__ == '__main__':
    unittest.main()
