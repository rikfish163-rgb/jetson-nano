"""P1 terminal-line identity and front-axle approach, no actuator output."""
from __future__ import division
import os
import sys
import unittest
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from robot.parallel_parking.reference_vision import terminal_lines,ReferenceVision,paint_mask
from robot.parallel_parking.reference_core import ReferenceRun
from robot.parallel_parking.open_loop_core import make_slot_plan


class TerminalTests(unittest.TestCase):
    def test_bright_floor_does_not_become_continuing_white_rail(self):
        floor=np.full((120,160,3),190,np.uint8)
        self.assertFalse(paint_mask(floor,dict(v_min=165,s_max=75),200.).any())
        cv2.line(floor,(80,10),(80,110),(255,255,255),5)
        mask=paint_mask(floor,dict(v_min=165,s_max=75),200.)
        self.assertGreater(mask[20:100,78:83].mean(),200)
        self.assertFalse(mask[:,10:60].any())

    def scene(self, continued=False, clipped=False, wrong_width=False):
        metric = lambda u, v: ((600-v)/400., (600-u)/400.)
        mask = np.zeros((600,1200), np.uint8)
        valid = np.ones_like(mask)
        def pixel(x,y): return (int(600-y*400),int(600-x*400))
        lines = [[(.95,-.22),(.95,-.58 if not wrong_width else -.85)],
                 [(.25,-.58),(.95 if not continued else 1.3,-.58)],
                 [(.25,-.22),(.25,-.58)],[(.65,-.22),(1.3,-.22)]]
        for a,b in lines: cv2.line(mask,pixel(*a),pixel(*b),255,5)
        if clipped: valid[:220] = 0
        return terminal_lines(lines,mask,valid,metric,400.)

    def test_p1_end_requires_short_boundary_back_rail_and_free_visible_continuation(self):
        lines = self.scene()
        self.assertEqual(len(lines),1)
        self.assertAlmostEqual(lines[0][0][0],.95)

    def test_shared_divider_is_not_p1(self): self.assertEqual(self.scene(continued=True),[])
    def test_clipped_view_cannot_prove_end(self): self.assertEqual(self.scene(clipped=True),[])
    def test_wrong_width_is_not_p1(self): self.assertEqual(self.scene(wrong_width=True),[])

    def test_visible_terminal_with_wrong_scale_reports_width_without_locking(self):
        metric=lambda u,v:((600-v)/400.,(600-u)/400.)
        mask=np.zeros((600,1200),np.uint8);valid=np.ones_like(mask)
        lines=[[(.95,-.22),(.95,-.74)],[(.65,-.74),(.95,-.74)],
               [(.65,-.22),(1.3,-.22)]]
        for a,b in lines:
            pts=[(int(600-y*400),int(600-x*400)) for x,y in (a,b)]
            cv2.line(mask,pts[0],pts[1],255,5)
        diagnostic={}
        self.assertEqual(terminal_lines(lines,mask,valid,metric,400.,diagnostic),[])
        self.assertAlmostEqual(diagnostic['width_mismatches'][0]['width_m'],.52)

    def test_partial_end_cannot_hide_wrong_full_width(self):
        metric=lambda u,v:((600-v)/400.,(600-u)/400.)
        mask=np.zeros((600,1200),np.uint8);valid=np.ones_like(mask)
        lines=[[(.95,-.35),(.95,-.74)],[(.65,-.74),(.95,-.74)],
               [(.65,-.22),(1.3,-.22)]]
        for a,b in [[(.95,-.22),(.95,-.74)]]+lines[1:]:
            pts=[(int(600-y*400),int(600-x*400)) for x,y in (a,b)]
            cv2.line(mask,pts[0],pts[1],255,5)
        diagnostic={}
        self.assertEqual(terminal_lines(lines,mask,valid,metric,400.,diagnostic),[])
        self.assertAlmostEqual(diagnostic['width_mismatches'][0]['width_m'],.52)

    def test_thick_paint_uses_centreline_instead_of_slanted_hough_edge(self):
        metric=lambda u,v:((600-v)/400.,(600-u)/400.)
        mask=np.zeros((600,1200),np.uint8);valid=np.ones_like(mask)
        actual=[[(.95,-.22),(.95,-.58)],[(.65,-.58),(.95,-.58)],
                [(.65,-.22),(1.3,-.22)]]
        for a,b in actual:
            pts=[(int(600-y*400),int(600-x*400)) for x,y in (a,b)]
            cv2.line(mask,pts[0],pts[1],255,5)
        segments=[[ (.93,-.22),(.97,-.58)]]+actual[1:]
        found=terminal_lines(segments,mask,valid,metric,400.)
        self.assertEqual(len(found),1)
        for x,y in found[0]:self.assertAlmostEqual(x,.95,delta=.003)

    def test_road_edge_continuation_does_not_hide_outer_bay_end(self):
        metric=lambda u,v:((600-v)/400.,(600-u)/400.)
        mask=np.zeros((600,1200),np.uint8);valid=np.ones_like(mask)
        lines=[[(.95,-.22),(.95,-.58)],[(.65,-.58),(.95,-.58)],
               [(.65,-.22),(1.3,-.22)]]
        for a,b in lines:
            pts=[(int(600-y*400),int(600-x*400)) for x,y in (a,b)]
            cv2.line(mask,pts[0],pts[1],255,5)
        self.assertEqual(len(terminal_lines(lines,mask,valid,metric,400.)),1)

    def test_overlong_hough_end_requires_separate_mouth_intersection_to_trim(self):
        metric=lambda u,v:((600-v)/400.,(600-u)/400.)
        mask=np.zeros((600,1200),np.uint8);valid=np.ones_like(mask)
        lines=[[(.95,-.13),(.95,-.58)],[(.65,-.58),(.95,-.58)],
               [(.65,-.22),(1.3,-.22)]]
        for a,b in lines:
            pts=[(int(600-y*400),int(600-x*400)) for x,y in (a,b)]
            cv2.line(mask,pts[0],pts[1],255,5)
        found=terminal_lines(lines,mask,valid,metric,400.)
        self.assertEqual(len(found),1)
        self.assertAlmostEqual(found[0][0][1],-.22)
        self.assertEqual(terminal_lines(lines[:2],mask,valid,metric,400.),[])


class FlowFreeReferenceTests(unittest.TestCase):
    def test_blind_distance_uses_measured_speed_points_for_each_actual_command(self):
        task=self.task(forward_speed_table=[[15,.13],[20,.18],[25,.23],[30,.28]])
        task.set_command(15,0.)
        task.set_command(20,2.)
        task.set_command(0,3.)
        task.advance(4.)
        self.assertAlmostEqual(task.distance,.13*2+.18)

    def task(self,**options):
        return ReferenceRun(make_slot_plan('P1',countdown_seconds=2),**options)

    def observation(self,t,line=None):
        return dict(source='p1_reference_camera',frame='base_link',stamp=1000+t,
            wheelbase_m=.26,p1_lines=[] if line is None else [[(line,-.22),(line,-.58)]])

    def update(self,task,t,line=None):
        task.observe(self.observation(t,line),t,1000+t)
        return task.tick(t)

    def searching(self,**options):
        task=self.task(**options)
        for i in range(41):self.update(task,i*.05)
        return task

    def test_default_search_speed_is_15(self):
        task=self.searching()
        self.assertEqual(self.update(task,2.05),(15,0))

    def test_two_consecutive_images_lock_without_motion_fields(self):
        task=self.searching()
        self.update(task,2.05,.9)
        self.assertIsNone(task.line)
        self.update(task,2.10,.894)
        self.assertIsNotNone(task.line)
        self.assertEqual(task.reason,'APPROACH_P1_LINE')

    def test_missing_candidate_breaks_two_frame_confirmation(self):
        task=self.searching()
        self.update(task,2.05,.9)
        self.update(task,2.10)
        self.update(task,2.15,.888)
        self.assertIsNone(task.line)
        self.update(task,2.20,.882)
        self.assertIsNotNone(task.line)

    def test_duplicate_source_image_does_not_confirm(self):
        task=self.searching()
        self.update(task,2.05,.9)
        task.observe(self.observation(2.05,.9),2.1,1002.1)
        task.tick(2.1)
        self.assertIsNone(task.line)

    def test_search_keeps_following_when_front_line_is_absent(self):
        task=self.searching(seek_timeout_s=1.)
        for i in range(41,241):self.assertEqual(self.update(task,i*.05),(15,0))
        self.assertFalse(task.finished)

    def test_blind_approach_stops_and_completes_without_flow(self):
        task=self.searching(reference_only=True)
        self.update(task,2.05,.51)
        self.update(task,2.10,.504)
        moving=False
        for i in range(43,161):
            command=self.update(task,i*.05)
            moving=moving or command[0]>0
            if task.finished:break
        self.assertTrue(moving)
        self.assertTrue(task.finished)
        self.assertEqual(task.reason,'COMPLETE')
        self.assertEqual(command,(0,0))

    def test_actual_lane_stop_does_not_consume_blind_distance(self):
        task=self.searching()
        self.update(task,2.05,.9)
        self.update(task,2.10,.894)
        before=task.gap
        for i in range(43,53):
            t=i*.05
            task.set_command(0,task.last)
            self.update(task,t)
            task.set_command(0,t)
        self.assertAlmostEqual(task.gap,before)

    def test_visible_line_corrects_timed_estimate(self):
        task=self.searching()
        self.update(task,2.05,.9)
        self.update(task,2.10,.894)
        self.update(task,2.15,.85)
        self.assertAlmostEqual(task.gap,.59)
        self.assertEqual(task.gap_source,'line_image')

    def test_distant_divider_cannot_replace_locked_line(self):
        task=self.searching()
        self.update(task,2.05,.9)
        self.update(task,2.10,.894)
        self.update(task,2.15,1.5)
        self.assertLess(task.gap,.64)
        self.assertEqual(task.gap_source,'timed_last_seen_line')

    def test_legacy_flow_failure_cannot_block_two_image_confirmation(self):
        task=self.searching()
        for t in (2.05,2.10):
            data=self.observation(t,.9)
            data.update(motion_valid=False,pose=[float('nan'),0.,0.])
            task.observe(data,t,1000+t);task.tick(t)
        self.assertIsNotNone(task.line)

    def test_camera_missing_and_stale_are_still_detected(self):
        task=self.task()
        for i in range(210):self.assertEqual(task.tick(i*.05),(0,0))
        self.assertEqual(task.reason,'REFERENCE_CAMERA_TIMEOUT')
        task=self.searching()
        self.update(task,2.05,.9);self.update(task,2.10,.894)
        for i in range(43,53):task.tick(i*.05)
        self.assertEqual(task.reason,'REFERENCE_CAMERA_STALE')

    def test_locked_approach_timeout_is_retained(self):
        task=self.searching(seek_timeout_s=1.)
        self.update(task,2.05,.9);self.update(task,2.10,.894)
        for i in range(43,65):self.update(task,i*.05)
        self.assertEqual(task.reason,'P1_SEARCH_LIMIT')

    def test_near_visible_overshoot_exits(self):
        task=self.searching()
        self.update(task,2.05,.30);self.update(task,2.10,.294)
        self.assertEqual(self.update(task,2.15,.23),(0,0))
        self.assertEqual(task.reason,'FRONT_AXLE_PAST_P1_LINE')

    def test_invalid_image_observation_is_rejected(self):
        for change in (dict(source='other'),dict(stamp=2000),dict(frame='map'),
                       dict(wheelbase_m=.4),dict(p1_lines='bad'),
                       dict(p1_lines=[[(float('nan'),-.22),(.9,-.58)]])):
            task=self.task();data=self.observation(0,.9);data.update(change)
            with self.assertRaises(ValueError):task.observe(data,0,1000)

    def test_bad_control_clock_exits(self):
        for t in (-1.,float('nan'),float('inf'),.3):
            task=self.task();task.tick(0.)
            self.assertEqual(task.tick(t),(0,0))
            self.assertEqual(task.reason,'REFERENCE_CONTROL_LOOP_GAP')

    def test_ambiguous_candidates_break_confirmation(self):
        task=self.searching();self.update(task,2.05,.9)
        data=self.observation(2.10,.894)
        data['p1_lines'].append([(.8,-.22),(.8,-.58)])
        task.observe(data,2.10,1002.10);task.tick(2.10)
        self.update(task,2.15,.888)
        self.assertIsNone(task.line)

    def test_camera_interruption_breaks_two_image_confirmation(self):
        task=self.searching();self.update(task,2.05,.9)
        for i in range(42,51):task.tick(i*.05)
        self.assertEqual(task.reason,'WAIT_REFERENCE_CAMERA')
        self.update(task,2.55,.84)
        self.assertIsNone(task.line)
        self.update(task,2.60,.834)
        self.assertIsNotNone(task.line)

    def test_delayed_image_uses_recent_command_distance(self):
        task=self.searching()
        self.update(task,2.05,.9);self.update(task,2.10,.894)
        for i in range(43,49):task.tick(i*.05)
        task.observe(self.observation(2.15,.85),2.45,1002.45)
        self.assertEqual(task.tick(2.45),(15,0))
        self.assertAlmostEqual(task.gap,.85-.26-.12*.30)


class ProjectionTests(unittest.TestCase):
    def test_parking_projection_loads_without_changing_base_camera(self):
        from robot.parallel_parking.reference_vision import load_reference_config
        root=os.path.dirname(os.path.dirname(__file__))
        base=os.path.join(root,'config/competition.yaml')
        parking=os.path.join(root,'parallel_parking/reference_camera.yaml')
        original=load_reference_config(base)
        cfg=load_reference_config(base,parking)
        self.assertEqual(cfg['front_camera']['K'],original['front_camera']['K'])
        self.assertEqual(cfg['front_camera']['D'],original['front_camera']['D'])
        self.assertNotEqual(cfg['front_camera']['H'],original['front_camera']['H'])
        self.assertEqual(load_reference_config(base),original)
        points=np.float32([[395.1447448730469,165.22877502441406],
                           [455.84765625,164.8170928955078],
                           [403.78472900390625,170.44825744628906],
                           [483.42034912109375,171.7225799560547]])
        c=cfg['front_camera']
        pixels=cv2.perspectiveTransform(points.reshape(-1,1,2),np.array(c['H']).reshape(3,3)).reshape(-1,2)
        ground=np.column_stack(((c['origin_v']-pixels[:,1])/c['pixels_per_m'],
                                (c['origin_u']-pixels[:,0])/c['pixels_per_m']))
        np.testing.assert_allclose(ground,[[1.60,-.29],[1.60,-.65],[1.33,-.29],[1.33,-.65]],atol=1e-5)

    def test_front_line_beyond_old_canvas_is_in_search_canvas(self):
        import yaml
        path=os.path.join(os.path.dirname(__file__),'../config/competition.yaml')
        with open(path) as stream:cfg=yaml.safe_load(stream)
        vision=ReferenceVision(cfg)
        c=vision.camera
        self.assertGreater(c['origin_v']/c['pixels_per_m'],2.5)
        v=c['origin_v']-2.*c['pixels_per_m']
        self.assertGreater(v,0)
        self.assertAlmostEqual(vision.detector.metric(c['origin_u']-360,v)[0],2.)

    def test_lower_resolution_preserves_metric_projection_and_input_config(self):
        import yaml,copy
        path=os.path.join(os.path.dirname(__file__),'../config/competition.yaml')
        with open(path) as stream:cfg=yaml.safe_load(stream)
        original=copy.deepcopy(cfg)
        vision=ReferenceVision(cfg)
        self.assertEqual(cfg,original)
        c=cfg['front_camera']
        for u,v in ((100.,100.),(300.,450.),(500.,250.)):
            expected=((c['origin_v']-v)/c['pixels_per_m'],(c['origin_u']-u)/c['pixels_per_m'])
            shift=vision.detector.c['origin_v']-c['origin_v']*.5
            actual=vision.detector.metric(u*.5,v*.5+shift)
            np.testing.assert_allclose(actual,expected,atol=1e-10)


class FlowFreeVisionTests(unittest.TestCase):
    def setUp(self):
        from robot.parallel_parking import reference_vision as module
        self.module=module
        self.original=module.terminal_lines
        module.terminal_lines=lambda *args:[[(.9,-.22),(.9,-.58)]]
        root=os.path.dirname(os.path.dirname(__file__))
        cfg=module.load_reference_config(os.path.join(root,'config/competition.yaml'))
        self.vision=module.ReferenceVision(cfg)
        self.frame=np.zeros((360,640,3),np.uint8)

    def tearDown(self):self.module.terminal_lines=self.original

    def test_first_image_detects_without_waiting_for_floor_stability(self):
        result=self.vision.observe(self.frame,1.)[0]
        self.assertEqual(result['reason'],'p1_terminal_seen')
        self.assertEqual(len(result['p1_lines']),1)
        self.assertEqual(result['frame'],'base_link')
        self.assertNotIn('pose',result)
        self.assertNotIn('motion_valid',result)

    def test_duplicate_image_is_skipped(self):
        self.vision.observe(self.frame,1.)
        self.assertIsNone(self.vision.observe(self.frame,1.))

    def test_bad_image_does_not_latch_a_tracking_failure(self):
        with self.assertRaises(ValueError):self.vision.observe(self.frame[:100],1.)
        self.assertEqual(self.vision.observe(self.frame,1.1)[0]['reason'],'p1_terminal_seen')


if __name__=='__main__': unittest.main()
