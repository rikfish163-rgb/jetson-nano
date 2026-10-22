#!/usr/bin/env python2
from __future__ import division
import math
import unittest
import numpy as np
import cv2
import parking_line_stop_test as common
from parking_line_stop_core import StopRun
from parking_reference_align_core import (front_line_gap,dual_midline,midpoint_pose,
                                          FixedTerminalLock,ReferenceAlignRun)
from parking_reference_align_test import arguments,make_follower,MidlineVision,P3LaneVision
from robot.parallel_parking.reference_vision import terminal_lines


def line(gap):
    return [[gap+.26,-.30],[gap+.26,-.66]]


def boundaries(offset=0.,slope=0.,curve=0.):
    x = np.linspace(.25,1.3,24)
    return dict(LEFT=[[float(v),float(.30+offset+slope*(v-.26)+curve*(v-.26)**2)] for v in x],
                RIGHT=[[float(v),float(-.30+offset+slope*(v-.26)+curve*(v-.26)**2)] for v in x])


class FakeFollower(object):
    def __init__(self,command=(16,7)):
        self.output = command
        self.observations = []
    def observe(self,observation,now):
        self.observations.append((observation,now))
    def command(self,now):
        return self.output
    def diagnostics(self,now):
        return dict(lane_age_s=0.,lane_reason='ok')


class GeometryTests(unittest.TestCase):
    def test_gap_uses_front_axle_and_is_endpoint_order_independent(self):
        self.assertAlmostEqual(.7,front_line_gap(line(.7)))
        self.assertAlmostEqual(.7,front_line_gap(list(reversed(line(.7)))))
        self.assertAlmostEqual(.8,front_line_gap(line(.7),.16))

    def test_gap_is_perpendicular_for_yawed_line(self):
        theta = math.radians(8)
        n = np.array([math.cos(theta),math.sin(theta)])
        tangent = np.array([-n[1],n[0]])
        center = np.array([.26,0.])+n*.8
        self.assertAlmostEqual(.8,front_line_gap([center-tangent*.5,center-tangent*.8]))

    def test_invalid_and_nontransverse_line_rejected(self):
        for value in ([[0,0],[0,0]],[[0,0],[1,0]],[[1,0],[float('nan'),1]]):
            with self.assertRaises(ValueError):front_line_gap(value)

    def test_midpoint_averages_actual_two_sides_and_pose_is_at_front_axle(self):
        middle = dual_midline(boundaries(.04,.1))
        self.assertTrue(middle['valid'])
        pose = midpoint_pose(middle)
        self.assertTrue(pose['valid'])
        self.assertAlmostEqual(.04,pose['lateral_m'],places=6)
        self.assertAlmostEqual(math.degrees(math.atan(.1)),pose['heading_deg'],places=6)

    def test_missing_side_is_not_inferred(self):
        b = boundaries()
        b['RIGHT'] = []
        self.assertFalse(dual_midline(b)['valid'])

    def test_width_and_observation_support_rejected(self):
        b = boundaries()
        b['LEFT'] = [[x,y+1.] for x,y in b['LEFT']]
        self.assertFalse(dual_midline(b)['valid'])
        b = boundaries(); b['LEFT'] = b['LEFT'][:3]
        self.assertFalse(dual_midline(b)['valid'])

    def test_bend_cannot_claim_straight_alignment(self):
        self.assertFalse(midpoint_pose(dual_midline(boundaries(curve=.5)))['valid'])


class VisionAdapterTests(unittest.TestCase):
    def test_measured_midpoint_replaces_lane_path_on_same_frame(self):
        original = P3LaneVision.observe
        def fake_observe(self,frame,stamp):
            return dict(boundaries=boundaries(.06),lane_observation=dict(stamp=stamp,
                        points=[[.4,.3]],confidence=.9)),np.zeros((600,480,3),np.uint8)
        vision = MidlineVision.__new__(MidlineVision)
        vision.pixel_origin = np.array([1.5,.6])
        vision.pixel_inverse = np.array([[0.,-400.],[-400.,0.]])
        P3LaneVision.observe = fake_observe
        try:
            result,debug = vision.observe(None,100.)
        finally:
            P3LaneVision.observe = original
        self.assertTrue(result['dual_midline']['valid'])
        self.assertEqual(result['dual_midline']['points'],result['lane_observation']['points'])
        self.assertEqual(100.,result['lane_observation']['stamp'])
        self.assertGreater(np.count_nonzero(debug),0)

    def terminal(self,outer_end):
        segments = [[[.25,-.30],[1.6,-.30]],[[.25,-.66],[outer_end,-.66]],
                    [[1.2,-.30],[1.2,-.66]]]
        mask = np.zeros((700,600),np.uint8)
        def pixel(point):return (int(300-point[1]*200),int(600-point[0]*200))
        for a,b in segments:cv2.line(mask,pixel(a),pixel(b),255,4)
        valid = np.full(mask.shape,255,np.uint8)
        metric = lambda u,v:((600-v)/200.,(300-u)/200.)
        return terminal_lines(segments,mask,valid,metric,200.)

    def test_actual_terminal_detector_accepts_row_end_and_rejects_shared_divider(self):
        self.assertEqual(1,len(self.terminal(1.2)))
        self.assertEqual([],self.terminal(1.6))


class LockTests(unittest.TestCase):
    def test_needs_three_fresh_identity_consistent_frames(self):
        lock = FixedTerminalLock()
        self.assertFalse(lock.observe([line(1.)],100.))
        self.assertFalse(lock.observe([line(1.)],100.))
        self.assertEqual(1,lock.votes)
        self.assertFalse(lock.observe([line(.99)],100.1))
        self.assertTrue(lock.observe([line(.98)],100.2))
        self.assertTrue(lock.locked)

    def test_missing_or_ambiguous_frames_reset_acquisition(self):
        lock = FixedTerminalLock()
        lock.observe([line(1.)],100.)
        lock.observe([],100.1)
        self.assertEqual(0,lock.votes)
        lock.observe([line(1.),line(2.)],100.2)
        self.assertEqual('reference_ambiguous',lock.reason)
        self.assertFalse(lock.locked)

    def test_locked_reference_never_switches_on_gap_jump(self):
        lock = FixedTerminalLock()
        for i in range(3):lock.observe([line(1.)],100.+i*.1)
        self.assertFalse(lock.observe([line(1.6)],100.3))
        self.assertEqual('reference_identity_jump',lock.reason)
        self.assertAlmostEqual(1.,lock.gap)
        self.assertTrue(lock.observe([line(.97)],100.4))

    def test_bad_geometry_does_not_crash_or_lock(self):
        lock = FixedTerminalLock()
        self.assertFalse(lock.observe([[[1,0],[2,0]]],100.))
        self.assertFalse(lock.locked)


class RunTests(unittest.TestCase):
    def make(self,start=True):
        task = ReferenceAlignRun(StopRun(speed=16,seek_seconds=15.,max_seconds=30.,camera_timeout=.8),FakeFollower())
        if start:task.start(0.)
        return task

    def feed(self,task,now,gap=None,offset=0.,slope=0.,midline=True):
        stamp = 100.+now
        task.observe_scene(dict(fixed_reference=dict(stamp=stamp,p1_lines=[] if gap is None else [line(gap)]),
                                dual_midline=dual_midline(boundaries(offset,slope)) if midline else {}))
        task.observe(0 if gap is None else 1,stamp,now,False)

    def lock(self,task,gap):
        for now in (0.,.1,.2):self.feed(task,now,gap)

    def test_countdown_and_observe_never_command_motion(self):
        task = self.make(False)
        self.lock(task,1.2)
        self.assertEqual((0,0),task.tick(.2,100.2))
        self.assertTrue(task.reference.locked)
        self.assertTrue(task.observe_only)

    def test_seek_then_slow_on_candidate_and_locked_reference(self):
        task = self.make()
        self.feed(task,0.)
        self.assertEqual((16,7),task.tick(0.,100.))
        self.feed(task,.1,.9)
        self.assertEqual((12,7),task.tick(.1,100.1))
        self.feed(task,.2,.88);self.feed(task,.3,.86)
        self.assertEqual((12,7),task.tick(.3,100.3))

    def test_brake_while_confirming_already_near_target(self):
        task = self.make()
        self.feed(task,0.,.72)
        self.assertEqual((0,0),task.tick(0.,100.))
        self.assertEqual('STOP_CONFIRM_REFERENCE',task.reason)

    def test_stationary_success_needs_distance_pose_and_fresh_time_span(self):
        task = self.make();self.lock(task,.73)
        self.assertEqual((0,0),task.tick(.2,100.2))
        self.assertEqual('HOLD',task.phase)
        for now in (.3,.4,.6):
            self.feed(task,now,.71)
            task.tick(now,100.+now)
        self.assertFalse(task.finished)
        self.feed(task,.8,.71)
        self.assertEqual((0,0),task.tick(.8,100.8))
        self.assertEqual('ALIGNMENT_COMPLETE',task.reason)

    def test_repeated_frame_cannot_fill_stationary_time(self):
        task = self.make();self.lock(task,.73);task.tick(.2,100.2)
        for now in (.3,.4,.5):self.feed(task,now,.71)
        task.tick(.8,100.8)
        self.assertFalse(task.finished)
        self.assertEqual(3,task.verify_votes)
        task.observe(1,100.5,.8,False)
        self.assertEqual(3,task.verify_votes)

    def test_wrong_lateral_heading_or_missing_side_stays_stopped(self):
        for offset,slope,midline in ((.12,0.,True),(0.,.15,True),(0.,0.,False)):
            task = self.make();self.lock(task,.73);task.tick(.2,100.2)
            for now in (.3,.5,.8,1.1,1.4,1.7,2.3):
                self.feed(task,now,.71,offset,slope,midline)
                self.assertEqual((0,0),task.tick(now,100.+now))
            self.assertEqual('ALIGNMENT_UNCONFIRMED_STOPPED',task.reason)

    def test_lost_target_brakes_without_using_stale_gap(self):
        task = self.make();self.lock(task,1.2)
        self.feed(task,.3)
        self.assertEqual((0,0),task.tick(.3,100.3))
        self.assertEqual('STOP_REFERENCE_MISSING',task.reason)
        self.feed(task,1.3)
        self.assertEqual((0,0),task.tick(1.3,101.3))
        self.assertEqual('REFERENCE_LOST_STOPPED',task.reason)

    def test_too_close_stops_instead_of_blind_reverse(self):
        task = self.make();self.lock(task,.60)
        self.assertEqual((0,0),task.tick(.2,100.2))
        self.assertEqual('REFERENCE_TOO_CLOSE_STOPPED',task.reason)

    def test_search_timeout_does_not_claim_arrival(self):
        task = self.make();self.feed(task,15.1)
        self.assertEqual((0,0),task.tick(15.1,115.1))
        self.assertEqual('REFERENCE_NOT_FOUND',task.reason)

    def test_lane_dropout_and_camera_expiry_stop(self):
        task = self.make();self.lock(task,1.2)
        task.follower.output = (0,0)
        self.assertEqual((0,0),task.tick(.2,100.2))
        self.assertEqual('WAIT_LANE',task.reason)
        self.assertEqual((0,0),task.tick(1.1,101.1))
        self.assertEqual('CAMERA_TIMEOUT',task.reason)

    def test_hold_never_rearms_even_if_gap_increases(self):
        task = self.make();self.lock(task,.73);task.tick(.2,100.2)
        self.feed(task,.5,.82)
        self.assertEqual((0,0),task.tick(.5,100.5))
        self.assertEqual('HOLD',task.phase)


class ArgumentTests(unittest.TestCase):
    def test_alignment_horizon_is_local_and_steers_toward_midpoint(self):
        from robot.common.config import load_config
        import os
        baseline = load_config(os.path.join(common.ROOT,'src/robot/config'))['lookahead']
        follower = make_follower(arguments([]))
        self.assertEqual(.65,follower.follower.core.cfg['lookahead'])
        for offset,expected in ((.08,1),(-.08,-1)):
            follower = make_follower(arguments([]))
            follower.observe(dict(stamp=100.,frame='base_link',confidence=.9,
                                  points=dual_midline(boundaries(offset))['points']),100.)
            speed,steer = follower.command(100.)
            self.assertGreater(speed,0)
            self.assertGreater(steer*expected,0)
        self.assertEqual(baseline,load_config(os.path.join(common.ROOT,'src/robot/config'))['lookahead'])

    def test_default_preview_without_motion(self):
        args = arguments([])
        self.assertFalse(args.execute)
        self.assertFalse(args.observe)
        self.assertEqual(0,args.steering)
        self.assertEqual(16,args.speed)
        self.assertEqual(.70,args.target_gap_m)

    def test_invalid_calibration_arguments_rejected(self):
        for values in (['--steering','-3'],['--side','left'],['--target-gap-m','nan'],
                       ['--target-gap-m','.3'],['--speed','10'],['--stop-margin-m','.09'],
                       ['--align-lookahead-m','.2']):
            with self.assertRaises(ValueError):arguments(values)


if __name__ == '__main__':
    unittest.main()
