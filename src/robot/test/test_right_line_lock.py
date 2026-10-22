"""Integrated right lock, current-steering gap and near-BEV tests; no actuators."""
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
from robot.common.geometry import bicycle
from robot.common.geometry import local
from robot.common.planning import intersection_path


class RightLineTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg.update(wait_green=False,right_turn_full_lock=True,right_direct_left_follow=False,right_turn_entry=0,right_reverse_entry_m=0,
                   right_turn_exit=.25,lookahead=.55,bypass_enabled=False)
        self.c = Controller(cfg)
        self.c.cfg['right_lock_command_rad'] = cfg['max_steer']
        self.addCleanup(self.c.close)

    def start(self):
        self.c.start_follow(intersection_path(self.c.pose,'RIGHT',self.c.cfg),'RIGHT',1)

    def test_left_reference_keeps_right_active_until_stably_aligned(self):
        self.c.cfg.update(right_direct_left_follow=True,right_lock_min_angle_deg=50,lidar_enabled=False)
        self.start()
        self.step(1.8,0,[(.25,.3),(.45,.3),(.65,.3)])
        self.assertEqual(self.c.right_lock['phase'],'LOCK')
        self.step(1.9,-49,[(.25,.3),(.45,.3),(.65,.3)])
        self.assertEqual(self.c.right_lock['phase'],'LOCK')
        command=self.step(2,-50.01,[(.25,.3),(.45,.3),(.65,.3)])
        self.assertEqual(self.c.state,'MANEUVER')
        self.assertEqual(self.c.action,'RIGHT')
        self.assertEqual(self.c.right_lock['phase'],'ALIGN_LEFT')
        for t in (2.1,2.2):
            self.step(t,-50.01,[(.25,.3),(.45,.3),(.65,.3)])
        self.assertEqual(self.c.action,'RIGHT')
        for t in (2.3,2.4,2.6):
            self.step(t,-50.01,[(.25,.3),(.45,.3),(.65,.3)])
        self.assertEqual(self.c.state,'LANE')
        self.assertIsNone(self.c.right_lock)
        self.assertTrue(self.c.follow_left_boundary)
        self.c.lane_command(2.7)
        self.assertEqual(self.c.lane_source,'left_boundary')
        self.c.observe_lane([(.3,-.2),(.5,-.2),(.7,-.2)],.9,3.2)
        self.assertEqual(self.c.lane_command(3.2),(0,0))
        self.assertEqual(self.c.state,'GAP')
        self.assertEqual(self.c.lane_source,'left_boundary')

    def test_left_offset_is_normal_thirty_cm_and_centered_command_is_zero(self):
        self.c.observe_left_boundary([(.25,.3),(.45,.3),(.65,.3)],1)
        line=self.c.left_exit_line(1)
        self.assertAlmostEqual(line['lateral'],0)
        self.assertTrue(all(abs(y)<1e-8 for x,y in line['points']))
        self.assertAlmostEqual(self.c.left_reference_command(line,1)[1],0)
        slope=.4
        intercept=.3*math.sqrt(1+slope*slope)
        self.c.observe_left_boundary([(x,slope*x+intercept) for x in (.25,.45,.65)],2)
        line=self.c.left_exit_line(2)
        for x,y in line['points']: self.assertAlmostEqual(y,slope*x)

    def test_alignment_lookahead_is_independent_of_following_lookahead(self):
        self.c.cfg.update(right_align_lookahead=.3,lookahead=.55,steering_command_scale_rad=.1)
        line=dict(points=[(.3,.04),(.55,.04),(.8,.04)],heading=0,lateral=.04)
        self.c.action='RIGHT'
        self.c.right_lock=dict(phase='ALIGN_LEFT')
        self.c.left_reference_command(line,1)
        expected=math.atan2(2*self.c.cfg['wheelbase']*.04,.3**2+.04**2)/self.c.cfg['max_steer']*.1
        self.assertAlmostEqual(self.c.left_reference['target_steer'],expected)
        self.assertEqual(self.c.left_reference['lookahead_m'],.3)
        self.c.action=None
        self.c.right_lock=None
        self.c.follow_left_boundary=True
        self.c.left_reference_command(line,2)
        expected=math.atan2(2*self.c.cfg['wheelbase']*.04,.55**2+.04**2)/self.c.cfg['max_steer']*.1
        self.assertAlmostEqual(self.c.left_reference['target_steer'],expected)
        self.assertEqual(self.c.left_reference['lookahead_m'],.55)

    def test_turn_capture_distance_is_independent_of_lane_width(self):
        self.c.cfg.update(right_direct_left_follow=True,right_left_capture_max_m=.8,lidar_enabled=False)
        self.start()
        heading=math.radians(-40)
        for stamp,d in ((2,.65),(3,.79)):
            points=[(x,math.tan(heading)*x+d/math.cos(heading)) for x in (.2,.4,.6,.8,1)]
            self.c.observe_left_boundary(points,stamp)
            line=self.c.left_exit_line(stamp)
            self.assertIsNotNone(line)
            self.assertAlmostEqual(line['lateral'],d-.30)
        self.assertEqual(self.c.cfg['lane_width'],.60)
        self.assertEqual(self.c.left_fit_diagnostic['reason'],'ok')
        self.c.observe_left_boundary([(.2,.81),(.4,.81),(.6,.81)],4)
        self.assertIsNone(self.c.left_exit_line(4))
        self.assertEqual(self.c.left_fit_diagnostic['reason'],'left_distance_too_far')
        self.c.right_lock=None
        self.c.observe_left_boundary([(.2,.65),(.4,.65),(.6,.65)],5)
        self.assertIsNone(self.c.left_exit_line(5))

    def test_left_fit_reports_fresh_empty_and_stale_separately(self):
        self.c.observe_left_boundary([],2)
        self.assertIsNone(self.c.left_exit_line(2))
        self.assertEqual(self.c.left_fit_diagnostic['reason'],'too_few_near_points')
        self.assertEqual(self.c.left_fit_diagnostic['near_points'],0)
        self.assertIsNone(self.c.left_exit_line(3))
        self.assertEqual(self.c.left_fit_diagnostic['reason'],'stale')

    def test_direct_alignment_cannot_bypass_maximum_turn_angle(self):
        self.c.cfg.update(right_direct_left_follow=True,lidar_enabled=False)
        self.start()
        self.step(2,-60,[(.25,.3),(.45,.3),(.65,.3)])
        self.assertEqual(self.c.right_lock['phase'],'ALIGN_LEFT')
        self.assertEqual(self.step(2.1,-120.3,[(.25,.3),(.45,.3),(.65,.3)]),(0,0))
        self.assertEqual(self.c.state,'FAULT')
        self.assertEqual(self.c.reason,'right_alignment_angle_limit')

    def test_direct_capture_cannot_bypass_limit_with_new_valid_line(self):
        self.c.cfg.update(right_direct_left_follow=True,lidar_enabled=False)
        self.start()
        self.assertEqual(self.step(2,-120.3,[(.25,.3),(.45,.3),(.65,.3)]),(0,0))
        self.assertEqual(self.c.state,'FAULT')
        self.assertEqual(self.c.reason,'right_exit_not_found')

    def test_left_reference_limits_reversal_and_does_not_saturate_small_error(self):
        from robot.common.contracts import encode_command
        self.c.cfg['steering_command_scale_rad']=.1
        self.c.observe_applied_steering(-.1,1)
        line=dict(points=[(.3,.05),(.5,.05),(.7,.05)],heading=0,lateral=.05)
        command=self.c.left_reference_command(line,1)
        raw=encode_command(command[0],command[1],self.c.cfg,0)['steering_raw']
        self.assertEqual(raw,-20)
        self.c.left_reference=None
        self.c.applied_stamp=-1
        self.c.issued_steer=0
        command=self.c.left_reference_command(line,2)
        self.assertLess(abs(command[1]),.1)
        for i in range(40):
            self.c.issued_steer=command[1]
            command=self.c.left_reference_command(line,2+(i+1)*.05)
        raw=encode_command(command[0],command[1],self.c.cfg,0)['steering_raw']
        self.assertTrue(0 < raw < 10)

    def test_left_reference_converges_to_offset_center_in_ideal_bicycle_model(self):
        self.c.cfg.update(steering_command_scale_rad=.1,lidar_enabled=False)
        # Start both directions inside the guarded road footprint. The old
        # (.1,.2) pose puts the front corner into the left tape margin and
        # must stop; the explicit crossing cases are covered by the U-turn
        # boundary tests instead of expecting convergence from that pose.
        for lateral,yaw in ((-.1,-.2),(.07,.1)):
            self.c.pose=(0,lateral,yaw)
            self.c.left_reference=None
            self.c.issued_steer=0
            self.c.left_boundary_stamp=-1
            for i in range(400):
                t=1+i*.05
                yaw=self.c.pose[2]
                intercept=(.3-self.c.pose[1])/math.cos(yaw)
                points=[(x,intercept-math.tan(yaw)*x) for x in (.2,.4,.6,.8,1)]
                self.c.observe_left_boundary(points,t)
                line=self.c.left_exit_line(t)
                self.assertIsNotNone(line)
                speed,steer=self.c.left_reference_command(line,t)
                self.c.issued_steer=steer
                physical=steer/.1*self.c.cfg['max_steer']
                self.c.pose=bicycle(self.c.pose,speed*.01*.05,physical,self.c.cfg['wheelbase'])
            self.assertLess(abs(self.c.pose[1]),.03)
            self.assertLess(abs(self.c.pose[2]),math.radians(3))

    def test_direct_left_waits_for_reverse_entry_and_fresh_left(self):
        self.c.cfg.update(right_direct_left_follow=True,right_reverse_entry_m=.45,lidar_enabled=False)
        self.start()
        self.c.observe_left_boundary([(.25,.3),(.45,.3),(.65,.3)],2)
        self.assertLess(self.c.right_lock_tick(2)[0],0)
        self.assertFalse(self.c.follow_left_boundary)

    def test_left_reference_duplicate_frames_and_loss_do_not_complete_alignment(self):
        self.c.cfg.update(right_direct_left_follow=True,lidar_enabled=False)
        self.start()
        self.step(2,-60,[(.25,.3),(.45,.3),(.65,.3)])
        for t in (2.1,2.2,2.3): self.c.right_lock_tick(t)
        self.assertEqual(self.c.exit_count,1)
        self.assertEqual(self.c.right_lock['stable_s'],0)
        self.assertEqual(self.c.right_lock_tick(2.6),(0,0))
        self.assertEqual(self.c.exit_count,0)
        self.assertEqual(self.c.right_lock['stable_s'],0)
        self.assertEqual(self.c.action,'RIGHT')
        self.step(2.7,-60,[(.25,.3),(.45,.3),(.65,.3)])
        self.assertEqual(self.c.right_lock['stable_s'],0)
        self.assertEqual(self.c.exit_count,1)

    def test_left_reference_mode_does_not_complete_from_unrelated_center(self):
        self.c.cfg.update(right_direct_left_follow=True,lidar_enabled=False)
        self.start()
        self.c.left_exit_line=lambda now:None
        for t in (2,2.1,2.2,2.3):
            self.step(t,-90,lane=[(.3,0),(.5,0),(.7,0)])
        self.assertEqual(self.c.right_lock['phase'],'LOCK')
        self.step(3,-121)
        self.assertEqual(self.c.state,'FAULT')
        self.assertEqual(self.c.reason,'right_exit_not_found')

    def test_trusted_center_can_replace_rejected_left_fit(self):
        self.c.cfg['lidar_enabled'] = False
        self.start()
        self.c.left_exit_line = lambda now: None
        self.c.set_pose((0,0,math.radians(-90)),2)
        for i in range(self.c.cfg['exit_frames']):
            t=2+i*.1
            self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.62,t)
            self.c.right_lock_tick(t)
        self.assertEqual(self.c.right_lock['phase'],'ALIGN')
        self.assertEqual(self.c.right_lock['exit_source'],'center')
        for i in range(self.c.cfg['exit_frames']):
            t=3+i*.1
            self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.62,t)
            self.c.right_lock_tick(t)
            if self.c.right_lock is None: break
        self.assertIsNone(self.c.right_lock)
        self.assertEqual(self.c.state,'LANE')

    def test_center_duplicate_frames_do_not_confirm_exit(self):
        self.start()
        self.c.left_exit_line = lambda now: None
        self.c.set_pose((0,0,math.radians(-90)),2)
        self.c.observe_lane([(.3,0),(.5,0),(.7,0)],.62,2)
        for t in (2,2.1,2.2): self.c.right_lock_tick(t)
        self.assertEqual(self.c.right_lock['phase'],'LOCK')
        self.assertIsNone(self.c.right_center_exit(2.6,90))

    def test_right_lock_point05_encodes_minus_11(self):
        from robot.common.contracts import encode_command
        self.c.cfg.update(steering_command_scale_rad=.1,right_lock_command_rad=.05,lidar_enabled=False)
        self.start()
        command = self.c.right_lock_tick(2)
        self.assertEqual(command[1],-.05)
        self.assertEqual(encode_command(command[0],command[1],self.c.cfg,0)['steering_raw'],-11)

    def test_reverse_entry_uses_signed_distance_then_stops_before_forward_lock(self):
        self.c.cfg.update(right_reverse_entry_m=.30,lidar_enabled=False)
        self.start()
        for x in (.31,-.29):
            self.c.set_pose((x,0,0),2)
            self.assertEqual(self.c.right_lock_tick(2),(-self.c.cfg['speed_raw']['action'],0.0))
        self.c.set_pose((-.301,0,0),2.1)
        self.assertEqual(self.c.right_lock_tick(2.1),(0,0))
        self.assertEqual(self.c.right_lock_tick(2.2),(self.c.cfg['speed_raw']['action'],-self.c.cfg['max_steer']))

    def test_entry_advances_thirty_cm_before_full_lock(self):
        self.c.cfg.update(right_turn_entry=.30,lidar_enabled=False)
        self.start()
        self.c.set_pose((.29,0,0),2)
        self.assertEqual(self.c.right_lock_tick(2),(self.c.cfg['speed_raw']['action'],0.0))
        self.assertEqual(self.c.right_lock['phase'],'ENTRY')
        self.c.set_pose((.301,0,0),2.1)
        self.assertEqual(self.c.right_lock_tick(2.1)[1],-self.c.cfg['max_steer'])
        self.assertEqual(self.c.right_lock['phase'],'LOCK')

    def step(self,t,angle=0,left=None,lane=None):
        self.c.set_pose((0,0,math.radians(angle)),t)
        self.c.scan = Scan([float('inf')]*360,-math.pi,math.pi/180,.05,6,
                           self.c.pose,self.c.cfg['lidar'],t)
        if left is not None:
            self.c.observe_left_boundary(left,t)
        if lane is not None:
            self.c.observe_lane(lane,.8,t)
        return self.c.tick(t)

    def test_right_is_full_lock_even_with_visible_original_lane(self):
        self.start()
        for t in (1,1.1,1.2):
            cmd = self.step(t,left=[(.25,.3),(.45,.3),(.65,.3)])
            self.assertEqual(cmd,(self.c.cfg['speed_raw']['action'],-self.c.cfg['max_steer']))
        self.assertEqual(self.c.action,'RIGHT')

    def test_exit_at_43_degrees_enters_alignment_without_releasing_lane(self):
        self.c.cfg['lidar_enabled'] = False
        self.start()
        self.c.set_pose((0,0,math.radians(-120.3)),2)
        self.c.left_exit_line = lambda now: dict(heading=math.radians(-43),
            lateral=.16,points=[(.3,.1),(.5,-.08),(.7,-.26)])
        self.c.right_lock_tick(2)
        self.assertEqual(self.c.right_lock['phase'],'ALIGN')
        self.assertEqual(self.c.state,'MANEUVER')
        self.assertEqual(self.c.action,'RIGHT')

    def test_exit_above_50_degrees_still_hits_120_degree_limit(self):
        self.start()
        self.c.set_pose((0,0,math.radians(-120.3)),2)
        self.c.left_exit_line = lambda now: dict(heading=math.radians(-51),lateral=.16)
        self.assertEqual(self.c.right_lock_tick(2),(0,0))
        self.assertEqual(self.c.reason,'right_exit_not_found')

    def test_left_boundary_aligns_and_overrides_wrong_center_then_releases(self):
        self.start()
        wrong = [(.3,.25),(.5,.3),(.7,.35)]
        for t in (2,2.1,2.2):
            cmd = self.step(t,-88,[(.25,.3),(.45,.3),(.65,.3)],wrong)
            self.assertAlmostEqual(cmd[1],0,places=6)
        self.assertIsNone(self.c.action)
        self.assertEqual(self.c.last_completed_action,'RIGHT')
        self.assertEqual(self.c.state,'LANE')
        self.assertAlmostEqual(self.step(2.3,-88,[(.25,.3),(.45,.3),(.65,.3)],wrong)[1],0,places=6)

    def test_duplicate_boundary_frames_do_not_finish(self):
        self.start()
        self.step(2,-85,[(.25,.3),(.45,.3),(.65,.3)])
        self.step(2.1,-85)
        self.step(2.2,-85)
        self.assertEqual(self.c.action,'RIGHT')

    def test_boundary_from_before_action_cannot_be_exit(self):
        self.c.observe_left_boundary([(.25,.3),(.45,.3),(.65,.3)],.95)
        self.start()
        self.assertIsNone(self.c.left_exit_line(1.1))

    def test_cross_line_and_right_side_line_are_not_left_exit(self):
        for points in ([ (.4,.1),(.4,.3),(.4,.5)],[(.25,-.3),(.45,-.3),(.65,-.3)]):
            self.start()
            self.assertEqual(self.step(2,-70,points)[1],-self.c.cfg['max_steer'])
            self.c.pose = (0,0,0)

    def test_no_line_does_not_drive_a_full_circle(self):
        self.start()
        self.assertEqual(self.step(2,-125),(0,0))
        self.assertEqual(self.c.reason,'right_exit_not_found')

    def test_gap_snapshots_current_applied_not_old_lane_steering(self):
        self.step(1,lane=[(.3,.15),(.5,.2),(.7,.2)])
        self.c.observe_applied_steering(-.12,1.1)
        self.assertAlmostEqual(self.step(1.15,lane=[])[1],-.12)
        self.c.observe_applied_steering(.25,1.2)
        self.assertAlmostEqual(self.step(1.25,lane=[])[1],-.12)

    def test_stale_applied_uses_last_issued_command(self):
        cmd = self.step(1,lane=[(.3,.1),(.5,.1),(.7,.1)])
        self.c.observe_applied_steering(-.4,.1)
        self.assertAlmostEqual(self.step(1.1,lane=[])[1],cmd[1])

    def test_right_still_checks_estop(self):
        self.start()
        self.c.estop = True
        self.assertEqual(self.step(2),(0,0))

    def test_wait_and_next_sign_stay_integrated(self):
        self.c.cfg['intersection_wait_s'] = 1
        self.c.pending = 'RIGHT'
        self.c.observe_ground(dict(source='front',markers=[dict(kind='junction',x=.3,y=0)],slots=[]),1)
        self.assertEqual(self.step(1),(0,0))
        self.assertEqual(self.c.state,'INTERSECTION_WAIT')
        self.assertEqual(self.step(1.9),(0,0))
        self.assertEqual(self.step(2)[1],-self.c.cfg['max_steer'])
        for t in (2.1,2.2):
            self.c.observe_sign('LEFT',.9,t,t)
        for t in (3,3.1,3.2):
            self.step(t,-90,[(.55,.3),(.60,.3),(.65,.3)])
        self.assertIsNone(self.c.action)
        self.assertIsNone(self.c.pending)
        self.assertIsNone(self.c.right_lock)

    def test_short_near_left_line_and_stale_line(self):
        self.start()
        self.step(2,-85,[(.55,.3),(.60,.3),(.65,.3)])
        self.assertEqual(self.c.right_lock['phase'],'ALIGN')
        self.assertIsNone(self.c.left_exit_line(2.6))
        self.assertEqual(self.step(6.1,-85),(0,0))
        self.assertEqual(self.c.reason,'right_alignment_line_lost')

    def test_bicycle_closed_loop_reaches_observed_exit(self):
        self.start()
        radius = self.c.cfg['wheelbase']/math.tan(self.c.cfg['max_steer'])
        boundary = [(radius+.30,-.05*i) for i in range(80)]
        phases = set()
        for i in range(400):
            t = 1+i*.05
            points = [local(self.c.pose,p) for p in boundary]
            points = [p for p in points if .55 <= p[0] <= 1.10 and abs(p[1]) < .59]
            self.c.observe_left_boundary(points,t)
            self.c.scan = Scan([float('inf')]*360,-math.pi,math.pi/180,.05,6,
                               self.c.pose,self.c.cfg['lidar'],t)
            cmd = self.c.tick(t)
            if self.c.right_lock:
                phases.add(self.c.right_lock['phase'])
            else:
                break
            self.c.observe_applied_steering(cmd[1],t)
            self.c.set_pose(bicycle(self.c.pose,cmd[0]*.008*.05,cmd[1],self.c.cfg['wheelbase']),t)
        self.assertEqual(phases,set(['LOCK','ALIGN']))
        self.assertIsNone(self.c.action,self.c.reason)
        self.assertLess(abs(math.degrees(self.c.pose[2])+90),15)


class NearVisionTests(unittest.TestCase):
    def setUp(self):
        try:
            import imp
            import rospy
        except ImportError:
            self.skipTest('requires sourced ROS Python')
        self.ros = rospy
        self.v = imp.load_source('right_near_vision',os.path.join(
            os.path.dirname(ROOT),'ros','camera','scripts','camera_yihan_web.py'))

    def test_near_windows_preserve_calibration_and_allow_short_segment(self):
        self.v.configure_lane_windows(20,.10)
        self.assertEqual(self.v.BEV_SIZE,(480,400))
        self.assertEqual(self.v.NUM_WINDOWS,14)
        self.assertEqual(len(self.v.previous_left_points),14)
        points = [dict(x=240,y=y,usable=True,window_index=i) for i,y in enumerate((380,360,340))]
        self.assertEqual(len(self.v.select_nearest_lane_path_segment([points])),3)

    def test_boundary_message_never_includes_predicted_points(self):
        points = [dict(x=120,y=v,usable=True,observed=True,score=1,geometry_ok=True)
                  for v in (380,340,300)]
        points.append(dict(x=120,y=260,usable=True,observed=False,score=1))
        msg = self.v.build_left_boundary_message(points,self.ros.Time.from_sec(2))
        self.assertEqual(len(msg.poses),3)
        self.assertAlmostEqual(msg.poses[0].pose.position.x,.55)
        self.assertEqual(msg.header.stamp.to_sec(),2)

    def test_short_near_mask_is_tracked_without_new_detector(self):
        import numpy as np
        self.v.configure_lane_windows(20,.10)
        mask = np.zeros((400,480),dtype=np.uint8)
        mask[322:382,117:122] = 255
        mask[322:382,359:364] = 255
        result = self.v.track_metric_lane(mask)
        msg = self.v.build_lane_path_message(result['center_segments'],self.ros.Time.from_sec(2))
        self.assertGreaterEqual(len(msg.poses),3)
        self.assertGreaterEqual(result['tracking_confidence'],.35)
        self.assertEqual(self.v.MAX_EMPTY_WINDOWS,4)


if __name__ == '__main__':
    unittest.main()
