"""Bottom-only S entry and total travel from the preceding fixed straight."""
import copy
import math
import unittest

from test_core import CONFIG
from robot.master.controller import Controller
from robot.parking.entry import ExplicitStraightEntry


class ParkingBottomFallbackTests(unittest.TestCase):
    def config(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, lidar_enabled=False,
                   parking_lidar_enabled=False, parking_mode='forward_center',
                   parking_entry_style='S', parking_slot='P4',
                   parking_entry_speed_raw=16, parking_final_speed_raw=12,
                   parking_bottom_clearance_m=.09,
                   parking_blue_max_travel_m=1.7)
        return cfg

    def core(self):
        core = Controller(self.config())
        self.addCleanup(core.close)
        core.last_blue_trigger = dict(stamp=1., vehicle_pose=core.pose,
                                      travelled_m=0.)
        core.parking_straight_travel = dict(stamp=1., vehicle_pose=core.pose,
                                           travelled_m=0., source='fixed_straight')
        return core

    def test_bottom_without_sides_or_p_projection_is_confirmed_then_followed(self):
        entry = ExplicitStraightEntry(self.config(), (0., 0., 0.), 1.)
        line = [[(.8, -.19), (.8, .19)]]
        entry.observe(line, 1.1, (0., 0., 0.))
        self.assertEqual(entry.command(1.1, (0., 0., 0.)), (16, 0.))
        entry.observe(line, 1.2, (0., 0., 0.))
        self.assertIsNotNone(entry.bottom)
        self.assertEqual(entry.command(1.2, (0., 0., 0.))[0], 16)
        self.assertEqual(entry.reason, 'parking_follow_bottom_line')
        stop_x = .8-entry.cfg['wheelbase']-entry.cfg['front_overhang']-.09+.001
        entry.observe(line, 1.3, (stop_x, 0., 0.))
        self.assertEqual(entry.command(1.3, (stop_x, 0., 0.)), (0, 0.))
        self.assertEqual(entry.status, 'FINISHED')

    def test_lost_side_pair_can_lock_bottom_from_previous_corridor(self):
        entry = ExplicitStraightEntry(self.config(), (0., 0., 0.), 1.)
        entry.observe([[ (.4, -.19), (1., -.19)],
                       [(.4, .19), (1., .19)]], 1.1, (0., 0., 0.))
        self.assertGreater(entry.command(1.1, (0., 0., 0.))[0], 0)
        for stamp in (1.2, 1.3):
            entry.observe([[(1., -.19), (1., .19)]], stamp, (0., 0., 0.))
        self.assertIsNone(entry.view)
        self.assertIsNotNone(entry.bottom)
        self.assertGreater(entry.command(1.3, (0., 0., 0.))[0], 0)

    def test_left_side_keeps_correcting_after_pair_loss_with_bottom(self):
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1.)
        left = [(.4,.23),(1.,.23)]
        right = [(.4,-.15),(1.,-.15)]
        bottom = [(1.,-.15),(1.,.23)]
        entry.observe([right,left,bottom], 1.1, (0.,0.,0.))
        entry.command(1.1, (0.,0.,0.))
        entry.observe([left,bottom], 1.2, (0.,0.,0.))
        speed, steer = entry.command(1.2, (0.,0.,0.))
        self.assertGreater(speed, 0)
        self.assertGreater(steer, 0)
        self.assertEqual(entry.reason, 'parking_follow_left_side')
        self.assertEqual(entry.debug['steering_reference'], 'left_side')
        self.assertIsNotNone(entry.bottom)

    def test_p_associated_left_can_start_without_pair_or_bottom(self):
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1., (1.,.04))
        entry.observe([[ (.4,.23),(1.,.23)]], 1.1, (0.,0.,0.))
        speed, steer = entry.command(1.1, (0.,0.,0.))
        self.assertGreater(speed, 0)
        self.assertGreater(steer, 0)
        self.assertEqual(entry.reason, 'parking_follow_left_side')

    def test_left_fallback_offsets_normal_to_slanted_side(self):
        angle = math.radians(25)
        c,s = math.cos(angle),math.sin(angle)
        def point(x,y):return (c*x-s*y,s*x+c*y)
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1., point(1.,0.))
        side = [point(.4,.19),point(1.,.19)]
        entry.observe([side], 1.1, (0.,0.,0.))
        entry.command(1.1, (0.,0.,0.))
        self.assertEqual(entry.reason, 'parking_follow_left_side')
        x,y = entry.single['center']
        self.assertAlmostEqual(-s*x+c*y, 0., places=6)

    def test_side_fallback_rejects_neighbour_and_background(self):
        for side in ([ (.4,-.57),(1.,-.57)],
                     [(.4,.57),(1.,.57)],
                     [(1.1,.19),(1.5,.19)]):
            entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1., (1.,0.))
            entry.observe([side], 1.1, (0.,0.,0.))
            self.assertEqual(entry.command(1.1, (0.,0.,0.)), (16,0.))
            self.assertFalse(entry.debug['geometry_ready'])

    def test_missing_left_does_not_reuse_old_steering(self):
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1., (1.,.04))
        left,bottom = [(.4,.23),(1.,.23)],[(1.,-.15),(1.,.23)]
        entry.observe([left,bottom], 1.1, (0.,0.,0.))
        self.assertGreater(entry.command(1.1, (0.,0.,0.))[1], 0)
        entry.observe([bottom], 1.2, (0.,0.,0.))
        self.assertEqual(entry.command(1.2, (0.,0.,0.))[1], 0.)
        self.assertGreater(entry.command(3., (0.,0.,0.))[0], 0)
        self.assertEqual(entry.command(3., (0.,0.,0.))[1], 0.)

    def test_pair_and_bottom_stop_take_priority_over_left_fallback(self):
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1.)
        lines = [[(.4,-.19),(1.,-.19)],[ (.4,.19),(1.,.19)],[(1.,-.19),(1.,.19)]]
        entry.observe(lines, 1.1, (0.,0.,0.))
        entry.command(1.1, (0.,0.,0.))
        self.assertEqual(entry.reason, 'parking_center_between_sides')
        pose = (.59,0.,0.)
        entry.observe(lines[1:], 1.2, pose)
        self.assertEqual(entry.command(1.2, pose), (0,0.))
        self.assertEqual(entry.status, 'FINISHED')

    def test_short_left_continues_locked_bay_but_cannot_acquire_one(self):
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1.)
        bottom = [(.6,-.15),(.6,.23)]
        entry.observe([[ (.3,.23),(.6,.23)],[(.3,-.15),(.6,-.15)],bottom],
                      1.1, (0.,0.,0.))
        entry.command(1.1, (0.,0.,0.))
        entry.observe([[ (.53,.23),(.6,.23)],bottom], 1.2, (0.,0.,0.))
        self.assertGreater(entry.command(1.2, (0.,0.,0.))[1], 0)
        self.assertEqual(entry.reason, 'parking_follow_left_side')
        fresh = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1., (.6,.04))
        fresh.observe([[ (.53,.23),(.6,.23)]], 1.2, (0.,0.,0.))
        self.assertEqual(fresh.command(1.2, (0.,0.,0.)), (16,0.))
        self.assertFalse(fresh.debug['geometry_ready'])

    def test_locked_left_rejects_neighbour_wrong_heading_and_past_bottom(self):
        for side in ([ (.4,.61),(1.,.61)],
                     [(.4,.23),(.7,.40)],
                     [(1.02,.23),(1.3,.23)]):
            entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1.)
            bottom = [(1.,-.15),(1.,.23)]
            entry.observe([[ (.4,.23),(1.,.23)],[(.4,-.15),(1.,-.15)],bottom],
                          1.1, (0.,0.,0.))
            entry.command(1.1, (0.,0.,0.))
            entry.observe([side,bottom], 1.2, (0.,0.,0.))
            self.assertEqual(entry.command(1.2, (0.,0.,0.))[1], 0.)
            self.assertIsNone(entry.single)

    def test_left_observations_refresh_reference_but_duplicate_frames_do_not(self):
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1., (1.,.04))
        line = [[(.4,.23),(1.,.23)]]
        for stamp in (1.1,1.8,2.5):
            entry.observe(line, stamp, (0.,0.,0.))
            self.assertGreater(entry.command(stamp, (0.,0.,0.))[1], 0)
        entry.observe(line, 2.5, (0.,0.,0.))
        self.assertGreater(entry.command(4., (0.,0.,0.))[0], 0)
        self.assertEqual(entry.stamp, 2.5)
        entry.observe(line, 4.1, (0.,0.,0.))
        self.assertEqual(entry.command(4.1, (0.,0.,0.)), (16,0.))

    def test_short_line_extraction_is_opt_in_for_s_parking(self):
        import cv2
        import numpy as np
        from robot.camera.vision import GroundDetector
        detector = GroundDetector(self.config())
        mask = np.zeros((400,480), dtype=np.uint8)
        cv2.line(mask, (300,200), (300,232), 255, 2)
        self.assertEqual(detector.parking_lines(mask), [])
        lines = detector.parking_lines(mask,min_length_m=.04)
        self.assertTrue(lines)
        self.assertTrue(any(abs(a[0]-b[0])>=.04 for a,b in lines))

    def test_s_ignores_2_1m_cap_and_stops_only_at_measured_bottom_clearance(self):
        cfg = self.config()
        cfg['parking_blue_max_travel_m'] = 2.1
        core = Controller(cfg)
        self.addCleanup(core.close)
        core.parking_straight_travel = dict(stamp=1., vehicle_pose=core.pose,
                                           travelled_m=0., source='fixed_straight')
        core.set_pose((2.2,0.,0.), 2.)
        core.execute('parking','start_explicit_parking',2.,trigger='sign')
        lines = [[(2.6,-.19),(3.4,-.19)],[(2.6,.19),(3.4,.19)],
                 [(3.4,-.19),(3.4,.19)]]
        core.parking_entry.observe(lines, 2.1, core.pose)
        self.assertGreater(core.tick(2.1)[0], 0)
        self.assertFalse(core.parking_entry.debug['distance_limit_enabled'])
        self.assertIsNone(core.parking_entry.debug['travel_limit_m'])
        stop_x=3.4-cfg['wheelbase']-cfg['front_overhang']-.09+.001
        core.set_pose((stop_x,0.,0.), 2.2)
        core.parking_entry.observe(lines,2.2,core.pose)
        self.assertEqual(core.tick(2.2), (0,0.))
        self.assertEqual(core.reason, 'parking_at_bottom_clearance')
        self.assertEqual(core.state,'FINISHED')

    def test_bottom_search_rejects_side_lines_neighbour_bay_and_broad_stripe(self):
        for line in ([[(.4, -.19), (1., -.19)]],
                     [[(.8, .25), (.8, .63)]],
                     [[(.8, -.5), (.8, .5)]],
                     [[(.8, -.05), (.8, .05)]]):
            entry = ExplicitStraightEntry(self.config(), (0., 0., 0.), 1.)
            for stamp in (1.1, 1.2):
                entry.observe(line, stamp, (0., 0., 0.))
            self.assertIsNone(entry.bottom)
            self.assertEqual(entry.command(1.2, (0., 0., 0.)), (16, 0.))

    def test_duplicate_or_changed_bottom_does_not_confirm(self):
        entry = ExplicitStraightEntry(self.config(), (0., 0., 0.), 1.)
        entry.observe([[ (.8, -.19), (.8, .19)]], 1.1, (0., 0., 0.))
        entry.observe([[ (.8, -.19), (.8, .19)]], 1.1, (0., 0., 0.))
        self.assertIsNone(entry.bottom)
        entry.observe([[ (1.1, -.19), (1.1, .19)]], 1.2, (0., 0., 0.))
        self.assertIsNone(entry.bottom)

    def test_previous_side_ends_reject_the_bay_mouth_as_bottom(self):
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1.)
        entry.observe([[(.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)]],
                      1.1, (0.,0.,0.))
        entry.command(1.1, (0.,0.,0.))
        for stamp in (1.2,1.3):
            entry.observe([[ (.4,-.19),(.4,.19)]], stamp, (0.,0.,0.))
        self.assertIsNone(entry.bottom)

    def test_large_travel_without_initial_geometry_continues_without_distance_fault(self):
        core = self.core()
        core.set_pose((1.2, 0., 0.), 2.)
        core.execute('parking', 'start_explicit_parking', 2., trigger='sign')
        core.set_pose((3., 0., 0.), 3.)
        self.assertEqual(core.tick(3.), (16, 0.))
        self.assertEqual(core.state, 'PARKING')
        self.assertEqual(core.reason, 'tracking_parking')
        self.assertAlmostEqual(core.parking_straight_travel['travelled_m'], 3.)

    def test_queued_s_parking_does_not_cap_preceding_straight(self):
        core = self.core()
        core.state, core.action, core.next_direction = 'MANEUVER', 'STRAIGHT', 'PARKING'
        core.set_pose((3., 0., 0.), 2.)
        core.straight_search={}
        core.straight_search_tick=lambda now:(16,0.)
        self.assertEqual(core.tick(2.), (16, 0.))
        self.assertNotEqual(core.state, 'FAULT')

    def test_distance_accumulates_curved_path_and_rejects_old_pose_updates(self):
        core = self.core()
        core.set_pose((.6, 0., 0.), 2.)
        core.set_pose((.6, .6, 0.), 3.)
        core.set_pose((0., .6, 0.), 4.)
        core.set_pose((0., 0., 0.), 2.)
        self.assertAlmostEqual(core.parking_straight_travel['travelled_m'], 1.8)

    def test_s_can_follow_bottom_without_fixed_straight_reference(self):
        core = self.core()
        core.parking_straight_travel = None
        core.execute('parking', 'start_explicit_parking', 2., trigger='sign')
        for stamp in (2.1,2.2):
            core.parking_entry.observe([[ (.8,-.19),(.8,.19)]],stamp,core.pose)
        self.assertGreater(core.tick(2.2)[0],0)
        self.assertEqual(core.state,'PARKING')

    def test_old_reserve_threshold_does_not_stop_s_side_tracking(self):
        core = self.core()
        core.set_pose((1.68, 0., 0.), 2.)
        core.execute('parking', 'start_explicit_parking', 2., trigger='sign')
        core.parking_entry.observe([[(2.08,-.19),(2.68,-.19)],
                                    [(2.08,.19),(2.68,.19)]], 2.1, core.pose)
        self.assertGreater(core.tick(2.1)[0],0)
        self.assertEqual(core.state,'PARKING')
        self.assertIsNone(core.parking_straight_travel['parking_limit_m'])

    def test_real_blue_dispatch_records_vehicle_pose_and_new_blue_resets_distance(self):
        core = self.core()
        def dispatch_blue(now, point):
            core.action = None
            core.pending = 'STRAIGHT'
            core.marker = (point, now)
            core.front_marker_stamp = now
            core.front_blue_lines = [dict(point=point,yaw=0.)]
            core.dispatch(now)
        dispatch_blue(1., (.6, 0.))
        self.assertEqual(core.last_blue_trigger['vehicle_pose'], [0., 0., 0.])
        core.set_pose((.3, 0., 0.), 2.)
        self.assertAlmostEqual(core.last_blue_trigger['travelled_m'], .3)
        core.dispatch(2.1)  # The running action cannot dispatch the same blue again.
        self.assertAlmostEqual(core.last_blue_trigger['travelled_m'], .3)
        dispatch_blue(3., (.9, 0.))
        self.assertEqual(core.last_blue_trigger['vehicle_pose'], [.3, 0., 0.])
        self.assertEqual(core.last_blue_trigger['travelled_m'], 0.)

    def test_distance_limit_does_not_stop_unrelated_route_before_parking_is_known(self):
        core = self.core()
        core.set_pose((1.8, 0., 0.), 2.)
        core.observe_lane([(.5,0.),(.7,0.),(.9,0.)], .99, 2.)
        self.assertGreater(core.tick(2.)[0], 0)
        self.assertNotEqual(core.state, 'FAULT')

    def test_t_entry_obeys_same_distance_limit(self):
        core = self.core()
        core.cfg['parking_entry_style'] = 'T'
        core.set_pose((1.7, 0., 0.), 2.)
        core.execute('parking', 'start_explicit_parking', 2., trigger='blue')
        self.assertEqual(core.tick(2.1), (0, 0.))
        self.assertEqual(core.reason, 'parking_blue_distance_limit')

    def test_stale_bottom_continues_without_claiming_arrival_from_pose(self):
        entry = ExplicitStraightEntry(self.config(), (0., 0., 0.), 1.)
        for stamp in (1.1,1.2):
            entry.observe([[ (.8,-.19),(.8,.19)]], stamp, (0.,0.,0.))
        self.assertGreater(entry.command(10.75, (0.,0.,0.))[0], 0)
        self.assertEqual(entry.reason, 'parking_follow_bottom_line')
        self.assertAlmostEqual(entry.debug['line_age_s'], 9.55)
        self.assertEqual(entry.debug['distance_source'], 'measured_bottom_plus_pose')
        stop_x = .8-entry.cfg['wheelbase']-entry.cfg['front_overhang']-.09+.001
        self.assertEqual(entry.command(10.85, (stop_x,0.,0.))[0],12)
        self.assertIsNone(entry.status)
        self.assertFalse(entry.debug['bottom_arrival_confirmed'])

    def test_stale_pair_and_missing_geometry_both_continue(self):
        entry = ExplicitStraightEntry(self.config(), (0.,0.,0.), 1.)
        self.assertEqual(entry.command(10.75, (0.,0.,0.)), (16,0.))
        self.assertEqual(entry.reason, 'parking_missing_white_straight')
        entry.observe([[ (.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)]],
                      1.2, (0.,0.,0.))
        self.assertGreater(entry.command(10.75, (.1,0.,0.))[0], 0)
        self.assertEqual(entry.reason, 'parking_center_between_sides')

    def test_other_forward_modes_keep_freshness_check(self):
        from robot.parking.forward import ForwardParking
        entry = ForwardParking(self.config(), (0.,0.,0.), 1.)
        entry.observe([[ (.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)]],
                      1.2, (0.,0.,0.))
        self.assertEqual(entry.command(10.75, (0.,0.,0.)), (0,0.))
        self.assertEqual(entry.reason, 'parking_front_stale')

    def test_fixed_straight_origin_excludes_recorded_blue_approach(self):
        core = self.core()
        core.cfg['straight_speed_raw'] = 30
        core.cfg['raw_to_mps']['forward'] = .07076262567077475/(30*.25)
        core.set_pose((.4132697112,0.,0.), 2.)
        core.start_follow([tuple(core.pose)+(1,), (1.6632697112,0.,0.,1)],
                          'STRAIGHT', 2.)
        core.next_direction = 'PARKING'
        core.straight_search_tick = lambda now: (30,0.)
        core.set_pose((1.6354064555,0.,0.), 3.)
        self.assertGreater(core.tick(3.)[0], 0)
        self.assertNotEqual(core.state, 'FAULT')
        self.assertAlmostEqual(core.parking_straight_travel['travelled_m'],
                               1.6354064555-.4132697112)

    def test_fixed_straight_handoff_keeps_travel_history_without_capping_entry(self):
        core = self.core()
        core.set_pose((.413,0.,0.), 2.)
        core.start_follow([tuple(core.pose)+(1,), (1.663,0.,0.,1)], 'STRAIGHT', 2.)
        core.set_pose((1.663,0.,0.), 3.)
        core.execute('mission','resume_lane')
        core.execute('parking','start_explicit_parking',3.,trigger='sign')
        core.parking_entry.observe([[(2.063,-.19),(2.663,-.19)],
                                    [(2.063,.19),(2.663,.19)]],3.1,core.pose)
        self.assertGreater(core.tick(3.1)[0],0)
        self.assertAlmostEqual(core.parking_straight_travel['travelled_m'],1.25)
        core.set_pose((2.113,0.,0.),4.)
        self.assertGreater(core.tick(4.)[0],0)
        self.assertEqual(core.state,'PARKING')

    def test_next_fixed_straight_starts_a_new_budget(self):
        core = self.core()
        core.set_pose((1.,0.,0.), 2.)
        core.start_follow([tuple(core.pose)+(1,), (2.25,0.,0.,1)], 'STRAIGHT', 2.)
        self.assertEqual(core.parking_straight_travel['travelled_m'],0.)
        core.set_pose((1.4,0.,0.), 3.)
        self.assertAlmostEqual(core.parking_straight_travel['travelled_m'],.4)
        core.start_follow([tuple(core.pose)+(1,), (2.65,0.,0.,1)], 'STRAIGHT', 3.)
        self.assertEqual(core.parking_straight_travel['travelled_m'],0.)
        self.assertEqual(core.parking_straight_travel['vehicle_pose'],[1.4,0.,0.])
