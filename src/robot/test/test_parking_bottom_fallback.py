"""Bottom-only S entry and total travel from the preceding fixed straight."""
import copy
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
        self.assertEqual(entry.command(1.1, (0., 0., 0.)), (0, 0.))
        entry.observe(line, 1.2, (0., 0., 0.))
        self.assertIsNotNone(entry.bottom)
        self.assertEqual(entry.command(1.2, (0., 0., 0.))[0], 16)
        self.assertEqual(entry.reason, 'parking_follow_bottom_line')
        stop_x = .8-entry.cfg['wheelbase']-entry.cfg['front_overhang']-.09+.001
        entry.observe([], 1.3, (stop_x, 0., 0.))
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

    def test_bottom_search_rejects_side_lines_neighbour_bay_and_broad_stripe(self):
        for line in ([[(.4, -.19), (1., -.19)]],
                     [[(.8, .25), (.8, .63)]],
                     [[(.8, -.5), (.8, .5)]],
                     [[(.8, -.05), (.8, .05)]]):
            entry = ExplicitStraightEntry(self.config(), (0., 0., 0.), 1.)
            for stamp in (1.1, 1.2):
                entry.observe(line, stamp, (0., 0., 0.))
            self.assertIsNone(entry.bottom)
            self.assertEqual(entry.command(1.2, (0., 0., 0.)), (0, 0.))

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

    def test_distance_counts_preceding_straight_and_latches_at_limit(self):
        core = self.core()
        core.set_pose((1.2, 0., 0.), 2.)
        core.execute('parking', 'start_explicit_parking', 2., trigger='sign')
        core.set_pose((1.7, 0., 0.), 3.)
        self.assertEqual(core.tick(3.), (0, 0.))
        self.assertEqual(core.state, 'FAULT')
        self.assertEqual(core.reason, 'parking_straight_distance_limit')
        self.assertAlmostEqual(core.parking_straight_travel['travelled_m'], 1.7)
        core.set_pose((1.8, 0., 0.), 3.1)
        self.assertEqual(core.tick(3.1), (0, 0.))

    def test_queued_parking_already_limits_preceding_straight(self):
        core = self.core()
        core.state, core.action, core.next_direction = 'MANEUVER', 'STRAIGHT', 'PARKING'
        core.set_pose((1.7, 0., 0.), 2.)
        self.assertEqual(core.tick(2.), (0, 0.))
        self.assertEqual(core.reason, 'parking_straight_distance_limit')

    def test_distance_accumulates_curved_path_and_rejects_old_pose_updates(self):
        core = self.core()
        core.set_pose((.6, 0., 0.), 2.)
        core.set_pose((.6, .6, 0.), 3.)
        core.set_pose((0., .6, 0.), 4.)
        core.set_pose((0., 0., 0.), 2.)
        self.assertAlmostEqual(core.parking_straight_travel['travelled_m'], 1.8)

    def test_missing_fixed_straight_reference_keeps_s_parking_stopped(self):
        core = self.core()
        core.parking_straight_travel = None
        core.execute('parking', 'start_explicit_parking', 2., trigger='sign')
        self.assertEqual(core.tick(2.1), (0, 0.))
        self.assertEqual(core.reason, 'parking_straight_origin_missing')

    def test_near_limit_reserves_distance_for_command_expiry(self):
        core = self.core()
        core.set_pose((1.68, 0., 0.), 2.)
        core.execute('parking', 'start_explicit_parking', 2., trigger='sign')
        core.parking_entry.observe([[(2.08,-.19),(2.68,-.19)],
                                    [(2.08,.19),(2.68,.19)]], 2.1, core.pose)
        self.assertEqual(core.tick(2.1), (0, 0.))
        self.assertEqual(core.reason, 'parking_straight_distance_limit')
        self.assertGreater(core.parking_straight_travel['command_reserve_m'], .02)

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

    def test_stale_bottom_image_cannot_issue_motion(self):
        entry = ExplicitStraightEntry(self.config(), (0., 0., 0.), 1.)
        for stamp in (1.1,1.2):
            entry.observe([[ (.8,-.19),(.8,.19)]], stamp, (0.,0.,0.))
        self.assertEqual(entry.command(4., (0.,0.,0.)), (0,0.))
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

    def test_fixed_straight_handoff_keeps_budget_for_entry_until_1_7m(self):
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
        self.assertEqual(core.tick(4.),(0,0.))
        self.assertEqual(core.reason,'parking_straight_distance_limit')

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
