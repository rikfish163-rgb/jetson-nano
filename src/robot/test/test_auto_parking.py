"""AUTO parking regressions; no ROS master or motor output."""
from __future__ import division
from robot.lidar.scan import Scan as lidar_Scan
import copy
import math
import os
import sys
import unittest
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))
from robot.master.controller import Controller
from robot.common import planning as planning
from robot.common import geometry as geometry
from robot.common.contracts import validate_config

with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
    CONFIG = yaml.safe_load(stream)


class Scan:
    def evidence(self, point):
        return dict(classification=self.classify(point),cause='test_fixture')

    stamp = 10.8
    valid_rays = 360
    shape_filter = False
    obstacles = []
    def coverage(self, points):
        return 1.0
    def classify(self, point):
        return 'FREE'


class AutoParkingTest(unittest.TestCase):
    def core(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(parking_slot='AUTO', wait_green=False, parking_observe_s=0.5)
        core = Controller(cfg)
        core.scan = Scan()
        self.addCleanup(core.close)
        return core

    def observe(self, core, stamp, source='front', slots=None):
        if slots is None:
            slots = [dict(x=x, y=-.55, yaw=math.pi/2, kind='perpendicular')
                     for x in (.65, 1.55)]
        core.observe_ground(dict(source=source, part='slots', markers=[], slots=slots), stamp)

    def trigger(self, core, kind='junction'):
        core.pending = 'PARKING'
        core.observe_ground(dict(source='front', part='markers', slots=[],
            markers=[dict(kind=kind, x=.2, y=0)]), 10.0)
        return core.dispatch(10.0)

    def test_auto_config_is_accepted(self):
        validate_config(self.core().cfg)

    def test_reverse_plan_auto_rejects_parallel_bay(self):
        c = self.core()
        c.cfg['parking_mode'] = 'reverse_plan'
        parallel = dict(id='P1', kind='parallel', pose=(.65, -.55, 0.),
                        length=.70, width=.36)
        path, reason, selected, rows = planning.auto_parking_plan(
            c.pose, [parallel], c.scan, c.cfg)
        self.assertEqual(path, [])
        self.assertEqual(reason, 'no_empty_reachable_bay')
        self.assertIsNone(selected)
        self.assertEqual(rows[0]['plan'], 'unsupported_reverse_slot')

    def sign(self, core, label, start):
        votes = core.cfg['parking_sign_votes'] if label == 'PARKING' else core.cfg['sign_votes']
        for i in range(votes):
            now = start+i*.1
            core.observe_sign(label,.99,now,now)

    def test_p_latches_until_later_blue_without_timeout_or_overwrite(self):
        c = self.core()
        self.sign(c,'PARKING',1.0)
        self.sign(c,'LEFT',2.0)
        self.assertEqual(c.pending,'PARKING')
        c.scan.stamp = 10.0
        c.observe_lane([(.3,0),(.6,0)],1.0,10.0)
        self.assertGreater(c.tick(10.0)[0],0)
        self.assertEqual(c.pending,'PARKING')
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.future)
        self.assertEqual(self.trigger(c),(0,0.0))
        self.assertEqual(c.state,'PARKING_SCAN')

    def test_blue_before_p_cannot_trigger_parking(self):
        c = self.core()
        c.cfg['blue_default_straight'] = False  # isolate P timing from the straight fallback
        data = dict(source='front',part='markers',slots=[],
                    markers=[dict(kind='junction',x=.2,y=0)])
        c.observe_ground(data,1.0)
        c.scan.stamp = 1.0
        c.tick(1.0)
        self.assertNotEqual(c.state,'PARKING_SCAN')
        self.sign(c,'PARKING',2.0)
        self.assertIsNone(c.dispatch(2.3))
        # A delayed frame captured before the P confirmation is still old.
        c.observe_ground(data,1.9)
        self.assertIsNone(c.dispatch(2.3))
        c.observe_ground(data,2.4)
        self.assertEqual(c.dispatch(2.4),(0,0.0))
        self.assertEqual(c.state,'PARKING_SCAN')

    def test_short_blue_does_not_trigger_auto(self):
        c = self.core()
        self.assertIsNone(self.trigger(c, 'tick'))
        self.assertEqual(c.state, 'LANE')

    def test_long_blue_stops_then_requires_new_front_observations(self):
        c = self.core()
        self.observe(c, 9.9)
        self.assertEqual(self.trigger(c), (0, 0.0))
        self.assertEqual(c.state, 'PARKING_SCAN')
        self.observe(c, 10.2, source='rear')
        self.assertEqual(c.tick(10.8), (0, 0.0))
        self.assertIsNone(c.future)
        self.observe(c, 10.3)
        c.tick(10.8)
        self.assertIsNone(c.future)

    def test_occupied_near_bay_is_rejected_even_with_high_coverage(self):
        c = self.core()
        self.trigger(c)
        self.observe(c, 10.3)
        self.observe(c, 10.8)
        c.scan.obstacles = [(.65, -.55)]
        rows = planning.rank_parking_slots(c.pose, c.parking_candidates, c.scan, c.cfg)
        self.assertEqual([s['occupancy'] for s in rows], ['OCCUPIED', 'FREE'])
        self.assertAlmostEqual(rows[0]['approach_distance'], 0, places=2)
        self.assertGreater(rows[1]['approach_distance'], .8)

    def test_unknown_is_not_an_empty_bay(self):
        c = self.core()
        self.trigger(c)
        self.observe(c, 10.3)
        self.observe(c, 10.8)
        c.scan.coverage = lambda points: .2
        rows = planning.rank_parking_slots(c.pose, c.parking_candidates, c.scan, c.cfg)
        self.assertTrue(all(s['occupancy'] == 'UNKNOWN' for s in rows))
        c.tick(10.8)
        self.assertEqual(c.state, 'PARKING_SCAN')
        self.assertIsNone(c.future)

    def test_left_and_right_bays_face_toward_road(self):
        c = self.core()
        self.observe(c, 10.5, slots=[dict(x=.65,y=y,yaw=math.pi/2,kind='perpendicular')
                                   for y in (-.55,.55)])
        for slot in c.parking_candidates:
            self.assertLess(slot['pose'][1] * math.sin(slot['pose'][2]), 0)

    def test_locked_rear_observation_does_not_reselect_or_rename_bay(self):
        c = self.core()
        self.observe(c, 10.5)
        c.slot = c.parking_candidates[1]
        c.slot_locked = True
        target = dict(c.slot)
        self.observe(c, 10.8, source='rear')
        self.assertEqual(c.slot, target)
        self.assertEqual(c.slot_stamp, 10.8)
        self.observe(c,10.7,source='front')
        self.assertEqual(c.slot_stamp,10.8)

    def test_real_scan_distinguishes_near_obstacle_and_far_clear_bay(self):
        c = self.core()
        self.observe(c,10.5)
        rays = [float('inf')]*720
        angle = math.atan2(-.55,.65)
        index = int(round((angle+math.pi)/(2*math.pi/720)))
        rays[index] = math.hypot(.65,.55)
        scan = lidar_Scan(rays,-math.pi,2*math.pi/720,.05,6,c.pose,c.cfg['lidar'],10.5)
        rows = planning.rank_parking_slots(c.pose,c.parking_candidates,scan,c.cfg)
        self.assertEqual([s['occupancy'] for s in rows],['OCCUPIED','FREE'])

    def test_invalid_auto_tuning_is_rejected(self):
        for name,value in (('parking_prep_offset',-1),('parking_observe_s',float('nan')),
                           ('parking_sync_s',0),('parking_road_half_width',0)):
            cfg = copy.deepcopy(self.core().cfg)
            cfg[name] = value
            with self.assertRaises(ValueError):
                validate_config(cfg)

    def test_near_infeasible_falls_back_to_far_and_prefix_is_straight(self):
        c = self.core()
        self.observe(c, 10.5)
        original = planning.hybrid_plan
        calls = []
        def stub(start, goal, cfg, obstacles=(), allowed=None, final_direction=-1):
            calls.append(start)
            if len(calls) == 1:
                return [], 'no_path'
            return [tuple(start)+(-1,0), tuple(goal)+(-1,0)], 'planned'
        planning.hybrid_plan = stub
        try:
            path, reason, slot, rows = planning.auto_parking_plan(
                c.pose, c.parking_candidates, c.scan, c.cfg)
        finally:
            planning.hybrid_plan = original
        self.assertEqual(reason, 'planned')
        self.assertAlmostEqual(slot['pose'][0], 1.55)
        self.assertEqual(rows[0]['plan'], 'no_path')
        self.assertEqual(path[0][:3], c.pose)
        prefix = [p for p in path if p[3] == 1]
        self.assertTrue(prefix)
        self.assertTrue(all(p[1] == 0 and p[2] == 0 and p[4] == 0 for p in prefix))

    def test_parking_sweep_does_not_extend_past_planned_stop(self):
        c = self.core()
        c.start_follow([(0,0,0,-1,0),(-.1,0,0,-1,0)], 'PARKING', 10)
        c.scan.obstacles = [(-.3,0)]  # behind the planned final body, not in its sweep
        self.assertEqual(c.checked_command((-16,0),10.8,False), (-16,0))
        c.scan.obstacles = [(-.15,0)]  # actually in the reverse sweep
        self.assertEqual(c.checked_command((-16,0),10.8,False), (0,0.0))

    def test_real_far_bay_plan_with_occupied_near_bay_reaches_terminal(self):
        c = self.core()
        self.trigger(c)
        self.observe(c,10.3)
        self.observe(c,10.8)
        c.scan.obstacles = [(.65,-.55)]
        c.tick(10.8)
        self.assertEqual(c.state,'AUTO_PLANNING')
        c.future.result(timeout=15)
        c.tick(10.8)
        self.assertEqual(c.state,'PARKING',c.reason)
        self.assertAlmostEqual(c.slot['pose'][0],1.55)
        self.assertGreater(c.slot['approach_distance'],.8)
        # Timestamped front/rear observations of the SAME stationary world bay.
        for i in range(800):
            now = 10.85+i*.05
            c.scan.stamp = now
            xy = geometry.local(c.pose,c.slot['pose'])
            seen = dict(x=xy[0],y=xy[1],yaw=geometry.wrap(c.slot['pose'][2]-c.pose[2]),kind='perpendicular')
            self.observe(c,now,'front' if i%2 else 'rear',[seen])
            c.set_pose(c.pose,now)
            speed,steer = c.tick(now)
            if c.state == 'FINISHED':
                break
            gain = c.cfg['raw_to_mps']['forward' if speed >= 0 else 'reverse']
            c.pose = geometry.bicycle(c.pose,speed*gain*.05,steer,c.cfg['wheelbase'])
        self.assertEqual(c.state,'FINISHED',c.reason)
        self.assertTrue(geometry.inside_slot(c.pose,c.slot,c.cfg))


if __name__ == '__main__':
    unittest.main()
