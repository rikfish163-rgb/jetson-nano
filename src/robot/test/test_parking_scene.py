"""Measured side-parking scene tests; no ROS node or actuator is started."""
from __future__ import division

import copy
import math
import os
import unittest

import yaml

from robot.common.geometry import local
from robot.master.controller import Controller
from robot.lidar.scan import Scan as RealScan
from robot.parallel_parking.scene import ParallelParkingScene
from robot.parallel_parking.scene import build_parallel_scene
from robot.parallel_parking.scene import invert_relative_pose
from robot.parallel_parking.scene import public_scene
from robot.parallel_parking.scene import _partial_slot_from_lines
from robot.parallel_parking.scene import _region
from robot.camera.observations import _observe_parallel_scene


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
    CONFIG = yaml.safe_load(stream)


def scene_cfg(**changes):
    cfg = dict(parking_slot='P1', pose_mode='command_model',
               parallel_parking_confirm_frames=3, lidar_timeout=.5,
               parallel_parking_slot_drift_m=.12,
               lidar={'min_rays': 10, 'x': 0., 'y': 0., 'yaw': 0.},
               slots={'P1': dict(kind='parallel', length=.70, width=.36,
                                 min_candidates=1),
                      'P2': dict(kind='parallel', length=.70, width=.36,
                                 min_candidates=1),
                      'P3': dict(kind='parallel', length=.70, width=.36,
                                 min_candidates=1),
                      'P4': dict(kind='perpendicular', length=.45, width=.38,
                                 rank=-1, min_candidates=2),
                      'P5': dict(kind='perpendicular', length=.45, width=.38,
                                 rank=0, min_candidates=1)})
    cfg.update(changes)
    return cfg


def slots(*rows, **kwargs):
    source = kwargs.pop('source', 'front')
    frame = kwargs.pop('frame', None)
    data = dict(source=source, part='slots', slots=list(rows), **kwargs)
    if frame is not None:
        data['frame'] = frame
    return data


def slot(x=.8, y=-.45, yaw=0., slot_id=None, kind='parallel'):
    row = dict(x=x, y=y, yaw=yaw, kind=kind)
    if slot_id is not None:
        row['id'] = slot_id
    return row


class Scan(object):
    def __init__(self, stamp, obstacles=None, valid_rays=100):
        self.stamp = stamp
        self.pose = (0., 0., 0.)
        self.obstacles = list(obstacles or [])
        self.valid_rays = valid_rays

    def coverage(self, points):
        return 1.0


class SelectiveScan(Scan):
    """Fresh scan whose coverage deliberately omits the adjacent bay."""
    def coverage(self, points):
        return 1.0 if max(point[0] for point in points) < 1.3 else 0.0


class RecordingTask(object):
    phase = 'TRACK'
    reason = 'tracking'

    def __init__(self):
        self.scenes = []

    def observe(self, scene):
        self.scenes.append(scene)

    def command(self, now):
        return 12, 0.0


class SceneContext(object):
    def __init__(self, cfg, scan):
        self.cfg = cfg
        self.pose = (0., 0., 0.)
        self.parallel_scene = None
        self.parallel_parking = None
        self.scan = scan




def wire_scene(stamp, ready=None, occupancy=None):
    scene = dict(stamp=stamp, frame='measured_bay', pose_source='vision',
                 pose=[0., 0., 0.],
                 regions=[[[-2., -2.], [2., -2.], [2., 2.], [-2., 2.]]],
                 obstacles=[],
                 slot=dict(id='P1', pose=[0., .45, 0.],
                           length=.70, width=.36))
    if ready is not None:
        scene['ready'] = ready
    if occupancy is not None:
        scene['occupancy'] = occupancy
    return scene


class ParkingSceneTests(unittest.TestCase):
    def real_scan(self, stamp, vehicle_pose, obstacle=None):
        count = 360
        angle_min = -math.pi
        increment = 2 * math.pi / count
        ranges = [float('inf')] * count
        if obstacle is not None:
            relative = (obstacle[0] - vehicle_pose[0],
                        obstacle[1] - vehicle_pose[1])
            index = int(round((math.atan2(relative[1], relative[0]) -
                              angle_min) / increment))
            index %= count
            ranges[index] = math.hypot(relative[0], relative[1])
        return RealScan(ranges, angle_min, increment, .05, 6.0,
                        vehicle_pose, CONFIG['lidar'], stamp)

    def test_relative_slot_is_inverted_and_confirmed_in_fixed_bay_frame(self):
        builder = ParallelParkingScene(scene_cfg(parallel_parking_confirm_frames=3))
        observed = None
        for stamp, x in ((1., .80), (1.1, .62), (1.2, .44)):
            observed = builder.observe(
                (0., 0., 0.), slots(slot(x=x)), stamp,
                scan=Scan(stamp), pose_source='vision')
        self.assertTrue(observed['ready'])
        self.assertEqual(observed['frame'], 'measured_bay')
        self.assertEqual(observed['pose_source'], 'vision')
        self.assertEqual(observed['slot']['pose'], (0., 0., 0.))
        self.assertAlmostEqual(observed['pose'][0], -.44)
        self.assertAlmostEqual(observed['pose'][1], .45)
        self.assertEqual(observed['confirmations'], 3)
        road = observed['regions'][0]
        self.assertAlmostEqual(max(point[1] for point in road) -
                               min(point[1] for point in road), 1.20)
        self.assertAlmostEqual(max(point[0] for point in road), 1.56)
        self.assertEqual(invert_relative_pose((.8, -.45, 0.)), (-.8, .45, 0.))

    def test_auto_requires_one_designated_target(self):
        cfg = scene_cfg(parking_slot='AUTO')
        data = slots(slot(.7, slot_id=None), slot(1.4, slot_id=None))
        self.assertIsNone(build_parallel_scene(
            cfg, None, (0., 0., 0.), data, 1., scan=Scan(1.)))
        # An explicit target name is sufficient; geometry alone never picks
        # the nearest empty candidate in AUTO.
        selected = build_parallel_scene(
            cfg, None, (0., 0., 0.),
            slots(slot(.7), slot(1.4), target_id='P2'), 1.,
            scan=Scan(1.), pose_source='vision')
        self.assertEqual(selected['target_id'], 'P2')
        self.assertEqual(selected['slot']['id'], 'P2')

    def test_forward_plan_selects_p4_last_and_p5_first_from_two_perpendicular_bays(self):
        rows = slots(slot(.2, kind='perpendicular'),
                     slot(.8, kind='perpendicular'))
        for target, expected_x in (('P4', .8), ('P5', .2)):
            cfg = scene_cfg(parking_mode='forward_plan', parking_slot=target,
                            parallel_parking_confirm_frames=1)
            observed = build_parallel_scene(
                cfg, None, (0., 0., 0.), rows, 1., scan=Scan(1.),
                pose_source='vision')
            self.assertIsNotNone(observed)
            self.assertEqual(observed['target_id'], target)
            self.assertEqual(observed['slot']['kind'], 'perpendicular')
            self.assertAlmostEqual(observed['slot']['length'], .45)
            self.assertAlmostEqual(observed['slot']['width'], .38)
            self.assertAlmostEqual(observed['_relative_slot'][0], expected_x)

    def test_forward_plan_requires_two_same_kind_perpendicular_candidates(self):
        for target in ('P4', 'P5'):
            cfg = scene_cfg(parking_mode='forward_plan', parking_slot=target,
                            parallel_parking_confirm_frames=1)
            self.assertIsNone(build_parallel_scene(
                cfg, None, (0., 0., 0.),
                slots(slot(.2, kind='perpendicular')), 1., scan=Scan(1.),
                pose_source='vision'))
            mixed = slots(slot(.2, kind='perpendicular'), slot(.8))
            self.assertIsNone(build_parallel_scene(
                cfg, None, (0., 0., 0.), mixed, 1., scan=Scan(1.),
                pose_source='vision'))

    def test_forward_plan_target_hint_supplies_perpendicular_nominal_dimensions(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='AUTO',
                        parallel_parking_confirm_frames=1)
        rows = slots(slot(.2, kind='perpendicular'),
                     slot(.8, kind='perpendicular'), target_id='P4')
        observed = build_parallel_scene(
            cfg, None, (0., 0., 0.), rows, 1., scan=Scan(1.),
            pose_source='vision')
        self.assertEqual(observed['target_id'], 'P4')
        self.assertAlmostEqual(observed['slot']['length'], .45)
        self.assertAlmostEqual(observed['slot']['width'], .38)

    def test_forward_plan_rejects_geometry_outside_nominal_size_tolerance(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P4',
                        parallel_parking_confirm_frames=1)
        rows = slots(slot(.2, kind='perpendicular'),
                     slot(.8, kind='perpendicular'))
        for row in rows['slots']:
            row.update(length=.60, width=.38)
        self.assertIsNone(build_parallel_scene(
            cfg, None, (0., 0., 0.), rows, 1., scan=Scan(1.),
            pose_source='vision'))

    def test_forward_plan_uses_nominal_dimensions_after_within_tolerance_match(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P4',
                        parallel_parking_confirm_frames=1)
        rows = slots(slot(.2, kind='perpendicular'),
                     slot(.8, kind='perpendicular'))
        for row in rows['slots']:
            row.update(length=.49, width=.42)
        observed = build_parallel_scene(
            cfg, None, (0., 0., 0.), rows, 1., scan=Scan(1.),
            pose_source='vision')
        self.assertAlmostEqual(observed['slot']['length'], .45)
        self.assertAlmostEqual(observed['slot']['width'], .38)

    def test_target_only_scene_does_not_widen_exact_road_and_bay_region(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=1)
        observed = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8)), 1., scan=Scan(1.),
            pose_source='vision')
        self.assertEqual(len(observed['regions']), 2)
        self.assertEqual(observed['_neighbour_slots'], [])
        self.assertEqual(observed['region_evidence'], [])

    def test_occupied_or_unknown_neighbour_is_excluded_from_regions(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=1)
        rows = slots(slot(.8), slot(1.5))
        occupied = build_parallel_scene(
            cfg, None, (0., 0., 0.), rows, 1.,
            scan=Scan(1., obstacles=[(1.5, -.45)]), pose_source='vision')
        self.assertEqual(len(occupied['regions']), 2)
        self.assertEqual(occupied['region_evidence'][0]['occupancy'],
                         'OCCUPIED')
        self.assertFalse(occupied['region_evidence'][0]['included'])
        unknown = build_parallel_scene(
            cfg, None, (0., 0., 0.), rows, 1., scan=SelectiveScan(1.),
            pose_source='vision')
        self.assertEqual(len(unknown['regions']), 2)
        self.assertEqual(unknown['region_evidence'][0]['occupancy'],
                         'UNKNOWN')
        self.assertFalse(unknown['region_evidence'][0]['included'])

    def test_cached_neighbour_keeps_nominal_exact_dimensions(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=1)
        rows = slots(slot(.8), slot(1.5))
        rows['slots'][1].update(length=.74, width=.39)
        observed = build_parallel_scene(
            cfg, None, (0., 0., 0.), rows, 1., scan=Scan(1.),
            pose_source='vision')
        self.assertEqual(len(observed['_neighbour_slots']), 1)
        neighbour = observed['_neighbour_slots'][0]
        self.assertAlmostEqual(neighbour['length'], .70)
        self.assertAlmostEqual(neighbour['width'], .36)
        self.assertAlmostEqual(observed['region_evidence'][0]['length'], .70)
        self.assertAlmostEqual(observed['region_evidence'][0]['width'], .36)
        neighbour_region = observed['regions'][-1]
        self.assertAlmostEqual(math.hypot(
            neighbour_region[1][0] - neighbour_region[0][0],
            neighbour_region[1][1] - neighbour_region[0][1]), .70)
        self.assertAlmostEqual(math.hypot(
            neighbour_region[2][0] - neighbour_region[1][0],
            neighbour_region[2][1] - neighbour_region[1][1]), .36)

        # Evidence is public; geometry cache stays with the owning producer.
        wire = public_scene(observed)
        self.assertIn('region_evidence', wire)
        self.assertNotIn('_neighbour_slots', wire)

    def test_initial_unknown_neighbour_can_recover_on_fresh_partial_frame(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=1)
        first = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8), slot(1.5)), 1.,
            scan=SelectiveScan(1.), pose_source='vision')
        self.assertEqual(len(first['regions']), 2)
        self.assertEqual(first['region_evidence'][0]['occupancy'], 'UNKNOWN')
        ctx = SceneContext(cfg, Scan(1.1))
        ctx.parallel_scene = first
        lines = [[[.45, -.63], [1.15, -.63]],
                 [[.45, -.27], [1.15, -.27]],
                 [[1.15, -.63], [1.15, -.27]]]
        _observe_parallel_scene(
            ctx, dict(source='front', part='slots', slots=[], lines=lines), 1.1)
        self.assertEqual(len(ctx.parallel_scene['regions']), 3)
        self.assertEqual(ctx.parallel_scene['region_evidence'][0]['occupancy'],
                         'FREE')

    def test_invalid_target_geometry_status_cannot_establish_scene(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=1)
        rows = slots(slot(.8), slot(1.5))
        rows['slots'][0]['geometry_status'] = 'invalid'
        observed = build_parallel_scene(
            cfg, None, (0., 0., 0.), rows, 1., scan=Scan(1.),
            pose_source='vision')
        self.assertIsNone(observed)
    def test_partial_refresh_rechecks_cached_neighbour_occupancy(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=1)
        first = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8), slot(1.5)), 1.,
            scan=Scan(1.), pose_source='vision')
        self.assertEqual(len(first['_neighbour_slots']), 1)
        lines = [[[.45, -.63], [1.15, -.63]],
                 [[.45, -.27], [1.15, -.27]],
                 [[1.15, -.63], [1.15, -.27]]]
        ctx = SceneContext(cfg, Scan(1.1, obstacles=[(1.5, -.45)]))
        ctx.parallel_scene = first
        _observe_parallel_scene(
            ctx, dict(source='front', part='slots', slots=[], lines=lines), 1.1)
        self.assertEqual(len(ctx.parallel_scene['regions']), 2)
        self.assertEqual(ctx.parallel_scene['region_evidence'][0]['occupancy'],
                         'OCCUPIED')
        self.assertEqual(len(ctx.parallel_scene['_neighbour_slots']), 1)
        ctx.scan = Scan(1.2)
        _observe_parallel_scene(
            ctx, dict(source='front', part='slots', slots=[], lines=lines), 1.2)
        self.assertEqual(len(ctx.parallel_scene['regions']), 3)
        self.assertEqual(ctx.parallel_scene['region_evidence'][0]['occupancy'],
                         'FREE')
        self.assertTrue(ctx.parallel_scene['region_evidence'][0]['included'])

    def test_forward_plan_perpendicular_heading_points_into_each_bay(self):
        for y, expected in ((-.45, -math.pi / 2),
                            (.45, math.pi / 2)):
            cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P4',
                            parallel_parking_confirm_frames=1)
            rows = slots(slot(.2, y=y, yaw=math.pi / 2,
                              kind='perpendicular'),
                         slot(.8, y=y, yaw=math.pi / 2,
                              kind='perpendicular'))
            observed = build_parallel_scene(
                cfg, None, (0., 0., 0.), rows, 1., scan=Scan(1.),
                pose_source='vision')
            self.assertAlmostEqual(observed['_relative_slot'][2], expected)

    def test_forward_plan_is_front_only_in_scene_builder_and_observations(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P4',
                        parallel_parking_confirm_frames=1)
        row = slots(slot(.2, kind='perpendicular', slot_id='P4'))
        self.assertIsNone(build_parallel_scene(
            cfg, None, (0., 0., 0.),
            dict(row, source='rear', frame='base_link'), 1.,
            scan=Scan(1.), pose_source='vision'))
        ctx = SceneContext(cfg, Scan(1.))
        _observe_parallel_scene(ctx, row, 1.)
        self.assertIsNotNone(ctx.parallel_scene)
        _observe_parallel_scene(
            ctx, dict(row, source='rear', frame='base_link'), 1.1)
        self.assertEqual(ctx.parallel_scene['stamp'], 1.)

    def test_locked_forward_scene_accepts_bounded_far_edge_partial_refresh(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=1)
        previous = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8)), 1., scan=Scan(1.),
            pose_source='vision')
        lines = [[[.45, -.63], [1.15, -.63]],
                 [[.45, -.27], [1.15, -.27]],
                 [[1.15, -.63], [1.15, -.27]]]
        partial = _partial_slot_from_lines(previous, lines, cfg)
        self.assertEqual(partial['geometry_status'], 'supported_partial')
        ctx = SceneContext(cfg, Scan(1.1))
        ctx.parallel_scene = previous
        _observe_parallel_scene(
            ctx, dict(source='front', part='slots', slots=[], lines=lines), 1.1)
        self.assertEqual(ctx.parallel_scene['target_id'], 'P1')
        self.assertEqual(ctx.parallel_scene['geometry_status'],
                         'supported_partial')
        self.assertEqual(ctx.parallel_scene['observation_source'], 'front')
        self.assertAlmostEqual(ctx.parallel_scene['_relative_slot'][0], .8)

    def test_partial_lines_do_not_establish_or_erase_unconfirmed_target(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=3)
        previous = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8)), 1., scan=Scan(1.),
            pose_source='vision')
        self.assertEqual(previous['confirmations'], 1)
        lines = [[[.45, -.63], [1.15, -.63]],
                 [[.45, -.27], [1.15, -.27]],
                 [[1.15, -.63], [1.15, -.27]]]
        ctx = SceneContext(cfg, Scan(1.1))
        ctx.parallel_scene = previous
        _observe_parallel_scene(
            ctx, dict(source='front', part='slots', slots=[], lines=lines), 1.1)
        self.assertEqual(ctx.parallel_scene['stamp'], 1.)
        self.assertEqual(ctx.parallel_scene['confirmations'], 1)

    def test_partial_lines_require_two_sides_and_far_edge(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=1)
        previous = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8)), 1., scan=Scan(1.),
            pose_source='vision')
        lines = [[[.45, -.63], [1.15, -.63]],
                 [[.45, -.27], [1.15, -.27]]]
        self.assertIsNone(_partial_slot_from_lines(previous, lines, cfg))
        ctx = SceneContext(cfg, Scan(1.1))
        ctx.parallel_scene = previous
        _observe_parallel_scene(
            ctx, dict(source='front', part='slots', slots=[], lines=lines), 1.1)
        self.assertEqual(ctx.parallel_scene['stamp'], 1.)

    def test_same_exposure_slots_and_lines_refresh_once_from_complete_slots(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P1',
                        parallel_parking_confirm_frames=3)
        previous = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8)), 1., scan=Scan(1.),
            pose_source='vision')
        lines = [[[.45, -.63], [1.15, -.63]],
                 [[.45, -.27], [1.15, -.27]],
                 [[1.15, -.63], [1.15, -.27]]]
        ctx = SceneContext(cfg, Scan(1.1))
        ctx.parallel_scene = previous
        _observe_parallel_scene(
            ctx, dict(source='front', part='slots', slots=[slot(.8)],
                      lines=lines), 1.1)
        self.assertEqual(ctx.parallel_scene['confirmations'], 2)
        self.assertEqual(ctx.parallel_scene['geometry_status'], 'complete')

    def test_perpendicular_region_runs_from_negative_length_mouth_to_front(self):
        cfg = scene_cfg(parking_mode='forward_plan')
        slot_model = dict(pose=(0., 0., math.pi / 2), length=.45,
                          width=.38, kind='perpendicular')
        vehicle = (1., 0., math.pi / 2)
        regions = _region(vehicle, slot_model, cfg)
        road, bay = regions
        road_local = [tuple(round(v, 8) for v in local(slot_model['pose'], p))
                      for p in road]
        bay_local = [local(slot_model['pose'], p) for p in bay]
        self.assertAlmostEqual(min(p[0] for p in road_local), -1.425)
        self.assertAlmostEqual(max(p[0] for p in road_local), -.225)
        self.assertLess(min(p[0] for p in road_local), -.225)
        self.assertAlmostEqual(min(p[1] for p in road_local), -3.0)
        self.assertAlmostEqual(max(p[1] for p in road_local), 1.0)
        self.assertAlmostEqual(min(p[0] for p in bay_local), -.225)
        self.assertAlmostEqual(max(p[0] for p in bay_local), .225)

    def test_locked_perpendicular_region_does_not_follow_lateral_pose_drift(self):
        cfg = scene_cfg(parking_mode='forward_plan', parking_slot='P4',
                        parallel_parking_confirm_frames=1)
        first = build_parallel_scene(
            cfg, None, (0., 0., 0.),
            slots(slot(.2, y=-1., yaw=math.pi / 2,
                       kind='perpendicular'),
                  slot(.8, y=-1., yaw=math.pi / 2,
                       kind='perpendicular')),
            1., scan=Scan(1.), pose_source='vision')
        current = build_parallel_scene(
            cfg, first, (0., .05, 0.),
            slots(slot(.2, y=-1.05, yaw=math.pi / 2,
                       kind='perpendicular'),
                  slot(.8, y=-1.05, yaw=math.pi / 2,
                       kind='perpendicular')),
            1.1, scan=Scan(1.1), pose_source='vision')
        self.assertIsNotNone(current)
        self.assertEqual(current['regions'], first['regions'])

    def test_lidar_occupancy_blocks_ready_scene(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=1)
        # (.8,-.45) in the vehicle frame maps to the observed bay origin.
        observed = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot()), 1.,
            scan=Scan(1., obstacles=[(.8, -.45)]), pose_source='vision')
        self.assertEqual(observed['occupancy'], 'OCCUPIED')
        self.assertFalse(observed['ready'])

    def test_odom_keeps_fixed_slot_while_vehicle_pose_changes(self):
        cfg = scene_cfg(pose_mode='odom', parallel_parking_confirm_frames=2)
        previous = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8)), 1.,
            scan=Scan(1.), pose_source='odom')
        current = build_parallel_scene(
            cfg, previous, (.2, 0., 0.), slots(slot(.6)), 1.1,
            scan=Scan(1.1), pose_source='odom')
        self.assertEqual(current['pose'], (.2, 0., 0.))
        self.assertAlmostEqual(current['slot']['pose'][0], .8)
        self.assertAlmostEqual(current['slot']['pose'][1], -.45)
        self.assertTrue(current['ready'])
        self.assertEqual(current['pose_source'], 'odom')

    def test_command_model_cannot_be_called_a_measured_source(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=1)
        self.assertIsNone(build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot()), 1.,
            scan=Scan(1.), pose_source='command_model'))

    def test_command_model_cannot_relabel_external_pose_sources(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=1)
        for source in ('odom', 'fused', 'lidar'):
            self.assertIsNone(build_parallel_scene(
                cfg, None, (0., 0., 0.), slots(slot()), 1.,
                scan=Scan(1.), pose_source=source), source)

    def test_unlabelled_requested_target_requires_all_configured_candidates(self):
        cfg = scene_cfg(parking_slot='P1', parallel_parking_confirm_frames=1)
        cfg['slots']['P1']['min_candidates'] = 3
        self.assertIsNone(build_parallel_scene(
            cfg, None, (0., 0., 0.),
            slots(slot(.7), slot(1.4)), 1., scan=Scan(1.),
            pose_source='vision'))

    def test_rear_partial_tracks_only_the_locked_front_geometry(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=3)
        cfg['slots']['P1']['min_candidates'] = 3
        complete = slots(slot(.7), slot(1.4), slot(2.1))
        scene = None
        for stamp in (1.0, 1.1, 1.2):
            scene = build_parallel_scene(
                cfg, scene, (0., 0., 0.), complete, stamp,
                scan=Scan(stamp), pose_source='vision')
        self.assertTrue(scene['ready'])
        # A rear frame cannot establish a target before front confirmation.
        self.assertIsNone(build_parallel_scene(
            cfg, None, (0., 0., 0.),
            slots(slot(.3), source='rear', frame='base_link'), 1.0,
            scan=Scan(1.0), pose_source='vision'))
        tracked = build_parallel_scene(
            cfg, scene, (.1, 0., 0.),
            slots(slot(.6), source='rear', frame='base_link'), 1.3,
            scan=Scan(1.3), pose_source='vision')
        self.assertIsNotNone(tracked)
        self.assertEqual(tracked['target_id'], 'P1')
        self.assertEqual(tracked['confirmations'], 4)
        self.assertIsNone(build_parallel_scene(
            cfg, scene, (.1, 0., 0.),
            slots(slot(1.0), source='rear', frame='base_link'), 1.4,
            scan=Scan(1.4), pose_source='vision'))
        self.assertIsNone(build_parallel_scene(
            cfg, scene, (.1, 0., 0.),
            slots(slot(.60), slot(.61), source='rear', frame='base_link'), 1.5,
            scan=Scan(1.5), pose_source='vision'))
        self.assertIsNone(build_parallel_scene(
            cfg, scene, (1.0, 0., 0.),
            slots(slot(-.2), source='rear', frame='base_link'), 1.6,
            scan=Scan(1.6), pose_source='vision'))

    def test_unknown_front_latch_recovers_with_rear_free_and_empty_front(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=2)
        builder = ParallelParkingScene(cfg)
        front = slots(slot(.8), frame='base_link')
        first = builder.observe((0., 0., 0.), front, 1.0,
                                scan=None, pose_source='vision')
        self.assertEqual(first['occupancy'], 'UNKNOWN')
        locked = builder.observe((0., 0., 0.), front, 1.1,
                                 scan=None, pose_source='vision')
        self.assertFalse(locked['ready'])
        self.assertTrue(locked['geometry_locked'])
        # A later empty front image cannot release the geometry identity.
        self.assertIsNone(builder.observe(
            (0., 0., 0.), slots(frame='base_link'), 1.2,
            scan=None, pose_source='vision'))
        self.assertEqual(builder.scene['stamp'], 1.1)
        recovered = builder.observe(
            (0., 0., 0.), slots(slot(.8), source='rear', frame='base_link'),
            1.3, scan=Scan(1.3), pose_source='vision')
        self.assertTrue(recovered['ready'])
        self.assertEqual(recovered['target_id'], 'P1')

    def test_scan_future_stamp_is_motion_compensated_and_late_scan_unknown(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=1,
                        lidar=copy.deepcopy(CONFIG['lidar']))
        cfg['slots']['P1']['min_candidates'] = 1
        data = slots(slot(.8))
        # The camera pose is at t=1.0; the scan was captured 0.2 s later
        # after a nonzero motion step.  The world obstacle still maps into
        # the measured bay and must block it.
        scan = self.real_scan(1.2, (.25, 0., 0.), obstacle=(1.0, -.45))
        observed = build_parallel_scene(
            cfg, None, (.2, 0., 0.), data, 1.0, scan=scan,
            pose_source='vision')
        self.assertEqual(observed['occupancy'], 'OCCUPIED')
        self.assertTrue(observed['occupancy_detail']['fresh'])
        late = self.real_scan(1.8, (.25, 0., 0.))
        delayed = build_parallel_scene(
            cfg, None, (.2, 0., 0.), data, 1.0, scan=late,
            pose_source='vision')
        self.assertEqual(delayed['occupancy'], 'UNKNOWN')
        self.assertFalse(delayed['ready'])
        stale_obstacle = self.real_scan(
            1.8, (.25, 0., 0.), obstacle=(1.0, -.45))
        delayed_obstacle = build_parallel_scene(
            cfg, None, (.2, 0., 0.), data, 1.0, scan=stale_obstacle,
            pose_source='vision')
        self.assertEqual(delayed_obstacle['occupancy'], 'UNKNOWN')
        self.assertFalse(delayed_obstacle['occupancy_detail']['fresh'])

    def test_camera_observation_publishes_one_scene_after_confirmation(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, parking_mode='parallel_reverse',
                   parking_slot='P1', lidar_enabled=True,
                   parallel_parking_confirm_frames=2)
        cfg['slots']['P1']['min_candidates'] = 1
        core = Controller(cfg)
        self.addCleanup(core.close)
        core.scan = Scan(10.)
        frame = slots(slot(.8))
        core.observe_ground(frame, 10.)
        self.assertFalse(core.parallel_scene['ready'])
        core.observe_ground(frame, 10.1)
        self.assertTrue(core.parallel_scene['ready'])
        self.assertEqual(core.parallel_scene['slot']['id'], 'P1')
        self.assertEqual(core.parallel_scene['pose_source'], 'vision')

    def test_rear_empty_frame_does_not_clear_front_confirmation(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, parking_mode='parallel_reverse',
                   parking_slot='P1', lidar_enabled=True,
                   parallel_parking_confirm_frames=2)
        cfg['slots']['P1']['min_candidates'] = 1
        core = Controller(cfg)
        self.addCleanup(core.close)
        core.scan = Scan(10.)
        front = slots(slot(.8), frame='base_link')
        core.observe_ground(front, 10.)
        self.assertFalse(core.parallel_scene['ready'])
        first = core.parallel_scene
        core.observe_ground(
            slots(slot(.2), source='rear', frame='base_link'), 10.1)
        self.assertEqual(core.parallel_scene['stamp'], first['stamp'])
        self.assertEqual(core.parallel_scene['slot'], first['slot'])
        self.assertEqual(core.parallel_scene['confirmations'], 1)
        core.observe_ground(front, 10.2)
        self.assertTrue(core.parallel_scene['ready'])
        core.set_pose((.1, 0., 0.), 10.25)
        core.observe_ground(
            slots(slot(.7), source='rear', frame='base_link'), 10.3)
        self.assertEqual(core.parallel_scene['target_id'], 'P1')
        self.assertGreater(core.parallel_scene['confirmations'], 2)

    def test_camera_updates_active_task_with_new_measured_pose(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, parking_mode='parallel_reverse',
                   parking_slot='P1', lidar_enabled=True,
                   parallel_parking_confirm_frames=1,
                   parallel_parking_slot_drift_m=.25)
        cfg['slots']['P1']['min_candidates'] = 1
        core = Controller(cfg)
        self.addCleanup(core.close)
        task = RecordingTask()
        # Inject only the task object; the real parallel module still validates
        # and routes the scene through its declared observe operation.
        core._state_data['parallel_parking'] = task
        core.scan = Scan(10.)
        core.observe_ground(slots(slot(.8)), 10.)
        core.scan.obstacles = [(.6, -.45)]
        core.observe_ground(slots(slot(.6)), 10.1)
        self.assertEqual(len(task.scenes), 2)
        self.assertNotEqual(task.scenes[0]['pose'], task.scenes[1]['pose'])
        self.assertAlmostEqual(task.scenes[1]['pose'][0], -.6)

    def test_duplicate_stamp_does_not_confirm_again(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=3)
        first = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot()), 1., scan=Scan(1.),
            pose_source='vision')
        duplicate = build_parallel_scene(
            cfg, first, (0., 0., 0.), slots(slot()), 1., scan=Scan(1.),
            pose_source='vision')
        self.assertIsNone(duplicate)
        self.assertEqual(first['confirmations'], 1)

    def test_public_scene_round_trip_keeps_confirmation_state(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=3)
        first = build_parallel_scene(
            cfg, None, (0., 0., 0.), slots(slot(.8)), 1., scan=Scan(1.),
            pose_source='vision')
        second = build_parallel_scene(
            cfg, public_scene(first), (0., 0., 0.), slots(slot(.6)), 1.1,
            scan=Scan(1.1), pose_source='vision')
        self.assertEqual(second['confirmations'], 2)

    def test_relative_jump_resets_unconfirmed_latch(self):
        builder = ParallelParkingScene(scene_cfg(parallel_parking_confirm_frames=3))
        first = builder.observe(
            (0., 0., 0.), slots(slot(.8)), 1., scan=Scan(1.),
            pose_source='vision')
        self.assertEqual(first['confirmations'], 1)
        self.assertIsNone(builder.observe(
            (0., 0., 0.), slots(slot(1.5)), 1.1, scan=Scan(1.1),
            pose_source='vision'))
        restarted = builder.observe(
            (0., 0., 0.), slots(slot(.9)), 1.2, scan=Scan(1.2),
            pose_source='vision')
        self.assertEqual(restarted['confirmations'], 1)

    def test_valid_rays_without_coverage_are_unknown(self):
        class IncompleteScan(object):
            stamp = 1.
            valid_rays = 100
            obstacles = []
        observed = build_parallel_scene(
            scene_cfg(parallel_parking_confirm_frames=1), None,
            (0., 0., 0.), slots(slot()), 1., scan=IncompleteScan(),
            pose_source='vision')
        self.assertEqual(observed['occupancy'], 'UNKNOWN')
        self.assertFalse(observed['ready'])

    def test_lidar_free_requires_fresh_stamp_rays_pose_and_extrinsic(self):
        cfg = scene_cfg(parallel_parking_confirm_frames=1)

        class MissingStamp(object):
            pose = (0., 0., 0.)
            valid_rays = 100
            obstacles = []
            def coverage(self, points):
                return 1.0

        class MissingPose(object):
            stamp = 1.
            valid_rays = 100
            obstacles = []
            def coverage(self, points):
                return 1.0

        cases = [
            (Scan(1., valid_rays=0), cfg),
            (MissingStamp(), cfg),
            (MissingPose(), cfg),
            (Scan(1.), dict(cfg, lidar=dict(cfg['lidar'], x='bad'))),
        ]
        for scan, case_cfg in cases:
            observed = build_parallel_scene(
                case_cfg, None, (0., 0., 0.), slots(slot()), 1.,
                scan=scan, pose_source='vision')
            self.assertEqual(observed['occupancy'], 'UNKNOWN', repr(scan))
            self.assertFalse(observed['ready'])

    def test_active_task_stops_on_unknown_and_recovers_on_new_free_scene(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, parking_mode='parallel_reverse',
                   parking_slot='P1')
        core = Controller(cfg)
        self.addCleanup(core.close)
        task = RecordingTask()
        core._state_data['parallel_parking'] = task
        core.scan = Scan(10.)
        core.checked_command = lambda command, now, allow: command

        core.observe_parallel_scene(
            wire_scene(10., ready=False, occupancy='UNKNOWN'), 10.)
        self.assertEqual(core.parallel_parking_tick(10.), (0, 0))
        self.assertEqual(core.reason, 'parallel_parking_scene_unconfirmed')

        core.observe_parallel_scene(
            wire_scene(10.1, ready=True, occupancy='FREE'), 10.1)
        self.assertEqual(core.parallel_parking_tick(10.1), (12, 0.0))

    def test_active_task_stops_when_scene_metadata_is_missing(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, parking_mode='parallel_reverse',
                   parking_slot='P1')
        core = Controller(cfg)
        self.addCleanup(core.close)
        core._state_data['parallel_parking'] = RecordingTask()
        core.scan = Scan(10.)
        core.checked_command = lambda command, now, allow: command
        core.observe_parallel_scene(wire_scene(10.), 10.)
        self.assertEqual(core.parallel_parking_tick(10.), (0, 0))
        self.assertEqual(core.reason, 'parallel_parking_scene_unconfirmed')

    def test_active_task_stops_without_a_production_scene(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.update(wait_green=False, parking_mode='parallel_reverse',
                   parking_slot='P1')
        core = Controller(cfg)
        self.addCleanup(core.close)
        core._state_data['parallel_parking'] = RecordingTask()
        core.scan = Scan(10.)
        core.checked_command = lambda command, now, allow: command
        self.assertEqual(core.parallel_parking_tick(10.), (0, 0))
        self.assertEqual(core.reason, 'parallel_parking_scene_unconfirmed')


if __name__ == '__main__':
    unittest.main()
