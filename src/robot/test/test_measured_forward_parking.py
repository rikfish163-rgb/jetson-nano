"""Closed-loop, no-actuator checks for the measured forward parking task."""
from __future__ import division

import copy
import os
import unittest

import yaml

from robot.common.geometry import bicycle
from robot.common.geometry import world
from robot.parallel_parking.planner import ForwardParking
from robot.parallel_parking.scene import _region


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
with open(os.path.join(ROOT, 'config', 'competition.yaml')) as stream:
    CONFIG = yaml.safe_load(stream)


def _cfg(slot_id):
    cfg = copy.deepcopy(CONFIG)
    cfg.update(parking_mode='forward_plan', parking_slot=slot_id,
               parallel_parking_confirm_frames=3,
               parallel_parking_slot_body_margin_m=.01,
               parallel_parking_goal_position_tolerance_m=.03,
               parallel_parking_goal_yaw_tolerance_deg=10.)
    return cfg


def _scene(slot, pose, regions, stamp=1., ready=True, occupancy='FREE',
           obstacles=()):
    return dict(frame='measured_bay', pose=tuple(pose), stamp=stamp,
                pose_source='vision', observation_source='front',
                regions=regions, obstacles=list(obstacles), slot=slot,
                target_id=slot['id'], confirmations=3, ready=ready,
                occupancy=occupancy)


def _production_regions(cfg, slot, pose, neighbor_center=None):
    """Use the measured road plus exact bay rectangles from the scene builder."""
    regions = _region(pose, slot, cfg)
    if not regions:
        raise AssertionError('production scene corridor is empty')
    if neighbor_center is not None:
        half_length = slot['length'] / 2.0
        half_width = slot['width'] / 2.0
        regions.append([world(slot['pose'], (neighbor_center - half_length,
                                             -half_width)),
                        world(slot['pose'], (neighbor_center + half_length,
                                             -half_width)),
                        world(slot['pose'], (neighbor_center + half_length,
                                             half_width)),
                        world(slot['pose'], (neighbor_center - half_length,
                                             half_width))])
    return regions


def _prepared_parallel_start(cfg):
    del cfg
    return (-1.2, .4, 0.)


class MeasuredForwardParkingTests(unittest.TestCase):
    def _run_bicycle(self, task, pose, stamp=1.):
        """Advance a real bicycle pose until TRACK gives way to CONFIRM."""
        for unused in range(1400):
            speed, steering = task.command(stamp)
            if task.phase != 'TRACK':
                break
            self.assertGreater(speed, 0)
            raw_to_mps = task.cfg['raw_to_mps']['forward']
            pose = bicycle(pose, speed * raw_to_mps * .05,
                           steering, task.cfg['wheelbase'])
            stamp += .05
            task.observe(_scene(task.slot, pose, task.scene['regions'], stamp,
                                obstacles=task.scene['obstacles']))
        self.assertEqual(task.phase, 'CONFIRM')
        return pose, stamp

    def _confirm(self, task, pose, stamp):
        for unused in range(4):
            stamp += .02
            task.observe(_scene(task.slot, pose, task.scene['regions'], stamp,
                                obstacles=task.scene['obstacles']))
            self.assertEqual(task.command(stamp), (0, 0.0))
        self.assertEqual(task.phase, 'DONE')

    def test_parallel_bay_bicycle_loop_reaches_four_wheel_confirm(self):
        slot = dict(id='P1', kind='parallel', pose=(0., 0., 0.),
                    length=.70, width=.36)
        cfg = _cfg('P1')
        start = _prepared_parallel_start(cfg)
        # The neighboring bay is a separately measured FREE exact rectangle;
        # it is evidence for the prepared approach, never a widened road.
        regions = _production_regions(cfg, slot, start, neighbor_center=-.70)
        task = ForwardParking(cfg, _scene(slot, start, regions))
        planned = task.plan()
        self.assertTrue(planned[0], planned[1])
        self.assertTrue(all(row[3] == 1 for row in planned[0]))
        task.accept_plan(planned)
        pose, stamp = self._run_bicycle(task, start)
        self._confirm(task, pose, stamp)

    def test_perpendicular_aligned_bay_uses_forward_straight_bicycle_loop(self):
        slot = dict(id='P4', kind='perpendicular', pose=(0., 0., 0.),
                    length=.45, width=.38)
        start = (-.8, 0., 0.)
        regions = _production_regions(_cfg('P4'), slot, start)
        task = ForwardParking(_cfg('P4'), _scene(slot, start, regions))
        planned = task.plan()
        self.assertTrue(planned[0], planned[1])
        self.assertEqual(planned[1], 'forward_parking_straight')
        self.assertTrue(all(row[3] == 1 for row in planned[0]))
        task.accept_plan(planned)
        pose, stamp = self._run_bicycle(task, start)
        self._confirm(task, pose, stamp)

    def test_unknown_measurement_stops_and_free_frame_can_resume(self):
        slot = dict(id='P1', kind='parallel', pose=(0., 0., 0.),
                    length=.70, width=.36)
        cfg = _cfg('P1')
        start = _prepared_parallel_start(cfg)
        regions = _production_regions(cfg, slot, start, neighbor_center=-.70)
        task = ForwardParking(cfg, _scene(slot, start, regions))
        task.accept_plan(task.plan())
        unknown = _scene(slot, start, regions, 1.1, ready=False,
                         occupancy='UNKNOWN')
        task.observe(unknown)
        self.assertEqual(task.command(1.1), (0, 0.0))
        self.assertEqual(task.reason, 'parallel_parking_scene_unconfirmed')
        free = _scene(slot, start, regions, 1.2)
        task.observe(free)
        self.assertGreater(task.command(1.2)[0], 0)

    def test_common_parallel_approach_stops_when_exact_corridor_has_no_path(self):
        slot = dict(id='P1', kind='parallel', pose=(0., 0., 0.),
                    length=.70, width=.36)
        cfg = _cfg('P1')
        start = (-1.2, .4, 0.)
        regions = _production_regions(cfg, slot, start)
        task = ForwardParking(cfg, _scene(slot, start, regions))
        path, reason = task.plan()
        self.assertFalse(path)
        self.assertTrue(reason.startswith('forward_parking_'))
        task.accept_plan((path, reason))
        self.assertEqual(task.phase, 'BLOCKED')
        self.assertEqual(task.command(1.1), (0, 0.0))

    def test_blocked_start_fails_closed_without_reverse_fallback(self):
        slot = dict(id='P4', kind='perpendicular', pose=(0., 0., 0.),
                    length=.45, width=.38)
        start = (-.8, 0., 0.)
        regions = _production_regions(_cfg('P4'), slot, start)
        task = ForwardParking(
            _cfg('P4'), _scene(slot, start, regions, obstacles=[(-.8, 0.)]))
        path, reason = task.plan()
        self.assertFalse(path)
        self.assertEqual(reason, 'forward_parking_start_blocked')
        task.accept_plan((path, reason))
        self.assertEqual(task.phase, 'BLOCKED')
        self.assertEqual(task.command(1.1), (0, 0.0))

    def test_forward_task_requires_front_observation_provenance(self):
        slot = dict(id='P4', kind='perpendicular', pose=(0., 0., 0.),
                    length=.45, width=.38)
        start = (-.8, 0., 0.)
        regions = _production_regions(_cfg('P4'), slot, start)
        scene = _scene(slot, start, regions)
        scene.pop('observation_source')
        task = ForwardParking(_cfg('P4'), scene)
        task.accept_plan(task.plan())
        self.assertEqual(task.command(1.1), (0, 0.0))
        self.assertEqual(task.reason, 'parallel_parking_scene_unconfirmed')


if __name__ == '__main__':
    unittest.main()
