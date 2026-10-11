#!/usr/bin/env python2
"""Offline checks for the forward/right one-shot test; no ROS output."""
import unittest
import forward_right_test as target
from open_loop_core import OneShot


class ForwardRightTest(unittest.TestCase):
    def test_duration_overrides(self):
        rows = target.make_stages(straight_seconds=.8, right_seconds=1.2)
        self.assertEqual(tuple(rows[1])[1:], (.8, 30, 0))
        self.assertEqual(tuple(rows[2])[1:], (1.2, 30, -22))

    def test_invalid_duration(self):
        for value in (0, -1, float('nan'), float('inf'), 11):
            with self.assertRaises(ValueError):
                target.make_stages(straight_seconds=value)
            with self.assertRaises(ValueError):
                target.make_stages(right_seconds=value)

    def test_timing_and_final_stop(self):
        task = OneShot(target.make_stages())
        changes, previous = [], None
        for i in range(101):
            command = task.tick(i / 20.)
            if command != previous:
                changes.append((i / 20., command))
                previous = command
        self.assertEqual(changes, [(0., (0, 0)), (3., (30, 0)),
                                   (3.5, (30, -22)), (4., (0, 0))])
        self.assertTrue(task.finished)
        self.assertEqual(task.tick(100.), (0, 0))

    def test_control_gap_aborts_motion(self):
        task = OneShot(target.make_stages())
        for i in range(62):
            task.tick(i / 20.)
        self.assertEqual(task.tick(3.5), (0, 0))
        self.assertEqual(task.reason, 'CONTROL_LOOP_GAP')
        self.assertEqual(task.tick(3.55), (0, 0))

    def test_preview_never_executes(self):
        original = target.execute
        def forbidden(*args, **kwargs):
            self.fail('preview must not invoke hardware executor')
        target.execute = forbidden
        try:
            self.assertEqual(target.main([]), 0)
        finally:
            target.execute = original

    def test_execute_uses_existing_isolated_bridge(self):
        calls, original = [], target.execute
        def capture(stages, **kwargs):
            calls.append((stages, kwargs))
            return 0
        target.execute = capture
        try:
            self.assertEqual(target.main(['--execute']), 0)
        finally:
            target.execute = original
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0][1]['auto_start'])
        self.assertEqual(calls[0][1]['bridge_node'], '/right_test_bridge')
        self.assertEqual(calls[0][1]['command_topic'], '/right/two_stage_cmd')


if __name__ == '__main__':
    unittest.main()
