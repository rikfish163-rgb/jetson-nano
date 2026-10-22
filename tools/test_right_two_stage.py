#!/usr/bin/env python2
"""Offline checks; no ROS initialization or hardware commands."""
import unittest
import right_two_stage_test as target
from open_loop_core import OneShot, require_ownership


class RightTest(unittest.TestCase):
    def test_full_sequence_and_terminal_stop(self):
        stages = target.make_stages(target.arguments([]))
        self.assertEqual([(r.speed, r.steering, r.seconds) for r in stages],
                         [(0, 0, 3.), (-30, 0, 1.), (30, -22, 3.), (0, 0, .7)])
        task = OneShot(stages)
        changes = []
        old = None
        for i in range(161):
            command = task.tick(i / 20.)
            if command != old:
                changes.append((i / 20., command))
                old = command
        self.assertEqual(changes, [(0., (0, 0)), (3., (-30, 0)),
                                   (4., (30, -22)), (7., (0, 0))])
        self.assertTrue(task.finished)
        self.assertEqual(task.tick(100.), (0, 0))

    def test_loop_gap_stops_without_resuming(self):
        task = OneShot(target.make_stages(target.arguments([])))
        for i in range(62):
            task.tick(i / 20.)
        self.assertEqual(task.tick(3.5), (0, 0))
        self.assertEqual(task.reason, 'CONTROL_LOOP_GAP')
        self.assertEqual(task.tick(3.55), (0, 0))

    def test_invalid_parameters(self):
        for flag, value in [('--reverse-speed', '-30'), ('--forward-speed', '31'),
                            ('--forward-steering', '-23'), ('--reverse-seconds', '0'),
                            ('--forward-seconds', 'nan'), ('--countdown', 'inf')]:
            with self.assertRaises(ValueError):
                target.make_stages(target.arguments([flag, value]))

    def test_overrides(self):
        rows = target.make_stages(target.arguments(
            ['--reverse-seconds', '1.2', '--forward-seconds', '2.5',
             '--forward-steering', '-18']))
        self.assertEqual(tuple(rows[1])[1:], (1.2, -30, 0))
        self.assertEqual(tuple(rows[2])[1:], (2.5, 30, -18))

    def test_preview_does_not_execute(self):
        original = target.execute
        def forbidden(*a, **kw):
            self.fail('preview invoked executor')
        target.execute = forbidden
        try:
            self.assertEqual(target.main([]), 0)
        finally:
            target.execute = original

    def test_dedicated_control_ownership(self):
        own = '/right_test'
        bridge = target.PROFILE['bridge_node']
        cmd = target.PROFILE['command_topic']
        pubs = {cmd: [own], '/ackermann_cmd': [bridge],
                '/base_controller/status': ['/base_controller']}
        subs = {cmd: [bridge], '/ackermann_cmd': ['/base_controller']}
        require_ownership(pubs, subs, own, command=cmd, bridge=bridge)
        pubs['/control/cmd'] = ['/competition_controller']
        with self.assertRaises(ValueError):
            require_ownership(pubs, subs, own, command=cmd, bridge=bridge)


if __name__ == '__main__':
    unittest.main()
