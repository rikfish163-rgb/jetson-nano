#!/usr/bin/env python2
"""Offline checks for the one-second left/straight command sequence."""
import unittest
import threading
import time

import left_straight_test as target
from open_loop_core import require_ownership


class LeftStraightTest(unittest.TestCase):
    def test_each_stage_accepts_independent_signed_commands(self):
        stages = target.make_stages(2., 1., 1., entry_speed=-30, entry_steer=-22,
                                    turn_speed=30, turn_steer=22,
                                    straight_speed=-20, straight_steer=0)
        self.assertEqual([(s.seconds, s.speed, s.steering) for s in stages[1:-1]],
                         [(1., -30, -22), (2., 30, 22), (1., -20, 0)])
        task, previous, changes = target.OutputTimedRun(stages), (0, 0), []
        for i in range(181):
            now = i*.05
            task.observe(now, *previous)
            command = task.tick(now)
            if command != previous:
                changes.append((now, command))
                previous = command
        self.assertEqual([cmd for _, cmd in changes],
                         [(-30, -22), (30, 22), (-20, 0), (0, 0)])
        for i, duration in enumerate((1., 2., 1.)):
            self.assertGreaterEqual(changes[i+1][0]-changes[i][0], duration)
        self.assertTrue(task.finished)

    def test_signed_cli_and_limits(self):
        calls, original = [], target.execute
        target.execute = lambda stages: calls.append(stages) or 0
        try:
            self.assertEqual(target.main(['--execute', '--entry-seconds', '1',
                '--entry-speed', '-30', '--entry-steer', '-22',
                '--turn-seconds', '2', '--turn-speed', '30', '--turn-steer', '22',
                '--straight-seconds', '1', '--straight-speed', '-20', '--straight-steer', '0']), 0)
            self.assertEqual([(s.speed, s.steering) for s in calls[0][1:-1]],
                             [(-30, -22), (30, 22), (-20, 0)])
        finally:
            target.execute = original
        for field, value in (('entry_speed', -31), ('turn_speed', 31),
                             ('turn_steer', -23), ('straight_steer', 23),
                             ('straight_speed', 1.5)):
            with self.assertRaises(ValueError):
                target.make_stages(**{field: value})

    def test_reverse_right_one_second_then_stop(self):
        for seconds in (1., 2.):
            task = target.OutputTimedRun(target.make_reverse_right_stages(seconds))
            previous, changes = (0, 0), []
            for i in range(141):
                now = i*.05
                task.observe(now, *previous)
                command = task.tick(now)
                if command != previous:
                    changes.append((now, command))
                    previous = command
            self.assertEqual([cmd for _, cmd in changes], [(-30, -22), (0, 0)])
            self.assertGreaterEqual(changes[1][0]-changes[0][0], seconds)
            self.assertTrue(task.finished)
        for invalid in (0, -1, 11, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                target.make_reverse_right_stages(invalid)

    def test_entry_is_separate_from_two_second_left(self):
        task = target.OutputTimedRun(target.make_stages(2., 1., entry_seconds=2.1))
        previous = (0, 0)
        changes = []
        for i in range(201):
            now = i*.05
            task.observe(now, *previous)
            command = task.tick(now)
            if command != previous:
                changes.append((now, command))
                previous = command
        self.assertEqual([cmd for _, cmd in changes],
                         [(30, 0), (30, 22), (30, 0), (0, 0)])
        self.assertGreaterEqual(changes[1][0]-changes[0][0], 2.1)
        self.assertGreaterEqual(changes[2][0]-changes[1][0], 2.)
        self.assertTrue(task.finished)

    def test_slow_ownership_query_does_not_block_control_thread(self):
        entered, release = threading.Event(), threading.Event()
        def slow_query():
            entered.set()
            release.wait(2.)
        watch = target.OwnershipWatch(slow_query, target.monotonic_clock())
        watch.start()
        try:
            self.assertTrue(entered.wait(1.))
            clock = target.monotonic_clock()
            started = clock()
            for _ in range(8):
                watch.check()
                time.sleep(.05)
            self.assertLess(clock()-started, .7)
            self.assertFalse(release.is_set())
        finally:
            release.set()
            watch.close()

    def test_failed_and_stale_ownership_checks_stop(self):
        def conflict():
            raise ValueError('another controller')
        now = [0.]
        watch = target.OwnershipWatch(conflict, lambda: now[0])
        watch.start()
        watch.thread.join(1.)
        with self.assertRaises(ValueError):
            watch.check()
        watch.close()
        watch = target.OwnershipWatch(lambda: None, lambda: now[0])
        now[0] = 2.1
        with self.assertRaises(ValueError):
            watch.check()
        watch.close()

    def test_hold_left_never_switches_to_straight(self):
        task = target.OutputTimedRun(target.make_hold_stages())
        self.assertEqual(task.tick(0), (0, 0))
        self.assertEqual(task.tick(3), (30, 22))
        for i in range(1201):
            now = 3+i*.05
            task.observe(now, 30, 22)
            self.assertEqual(task.tick(now), (30, 22))
        self.assertAlmostEqual(task.output_seconds, 60.)
        self.assertFalse(task.finished)
        task.observe(63.05, 0, 0)
        task.observe(64., 0, 0)
        self.assertAlmostEqual(task.output_seconds, 60.)

    def test_waits_for_base_output_before_timing(self):
        task = target.OutputTimedRun(target.make_stages())
        self.assertEqual(task.tick(0), (0, 0))
        self.assertEqual(task.tick(3), (30, 22))
        for i in range(19):
            task.observe(3.4+i*.05, 30, 22)
        self.assertEqual(task.tick(4.3), (30, 22))
        task.observe(4.45, 30, 22)
        self.assertEqual(task.tick(4.45), (30, 0))
        self.assertGreaterEqual(task.completed[0][1], 1.)

    def test_stopped_output_and_feedback_gaps_do_not_count(self):
        task = target.OutputTimedRun(target.make_stages(.5, .5))
        task.tick(0)
        task.tick(3)
        task.observe(3., 30, 22)
        task.observe(3.2, 30, 22)
        task.observe(3.25, 0, 0)
        task.observe(3.4, 30, 22)
        task.observe(3.8, 30, 22)  # unknown interval: no credit
        self.assertAlmostEqual(task.output_seconds, .2)
        self.assertEqual(task.tick(3.8), (30, 22))
        task.observe(4., 30, 22)
        task.observe(4.15, 30, 22)
        self.assertEqual(task.tick(4.15), (30, 0))
        self.assertEqual(task.output_seconds, 0)
        task.observe(4.2, 30, 22)  # previous stage cannot credit straight
        task.observe(4.3, 30, 0)
        task.observe(4.5, 30, 0)
        task.observe(4.7, 30, 0)
        self.assertEqual(task.tick(4.7), (30, 0))
        task.observe(4.85, 30, 0)
        self.assertEqual(task.tick(4.85), (0, 0))
        self.assertEqual(task.tick(5.6), (0, 0))
        self.assertTrue(task.finished)

    def test_missing_output_fails_instead_of_claiming_complete(self):
        task = target.OutputTimedRun(target.make_stages())
        task.tick(0)
        task.tick(3)
        with self.assertRaises(ValueError):
            task.tick(6.01)
        self.assertEqual(task.completed, [])

    def test_custom_durations_and_invalid_values(self):
        stages = target.make_stages(.5, 1.5)
        self.assertEqual((stages[1].seconds, stages[2].seconds), (.5, 1.5))
        for value in (0, -1, 11, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                target.make_stages(value, 1.)
            with self.assertRaises(ValueError):
                target.make_stages(1., value)

    def test_timed_commands_and_final_stop(self):
        task = target.OutputTimedRun(target.make_stages())
        changes, previous = [], (0, 0)
        for i in range(141):
            now = i / 20.
            task.observe(now, *previous)
            command = task.tick(now)
            if command != previous:
                changes.append((now, command))
                previous = command
        self.assertEqual([cmd for _, cmd in changes], [(30, 22), (30, 0), (0, 0)])
        self.assertGreaterEqual(changes[1][0]-changes[0][0], 1.)
        self.assertGreaterEqual(changes[2][0]-changes[1][0], 1.)
        self.assertEqual(len(task.completed), 2)
        self.assertTrue(task.finished)
        self.assertEqual(task.tick(100.), (0, 0))

    def test_wrong_steering_never_credits_time(self):
        task = target.OutputTimedRun(target.make_stages())
        task.tick(0)
        task.tick(3)
        for i in range(30):
            task.observe(3+i*.05, 30, 0)
        self.assertEqual(task.output_seconds, 0)
        self.assertEqual(task.tick(4.5), (30, 22))

    def test_preview_never_executes(self):
        original = target.execute
        def forbidden(*args, **kwargs):
            self.fail('preview invoked actuator executor')
        target.execute = forbidden
        try:
            self.assertEqual(target.main([]), 0)
            self.assertEqual(target.main(['--hold-left']), 0)
            self.assertEqual(target.main(['--reverse-right-seconds', '1']), 0)
        finally:
            target.execute = original

    def test_competing_controller_rejected(self):
        own = '/left_straight_test'
        command, bridge = (target.PROFILE['command_topic'],
                           target.PROFILE['bridge_node'])
        pubs = {command: [own], '/ackermann_cmd': [bridge],
                '/base_controller/status': ['/base_controller']}
        subs = {command: [bridge], '/ackermann_cmd': ['/base_controller']}
        require_ownership(pubs, subs, own, command=command, bridge=bridge)
        pubs['/control/cmd'] = ['/competition_controller']
        with self.assertRaises(ValueError):
            require_ownership(pubs, subs, own, command=command, bridge=bridge)


if __name__ == '__main__':
    unittest.main()
