#!/usr/bin/env python2
from __future__ import division
import unittest
from parking_line_stop_core import StopRun
from p3_approach import P3ApproachRun
from p3_parking_test import P3Run,arguments as p3_arguments
from p2_parking_test import P2Run,arguments,main,common


class Follower(object):
    def __init__(self):
        self.output = (30,8)
        self.calls = 0
    def observe(self,observation,now):pass
    def command(self,now):
        self.calls += 1
        return self.output


class P2Tests(unittest.TestCase):
    def approach(self,**options):
        task = P2Run(P3ApproachRun(StopRun(steering=0,camera_timeout=.8)),
                     lane_follower=Follower(),**options)
        task.start(0.)
        for now in (0.,.1):
            task.observe(2,100.+now,now,False)
            self.assertEqual((30,8),task.tick(now,100.+now))
        return task

    def confirmed(self,**options):
        task = self.approach(**options)
        task.observe(0,100.2,.2,True)
        self.assertEqual((0,0),task.tick(.2,100.2))
        self.assertEqual('P2_STOP_HOLD',task.reason)
        task.observe(0,100.35,.35,True)
        self.assertEqual((0,0),task.tick(.35,100.35))
        self.assertEqual((0,0),task.tick(.69,100.69))
        return task

    def test_saved_five_stage_commands_full_durations_and_final_stop(self):
        task = self.confirmed()
        calls = task.lane_follower.calls
        now = .71
        for duration,expected in zip((1.,.2,3.4,.1,2.1),
                                     ((30,0),(-30,22),(-30,-22),(-30,0),(-30,22))):
            self.assertEqual(expected,task.tick(now,100.+now))
            self.assertAlmostEqual(now+duration,task.deadline)
            self.assertEqual(expected,task.tick(task.deadline-.01,100.+task.deadline-.01))
            now = task.deadline+.01
        self.assertEqual((0,0),task.tick(now,100.+now))
        self.assertEqual('P2_COMPLETE',task.reason)
        self.assertEqual(calls,task.lane_follower.calls)
        self.assertEqual((0,0),task.tick(now+20.,120.+now))

    def test_first_forward_and_later_reverse_speeds_independent(self):
        task = self.confirmed(step1_speed=25,reverse_speed=28)
        self.assertEqual((25,0),task.tick(.71,100.71))
        self.assertEqual((25,0),task.tick(1.70,101.70))
        self.assertEqual((-28,22),task.tick(1.72,101.72))

    def test_late_poll_does_not_skip_first_forward_or_second_reverse(self):
        task = self.confirmed()
        self.assertEqual((30,0),task.tick(4.,104.))
        self.assertAlmostEqual(5.,task.deadline)
        self.assertEqual((-30,22),task.tick(7.,107.))
        self.assertAlmostEqual(7.2,task.deadline)

    def test_no_curves_does_not_start_timed_motion(self):
        task = self.approach()
        task.observe(0,100.2,.2,False)
        self.assertEqual((30,8),task.tick(.2,100.2))
        self.assertEqual(-1,task.index)

    def test_unconfirmed_end_never_starts_forward_stage(self):
        task = self.approach()
        task.observe(0,100.2,.2,True);task.tick(.2,100.2)
        for i in range(3,24):
            now = i*.1
            task.observe(i%2,100.+now,now,True)
            self.assertEqual((0,0),task.tick(now,100.+now))
        self.assertEqual('END_UNCONFIRMED_STOPPED',task.reason)
        self.assertEqual(-1,task.index)

    def test_camera_loss_in_confirmation_does_not_start_forward_stage(self):
        task = self.approach()
        task.observe(0,100.2,.2,True);task.tick(.2,100.2)
        self.assertEqual((0,0),task.tick(1.1,101.1))
        self.assertEqual('CAMERA_TIMEOUT',task.reason)

    def test_lane_dropout_stays_zero_and_cannot_trigger_parking(self):
        task = self.approach()
        task.lane_follower.output = (0,0)
        self.assertEqual((0,0),task.tick(.15,100.15))
        self.assertEqual('P2_WAIT_LANE',task.reason)
        task.lane_started = False
        task.observe(0,100.2,.2,True)
        self.assertEqual((0,0),task.tick(.2,100.2))
        self.assertEqual('NO_LANE_BEFORE_PARKING',task.reason)

    def test_operator_stop_latches_in_every_stage(self):
        for stage in range(5):
            task = self.confirmed()
            now = .71
            for i in range(stage+1):
                task.tick(now,100.+now)
                now = task.deadline+.01
            self.assertEqual((0,0),task.abort('operator stop'))
            self.assertEqual((0,0),task.tick(now+10.,110.+now))

    def test_p2_defaults_and_cli_tuning_do_not_change_p3_saved_values(self):
        args = arguments([])
        self.assertEqual((1.,.2,3.4,.1,2.1),tuple(getattr(args,'step%d_seconds' % i) for i in range(1,6)))
        self.assertEqual((30,0,'right',30,30),(args.speed,args.steering,args.side,args.step1_speed,args.reverse_speed))
        self.assertEqual((15.,30.,2,.1,.5),(args.seek_seconds,args.max_seconds,args.end_zero_frames,args.end_zero_seconds,args.parking_pause_seconds))
        tuned = arguments(['--step1-seconds','1.4','--step3-seconds','3.9'])
        self.assertEqual((1.4,3.9),(tuned.step1_seconds,tuned.step3_seconds))
        saved = p3_arguments([])
        self.assertEqual((1.5,.2,3.4,.1,2.1),tuple(getattr(saved,'step%d_seconds' % i) for i in range(1,6)))
        self.assertEqual((1.5,.2,3.4,.1,2.1),P3Run(None).seconds)

    def test_invalid_parameters_rejected(self):
        for argv in (['--step1-speed','-30'],['--reverse-speed','31'],
                     ['--step1-seconds','nan'],['--step2-seconds','0'],
                     ['--steering','-3'],['--end-zero-frames','1']):
            with self.assertRaises(ValueError):arguments(argv)

    def test_default_preview_never_calls_ros_runner(self):
        original = common.run
        def forbidden(*args,**kwargs):raise AssertionError('preview invoked ROS')
        common.run = forbidden
        try:self.assertEqual(0,main([]))
        finally:common.run = original

    def test_execute_wires_shared_vision_follower_and_p2_completion(self):
        original = common.run
        calls = []
        def runner(args,**kwargs):
            calls.append(kwargs)
            task = kwargs['task_factory'](StopRun())
            self.assertIsInstance(task,P2Run)
            self.assertIsInstance(task.approach,P3ApproachRun)
            self.assertEqual(30,task.lane_follower.follower.cfg['speed_raw']['lane'])
            return 0
        common.run = runner
        try:self.assertEqual(0,main(['--execute']))
        finally:common.run = original
        self.assertEqual(('P2_COMPLETE',),calls[0]['success_reasons'])
        self.assertTrue(calls[0]['lane_first'])
        self.assertEqual('/parking_p2/status',calls[0]['status_topic'])


if __name__ == '__main__':
    unittest.main()
