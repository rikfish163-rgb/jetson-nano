"""Scene-driven maneuver dispatch never drives the chassis in these tests."""
import copy
import unittest
import test_blue_uturn as fixture
from robot.common.contracts import validate_config
from robot.master.telemetry import controller_status
from robot.uturn.relative import RelativeUturn
from robot.uturn.relative import decode_scene
from robot.common.geometry import distance
from robot.master.controller import Controller


class ManeuverIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.fx = fixture.BlueUturnTests('test_low_confidence_does_not_cache_uturn')
        self.fx.setUp()
        self.addCleanup(self.fx.doCleanups)
        # Mode is selected at startup, not changed under an already-built runtime.
        cfg=dict(copy.deepcopy(self.fx.c.cfg),parking_mode='parallel_reverse',parking_slot='AUTO')
        self.c = self.fx.c = Controller(cfg)
        self.addCleanup(self.c.close)

    def trigger(self):
        self.c.observe_sign('PARKING', .9, 1, 1)
        self.fx.ground(2, [(.3, 0)])
        return self.c.dispatch(2)

    def test_parallel_mode_is_a_separate_valid_mode(self):
        validate_config(self.c.cfg)
        self.c.cfg['parking_slot'] = 'P4'
        with self.assertRaises(ValueError):
            validate_config(self.c.cfg)

    def test_sign_waits_for_blue_before_planning(self):
        self.c.observe_sign('PARKING', .9, 1, 1)
        self.assertIsNone(self.c.dispatch(1))
        self.assertEqual(self.c.state, 'LANE')
        self.assertEqual(self.c.pending, 'PARKING')

    def test_blue_consumes_sign_and_locks_parallel_action(self):
        self.assertEqual(self.trigger(), (0, 0))
        self.assertEqual(self.c.state, 'PARALLEL_PARKING')
        self.assertEqual(self.c.action, 'PARKING')
        self.assertIsNone(self.c.pending)
        self.c.observe_sign('LEFT', .99, 2.1, 2.1)
        self.assertIsNone(self.c.pending)

    def test_no_scene_has_no_fallback_or_motion(self):
        self.trigger()
        self.assertEqual(self.c.parallel_parking_tick(2.3), (0, 0))
        self.assertEqual(self.c.reason, 'parallel_parking_scene_missing')
        self.assertIsNone(self.c.follower)

    def test_timeout_and_estop_still_stop(self):
        self.trigger()
        self.c.estop = True
        self.assertEqual(self.c.tick(2.1), (0, 0))
        self.assertEqual(self.c.reason, 'emergency_stop')
        self.c.estop = False
        # The executor enforces its timeout even without a pose producer.
        self.assertEqual(self.c.parallel_parking_tick(200), (0, 0))
        self.assertEqual(self.c.state, 'FAULT')

    def test_parking_still_requires_live_scan_even_if_lane_lidar_disabled(self):
        self.trigger()
        self.assertEqual(self.c.tick(2.1), (0, 0))
        self.assertEqual(self.c.reason, 'scan_missing_or_stale')

    def test_new_mode_does_not_change_normal_lane_config(self):
        before = copy.deepcopy(self.c.cfg)
        self.trigger()
        for key in ('max_steer', 'steering_raw_limit', 'lookahead', 'speed_raw'):
            self.assertEqual(before[key], self.c.cfg[key])

    def test_telemetry_marks_separate_scene_frame(self):
        status = controller_status(self.c, self.c.cfg, 2, False, False, (0, 0), 0)
        self.assertIn('scene_maneuver', status)
        self.assertIsNone(status['scene_maneuver'])

    def scene(self, stamp=2.3, slot_id='P1'):
        return dict(stamp=stamp,frame='measured_bay',pose_source='vision',
                    pose=[0,0,0],regions=[[[-2,-2],[2,-2],[2,2],[-2,2]]],
                    ready=True,occupancy='FREE',confirmations=3,target_id=slot_id,
                    slot=dict(id=slot_id,pose=[0,.48,0],length=.70,width=.36))

    def test_requested_bay_is_not_silently_substituted(self):
        self.c.cfg['parking_slot']='P2'
        self.trigger()
        self.c.observe_parallel_scene(self.scene(),2.3)
        self.assertEqual(self.c.parallel_parking_tick(2.3),(0,0))
        self.assertEqual(self.c.reason,'parallel_parking_requested_slot_missing')

    def test_auto_rejects_unconfigured_or_oversized_slot(self):
        self.trigger()
        self.c.observe_parallel_scene(self.scene(slot_id='P99'),2.3)
        self.assertEqual(self.c.parallel_parking_tick(2.3),(0,0))
        self.assertEqual(self.c.reason,'parallel_parking_wrong_slot_kind')
        raw=self.scene(stamp=2.4)
        raw['slot']['length']=2.0
        self.c.observe_parallel_scene(raw,2.4)
        self.assertEqual(self.c.parallel_parking_tick(2.4),(0,0))
        self.assertEqual(self.c.reason,'parallel_parking_slot_size_mismatch')

    def test_scene_must_be_newer_than_the_trigger(self):
        self.c.observe_parallel_scene(self.scene(stamp=1.9),1.9)
        self.trigger()
        self.assertEqual(self.c.parallel_parking_tick(2.3),(0,0))
        self.assertEqual(self.c.reason,'parallel_parking_scene_missing')

    def test_parallel_command_converts_physical_angle_once(self):
        self.trigger()
        self.c.cfg['steering_command_scale_rad']=.1
        class Task(object):
            phase,reason='TRACK','test'
            def command(task,now):return 12,self.c.cfg['max_steer']
        self.c.parallel_parking=Task()
        self.c.checked_command=lambda command,now,allow_bypass:command
        speed,steer=self.c.parallel_parking_tick(2.3)
        self.assertEqual(speed,12)
        self.assertAlmostEqual(steer,.1)

    def test_completed_parallel_parking_is_terminal(self):
        self.trigger()
        class Task(object):
            phase,reason='DONE','complete'
            def command(task,now):return 0,0.
        self.c.parallel_parking=Task()
        self.assertEqual(self.c.parallel_parking_tick(2.3),(0,0))
        self.assertEqual(self.c.state,'FINISHED')
        self.assertEqual(self.c.tick(2.4),(0,0))

    def test_planning_exception_becomes_explicit_fault(self):
        self.trigger()
        class Task(object):phase='PLAN'
        class FailedFuture(object):
            def done(future):return True
            def result(future):raise RuntimeError('test failure')
        self.c.parallel_parking=Task()
        self.c.parallel_future=FailedFuture()
        self.assertEqual(self.c.parallel_parking_tick(2.3),(0,0))
        self.assertEqual(self.c.state,'FAULT')
        self.assertIn('planner_error',self.c.reason)

    def test_replan_result_cannot_revive_a_changed_scene_reference(self):
        self.c.cfg.update(uturn_relative_enabled=True,uturn_replan_limit=1)
        self.c.state,self.c.action='UTURN','UTURN'
        self.c.uturn={}
        raw=dict(self.scene(),followed_boundary='RIGHT')
        task=RelativeUturn(self.c.cfg,decode_scene(raw))
        task.phase,task.reason='BLOCKED','relative_uturn_path_blocked'
        self.c.relative_uturn=task
        class Deferred(object):
            ready=False
            def done(future):return future.ready
            def result(future):return True
        future=Deferred()
        self.c.executor.submit=lambda *args:future
        self.assertEqual(self.c.relative_uturn_tick(2.3),(0,0))
        self.assertEqual(self.c.reason,'relative_uturn_replanning')
        self.assertEqual(self.c.relative_uturn_tick(2.31),(0,0))
        task.observe(decode_scene(dict(raw,frame='wrong_junction',stamp=2.32)))
        future.ready=True
        self.assertEqual(self.c.relative_uturn_tick(2.33),(0,0))
        self.assertIs(self.c.relative_uturn,task)
        self.assertEqual(self.c.reason,'relative_uturn_reference_changed')
        self.assertIsNone(self.c.relative_replan_future)

    def test_invalid_new_parameters_fail_before_start(self):
        for change in (dict(uturn_goal_forward_min=.2,uturn_goal_forward_max=-.2),
                       dict(uturn_replan_limit=1.5),
                       dict(parallel_parking_sample_step_m=.5),
                       dict(parallel_parking_speed_raw=1000),
                       dict(parallel_parking_confirm_frames=1)):
            cfg=dict(self.c.cfg,**change)
            with self.assertRaises(ValueError):validate_config(cfg)

    def test_parking_scan_sweep_does_not_extend_through_planned_stop(self):
        self.trigger()
        class Follower(object):
            path=[(.02,0,0,-1,0)]
            segment_end=0
        class Task(object):
            follower=Follower()
            scene=dict(pose=(0,0,0))
        self.c.parallel_parking=Task()
        self.c.pose=(4,3,0)  # Deliberately a different scan/command origin.
        self.c.scan_ready=lambda now:True
        horizons=[]
        def clear(path,unknown,report=False):
            horizons.append(max(distance(self.c.pose,p) for p in path))
            return True
        self.c.sweep_clear=clear
        self.assertEqual(self.c.checked_command((-12,0),2.3,False),(-12,0))
        self.assertLessEqual(horizons[0],.040001)


if __name__ == '__main__':
    unittest.main()
