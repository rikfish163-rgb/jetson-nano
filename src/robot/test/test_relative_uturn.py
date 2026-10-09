import copy
import json
import unittest
from test_blue_uturn import BlueUturnTests
from robot.uturn.relative import RelativeUturn
from robot.uturn.relative import decode_scene


class RelativeExecutionTests(unittest.TestCase):
    def setUp(self):
        self.fixture=BlueUturnTests('test_low_confidence_does_not_cache_uturn')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.cfg=self.fixture.cfg
        self.raw=dict(frame='visual_junction_1',followed_boundary='RIGHT',
                      regions=[[[-1,-1],[2,-1],[2,2],[-1,2]]],pose=[0,0,0],
                      stamp=1,pose_source='vision')

    def task(self):
        t=RelativeUturn(self.cfg,decode_scene(json.loads(json.dumps(self.raw))))
        t.accept_plan(t.plan())
        return t

    def test_reject_command_model_and_unknown_area(self):
        self.assertEqual(decode_scene(self.raw)['pose_source'],'vision')
        for change in (dict(pose_source='command_model'),dict(regions=[]),dict(followed_boundary='BOTH')):
            raw=dict(self.raw,**change)
            with self.assertRaises(ValueError):decode_scene(raw)

    def test_goal_uses_lane_center_and_not_misaligned_car(self):
        scene=decode_scene(dict(self.raw,pose=[0,.03,.05],lane_reference=[0,0,0]))
        task=RelativeUturn(self.cfg,scene)
        self.assertAlmostEqual(task.goal[0],0.)
        self.assertAlmostEqual(task.goal[1],.60)
        self.assertAlmostEqual(abs(task.goal[2]),3.141592653589793)

    def test_uturn_speed_override_does_not_change_lane(self):
        before=copy.deepcopy(self.cfg['speed_raw'])
        self.cfg['uturn_speed_raw']=26
        task=RelativeUturn(self.cfg,decode_scene(self.raw))
        self.assertEqual(task.cfg['speed_raw']['action'],26)
        self.assertEqual(self.cfg['speed_raw'],before)

    def test_stale_pose_stops_even_with_valid_plan(self):
        t=self.task()
        self.assertEqual(t.command(2),(0,0))
        self.assertEqual(t.reason,'relative_uturn_pose_stale')

    def test_reference_change_blocks(self):
        t=self.task()
        t.observe(decode_scene(dict(self.raw,stamp=1.1,frame='different')))
        self.assertEqual(t.command(1.1),(0,0))
        self.assertEqual(t.phase,'BLOCKED')

    def test_reference_change_latches_replan_block(self):
        t=self.task()
        t.observe(decode_scene(dict(self.raw,stamp=1.1,frame='different')))
        self.assertFalse(t.replan(1.1))
        self.assertEqual(t.reason,'relative_uturn_replan_reference_invalid')
        self.assertFalse(t.replan(1.2))
        self.assertEqual(t.reason,'relative_uturn_replan_reference_invalid')

    def test_blocked_remaining_path_stops(self):
        t=self.task()
        t.observe(decode_scene(dict(self.raw,stamp=1.1,obstacles=[[0,0]])))
        self.assertEqual(t.command(1.1),(0,0))
        self.assertEqual(t.phase,'BLOCKED')

    def test_compiled_convex_area_keeps_boundary_inside(self):
        t=self.task()
        self.assertTrue(t.allowed((-1.0,0.0)))
        self.assertTrue(t.allowed((2.0,2.0)))
        self.assertFalse(t.allowed((2.0001,0.0)))

    def test_measured_path_reaches_goal_both_gears(self):
        self.cfg['cusp_pause']=0
        t=self.task()
        commands=[]
        now=1
        for p in t.follower.path:
            now+=.01
            t.observe(decode_scene(dict(self.raw,stamp=now,pose=list(p[:3]))))
            commands.append(t.command(now)[0])
        for i in range(5):
            now+=.01
            t.observe(decode_scene(dict(self.raw,stamp=now,pose=list(t.goal))))
            t.command(now)
        self.assertEqual(t.phase,'DONE')
        self.assertTrue(any(v<0 for v in commands))
        self.assertTrue(any(v>0 for v in commands))

    def test_controller_missing_scene_never_falls_back(self):
        c=self.fixture.c
        c.cfg['uturn_relative_enabled']=True
        c.state,c.action='UTURN','UTURN'
        c.uturn=dict(phase='WAIT_START')
        c.wait_until=0
        self.assertEqual(c.uturn_tick(2),(0,0))
        self.assertEqual(c.reason,'relative_uturn_scene_missing')

    def test_ten_degree_goal_requires_fresh_confirmations(self):
        t=self.task()
        t.phase='CONFIRM'
        pose=list(t.goal)
        pose[2]+=.20
        for stamp in (1.1,1.2,1.3):
            t.observe(decode_scene(dict(self.raw,stamp=stamp,pose=pose)))
            t.command(stamp)
        self.assertEqual(t.phase,'CONFIRM')
        t.observe(decode_scene(dict(self.raw,stamp=1.4,pose=list(t.goal))))
        for i in range(5):t.command(1.4)
        self.assertEqual(t.phase,'CONFIRM')

    def test_nine_degree_goal_completes_with_default_ten_degree_tolerance(self):
        t=self.task()
        t.phase='CONFIRM'
        pose=list(t.goal)
        pose[2]+=0.15
        for stamp in (1.1,1.2,1.3):
            t.observe(decode_scene(dict(self.raw,stamp=stamp,pose=pose)))
            t.command(stamp)
        self.assertEqual(t.phase,'DONE')

    def test_goal_heading_tolerance_is_configurable(self):
        self.cfg['uturn_goal_heading_tolerance_deg']=20
        t=self.task()
        t.phase='CONFIRM'
        pose=list(t.goal)
        pose[2]+=.20
        for stamp in (1.1,1.2,1.3):
            t.observe(decode_scene(dict(self.raw,stamp=stamp,pose=pose)))
            t.command(stamp)
        self.assertEqual(t.phase,'DONE')

    def test_tracking_error_uses_configured_limit(self):
        self.cfg['uturn_tracking_error_max']=.05
        t=self.task()
        t.observe(decode_scene(dict(self.raw,stamp=1.1,pose=[0,.07,0])))
        self.assertEqual(t.command(1.1),(0,0))
        self.assertEqual(t.phase,'BLOCKED')
        self.assertEqual(t.reason,'relative_uturn_tracking_error')

    def test_replan_keeps_original_target_and_is_bounded(self):
        self.cfg.update(uturn_replan_limit=1,uturn_replan_cooldown_s=1.0)
        t=self.task()
        original=tuple(t.goal)
        self.assertTrue(t.replan(1.0))
        self.assertEqual(tuple(t.goal),original)
        self.assertEqual(t.replan_attempts,1)
        self.assertFalse(t.replan(1.5))
        self.assertEqual(t.reason,'relative_uturn_replan_cooldown')
        self.assertFalse(t.replan(2.1))
        self.assertEqual(t.reason,'relative_uturn_replan_limit')

    def test_replan_never_runs_before_a_plan(self):
        self.cfg['uturn_replan_limit']=2
        t=RelativeUturn(self.cfg,decode_scene(json.loads(json.dumps(self.raw))))
        self.assertFalse(t.replan(1.0))
        self.assertEqual(t.reason,'relative_uturn_replan_unavailable')

    def test_self_crossing_region_rejected(self):
        raw=dict(self.raw,regions=[[[0,0],[1,1],[0,1],[1,0]]])
        with self.assertRaises(ValueError):decode_scene(raw)

    def test_scene_rejects_nonfinite_and_degenerate_geometry(self):
        for change in (dict(pose=[float('nan'),0,0]),
                       dict(pose=[0,0,1e308]),
                       dict(stamp=float('inf')),
                       dict(regions=[[[0,0],[1,0],[2,0]]]),
                       dict(obstacles=[{'x':0,'y':0}])):
            with self.assertRaises(ValueError):
                decode_scene(dict(self.raw,**change))

    def test_scene_accepts_bounded_yaw(self):
        scene=decode_scene(dict(self.raw,pose=[0,0,3.14]))
        self.assertAlmostEqual(scene['pose'][2],3.14)
