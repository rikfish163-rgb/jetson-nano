import math
import unittest
from test_blue_uturn import BlueUturnTests
from robot.common.geometry import local
from robot.common.geometry import wrap
from robot.common.geometry import footprint
from robot.uturn.relative import UturnFollower
import robot.uturn.planner as uturn_planner
from robot.uturn.planner import uturn_goal
from robot.uturn.planner import uturn_goal_candidates
from robot.uturn.planner import three_arc_candidate
from robot.uturn.planner import plan_uturn


class RelativeUturnTests(unittest.TestCase):
    def test_search_mode_plans_both_lane_changes_without_three_arc_shortcut(self):
        fixture=BlueUturnTests('test_low_confidence_does_not_cache_uturn')
        fixture.setUp()
        try:
            cfg=dict(fixture.cfg,uturn_planner_mode='search',uturn_max_cusps=4)
            original=dict(cfg)
            for side,sign in [('RIGHT',1),('LEFT',-1)]:
                allowed=lambda q: -.5 <= q[0] <= .8 and -.27 <= sign*q[1] <= .87
                path,reason=plan_uturn((0,0,0),side,cfg,allowed)
                self.assertTrue(path,reason)
                self.assertEqual(reason,'uturn_hybrid_search')
                self.assertEqual(path[0][3],1)
                self.assertEqual(path[-1][3],1)
                self.assertIn(-1,[p[3] for p in path])
                self.assertTrue(all(allowed(q) for p in path for q in footprint(p,cfg,spacing=.025)))
                self.assertTrue(all(abs(p[4])<=cfg['max_steer'] for p in path))
                follower=UturnFollower(path,cfg)
                now=0.
                changes=0
                while not follower.done:
                    end=follower.segment_end
                    pose=path[end][:3]
                    command=follower.command(pose,now)
                    self.assertEqual(command,(0,0.0))
                    if follower.done:break
                    changes+=1
                    now+=cfg['cusp_pause']+.01
                    follower.command(pose,now)
                    self.assertGreater(follower.segment_end,end)
                    now+=1.
                self.assertGreater(changes,2)
                self.assertLessEqual(changes,4)
                self.assertLess(math.hypot(path[-1][0],path[-1][1]-sign*.6),.045)
                self.assertLess(abs(wrap(path[-1][2]-math.pi)),math.radians(10))
            self.assertEqual(cfg,original)
        finally:fixture.doCleanups()

    def test_search_rejects_corridor_narrower_than_body(self):
        fixture=BlueUturnTests('test_low_confidence_does_not_cache_uturn')
        fixture.setUp()
        try:
            path,reason=plan_uturn((0,0,0),'RIGHT',
                dict(fixture.cfg,uturn_planner_mode='search'),lambda p:abs(p[1])<.1)
            self.assertFalse(path)
            self.assertEqual(reason,'uturn_endpoint_blocked')
        finally:fixture.doCleanups()

    def test_search_configuration_rejects_invalid_mode_and_costs(self):
        fixture=BlueUturnTests('test_low_confidence_does_not_cache_uturn')
        fixture.setUp()
        try:
            for change in [dict(uturn_planner_mode='other'),
                           dict(uturn_reverse_penalty=-1),dict(uturn_max_cusps=1.5)]:
                with self.assertRaises(ValueError):
                    plan_uturn((0,0,0),'RIGHT',dict(fixture.cfg,**change),lambda q:True)
        finally:fixture.doCleanups()

    def test_goal_and_arcs_are_relative_and_mirrored(self):
        for start in ((0,0,0),(2,-1,.7)):
            for side,offset in (('LEFT',-.6),('RIGHT',.6)):
                goal=uturn_goal(start,side)
                self.assertAlmostEqual(local(start,goal)[0],0)
                self.assertAlmostEqual(local(start,goal)[1],offset)
                for radius in (.52,.65,.8):
                    path=three_arc_candidate(start,side,radius,.26)
                    for a,b in zip(path[-1][:2],goal[:2]):
                        self.assertAlmostEqual(a,b)
                    self.assertAlmostEqual(wrap(path[-1][2]-goal[2]),0)
                    self.assertEqual(set(p[3] for p in path),set((-1,1)))
                    self.assertAlmostEqual(abs(wrap(path[-1][2]-start[2])),math.pi)

    def test_unknown_boundary_rejected(self):
        for side in ('center','BOTH',None):
            with self.assertRaises(ValueError):uturn_goal((0,0,0),side)

    def test_goal_candidates_use_configured_spacing_and_forward_window(self):
        cfg=dict(uturn_lane_spacing=.60,uturn_goal_forward_min=-.20,
                 uturn_goal_forward_max=.30,uturn_goal_forward_step=.20,
                 uturn_goal_heading_tolerance_deg=10)
        candidates=uturn_goal_candidates((0,0,0),'LEFT',cfg)
        self.assertEqual([offset for unused,offset in candidates],[-.20,0,.20,.30])
        self.assertAlmostEqual(candidates[0][0][0],-.20)
        self.assertAlmostEqual(candidates[0][0][1],-.60)
        self.assertAlmostEqual(abs(wrap(candidates[-1][0][2])),math.pi)

    def test_invalid_goal_window_is_bounded(self):
        for cfg in (dict(uturn_goal_forward_min=.3,uturn_goal_forward_max=-.1),
                    dict(uturn_goal_forward_min=-2,uturn_goal_forward_max=2,
                         uturn_goal_forward_step=.001),
                    dict(uturn_lane_spacing=float('nan'))):
            with self.assertRaises(ValueError):
                uturn_goal_candidates((0,0,0),'RIGHT',cfg)

    def test_nonzero_forward_candidate_keeps_model_steer_below_physical_limit(self):
        path=three_arc_candidate((0,0,0),'RIGHT',.65,.26,.60,.30)
        self.assertAlmostEqual(path[-1][0],.30,places=6)
        self.assertAlmostEqual(path[-1][1],.60,places=6)
        self.assertAlmostEqual(wrap(path[-1][2]),math.pi)
        self.assertTrue(all(abs(p[4]) <= math.atan(.26/.65)+1e-10 for p in path))

    def test_planner_prefers_zero_forward_target_when_clear(self):
        fixture=BlueUturnTests('test_low_confidence_does_not_cache_uturn')
        fixture.setUp()
        try:
            cfg=dict(fixture.cfg,uturn_goal_forward_min=-.20,
                     uturn_goal_forward_max=.20,uturn_goal_forward_step=.20)
            path,reason=plan_uturn((0,0,0),'RIGHT',cfg,lambda unused: True)
            self.assertTrue(path)
            self.assertEqual(reason,'uturn_three_arcs')
            self.assertAlmostEqual(local((0,0,0),path[-1][:3])[0],0,places=6)
        finally:fixture.doCleanups()

    def test_planner_checks_area_and_body(self):
        fixture=BlueUturnTests('test_low_confidence_does_not_cache_uturn')
        fixture.setUp()
        try:
            cfg=fixture.cfg
            self.assertEqual(plan_uturn((0,0,0),'RIGHT',cfg,None)[1],'uturn_area_unknown')
            path,reason=plan_uturn((0,0,0),'RIGHT',cfg,lambda p: -1<p[0]<2 and -1<p[1]<2)
            self.assertTrue(path)
            self.assertEqual(reason,'uturn_three_arcs')
            self.assertEqual(plan_uturn((0,0,0),'RIGHT',cfg,lambda p: p[1]>=0)[1],
                             'uturn_endpoint_blocked')
        finally:fixture.doCleanups()

    def test_hybrid_fallback_uses_one_total_budget_for_goal_window(self):
        fixture=BlueUturnTests('test_low_confidence_does_not_cache_uturn')
        fixture.setUp()
        original_arc=uturn_planner.three_arc_candidate
        original_hybrid=uturn_planner.hybrid_plan
        original_clock=uturn_planner._monotonic
        try:
            cfg=dict(fixture.cfg,uturn_goal_forward_min=-.20,
                     uturn_goal_forward_max=.40,uturn_goal_forward_step=.20,
                     planner_timeout=.10)
            clock=[0.0]
            budgets=[]
            def fake_clock():
                return clock[0]
            def no_arc(*unused_args,**unused_kwargs):
                return []
            def no_hybrid(start,goal,search_cfg,obstacles,allowed,final_direction=1):
                budgets.append(search_cfg['planner_timeout'])
                clock[0]+=.06
                return [],'no_path'
            uturn_planner._monotonic=fake_clock
            uturn_planner.three_arc_candidate=no_arc
            uturn_planner.hybrid_plan=no_hybrid
            path,reason=plan_uturn((0,0,0),'RIGHT',cfg,lambda unused: True)
            self.assertFalse(path)
            self.assertEqual(reason,'planner_timeout')
            self.assertEqual(len(budgets),2)
            self.assertAlmostEqual(budgets[0],.10)
            self.assertAlmostEqual(budgets[1],.04)
        finally:
            uturn_planner.three_arc_candidate=original_arc
            uturn_planner.hybrid_plan=original_hybrid
            uturn_planner._monotonic=original_clock
            fixture.doCleanups()
