import unittest
import os
import math
from uturn_open_loop import schedule, run_segments, select_segments, extend_last_turn, right_turn_segment6, replay_segments, ROOT, yaml, plan_uturn


class ScheduleTests(unittest.TestCase):
    def cfg(self):
        return dict(wheelbase=.26,max_steer=.5,steering_command_scale_rad=.1,
            steering_raw_limit=22,steering_sign=1,speed_raw_limit=100,speed_sign=1,
            uturn_calibration=dict(forward_mps_per_raw=.01,reverse_mps_per_raw=.005,
                forward_left_radius=.4,forward_right_radius=.5,
                reverse_left_radius=.6,reverse_right_radius=1.5,reverse_steering_sign=-1))

    def test_merge_and_keep_gear_change(self):
        path=[(0,0,0,1,0),(.1,0,0,1,0),(.2,0,0,1,0),(.1,0,0,-1,0)]
        result=schedule(path,self.cfg(),20)
        self.assertEqual(len(result),2)
        self.assertAlmostEqual(result[0]['seconds'],1)
        self.assertAlmostEqual(result[1]['seconds'],1)
        self.assertEqual(result[1]['speed'],-20)

    def test_empty_rejected(self):
        with self.assertRaises(ValueError):schedule([],self.cfg(),26)

    def test_segment6_changes_only_steering(self):
        plan=[dict(speed=26,steering=22,seconds=1.) for unused in range(7)]
        plan[5]=dict(speed=-26,steering=0,seconds=.83)
        changed=right_turn_segment6(plan)
        self.assertEqual(changed[:5],plan[:5])
        self.assertEqual(changed[6:],plan[6:])
        self.assertEqual(changed[5],dict(speed=-26,steering=-22,seconds=.83))
        self.assertEqual(plan[5]['steering'],0)
        with self.assertRaises(ValueError):right_turn_segment6(plan[:5])
        with self.assertRaises(ValueError):right_turn_segment6(changed)

    def test_reverse_right_replay_turns_nose_left(self):
        cfg=self.cfg();cfg.update(front_overhang=.1,rear_overhang=.1,body_width=.24)
        cfg['uturn_calibration']['reverse_steering_sign']=1
        plan=[dict(speed=-26,steering=-22,seconds=.83)]
        end=replay_segments(plan,cfg,lambda p:True)
        self.assertGreater(end[2],0)
        with self.assertRaises(ValueError):replay_segments(plan,cfg,lambda p:False)

    def test_tail_extension_preserves_prefix_and_increases_only_last_duration(self):
        from robot.common.geometry import bicycle
        from robot.uturn.calibration import limit
        cfg=self.cfg();cfg.update(front_overhang=.1,rear_overhang=.1,body_width=.24)
        steer=limit(cfg,1,1)
        end=bicycle((0,0,0),.2,steer,cfg['wheelbase'])
        path=[(0,0,0,1,steer),tuple(end)+(1,steer)]
        extra=extend_last_turn(path,cfg,26,.4,lambda p:True)
        self.assertEqual(extra[:len(path)],path)
        before=schedule(path,cfg,26);after=schedule(extra,cfg,26)
        self.assertEqual(len(after),len(before))
        self.assertAlmostEqual(after[-1]['seconds']-before[-1]['seconds'],.4)
        with self.assertRaises(ValueError):extend_last_turn(path,cfg,26,float('nan'),lambda p:True)
        with self.assertRaises(ValueError):extend_last_turn(path,cfg,26,.4,lambda p:False)

    def test_only_start_and_gear_changes_pause(self):
        events=[]
        segments=[dict(speed=26,steering=s,seconds=1) for s in (22,0,-22)]
        segments.append(dict(speed=-26,steering=22,seconds=2))
        run_segments(segments,.7,lambda *args:events.append(args))
        self.assertEqual(events,[(0,22,.7),(26,22,1),(26,0,1),(26,-22,1),
                                 (0,22,.7),(-26,22,2)])

    def test_half_lock_rejected(self):
        with self.assertRaises(ValueError):
            schedule([(0,0,0,1,0),(.1,.01,.1,1,.25)],self.cfg(),26)

    def test_stop_after_first_excludes_remaining_motion(self):
        plan=[dict(speed=26,steering=22,seconds=2.8),dict(speed=-26,steering=22,seconds=.4)]
        self.assertEqual(select_segments(plan,1),plan[:1])
        self.assertEqual(len(plan),2)
        with self.assertRaises(ValueError):select_segments(plan,0)
        with self.assertRaises(ValueError):select_segments(plan,3)

    def test_left_uturn_starts_left_and_never_reverses_yaw(self):
        folder=os.path.join(ROOT,'src/robot/config')
        with open(os.path.join(folder,'competition.yaml')) as f:cfg=yaml.safe_load(f)
        with open(os.path.join(folder,'uturn_vision.yaml')) as f:cfg.update(yaml.safe_load(f))
        cfg.update(planner_full_lock_only=True,planner_turn_direction=1,
                   planner_timeout=30.,steering_command_scale_rad=.1)
        path,reason=plan_uturn((0,0,0),'RIGHT',cfg,
            lambda p:-.5<=p[0]<=1.6 and -.27<=p[1]<=.87)
        self.assertTrue(path,reason)
        result=schedule(path,cfg,26)
        self.assertEqual((result[0]['speed'],result[0]['steering']),(26,22))
        self.assertTrue(any(s['speed']<0 for s in result))
        for step in result:
            self.assertGreaterEqual(step['speed']*step['steering'],0)
            self.assertIn(step['steering'],(-22,0,22))
        self.assertLess(math.hypot(path[-1][0],path[-1][1]-.6),.045)
        self.assertLess(abs(abs(path[-1][2])-math.pi),math.radians(10))


if __name__=='__main__':unittest.main()
