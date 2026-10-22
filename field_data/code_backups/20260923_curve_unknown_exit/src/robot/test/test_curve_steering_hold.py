import unittest
from test_curve_entry_control import CurveEntryControlTests
from robot.common.contracts import encode_command


class CurveSteeringHoldTests(CurveEntryControlTests):

    def raw(self, command):
        return encode_command(command[0],command[1],self.c.cfg,0)['steering_raw']

    def test_bad_frame_keeps_supported_lock_instead_of_driving_straight(self):
        for sign,direction,t in ((-1,'RIGHT',1.),(1,'LEFT',3.)):
            self.assertEqual(self.raw(self.observe([(.5,sign*.15),(.7,sign*.15),(.9,sign*.15)],t,direction)),sign*22)
            for i,y in enumerate((0.,-sign*.1,0.)):
                command=self.observe([(.5,y),(.7,y),(.9,y)],t+.1*(i+1),direction)
                self.assertEqual(command[0],16)
                self.assertEqual(self.raw(command),sign*22)

    def test_decreasing_steer_is_smoothed_but_increasing_is_immediate(self):
        self.observe([(.5,-.15),(.7,-.15),(.9,-.15)],1.)
        reduced=self.observe([(.5,-.02),(.7,-.02),(.9,-.02)],1.1)
        self.assertTrue(-22<self.raw(reduced)<-10,self.raw(reduced))
        strong=self.observe([(.5,-.15),(.7,-.15),(.9,-.15)],1.2)
        self.assertEqual(self.raw(strong),-22)

    def test_repeated_bad_frames_cannot_renew_hold_forever(self):
        self.c.cfg['gap_max_seconds']=.3
        self.observe([(.5,-.15),(.7,-.15),(.9,-.15)],1.)
        for t in (1.1,1.2):
            self.assertEqual(self.raw(self.observe([(.5,0),(.7,0),(.9,0)],t)),-22)
        self.assertEqual(self.observe([(.5,0),(.7,0),(.9,0)],1.4)[0],0)

    def test_distance_limit_cannot_be_refreshed_by_bad_frames(self):
        self.c.cfg['gap_max_distance']=.05
        self.observe([(.5,-.15),(.7,-.15),(.9,-.15)],1.)
        self.c.pose=(.1,0.,0.)
        self.assertEqual(self.observe([(.5,0),(.7,0),(.9,0)],1.1)[0],0)

    def test_empty_path_after_conflict_does_not_restart_loss_budget(self):
        self.c.cfg['gap_max_seconds']=.3
        self.observe([(.5,-.15),(.7,-.15),(.9,-.15)],1.)
        self.observe([(.5,0),(.7,0),(.9,0)],1.2)
        self.c.observe_lane([],0.,1.4)
        self.assertEqual(self.c.tick(1.4)[0],0)

    def test_duplicate_frame_does_not_run_filter_twice(self):
        self.observe([(.5,-.15),(.7,-.15),(.9,-.15)],1.)
        reduced=self.observe([(.5,-.02),(.7,-.02),(.9,-.02)],1.1)
        self.assertAlmostEqual(self.c.lane_command(1.15)[1],reduced[1])

    def test_exit_releases_history_and_stale_stream_still_stops(self):
        self.observe([(.5,-.15),(.7,-.15),(.9,-.15)],1.)
        self.c.observe_lane([(.5,0),(.7,0),(.9,0)],.9,1.1)
        self.assertEqual(self.c.tick(1.1)[1],0.)
        self.assertIsNone(self.c.curve_steering)
        self.assertEqual(self.c.tick(2.),(0,0.))

    def test_entry_can_reuse_fresh_same_side_lane_command(self):
        self.c.observe_lane([(.5,-.15),(.7,-.15),(.9,-.15)],.9,1.)
        self.assertEqual(self.raw(self.c.tick(1.)),-22)
        self.assertEqual(self.raw(self.observe([(.5,0),(.7,0),(.9,0)],1.1)),-22)

    def test_old_lock_does_not_bias_new_valid_path_after_expiry(self):
        self.c.cfg['gap_max_seconds']=.3
        self.observe([(.5,-.15),(.7,-.15),(.9,-.15)],1.)
        command=self.observe([(.5,-.02),(.7,-.02),(.9,-.02)],1.5)
        self.assertEqual(self.raw(command),-9)
