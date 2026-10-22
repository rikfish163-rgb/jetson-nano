"""A remembered far line and command odometry cannot prove arrival."""
import copy
import unittest
from test_core import CONFIG
from robot.common.geometry import world
from robot.parking.entry import ExplicitStraightEntry


class ParkingBottomArrivalTests(unittest.TestCase):
    def entry(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=0.,parking_entry_speed_raw=16,
                   parking_final_speed_raw=12,parking_bottom_clearance_m=.05)
        return ExplicitStraightEntry(cfg,(0.,0.,0.),1.)

    def test_eleven_second_old_bottom_cannot_finish_at_model_clearance(self):
        entry=self.entry()
        for stamp in (1.1,1.2):
            entry.observe([[(1.,-.19),(1.,.19)]],stamp,(0.,0.,0.))
        pose=(.63,0.,0.)
        self.assertEqual(entry.command(12.45,pose)[0],12)
        self.assertIsNone(entry.status)
        self.assertFalse(entry.debug['bottom_arrival_confirmed'])
        self.assertAlmostEqual(entry.debug['bottom_age_s'],11.25)

    def test_even_recent_far_measurement_cannot_finish_by_odometry_alone(self):
        entry=self.entry()
        for stamp in (1.1,1.2):
            entry.observe([[(1.,-.19),(1.,.19)]],stamp,(0.,0.,0.))
        self.assertGreater(entry.command(1.3,(.63,0.,0.))[0],0)
        self.assertIsNone(entry.status)

    def test_latest_frame_must_contain_bottom_to_confirm_arrival(self):
        entry=self.entry();p=entry.inner
        for stamp in (1.1,1.2):
            entry.observe([[ (.37,-.19),(.37,.19)]],stamp,(0.,0.,0.))
        entry.observe([],1.3,(0.,0.,0.))
        self.assertGreater(entry.command(1.3,(0.,0.,0.))[0],0)
        self.assertFalse(p.debug['bottom_arrival_confirmed'])
        entry.observe([[ (.37,-.19),(.37,.19)]],1.4,(0.,0.,0.))
        self.assertEqual(entry.command(1.4,(0.,0.,0.)),(0,0.))

    def test_off_center_short_bottom_remains_trackable(self):
        entry=self.entry()
        p=entry.inner
        p.remember_bottom([(1.,.15),(1.,.435)],1.1,'paired_far_end')
        entry.observe([[(.98,.15),(.98,.435)]],1.2,(0.,0.,0.))
        self.assertEqual(p.bottom_stamp,1.2)
        self.assertEqual(p.bottom_source,'tracked_bottom')
        # A neighbouring bay at the same depth must not take over the track.
        entry.observe([[(.97,-.435),(.97,-.15)]],1.3,(0.,0.,0.))
        self.assertEqual(p.bottom_stamp,1.2)

    def test_fresh_far_bottom_prevents_finish_then_measured_five_cm_stops(self):
        entry=self.entry()
        p=entry.inner
        for i in range(8):
            # The command model advances faster than the observed paint.
            pose=(i*.1,0.,0.)
            x=1.-i*.045
            p.pose=pose
            p.remember_bottom([world(pose,q) for q in ((x,-.19),(x,.19))],
                              1.1+i*.2,'tracked_bottom')
            self.assertGreater(entry.command(1.1+i*.2,pose)[0],0)
        self.assertAlmostEqual(p.debug['measured_bumper_clearance_m'],.355)
        for i in range(1,8):
            x=.685-i*.045
            pose=(.7+i*.06,0.,0.)
            entry.observe([[world(pose,q) for q in ((x,-.19),(x,.19))]],
                          2.5+i*.2,pose)
            command=entry.command(2.5+i*.2,pose)
            if i<7:self.assertGreater(command[0],0)
        self.assertEqual(command,(0,0.))
        self.assertEqual(entry.status,'FINISHED')
        self.assertTrue(entry.debug['bottom_arrival_confirmed'])

    def test_recorded_33cm_jump_cannot_replace_locked_front_bottom(self):
        entry=self.entry();p=entry.inner
        # Recorded last remembered stripe and visible bottom just before stop.
        old=[(.4107205486,.1653454287),(.3743183743,-.1173202426)]
        seen=[(.745,.49),(.7675,-.1875)]
        p.remember_bottom(old,1.1,'paired_far_end')
        for stamp in (12.35,12.45):
            entry.observe([seen],stamp,(0.,0.,0.))
            self.assertGreater(entry.command(stamp,(0.,0.,0.))[0],0)
            self.assertIsNone(entry.status)
        self.assertEqual(p.bottom_source,'paired_far_end')
        self.assertEqual(p.bottom_stamp,1.1)
        self.assertEqual(p.bottom,old)
        self.assertTrue(p.debug['bottom_loss_active'])

    def test_recovery_rejects_nearer_mouth_duplicates_and_adjacent_bay(self):
        for line in ([[(.4,-.19),(.4,.19)]],
                     [[(1.13,.25),(1.13,.63)]]):
            entry=self.entry();p=entry.inner
            p.remember_bottom([(.8,-.19),(.8,.19)],1.1,'paired_far_end')
            for stamp in (3.,3.1):entry.observe(line,stamp,(0.,0.,0.))
            self.assertEqual(p.bottom_stamp,1.1)
        entry=self.entry();p=entry.inner
        p.remember_bottom([(.8,-.19),(.8,.19)],1.1,'paired_far_end')
        for stamp in (3.,3.):
            entry.observe([[(1.13,-.19),(1.13,.19)]],stamp,(0.,0.,0.))
        self.assertEqual(p.bottom_stamp,1.1)


if __name__=='__main__':unittest.main()
