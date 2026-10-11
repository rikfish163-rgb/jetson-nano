"""After a measured bottom disappears, S entry travels only another 15 cm."""
import copy
import unittest
from test_core import CONFIG
from robot.parking.entry import ExplicitStraightEntry
from robot.parking.reverse_entry import ReverseStraightEntry


class BottomLossStopTests(unittest.TestCase):
    def entry(self,rear=False):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=0.,parking_entry_speed_raw=16,
                   parking_final_speed_raw=12)
        cls=ReverseStraightEntry if rear else ExplicitStraightEntry
        entry=cls(cfg,(0.,0.,0.),1.)
        sign=-1 if rear else 1
        line=[[(sign*.8,-.19),(sign*.8,.19)]]
        for stamp in (1.1,1.2):entry.observe(line,stamp,(0.,0.,0.))
        entry.command(1.6,(0.,0.,0.))
        return entry,line

    def test_front_and_rear_stop_fifteen_cm_from_first_missing_image(self):
        for rear in (False,True):
            entry,line=self.entry(rear);direction=-1 if rear else 1
            origin=(direction*.2,0.,0.)
            entry.observe([],1.7,origin)
            self.assertEqual(entry.command(1.7,origin)[0],direction*12)
            self.assertEqual(entry.command(1.75,(direction*.3,0.,0.))[0],direction*12)
            pose=(direction*.349,0.,0.)
            self.assertEqual(entry.command(1.8,pose)[0],direction*12)
            pose=(direction*.35,0.,0.)
            self.assertEqual(entry.command(1.9,pose),(0,0.))
            self.assertEqual(entry.status,'FINISHED')
            self.assertEqual(entry.reason,'parking_bottom_lost_15cm')
            self.assertAlmostEqual(entry.debug['bottom_loss_travel_m'],.15)
            self.assertEqual(entry.command(2.,pose),(0,0.))
            self.assertEqual(entry.reason,'parking_bottom_lost_15cm')

    def test_camera_silence_is_not_a_missing_bottom_observation(self):
        entry,line=self.entry()
        self.assertGreater(entry.command(12.,(.5,0.,0.))[0],0)
        self.assertIsNone(entry.status)
        self.assertFalse(entry.debug['bottom_loss_active'])

    def test_never_seen_bottom_does_not_arm_loss_distance(self):
        cfg=copy.deepcopy(CONFIG);cfg['parking_p_reverse_s']=0.
        entry=ExplicitStraightEntry(cfg,(0.,0.,0.),1.)
        entry.observe([],1.2,(0.,0.,0.))
        entry.command(1.3,(.2,0.,0.))
        self.assertIsNone(entry.status)
        self.assertIsNone(entry.inner.bottom_loss_origin)

    def test_reappearance_cancels_old_distance_and_next_loss_starts_over(self):
        entry,line=self.entry()
        entry.observe([],1.7,(.1,0.,0.));entry.command(1.7,(.1,0.,0.))
        entry.command(1.8,(.19,0.,0.))
        entry.observe(line,1.9,(.19,0.,0.));entry.command(1.9,(.19,0.,0.))
        self.assertFalse(entry.debug['bottom_loss_active'])
        entry.observe([],2.,(.2,0.,0.))
        self.assertGreater(entry.command(2.,(.2,0.,0.))[0],0)
        entry.observe([],2.1,(.25,0.,0.))
        self.assertGreater(entry.command(2.1,(.25,0.,0.))[0],0)
        self.assertEqual(entry.command(2.2,(.35,0.,0.)),(0,0.))

    def test_distance_counts_curved_path_and_not_repeated_ticks(self):
        entry,line=self.entry()
        entry.observe([],1.7,(.1,0.,0.));entry.command(1.7,(.1,0.,0.))
        entry.command(1.8,(.16,0.,0.))
        entry.command(1.9,(.16,0.,0.))
        self.assertAlmostEqual(entry.debug['bottom_loss_travel_m'],.06)
        self.assertEqual(entry.command(2.,(.16,.09,0.)),(0,0.))

    def test_retreat_does_not_contribute_to_loss_distance(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=.2,parking_entry_speed_raw=16,cusp_pause=.3)
        entry=ExplicitStraightEntry(cfg,(0.,0.,0.),1.)
        line=[[(.8,-.19),(.8,.19)]]
        entry.command(1.,(0.,0.,0.))
        for stamp in (1.1,1.2):entry.observe(line,stamp,(0.,0.,0.))
        entry.command(1.31,(0.,0.,0.))
        entry.observe([],1.4,(-.03,0.,0.))
        entry.command(1.51,(-.05,0.,0.))
        entry.command(1.82,(-.05,0.,0.))
        self.assertIsNone(entry.bottom_loss_origin)
        entry.observe([],1.9,(-.05,0.,0.))
        self.assertIsNone(entry.bottom_loss_origin)
        entry.observe(line,2.,(-.05,0.,0.))
        entry.observe([],2.1,(-.04,0.,0.))
        self.assertGreater(entry.command(2.1,(-.04,0.,0.))[0],0)
        self.assertEqual(entry.command(2.2,(.11,0.,0.)),(0,0.))

    def test_controller_finishes_and_preserves_loss_stop_reason(self):
        from robot.master.controller import Controller
        for rear in (False,True):
            cfg=copy.deepcopy(CONFIG)
            cfg.update(wait_green=False,lidar_enabled=False,parking_lidar_enabled=False,
                       parking_mode='forward_center',parking_entry_style='S',parking_p_reverse_s=0.)
            core=Controller(cfg);self.addCleanup(core.close)
            core.execute('parking','start_explicit_parking',1.,trigger='sign')
            if rear:core.parking_entry=ReverseStraightEntry(cfg,core.pose,1.)
            direction=-1 if rear else 1
            for stamp in (1.1,1.2):
                core.parking_entry.observe([[(direction*.8,-.19),(direction*.8,.19)]],stamp,core.pose)
            core.tick(1.6)
            core.set_pose((direction*.2,0.,0.),1.7)
            core.parking_entry.observe([],1.7,core.pose)
            self.assertNotEqual(core.tick(1.7)[0],0)
            core.set_pose((direction*.35,0.,0.),1.8)
            self.assertEqual(core.tick(1.8),(0,0.))
            self.assertEqual(core.state,'FINISHED')
            self.assertEqual(core.reason,'parking_bottom_lost_15cm')


if __name__=='__main__':unittest.main()
