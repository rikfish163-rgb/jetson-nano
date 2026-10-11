"""Both cameras move away from P before handing back to visual parking."""
import copy
import unittest

from test_core import CONFIG
from robot.parking.reverse_entry import ReverseStraightEntry
from robot.parking.entry import ExplicitStraightEntry
from robot.master.controller import Controller
from robot.common.config import load_config
from robot.common.contracts import validate_config
from test_core import PACKAGE_DIR
import os


class ParkingPReverseDelayTests(unittest.TestCase):
    def test_production_config_enables_three_seconds_and_rejects_invalid_time(self):
        cfg=load_config(os.path.join(PACKAGE_DIR,'config'))
        self.assertEqual(cfg['parking_p_reverse_s'],3.)
        validate_config(cfg)
        for value in (-1.,float('nan'),11.):
            cfg['parking_p_reverse_s']=value
            with self.assertRaises(ValueError):validate_config(cfg)

    def test_rear_search_before_p_does_not_run_away(self):
        entry=self.entry()
        self.assertEqual(entry.command(11.,(0.,0.,0.)),(-16,0.))
        self.assertEqual(entry.reason,'parking_reverse_straight_wait_p')
        self.assertFalse(entry.p_reverse_active)

    def test_controller_stops_prelude_if_command_is_blocked(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=3.,parking_entry_speed_raw=16,
                   parking_mode='forward_center',parking_entry_style='S')
        core=Controller(cfg)
        self.addCleanup(core.close)
        core.scan_ready=lambda now:True
        core.checked_command=lambda command,now,allow_bypass:(0,0.)
        core.execute('parking','start_explicit_parking',10.,trigger='sign')
        self.assertEqual(core.execute('parking','explicit_parking_tick',10.1).value,(0,0.))
        self.assertEqual(core.execute('parking','explicit_parking_tick',11.).value,(0,0.))
        self.assertEqual(core.state,'FAULT')
        self.assertTrue(core.reason.startswith('parking_p_reverse_guard:'))

    def test_controller_outputs_opposite_directions_and_keeps_stage_reason(self):
        for rear,expected in ((False,-16),(True,16)):
            cfg=copy.deepcopy(CONFIG)
            cfg.update(parking_p_reverse_s=3.,parking_entry_speed_raw=16,
                       parking_mode='forward_center',parking_entry_style='S',
                       parking_lidar_enabled=False)
            core=Controller(cfg)
            self.addCleanup(core.close)
            core.execute('parking','start_explicit_parking',10.,trigger='sign')
            if rear:
                core.parking_entry=ReverseStraightEntry(cfg,core.pose,10.,wait_for_sign=True)
                core.parking_entry.observe_sign('PARKING',.99,10.5,10.5,anchor=(-1.,0.))
            self.assertEqual(core.execute('parking','explicit_parking_tick',10.6).value,(0,0.))
            self.assertEqual(core.execute('parking','explicit_parking_tick',11.).value,(expected,0.))
            self.assertEqual(core.reason,'parking_p_reverse')
            self.assertEqual(core.state,'PARKING')

    def test_front_pauses_reverses_three_seconds_then_uses_new_geometry(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=3.,parking_entry_speed_raw=16)
        entry=ExplicitStraightEntry(cfg,(0.,0.,0.),10.)
        sides=[[(.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)]]
        self.assertEqual(entry.command(10.1,(0.,0.,0.)),(0,0.))
        for i in range(30):
            now=11.+i*.1
            entry.observe(sides,now,(0.,0.,0.))
            self.assertEqual(entry.command(now,(0.,0.,0.)),(-16,0.))
            self.assertEqual(entry.reason,'parking_p_reverse')
        self.assertEqual(entry.command(14.,(0.,0.,0.)),(0,0.))
        self.assertEqual(entry.command(14.4,(0.,0.,0.)),(16,0.))
        self.assertIsNotNone(entry.inner.view)
        old_stamp=entry.inner.stamp
        entry.observe(sides,14.3,(0.,0.,0.))
        self.assertEqual(entry.inner.stamp,old_stamp)
        sides=[[(x,y+.04) for x,y in line] for line in sides]
        entry.observe(sides,14.5,(0.,0.,0.))
        speed,steer=entry.command(14.5,(0.,0.,0.))
        self.assertEqual(speed,16)
        self.assertNotEqual(steer,0.)

    def entry(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_p_reverse_s=3.,parking_entry_speed_raw=16,
                   parking_bottom_clearance_m=.05)
        return ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)

    def sides(self, center=0.):
        return [[(-.4,center-.19),(-1.,center-.19)],
                [(-.4,center+.19),(-1.,center+.19)]]

    def test_p_runs_three_seconds_straight_then_accepts_fresh_side_correction(self):
        entry=self.entry()
        self.assertTrue(entry.observe_sign('PARKING',.99,10.5,10.5,anchor=(-1.,.04)))
        entry.observe(self.sides(),10.6,(0.,0.,0.))
        self.assertEqual(entry.command(10.6,(0.,0.,0.)),(0,0.))
        # Start at first command, not at the P callback's wall-clock time.
        for i in range(30):
            now=11.+i*.1
            speed,steer=entry.command(now,(0.,0.,0.))
            self.assertEqual(speed,16)
            self.assertAlmostEqual(steer,0.)
            self.assertEqual(entry.reason,'parking_p_reverse')
        self.assertFalse(entry.observe_sign('PARKING',.99,13.95,13.95,anchor=(-1.,.04)))
        entry.command(14.,(0.,0.,0.))
        self.assertNotEqual(entry.reason,'parking_p_reverse')
        self.assertEqual(entry.command(14.4,(0.,0.,0.))[0],-16)
        self.assertIsNotNone(entry.inner.view)
        entry.observe(self.sides(.04),14.5,(0.,0.,0.))
        speed,steer=entry.command(14.5,(0.,0.,0.))
        self.assertEqual(speed,-16)
        self.assertNotEqual(steer,0.)

    def test_late_p_projection_does_not_restart_prelude(self):
        entry=self.entry()
        entry.observe_sign('PARKING',.99,10.5,10.5)
        self.assertEqual(entry.command(10.6,(0.,0.,0.)),(0,0.))
        for i in range(30):
            now=11.+i*.1
            if i==15:
                self.assertTrue(entry.observe_sign('PARKING',.99,now,now,anchor=(-1.,.04)))
            self.assertEqual(entry.command(now,(0.,0.,0.)),(16,0.))
            self.assertEqual(entry.reason,'parking_p_reverse')
        entry.command(14.,(0.,0.,0.))
        self.assertNotEqual(entry.reason,'parking_p_reverse')

    def test_control_gap_does_not_skip_remaining_reverse_time(self):
        entry=self.entry()
        entry.observe_sign('PARKING',.99,10.5,10.5,anchor=(-1.,.04))
        self.assertEqual(entry.command(10.6,(0.,0.,0.)),(0,0.))
        self.assertEqual(entry.command(11.,(0.,0.,0.)),(16,0.))
        self.assertEqual(entry.command(15.,(0.,0.,0.)),(0,0.))
        self.assertEqual(entry.reason,'parking_p_reverse_control_gap')
        self.assertEqual(entry.status,'FAULT')


if __name__=='__main__':unittest.main()
