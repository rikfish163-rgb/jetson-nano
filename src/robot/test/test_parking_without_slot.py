"""P-directed visual parking requires no manual bay number; no actuators."""
import copy
import unittest

from test_core import CONFIG
from robot.common.contracts import validate_config
from robot.common.lidar_policy import explicit_parking_lidar_disabled
from robot.master.controller import Controller
from robot.parking.reverse_entry import ReverseStraightEntry, reverse_after_turn_enabled


class ParkingWithoutSlotTests(unittest.TestCase):
    def cfg(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.pop('parking_slot',None)
        cfg.update(parking_mode='forward_center',parking_entry_style='S',
                   parking_lidar_enabled=False,wait_green=False)
        return cfg

    def test_validation_and_reverse_gate_without_slot_key(self):
        cfg=self.cfg()
        validate_config(cfg)
        self.assertTrue(reverse_after_turn_enabled(cfg))
        self.assertTrue(explicit_parking_lidar_disabled(cfg,'PARKING','PARKING'))

    def test_ground_markers_and_front_entry_without_slot_key(self):
        c=Controller(self.cfg());self.addCleanup(c.close)
        c.observe_ground(dict(source='front',part='markers',markers=[],slots=[]),10.)
        result=c.execute('parking','start_explicit_parking',10.,trigger='sign').value
        self.assertEqual(result,(0,0.))
        self.assertEqual(c.state,'PARKING')
        c.parking_entry.observe([[(.3,-.19),(.8,-.19)],[(.3,.19),(.8,.19)]],10.1,c.pose)
        self.assertGreater(c.parking_entry.command(10.2,c.pose)[0],0)

    def test_rear_p_selects_bay_and_bottom_without_slot_key(self):
        e=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.,wait_for_sign=True)
        self.assertTrue(e.observe_sign('PARKING',.99,10.1,10.2,anchor=(-.9,.24)))
        lines=[[(-.3,y),(-.85,y)] for y in (-.33,.05,.43)]
        lines.append([(-.8,.05),(-.8,.43)])
        for stamp in (10.3,10.4):e.observe(lines,stamp,(0.,0.,0.))
        speed,steer=e.command(10.5,(0.,0.,0.))
        self.assertLess(speed,0)
        self.assertGreater(steer,0)
        self.assertTrue(e.debug['pair_seen'])
        self.assertTrue(e.debug['bottom_seen'])

    def test_slot_dimensions_cannot_retarget_visual_parking(self):
        cfg=self.cfg();cfg['slots']={}
        e=ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)
        e.observe_sign('PARKING',.99,10.1,10.2,anchor=(-.9,.24))
        e.observe([[(-.3,.43),(-.8,.43)]],10.3,(0.,0.,0.))
        self.assertGreater(e.command(10.5,(0.,0.,0.))[1],0)

    def test_complete_turn_loss_straight_p_sides_bottom_sequence(self):
        for turn in ('LEFT','RIGHT'):
            cfg=self.cfg();cfg['parking_entry_speed_raw']=16
            c=Controller(cfg);self.addCleanup(c.close)
            c.state='LANE';c.last_completed_action=turn
            c.scan_ready=lambda now:True
            c.checked_command=lambda command,now,allow_bypass:command
            c.observe_lane([(.3,0.),(.6,0.),(.9,0.)],.9,10.)
            c.execute('parking','turn_reverse_tick',10.)
            for stamp in (10.4,10.8,11.2,11.6,12.,12.4):
                c.observe_lane([],0.,stamp)
                c.execute('parking','turn_reverse_tick',stamp)
            self.assertEqual(c.state,'PARKING')
            e=c.parking_entry
            lines=[[(-.3,y),(-.85,y)] for y in (-.13,.25)]
            e.observe(lines,12.5,c.pose)
            self.assertEqual(c.tick(12.9),(-16,0.))
            self.assertIsNone(e.inner.view)  # paint cannot steer before P
            e.observe_sign('PARKING',.99,13.,13.,anchor=(-.9,.06))
            self.assertEqual(c.tick(13.05),(-16,0.))  # P alone supplies no rails
            e.observe(lines,13.1,c.pose)
            self.assertGreater(c.tick(13.2)[1],0.)
            self.assertIsNone(e.bottom)  # sides cannot declare parking complete
            bottom=[[(-.8,-.13),(-.8,.25)]]
            e.observe(lines+bottom,13.3,c.pose)
            self.assertIsNone(e.bottom)
            e.observe(lines+bottom,13.4,c.pose)
            self.assertIsNotNone(e.bottom)
            remaining=e.remaining()
            c.set_pose((-remaining-.001,0.,0.),13.5)
            self.assertEqual(c.tick(13.5),(0,0.))
            self.assertEqual(c.state,'FINISHED')
            self.assertEqual(c.reason,'parking_at_bottom_clearance')


if __name__=='__main__':unittest.main()
