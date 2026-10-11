"""S parking must not swallow confirmed road signs during or after a turn."""
import copy
import unittest
from test_core import CONFIG
from robot.master.controller import Controller


class ParkingRouteSignScopeTests(unittest.TestCase):
    def test_turn_and_its_exit_accept_next_direction_and_keep_red(self):
        for turn in ('LEFT','RIGHT'):
            for phase in ('MANEUVER','LANE'):
                for label in ('STRAIGHT','UTURN','RIGHT' if turn=='LEFT' else 'LEFT'):
                    cfg=copy.deepcopy(CONFIG)
                    cfg.update(parking_mode='forward_center',parking_entry_style='S',wait_green=False)
                    c=Controller(cfg);self.addCleanup(c.close)
                    c.state=phase;c.action=turn if phase=='MANEUVER' else None
                    c.last_completed_action=turn if phase=='LANE' else None
                    c.action_started=1.
                    c.observe_sign(label,.97,3.,3.)
                    self.assertIsNone(c.next_direction)
                    self.assertIsNone(c.pending)
                    c.observe_sign(label,.97,3.1,3.1)
                    if phase=='MANEUVER':
                        self.assertEqual(c.action,turn)
                        self.assertEqual(c.next_direction,label)
                        self.assertEqual(c.sign_info['decision'],'stored_next_direction')
                        c.execute('mission','resume_lane')
                    self.assertEqual(c.pending,label)
                    self.assertIsNone(c.next_direction)
                    # An accepted road instruction takes priority over reverse
                    # even after more than two seconds of fresh empty frames.
                    for stamp in (3.2,3.6,4.,4.4,4.8,5.2,5.6):
                        c.observe_lane([],0.,stamp)
                        self.assertIsNone(c.execute('parking','turn_reverse_tick',stamp).value)
                    self.assertIsNone(c.parking_entry)
                    self.assertEqual(c.pending,label)
                    c.observe_sign('RED',.99,6.,6.)
                    self.assertTrue(c.red)

    def test_confirmed_straight_clears_existing_reverse_loss_watch(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_mode='forward_center',parking_entry_style='S',wait_green=False)
        c=Controller(cfg);self.addCleanup(c.close)
        c.state='LANE';c.last_completed_action='LEFT'
        c.turn_reverse_watch=dict(acquired=True,stamp=2.9,loss_since=2.,loss_frames=3)
        for stamp in (3.,3.1):c.observe_sign('STRAIGHT',.96,stamp,stamp)
        self.assertEqual(c.pending,'STRAIGHT')
        self.assertIsNone(c.execute('parking','turn_reverse_tick',3.2).value)
        self.assertIsNone(c.turn_reverse_watch)
        self.assertIsNone(c.parking_entry)


if __name__=='__main__':unittest.main()
