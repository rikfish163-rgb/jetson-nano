"""S parking must not swallow confirmed road signs during or after a turn."""
import copy
import unittest
from test_core import CONFIG
from robot.master.controller import Controller


class ParkingRouteSignScopeTests(unittest.TestCase):
    def test_front_p_between_left_turns_does_not_occupy_route_queue(self):
        self.check_front_p_between_left_turns(False)

    def test_front_p_between_left_turns_with_side_lidar_enabled(self):
        self.check_front_p_between_left_turns(True)

    def check_front_p_between_left_turns(self, side_lidar):
        for phase in ('MANEUVER', 'LANE'):
            cfg=copy.deepcopy(CONFIG)
            cfg.update(parking_mode='forward_center',parking_entry_style='S',
                       parking_enabled=True,wait_green=False,
                       parking_side_lidar_enabled=side_lidar)
            c=Controller(cfg);self.addCleanup(c.close)
            c.state=phase
            c.action='LEFT' if phase=='MANEUVER' else None
            c.last_completed_action='LEFT' if phase=='LANE' else None
            c.action_started=1.
            c.parking_route_ready=True
            for stamp in (3.,3.1,3.2):c.observe_sign('PARKING',.97,stamp,stamp)
            self.assertIsNone(c.pending)
            self.assertIsNone(c.next_direction)
            self.assertEqual(c.sign_info['decision'],'front_p_ignored_left_reverse_route')
            for stamp in (4.,4.1):c.observe_sign('LEFT',.97,stamp,stamp)
            if phase=='MANEUVER':
                self.assertEqual(c.next_direction,'LEFT')
                c.execute('mission','resume_lane')
            self.assertEqual(c.pending,'LEFT')
            for stamp in (4.2,4.6,5.,5.4,5.8,6.2,6.6):
                c.observe_lane([],0.,stamp)
                self.assertIsNone(c.execute('parking','turn_reverse_tick',stamp).value)
            self.assertIsNone(c.parking_entry)

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
