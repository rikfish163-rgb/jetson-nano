"""One S configuration for front entry and post-turn rear entry; no actuators."""
import copy
import os
import unittest

from test_core import CONFIG, PACKAGE_DIR
from robot.common.config import load_config
from robot.master.controller import Controller
from robot.parking.entry import ExplicitStraightEntry
from robot.parking.reverse_entry import ReverseStraightEntry
from robot.lidar.left_bays import arm, observe, measure


class SharedParkingEntryTests(unittest.TestCase):
    def cfg(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_mode='forward_center',parking_entry_style='S',
                   parking_entry_speed_raw=16,parking_p_reverse_s=0.,
                   wait_green=False,parking_lidar_enabled=False)
        return cfg

    def test_both_turn_exits_require_lane_handoff_before_reverse(self):
        for turn in ('LEFT','RIGHT'):
            core=Controller(self.cfg());self.addCleanup(core.close)
            core.state,core.action='MANEUVER',turn
            setattr(core,turn.lower()+'_lock',dict(phase='WAIT_LANE',exit_started=10.))
            for stamp in (9.9,10.,10.1,10.5,10.9,11.3,11.7,12.):
                core.observe_lane([],0.,stamp)
                core.execute('parking','turn_reverse_tick',max(10.,stamp))
                self.assertIsNone(core.parking_entry)
            core.observe_lane([],0.,12.2)
            core.execute('parking','turn_reverse_tick',12.2)
            self.assertIsNone(core.parking_entry)
            self.assertEqual(core.action,turn)
            self.assertIsNone(core.turn_reverse_watch)

    def test_production_turn_handoff_lane_then_loss_reverses(self):
        for turn in ('LEFT','RIGHT'):
            cfg=load_config(os.path.join(PACKAGE_DIR,'config'))
            cfg.update(wait_green=False,lidar_enabled=False,parking_lidar_enabled=False,
                       parking_mode='forward_center',parking_entry_style='S')
            core=Controller(cfg);self.addCleanup(core.close)
            core.action=turn
            if turn=='LEFT':
                # Ordinary left: the stopped scan sees neither occupied bay.
                core.pending_at=9.
                core.blue_approach=dict(phase='STOP',stop_until=9.9)
                core.left_bay_cache=arm(9.,9.)
                observe(core.left_bay_cache,measure([]),9.99,9.99,cfg['left_bay_choice'])
            core.execute('mission','begin_blue_action',10.)
            for i in range(400):
                now=10.+i*.05
                core.front_marker_stamp=now
                core.observe_lane([],0.,now)
                command=core.tick(now)
                self.assertNotEqual(core.state,'FAULT',core.reason)
                self.assertIsNone(core.parking_entry)
            self.assertEqual(core.action,turn)
            # Exit search must acquire the lane and explicitly hand it back.
            for i in range(1,31):
                now=30.+i*.1
                core.front_marker_stamp=now
                core.observe_lane([(.3,0.),(.6,0.),(.9,0.)],.99,now)
                command=core.tick(now)
                if core.state=='LANE' and core.turn_reverse_watch and core.turn_reverse_watch['acquired']:
                    break
            self.assertEqual(core.state,'LANE',core.reason)
            self.assertGreater(command[0],0)
            self.assertTrue(core.turn_reverse_watch['acquired'])
            start=now
            for i in range(1,8):
                now=start+i*.4
                core.observe_lane([],0.,now)
                command=core.tick(now)
                if core.parking_entry is not None:break
            self.assertIsNotNone(core.parking_entry)
            self.assertEqual(core.action_source,'turn_path_loss')
            self.assertEqual(core.tick(now+.4),(-16,0.))
            self.assertIsNone(core.parking_straight_travel)

    def test_empty_exit_repeated_stale_or_interrupted_frames_do_not_reverse(self):
        for turn in ('LEFT','RIGHT'):
            core=Controller(self.cfg());self.addCleanup(core.close)
            core.state,core.action='MANEUVER',turn
            setattr(core,turn.lower()+'_lock',dict(phase='WAIT_LANE',exit_started=10.))
            core.observe_lane([],0.,10.1)
            for now in (10.1,10.4,12.2):
                core.execute('parking','turn_reverse_tick',now)
            self.assertIsNone(core.parking_entry)
            for stamp in (12.3,12.7,13.1):
                core.observe_lane([],0.,stamp)
                core.execute('parking','turn_reverse_tick',stamp)
            core.observe_lane([(.4,0.)],.01,13.2)
            core.execute('parking','turn_reverse_tick',13.2)
            self.assertIsNone(core.turn_reverse_watch)
            core.next_direction='STRAIGHT'
            for stamp in (13.3,13.7,14.1,14.5,14.9,15.3):
                core.observe_lane([],0.,stamp)
                core.execute('parking','turn_reverse_tick',stamp)
            self.assertIsNone(core.parking_entry)

    def entries(self, anchor):
        cfg=self.cfg()
        front=ExplicitStraightEntry(cfg,(0.,0.,0.),10.,anchor)
        rear=ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)
        rear.observe_sign('PARKING',.99,10.1,10.1,
                          anchor=None if anchor is None else (-anchor[0],anchor[1]))
        return front,rear

    def observe(self, front, rear, lines, stamp):
        front.observe(lines,stamp,(0.,0.,0.))
        rear.observe([[(-x,y) for x,y in line] for line in lines],stamp,(0.,0.,0.))

    def test_rear_uses_recent_front_bottom_matching_and_short_piece_tracking(self):
        front,rear=self.entries((.77,-.29))
        rails=[[(.55,-.20),(1.04,-.20)],[(.56,.18),(1.04,.18)],
               [(.55,-.195),(.7825,-.195)]]
        self.observe(front,rear,rails+[[(1.04,-.22),(1.04,.44)]],10.2)
        for entry in (front,rear):
            self.assertIsNotNone(entry.view)
            self.assertIsNotNone(entry.bottom)
            self.assertEqual(entry.bottom_source,'paired_far_end')
        self.observe(front,rear,[[(1.04,.08),(1.04,.18)]],10.3)
        self.assertEqual(front.bottom_stamp,rear.bottom_stamp)
        self.assertEqual(rear.bottom_stamp,10.3)
        # A short fragment must not shrink the corridor permanently: the next
        # full measurement is still our stripe, but adjacent paint is not.
        self.observe(front,rear,[[(1.04,-.20),(1.04,.18)]],10.4)
        self.assertEqual(rear.bottom_stamp,10.4)
        self.observe(front,rear,[[(1.04,.21),(1.04,.50)]],10.5)
        self.assertEqual(rear.bottom_stamp,10.4)

    def test_recognized_p_without_projection_can_still_center_like_front(self):
        front,rear=self.entries(None)
        lines=[[(.3,-.13),(.8,-.13)],[(.3,.25),(.8,.25)]]
        self.observe(front,rear,lines,10.2)
        a=front.command(10.5,(0.,0.,0.))
        b=rear.command(10.5,(0.,0.,0.))
        self.assertEqual(a[0],-b[0])
        self.assertAlmostEqual(a[1],b[1])
        self.assertNotEqual(b[1],0.)
        self.assertEqual(rear.debug['steering_reference'],'both_sides')
        self.assertTrue(rear.debug['geometry_ready'])


if __name__=='__main__':unittest.main()
