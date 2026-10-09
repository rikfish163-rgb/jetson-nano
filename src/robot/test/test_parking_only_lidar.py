"""Lidar occupancy is independent from vision-only road driving."""
import copy
import math
import unittest
import test_auto_parking as fixture
from robot.master.controller import Controller


class ParkingOnlyLidarTest(unittest.TestCase):
    def setUp(self):
        cfg=copy.deepcopy(fixture.CONFIG)
        cfg.update(wait_green=False,lidar_enabled=False,parking_enabled=True,
                   parking_slot='AUTO',parking_observe_s=.5)
        self.c=Controller(cfg)
        self.addCleanup(self.c.close)

    def trigger(self):
        self.c.pending='PARKING'
        self.c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind='junction',x=.2,y=0)]),10)
        self.assertEqual(self.c.dispatch(10),(0,0))

    def observe_slots(self):
        for t in (10.3,10.8):
            self.c.observe_ground(dict(source='front',part='slots',markers=[],
                slots=[dict(x=x,y=-.55,yaw=math.pi/2,kind='perpendicular')
                       for x in (.65,1.55)]),t)

    def test_no_scan_does_not_block_lane_or_pending_p_before_blue(self):
        self.c.observe_lane([(.3,0),(.6,0),(.9,0)],.9,10)
        self.assertGreater(self.c.tick(10)[0],0)
        for t in (10.1,10.2,10.3):self.c.observe_sign('PARKING',.99,t,t)
        self.assertEqual(self.c.pending,'PARKING')
        self.c.observe_lane([(.3,0),(.6,0),(.9,0)],.9,10.4)
        self.assertGreater(self.c.tick(10.4)[0],0)
        self.assertEqual(self.c.state,'LANE')

    def test_after_p_blue_missing_scan_waits_and_occupied_bays_are_rejected(self):
        self.trigger()
        self.assertEqual(self.c.tick(10),(0,0))
        self.assertEqual(self.c.reason,'scan_missing_or_stale')
        self.c.scan=fixture.Scan()
        self.c.scan.obstacles=[(.65,-.55),(1.55,-.55)]
        self.observe_slots()
        self.assertEqual(self.c.tick(10.8),(0,0))
        self.assertEqual(self.c.reason,'no_observed_empty_bay')
        self.assertTrue(all(r['occupancy']=='OCCUPIED' for r in self.c.parking_diagnostics))

    def test_p_can_select_free_bay_with_ordinary_lidar_control_disabled(self):
        self.trigger()
        self.c.scan=fixture.Scan()
        self.c.scan.obstacles=[(.65,-.55)]
        self.observe_slots()
        calls=[]
        self.c.executor.submit=lambda *args:calls.append(args)
        self.c.tick(10.8)
        self.assertEqual(self.c.state,'AUTO_PLANNING')
        self.assertEqual(len(calls),1)
        self.assertEqual([r['occupancy'] for r in self.c.parking_diagnostics],['OCCUPIED','FREE'])

    def test_obstacle_check_is_used_during_parking_not_ordinary_commands(self):
        self.c.scan=fixture.Scan()
        self.c.scan.obstacles=[(-.15,0)]
        self.assertEqual(self.c.checked_command((-16,0),10.8,False),(-16,0))
        self.c.start_follow([(0,0,0,-1,0),(-.1,0,0,-1,0)],'PARKING',10)
        self.assertEqual(self.c.checked_command((-16,0),10.8,False),(0,0))
        self.c.scan.obstacles=[]
        self.assertEqual(self.c.checked_command((-16,0),10.8,False),(-16,0))


if __name__=='__main__':unittest.main()
