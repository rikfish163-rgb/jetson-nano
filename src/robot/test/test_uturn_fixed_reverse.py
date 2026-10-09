import math
import unittest
from test_blue_uturn import BlueUturnTests

class FixedReverseTests(unittest.TestCase):
    ground=BlueUturnTests.__dict__['ground']
    start=BlueUturnTests.__dict__['start']
    def setUp(self):
        BlueUturnTests.__dict__['setUp'](self)
        self.c.cfg['uturn_reverse_distance_m']=1.6

    def test_first_left_waits_for_new_blue_then_reverses(self):
        self.start()
        self.c.uturn_begin_alignment(3)
        self.assertEqual(self.c.uturn['phase'],'WAIT_EXIT_BLUE')
        self.assertEqual(self.c.uturn_tick(3),(self.c.cfg['straight_speed_raw'],0))

    def test_reverse_distance_stops_before_second_left(self):
        self.start()
        self.c.uturn.update(phase='REVERSE_STRAIGHT',reverse_origin=(0,0,0))
        self.c.set_pose((-1.59,0,0),3)
        self.assertLess(self.c.uturn_tick(3)[0],0)
        self.c.set_pose((-1.6,0,0),3.1)
        self.assertEqual(self.c.uturn_tick(3.1),(0,0))
        self.assertEqual(self.c.uturn['phase'],'BRAKE_FORWARD')

    def test_forward_displacement_does_not_complete_reverse(self):
        self.start()
        self.c.uturn.update(phase='REVERSE_STRAIGHT',reverse_origin=(0,0,0))
        self.c.set_pose((1.6,0,0),3)
        self.assertNotEqual(self.c.uturn_tick(3)[0],0)
        self.assertEqual(self.c.uturn['phase'],'REVERSE_STRAIGHT')

    def test_full_handoff_requires_first_left_and_new_blue(self):
        self.start()
        c=self.c
        # A blue line alone cannot finish the first left turn.
        self.ground(2.4,[(1.2,0)])
        c.tick(2.4)
        self.assertEqual(c.uturn['phase'],'LEFT_FIRST')
        c.set_pose(c.exit_pose,3)
        for t in (3,3.1,3.2,3.3):
            self.ground(t,[(.8,0)])
            c.tick(t)
        self.assertEqual(c.uturn['phase'],'WAIT_EXIT_BLUE')
        self.ground(3.4,[(.8,0)])
        self.assertGreater(c.tick(3.4)[0],0)
        self.assertEqual(c.uturn['phase'],'WAIT_EXIT_BLUE')
        c.set_pose((c.pose[0],c.pose[1]+.5,c.pose[2]),3.5)
        self.ground(3.5,[(.3,0)])
        self.assertEqual(c.tick(3.5),(0,0))
        self.assertEqual(c.uturn['phase'],'BRAKE_REVERSE')
        t=c.wait_until+.01
        self.assertLess(c.tick(t)[0],0)
        origin=c.pose
        c.set_pose((origin[0]-1.6*math.cos(origin[2]),
                    origin[1]-1.6*math.sin(origin[2]),origin[2]),t+.1)
        self.assertEqual(c.tick(t+.1),(0,0))
        self.assertEqual(c.uturn['phase'],'BRAKE_FORWARD')
        t=c.wait_until+.01
        self.ground(t);c.tick(t)
        self.assertEqual(c.uturn['phase'],'LEFT_SECOND')
        c.set_pose(c.exit_pose,t+.1)
        for stamp in (t+.1,t+.2,t+.3,t+.4):
            self.ground(stamp,[(.8,0)])
            c.tick(stamp)
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.action)

    def test_inner_white_lane_cannot_finish_either_left(self):
        self.start()
        c=self.c
        for second in (False,True):
            c.uturn_follow(3,second=second)
            c.set_pose(c.exit_pose,3)
            for t in (3,3.1,3.2,3.3):
                self.ground(t)
                c.observe_lane([(.3,0),(.5,0),(.7,0)],.99,t)
                c.tick(t)
            self.assertEqual(c.state,'UTURN')
            self.assertNotEqual(c.uturn['phase'],'WAIT_EXIT_BLUE')

    def test_slanted_blue_cannot_finish_left(self):
        self.start()
        c=self.c
        c.set_pose(c.exit_pose,3)
        for t in (3,3.1,3.2,3.3):
            self.ground(t,[(.8,0)],yaw=.5)
            c.tick(t)
        self.assertNotEqual(c.uturn['phase'],'WAIT_EXIT_BLUE')

    def test_stale_blue_cannot_finish_left(self):
        self.start()
        c=self.c
        c.set_pose(c.exit_pose,3)
        self.ground(3,[(.8,0)])
        self.assertEqual(c.tick(5),(0,0))
        self.assertEqual(c.reason,'uturn_outer_blue_front_stale')

    def test_unrelated_blue_cannot_trigger_reverse(self):
        self.start()
        c=self.c
        c.set_pose(c.exit_pose,3)
        for t in (3,3.1,3.2,3.3):
            self.ground(t,[(.8,0)])
            c.tick(t)
        self.assertEqual(c.uturn['phase'],'WAIT_EXIT_BLUE')
        self.ground(3.4,[(.3,0)])
        c.tick(3.4)
        self.assertEqual(c.uturn['phase'],'WAIT_EXIT_BLUE')

    def test_estop_blocks_fixed_reverse(self):
        self.start()
        self.c.uturn.update(phase='REVERSE_STRAIGHT',reverse_origin=self.c.pose)
        self.c.estop=True
        self.assertEqual(self.c.tick(3),(0,0))
