import unittest
from test_direction_single_frame import DirectionSingleFrameTests


class BlueNudgeTests(unittest.TestCase):
    core = DirectionSingleFrameTests.__dict__['core']

    def frame(self,c,t,yaw=.4,visible=True):
        c.observe_lane([(.3,0),(.6,0),(.9,0)],.99,t)
        c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind='junction',x=.3,y=0)],
            blue_lines=[dict(x=.3,y=0,yaw=yaw,length=.8)] if visible else []),t)

    def test_green_starts_straight_then_blue_hands_off_without_alignment(self):
        c=self.core();c.begin_startup(1.)
        c.observe_ground(dict(source='front',part='markers',slots=[],markers=[]),1.1)
        c.cfg['blue_default_straight']=False
        self.assertEqual(c.tick(1.1),(c.cfg['speed_raw']['lane'],0))
        self.assertEqual(c.state,'STARTUP_STRAIGHT')
        self.frame(c,1.2,.4)
        speed,steer=c.tick(1.2)
        self.assertGreater(speed,0);self.assertEqual(steer,0)
        self.assertEqual(c.state,'LANE')

    def test_blue_does_not_stop_and_exits_despite_heading_error(self):
        c=self.core();c.observe_sign('STRAIGHT',.99,1.,1.)
        self.frame(c,1.1)
        speed,steer=c.tick(1.1)
        self.assertGreater(speed,0);self.assertEqual(steer,0)
        self.assertEqual(c.state,'LANE')
        self.frame(c,2.2)
        speed,steer=c.tick(2.2)
        self.assertGreater(speed,0);self.assertEqual(steer,0)
        self.assertEqual(c.state,'LANE')

    def test_missing_heading_does_not_stop(self):
        c=self.core();c.observe_sign('STRAIGHT',.99,1.,1.)
        self.frame(c,1.1,visible=False)
        self.assertGreater(c.tick(1.1)[0],0)

    def test_pending_sign_without_near_blue_keeps_lane(self):
        c=self.core();c.observe_sign('STRAIGHT',.99,1.,1.)
        self.frame(c,1.1)
        c.marker=None
        self.assertEqual(c.tick(1.1)[1],0)

    def test_estop_still_stops(self):
        c=self.core();self.frame(c,1.1);c.estop=True
        self.assertEqual(c.tick(1.1),(0,0))

    def test_same_blue_does_not_restart_but_next_blue_does(self):
        c=self.core();c.cfg['blue_default_straight']=True
        c.cfg['straight_distance']=.1
        self.frame(c,1.1);c.tick(1.1)
        self.frame(c,2.2);c.tick(2.2)
        c.set_pose((.11,0.,0.),2.3)
        self.frame(c,2.3);c.tick(2.3)
        self.frame(c,2.35);c.tick(2.35)
        self.assertIsNone(c.straight_search)
        c.set_pose((2.,0.,0.),2.4)
        self.frame(c,2.4)
        self.assertGreater(c.tick(2.4)[0],0)
        self.assertIsNone(c.straight_search)
        self.assertGreater(c.consumed_marker[0],2.)
