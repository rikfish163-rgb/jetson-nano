import math
import unittest
from test_direction_single_frame import DirectionSingleFrameTests
from robot.common.geometry import world
from robot.common.geometry import local
from robot.common.geometry import bicycle
from robot.common.contracts import command_to_model_steering
from robot.common.contracts import validate_config


def finish_blue_exit(c, t):
    """Feed measured normal on distinct frames, then simulate crossing the line."""
    for offset in (.01,.11,.21):
        c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind='junction',x=.3,y=0)],
            blue_lines=[dict(x=.3,y=0,yaw=0,length=.8)]),t+offset)
        c.tick(t+offset)
    assert c.straight_search['phase']=='CROSS_BLUE'
    point=world(c.pose,(.41,0))
    c.set_pose((point[0],point[1],c.pose[2]),t+.31)
    c.observe_ground(dict(source='front',part='markers',slots=[],markers=[],blue_lines=[]),t+.31)
    return c.tick(t+.31)


class StraightAlignmentTests(unittest.TestCase):
    core = DirectionSingleFrameTests.__dict__['core']

    def front(self, c, t, yaw=0, x=.8, marker=False, visible=True):
        c.observe_ground(dict(source='front',part='markers',slots=[],
            markers=[dict(kind='junction',x=x,y=0)] if marker else [],
            blue_lines=[dict(x=x,y=0,yaw=yaw,length=.8)] if visible else []),t)

    def maneuver(self):
        c=self.core()
        c.start_follow([(0,0,0,1),(2,0,0,1)],'STRAIGHT',1)
        c.consumed_marker=(-1,0)
        return c

    def test_signed_approach_uses_blue_normal_not_wrong_lane(self):
        for heading in (-.2,.2):
            c=self.core();c.cfg['steering_command_scale_rad']=.1
            c.observe_sign('STRAIGHT',.99,1,1)
            self.front(c,1,heading)
            c.observe_lane([(.3,-.3),(.6,-.3),(.9,-.3)],.99,1)
            speed,steer=c.tick(1)
            self.assertGreater(speed,0)
            self.assertGreater(heading*steer,0)
            self.assertLessEqual(abs(steer),c.cfg['steering_command_scale_rad'])

    def test_correction_updates_each_frame_and_zero_when_perpendicular(self):
        c=self.maneuver()
        for t,heading in ((1.1,.2),(1.2,-.2),(1.3,0)):
            self.front(c,t,heading)
            speed,steer=c.tick(t)
            if heading:self.assertGreater(heading*steer,0)
            else:self.assertEqual(steer,0)
            self.assertGreater(speed,0)

    def test_blue_normal_is_already_heading_and_has_pi_symmetry(self):
        c=self.maneuver();self.front(c,1.1,math.pi+.1)
        self.assertGreater(c.tick(1.1)[1],0)

    def test_marker_without_orientation_does_not_release(self):
        c=self.maneuver();self.front(c,1.1,x=.3,marker=True,visible=False)
        self.assertEqual(c.tick(1.1),(0,0))
        self.assertEqual(c.state,'MANEUVER')

    def test_three_distinct_aligned_frames_then_cross_before_lane(self):
        c=self.maneuver()
        self.front(c,1.1,.2,x=.3,marker=True)
        self.assertGreater(c.tick(1.1)[1],0)
        for t in (1.2,1.3):
            self.front(c,t,0,x=.3,marker=True);c.tick(t)
            self.assertEqual(c.straight_search['phase'],'ALIGN_BLUE')
        c.tick(1.31)
        self.assertEqual(c.straight_search['phase'],'ALIGN_BLUE')
        self.front(c,1.4,0,x=.3,marker=True);c.tick(1.4)
        self.assertEqual(c.straight_search['phase'],'CROSS_BLUE')
        c.observe_lane([(.3,.3),(.6,.4),(.9,.5)],.99,1.4)
        self.assertEqual(c.tick(1.41)[1],0)
        c.set_pose((.41,0,0),1.5)
        self.front(c,1.5,visible=False)
        c.tick(1.5)
        self.assertEqual(c.state,'LANE')

    def test_missing_or_misaligned_frame_resets_confirmation(self):
        for visible,yaw in ((False,0),(True,.2)):
            c=self.maneuver()
            for t in (1.1,1.2):self.front(c,t,0,x=.3,marker=True);c.tick(t)
            self.front(c,1.3,yaw,x=.3,marker=True,visible=visible);c.tick(1.3)
            self.front(c,1.4,0,x=.3,marker=True);c.tick(1.4)
            self.assertEqual(c.straight_search['phase'],'ALIGN_BLUE')

    def test_stale_camera_red_and_estop_stop_alignment(self):
        c=self.maneuver();self.front(c,1.1,.2)
        c.red=True;self.assertEqual(c.tick(1.1),(0,0))
        c.red=False;c.estop=True;self.assertEqual(c.tick(1.2),(0,0))
        c.estop=False;self.assertEqual(c.tick(3),(0,0))

    def test_side_longitudinal_and_short_lines_do_not_steer(self):
        for x,y,yaw,length in ((.8,2,.2,.8),(.8,0,math.pi/2,.8),(.8,0,.2,.1)):
            c=self.maneuver()
            c.observe_ground(dict(source='front',part='markers',slots=[],markers=[],
                blue_lines=[dict(x=x,y=y,yaw=yaw,length=length)]),1.1)
            self.assertEqual(c.tick(1.1)[1],0)

    def test_bicycle_feedback_converges_to_measured_blue_normal(self):
        c=self.maneuver();c.cfg['steering_command_scale_rad']=.1
        c.set_pose((0,0,.22),1)
        for i in range(45):
            t=1.1+i*.1;x,y=local(c.pose,(1.5,0))
            c.observe_ground(dict(source='front',part='markers',markers=[],slots=[],
                blue_lines=[dict(x=x,y=y,yaw=-c.pose[2],length=.8)]),t)
            speed,steer=c.tick(t)
            self.assertGreater(speed,0)
            c.set_pose(bicycle(c.pose,.02,command_to_model_steering(steer,c.cfg),
                               c.cfg['wheelbase']),t)
        self.assertLess(abs(c.pose[2]),math.radians(5))

    def test_alignment_config_rejects_invalid_values(self):
        for key,value in (('straight_align_distance',-1),
                          ('straight_align_tolerance_deg',90),('straight_blue_clearance',-1)):
            c=self.core();c.cfg[key]=value
            with self.assertRaises(ValueError):validate_config(c.cfg)
