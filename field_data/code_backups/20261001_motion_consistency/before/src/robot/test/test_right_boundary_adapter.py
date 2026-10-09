"""Typed boundary-message validation and steering feedback; no ROS nodes."""
import math
import unittest
from collections import deque
import test_ros_adapter as fixture


@unittest.skipUnless(fixture._rospy is not None and fixture.imp is not None,'requires ROS')
class BoundaryAdapterTests(unittest.TestCase):
    def setUp(self):
        self.env = fixture.RosAdapterTests('test_applied_raw_over_contract_clears_command')
        self.env.setUpClass()
        self.env.setUp()
        self.addCleanup(self.env.doCleanups)
        self.n = self.env.node
        self.n.history = deque([(9.8,(0,0,0)),(10.,(.2,0,0))])
        self.n.core.set_pose((.2,0,0),10.)

    def message(self,stamp=9.8,frame='base_link',x=.55):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        m = Path()
        m.header.stamp = fixture._rospy.Time.from_sec(stamp)
        m.header.frame_id = frame
        for i in range(3):
            p = PoseStamped()
            p.pose.position.x,p.pose.position.y = x+.1*i,.3
            m.poses.append(p)
        return m

    def test_boundary_uses_capture_pose_and_does_not_replace_center(self):
        self.n.left_boundary(self.message())
        self.assertAlmostEqual(self.n.core.left_boundary[0][0],.55)
        self.assertEqual(self.n.core.pose,(.2,0,0))
        self.assertEqual(self.n.core.lane,[])

    def test_wrong_frame_stale_future_and_nonfinite_are_rejected(self):
        messages = [self.message(frame='camera'),self.message(stamp=8),
                    self.message(stamp=11),self.message(x=float('nan')),
                    self.message(x=float('inf')),self.message(x=9)]
        for m in messages:
            self.n.left_boundary(m)
            self.assertEqual(self.n.core.left_boundary_stamp,-1)
        self.assertEqual(len(self.env.fake_rospy.warnings),len(messages))

    def test_oversized_boundary_is_rejected(self):
        m = self.message()
        m.poses *= 334
        self.n.left_boundary(m)
        self.assertEqual(self.n.core.left_boundary_stamp,-1)

    def test_duplicate_and_empty_boundary(self):
        self.n.left_boundary(self.message())
        self.n.left_boundary(self.message(x=.9))
        self.assertAlmostEqual(self.n.core.left_boundary[0][0],.55)
        m = self.message(stamp=10)
        m.poses = []
        self.n.left_boundary(m)
        self.assertEqual(self.n.core.left_boundary,[])
        self.assertIsNone(self.n.core.left_exit_line(10))

    def test_applied_raw_normalized_for_gap(self):
        self.n.applied_cb(self.applied(18,-22))
        self.assertAlmostEqual(self.n.core.current_steering(10),-self.n.cfg['max_steer'])
        self.assertAlmostEqual(self.n.core.applied_stamp,10)

    def test_command_scale_01_amplifies_and_keeps_raw_limit(self):
        from robot.common.contracts import encode_command
        self.n.cfg['steering_command_scale_rad'] = .1
        for angle,expected in ((0,0),(.05,11),(-.05,-11),(.1,22),(.3,22),(-.3,-22)):
            self.assertEqual(encode_command(16,angle,self.n.cfg,0)['steering_raw'],expected)

    def test_scaled_gap_hold_does_not_amplify_feedback_again(self):
        from robot.common.contracts import encode_command
        self.n.cfg['steering_command_scale_rad'] = .1
        self.n.applied_cb(self.applied(16,11))
        held = self.n.core.current_steering(10)
        self.assertAlmostEqual(held,.05)
        self.assertEqual(encode_command(16,held,self.n.cfg,0)['steering_raw'],11)
        self.assertAlmostEqual(self.n.applied[1],self.n.cfg['max_steer']/2)

    def test_bad_applied_value_cannot_be_held_for_gap(self):
        self.n.core.issued_steer = .1
        self.n.applied_cb(self.applied(18,-22))
        self.n.applied_cb(self.applied(18,float('nan')))
        self.assertEqual(self.n.core.current_steering(10),.1)

    def test_startup_neutral_trim_is_not_modelled_as_a_turn(self):
        self.n.core.state='STARTUP_STRAIGHT'
        self.n.cfg['startup_steering_raw']=-2
        self.n.applied_cb(self.applied(20,-2))
        self.assertEqual(self.n.applied,(20,0.))
        self.n.core.state='LANE'
        self.n.applied_at=0
        self.n.applied_cb(self.applied(20,-2))
        self.assertLess(self.n.applied[1],0.)

    def applied(self,speed,steer,stamp=10):
        msg = fixture._AppliedMessage(speed,steer)
        msg.header = fixture._Header(stamp,'base_link')
        return msg

    def test_delayed_applied_source_is_not_refreshed_by_arrival(self):
        self.n.applied_cb(self.applied(18,-22,9.8))
        self.assertEqual(self.n.core.applied_stamp,9.8)
        self.n.core.issued_steer = .1
        self.assertEqual(self.n.core.current_steering(10.1),.1)

    def test_stale_future_and_replayed_applied_are_rejected(self):
        self.n.core.issued_steer = .1
        self.n.applied_cb(self.applied(18,-22))
        for stamp in (10,9.99,9,11):
            self.n.applied_cb(self.applied(18,-22,stamp))
            self.assertEqual(self.n.applied,(0,0))
            self.assertEqual(self.n.core.current_steering(10),.1)


if __name__ == '__main__':
    unittest.main()
