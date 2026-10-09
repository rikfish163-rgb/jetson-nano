"""Nearest bay pair selection and small in-bay corrections, without actuators."""
import unittest
import test_direction_single_frame as fixtures
from robot.parking.forward import ForwardParking
from robot.common.contracts import encode_command


class NearestPairTests(unittest.TestCase):
    def parking(self):
        c=fixtures.DirectionSingleFrameTests.__dict__['core'](self)
        c.cfg.update(parking_entry_speed_raw=16,steering_command_scale_rad=.1)
        return c,ForwardParking(c.cfg,c.pose,1.)

    def test_near_offset_pair_wins_over_far_centered_pair(self):
        c,p=self.parking()
        near=[[(.3,.27),(.8,.27)],[(.3,-.11),(.8,-.11)]]
        far=[[(1.1,.19),(1.7,.19)],[(1.1,-.19),(1.7,-.19)]]
        for lines in (far+near, list(reversed(far+near))):
            p.observe(lines,p.stamp+2,c.pose)
            self.assertAlmostEqual(p.view['center'][0],.55)
            self.assertAlmostEqual(p.view['center'][1],.08)

    def test_far_bottom_cannot_stop_nearest_pair(self):
        c,p=self.parking()
        near=[[(.3,.27),(.8,.27)],[(.3,-.11),(.8,-.11)]]
        far=[[(1.1,.19),(1.7,.19)],[(1.1,-.19),(1.7,-.19)],
             [(1.7,-.19),(1.7,.19)]]
        p.observe(far+near,1.,c.pose)
        self.assertIsNone(p.bottom)
        self.assertEqual(p.command(1.,c.pose)[0],16)
        p.observe(near+[[(.8,-.11),(.8,.27)]],1.1,c.pose)
        self.assertEqual(p.command(1.1,c.pose)[0],16)
        p.observe([],1.2,(.431,0.,0.))
        self.assertEqual(p.command(1.2,(.431,0.,0.)),(0,0.))

    def test_both_steering_directions_are_small_and_missing_pair_is_straight(self):
        for sign in (-1,1):
            c,p=self.parking()
            lines=[[(.3,sign*.1+.19),(.9,sign*.25+.19)],
                   [(.3,sign*.1-.19),(.9,sign*.25-.19)]]
            p.observe(lines,1.,c.pose)
            speed,steer=p.command(1.,c.pose)
            self.assertEqual(speed,16)
            self.assertGreater(sign*steer,0)
            self.assertLessEqual(abs(encode_command(speed,steer,c.cfg,0)['steering_raw']),6)
            p.observe(lines[:1],1.1,c.pose)
            self.assertEqual(p.command(1.1,c.pose),(16,0.))


if __name__=='__main__':unittest.main()
