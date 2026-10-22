"""Front blue observations must reach rear parking, without becoming a bottom."""
import copy
import unittest
from test_core import CONFIG
from robot.common.geometry import bicycle
from robot.master.controller import Controller
from robot.parking.reverse_entry import ReverseStraightEntry


class FrontBlueReverseTests(unittest.TestCase):
    def core(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(parking_mode='forward_center',parking_entry_style='S',
                   parking_lidar_enabled=False,steering_command_scale_rad=.03)
        c=Controller(cfg);self.addCleanup(c.close)
        c.state=c.action='PARKING'
        c.parking_entry=ReverseStraightEntry(cfg,c.pose,10.,wait_for_sign=True)
        c.parking_entry.observe_sign('PARKING',.96,10.1,10.1,anchor=(-.9,0.))
        return c

    def frame(self,c,stamp,angle=.12):
        c.observe_ground(dict(source='front',part='markers',markers=[],slots=[],
            blue_lines=[dict(x=.60,y=.12,yaw=angle,length=.52)]),stamp)

    def test_front_blue_corrects_reverse_heading_without_white_lines(self):
        for angle in (-.12,.12):
            c=self.core()
            for stamp in (10.2,10.3):self.frame(c,stamp,angle)
            speed,steer=c.parking_entry.command(10.5,c.pose)
            self.assertLess(speed,0)
            self.assertGreater(bicycle(c.pose,-.1,steer,c.cfg['wheelbase'])[2]*angle,0)
            self.assertTrue(c.parking_entry.debug['blue_used'])
            self.assertEqual(c.parking_entry.debug['blue_camera_source'],'front')
            self.assertIsNone(c.parking_entry.bottom)

    def test_one_frame_or_stale_front_blue_does_not_steer(self):
        c=self.core();self.frame(c,10.2)
        self.assertEqual(c.parking_entry.command(10.5,c.pose)[1],0.)
        self.frame(c,10.3)
        self.assertEqual(c.parking_entry.command(12.,c.pose)[1],0.)

    def test_empty_rear_observations_do_not_erase_front_blue(self):
        c=self.core()
        for stamp in (10.2,10.3):
            self.frame(c,stamp)
            c.observe_ground(dict(source='rear',part='parking_lines',lines=[],
                blue_lines=[],markers=[],slots=[]),stamp)
        self.assertNotEqual(c.parking_entry.command(10.5,c.pose)[1],0.)


if __name__=='__main__':unittest.main()
