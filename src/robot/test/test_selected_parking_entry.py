"""Sign/blue dispatch for explicit P4/P5 S/T entry; no actuators."""
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command, validate_config
from robot.master.controller import Controller
from test_continuous_obstacles import _SyntheticScan


class SelectedParkingEntryTests(unittest.TestCase):
    def core(self,style,slot='P4'):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(parking_mode='forward_center',parking_entry_style=style,
                   parking_slot=slot,wait_green=False,lidar_enabled=True,
                   steering_command_scale_rad=.03)
        c=Controller(cfg);self.addCleanup(c.close)
        c.last_blue_trigger=dict(stamp=.5,vehicle_pose=c.pose,travelled_m=0.)
        c.parking_straight_travel=dict(stamp=.5,vehicle_pose=c.pose,travelled_m=0.)
        c.scan=_SyntheticScan(1.)
        c.pending,c.pending_at='PARKING',1.
        return c

    def tick(self,c,now):
        c.scan=_SyntheticScan(now)
        c.front_marker_stamp=now
        return c.tick(now)

    def start_after_blue(self,c):
        c.action,c.pending='PARKING',None
        c.state='BLUE_STOP'
        c.blue_approach=dict(parking_anchor=None)
        c.execute('mission','begin_blue_action',1.)

    def test_s_starts_from_sign_without_waiting_for_blue(self):
        c=self.core('S')
        self.assertEqual(c.dispatch(1.),(0,0.))
        self.assertEqual(c.state,'PARKING')
        self.assertEqual(c.action,'PARKING')
        self.assertIsNone(c.pending)
        self.assertIsNotNone(c.parking_entry)

    def test_s_without_measured_white_geometry_stays_stopped(self):
        c=self.core('S');c.dispatch(1.)
        self.assertEqual(self.tick(c,1.1),(0,0.))
        self.assertEqual(c.state,'PARKING')
        self.assertIsNone(c.marker)

    def test_s_camera_sides_and_bottom_finish_at_front_bumper_clearance(self):
        for slot in ('P4','P5'):
            c=self.core('S',slot);c.dispatch(1.)
            lines=[[(.4,-.19),(1.,-.19)],[(.4,.19),(1.,.19)],
                   [(1.,-.19),(1.,.19)]]
            c.observe_ground(dict(source='front',part='parking_lines',
                                  lines=lines),1.1)
            command=self.tick(c,1.1)
            encoded=encode_command(command[0],command[1],c.cfg,0)
            self.assertEqual(encoded['speed_raw'],c.cfg['parking_entry_speed_raw'])
            self.assertEqual(encoded['steering_raw'],0)
            self.assertEqual(c.state,'PARKING')
            self.assertIsNotNone(c.parking_entry.bottom)

            # Supply a new camera frame at the measured stop pose. Clearance
            # is measured ahead of the bumper, rather than at the rear axle.
            stop_x=(1.-c.cfg['wheelbase']-c.cfg['front_overhang']-
                    c.cfg['parking_bottom_clearance_m']+.001)
            c.set_pose((stop_x,0.,0.),1.2)
            c.observe_ground(dict(source='front',part='parking_lines',
                lines=[[(x-stop_x,y) for x,y in line] for line in lines]),1.2)
            self.assertEqual(self.tick(c,1.2),(0,0.))
            self.assertEqual(c.state,'FINISHED')
            self.assertEqual(c.reason,'parking_at_bottom_clearance')
            self.assertEqual(self.tick(c,1.3),(0,0.))

    def test_t_waits_for_a_blue_line_after_the_sign(self):
        c=self.core('T')
        self.assertIsNone(c.dispatch(1.))
        self.assertEqual(c.pending,'PARKING')
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.parking_entry)

    def test_t_confirmed_sign_and_camera_blue_drive_the_terminal_right_step(self):
        c=self.core('T');c.pending=None
        c.cfg.update(blue_align_duration_s=.1,intersection_wait_s=0.)
        for stamp in (1.,1.1,1.2):
            c.observe_sign('PARKING',.99,stamp,stamp)
        self.assertEqual(c.pending,'PARKING')
        self.assertIsNone(c.dispatch(1.2))

        def blue_frame(stamp,pose_x=0.):
            c.set_pose((pose_x,0.,0.),stamp)
            x=.6-pose_x
            c.observe_ground(dict(source='front',part='markers',slots=[],
                markers=[dict(kind='junction',x=x,y=0.,length=.6)],
                blue_lines=[dict(x=x,y=0.,yaw=0.,length=.6)]),stamp)
            return self.tick(c,stamp)

        for stamp in (2.,2.1,2.2):
            blue_frame(stamp)
        self.assertEqual(c.state,'BLUE_APPROACH')
        self.assertEqual(c.action,'PARKING')
        blue_frame(2.35)
        self.assertEqual(c.blue_approach['phase'],'ADVANCE')
        self.assertEqual(blue_frame(2.5,c.cfg['blue_aligned_advance_m']),(0,0.))
        self.assertEqual(c.state,'BLUE_STOP')
        self.assertEqual(blue_frame(2.6,c.cfg['blue_aligned_advance_m']),(0,0.))
        self.assertEqual(c.state,'PARKING')
        self.assertIsNone(c.pending)
        self.assertIsNotNone(c.parking_entry)

        for i in range(10):
            command=self.tick(c,2.7+i*.1)
            encoded=encode_command(command[0],command[1],c.cfg,0)
            self.assertEqual(encoded['speed_raw'],c.cfg['parking_entry_speed_raw'])
            self.assertEqual(encoded['steering_raw'],-c.cfg['steering_raw_limit'])
        self.assertEqual(self.tick(c,3.7),(0,0.))
        self.assertEqual(c.state,'FINISHED')
        self.assertEqual(self.tick(c,3.8),(0,0.))

    def test_t_blue_completion_starts_exact_one_second_right_full_lock(self):
        for slot in ('P4','P5'):
            c=self.core('T',slot);self.start_after_blue(c)
            for i in range(10):
                command=self.tick(c,1.+i*.1)
                self.assertGreater(command[0],0)
                self.assertEqual(command[1],-.03)
            self.assertEqual(self.tick(c,2.),(0,0.))
            self.assertEqual(c.state,'FINISHED')
            self.assertEqual(self.tick(c,2.1),(0,0.))

    def test_t_interruption_stays_stopped_when_scan_returns(self):
        c=self.core('T');c.cfg['parking_lidar_enabled']=True
        self.start_after_blue(c)
        self.assertGreater(self.tick(c,1.1)[0],0)
        c.scan=None
        self.assertEqual(c.tick(1.2),(0,0.))
        self.assertEqual(c.state,'FAULT')
        self.assertEqual(self.tick(c,1.3),(0,0.))

    def test_explicit_style_rejects_unknown_or_parallel_slot(self):
        c=self.core('S')
        for style,slot in [('X','P4'),('S','P3'),('T','P2')]:
            c.cfg.update(parking_entry_style=style,parking_slot=slot)
            with self.assertRaises(ValueError):validate_config(c.cfg)


if __name__=='__main__':unittest.main()
