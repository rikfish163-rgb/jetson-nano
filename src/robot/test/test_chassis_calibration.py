"""Motion geometry must be independent of the tuned lane steering gain."""
from __future__ import division
import copy
import json
import math
import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command, model_to_command_steering, validate_config
from robot.common.geometry import bicycle, distance, wrap
from robot.motion.calibration import raw_angle, command_angle, speed_gain
from robot.turn.planner import turn_parameters
from robot.motion.odometry import CommandOdometry
from robot.master.controller import Controller


class ChassisCalibrationTests(unittest.TestCase):
    def setUp(self):
        root=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.cfg=load_config(os.path.join(root,'config'))
        self.cfg.update(max_steer=.2,steering_command_scale_rad=.03)

    def test_raw_full_lock_is_unchanged_by_lane_limit(self):
        for maximum in (.2,.46275):
            cfg=dict(self.cfg,max_steer=maximum)
            for sign,radius in ((1,.6784646272),(-1,.9925)):
                wire=encode_command(20,model_to_command_steering(sign*maximum,cfg),cfg,0)
                self.assertEqual(wire['steering_raw'],sign*22)
                self.assertAlmostEqual(raw_angle(cfg,20,sign*22),sign*math.atan(.26/radius))
                cfg[('left_' if sign>0 else 'right_')+'turn_full_lock']=True
                self.assertAlmostEqual(turn_parameters(cfg,'LEFT' if sign>0 else 'RIGHT')['effective_radius'],radius)

    def test_raw_roundtrip_both_gears_and_wiring_signs(self):
        for speed_sign in (-1,1):
            for steering_sign in (-1,1):
                cfg=dict(self.cfg,speed_sign=speed_sign,steering_sign=steering_sign)
                for gear in (-1,1):
                    for raw in (-22,-11,0,11,22):
                        angle=raw_angle(cfg,gear*26,raw)
                        command=command_angle(cfg,gear,angle)
                        self.assertEqual(encode_command(gear*26,command,cfg,0)['steering_raw'],raw)

    def test_straight125_uses_feedback_not_raw_or_tick_count(self):
        for raw in (12,20,24,26,30):
            velocity=raw*speed_gain(self.cfg,raw)
            duration=1.25/velocity
            odom=CommandOdometry((0.,0.,0.),.26)
            # Irregular command feedback and sparse control reads.
            stamp=0.
            while stamp<duration:
                odom.observe(stamp,velocity,0.)
                stamp+=.17
            self.assertAlmostEqual(odom.estimate(duration)[0],1.25,places=8)

    def test_collision_preview_and_odometry_use_same_full_lock_arc(self):
        c=Controller(dict(self.cfg,wait_green=False,lidar_enabled=True))
        self.addCleanup(c.close)
        c._runtime.overrides['scan_ready']=lambda now:True
        paths=[]
        c._runtime.overrides['sweep_clear']=lambda path,*args,**kwargs:paths.append(path) or True
        for raw in (22,-22):
            command=raw/22.*.03
            c.checked_command((20,command),1.,False)
            horizon=c.cfg['obstacle_stop_distance']
            steering=raw_angle(c.cfg,20,raw)
            odom=CommandOdometry(c.pose,.26,timeout=10.)
            odom.observe(0.,20*speed_gain(c.cfg,20),steering)
            expected=odom.estimate(horizon/(20*speed_gain(c.cfg,20)))
            self.assertLess(distance(paths[-1][-1],expected),1e-9)
            self.assertLess(abs(wrap(paths[-1][-1][2]-expected[2])),1e-9)

    def test_invalid_chassis_mapping_is_rejected_before_startup(self):
        for key,value in (('forward_left_radius',0),('forward_mps_per_raw',float('nan')),
                          ('reverse_right_radius',True),('reverse_steering_sign',0)):
            cfg=copy.deepcopy(self.cfg)
            cfg['chassis_calibration'][key]=value
            with self.assertRaises(ValueError):validate_config(cfg)

    def test_speed_points_are_direction_specific_and_interpolate_between_measurements(self):
        cfg=copy.deepcopy(self.cfg)
        cfg['chassis_calibration']['speed_points']={
            'forward':[[15,.13],[20,.18],[25,.23],[30,.28]],
            'reverse':[[15,.17],[20,.19],[25,.24],[30,.29]]}
        for raw,velocity in ((15,.13),(20,.18),(25,.23),(30,.28),
                             (-15,.17),(-20,.19),(-25,.24),(-30,.29),(18,.16),(-18,.182)):
            self.assertAlmostEqual(abs(raw)*speed_gain(cfg,raw),velocity)
        self.assertAlmostEqual(10*speed_gain(cfg,10),.13*10/15)
        self.assertAlmostEqual(35*speed_gain(cfg,35),.28*35/30)

    def test_invalid_speed_points_are_rejected(self):
        for points in ([],[[15,0]],[[15,float('nan')]],[[20,.2],[15,.1]],
                       [[15,.2],[20,.1]],[[True,.1]],[[15,.1,.2]]):
            cfg=copy.deepcopy(self.cfg)
            cfg['chassis_calibration']['speed_points']={'forward':points,'reverse':[[15,.1]]}
            with self.assertRaises(ValueError):validate_config(cfg)

    def test_installed_speed_mapping_matches_corrected_measurement_evidence(self):
        root=os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        source=self.cfg['chassis_calibration']['speed_calibration_source']
        with open(os.path.join(root,source)) as stream:session=json.load(stream)
        self.assertTrue(session['complete'])
        self.assertEqual(len(session['trials']),8)
        for row in session['trials']:
            self.assertEqual(row['status'],'measured')
            raw=row['signed_speed_raw']
            expected=row['distance_cm']/100./row['command_seconds']
            self.assertAlmostEqual(abs(raw)*speed_gain(self.cfg,raw),expected)

    def test_status_exposes_the_installed_speed_points(self):
        from robot.master.telemetry import controller_status
        c=Controller(self.cfg)
        self.addCleanup(c.close)
        status=controller_status(c,c.cfg,1.,True,True,(15,0),0)
        self.assertEqual(status['motion_model']['speed_points'],self.cfg['chassis_calibration']['speed_points'])


if __name__ == '__main__':unittest.main()
