import unittest
from blue_test_helpers import enter_blue_action, clear_blue
import test_direction_single_frame as fixtures


class RequestedManeuversTests(unittest.TestCase):
    def test_configured_right_duration_preserves_full_lock_and_waits_for_exit(self):
        from robot.common.contracts import encode_command
        c=self.core();self.addCleanup(c.close)
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True,
                     timed_bypass_settle_s=.4,timed_bypass_left_s=2.6,
                     timed_bypass_right_s=7.8)
        c.cfg['speed_raw']['action']=20
        self.scan(c,1.,(.49,0))
        self.assertEqual(c.execute('obstacle','begin_timed_bypass',1.).value,(0,.1))
        for i in range(1,216):
            t=1.+i*.05;self.scan(c,t,None)
            command=c.tick(t)
            raw=encode_command(command[0],command[1],c.cfg,0)
            self.assertEqual(raw['speed_raw'],0 if i<8 else 20)
            self.assertEqual(raw['steering_raw'],22 if i<60 else -22)
            if i>=60:self.assertEqual(c.reason,'bypass_right_7.8s')
        self.scan(c,11.8,None)
        self.assertEqual(c.tick(11.8),(0,0.))
        self.assertEqual(c.reason,'bypass_wait_exit_lane')
        for t in (11.9,12.,12.1):
            self.scan(c,t,None)
            c.observe_lane([(.3,0),(.6,0)],.9,t);c.tick(t)
        self.assertEqual(c.state,'LANE')

    def test_invalid_bypass_right_duration_rejected(self):
        from robot.common.contracts import validate_config
        c=self.core();self.addCleanup(c.close)
        for duration in (0,-1,12.1,float('nan'),True):
            c.cfg['timed_bypass_right_s']=duration
            with self.assertRaises(ValueError):validate_config(c.cfg)

    def test_configured_bypass_left_duration_at_speed_20(self):
        c=self.core();self.addCleanup(c.close)
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True,
                     timed_bypass_settle_s=.4,timed_bypass_left_s=2.6)
        c.cfg['speed_raw']['action']=20
        self.scan(c,1.,(.49,0))
        self.assertEqual(c.execute('obstacle','begin_timed_bypass',1.).value,(0,.1))
        for i in range(1,180):
            t=1.+i*.05;self.scan(c,t,None)
            command=c.tick(t)
            self.assertEqual(command,(0,.1) if i<8 else (20,.1 if i<60 else -.1))
            if i==50:self.assertEqual(c.reason,'bypass_left_2.6s')
        self.scan(c,10.,None)
        self.assertEqual(c.tick(10.),(0,0.))
        self.assertEqual(c.timed_bypass['phase'],'REACQUIRE')

    def test_invalid_bypass_left_duration_rejected(self):
        from robot.common.contracts import validate_config
        c=self.core();self.addCleanup(c.close)
        for duration in (0,-1,6.1,float('nan'),True):
            c.cfg['timed_bypass_left_s']=duration
            with self.assertRaises(ValueError):validate_config(c.cfg)

    def test_bypass_uses_requested_action_speed_in_both_arcs(self):
        c=self.core();self.addCleanup(c.close)
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        c.cfg['speed_raw']['action']=18
        self.scan(c,1.,(.49,0))
        self.assertEqual(c.execute('obstacle','begin_timed_bypass',1.).value,(18,.1))
        for i in range(1,161):
            t=1.+i*.05;self.scan(c,t,None)
            command=c.tick(t)
            if c.timed_bypass['phase'] in ('LEFT','RIGHT'):
                self.assertEqual(command,(18,.1 if c.timed_bypass['phase']=='LEFT' else -.1))

    def test_left_settle_is_stationary_and_not_part_of_two_second_drive(self):
        c=self.core();self.addCleanup(c.close)
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True,
                     timed_bypass_settle_s=.4)
        self.scan(c,1.,(.49,0))
        self.assertEqual(c.execute('obstacle','begin_timed_bypass',1.).value,(0,.1))
        for i in range(1,49):
            t=1.+i*.05;self.scan(c,t,None)
            command=c.tick(t)
            self.assertEqual(command,(0,.1) if i<8 else (c.cfg['speed_raw']['action'],.1) if i<48 else (c.cfg['speed_raw']['action'],-.1))
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')

    def test_stale_scan_restarts_settle_without_advancing_drive(self):
        c=self.core();self.addCleanup(c.close)
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True,
                     timed_bypass_settle_s=.4)
        self.scan(c,1.,(.49,0));c.execute('obstacle','begin_timed_bypass',1.)
        self.assertEqual(c.tick(2.),(0,0.))
        self.scan(c,2.1,None)
        self.assertEqual(c.tick(2.1),(0,.1))
        self.assertEqual(c.timed_bypass['elapsed_s'],0.)

    def test_vehicle_script_uses_requested_steering_mapping(self):
        import os
        import shlex
        from robot.common.contracts import encode_command
        root=os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        with open(os.path.join(root,'tools','sign_detector_20260916','run_yolo_vehicle.sh')) as stream:
            tokens=shlex.split(stream.read().split('exec roslaunch ',1)[1].replace('\\\n',' '))
        args=dict(token.split(':=',1) for token in tokens if ':=' in token)
        self.assertEqual(float(args['steering_command_scale_rad']),.1)
        c=self.core();self.addCleanup(c.close)
        c.cfg['steering_command_scale_rad']=float(args['steering_command_scale_rad'])
        self.assertEqual(encode_command(24,.1,c.cfg,0)['steering_raw'],22)
        self.scan(c,1.,(.49,0))
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        speed,steer=c.execute('obstacle','begin_timed_bypass',1.).value
        self.assertEqual(encode_command(speed,steer,c.cfg,0)['steering_raw'],22)

    def test_bypass_returns_to_lane_preserving_straight_until_blue(self):
        c=self.core()
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True,sign_ttl=0,
                     blue_default_straight=False,lookahead=.55,straight_speed_raw=24,
                     straight_distance=1.25,parking_entry_speed_raw=12,sign_hz=5)
        self.scan(c,1.,(.49,0));c.execute('obstacle','begin_timed_bypass',1.)
        for i in range(1,160):
            t=1.+i*.05;self.scan(c,t,None);c.observe_lane([],0.,t)
            if i in (60,64):c.observe_sign('STRAIGHT',.99,t,t)
            self.assertEqual(c.tick(t),(c.cfg['speed_raw']['action'],.1 if i<40 else -.1))
        for t in (9.,9.1,9.2):
            self.scan(c,t,None);c.observe_lane([(.3,0),(.6,0)],.9,t);c.tick(t)
        self.assertEqual(c.state,'LANE')
        self.assertEqual(c.pending,'STRAIGHT')
        self.assertTrue(c.timed_bypass_completed)
        c.scan=None
        clear_blue(c,9.8)
        enter_blue_action(c,'STRAIGHT',10.2)
        self.assertIsNone(c.pending)
        self.assertEqual(c.straight_search['phase'],'STRAIGHT_DISTANCE')
        c.set_pose((1.26,0,0),10.3)
        for t in (10.3,10.4,10.5):
            c.observe_lane([(.3,0),(.6,0)],.9,t);c.tick(t)
        self.assertEqual(c.state,'LANE')
        c.observe_sign('STRAIGHT',.99,10.6,10.6)
        c.observe_lane([(.3,0),(.6,0)],.9,10.6);c.tick(10.6)
        self.assertIsNone(c.action)

    def test_full_profile_green_bypass_and_lane_handoff(self):
        import os
        from robot.common.config import load_config
        from robot.master.controller import Controller
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load_config(os.path.join(root, 'config'))
        # Apply full.launch's overrides after module configs, as stack.launch does.
        import xml.etree.ElementTree as ET
        launch = ET.parse(os.path.join(root, 'launch', 'full.launch')).getroot()
        defaults = dict((arg.get('name'),arg.get('default')) for arg in launch.findall('arg'))
        for key in ('wait_green','blue_default_straight','lidar_enabled','parking_enabled'):
            cfg[key] = defaults[key] == 'true'
        cfg['steering_command_scale_rad'] = float(defaults['steering_command_scale_rad'])
        self.assertTrue(cfg['timed_bypass_enabled'])
        self.assertFalse(cfg['bypass_enabled'])
        self.assertEqual(cfg['timed_bypass_trigger_distance_m'], .85)
        self.assertEqual(cfg['timed_bypass_left_s'], 2.6)
        self.assertEqual(cfg['timed_bypass_right_s'], 7.8)
        self.assertTrue(cfg['wait_green'])
        c = Controller(cfg);self.addCleanup(c.close)
        self.scan(c,1.,(.80,0))
        c.observe_lane([(.3,0),(.6,0)],.9,1.)
        self.assertEqual(c.tick(1.),(0,0.))
        self.assertEqual(c.state,'WAIT_GREEN')
        for i in range(10):
            t=1.+i*.1;c.observe_sign('GREEN',.99,t,t)
        self.assertEqual(c.state,'STARTUP_STRAIGHT')
        # Simulate completion of the main system's green-light straight segment.
        travelled=max(1.25,cfg.get('straight_distance',1.25))+.01
        c.set_pose((travelled,0,0),1.95)
        self.scan(c,1.95,None)
        c.observe_lane([(.3,0),(.6,0)],.9,1.95)
        self.assertGreater(c.tick(1.95)[0],0)
        self.assertEqual(c.state,'LANE')
        self.scan(c,2.,(travelled+.80,0))
        c.observe_lane([(.3,0),(.6,0)],.9,2.)
        command=c.tick(2.)
        self.assertEqual(command,(0,.1),c.reason)
        for i in range(1,216):
            t=2.+i*.05;self.scan(c,t,None)
            c.observe_lane([(.3,0),(.6,0)],.9,t)
            self.assertEqual(c.tick(t),(0,.1) if i<8 else (c.cfg['speed_raw']['action'],.1 if i<60 else -.1))
            self.assertEqual(c.state,'TIMED_BYPASS')
            if i<210:self.assertEqual(c.exit_count,0)
        self.scan(c,12.8,None)
        c.observe_lane([(.3,0),(.6,0)],.9,12.8)
        self.assertGreater(c.tick(12.8)[0],0)
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.action)

    def test_completed_bypass_ignores_later_obstacles_and_missing_scan(self):
        c=self.core();self.addCleanup(c.close)
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        self.scan(c,1.,(.49,0));c.execute('obstacle','begin_timed_bypass',1.)
        for i in range(1,161):
            t=1.+i*.05;self.scan(c,t,None)
            c.observe_lane([(.3,0),(.6,0)],.9,t);c.tick(t)
        self.assertEqual(c.state,'LANE')
        for t,point in ((9.1,(.49,0)),(9.2,(.20,0)),(11.,None)):
            self.scan(c,t,point)
            if point is None:c.scan=None
            c.observe_lane([(.3,0),(.6,0)],.9,t)
            self.assertGreater(c.tick(t)[0],0,c.reason)
            self.assertEqual(c.state,'LANE')
            self.assertIsNone(c.timed_bypass)
        c.estop=True
        self.assertEqual(c.tick(11.1),(0,0.))
        self.assertEqual(c.reason,'emergency_stop')
        c.estop=False;c.red=True
        self.assertEqual(c.tick(11.1),(0,0.))
        self.assertEqual(c.reason,'red_latched')
        c.red=False;c.action='PARKING';c.state='PARKING'
        c.cfg['parking_mode']='hybrid'
        self.assertEqual(c.tick(11.2),(0,0.))
        self.assertEqual(c.reason,'scan_missing_or_stale')
        self.assertEqual(c.checked_command((12,0.),11.2,False),(0,0.))
        fresh=self.core();self.addCleanup(fresh.close)
        self.assertFalse(fresh.timed_bypass_completed)

    def test_configured_bypass_clears_stationary_cone_before_lane_handoff(self):
        import math
        import os
        from robot.common.config import load_config
        from robot.common.geometry import bicycle
        from robot.common.contracts import command_to_model_steering
        from robot.master.controller import Controller
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cfg = load_config(os.path.join(root, 'config'))
        cfg.update(wait_green=False, blue_default_straight=False,
                   steering_command_scale_rad=.1)
        c = Controller(cfg)
        self.addCleanup(c.close)
        # Keep a finite cone fixed in world coordinates as the vehicle moves.
        # Earlier tests removed the obstacle immediately after triggering.
        points = [(1.0+.045*math.cos(i*math.pi/12),
                   .045*math.sin(i*math.pi/12)) for i in range(24)]
        phases = set()
        entered = False
        for i in range(300):
            now = 1.0+i*.05
            self.scan(c, now, None)
            c.scan.motion_obstacles = points
            c.scan.obstacles = points
            # Simulate lane reappearing after two seconds of right turn.
            task = c.timed_bypass
            visible = task is None or (task['phase']=='RIGHT' and task['elapsed_s']>=2.)
            c.observe_lane([(.3,0),(.6,0)] if visible else [], .9 if visible else 0., now)
            command = c.tick(now)
            if c.timed_bypass is not None and c.timed_bypass['phase']=='SETTLE_LEFT':
                self.assertEqual(command,(0,.1))
            else:
                self.assertGreater(command[0], 0,
                    'stationary cone stopped bypass: '+c.reason)
            if c.timed_bypass is not None:
                entered = True
                phases.add(c.timed_bypass['phase'])
            elif entered:
                self.assertEqual(c.state, 'LANE')
                self.assertEqual(phases, set(('SETTLE_LEFT','LEFT', 'RIGHT')))
                return
            physical = command_to_model_steering(command[1], cfg)
            c.set_pose(bicycle(c.pose, command[0]*cfg['raw_to_mps']['forward']*.05,
                               physical, cfg['wheelbase']), now)
        self.fail('bypass did not return to lane within the replay')

    def test_timed_bypass_can_reach_half_meter_trigger(self):
        from robot.lidar.scan import Scan
        c=self.core();c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        scan=Scan([float('inf')]*360,-3.14159,6.28318/360,.05,6,c.pose,c.cfg['lidar'],1.)
        scan.shape_filter=True;scan.motion_obstacles=[(.66,.02)]
        scan.obstacles=list(scan.motion_obstacles);c.scan=scan
        self.assertEqual(c.checked_command((26,0.),1.,True),(26,0.))
        scan.motion_obstacles=[(.49,0.)];scan.obstacles=list(scan.motion_obstacles)
        self.assertGreater(c.checked_command((26,0.),1.,True)[0],0)
        self.assertEqual(c.state,'TIMED_BYPASS')

    def core(self):
        c=fixtures.DirectionSingleFrameTests.__dict__['core'](self)
        c.cfg['steering_command_scale_rad']=.1
        return c

    def test_parking_motion_after_blue_without_lane(self):
        c=self.core();c.cfg['parking_mode']='forward_center'
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        enter_blue_action(c,'PARKING',1.2)
        c.tick(1.2)
        self.assertEqual(c.action,'PARKING')
        c.observe_ground(dict(source='front',part='parking_lines',slots=[],markers=[],lines=[]),1.3)
        self.assertEqual(c.tick(1.3),(c.cfg['parking_entry_speed_raw'],0.))
        c.observe_ground(dict(source='front',part='parking_lines',slots=[],markers=[],
            lines=[[(.4,.19),(1.,.19)],[(.4,-.19),(1.,-.19)],[(1.,-.19),(1.,.19)]]),1.4)
        self.assertEqual(c.tick(1.4),(c.cfg['parking_entry_speed_raw'],0.))
        c.set_pose((.64,0,0),1.5)
        self.assertEqual(c.tick(1.5),(0,0.))
        self.assertEqual(c.state,'FINISHED')
        self.assertEqual(c.tick(2.),(0,0.))
        self.assertEqual(c.reason,'parking_at_bottom_clearance')

    def test_forward_parking_center_and_bottom(self):
        from robot.parking.forward import ForwardParking
        c = self.core(); p = ForwardParking(c.cfg,c.pose,1.)
        p.observe([],1.,c.pose)
        self.assertEqual(p.command(1.,c.pose),(c.cfg['parking_entry_speed_raw'],0.))
        lines = [[(.4,.24),(1.,.24)],[(.4,-.14),(1.,-.14)]]
        p.observe(lines,1.1,c.pose)
        speed,steer=p.command(1.1,c.pose)
        self.assertGreater(speed,0);self.assertGreater(steer,0)
        p.observe([],1.2,c.pose)
        self.assertEqual(p.command(1.2,c.pose),(c.cfg['parking_entry_speed_raw'],0.))
        p.observe(lines+[[(1.,-.14),(1.,.24)]],1.3,c.pose)
        self.assertGreater(p.command(1.3,c.pose)[0],0)
        p.observe([],1.4,(.64,0.,0.))
        self.assertEqual(p.command(1.4,(.64,0.,0.)),(0,0.))
        self.assertEqual(p.status,'FINISHED')

    def test_parking_follows_single_line_before_pair_and_while_pair_stream_stale(self):
        c=self.core();c.cfg['parking_mode']='forward_center'
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        enter_blue_action(c,'PARKING',1.2)
        single=[(.3,.18),(.5,.18),(.7,.18)]
        c.observe_lane([(.3,0),(.6,0)],.9,1.2,{'LEFT':single})
        self.assertGreater(c.tick(1.2)[1],0)
        self.assertEqual(c.lane_source,'park_line_left')
        pair=[[(.4,.19),(1.,.19)],[(.4,-.19),(1.,-.19)]]
        c.observe_ground(dict(source='front',part='parking_lines',slots=[],markers=[],lines=pair),1.3)
        for t in (1.3,3.):
            # A visible boundary can be usable even without a trusted center.
            c.observe_lane([],0.,t,{'LEFT':single})
            self.assertGreater(c.tick(t)[1],0)
            self.assertEqual(c.parking_entry.phase,'APPROACH')
            self.assertEqual(c.reason,'parking_follow_single_line')
        c.observe_lane([],0.,3.1,{})
        c.observe_ground(dict(source='front',part='parking_lines',slots=[],markers=[],lines=[]),3.1)
        self.assertEqual(c.tick(3.1),(c.cfg['parking_entry_speed_raw'],0.))
        c.observe_ground(dict(source='front',part='parking_lines',slots=[],markers=[],lines=pair),3.2)
        self.assertGreater(c.tick(3.2)[0],0)
        self.assertEqual(c.parking_entry.phase,'CENTER')
        c.observe_lane([],0.,3.3,{'LEFT':single})
        c.observe_ground(dict(source='front',part='parking_lines',slots=[],markers=[],lines=[]),3.3)
        self.assertEqual(c.tick(3.3),(c.cfg['parking_entry_speed_raw'],0.))
        self.assertEqual(c.reason,'parking_missing_white_straight')

    def test_center_loss_does_not_chase_other_side_and_bottom_still_stops(self):
        c=self.core();c.cfg['parking_mode']='forward_center'
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        enter_blue_action(c,'PARKING',1.2)
        c.tick(1.2)
        pair=[[(.4,.19),(1.,.19)],[(.4,-.19),(1.,-.19)]]
        c.observe_ground(dict(source='front',part='parking_lines',lines=pair),1.3)
        c.tick(1.3)
        self.assertEqual(c.parking_entry.phase,'CENTER')
        c.park_line_side='LEFT'
        right=[(.3,-.18),(.5,-.18),(.7,-.18)]
        c.observe_lane([],0.,3.,{'RIGHT':right})
        c.observe_ground(dict(source='front',part='parking_lines',lines=[]),3.)
        self.assertEqual(c.tick(3.),(c.cfg['parking_entry_speed_raw'],0.))
        self.assertEqual(c.reason,'parking_missing_white_straight')
        c.observe_ground(dict(source='front',part='parking_lines',
            lines=pair+[[(1.,-.19),(1.,.19)]]),3.1)
        c.set_pose((.64,0,0),3.2)
        c.observe_lane([],0.,3.2,{'RIGHT':right})
        self.assertEqual(c.tick(3.2),(0,0.))
        self.assertEqual(c.state,'FINISHED')

    def test_fresh_pair_in_center_has_priority_over_single_boundary(self):
        c=self.core();c.cfg['parking_mode']='forward_center'
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        enter_blue_action(c,'PARKING',1.2)
        c.tick(1.2)
        pair=[[(.4,.19),(1.,.19)],[(.4,-.19),(1.,-.19)]]
        c.observe_ground(dict(source='front',part='parking_lines',lines=pair),1.3)
        c.tick(1.3)
        c.observe_lane([],0.,1.4,{'LEFT':[(.3,.18),(.5,.18),(.7,.18)]})
        self.assertEqual(c.tick(1.4),(c.cfg['parking_entry_speed_raw'],0.))
        self.assertEqual(c.parking_entry.reason,'parking_center_between_sides')

    def test_parking_speed_16_for_side_and_pair_then_stop_at_bottom_clearance(self):
        c=self.core();c.cfg.update(parking_mode='forward_center',parking_entry_speed_raw=16)
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        enter_blue_action(c,'PARKING',1.2)
        c.observe_lane([],0.,1.2,{'LEFT':[(.3,.18),(.5,.18),(.7,.18)]})
        self.assertEqual(c.tick(1.2)[0],16)
        c.observe_lane([],0.,1.3,{})
        pair=[[(.4,.19),(1.,.19)],[(.4,-.19),(1.,-.19)]]
        c.observe_ground(dict(source='front',part='parking_lines',lines=pair),1.3)
        self.assertEqual(c.tick(1.3)[0],16)
        c.observe_ground(dict(source='front',part='parking_lines',
            lines=pair+[[(1.,-.19),(1.,.19)]]),1.4)
        self.assertEqual(c.tick(1.4)[0],16)
        c.set_pose((.64,0.,0.),1.5)
        c.observe_ground(dict(source='front',part='parking_lines',lines=[]),1.5)
        self.assertEqual(c.tick(1.5),(0,0.))
        self.assertEqual(c.reason,'parking_at_bottom_clearance')

    def test_missing_paint_continues_past_old_timeout_until_bottom_reached(self):
        c=self.core();c.cfg.update(parking_mode='forward_center',parking_entry_speed_raw=16)
        for t in (1.,1.1,1.2):c.observe_sign('PARKING',.95,t,t)
        enter_blue_action(c,'PARKING',1.2)
        c.tick(1.2)
        for t in (2.,40.,80.):
            c.observe_ground(dict(source='front',part='parking_lines',lines=[]),t)
            c.observe_lane([],0.,t,{})
            self.assertEqual(c.tick(t),(16,0.))
            self.assertEqual(c.reason,'parking_missing_white_straight')
            self.assertEqual(c.state,'PARKING')
        c.observe_lane([],0.,80.1,{'RIGHT':[(.3,-.18),(.5,-.18),(.7,-.18)]})
        self.assertLess(c.tick(80.1)[1],0)
        c.observe_lane([],0.,80.2,{})
        pair=[[(.4,.24),(1.,.24)],[(.4,-.14),(1.,-.14)]]
        c.observe_ground(dict(source='front',part='parking_lines',lines=pair),80.2)
        self.assertGreater(c.tick(80.2)[1],0)
        c.observe_ground(dict(source='front',part='parking_lines',
            lines=pair+[[(1.,-.14),(1.,.24)]]),80.3)
        self.assertGreater(c.tick(80.3)[0],0)
        c.set_pose((.64,0.,0.),80.4)
        c.observe_ground(dict(source='front',part='parking_lines',lines=[]),80.4)
        self.assertEqual(c.tick(80.4),(0,0.))
        self.assertEqual(c.state,'FINISHED')

    def test_mouth_not_bottom_and_stale_camera_stops(self):
        from robot.parking.forward import ForwardParking
        c=self.core();p=ForwardParking(c.cfg,c.pose,1.)
        p.observe([[ (.4,.19),(1.,.19)],[(.4,-.19),(1.,-.19)],
                   [(.4,-.19),(.4,.19)]],1.,c.pose)
        self.assertIsNone(p.bottom)
        self.assertGreater(p.command(1.,c.pose)[0],0)
        self.assertEqual(p.command(3.,c.pose),(0,0.))

    def scan(self,c,t,point):
        class Scan(object):
            pass
        s=Scan();s.stamp=t;s.valid_rays=360;s.motion_obstacles=[point] if point else []
        s.obstacles=s.motion_obstacles;s.shape_filter=True
        s.shape_mode='line_reject';s.round_clusters=[]
        s.evidence=lambda p:dict(source='test_point')
        c.scan=s

    def test_obstacle_distance_and_left_right_timing(self):
        c=self.core();c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        self.scan(c,1.,(.51,0))
        self.assertIsNone(c.execute('obstacle','begin_timed_bypass',1.).value)
        self.scan(c,1.,(.49,0))
        self.assertEqual(c.execute('obstacle','begin_timed_bypass',1.).value,(c.cfg['speed_raw']['action'],.1))
        for i in range(1,40):
            t=1.+i*.05;self.scan(c,t,None)
            cmd=c.tick(t)
            self.assertEqual(cmd,(c.cfg['speed_raw']['action'],.1))
        self.scan(c,3.,None);cmd=c.tick(3.)
        self.assertEqual(cmd,(c.cfg['speed_raw']['action'],-.1))
        for i in range(41,160):
            t=1.+i*.05;self.scan(c,t,None)
            c.observe_lane([],0.,t);cmd=c.tick(t)
            self.assertEqual(cmd,(c.cfg['speed_raw']['action'],-.1))
        self.scan(c,9.,None)
        self.assertEqual(c.tick(9.),(0,0.))
        self.assertEqual(c.reason,'bypass_wait_exit_lane')
        self.assertEqual(c.state,'TIMED_BYPASS')
        self.assertFalse(c.timed_bypass_completed)
        self.assertEqual(c.action,'BYPASS')
        for t in (9.1,9.2,9.3):
            self.scan(c,t,None)
            c.observe_lane([(.3,0),(.6,0)],.9,t);c.tick(t)
        self.assertEqual(c.state,'LANE')

    def test_right_turn_requires_six_seconds_even_when_lane_returns(self):
        c=self.core();self.addCleanup(c.close)
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        self.scan(c,1.,(.49,0));c.execute('obstacle','begin_timed_bypass',1.)
        for i in range(1,41):
            t=1.+i*.05;self.scan(c,t,None)
            c.observe_lane([],0.,t);c.tick(t)
        self.assertEqual(c.timed_bypass['phase'],'RIGHT')
        for i in range(41,160):
            t=1.+i*.05;self.scan(c,t,None)
            c.observe_lane([(.3,0),(.6,0)],.9,t)
            self.assertEqual(c.tick(t),(c.cfg['speed_raw']['action'],-.1))
            self.assertEqual(c.state,'TIMED_BYPASS')
        self.scan(c,9.,None)
        c.observe_lane([(.3,0),(.6,0)],.9,9.)
        self.assertGreater(c.tick(9.)[0],0)
        self.assertEqual(c.state,'LANE')
        self.assertIsNone(c.timed_bypass)
        self.assertIsNone(c.action)

    def test_red_pause_does_not_skip_timed_segments(self):
        c=self.core();c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        self.scan(c,1.,(.49,0));c.execute('obstacle','begin_timed_bypass',1.)
        c.red=True;self.assertEqual(c.tick(1.1),(0,0.))
        c.red=False;self.scan(c,10.,None)
        self.assertEqual(c.tick(10.),(c.cfg['speed_raw']['action'],.1))
        self.assertEqual(c.timed_bypass['phase'],'LEFT')

    def test_bypass_ignores_sweep_obstacle_but_stale_scan_stops(self):
        c=self.core();c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        self.scan(c,1.,(.49,0));c.execute('obstacle','begin_timed_bypass',1.)
        self.assertEqual(c.tick(2.),(0,0.))
        self.assertEqual(c.reason,'scan_missing_or_stale')
        self.scan(c,2.1,(.35,0))
        self.assertEqual(c.tick(2.1),(c.cfg['speed_raw']['action'],.1))
        self.assertEqual(c.obstacle_check['reason'],'timed_bypass_sweep_disabled')
        for i in range(1,41):
            t=2.1+i*.05;self.scan(c,t,(.20,0));command=c.tick(t)
        self.assertEqual(command,(c.cfg['speed_raw']['action'],-.1))
        c.estop=True
        self.assertEqual(c.tick(4.2),(0,0.))
        self.assertEqual(c.reason,'emergency_stop')

    def test_sweep_guard_remains_outside_timed_bypass(self):
        c=self.core();self.addCleanup(c.close)
        c.cfg.update(lidar_enabled=True,timed_bypass_enabled=True)
        self.scan(c,1.,(.20,0))
        self.assertEqual(c.checked_command((24,0.),1.,False),(0,0.))
        self.assertEqual(c.reason,'lidar_obstacle_in_sweep')


if __name__=='__main__':unittest.main()
