"""P4/P5 laser geometry and sign-owned selection; no ROS or vehicle I/O."""
import copy
import math
import unittest
from test_core import CONFIG
from robot.master.controller import Controller
from robot.common.contracts import encode_command
from robot.lidar.left_bays import measure, arm, observe, decide


def scene(p4=True,p5=True,angle=0.,dx=0.,dy=0.):
    points=[]
    # Normal single-obstacle scenes have a sign only in the empty bay.
    sign_positions=(.61,) if p4 and not p5 else (1.,) if p5 and not p4 else (1.,.61)
    for x in sign_positions:
        points.extend((x-.09+i*.009,1.44) for i in range(21))
    if p4:points.extend((1.03+i*.007,1.12+i*.009) for i in range(7))
    if p5:points.extend((.58+i*.01,1.10+i*.01) for i in range(8))
    c,s=math.cos(angle),math.sin(angle)
    return [(c*x-s*y+dx,s*x+c*y+dy) for x,y in points]


class BayGeometryTest(unittest.TestCase):
    def test_single_sign_above_or_below_obstacle_after_vehicle_translation(self):
        for occupied,sign_x,obstacle_x,target in (('P5',1.,.61,'P4'),
                                                 ('P4',.61,1.,'P5')):
            points=[(sign_x-.09+i*.009,1.44) for i in range(21)]
            points.extend((obstacle_x+i*.007,1.12+i*.009) for i in range(7))
            for angle in (0.,math.radians(14)):
                c,s=math.cos(angle),math.sin(angle)
                moved=[(sign_x+c*(x-sign_x)-s*(y-1.44)+.05,
                        1.44+s*(x-sign_x)+c*(y-1.44)+.05) for x,y in points]
                d=measure(moved)
                self.assertEqual(d['counts'][occupied],7)
                self.assertEqual(d['counts']['P4' if occupied=='P5' else 'P5'],0)
                self.assertEqual(d['reason'],'single_sign_boxes')
                cache=arm(1.,1.);observe(cache,d,1.1,1.1,{})
                self.assertEqual(decide(cache,1.1,1.,{})['target'],target)

    def test_sign_alone_is_not_an_obstacle(self):
        d=measure([(1.-.09+i*.009,1.44) for i in range(21)])
        self.assertEqual(d['counts'],dict(P4=0,P5=0))

    def test_board_outside_twenty_cm_cannot_move_boxes(self):
        points=[(1.-.09+i*.009,1.75) for i in range(21)]+[(.61,1.45)]
        d=measure(points)
        self.assertEqual(d['reason'],'calibrated_boxes')
        self.assertEqual(d['counts'],dict(P4=0,P5=0))

    def test_board_midpoint_nineteen_cm_from_each_reference_matches(self):
        for sign_x,sign_y,obstacle_x,bay in ((1.,1.43,.61,'P5'),(.61,1.44,1.,'P4')):
            points=[(sign_x-.09+i*.009,sign_y+.19) for i in range(21)]
            points.append((obstacle_x,sign_y+.19-.30))
            d=measure(points)
            self.assertEqual(d['reason'],'single_sign_boxes')
            self.assertEqual(d['counts'][bay],1)

    def test_distant_line_without_obstacle_support_cannot_hide_calibrated_points(self):
        far=[(1.2-.09+i*.009,2.95) for i in range(21)]
        for obstacle,expected in (((1.05,1.14),dict(P4=1,P5=0)),
                                  ((.62,1.14),dict(P4=0,P5=1))):
            d=measure(far+[obstacle])
            self.assertEqual(d['counts'],expected)
            self.assertEqual(d['reason'],'calibrated_boxes')

    def test_empty_box_uses_same_supported_board_not_a_distant_wall(self):
        points=scene(True,False)+[(-.26-.2+i*.009,4.) for i in range(50)]
        d=measure(points)
        self.assertEqual(d['counts'],dict(P4=7,P5=0))
        self.assertEqual(d['boxes'][0]['reference'],d['boxes'][1]['reference'])

    def test_points_beside_a_four_metre_line_are_outside_the_shown_bay_area(self):
        points=[(4.-.09+i*.009,.15) for i in range(21)]+[(4.4,-.15)]
        d=measure(points)
        self.assertEqual(d['counts'],dict(P4=0,P5=0))
        self.assertEqual(d['reason'],'calibrated_boxes')

    def test_both_and_empty_exclude_side_lines(self):
        for a,b in ((True,True),(True,False),(False,True),(False,False)):
            d=measure(scene(a,b))
            self.assertTrue(d['valid'])
            self.assertEqual(d['counts'],dict(P4=7 if a else 0,P5=8 if b else 0))

    def test_obstacles_are_counted_without_any_reference_lines(self):
        d=measure([(1.05,1.14),(.62,1.14)])
        self.assertTrue(d['valid'])
        self.assertEqual(d['counts'],dict(P4=1,P5=1))

    def test_one_reference_line_does_not_disable_boxes(self):
        d=measure(scene()[:21]+[(.62,1.14)])
        self.assertTrue(d['valid'])
        self.assertEqual(d['counts'],dict(P4=0,P5=1))
        self.assertEqual(len(d['boxes']),2)

    def test_one_point_in_one_fresh_frame_selects_without_extra_confirmation(self):
        c=arm(10.,10.)
        d=measure([(1.05,1.14)])
        observe(c,d,9.9,10.,{})
        self.assertEqual(c['frames'],0)
        observe(c,d,10.1,10.1,{})
        observe(c,d,10.1,10.2,{})
        self.assertEqual(c['frames'],1)
        self.assertEqual(decide(c,10.2,10.,{})['target'],'P5')
        observe(c,d,10.3,10.3,{})
        observe(c,d,10.5,10.5,{})
        self.assertEqual(decide(c,10.5,10.,{})['target'],'P5')
        self.assertIsNone(decide(c,12.1,10.,{})['target'])
        self.assertIsNone(decide(c,10.5,11.,{})['target'])

    def test_both_selects_p4_profile_and_empty_frame_preserves_evidence(self):
        c=arm(0.,0.)
        for t in (.1,.2,.3):observe(c,measure(scene()),t,t,{})
        self.assertEqual(decide(c,.3,0.,{})['occupied'],'both_occupied')
        self.assertEqual(decide(c,.3,0.,{})['target'],'P4')
        self.assertEqual(decide(c,.3,0.,{})['entry_s'],1.8)
        observe(c,measure([]),.4,.4,{})
        self.assertEqual(decide(c,.4,0.,{})['target'],'P4')

    def test_occupied_evidence_survives_empty_frames_until_blue_but_not_next_sign(self):
        c=arm(1.,1.)
        observe(c,measure([(1.05,1.14)]),1.1,1.1,{})
        for t in (1.2,1.3,2.,3.,4.):observe(c,measure([]),t,t,{})
        result=decide(c,4.,1.,{})
        self.assertEqual(result['target'],'P5')
        self.assertEqual(result['evidence_stamp'],1.1)
        self.assertEqual(result['measurement']['counts'],dict(P4=1,P5=0))
        self.assertEqual(result['latest_measurement']['counts'],dict(P4=0,P5=0))
        other=arm(5.,5.);observe(other,measure([]),5.1,5.1,{})
        self.assertIsNone(decide(other,5.1,5.,{})['target'])


class BayIntegrationTest(unittest.TestCase):
    def make(self):
        cfg=copy.deepcopy(CONFIG)
        cfg.update(wait_green=False,lidar_enabled=False,left_timed_enabled=True,
                   left_bay_choice=dict(enabled=True),steering_command_scale_rad=.03)
        c=Controller(cfg);self.addCleanup(c.close)
        return c

    def prepare(self,p4=True,p5=False):
        c=self.make()
        c.observe_sign('LEFT',.99,1.,1.)
        c.observe_sign('LEFT',.99,1.1,1.1)
        self.assertEqual(c.pending,'LEFT')
        for t in (1.2,1.3,1.4):observe(c.left_bay_cache,measure(scene(p4,p5)),t,t,{})
        c.marker=((.7,0.),1.4);c.front_marker_stamp=1.4
        c.front_blue_lines=[dict(point=(.7,0.),yaw=0.)]
        c.blue_approach_tick=lambda now:(0,0)
        c.dispatch(1.4)
        # Unit-level action tests start with the stationary wait already over.
        # The production-flow test below exercises the entire approach instead.
        c.blue_approach.update(phase='STOP',stop_until=1.1)
        return c

    def test_blue_freezes_selection_before_turn(self):
        c=self.prepare()
        c.execute('mission','freeze_left_bay',1.4)
        self.assertEqual(c.left_bay_selection['target'],'P5')
        observe(c.left_bay_cache,measure(scene(False,True)),1.5,1.5,{})
        self.assertEqual(c.left_bay_selection['target'],'P5')

    def test_selection_waits_for_scan_captured_after_stationary_wait(self):
        c=self.prepare(True,False)
        c.blue_approach.update(phase='STOP',stop_until=3.)
        c.state='BLUE_STOP'
        # Moving/settling observations must not select the bay.
        observe(c.left_bay_cache,measure(scene(True,False)),2.9,2.9,{})
        self.assertEqual(c.execute('mission','begin_blue_action',3.).value,(0,0))
        self.assertIsNone(c.left_lock)
        self.assertFalse(c.left_bay_selection.get('frozen',False))
        # Even a delayed scan received after the wait must be rejected.
        observe(c.left_bay_cache,measure(scene(True,False)),2.99,3.05,{})
        self.assertEqual(c.execute('mission','begin_blue_action',3.05).value,(0,0))
        self.assertIsNone(c.left_lock)
        observe(c.left_bay_cache,measure(scene(False,True)),3.1,3.1,{})
        c.execute('mission','begin_blue_action',3.1)
        self.assertEqual(c.left_lock['bay_profile']['target'],'P4')
        self.assertEqual(c.left_lock['bay_profile']['entry_s'],1.8)
        self.assertEqual(c.left_lock['bay_profile']['scan_stamp'],3.1)

    def test_empty_stationary_scan_cannot_reuse_moving_occupancy(self):
        c=self.prepare(True,False)
        c.blue_approach.update(phase='STOP',stop_until=3.)
        observe(c.left_bay_cache,measure([]),3.1,3.1,{})
        c.execute('mission','begin_blue_action',3.1)
        self.assertNotIn('bay_profile',c.left_lock)
        self.assertEqual(c.left_bay_selection['status'],'empty')

    def test_non_left_actions_skip_bay_selection_even_with_occupied_cache(self):
        for action in ('RIGHT','STRAIGHT','UTURN','PARKING'):
            c=self.prepare(False,True)
            c.action=action
            self.assertIsNone(c.execute('mission','freeze_left_bay',1.4).value)


    def test_both_profiles_exact_raw_and_timed_straight(self):
        for p4,entry,target in ((True,0.,'P5'),(False,1.8,'P4')):
            c=self.prepare(p4,not p4)
            c.execute('mission','freeze_left_bay',1.4)
            c.execute('mission','begin_blue_action',2.)
            self.assertEqual(c.left_lock['bay_profile']['target'],target)
            # 20 Hz ticks, with no visual handoff allowed to shorten the stages.
            for i in range(int(round((entry+5.2)*20))):
                elapsed=i*.05
                speed,steer=c.execute('turn','left_lock_tick',2.+elapsed).value
                raw=encode_command(speed,steer,c.cfg,0)
                expected=0 if elapsed<entry-1e-8 or elapsed>=entry+3.6-1e-8 else 22
                self.assertEqual((raw['speed_raw'],raw['steering_raw']),(30,expected))
            self.assertEqual(c.execute('turn','left_lock_tick',2.+entry+5.2).value,(0,0))
            self.assertEqual(c.left_lock['phase'],'BAY_SETTLE')
            for i in range(1,15):c.execute('turn','left_lock_tick',2.+entry+5.2+i*.05)
            self.assertEqual(c.left_lock['phase'],'FINISHED')
            self.assertEqual(c.state,'FINISHED')
            self.assertTrue(c.parking_completed)
            # A visible lane must never resume motion after either bay profile.
            for i in range(1,5):
                now=2.+entry+6.+i*.1
                c.observe_lane([(.3,0.),(.6,0.),(.9,0.)],.9,now)
                self.assertEqual(c.tick(now),(0,0))
                self.assertEqual(c.reason,'left_bay_'+target+'_complete')

    def test_empty_boxes_keep_original_left(self):
        c=self.prepare(False,False)
        self.assertIsNone(c.left_bay_selection['target'])
        c.execute('mission','begin_blue_action',2.)
        self.assertNotIn('bay_profile',c.left_lock)

    def test_new_left_cache_is_not_previous_junction(self):
        c=self.prepare();old=c.left_bay_cache
        c.resume_lane()
        c.observe_sign('LEFT',.99,5.,5.);c.observe_sign('LEFT',.99,5.1,5.1)
        self.assertEqual(c.left_bay_cache['frames'],0)
        self.assertNotEqual(c.left_bay_cache['sign_at'],old['sign_at'])

    def test_both_profile_is_p4_without_fault(self):
        c=self.prepare(True,True)
        c.execute('mission','begin_blue_action',2.)
        self.assertEqual(c.left_lock['bay_profile']['entry_s'],1.8)
        self.assertEqual(c.left_lock['bay_profile']['occupied'],'both_occupied')
        self.assertEqual(c.state,'MANEUVER')

    def test_queued_left_has_its_own_scan_cache(self):
        c=self.make();c.state='MANEUVER';c.action='RIGHT'
        c.route_action_started=0.;c.action_started=0.
        c.observe_sign('LEFT',.99,10.,10.);c.observe_sign('LEFT',.99,10.1,10.1)
        self.assertEqual(c.next_direction,'LEFT')
        for t in (10.2,10.3,10.4):observe(c.left_bay_cache,measure(scene(False,True)),t,t,{})
        c.resume_lane()
        self.assertEqual(c.pending,'LEFT')
        self.assertEqual(decide(c.left_bay_cache,10.4,c.pending_at,{})['target'],'P4')

    def test_completed_bay_profile_never_restarts_reverse_search(self):
        c=self.prepare()
        c.cfg.update(parking_enabled=True,parking_mode='forward_center',parking_entry_style='S',
                     parking_entry_speed_raw=16,parking_lidar_enabled=False)
        c.execute('mission','begin_blue_action',2.)
        for i in range(119):
            now=2.+i*.05
            c.lane_stamp=now;c.lane=[]
            c.execute('turn','left_lock_tick',now)
            if now<7.9-1e-9:
                self.assertIsNone(c.execute('parking','turn_reverse_tick',now).value)
        self.assertEqual(c.left_lock['phase'],'FINISHED')
        self.assertEqual(c.state,'FINISHED')
        self.assertTrue(c.parking_completed)
        for i in range(1,10):
            now=7.9+i*.25;c.lane_stamp=now;c.lane=[]
            c.execute('parking','turn_reverse_tick',now)
            self.assertEqual(c.tick(now),(0,0))
            self.assertEqual(c.state,'FINISHED')
        self.assertIsNone(c.parking_entry)
        self.assertNotEqual(c.state,'PARKING')
        c.resume_lane()
        for i in range(12):
            now=11.+i*.25;c.lane_stamp=now;c.lane=[]
            self.assertIsNone(c.execute('parking','turn_reverse_tick',now).value)
        self.assertTrue(c.parking_completed)

    def test_interrupted_profile_does_not_mark_parking_complete(self):
        c=self.prepare();c.execute('mission','begin_blue_action',2.)
        for i in range(40):c.execute('turn','left_lock_tick',2.+i*.05)
        self.assertFalse(c.parking_completed)

    def test_red_cancels_selected_straight_without_resuming_timer(self):
        c=self.prepare();c.execute('mission','begin_blue_action',2.)
        for i in range(74):c.execute('turn','left_lock_tick',2.+i*.05)
        self.assertEqual(c.left_lock['phase'],'TIMED_STRAIGHT')
        c.red=True
        self.assertEqual(c.tick(5.7),(0,0))
        self.assertEqual(c.state,'FAULT')

    def test_stale_lidar_waits_for_stopped_scan(self):
        c=self.make();c.pending='LEFT';c.pending_at=1.
        c.left_bay_cache=arm(1.,1.)
        for t in (1.1,1.2,1.3):observe(c.left_bay_cache,measure(scene()),t,t,{})
        c.marker=((.7,0.),4.);c.front_marker_stamp=4.
        c.front_blue_lines=[dict(point=(.7,0.),yaw=0.)]
        c.blue_approach_tick=lambda now:(0,0)
        c.dispatch(4.)
        c.blue_approach.update(phase='STOP',stop_until=4.)
        self.assertEqual(c.execute('mission','begin_blue_action',4.1).value,(0,0))
        self.assertIsNone(c.left_lock)
        self.assertEqual(c.reason,'blue_stop_wait_left_bay_scan')

    def test_loaded_vehicle_config_enables_global_choice(self):
        import os
        from robot.common.config import load_config
        from robot.common.contracts import validate_config
        root=os.path.join(os.path.dirname(__file__),'..','config')
        cfg=load_config(root)
        validate_config(cfg)
        self.assertTrue(cfg['left_bay_choice']['enabled'])
        self.assertEqual(cfg['left_bay_choice']['turn_s'],3.6)

    def test_selected_bay_shares_blue_approach_and_stop_before_own_sequence(self):
        from test_blue_timed_production import SETTINGS
        for occupied,expected_entry in (('P4',0.),('P5',1.8),('both',1.8),('empty',None)):
            c=self.make()
            c.cfg.update(blue_timed_enabled=True,blue_stop_test=dict(SETTINGS),intersection_wait_s=1.)
            c.cfg['blue_stop_test']['forward_seconds']=1.
            c.observe_sign('LEFT',.99,1.,1.);c.observe_sign('LEFT',.99,1.1,1.1)
            for t in (1.2,1.3,1.4):
                observe(c.left_bay_cache,measure(scene(occupied in ('P4','both'),occupied in ('P5','both'))),t,t,{})
            c.action='LEFT';c.pending=None;c.state='BLUE_APPROACH'
            c.last_blue_trigger=dict(stamp=1.4)
            c.blue_approach=dict(image_timed=True,phase='STOP_LINE',point=(1,0),yaw=0,
                                observed_stamp=1.4,started=1.4)
            cam=c.cfg['front_camera']
            for t,row in ((1.5,.3),(1.6,.3),(1.7,.3),(1.8,.7),(1.9,.7),(2.,.7)):
                # Replay the field failure: occupancy disappears before blue.
                observe(c.left_bay_cache,measure([]),t,t,{})
                c.front_marker_stamp=t
                c.front_blue_image_lines=[dict(x=(cam['origin_v']-row*(cam['bev_height']-1))/cam['pixels_per_m'],y=0,yaw=0,length=.6)]
                c.execute('turn','blue_approach_tick',t)
            self.assertEqual(c.state,'BLUE_APPROACH')
            self.assertFalse((c.left_bay_selection or {}).get('frozen',False))
            for t in (2.1,2.3,2.5,2.7,2.9,2.99):
                c.front_marker_stamp=t
                speed,steer=c.execute('turn','blue_approach_tick',t).value
                raw=encode_command(speed,steer,c.cfg,0)
                self.assertEqual((raw['speed_raw'],raw['steering_raw']),(20,0))
                self.assertIsNone(c.left_lock)
            c.front_marker_stamp=3.
            self.assertEqual(c.execute('turn','blue_approach_tick',3.).value,(0,0))
            self.assertEqual(c.state,'BLUE_STOP')
            self.assertEqual(c.blue_approach['phase'],'STOP')
            self.assertFalse((c.left_bay_selection or {}).get('frozen',False))
            self.assertIsNone(c.left_lock)
            for t in (3.1,3.5,3.99):
                c.front_marker_stamp=t
                self.assertEqual(c.execute('turn','blue_approach_tick',t).value,(0,0))
                self.assertIsNone(c.left_lock)
            c.front_marker_stamp=4.
            self.assertEqual(c.execute('turn','blue_approach_tick',4.).value,(0,0))
            self.assertIsNone(c.left_lock)
            self.assertEqual(c.reason,'blue_stop_wait_left_bay_scan')
            observe(c.left_bay_cache,measure(scene(occupied in ('P4','both'),occupied in ('P5','both'))),4.1,4.1,{})
            c.front_marker_stamp=4.1
            c.execute('turn','blue_approach_tick',4.1)
            self.assertEqual(c.left_bay_selection['decision_stamp'],4.1)
            self.assertEqual(c.state,'MANEUVER')
            self.assertIsNone(c.blue_approach)
            if expected_entry is None:
                self.assertNotIn('bay_profile',c.left_lock)
                continue
            self.assertEqual(c.left_lock['bay_profile']['entry_s'],expected_entry)
            speed,steer=c.execute('turn','left_lock_tick',4.15).value
            out=encode_command(speed,steer,c.cfg,0)
            self.assertEqual((out['speed_raw'],out['steering_raw']),(30,22 if expected_entry==0 else 0))

    def test_stopped_selection_is_frozen_for_the_maneuver(self):
        c=self.prepare()
        self.assertIsNone(c.left_bay_selection['target'])
        for t in (1.5,1.6,1.7):observe(c.left_bay_cache,measure(scene(False,True)),t,t,{})
        selected=c.execute('mission','freeze_left_bay',1.7).value
        self.assertEqual(selected['target'],'P4')
        for t in (1.8,1.9,2.):observe(c.left_bay_cache,measure(scene(True,False)),t,t,{})
        self.assertEqual(c.execute('mission','freeze_left_bay',2.).value['target'],'P4')


if __name__=='__main__':unittest.main()
