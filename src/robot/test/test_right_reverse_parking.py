"""Right-exit loss and rear S entry, offline: never publishes vehicle commands."""
import copy
import json
import math
import unittest

from test_core import CONFIG
from robot.master.controller import Controller
from robot.parking.reverse_entry import ReverseStraightEntry
from robot.common.geometry import bicycle, world
from robot.common.contracts import ground


class RightReverseParkingTests(unittest.TestCase):
    def cfg(self):
        cfg = copy.deepcopy(CONFIG)
        cfg.pop('parking_slot',None)
        cfg.update(parking_enabled=True,
                   parking_mode='forward_center', parking_entry_style='S',
                   wait_green=False, parking_lidar_enabled=False,
                   steering_command_scale_rad=.03, parking_entry_speed_raw=12)
        return cfg

    def core(self, previous='RIGHT'):
        core = Controller(self.cfg())
        self.addCleanup(core.close)
        core.state, core.action = 'LANE', None
        core.last_completed_action = previous
        core.scan_ready = lambda now: True
        core.checked_command = lambda command, now, allow_bypass: command
        return core

    def frame(self, core, stamp, valid):
        core.lane_stamp = stamp
        core.lane_confidence = .9
        core.lane = [(.4, 0.), (.6, 0.), (.8, 0.)] if valid else []

    def gate(self, core, now):
        return core.execute('parking', 'turn_reverse_tick', now).value

    def arm_reverse(self, core):
        self.frame(core,8.,True);self.gate(core,8.)
        for stamp in (8.4,8.8,9.2,9.6,10.,10.4):
            self.frame(core,stamp,False);self.gate(core,stamp)

    def test_left_and_right_routes_reverse_without_special_switch(self):
        for previous in ('LEFT','RIGHT'):
            core=self.core(previous)
            self.assertNotIn('parking_reverse_after_right',core.cfg)
            self.arm_reverse(core)
            self.assertIsInstance(core.parking_entry,ReverseStraightEntry)
            self.assertEqual(core.tick(10.8),(-12,0.))
            self.assertTrue(core.parking_entry.observe_sign('PARKING',.99,10.9,10.9,anchor=(-.9,.06)))
            core.parking_entry.observe(self.lines(offset=.06),11.,core.pose)
            self.assertNotEqual(core.tick(11.1)[1],0.)

    def test_active_left_turn_finishes_before_loss_can_reverse(self):
        core=self.core();core.state='MANEUVER';core.action='LEFT'
        core.left_lock=dict(phase='TIMED_TURN')
        self.arm_reverse(core)
        self.assertIsNone(core.parking_entry)

    def test_front_p_after_left_does_not_dispatch_forward_parking(self):
        core=self.core('LEFT');core.pending='PARKING'
        self.assertIsNone(core.dispatch(10.))
        self.assertIsNone(core.parking_entry)

    def test_left_handoff_records_route_for_rear_entry(self):
        core=self.core(None)
        core.state='MANEUVER';core.action='LEFT';core.action_source='sign'
        core.execute('mission','resume_lane')
        self.assertEqual(core.last_completed_action,'LEFT')
        self.arm_reverse(core)
        self.assertIsInstance(core.parking_entry,ReverseStraightEntry)

    def test_straight_route_keeps_front_p_entry(self):
        core=self.core('STRAIGHT');core.pending='PARKING'
        core.dispatch(10.)
        self.assertIsNotNone(core.parking_entry)
        self.assertNotEqual(getattr(core.parking_entry,'camera_source','front'),'rear')

    def test_loss_requires_two_seconds_of_fresh_empty_frames(self):
        core=self.core()
        self.frame(core,10.,True);self.gate(core,10.)
        for stamp in (10.1,10.4,10.8,11.2,11.6,12.0):
            self.frame(core,stamp,False);self.gate(core,stamp)
            self.assertIsNone(core.parking_entry)
        self.frame(core,12.1,False);self.gate(core,12.1)
        self.assertIsInstance(core.parking_entry,ReverseStraightEntry)

    def test_visible_low_confidence_path_resets_loss_confirmation(self):
        core=self.core()
        self.frame(core,10.,True);self.gate(core,10.)
        for stamp in (10.1,10.5,10.9,11.3):
            self.frame(core,stamp,False);self.gate(core,stamp)
        self.frame(core,11.5,True);core.lane_confidence=.01
        self.gate(core,11.5)
        self.assertIsNone(core.turn_reverse_watch['loss_since'])
        for stamp in (11.6,12.,12.4,12.8,13.2,13.5):
            self.frame(core,stamp,False);self.gate(core,stamp)
            self.assertIsNone(core.parking_entry)
        self.frame(core,13.6,False);self.gate(core,13.6)
        self.assertIsInstance(core.parking_entry,ReverseStraightEntry)

    def test_camera_gap_cannot_count_as_two_seconds_of_empty_frames(self):
        core=self.core()
        self.frame(core,10.,True);self.gate(core,10.)
        self.frame(core,10.1,False);self.gate(core,10.1)
        self.gate(core,12.2)
        self.frame(core,12.3,False);self.gate(core,12.3)
        self.assertIsNone(core.parking_entry)
        self.assertEqual(core.turn_reverse_watch['loss_since'],12.3)

    def test_right_then_confirmed_loss_stops_before_reverse(self):
        core = self.core()
        self.frame(core, 10., True)
        self.assertIsNone(self.gate(core, 10.))
        self.frame(core, 10.1, False)
        self.assertEqual(self.gate(core, 10.1), (0, 0.))
        self.assertIsNone(core.parking_entry)
        for stamp in (10.4,10.8,11.2,11.6,12.,12.1):
            self.frame(core,stamp,False)
            self.assertEqual(self.gate(core,stamp),(0,0.))
        self.assertIsInstance(core.parking_entry, ReverseStraightEntry)
        self.assertEqual(core.state, 'PARKING')
        self.assertEqual(core.tick(12.15), (0, 0.))
        self.assertNotEqual(core.state, 'FAULT')  # no preceding STRAIGHT required

    def test_production_right_loss_reverses_straight_until_rear_p(self):
        core=self.core()
        self.arm_reverse(core)
        entry=core.parking_entry
        for stamp in (10.6,10.7):
            entry.observe(self.lines(offset=.06),stamp,core.pose)
            entry.observe_blue([dict(point=(-.6,0.),yaw=.12,length=.7)],stamp,core.pose)
        self.assertEqual(core.tick(10.9),(-12,0.))
        self.assertEqual(core.reason,'parking_reverse_straight_wait_p')
        self.assertIsNone(entry.bottom)
        self.assertEqual(core.tick(50.),(-12,0.))  # no visual-entry timeout before P
        self.assertFalse(entry.observe_sign('PARKING',.99,48.,50.))  # stale
        self.assertFalse(entry.observe_sign('RIGHT',.99,50.,50.))
        self.assertTrue(entry.observe_sign('PARKING',.99,50.1,50.1,anchor=(-.9,.06)))
        entry.observe(self.lines(offset=.06),50.2,core.pose)
        self.assertNotEqual(core.tick(50.3)[1],0.)
        self.assertEqual(entry.debug['rear_sign']['decision'],'parking_started')
        self.assertEqual(core.tick(86.),(0,0.))  # visual-entry timeout still retained

    def test_rear_p_rejects_prearm_low_confidence_duplicates_and_front_p(self):
        entry=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.,wait_for_sign=True)
        self.assertFalse(entry.observe_sign('PARKING',.99,9.9,10.1))
        self.assertFalse(entry.observe_sign('PARKING',.1,10.1,10.1))
        self.assertFalse(entry.observe_sign('PARKING',.99,10.1,10.1))
        entry.sign_anchor=(1.,0.)
        self.assertEqual(entry.command(10.5,(0.,0.,0.)),(-12,0.))
        self.assertIsNone(entry.parking_started)
        self.assertTrue(entry.observe_sign('PARKING',.99,10.6,10.6,anchor=(-.9,0.)))
        self.assertFalse(entry.observe_sign('PARKING',.99,10.7,10.7))
        self.assertEqual(entry.parking_started,10.6)

    def test_rear_sign_callback_checks_camera_and_never_queues_front_route(self):
        import threading
        import robot.master.controller_node as node_module
        from test_sign_training_roi import Value
        core=self.core();core.state=core.action='PARKING'
        core.parking_entry=ReverseStraightEntry(core.cfg,core.pose,10.,wait_for_sign=True)
        node=node_module.Node.__new__(node_module.Node)
        node.cfg,node.core,node.lock=core.cfg,core,threading.RLock()
        node.history=[(10.4,(1.,2.,math.pi/2.))]
        node.rear_parking_projector=Value(position=lambda bounds:(-.9,.24) if bounds==[200,100,60,60] else None)
        original=node_module.rospy
        node_module.rospy=Value(Time=Value(now=lambda:Value(to_sec=lambda:10.5)),
            loginfo=lambda *args:None,logwarn_throttle=lambda *args:None)
        try:
            for source in ('front','rear'):
                node.rear_sign(Value(data=json.dumps(dict(camera_source=source,
                    label='PARKING',confidence=.99,stamp=10.4,sign_bounds=[200,100,60,60]))))
                if source=='front':self.assertIsNone(core.parking_entry.parking_started)
            self.assertEqual(core.parking_entry.parking_started,10.5)
            self.assertAlmostEqual(core.parking_entry.inner.sign_anchor[0],.76)
            self.assertAlmostEqual(core.parking_entry.inner.sign_anchor[1],-1.1)
            self.assertIsNone(core.pending)
            self.assertIsNone(core.next_direction)
        finally:
            node_module.rospy=original

    def test_other_route_or_disabled_does_not_reverse(self):
        for previous in ('UTURN', 'STRAIGHT', None):
            core = self.core(previous)
            self.frame(core, 10., True); self.gate(core, 10.)
            self.frame(core, 10.4, False); self.gate(core, 10.4)
            self.frame(core, 10.8, False); self.gate(core, 10.8)
            self.assertIsNone(core.parking_entry)
        core = self.core(); core.cfg['parking_enabled'] = False
        self.frame(core, 10., True); self.gate(core, 10.)
        self.frame(core, 10.4, False)
        self.assertIsNone(self.gate(core, 10.4))

    def test_never_acquired_stale_or_duplicate_frames_cannot_trigger(self):
        core = self.core()
        self.frame(core, 10., False); self.gate(core, 10.)
        self.frame(core, 10.4, False); self.gate(core, 10.4)
        self.assertIsNone(core.parking_entry)
        self.frame(core, 11., True); self.gate(core, 11.)
        self.frame(core, 11.1, False); self.gate(core, 11.1)
        self.gate(core, 11.4)  # one source image is not two confirmations
        self.gate(core, 12.)   # stale is not missing paint
        self.assertIsNone(core.parking_entry)

    def test_recovery_resets_loss_and_new_instruction_cancels_route(self):
        core = self.core()
        self.frame(core, 10., True); self.gate(core, 10.)
        self.frame(core, 10.1, False); self.gate(core, 10.1)
        self.frame(core, 10.4, True); self.gate(core, 10.4)
        self.frame(core, 10.5, False); self.gate(core, 10.5)
        self.assertIsNone(core.parking_entry)
        core.pending = 'LEFT'
        self.frame(core, 10.9, False)
        self.assertIsNone(self.gate(core, 10.9))
        self.assertIsNone(core.parking_entry)

    def test_pending_p_does_not_dispatch_forward_after_right(self):
        core = self.core(); core.pending = 'PARKING'
        self.assertIsNone(core.dispatch(10.))
        self.assertIsNone(core.parking_entry)

    def test_right_exit_alignment_can_arm_before_lane_handoff(self):
        core = self.core(None)
        core.state, core.action = 'MANEUVER', 'RIGHT'
        core.right_lock = dict(phase='ALIGN_LANE', exit_started=7.)
        self.arm_reverse(core)
        self.assertIsInstance(core.parking_entry, ReverseStraightEntry)

    def lines(self, depth=.8, offset=0.):
        return [[(-.3, -.19+offset), (-depth, -.19+offset)],
                [(-.3, .19+offset), (-depth, .19+offset)],
                [(-depth, -.19+offset), (-depth, .19+offset)]]

    def test_rear_bottom_acquisition_does_not_depend_on_side_endpoints(self):
        entry=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.)
        # Visible side fragments end at 0.6 m; the actual transverse bottom
        # is at 0.9 m. Side endpoints must not define the stopping distance.
        lines=self.lines(depth=.6)[:2]+[[(-.9,-.19),(-.9,.19)]]
        entry.observe(lines,10.1,(0.,0.,0.))
        self.assertIsNone(entry.bottom)
        entry.observe(lines,10.2,(0.,0.,0.))
        self.assertIsNotNone(entry.bottom)
        self.assertEqual(entry.inner.bottom_source,'observed_bottom')
        expected=.9-self.cfg()['rear_overhang']-self.cfg()['parking_bottom_clearance_m']
        self.assertAlmostEqual(entry.remaining(),expected)
        # Later shorter side rails and a different cross-line cannot replace
        # the confirmed bottom with their common endpoint.
        entry.observe(self.lines(depth=.6),10.3,(0.,0.,0.))
        self.assertAlmostEqual(entry.remaining(),expected)
        self.assertEqual(entry.inner.bottom_stamp,10.2)

    def test_sides_alone_never_finish_or_set_bottom_distance(self):
        entry=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.)
        for stamp,x in ((10.4,0.),(10.6,-.5),(10.8,-1.)):
            pose=(x,0.,0.)
            lines=[[world(pose,p) for p in line] for line in self.lines(offset=.05)[:2]]
            entry.observe(lines,stamp,pose)
            speed,steer=entry.command(stamp,pose)
            self.assertLess(speed,0)
            self.assertNotEqual(steer,0)
            self.assertIsNone(entry.bottom)
            self.assertIsNone(entry.remaining())
            self.assertNotEqual(entry.status,'FINISHED')

    def test_single_frame_bottom_near_side_ends_cannot_finish(self):
        entry=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.)
        entry.observe(self.lines(),10.1,(0.,0.,0.))
        self.assertIsNone(entry.bottom)
        entry.observe(self.lines()[:2],10.2,(0.,0.,0.))
        self.assertLess(entry.command(10.5,(-.8,0.,0.))[0],0)
        self.assertIsNone(entry.bottom)

    def test_rear_geometry_produces_negative_speed_after_gear_pause(self):
        entry = ReverseStraightEntry(self.cfg(), (0.,0.,0.), 10.)
        entry.observe(self.lines(), 10.1, (0.,0.,0.))
        self.assertEqual(entry.command(10.1, (0.,0.,0.)), (0,0.))
        speed, steer = entry.command(10.5, (0.,0.,0.))
        self.assertLess(speed, 0)
        self.assertAlmostEqual(steer, 0., places=6)
        self.assertEqual(entry.debug['camera_source'], 'rear')

    def test_reverse_steer_moves_rear_towards_lateral_target(self):
        for offset in (-.06, .06):
            entry = ReverseStraightEntry(self.cfg(), (0.,0.,0.), 10.)
            entry.observe(self.lines(offset=offset), 10.2, (0.,0.,0.))
            speed, steer = entry.command(10.5, (0.,0.,0.))
            self.assertLess(speed, 0)
            moved = bicycle((0.,0.,0.), -.1, steer, .26)
            self.assertGreater(moved[1]*offset, 0)

    def test_stop_uses_rear_bumper_not_front_bumper(self):
        cfg = self.cfg()
        entry = ReverseStraightEntry(cfg, (0.,0.,0.), 10.)
        entry.observe(self.lines(), 10.1, (0.,0.,0.))
        entry.observe(self.lines(), 10.2, (0.,0.,0.))
        expected = .8-cfg['rear_overhang']-cfg['parking_bottom_clearance_m']
        self.assertAlmostEqual(entry.remaining(), expected, places=6)
        pose = (-expected-.001,0.,0.)
        entry.observe(self.lines(), 10.5, pose)
        self.assertEqual(entry.command(10.5, pose), (0,0.))
        self.assertEqual(entry.status, 'FINISHED')

    def test_missing_geometry_reverses_and_locked_geometry_matches_front_memory(self):
        entry = ReverseStraightEntry(self.cfg(), (0.,0.,0.), 10.)
        self.assertEqual(entry.command(10.5, (0.,0.,0.)), (-12,0.))
        entry.observe(self.lines(), 10.6, (0.,0.,0.))
        self.assertLess(entry.command(12., (0.,0.,0.))[0],0)

    def test_empty_rear_has_no_separate_search_stage(self):
        entry = ReverseStraightEntry(self.cfg(), (0.,0.,0.), 10.)
        entry.observe([],10.2,(0.,0.,0.))
        self.assertEqual(entry.command(10.5,(0.,0.,0.)),(-12,0.))
        entry.observe([],10.6,(0.,0.,0.))
        self.assertEqual(entry.command(10.6,(0.,0.,0.)),(-12,0.))
        self.assertEqual(entry.reason,'parking_reverse_missing_white_straight')
        entry.observe(self.lines(offset=.06),10.7,(-.01,0.,0.))
        speed,steer=entry.command(10.7,(-.01,0.,0.))
        self.assertLess(speed,0)
        self.assertNotEqual(steer,0.)
        self.assertNotEqual(entry.reason,'parking_reverse_search_geometry')
        # Once a bay was acquired, loss must not restart blind searching.
        entry.observe([],10.8,(-.01,0.,0.))
        entry.command(10.8,(-.01,0.,0.))
        self.assertNotEqual(entry.reason,'parking_reverse_search_geometry')

    def test_front_left_reference_becomes_rear_right_reference(self):
        from robot.parking.entry import ExplicitStraightEntry
        for yaw in (0.,.7,-1.2):
            cfg=self.cfg();pose=(1.2,-.4,yaw)
            front=ExplicitStraightEntry(cfg,(0.,0.,0.),10.)
            rear=ReverseStraightEntry(cfg,pose,10.)
            def observe(lines,stamp):
                front.observe(lines,stamp,(0.,0.,0.))
                rear.observe([[world(pose,(-x,y)) for x,y in line] for line in lines],stamp,pose)
            pair=[[(.3,-.14),(.9,-.14)],[(.3,.24),(.9,.24)]]
            observe(pair,10.2)
            front.command(10.5,(0.,0.,0.));rear.command(10.5,pose)
            observe([[(.4,.24),(.8,.24)]],10.6)
            fs,fa=front.command(10.6,(0.,0.,0.));rs,ra=rear.command(10.6,pose)
            self.assertGreater(fs,0)
            self.assertEqual(rs,-fs)
            # Both directions use the same centerline and full chassis range.
            self.assertGreater(ra*fa,0.)
            self.assertGreaterEqual(abs(ra),abs(fa))
            self.assertAlmostEqual(ra,rear.inner.limited_steer(rear.inner.single))
            self.assertEqual(front.debug['steering_reference'],'left_side')
            self.assertEqual(rear.debug['steering_reference'],'right_side')
            self.assertTrue(rear.debug['right_seen'])
            self.assertFalse(rear.debug['left_seen'])
            self.assertTrue(all(y<0 for x,y in rear.debug['side_local']))
            self.assertEqual(rear.reason,'parking_follow_right_side')
            observe([[(.4,-.14),(.8,-.14)]],10.7)
            speed,steer=rear.command(10.7,pose)
            self.assertEqual(speed,-12)
            self.assertGreater(steer*ra,0.)
            self.assertEqual(rear.debug['steering_reference'],'left_side')
            self.assertTrue(rear.debug['left_seen'])
            self.assertFalse(rear.debug['right_seen'])

    def test_right_side_corrects_without_waiting_for_a_pair_or_p_sign(self):
        for offset in (-.04,.04):
            cfg=self.cfg();cfg['parking_entry_speed_raw']=16
            entry=ReverseStraightEntry(cfg,(0.,0.,0.),10.)
            entry.observe([[(-.3,.19+offset),(-.8,.19+offset)]],10.2,(0.,0.,0.))
            speed,steer=entry.command(10.5,(0.,0.,0.))
            self.assertEqual(speed,-16)
            self.assertEqual(entry.reason,'parking_follow_right_side')
            self.assertEqual(entry.debug['steering_reference'],'right_side')
            self.assertGreater(bicycle((0.,0.,0.),-.1,steer,.26)[1]*offset,0)

    def test_reverse_uses_configured_front_entry_and_final_speeds(self):
        cfg=self.cfg();cfg.update(parking_entry_speed_raw=16,parking_final_speed_raw=12)
        entry=ReverseStraightEntry(cfg,(0.,0.,0.),10.)
        self.assertEqual(entry.command(10.5,(0.,0.,0.)),(-16,0.))
        depth=cfg['rear_overhang']+cfg['parking_bottom_clearance_m']+.2
        for stamp in (10.6,10.7):
            entry.observe([[(-depth,-.19),(-depth,.19)]],stamp,(0.,0.,0.))
        self.assertEqual(entry.command(10.8,(0.,0.,0.)),(-12,0.))

    def test_front_lines_cannot_refresh_reverse_entry(self):
        core = self.core()
        core.parking_entry = ReverseStraightEntry(core.cfg, core.pose, 10.)
        core.observe_ground(dict(source='front', part='parking_lines',
                                 lines=self.lines(), markers=[], slots=[]), 10.2)
        self.assertLess(core.parking_entry.stamp, 10.)
        core.observe_ground(dict(source='rear', part='parking_lines',
                                 lines=self.lines(), markers=[], slots=[]), 10.3)
        self.assertEqual(core.parking_entry.stamp, 10.3)

    def test_real_command_guard_preserves_rear_steering_reason_and_log_geometry(self):
        from robot.master.controller_node import Node
        core=Controller(self.cfg());self.addCleanup(core.close)
        core.state=core.action='PARKING'
        core.parking_entry=ReverseStraightEntry(core.cfg,core.pose,10.)
        core.parking_entry.observe(self.lines(offset=.04),10.2,core.pose)
        command=core.tick(10.5)
        self.assertLess(command[0],0)
        self.assertNotEqual(command[1],0)
        self.assertEqual(core.reason,'parking_center_between_sides')
        node=Node.__new__(Node);node.core=core
        node.queue_control_event(10.5,True,command)
        event=node.control_events[-1]
        self.assertEqual(event['parking_entry']['steering_reference'],'both_sides')
        self.assertEqual(len(event['parking_entry']['pair_local']),2)
        core.parking_entry.inner.debug['bottom_detection_reason']='no_transverse_segments'
        node.queue_control_event(10.6,True,command)
        self.assertEqual(len(node.control_events),1)
        node.queue_control_event(11.6,True,command)
        self.assertEqual(len(node.control_events),2)
        self.assertEqual(node.control_events[-1]['parking_entry']['bottom_detection_reason'],
                         'no_transverse_segments')
        core.state='LANE'
        node.queue_control_event(11.7,True,command)
        node.queue_control_event(13.,True,command)
        self.assertEqual(len(node.control_events),3)

    def test_rear_blue_heading_reaches_control_without_becoming_bottom(self):
        core=self.core();core.state=core.action='PARKING'
        core.parking_entry=ReverseStraightEntry(core.cfg,core.pose,10.)
        for stamp in (10.2,10.3):
            raw=json.dumps(dict(source='rear',part='parking_lines',frame='base_link',
                stamp=stamp,lines=[],markers=[],slots=[],
                blue_lines=[dict(x=-.6,y=0.,yaw=.12,length=.7)]))
            data,accepted=ground(raw,stamp,1.25);core.observe_ground(data,accepted)
        speed,steer=core.tick(10.5)
        self.assertLess(speed,0)
        self.assertLess(steer,0)  # negative velocity must turn body towards +yaw
        self.assertIsNone(core.parking_entry.bottom)
        self.assertEqual(core.parking_entry.debug['steering_reference'],'blue_heading')
        self.assertTrue(core.parking_entry.debug['blue_used'])
        self.assertEqual(core.tick(11.)[1],0.)  # no stale blue heading held

    def test_blue_cannot_override_measured_pair_correction(self):
        entry=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.)
        for stamp in (10.2,10.3):
            entry.observe(self.lines(offset=.04)[:2],stamp,(0.,0.,0.))
            entry.observe_blue([dict(point=(-.6,0.),yaw=0.,length=.7)],stamp,(0.,0.,0.))
        speed,steer=entry.command(10.5,(0.,0.,0.))
        self.assertGreater(steer,0)
        self.assertEqual(entry.debug['steering_reference'],'both_sides')
        self.assertFalse(entry.debug['blue_used'])
        # An unrelated blue orientation must not rotate an observed bay.
        for stamp in (10.6,10.7):
            entry.observe(self.lines(offset=.04)[:2],stamp,(0.,0.,0.))
            entry.observe_blue([dict(point=(-.6,0.),yaw=.5,length=.7)],stamp,(0.,0.,0.))
        entry.command(10.8,(0.,0.,0.))
        self.assertFalse(entry.debug['blue_used'])
        self.assertEqual(entry.debug['blue_reject_reason'],'bay_edges_priority')

    def test_short_or_unconfirmed_blue_never_steers(self):
        for length in (.1,.7):
            entry=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.)
            entry.observe_blue([dict(point=(-.6,0.),yaw=.12,length=length)],10.2,(0.,0.,0.))
            self.assertEqual(entry.command(10.5,(0.,0.,0.))[1],0.)

    def test_reverse_timeout_and_estop(self):
        entry = ReverseStraightEntry(self.cfg(), (0.,0.,0.), 10.)
        self.assertEqual(entry.command(50., (0.,0.,0.)), (0,0.))
        self.assertEqual(entry.status, 'FAULT')
        core = self.core(); core.state = core.action = 'PARKING'
        core.parking_entry = ReverseStraightEntry(core.cfg, core.pose, 10.)
        core.estop = True
        self.assertEqual(core.tick(10.5), (0,0.))
        self.assertEqual(core.reason, 'emergency_stop')

    def test_reverse_search_continues_beyond_old_limit_before_or_after_p(self):
        for seen_p in (False,True):
            entry=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.,wait_for_sign=True)
            self.assertLess(entry.command(10.5,(-1.7,0.,0.))[0],0)
            if seen_p:
                entry.observe_sign('PARKING',.99,10.6,10.6,anchor=(-3.,0.))
            self.assertLess(entry.command(10.7,(-1.899,0.,0.))[0],0)
            self.assertIsNone(entry.debug['travel_limit_m'])
            for now,x in ((10.8,-1.9),(10.9,-3.),(11.,-2.5),(11.1,-3.)):
                self.assertLess(entry.command(now,(x,0.,0.))[0],0)
                self.assertNotEqual(entry.status,'FAULT')
            self.assertAlmostEqual(entry.debug['travel_m'],4.)

    def test_losing_all_bay_geometry_does_not_restore_distance_stop(self):
        entry=ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.)
        pose=(-2.4,0.,0.)
        entry.observe([[world(pose,p) for p in line] for line in self.lines()[:2]],10.4,pose)
        self.assertLess(entry.command(10.5,pose)[0],0)
        entry.observe([],10.6,pose)
        self.assertLess(entry.command(10.7,pose)[0],0)
        self.assertIsNone(entry.debug['travel_limit_m'])

    def test_full_controller_reverse_search_has_no_distance_stop(self):
        core=self.core();core.state=core.action='PARKING'
        core.parking_entry=ReverseStraightEntry(core.cfg,core.pose,10.,wait_for_sign=True)
        core.set_pose((-3.,0.,0.),10.5)
        self.assertEqual(core.tick(10.5),(-12,0.))
        self.assertEqual(core.state,'PARKING')
        self.assertEqual(core.reason,'parking_reverse_straight_wait_p')

    def test_reverse_geometry_can_continue_beyond_search_cap_and_stop_at_bottom(self):
        core=self.core();core.state=core.action='PARKING'
        # Even a small forward-parking limit must not constrain rear entry.
        core.cfg['parking_blue_max_travel_m']=.5
        core.parking_entry=ReverseStraightEntry(core.cfg,core.pose,10.)
        for now,x in ((10.5,-.49),(11.,-1.7),(12.,-2.1),(13.,-3.)):
            core.set_pose((x,0.,0.),now)
            core.parking_entry.observe([[world(core.pose,p) for p in line]
                for line in self.lines()[:2]],now-.01,core.pose)
            speed,steer=core.tick(now)
            self.assertEqual(speed,-12)
            self.assertAlmostEqual(steer,0.)
            self.assertEqual(core.state,'PARKING')
            self.assertIsNone(core.parking_entry.debug['travel_limit_m'])
        self.assertAlmostEqual(core.parking_entry.debug['travel_m'],3.)
        # Removing the distance cap must not remove the measured bottom stop.
        lines=[[world(core.pose,p) for p in line] for line in self.lines()]
        core.parking_entry.observe(lines,13.05,core.pose)
        core.parking_entry.observe(lines,13.1,core.pose)
        remaining=core.parking_entry.remaining()
        self.assertIsNotNone(remaining)
        core.set_pose((core.pose[0]-remaining-.001,0.,0.),13.2)
        core.parking_entry.observe(lines,13.2,core.pose)
        self.assertEqual(core.tick(13.2),(0,0.))
        self.assertEqual(core.state,'FINISHED')

    def test_full_controller_uses_rear_memory_without_front_refresh(self):
        core = self.core()
        self.frame(core,8.,True)
        self.assertGreater(core.tick(8.)[0],0)
        for stamp in (8.4,8.8,9.2,9.6,10.,10.4):
            self.frame(core,stamp,False)
            self.assertEqual(core.tick(stamp),(0,0.))
        self.assertTrue(core.parking_entry.observe_sign('PARKING',.99,10.5,10.5,anchor=(-.9,0.)))
        raw = json.dumps(dict(source='rear',part='parking_lines',frame='base_link',
                              stamp=10.6,lines=self.lines(),markers=[],slots=[]))
        data, stamp = ground(raw,10.6,1.25)
        core.observe_ground(data,stamp)
        self.assertLess(core.tick(10.8)[0], 0)
        core.observe_ground(dict(source='front',part='parking_lines',
                                 lines=self.lines(),markers=[],slots=[]),12.)
        self.assertLess(core.tick(12.)[0],0)
        self.assertEqual(core.parking_entry.stamp,10.6)

    def test_full_transition_reverses_without_waiting_for_rear_geometry(self):
        core=self.core()
        self.frame(core,8.,True);core.tick(8.)
        for stamp in (8.4,8.8,9.2,9.6,10.,10.4):
            self.frame(core,stamp,False);core.tick(stamp)
        for stamp in (10.6,10.8):
            raw=json.dumps(dict(source='rear',part='parking_lines',frame='base_link',
                                stamp=stamp,lines=[],markers=[],slots=[]))
            data,accepted_stamp=ground(raw,stamp,1.25)
            core.observe_ground(data,accepted_stamp)
        self.assertEqual(core.tick(10.9),(-12,0.))
        self.assertEqual(core.reason,'parking_reverse_straight_wait_p')
        self.assertEqual(core.parking_entry.debug['rear_line_count'],0)
        self.assertEqual(core.parking_entry.debug['camera_source'],'rear')
        self.assertEqual(core.tick(11.4),(-12,0.))
        self.assertEqual(core.reason,'parking_reverse_straight_wait_p')

    def test_nonzero_world_pose_and_reverse_steering_calibration(self):
        for sign in (1, -1):
            cfg = self.cfg()
            cfg['chassis_calibration'] = dict(reverse_steering_sign=sign,
                                              reverse_mps_per_raw=.008)
            pose = (1.2, -.4, .7)
            lines = [[world(pose,p) for p in line] for line in self.lines(offset=.05)]
            entry = ReverseStraightEntry(cfg,pose,10.)
            entry.observe(lines,10.2,pose)
            speed, steer = entry.command(10.5,pose)
            self.assertLess(speed,0)
            self.assertGreater(steer*sign,0)

    def test_front_sign_cannot_retarget_rear_task(self):
        entry = ReverseStraightEntry(self.cfg(),(0.,0.,0.),10.)
        entry.sign_anchor = (1.,0.)
        self.assertIsNone(entry.inner.sign_anchor)
        entry.observe(self.lines(),9.9,(0.,0.,0.))
        self.assertIsNone(entry.inner.bottom)

    def test_oblique_bottom_stop_matches_front_with_rear_bumper(self):
        from robot.parking.entry import ExplicitStraightEntry
        cfg=self.cfg();origin=(1.2,-.4,.7)
        front=ExplicitStraightEntry(cfg,(0.,0.,0.),10.)
        # Compare the same bumper geometry; production front retains its own.
        front.inner.bumper_offset=cfg['rear_overhang']
        rear=ReverseStraightEntry(cfg,origin,10.)
        bottom=[[(.77,-.19),(.83,.19)]]
        for stamp in (10.2,10.3):
            front.observe(bottom,stamp,(0.,0.,0.))
            rear.observe([[world(origin,(-x,y)) for x,y in line] for line in bottom],stamp,origin)
        self.assertAlmostEqual(front.remaining(),rear.remaining())
        travel=front.remaining()+.001
        # Both keep the measured bottom through the same camera-update pause.
        self.assertGreater(front.command(12.,(0.,0.,0.))[0],0)
        self.assertLess(rear.command(12.,origin)[0],0)
        rear_pose=tuple(world(origin,(-travel,0.)))+(origin[2],)
        self.assertGreater(front.command(12.1,(travel,0.,0.))[0],0)
        self.assertLess(rear.command(12.1,rear_pose)[0],0)
        front.observe(bottom,12.2,(travel,0.,0.))
        rear.observe([[world(origin,(-x,y)) for x,y in line] for line in bottom],12.2,rear_pose)
        self.assertEqual(front.command(12.2,(travel,0.,0.)),(0,0.))
        self.assertEqual(rear.command(12.2,rear_pose),(0,0.))
        self.assertEqual(rear.status,'FINISHED')


class RearParkingCameraTests(unittest.TestCase):
    def test_rear_image_white_pair_has_priority_over_blue(self):
        import cv2
        import numpy as np
        from robot.camera.vision import GroundDetector
        fixture=RightReverseParkingTests('test_rear_geometry_produces_negative_speed_after_gear_pause')
        cfg=fixture.cfg();detector=GroundDetector(cfg)
        frame=np.zeros((400,480,3),np.uint8)
        for y in (-.19+.06,.19+.06):
            u=int(240+400*y)
            cv2.line(frame,(u,65),(u,295),(255,255,255),6)
        cv2.line(frame,(55,183),(425,195),(255,0,0),8)
        data,blue,white=detector.detect(frame,rear=True,include_slots=False)
        lines=detector.parking_lines(white,min_length_m=.04,rear=True)
        self.assertTrue(data['blue_lines'])
        core=Controller(cfg);self.addCleanup(core.close)
        core.state=core.action='PARKING'
        core.parking_entry=ReverseStraightEntry(cfg,core.pose,10.)
        for stamp in (10.2,10.3):
            payload=dict(source='rear',part='parking_lines',frame='base_link',
                         stamp=stamp,lines=lines,blue_lines=data['blue_lines'],markers=[],slots=[])
            checked,accepted=ground(json.dumps(payload),stamp,1.25)
            core.observe_ground(checked,accepted)
        speed,steer=core.tick(10.5)
        self.assertLess(speed,0)
        self.assertNotEqual(steer,0)
        debug=core.parking_entry.debug
        self.assertTrue(debug['pair_seen'])
        self.assertFalse(debug['blue_used'])
        self.assertEqual(debug['blue_reject_reason'],'bay_edges_priority')
        self.assertEqual(debug['steering_reference'],'both_sides')
        self.assertAlmostEqual(debug['pair_width_m'],.38,delta=.03)

    def test_rear_lines_use_negative_longitudinal_and_correct_lateral_coordinates(self):
        import cv2
        import numpy as np
        from robot.camera.vision import GroundDetector
        detector = GroundDetector(copy.deepcopy(CONFIG))
        mask = np.zeros((400,480),np.uint8)
        cv2.line(mask,(164,50),(164,270),255,3)
        cv2.line(mask,(316,50),(316,270),255,3)
        lines = detector.parking_lines(mask,min_length_m=.04,rear=True)
        self.assertTrue(lines)
        self.assertTrue(all(x<0 for line in lines for x,y in line))
        self.assertTrue(any(y<-.17 for line in lines for x,y in line))
        self.assertTrue(any(y>.17 for line in lines for x,y in line))

    def test_rear_status_selects_rear_observations_once_and_drops_stale_frame(self):
        from test_image_scheduling import SchedulingTests
        fixture = SchedulingTests('test_blue_latest_is_published_without_any_parking_work')
        node = fixture.ground()
        node.cfg.update(parking_mode='forward_center',parking_entry_style='S')
        class Message(object):
            data = json.dumps(dict(state='PARKING',parking_entry=dict(camera_source='rear')))
        node.status(Message())
        self.assertTrue(node.rear_parking_requested)
        calls = []
        def lines(white,u_offset=0,min_length_m=.12,rear=False):
            calls.append((rear,min_length_m))
            return [[(-.3,-.19),(-.7,-.19)]]
        node.detector.parking_lines = lines
        node.rear(fixture.image(9.9));node.tick(None);node.tick(None)
        self.assertEqual(calls,[(True,.04)])
        self.assertEqual([(m['source'],m['part']) for m in node.pub.messages],
                         [('rear','parking_lines')])
        node.rear(fixture.image(8.));node.tick(None)
        self.assertEqual(len(calls),1)


if __name__ == '__main__':
    unittest.main()
