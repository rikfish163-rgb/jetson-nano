"""Replay lane P, stopped selection and saved actions without chassis output."""
import copy
import unittest
from test_timed_parking import config,scene as base_scene,raw,Follower
from robot.master.controller import Controller
from robot.parking.timed_core import TimedParking
from robot.parking.side_selection import SideSelection
from robot.common.contracts import validate_config


def scene(t,count=2,curved=False):
    data = base_scene(t,count,curved)
    data['target_line'] = dict(run=0.,id='before_p_white',valid=True,locked=True,
                              visible=count>0,locked_at=.1,reason='TEST_TARGET',
                              line=[[1.,-.6],[1.,-.3]])
    return data


def p_frame(t,visible=True):
    return dict(stamp=t,camera_source='front',label='PARKING' if visible else '',
                confidence=.99 if visible else 0.,detections=[dict(label='park',
                confidence=.99,bounds=[550,100,36,45])] if visible else [])


def observation(task,stamp,a=0,b=0,valid=True):
    return dict(source='side_parking_lidar',frame=task.cfg['scan_frame'],stamp=stamp,
                request_stamp=task.selection.started,valid=valid,valid_rays=1440,
                line=dict(count=20,length=.3,rms=.005,midpoint=[.4,-.3]),
                counts={'1':a,'2':b})


class SideParkingTests(unittest.TestCase):
    def new_task(self):
        cfg=config();cfg.update(parking_mode='forward_center',parking_slot='')
        task=TimedParking(cfg,0.,Follower(),select_by_lidar=True)
        self.addCleanup(task.close)
        return task

    def task(self):
        task=self.new_task()
        for t in (.1,.2):
            task.observe_visual(scene(t),t)
            self.assertEqual(raw(task,t),(16,7))
        task.observe_visual(scene(.3,0,False),.3)
        task.observe_sign(p_frame(.3,False),.3)
        self.assertEqual(raw(task,.3),(0,0))
        self.assertEqual(task.phase,'SELECT_SLOT')
        self.assertAlmostEqual(task.selection.started,.3)
        return task

    def test_p_disappearance_stops_even_when_other_white_lines_remain(self):
        task=self.new_task()
        task.observe_visual(scene(.1,3),.1)
        self.assertEqual(raw(task,.1),(16,7))
        data=scene(.2,2,False);data['target_line']['visible']=False
        task.observe_visual(data,.2)
        self.assertEqual(raw(task,.2),(16,7))
        task.observe_sign(p_frame(.2,False),.2)
        self.assertEqual(raw(task,.2),(0,0))
        self.assertEqual(task.phase,'SELECT_SLOT')
        self.assertFalse(task.status(.2)['p_sign']['visible'])
        self.assertAlmostEqual(task.selection.started,.2)

    def test_target_visible_with_two_curves_does_not_stop(self):
        task=self.new_task()
        for t in (.1,.2,.3):
            task.observe_visual(scene(t,1,True),t)
            self.assertEqual(raw(task,t),(16,7))
        self.assertEqual(task.phase,'APPROACH')
        self.assertEqual(task.reason,'AUTO_LANE_FORWARD_WAIT_P_DISAPPEARANCE')

    def test_missing_white_target_does_not_stop_until_p_disappears(self):
        task=self.new_task()
        task.observe_visual(scene(.1),.1);raw(task,.1)
        data=scene(.2,3,False);data['target_line']['visible']=False
        task.observe_visual(data,.2)
        self.assertEqual(raw(task,.2),(16,7))
        task.observe_visual(scene(.3,1,True),.3)
        self.assertEqual(raw(task,.3),(16,7))
        data=scene(.4,3,True);data['target_line']['visible']=False
        task.observe_visual(data,.4)
        task.observe_sign(p_frame(.4,False),.4)
        self.assertEqual(raw(task,.4),(0,0))
        self.assertEqual(task.phase,'SELECT_SLOT')
        self.assertEqual(task.selection.started,.4)

    def test_unknown_unlocked_old_or_absent_target_never_starts_lidar(self):
        for kind in ('no_metadata','unlocked','pose_missing','old_run'):
            task=self.new_task()
            for t in (.1,.2,.3):
                data=scene(t,0,True)
                target=data['target_line']
                if kind=='no_metadata':del data['target_line']
                elif kind=='unlocked':target.update(locked=False,id=None,visible=None)
                elif kind=='pose_missing':target.update(valid=False,visible=None,line=None)
                else:target.update(run=-1.,locked_at=-.9)
                task.observe_visual(data,t)
                self.assertEqual(raw(task,t),(16,7))
                self.assertEqual(task.phase,'APPROACH')
                self.assertIsNone(task.selection)

    def test_camera_loss_stops_without_starting_selection(self):
        task=self.new_task()
        task.observe_visual(scene(.1),.1);raw(task,.1)
        self.assertEqual(raw(task,2.),(0,0))
        self.assertTrue(task.finished)
        self.assertIsNone(task.selection)

    def test_sign_message_gap_stops_without_selecting_a_bay(self):
        task=self.new_task()
        for t in range(17):
            data=base_scene(t+.1,0,True)
            task.observe_visual(data,t+.1)
            raw(task,t+.1)
        self.assertTrue(task.finished)
        self.assertEqual(task.reason,'P_SIGN_TIMEOUT')
        self.assertIsNone(task.selection)

    def test_white_lines_reappear_after_first_stop_without_vetoing_lidar(self):
        task=self.task()
        for t in (.45,.81,1.,1.5,2.,2.5):
            task.observe_visual(scene(t,4,True),t)
            self.assertEqual(raw(task,t),(0,0))
            self.assertEqual(task.phase,'SELECT_SLOT')
            self.assertFalse(task.finished)
        for t in (2.6,2.7,2.8):
            task.observe_visual(scene(t,4,True),t)
            task.observe_selection(observation(task,t,0,10),t)
        self.assertEqual(raw(task,2.8),(30,0))
        self.assertEqual(task.phase,'REVERSE')
        self.assertEqual(task.slot,'P2')

    def test_lidar_starts_immediately_but_saved_motor_pause_is_preserved(self):
        task=self.task()
        task.selection.options['confirm_frames']=1
        task.observe_visual(scene(.4,4,True),.4)
        task.observe_selection(observation(task,.4,10,0),.4)
        self.assertEqual(task.selection.slot,'P1')
        self.assertEqual(raw(task,.4),(0,0))
        self.assertEqual(task.phase,'SELECT_SLOT')
        self.assertEqual(raw(task,.79),(0,0))
        task.observe_visual(scene(.81,4,True),.81)
        self.assertEqual(raw(task,.81),(30,0))
        self.assertEqual(task.index,0)

    def test_all_three_choices_reuse_entire_saved_tables(self):
        for a,b,slot in ((10,0,'P1'),(0,10,'P2'),(9,9,'P3')):
            task=self.task()
            for t in (.9,1.,1.1):
                task.observe_visual(scene(t,0,True),t)
                task.observe_selection(observation(task,t,a,b),t)
                if t<1.1:self.assertEqual(raw(task,t),(0,0))
            now=1.1
            for i,stage in enumerate(task.cfg['parking_timed']['sequences'][slot]):
                task.observe_visual(scene(now,0,True),now)
                self.assertEqual(raw(task,now),(stage['speed'],stage['steering']))
                self.assertEqual(task.index,i)
                self.assertAlmostEqual(task.deadline-now,stage['seconds'])
                # New occupancy cannot retarget the frozen action.
                task.observe_selection(observation(task,now+.01,50,50),now+.01)
                end=task.deadline
                while now+.5<end:
                    now+=.5;task.observe_visual(scene(now,0,True),now)
                    self.assertEqual(raw(task,now),(stage['speed'],stage['steering']))
                now=end+.001
            task.observe_visual(scene(now,0,True),now)
            self.assertEqual(raw(task,now),(0,0))
            self.assertEqual(task.slot,slot)
            self.assertEqual(task.reason,slot+'_COMPLETE')

    def test_conflicts_invalid_duplicates_and_old_scans_never_choose_p3(self):
        task=self.task()
        task.observe_selection(observation(task,.2),.9)
        self.assertIsNone(task.selection.slot)
        for t in (.9,1.,1.1):
            task.observe_selection(observation(task,t,10,10),t)
            task.observe_visual(scene(t,0,True),t)
            self.assertEqual(raw(task,t),(0,0))
        self.assertEqual(task.reason,'BOTH_OCCUPIED_STOPPED')
        task.observe_selection(observation(task,1.2,valid=False),1.2)
        self.assertIsNone(task.selection.slot)
        for _ in range(5):task.observe_selection(observation(task,1.3),1.3)
        self.assertEqual(task.selection.votes,1)
        bad=observation(task,1.4);bad['request_stamp']=0.
        task.observe_selection(bad,1.4)
        self.assertEqual(task.selection.votes,0)
        task.observe_selection(observation(task,1.5),4.)
        self.assertEqual(task.selection.votes,0)
        self.assertIsNone(task.selection.slot)

    def test_timeout_latches_stop_even_if_data_arrives_later(self):
        task=self.task()
        t=task.selection.started+task.selection.options['selection_timeout_s']+.01
        task.observe_visual(scene(t,0,True),t)
        self.assertEqual(raw(task,t),(0,0))
        self.assertTrue(task.finished)
        self.assertEqual(task.reason,'SIDE_SELECTION_TIMEOUT_STOPPED')
        task.observe_selection(observation(task,t),t)
        self.assertEqual(raw(task,t+1.),(0,0))

    def controller(self):
        cfg=config();cfg.update(parking_mode='forward_center',parking_entry_style='S',
                                parking_slot='',parking_side_lidar_enabled=True,parking_sign_votes=3)
        c=Controller(cfg);self.addCleanup(c.close)
        return c

    def vote(self,c):
        for t in (3.,3.1,3.2):c.observe_sign('PARKING',.98,t,t)

    def test_lane_p_preempts_reverse_after_turn_and_needs_no_blue(self):
        for last in (None,'LEFT','RIGHT','STRAIGHT'):
            c=self.controller();c.state='LANE';c.last_completed_action=last
            c.parking_route_ready=False
            self.vote(c)
            if last == 'LEFT':
                self.assertIsNone(c.pending)
                self.assertFalse(c.parking_side_pending)
                self.assertEqual(c.sign_info['decision'],'front_p_ignored_left_reverse_route')
                continue
            self.assertEqual(c.pending,'PARKING')
            self.assertTrue(c.parking_side_pending)
            c.execute('mission','dispatch',3.3)
            self.assertEqual(c.state,'TIMED_PARKING')
            self.assertEqual(c.action_source,'lane_p')
            self.assertTrue(c.timed_parking.select_by_lidar)
            self.assertEqual(c.timed_parking.slot,'AUTO')
            self.assertIsNone(c.parking_entry)
            self.assertEqual(c.tick(3.4),(0,0.))

    def test_gap_and_queued_action_p_keep_original_route(self):
        for state in ('GAP','WAIT_OBSTACLE'):
            c=self.controller();c.state=state;c.parking_route_ready=True
            self.vote(c)
            self.assertEqual(c.pending,'PARKING')
            self.assertFalse(c.parking_side_pending)
        for action in ('LEFT','RIGHT','STRAIGHT','UTURN'):
            c=self.controller();c.state='MANEUVER';c.action=action
            c.action_started=1.;c.route_action_started=1.;c.parking_route_ready=True
            if action == 'UTURN':c.uturn=dict(next_sign_after=2.)
            self.vote(c)
            if action == 'LEFT':
                self.assertIsNone(c.next_direction)
                self.assertEqual(c.sign_info['decision'],'front_p_ignored_left_reverse_route')
                continue
            self.assertEqual(c.next_direction,'PARKING')
            self.assertFalse(c.parking_side_pending)
            c.execute('mission','resume_lane')
            self.assertEqual(c.pending,'PARKING')
            self.assertFalse(c.parking_side_pending)

    def test_gap_votes_cannot_finish_a_lane_only_vote_sequence(self):
        c=self.controller();c.state='GAP';c.parking_route_ready=True
        c.observe_sign('PARKING',.98,3.,3.)
        c.observe_sign('PARKING',.98,3.1,3.1)
        c.state='LANE'
        c.observe_sign('PARKING',.98,3.2,3.2)
        self.assertIsNone(c.pending)
        self.assertEqual(c.sign_count,1)

    def test_startup_never_arms_side_parking_and_invalid_config_rejected(self):
        for state in ('WAIT_GREEN','STARTUP_STRAIGHT'):
            c=self.controller();c.state=state;self.vote(c)
            self.assertIsNone(c.pending)
            self.assertFalse(c.parking_side_pending)
        cfg=self.controller().cfg
        for key,value in (('min_points',0),('confirm_frames',0),('scan_timeout_s',-1)):
            broken=copy.deepcopy(cfg);broken['parking_side_lidar'][key]=value
            with self.assertRaises(ValueError):validate_config(broken)

    def test_controller_marks_selected_slot_complete_instead_of_global_slot(self):
        for slot in ('P1','P2','P3'):
            c=self.controller();c.state='LANE';self.vote(c)
            c.dispatch(3.3)
            c.timed_parking.slot=slot
            c.timed_parking.abort(slot+'_COMPLETE')
            self.assertEqual(c.tick(3.4),(0,0.))
            self.assertEqual(c.state,'FINISHED')
            self.assertTrue(c.parking_completed)


if __name__=='__main__':unittest.main()
