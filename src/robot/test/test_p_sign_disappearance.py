"""Fresh P visibility and actual sign callback; no actuator publishers."""
import unittest,json
from test_timed_parking import config,scene,raw,Follower
from test_side_parking_integration import p_frame,observation
from robot.parking.timed_core import TimedParking,parking_sign_visible


class PSignDisappearanceTests(unittest.TestCase):
    def task(self,now=0.):
        task=TimedParking(config(),now,Follower(),select_by_lidar=True)
        self.addCleanup(task.close)
        return task

    def test_first_p_free_frame_stops_without_white_or_lane_metadata(self):
        task=self.task();event=task.observe_sign(p_frame(.1,False),.1)
        self.assertEqual(event['reason'],'P_DISAPPEARED')
        self.assertEqual(raw(task,.1),(0,0));self.assertEqual(task.phase,'SELECT_SLOT')
        self.assertAlmostEqual(task.selection.started,.1);self.assertFalse(task.finished)

    def test_all_p_candidates_count_even_with_low_confidence_small_size_or_route_rejection(self):
        for label in ('P','park','PARKING'):
            data=p_frame(.1);data.update(label='',confidence=0.,route_detections=[])
            data['detections'][0].update(label=label,confidence=.35,bounds=[620,100,19,15])
            data['detections'].insert(0,dict(label='right',confidence=.99,bounds=[100,50,40,60]))
            self.assertTrue(parking_sign_visible(data))

    def test_other_sign_without_p_is_a_p_free_image(self):
        data=p_frame(.1,False)
        data['detections']=[dict(label='right',confidence=.99,bounds=[100,50,40,60])]
        task=self.task();task.observe_sign(data,.1)
        self.assertEqual(raw(task,.1),(0,0));self.assertEqual(task.phase,'SELECT_SLOT')

    def test_duplicate_old_future_and_stale_frames_never_start_selection(self):
        task=self.task();task.observe_lane(scene(.1)['lane_observation'],.1)
        task.observe_sign(p_frame(.1),.1)
        for stamp,now in ((.1,.1),(-.1,.1),(.2,.1),(.2,10.)):
            self.assertIsNone(task.observe_sign(p_frame(stamp,False),now))
            self.assertIsNone(task.selection)
        self.assertEqual(raw(task,.1),(16,7))

    def test_bad_or_rear_metadata_does_not_mean_p_disappeared(self):
        for kind in ('missing','bad_list','bad_row','rear','startup','bad_bounds'):
            task=self.task();data=p_frame(.1,False)
            if kind=='missing':del data['detections']
            elif kind=='bad_list':data['detections']=None
            elif kind=='bad_row':data['detections']=[dict(label='park')]
            elif kind=='rear':data['camera_source']='rear'
            elif kind=='startup':data['startup_only']=True
            else:data=p_frame(.1);data['detections'][0]['bounds']=[0,0,-1,4]
            with self.assertRaises(ValueError):task.observe_sign(data,.1)
            self.assertEqual(task.phase,'APPROACH');self.assertIsNone(task.selection)

    def test_missing_sign_messages_timeout_without_selecting_p3(self):
        task=self.task();t=task.cfg['sign_timeout']+.1
        task.observe_lane(scene(t)['lane_observation'],t)
        self.assertEqual(raw(task,t),(0,0));self.assertEqual(task.reason,'P_SIGN_TIMEOUT')
        self.assertIsNone(task.selection)

    def test_visible_p_ignores_old_white_seek_deadline(self):
        task=self.task()
        for t in range(20):
            now=t+.1;task.observe_sign(p_frame(now),now)
            task.observe_lane(scene(now,0,True)['lane_observation'],now)
            self.assertEqual(raw(task,now),(16,7))
        self.assertIsNone(task.selection)

    def test_real_ros_callback_only_feeds_the_active_side_task(self):
        import test_sign_callback_scheduling as callbacks
        for state in ('TIMED_PARKING','MANEUVER'):
            harness=callbacks.SignCallbackSchedulingTests('test_ignored_parking_does_no_projection');harness.setUp()
            task=self.task(9.8);task.observe_lane(scene(9.9)['lane_observation'],9.9)
            harness.core.state=state;harness.core.timed_parking=task
            harness.node.parking_projector=None
            harness.node.sign(callbacks.Box(data=json.dumps(p_frame(10.,False))))
            self.assertEqual(raw(task,10.),(0,0) if state=='TIMED_PARKING' else (16,7))
            self.assertEqual(task.phase,'SELECT_SLOT' if state=='TIMED_PARKING' else 'APPROACH')

    def test_selection_and_saved_motion_need_no_new_sign_messages(self):
        task=self.task();task.observe_sign(p_frame(.1,False),.1);raw(task,.1)
        for t in (3.,3.1,3.2):task.observe_selection(observation(task,t,0,10),t)
        self.assertEqual(raw(task,3.2),(30,0));self.assertEqual(task.slot,'P2')

    def test_automatic_white_producer_starts_idle_without_publishers_or_worker(self):
        from robot.parking import timed_vision_node as module
        import test_sign_callback_scheduling as callbacks
        cfg=config();cfg.update(parking_mode='forward_center',parking_slot='')
        old_ros=module.rospy;self.addCleanup(setattr,module,'rospy',old_ros)
        module.rospy=callbacks.Box(get_param=lambda name:cfg,loginfo=lambda *a:None)
        node=module.Node()
        self.assertFalse(hasattr(node,'worker'));self.assertFalse(hasattr(node,'output'))


if __name__=='__main__':unittest.main()
