"""P-anchored line identity and disappearance handoff; no vehicle output."""
import unittest
from robot.parking.p_line_reference import PLineReference
from robot.parking.timed_core import TimedParking
from test_timed_parking import config,scene,raw,Follower


def sign(stamp, bounds=None, track_id=1):
    return dict(stamp=stamp,sign_bounds=bounds or [440,100,40,50],track_id=track_id)


class PReferenceTests(unittest.TestCase):
    def test_single_frame_misses_do_not_erase_pending_confirmation(self):
        target=PLineReference()
        lines=[[[1.,-.3],[1.,-.66]]]
        pixels=[[[400,210],[500,210]]]
        first=target.update(1.,lines,pixels,sign(1.))
        self.assertEqual(first['lock_frames'],1)
        missing=target.update(1.2,[],[],sign(1.2))
        self.assertEqual(missing['lock_frames'],1)
        target.update(1.4,lines,pixels,sign(1.4))
        target.update(1.6,[],[],sign(1.6))
        final=target.update(1.8,lines,pixels,sign(1.8))
        self.assertTrue(final['locked'])
        self.assertEqual(final['lock_frames'],3)

    def test_unlocked_p_can_move_to_a_new_detected_board_and_line(self):
        target=PLineReference()
        target.update(1.,[[[1.,-.3],[1.,-.66]]],[[[400,210],[500,210]]],sign(1.))
        latest=sign(1.2,[100,100,40,50],2)
        result=target.update(1.2,[[[1.7,-.3],[1.7,-.66]]],[[[60,210],[160,210]]],latest)
        self.assertEqual(result['p_track_id'],2)
        self.assertEqual(result['p_bounds'],latest['sign_bounds'])
        self.assertTrue(result['visible'])
        self.assertAlmostEqual(result['line'][0][0],1.7)

    def test_same_board_can_be_renumbered_before_lock_without_losing_votes(self):
        target=PLineReference()
        lines=[[[1.,-.3],[1.,-.66]]]
        pixels=[[[400,210],[500,210]]]
        target.update(1.,lines,pixels,sign(1.))
        second=target.update(1.2,lines,pixels,sign(1.2,track_id=2))
        self.assertEqual(second['lock_frames'],2)
        self.assertEqual(second['p_track_id'],2)
        self.assertTrue(target.update(1.4,lines,pixels,sign(1.4,track_id=2))['locked'])

    def test_p_nearest_line_beats_line_nearest_vehicle(self):
        target=PLineReference()
        lines=[[[1.7,-.3],[1.7,-.66]],[[1.,-.3],[1.,-.66]]]
        pixels=[[[400,210],[500,210]],[[400,290],[500,290]]]
        for stamp in (1.,1.2,1.4):
            result=target.update(stamp,lines,pixels,sign(stamp))
        self.assertTrue(result['locked'])
        self.assertAlmostEqual(result['line'][0][0],1.7)

    def locked(self):
        target=PLineReference()
        for stamp in (1.,1.2,1.4):
            result=target.update(stamp,[[[1.,-.3],[1.,-.66]]],
                                 [[[400,210],[500,210]]],sign(stamp))
        self.assertTrue(result['locked'])
        return target

    def test_same_target_loss_stops_even_with_other_white_lines_visible(self):
        target=self.locked()
        other=[[[1.7,-.3],[1.7,-.66]]]
        first=target.update(1.6,other,[[[400,210],[500,210]]],None)
        self.assertFalse(first['ended'])
        second=target.update(1.8,other,[[[400,210],[500,210]]],None)
        self.assertTrue(second['ended'])
        self.assertEqual(second['missing_frames'],2)

    def test_one_missing_frame_then_return_does_not_finish(self):
        target=self.locked()
        target.update(1.6,[],[],None)
        returned=target.update(1.8,[[[.95,-.3],[.95,-.66]]],
                               [[[405,215],[505,215]]],None)
        self.assertTrue(returned['visible'])
        self.assertFalse(returned['ended'])
        self.assertEqual(returned['missing_frames'],0)

    def test_confirmed_disappearance_stays_valid_when_line_reappears(self):
        from robot.parking.timed_core import validate_scene
        target=self.locked()
        target.update(1.6,[],[],None)
        self.assertTrue(target.update(1.8,[],[],None)['ended'])
        ref=target.update(2.,[[[.95,-.3],[.95,-.66]]],
                          [[[405,215],[505,215]]],sign(2.))
        data=scene(2.);data['p_reference']=ref
        validate_scene(data,config(),2.)
        self.assertTrue(ref['ended'])
        self.assertFalse(ref['visible'])
        self.assertEqual(ref['missing_frames'],2)

    def test_duplicate_frame_and_blank_before_acquisition_never_finish(self):
        target=self.locked()
        target.update(1.6,[],[],None)
        self.assertFalse(target.update(1.6,[],[],None)['ended'])
        target=PLineReference()
        for stamp in (1.,1.2,1.4):
            self.assertFalse(target.update(stamp,[],[],None)['ended'])

    def test_p_identity_cannot_change_after_line_lock(self):
        target=self.locked()
        result=target.update(1.6,[[[.95,-.3],[.95,-.66]]],
                             [[[405,215],[505,215]]],sign(1.6,[100,100,40,50],2))
        self.assertEqual(result['p_track_id'],1)
        self.assertTrue(result['visible'])

    def test_clipped_p_cannot_establish_initial_reference(self):
        target=PLineReference()
        for stamp in (1.,1.2,1.4):
            result=target.update(stamp,[[[1.,-.3],[1.,-.66]]],
                                 [[[600,210],[639,210]]],sign(stamp,[620,100,20,50]))
        self.assertFalse(result['locked'])


class HandoffTests(unittest.TestCase):
    def task(self):
        task=TimedParking(config(),0.,Follower(),select_by_lidar=True)
        self.addCleanup(task.close)
        return task

    def observe(self,task,stamp,locked=True,visible=True,ended=False,curved=False):
        from test_side_parking_integration import p_frame
        data=scene(stamp,count=2,curved=curved)
        data.update(stop_mode='target_white_disappearance')
        data['p_reference']=dict(locked=locked,visible=visible,ended=ended,
                                 reason='LINE_LEFT_VIEW' if ended else 'TRACKING_P_LINE')
        data['target_line']=dict(run=0.,id='before_p_white' if locked else None,
            valid=True,locked=locked,visible=visible if locked else None,
            locked_at=.1,reason='TEST_TARGET',line=[[1.,-.3],[1.,-.66]])
        task.observe_visual(data,stamp)
        task.observe_sign(p_frame(stamp,visible),stamp)

    def test_handoff_on_first_locked_line_disappearance_without_road_curve(self):
        task=self.task()
        self.observe(task,.1)
        self.assertEqual(raw(task,.1),(16,7))
        self.observe(task,.3,visible=False)
        self.assertEqual(raw(task,.3),(0,0))
        self.assertEqual(task.phase,'SELECT_SLOT')
        self.assertAlmostEqual(task.selection.started,.3)
        self.observe(task,.5,visible=False,ended=True)
        self.assertEqual(raw(task,.5),(0,0))
        self.assertAlmostEqual(task.selection.started,.3)
        self.observe(task,.7,visible=False,ended=True,curved=True)
        self.assertEqual(raw(task,.7),(0,0))
        self.assertEqual(task.phase,'SELECT_SLOT')

    def test_white_reference_is_not_required_while_p_remains_visible(self):
        task=self.task()
        for stamp in (.1,.2,.4):
            task.observe_visual(scene(stamp,0,True),stamp)
            self.assertEqual(raw(task,stamp),(16,7))
        self.assertIsNone(task.selection)

    def test_visible_near_line_never_stops_from_a_distance_estimate(self):
        task=self.task()
        self.observe(task,.1)
        self.assertGreater(raw(task,.1)[0],0)
        self.observe(task,.5)
        self.assertGreater(raw(task,.5)[0],0)
        self.assertEqual(task.phase,'APPROACH')

    def stopped(self):
        task=self.task()
        self.observe(task,.1)
        self.assertGreater(raw(task,.1)[0],0)
        self.observe(task,.5,visible=False,ended=True,curved=True)
        self.assertEqual(raw(task,.5),(0,0))
        return task

    def test_stopped_lidar_selection_does_not_need_front_lane_frames(self):
        from test_side_parking_integration import observation
        task=self.stopped()
        for stamp in (2.,2.1,2.2):
            task.observe_selection(observation(task,stamp,0,10),stamp)
            command=raw(task,stamp)
        self.assertEqual(command,(30,0))
        self.assertEqual(task.slot,'P2')
        self.assertEqual(task.phase,'REVERSE')

    def test_p3_runs_entire_saved_sequence_without_front_line_updates(self):
        from test_side_parking_integration import observation
        task=self.stopped()
        for stamp in (.9,1.,1.1):
            task.observe_selection(observation(task,stamp,0,0),stamp)
            raw(task,stamp)
        now=1.1
        for index,stage in enumerate(task.cfg['parking_timed']['sequences']['P3']):
            self.assertEqual(raw(task,now),(stage['speed'],stage['steering']))
            self.assertEqual(task.index,index)
            self.assertEqual(raw(task,task.deadline-.001),(stage['speed'],stage['steering']))
            now=task.deadline+.001
        self.assertEqual(raw(task,now),(0,0))
        self.assertEqual(task.reason,'P3_COMPLETE')

    def test_front_camera_still_required_during_approach(self):
        task=self.task()
        self.observe(task,.1)
        self.assertGreater(raw(task,.1)[0],0)
        self.assertEqual(raw(task,2.),(0,0))
        self.assertEqual(task.reason,'PARKING_CAMERA_TIMEOUT')


if __name__=='__main__':unittest.main()
