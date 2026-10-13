"""Regression: old white target metadata cannot trigger automatic parking."""
import unittest
from test_timed_parking import config,scene,raw,Follower
from test_side_parking_integration import p_frame
from robot.parking.timed_core import TimedParking

class OldWhiteTriggerTests(unittest.TestCase):
    def test_blank_white_frame_and_bends_do_not_start_lidar(self):
        task=TimedParking(config(),0.,Follower(),select_by_lidar=True)
        self.addCleanup(task.close)
        for t in (.1,.2,.3):
            data=scene(t,0,True)
            data['target_line']=dict(run=0.,id='before_p_white',valid=True,locked=True,
                visible=False,locked_at=.1,line=[[1.,-.6],[1.,-.3]],reason='TARGET_MISSING')
            task.observe_visual(data,t)
            self.assertEqual(raw(task,t),(16,7));self.assertIsNone(task.selection)
        task.observe_sign(p_frame(.4,False),.4)
        self.assertEqual(raw(task,.4),(0,0));self.assertEqual(task.phase,'SELECT_SLOT')

if __name__=='__main__':unittest.main()
