import os
import unittest
from robot.common.config import load_config
from robot.common.contracts import encode_command
from robot.master.controller import Controller


class TopmostLaneTests(unittest.TestCase):
    def setUp(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        cfg.update(wait_green=False,lidar_enabled=False,steering_command_scale_rad=.03,
                   lane_lateral_full_scale_m=.30)
        self.c=Controller(cfg)
        self.stamp=0.
        self.addCleanup(self.c.close)

    def command(self,points,confidence=.9):
        self.stamp+=1.
        self.c.observe_lane(points,confidence,self.stamp)
        speed,steer=self.c.lane_command(self.stamp)
        return encode_command(speed,steer,self.c.cfg,0)['steering_raw']

    def test_topmost_selection_is_preserved_during_full_right_lock(self):
        for path in ([ (.5,.5),(1.,-.15),(.8,.4)],[(1.,-.15),(.5,-.5)]):
            # The first path identifies a right bend: hold full right while
            # retaining the actual farthest point in the control diagnostics.
            self.assertEqual(self.command(path),-22)
            self.assertEqual(self.c.lane_target['target'],(1.,-.15))

    def test_same_offset_same_command_at_any_forward_distance(self):
        for forward in (.5,1.,1.2):
            self.assertEqual(self.command([(forward,.15)]),11)

    def test_zero_offset_mirrored_offsets_and_saturation(self):
        for lateral,raw in ((0.,0),(.15,11),(-.15,-11),(.6,22),(-.6,-22)):
            self.assertEqual(self.command([(1.,lateral)]),raw)

    def test_preview_distance_and_path_confidence_do_not_choose_target(self):
        for lookahead in (.1,1.,2.):
            self.c.cfg['lookahead']=lookahead
            self.assertEqual(self.command([(.5,.5),(1.,-.15)],.1),-11)

    def test_previous_right_exit_reference_cannot_override_target_path(self):
        self.c.follow_left_boundary=True
        self.c.right_tail_origin=(0.,0.,0.)
        self.assertEqual(self.command([(.5,.5),(1.,-.15)]),-11)
        self.assertEqual(self.c.lane_source,'center')

    def test_default_gain_reaches_full_lock_at_seven_point_five_centimetres(self):
        cfg=load_config(os.path.join(os.path.dirname(__file__),'../config'))
        self.c.cfg['lane_lateral_full_scale_m']=cfg['lane_lateral_full_scale_m']
        for lateral,expected in ((.0375,11),(-.0375,-11),(.075,22),(-.075,-22),(.30,22)):
            self.assertEqual(self.command([(1.,lateral)]),expected)


if __name__=='__main__':unittest.main()
