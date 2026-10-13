"""Road curve diagnostics and recorded bends cannot replace white disappearance."""
import json
import os
import unittest
import numpy as np
from test_timed_parking import config,scene as base_scene,raw,Follower
from robot.parking.timed_core import TimedParking,validate_scene
from robot.parking.timed_vision import road_curve_evidence


def observation(t,curved=False,request=0.):
    data=base_scene(t,7,False)
    data.update(stop_mode='road_curve',request_stamp=request,road_curved=curved,
                road_curve_sides=['LEFT'] if curved else [])
    return data


class CurveParkingTests(unittest.TestCase):
    def task(self,follower=None,now=0.):
        cfg=config();cfg.update(parking_mode='forward_center',parking_slot='')
        task=TimedParking(cfg,now,follower or Follower(),select_by_lidar=True)
        self.addCleanup(task.close)
        return task

    def boundary(self,curved=False,right=False):
        return [[float(x),(-.32 if right else .32)+(.4*(x-.9)**2 if curved else 0.)]
                for x in np.linspace(.45,1.5,12)]

    def test_either_reliable_boundary_is_enough_but_straight_missing_noise_are_not(self):
        options=config()['parking_timed']
        for boundaries in (dict(LEFT=self.boundary(True)),
                           dict(RIGHT=self.boundary(True,True)),
                           dict(LEFT=self.boundary(True),RIGHT=self.boundary(False,True))):
            evidence=road_curve_evidence(dict(boundaries=boundaries),options)
            self.assertTrue(evidence['road_curved'])
            self.assertFalse(evidence['both_curved'])
        for boundaries in ({},dict(LEFT=self.boundary()),
                           dict(RIGHT=self.boundary(True,True)[:3]),
                           dict(LEFT=[[.5,.3],[.6,.8],[.7,.1],[.8,.7],[.9,.2]])):
            self.assertFalse(road_curve_evidence(dict(boundaries=boundaries),options)['road_curved'])

    def test_bad_curve_boolean_and_future_request_are_rejected(self):
        data=observation(.1);data['road_curved']=1
        with self.assertRaises(ValueError):validate_scene(data,config(),.1)
        data=observation(.1,True,1.)
        with self.assertRaises(ValueError):validate_scene(data,config(),.1)

    def test_twenty_captured_bend_frames_do_not_stop_a_visible_white_target(self):
        path=os.path.join(os.path.dirname(__file__),'fixtures','parking_curve_stop_position.json')
        rows=json.load(open(path));self.assertEqual(len(rows),20)
        options=config()['parking_timed']
        for row in rows:
            lane=row['lane_observation'];evidence=road_curve_evidence(lane,options)
            self.assertTrue(evidence['road_curved'])
            stamp=lane['stamp']
            task=self.task(now=stamp-.1)
            from test_side_parking_integration import p_frame
            task.observe_sign(p_frame(stamp),stamp)
            data=dict(evidence,stamp=stamp,source='timed_parking_front',frame='base_link',
                      lines=0,lines_evaluated=False,stop_mode='road_curve',request_stamp=0.,
                      lane_observation=lane)
            data['target_line']=dict(run=0.,id='before_p_white',valid=True,locked=True,
                                    visible=True,locked_at=.1,reason='TARGET_VISIBLE',
                                    line=[[1.,-.6],[1.,-.3]])
            task.observe_visual(data,stamp)
            self.assertEqual(raw(task,stamp),(16,7))
            self.assertEqual(task.phase,'APPROACH')
            self.assertFalse(task.finished)


if __name__=='__main__':unittest.main()
