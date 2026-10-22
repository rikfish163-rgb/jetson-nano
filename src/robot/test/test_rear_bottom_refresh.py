"""Stop-position images plus model drift; no vehicle commands are published."""
import json
import os
import unittest
from test_core import PACKAGE_DIR
from robot.common.config import load_config
from robot.common.geometry import world
from robot.parking.reverse_entry import ReverseStraightEntry


class RearBottomRefreshTests(unittest.TestCase):
    def entry(self, anchor=(-.9,0.)):
        cfg=load_config(os.path.join(PACKAGE_DIR,'config'))
        cfg.update(parking_p_reverse_s=0.,parking_rear_p_reverse_s=0.,
                   steering_command_scale_rad=.03,pose_mode='command_model')
        e=ReverseStraightEntry(cfg,(0.,0.,0.),10.,wait_for_sign=True)
        e.observe_sign('PARKING',.99,10.1,10.1,anchor=anchor)
        return e

    def fixture(self):
        with open(os.path.join(os.path.dirname(__file__),'fixtures','rear_bottom_stop.json')) as f:
            return json.load(f)

    def rails(self, depth=.8125, center=0.):
        return [[(-.3,y),(-depth,y)] for y in (center-.19,center+.19)]

    def test_current_image_refreshes_bottom_despite_model_translation_and_yaw(self):
        data=self.fixture();e=self.entry(tuple(data['anchor']))
        e.observe(data['frames'][0],10.2,(0.,0.,0.))
        self.assertIsNotNone(e.bottom)
        pose=(-.4,0.,.4)
        e.observe([[world(pose,p) for p in side] for side in data['frames'][1]],10.6,pose)
        self.assertEqual(e.bottom_stamp,10.6)
        self.assertLess(e.command(10.7,pose)[0],0)
        self.assertTrue(.40<e.remaining()<.50)
        self.assertEqual(e.debug['distance_source'],'live_bottom_reference')

    def test_recorded_pair_recovers_after_old_prediction_has_passed_the_bottom(self):
        data=self.fixture();e=self.entry(tuple(data['anchor']))
        e.observe(data['frames'][0],10.2,(0.,0.,0.))
        pose=(-.48,0.,.4)
        e.observe([],12.,pose)
        self.assertLess(e.command(12.1,pose)[0],0)
        for i in (1,2):
            stamp=12.1+i*.2
            e.observe([[world(pose,p) for p in side] for side in data['frames'][i]],stamp,pose)
            self.assertLess(e.command(stamp+.05,pose)[0],0)
        self.assertIsNotNone(e.view)
        self.assertEqual(e.bottom_stamp,12.5)
        self.assertTrue(.40<e.remaining()<.50)

    def test_logged_far_bottom_loss_cannot_finish_on_estimated_distance(self):
        e=self.entry()
        e.observe(self.rails()+[[(-.8125,-.19),(-.8125,.19)]],10.2,(0.,0.,0.))
        self.assertAlmostEqual(e.remaining(),.7575)
        pose=(-.7693943175255192,0.,0.)
        e.observe([],15.1,pose)
        self.assertLess(e.command(15.2,pose)[0],0)
        self.assertNotEqual(e.status,'FINISHED')
        self.assertFalse(e.debug['bottom_arrival_confirmed'])

    def test_far_last_measurement_cannot_finish_between_camera_frames(self):
        e=self.entry()
        e.observe(self.rails()+[[(-.8125,-.19),(-.8125,.19)]],10.2,(0.,0.,0.))
        self.assertLess(e.command(10.6,(-.77,0.,0.))[0],0)
        self.assertNotEqual(e.status,'FINISHED')

    def test_continuous_recorded_images_keep_refreshing_with_accumulating_pose_error(self):
        data=self.fixture();e=self.entry(tuple(data['anchor']))
        for i in range(12):
            pose=(-i*.10,0.,i*.08)
            stamp=10.4+i*.2
            lines=data['frames'][i%3]
            e.observe([[world(pose,p) for p in side] for side in lines],stamp,pose)
            self.assertEqual(e.bottom_stamp,stamp)
            self.assertLess(e.command(stamp+.05,pose)[0],0)
            self.assertTrue(.40<e.remaining()<.50)

    def test_reacquisition_requires_two_distinct_consecutive_pair_frames(self):
        data=self.fixture();e=self.entry(tuple(data['anchor']))
        e.observe(data['frames'][0],10.2,(0.,0.,0.))
        pose=(-.48,0.,.4)
        lines=[[world(pose,p) for p in side] for side in data['frames'][1]]
        e.observe(lines,12.3,pose)
        e.observe(lines,12.3,pose)
        self.assertEqual(e.bottom_stamp,10.2)
        e.observe([],12.5,pose)
        e.observe(lines,12.7,pose)
        self.assertEqual(e.bottom_stamp,10.2)
        e.observe(lines,12.9,pose)
        self.assertEqual(e.bottom_stamp,12.9)

    def test_neighbor_pair_cannot_reinitialize_the_target_bottom(self):
        e=self.entry()
        e.observe(self.rails()+[[(-.8125,-.19),(-.8125,.19)]],10.2,(0.,0.,0.))
        for stamp in (12.,12.2,12.4):
            e.observe(self.rails(.7,.38)+[[(-.7,.19),(-.7,.57)]],stamp,(0.,0.,0.))
        self.assertEqual(e.bottom_stamp,10.2)

    def test_near_reference_loss_retains_the_last_twenty_cm_stop(self):
        e=self.entry()
        e.observe(self.rails()+[[(-.8125,-.19),(-.8125,.19)]],10.2,(0.,0.,0.))
        # Correctly observed approach to the configured 25.5 cm axle depth.
        for i in range(1,8):
            pose=(-i*.08,0.,0.)
            e.observe(self.rails()+[[(-.8125,-.19),(-.8125,.19)]],10.2+i*.2,pose)
        self.assertLessEqual(e.remaining(),.20)
        pose=(-.56,0.,0.)
        e.observe([],12.,pose)
        remaining=e.remaining()
        self.assertLess(e.command(12.1,(pose[0]-remaining+.001,0.,0.))[0],0)
        self.assertEqual(e.command(12.2,(pose[0]-remaining,0.,0.)),(0,0.))


if __name__=='__main__':unittest.main()
