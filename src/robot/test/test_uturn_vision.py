import math
import os
import unittest
import cv2
import numpy as np
import yaml
from robot.uturn.vision import GroundMotion
from robot.uturn.vision import corridor_regions
from robot.uturn.vision import rigid_motion
from robot.uturn.vision import VisionUturnScene
from robot.uturn.relative import polygon_contains
from robot.uturn.relative import RelativeUturn
from robot.uturn.relative import decode_scene
from robot.common.geometry import bicycle
from robot.common.geometry import wrap
from robot.common.contracts import command_to_model_steering


class VisionTests(unittest.TestCase):
    def test_explicit_reference_survives_unknown_but_does_not_invent_pose(self):
        cfg=yaml.safe_load(open(os.path.join(os.path.dirname(__file__),'../config/competition.yaml')))
        cfg['uturn_followed_boundary']='LEFT'
        scene=VisionUturnScene(cfg)
        scene.observe_lane(None,1.)
        self.assertEqual(scene.side,'LEFT')
        scene.set_active(True)
        blank=np.zeros((400,480),np.uint8)
        self.assertIsNone(scene.update(cv2.cvtColor(blank,cv2.COLOR_GRAY2BGR),blank,blank,2.))
        self.assertEqual(scene.reason,'vision_wait_straight_boundary_alignment')

    def image(self):
        image = np.zeros((400, 480), np.uint8)
        rng = np.random.RandomState(4)
        for u, v in zip(rng.randint(40, 440, 65), rng.randint(40, 360, 65)):
            cv2.rectangle(image, (int(u), int(v)), (int(u)+6, int(v)+6), 255, -1)
        return image

    def test_camera_translation_is_inverse_ground_motion(self):
        tracker = GroundMotion(dict(pixels_per_m=400., origin_u=240., origin_v=600.))
        first = self.image()
        tracker.update(first, first, 1.)
        # Ground moves 8 px down, 4 px right: car advances .02 m and .01 m left.
        second = cv2.warpAffine(first, np.float32([[1, 0, 4], [0, 1, 8]]), (480, 400))
        pose = tracker.update(second, second, 1.1)
        self.assertAlmostEqual(pose[0], .02, delta=.003)
        self.assertAlmostEqual(pose[1], .01, delta=.003)
        self.assertAlmostEqual(pose[2], 0., delta=.004)

    def test_no_features_or_gap_cannot_claim_fresh_pose(self):
        tracker = GroundMotion(dict(pixels_per_m=400., origin_u=240., origin_v=600.))
        blank = np.zeros((400, 480), np.uint8)
        with self.assertRaises(ValueError): tracker.update(blank, blank, 1.)
        first = self.image()
        tracker.update(first, first, 2.)
        with self.assertRaises(ValueError): tracker.update(first, first, 3.)

    def test_rotation_about_rear_axle_not_image_center(self):
        tracker = GroundMotion(dict(pixels_per_m=400., origin_u=240., origin_v=600.))
        first = self.image()
        tracker.update(first, first, 1.)
        transform = cv2.getRotationMatrix2D((240.,600.), 2., 1.)
        second = cv2.warpAffine(first,transform,(480,400))
        pose = tracker.update(second,second,1.1)
        self.assertAlmostEqual(pose[0],0.,delta=.005)
        self.assertAlmostEqual(pose[1],0.,delta=.005)
        # OpenCV rotates the floor counterclockwise in the metric frame;
        # the vehicle's measured yaw is the inverse rotation.
        self.assertAlmostEqual(pose[2],-math.radians(2.),delta=.005)

    def test_ransac_rejects_moving_outliers(self):
        a = np.random.RandomState(10).uniform(-.4,.4,(60,2))
        b = a + np.array([.01,-.02])
        b[:12] += .3
        rotation, shift, count = rigid_motion(a,b)
        np.testing.assert_allclose(rotation,np.eye(2),atol=1e-8)
        np.testing.assert_allclose(shift,[.01,-.02],atol=1e-8)
        self.assertEqual(count,48)

    def test_parallel_feature_support_is_insufficient(self):
        a = np.column_stack((np.linspace(0,1,30),np.zeros(30)))
        with self.assertRaises(ValueError): rigid_motion(a,a)

    def test_scene_locks_side_and_latches_tracking_loss(self):
        cfg = dict(front_camera=dict(pixels_per_m=400.,origin_u=240.,origin_v=600.))
        builder = VisionUturnScene(cfg)
        builder.observe_lane('LEFT',.9)
        builder.set_active(True)
        white = np.zeros((400,480),np.uint8)
        cv2.line(white,(120,20),(120,380),255,8)
        blue = self.image()
        gray = cv2.bitwise_or(white,blue)
        bev = cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR)
        scene = builder.update(bev,white,blue,1.)
        self.assertIsNotNone(scene)
        self.assertEqual(scene['followed_boundary'],'LEFT')
        builder.observe_lane('RIGHT',1.01)
        self.assertEqual(builder.update(bev,white,blue,1.1)['followed_boundary'],'LEFT')
        blank = np.zeros_like(white)
        self.assertIsNone(builder.update(np.zeros_like(bev),blank,blank,1.2))
        self.assertTrue(builder.failed)
        self.assertIsNone(builder.update(bev,white,blue,1.3))

    def test_unknown_side_cannot_start(self):
        builder = VisionUturnScene(dict(front_camera=dict(pixels_per_m=400.,origin_u=240.,origin_v=600.)))
        builder.observe_lane('LEFT',.8)
        builder.observe_lane('UNKNOWN',.9)
        builder.set_active(True)
        blank = np.zeros((400,480),np.uint8)
        self.assertIsNone(builder.update(cv2.cvtColor(blank,cv2.COLOR_GRAY2BGR),blank,blank,1.))
        self.assertEqual(builder.reason,'vision_followed_boundary_unknown')

    def test_feedback_reaches_adjacent_lane_at_two_actual_speeds(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        with open(os.path.join(root,'config','competition.yaml')) as f:
            cfg = yaml.safe_load(f)
        cfg.update(uturn_speed_raw=26,steering_command_scale_rad=.1)
        for side, direction, speed_gain in [('LEFT',-1,.008),('RIGHT',1,.0055)]:
            raw = dict(stamp=1.,frame='test',pose_source='vision',pose=[0.,0.,0.],
                       lane_reference=[0.,0.,0.],followed_boundary=side,
                       regions=corridor_regions(side,.6,-.1,1.2,-.5,1.6,.03))
            task = RelativeUturn(cfg,decode_scene(raw))
            task.accept_plan(task.plan())
            self.assertEqual(task.phase,'TRACK',task.reason)
            pose = (0.,0.,0.)
            gears = set()
            for step in range(1600):
                now = 1.+step*.05
                task.observe(decode_scene(dict(raw,stamp=now,pose=list(pose))))
                speed, steer = task.command(now)
                if speed: gears.add(1 if speed>0 else -1)
                pose = bicycle(pose,speed*speed_gain*.05,
                               command_to_model_steering(steer,cfg),cfg['wheelbase'])
                if task.phase in ('DONE','BLOCKED'): break
            self.assertEqual(task.phase,'DONE','%s pose=%r index=%r end=%r' % (
                task.reason,pose,task.follower.index,task.follower.path[task.follower.segment_end]))
            self.assertEqual(gears,set((-1,1)))
            self.assertAlmostEqual(pose[1],direction*.6,delta=.045)
            self.assertLess(abs(wrap(pose[2]-math.pi)),math.radians(10))

    def test_lateral_target_and_center_divider_mirror(self):
        for side, direction in [('LEFT', -1), ('RIGHT', 1)]:
            regions = corridor_regions(side, .6, -.1, 1.2, -.5, 1.6, .03)
            allowed = lambda point: any(polygon_contains(p, point) for p in regions)
            self.assertTrue(allowed((0., direction*.6)))
            self.assertTrue(allowed((.5, direction*.3)))
            self.assertFalse(allowed((-.3, direction*.3)))
            self.assertFalse(allowed((.5, direction*.92)))
            self.assertFalse(allowed((.5, -direction*.32)))


if __name__ == '__main__': unittest.main()
