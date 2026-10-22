import math
import unittest
import numpy as np
import cv2
from robot.common.geometry import local
from robot.camera.landmarks import BlueLandmarks
from robot.camera.landmarks import image_segments
from robot.uturn.vision import VisionUturnScene


class BlueLandmarkTests(unittest.TestCase):
    def lines(self, pose=(0,0,0), separation=1.2):
        # Horizontal start is deliberately offset: that dimension was not supplied.
        world = [((.15,separation/2),(1.15,separation/2)),
                 ((0,-.5),(0,.5)),
                 ((.15,-separation/2),(1.15,-separation/2))]
        return [np.array([local(pose,a),local(pose,b)]) for a,b in world]

    def test_confirmed_dimensions_and_no_guessed_horizontal_gap(self):
        model=BlueLandmarks({})
        model.update(self.lines(),(0,0,0),1.)
        self.assertIsNotNone(model.landmarks)
        self.assertAlmostEqual(model.separation,1.2)

    def test_wrong_separation_is_rejected(self):
        model=BlueLandmarks({})
        model.update(self.lines(separation=1.5),(0,0,0),1.)
        self.assertIsNone(model.landmarks)

    def test_two_parallel_lines_cannot_claim_full_pose_correction(self):
        model=BlueLandmarks({})
        model.update(self.lines(),(0,0,0),1.)
        pose=(.2,.1,.3)
        predicted=(.23,.13,.33)
        corrected=model.update([self.lines(pose)[0],self.lines(pose)[2]],predicted,1.1)
        self.assertEqual(corrected,predicted)
        self.assertFalse(model.corrected)

    def test_partial_perpendicular_lines_correct_pose_without_using_cut_endpoints(self):
        model=BlueLandmarks({})
        model.update(self.lines(),(0,0,0),1.)
        actual=(.2,.1,.3)
        partial=[]
        for line in self.lines(actual)[:2]:
            a,b=line
            partial.append(np.array([a+.2*(b-a),a+.7*(b-a)]))
        result=model.update(partial,(.23,.13,.33),1.1)
        self.assertTrue(model.corrected)
        np.testing.assert_allclose(result,actual,atol=1e-6)

    def test_large_jump_is_not_relocalized_to_wrong_marker(self):
        model=BlueLandmarks({})
        model.update(self.lines(),(0,0,0),1.)
        predicted=(.5,.5,0.)
        self.assertEqual(model.update(self.lines(),predicted,1.1),predicted)
        self.assertFalse(model.corrected)

    def test_wrong_side_parallel_segments_are_not_a_triplet(self):
        segments=self.lines()
        segments[2][:,0]*=-1
        model=BlueLandmarks({})
        model.update(segments,(0,0,0),1.)
        self.assertIsNone(model.landmarks)

    def test_acquisition_across_frames_does_not_require_all_lines_in_one_frame(self):
        model=BlueLandmarks({})
        model.update(self.lines()[:2],(0,0,0),1.)
        self.assertIsNone(model.landmarks)
        model.update(self.lines()[2:],(0,0,0),1.1)
        self.assertIsNotNone(model.landmarks)

    def test_wide_image_approach_collection_freezes_origin_at_trigger(self):
        cfg=dict(front_camera=dict(pixels_per_m=400.,origin_u=240.,origin_v=600.),
                 uturn_blue_landmarks_enabled=True)
        scene=VisionUturnScene(cfg)
        scene.observe_lane('LEFT',.9)
        scene.requested=True
        white=np.zeros((400,1200),np.uint8)
        blue=np.zeros_like(white)
        cv2.line(white,(480,20),(480,380),255,8)
        for u in (360,840): cv2.line(blue,(u,0),(u,220),255,8)
        cv2.line(blue,(400,280),(800,280),255,8)
        bev=np.zeros((400,1200,3),np.uint8)
        bev[white>0]=(255,255,255)
        bev[blue>0]=(255,80,0)
        self.assertIsNone(scene.update(bev,white,blue,1.))
        self.assertIsNotNone(scene.blue_landmarks.landmarks,scene.reason)
        scene.requested=False
        scene.set_active(True)
        transform=np.float32([[1,0,0],[0,1,8]])
        moved=[cv2.warpAffine(img,transform,(1200,400)) for img in (bev,white,blue)]
        result=scene.update(moved[0],moved[1],moved[2],1.1)
        self.assertIsNotNone(result,scene.reason)
        self.assertAlmostEqual(result['lane_reference'][0],.02,delta=.008)
        self.assertAlmostEqual(result['pose'][0],.02,delta=.008)
        self.assertEqual(result['followed_boundary'],'LEFT')
        self.assertTrue(scene.blue_landmarks.corrected)
        blank=np.zeros_like(blue)
        self.assertIsNone(scene.update(np.zeros_like(bev),blank,blank,1.2))
        self.assertTrue(scene.failed)
        self.assertIsNone(scene.update(moved[0],moved[1],moved[2],1.3))


if __name__=='__main__': unittest.main()
