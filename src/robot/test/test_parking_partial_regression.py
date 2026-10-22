from __future__ import division
import json
import unittest
import cv2
import numpy as np
from robot.parking.partial_model import PartialBayModel


class PartialModelRegressionTest(unittest.TestCase):
    @staticmethod
    def metric(u, v):
        return (600 - v) / 400., (600 - u) / 400.

    @staticmethod
    def pixel(point):
        return int(round(600 - 400 * point[1])), int(round(600 - 400 * point[0]))

    def scene(self, direction=-1):
        white = np.zeros((600, 1200), np.uint8)
        mouth_y = direction * .30
        for x in (.36, .74, 1.12):
            cv2.line(white, self.pixel((x, mouth_y)),
                     self.pixel((x, mouth_y + direction * .45)), 255, 5)
        cv2.line(white, self.pixel((.36, mouth_y + direction * .45)),
                 self.pixel((1.12, mouth_y + direction * .45)), 255, 5)
        for index in range(7):
            x = .405 + .11 * index
            cv2.line(white, self.pixel((x - .025, mouth_y)),
                     self.pixel((x + .025, mouth_y)), 255, 4)
        return white, np.full_like(white, 255)

    def test_left_and_right_layouts_have_symmetric_geometry(self):
        model = PartialBayModel({}, self.metric, 400)
        for direction in (-1, 1):
            white, valid = self.scene(direction)
            report = model.evaluate(white, valid, (.36, .30 * direction, 0, direction))
            self.assertEqual(2, len(report['candidates']))
            self.assertTrue(report['pair_identity_supported'])
            self.assertTrue(all(c['y'] * direction > 0 for c in report['candidates']))
            json.dumps(report)  # ROS String status must remain JSON serializable.

    def test_third_separator_resolves_shifted_single_bay_explanation(self):
        model = PartialBayModel({}, self.metric, 400)
        white, valid = self.scene()
        complete = model.evaluate(white, valid, (.36, -.30, 0, -1))
        shifted = model.evaluate(white, valid, (.74, -.30, 0, -1))
        self.assertGreater(complete['score'] - shifted['score'], .08)

    def test_visible_missing_back_is_not_equal_to_unseen_back(self):
        model = PartialBayModel({}, self.metric, 400)
        white, valid = self.scene()
        white[:, 815:] = 0
        missing = model.evaluate(white, valid, (.36, -.30, 0, -1))
        valid[:, 815:] = 0
        unseen = model.evaluate(white, valid, (.36, -.30, 0, -1))
        self.assertGreater(unseen['score'], missing['score'])
        self.assertFalse(unseen['parts']['back']['supported'])
        self.assertEqual(0., unseen['parts']['back']['visible_fraction'])


    def test_entrance_dashes_anchor_depth_when_side_ends_are_hidden(self):
        model = PartialBayModel({}, self.metric, 400)
        white, valid = self.scene()
        white[:, 726:765] = 0
        white[:, 815:] = 0
        valid[:, 815:] = 0
        model._prepare(white, valid)
        guides = model._entrance_guides()
        self.assertTrue(guides)
        self.assertTrue(any(g['count'] >= 2 and abs(g['point'][1] + .30) < .025
                            for g in guides))
        poses, _, _ = model._seeds()
        self.assertTrue(any(abs(p[1] + .30) < .025 for p in poses))

    def test_partial_occlusion_does_not_erase_visible_bay_geometry(self):
        model = PartialBayModel({}, self.metric, 400)
        white, valid = self.scene()
        white[:, 780:824] = 0
        report = model.evaluate(white, valid, (.36, -.30, 0, -1))
        self.assertEqual(2, len(report['candidates']))
        self.assertGreater(report['dash_count'], 0)
        for index in range(3):
            self.assertLess(report['parts']['side_%d' % index]['matched_length_m'], .45)

    def test_batch_scoring_preserves_scalar_decisions(self):
        model = PartialBayModel({}, self.metric, 400)
        white, valid = self.scene()
        white[:, 780:824] = 0
        poses = [[.36, -.30, 0, -1], [.74, -.30, 0, -1], [.36, -.12, 0, -1]]
        model._prepare(white, valid)
        batched = model._rank_poses(poses)
        for pose, ranked in zip(poses, batched):
            scalar = model._fit(pose)
            self.assertAlmostEqual(ranked['score'], scalar['score'], delta=.025)
            self.assertEqual(ranked['eligible'], bool(scalar['candidates']))

    def test_full_geometry_is_decoded_only_for_shortlist(self):
        model = PartialBayModel({}, self.metric, 400)
        white, valid = self.scene()
        original_fit = model._fit
        calls = []

        def counted_fit(pose):
            calls.append(pose)
            return original_fit(pose)

        model._fit = counted_fit
        _, diagnostic = model.detect(white, valid)
        self.assertGreater(diagnostic['model']['scored_poses'], len(calls))
        self.assertLessEqual(len(calls), 8)
        self.assertEqual(len(calls), diagnostic['model']['decoded_poses'])


if __name__ == '__main__':
    unittest.main()
