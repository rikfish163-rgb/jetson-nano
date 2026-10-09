# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import shutil
import sys
import tempfile
import unittest


SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from parking_calibration_check import check_readiness  # noqa: E402
from parking_controller import ParkingConfig  # noqa: E402


try:
    import yaml
except ImportError:  # pragma: no cover - the runtime package requires PyYAML
    yaml = None


@unittest.skipIf(yaml is None, "PyYAML is required")
class ParkingCalibrationCheckTest(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="parking_check_")

    def tearDown(self):
        shutil.rmtree(self.root)

    def path(self, name):
        return os.path.join(self.root, name)

    def write_yaml(self, name, data):
        path = self.path(name)
        with open(path, "w") as stream:
            yaml.safe_dump(data, stream, default_flow_style=False)
        return path

    @staticmethod
    def calibrated_profiles():
        profile = dict(
            (field, ParkingConfig.DEFAULTS[field])
            for field in ParkingConfig.CALIBRATED_SLOT_FIELDS
        )
        return {"P4": dict(profile), "P5": dict(profile)}

    def ready_paths(self):
        config = self.write_yaml("parking.yaml", {
            "calibration_complete": True,
            "supported_slot_ids": ["P4", "P5"],
            "default_slot_id": "P4",
            "parking_side": "right",
            "slot_profiles": self.calibrated_profiles(),
        })
        front = self.write_yaml("front.yaml", {
            "derive_pose_from_candidate": True,
        })
        rear = self.write_yaml("rear.yaml", {
            "line_color": "white",
            "expected_slot_width_m": 0.32,
            "camera_to_rear_axle_m": 0.10,
            "camera_to_rear_axle_calibrated": True,
            "line_color_calibrated": True,
        })
        exit_config = self.write_yaml("exit.yaml", {
            "use_candidate": True,
        })
        rear_bev = self.write_yaml("rear_bev.yaml", {
            "image_width": 640,
            "image_height": 480,
            "bev_width": 480,
            "bev_height": 400,
            "pixels_per_metre": 400.0,
            "camera_matrix": [
                [1.0, 0.0, 2.0],
                [0.0, 1.0, 3.0],
                [0.0, 0.0, 1.0],
            ],
            "distortion_coefficients": [0.1, 0.2, 0.0, 0.0, 0.0],
            "rectified_camera_matrix": [],
            "homography_rectified_to_bev": [],
        })
        front_calibration = self.write_yaml("front_calibration.yaml", {
            "image_width": 640,
            "image_height": 360,
            "camera_matrix": {},
            "distortion_coefficients": {},
        })
        rear_calibration = self.write_yaml("rear_calibration.yaml", {
            "image_width": 640,
            "image_height": 480,
            "camera_matrix": {
                "rows": 3, "cols": 3,
                "data": [1.0, 0.0, 2.0, 0.0, 1.0, 3.0, 0.0, 0.0, 1.0],
            },
            "distortion_coefficients": {
                "rows": 1, "cols": 5,
                "data": [0.1, 0.2, 0.0, 0.0, 0.0],
            },
        })
        return (config, front, rear, exit_config, rear_bev,
                front_calibration, rear_calibration)

    def test_current_workspace_is_not_ready_by_default(self):
        report = check_readiness()
        self.assertFalse(report["motion_ready"])
        self.assertIn("calibration_complete", report["blocking"])
        self.assertIn("slot_profiles_present", report["blocking"])
        self.assertIn("front_pose_source", report["blocking"])
        self.assertIn("exit_pose_source", report["blocking"])

    def test_complete_static_candidate_configuration_is_ready(self):
        paths = self.ready_paths()
        report = check_readiness(
            config_path=paths[0],
            front_config_path=paths[1],
            rear_config_path=paths[2],
            exit_config_path=paths[3],
            rear_bev_path=paths[4],
            front_calibration_path=paths[5],
            rear_calibration_path=paths[6],
            front_pose_mode="candidate_affine",
            exit_pose_mode="candidate_affine",
        )
        self.assertTrue(report["motion_ready"])
        self.assertTrue(report["static_visual_ready"])
        self.assertEqual(report["blocking"], [])

    def test_lane_pose_is_accepted_as_dynamic_exit_source(self):
        paths = self.ready_paths()
        report = check_readiness(
            config_path=paths[0],
            front_config_path=paths[1],
            rear_config_path=paths[2],
            exit_config_path=paths[3],
            rear_bev_path=paths[4],
            front_calibration_path=paths[5],
            rear_calibration_path=paths[6],
            front_pose_mode="lane_pose",
            exit_pose_mode="lane_pose",
            start_lane_nodes=True,
            start_lane_pose=True,
        )
        self.assertTrue(report["motion_ready"])
        self.assertTrue(report["static_visual_ready"])
        self.assertNotIn("exit_pose_source", report["blocking"])


if __name__ == "__main__":
    unittest.main()
