#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""Interactive multi-pose front-camera Metric BEV calibration.

The 12-by-9-square checkerboard (20 mm squares) must lie flat on the ground.
Its long, 11-inner-corner
direction must be aligned left/right, and its short, 8-inner-corner direction
must be aligned with the vehicle forward axis.  Pose coordinates describe the
centre of the inner-corner rectangle in the vehicle frame:

    x: forward from the rear-axle centre, metres
    y: left from the vehicle centre line, metres

The script only writes a candidate YAML and preview.  It never edits the
runtime source/configuration files.
"""

from __future__ import print_function

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


DEFAULT_POSES = (
    (0.55, 0.20),
    (0.55, 0.00),
    (0.55, -0.20),
    (1.10, 0.20),
    (1.10, 0.00),
    (1.10, -0.20),
)


def parse_pose(text):
    try:
        fields = text.split(",")
        if len(fields) != 2:
            raise ValueError
        x_value = float(fields[0])
        y_value = float(fields[1])
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError(
            "pose must be X,Y in metres, for example 0.55,-0.20"
        )
    if (math.isnan(x_value) or math.isinf(x_value) or
            math.isnan(y_value) or math.isinf(y_value)):
        raise argparse.ArgumentTypeError("pose must contain finite values")
    return (x_value, y_value)


def load_intrinsics(path):
    with open(path, "r") as stream:
        data = yaml.safe_load(stream)
    try:
        K = np.asarray(data["camera_matrix"]["data"], dtype=np.float64)
        D = np.asarray(
            data["distortion_coefficients"]["data"], dtype=np.float64
        )
    except (KeyError, TypeError):
        raise RuntimeError("invalid ROS camera calibration YAML: %s" % path)
    K = K.reshape(3, 3)
    D = D.reshape(-1, 1)
    if not np.isfinite(K).all() or not np.isfinite(D).all():
        raise RuntimeError("intrinsic calibration contains NaN or Inf")
    return K, D


def detect_corners(image, pattern_size):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

    for scale in (1.0, 1.5, 2.0, 3.0):
        if scale == 1.0:
            detect_gray = gray
        else:
            detect_gray = cv2.resize(
                gray,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_CUBIC,
            )

        found = False
        corners = None
        if hasattr(cv2, "findChessboardCornersSB"):
            try:
                found, corners = cv2.findChessboardCornersSB(
                    detect_gray,
                    pattern_size,
                    flags=cv2.CALIB_CB_NORMALIZE_IMAGE,
                )
            except Exception:
                found = False
                corners = None

        if not found:
            found, corners = cv2.findChessboardCorners(
                detect_gray,
                pattern_size,
                flags=flags,
            )
            if found:
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    50,
                    0.001,
                )
                corners = cv2.cornerSubPix(
                    detect_gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    criteria,
                )

        if found and corners is not None:
            corners = np.asarray(corners, dtype=np.float64)
            if scale != 1.0:
                corners /= scale
            return normalize_corner_order(corners, pattern_size)

    return None


def normalize_corner_order(corners, pattern_size):
    cols, rows = pattern_size
    grid = np.asarray(corners, dtype=np.float64).reshape(rows, cols, 2)

    # Row zero must be the visually upper/far row.
    if float(np.mean(grid[0, :, 1])) > float(np.mean(grid[-1, :, 1])):
        grid = grid[::-1, :, :]

    # Column zero must be image-left, which is vehicle-left for the front view.
    if float(np.mean(grid[:, 0, 0])) > float(np.mean(grid[:, -1, 0])):
        grid = grid[:, ::-1, :]

    return grid.reshape(-1, 1, 2)


def ground_points_for_pose(center_x, center_y, cols, rows, square_m):
    points = []
    centre_col = (cols - 1) / 2.0
    centre_row = (rows - 1) / 2.0

    for row in range(rows):
        # Image-top is farther forward after normalize_corner_order().
        x_forward = center_x + (centre_row - row) * square_m
        for col in range(cols):
            # Image-left is positive vehicle-left.
            y_left = center_y + (centre_col - col) * square_m
            points.append((x_forward, y_left))

    return np.asarray(points, dtype=np.float64)


def vehicle_points_to_bev(points, origin_u, origin_v, px_per_m):
    result = np.empty((len(points), 2), dtype=np.float64)
    result[:, 0] = origin_u - px_per_m * points[:, 1]
    result[:, 1] = origin_v - px_per_m * points[:, 0]
    return result


def fit_homography(source, destination, ransac_px):
    H, status = cv2.findHomography(
        source,
        destination,
        cv2.RANSAC,
        float(ransac_px),
    )
    if H is None:
        raise RuntimeError("findHomography failed")

    if status is None:
        inliers = np.ones(len(source), dtype=np.bool_)
    else:
        inliers = status.reshape(-1).astype(np.bool_)

    if int(np.count_nonzero(inliers)) < 4:
        raise RuntimeError("fewer than four homography inliers")

    # Remove RANSAC bias by refitting all accepted inliers without sampling.
    H_refined, _ = cv2.findHomography(
        source[inliers], destination[inliers], 0
    )
    if H_refined is not None:
        H = H_refined
    H = H / H[2, 2]
    return H, inliers


def evaluate_homography(H, source, destination):
    predicted = cv2.perspectiveTransform(
        source.reshape(-1, 1, 2).astype(np.float64), H
    ).reshape(-1, 2)
    errors = np.linalg.norm(predicted - destination, axis=1)
    return errors


def leave_one_pose_out(samples, K, D, new_K, args):
    all_errors = []
    if len(samples) < 3:
        return None

    for held_index in range(len(samples)):
        train_source = []
        train_destination = []
        held_source = None
        held_destination = None

        for index, sample in enumerate(samples):
            rectified = cv2.undistortPoints(
                sample["corners"], K, D, P=new_K
            ).reshape(-1, 2)
            ground = ground_points_for_pose(
                sample["pose"][0],
                sample["pose"][1],
                args.pattern_cols,
                args.pattern_rows,
                args.square_m,
            )
            destination = vehicle_points_to_bev(
                ground, args.origin_u, args.origin_v, args.px_per_m
            )
            if index == held_index:
                held_source = rectified
                held_destination = destination
            else:
                train_source.append(rectified)
                train_destination.append(destination)

        train_source = np.vstack(train_source)
        train_destination = np.vstack(train_destination)
        H, _ = fit_homography(
            train_source, train_destination, args.ransac_px
        )
        all_errors.extend(
            evaluate_homography(H, held_source, held_destination).tolist()
        )

    return np.asarray(all_errors, dtype=np.float64)


def ensure_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive front-camera Metric BEV calibration"
    )
    parser.add_argument(
        "--topic", default="/front/usb_cam/image_raw"
    )
    parser.add_argument("--intrinsics", required=True)
    parser.add_argument("--out", default="")
    parser.add_argument("--preview", default="")
    parser.add_argument("--pattern-cols", type=int, default=11,
                        help="inner corners across the board; 12 squares give 11")
    parser.add_argument("--pattern-rows", type=int, default=8,
                        help="inner corners front-to-back; 9 squares give 8")
    parser.add_argument("--square-m", type=float, default=0.020,
                        help="measured square side in metres (default: 0.020)")
    parser.add_argument(
        "--pose",
        action="append",
        type=parse_pose,
        default=[],
        help="inner-corner-grid centre X,Y in vehicle metres; repeatable",
    )
    parser.add_argument("--bev-width", type=int, default=480)
    parser.add_argument("--bev-height", type=int, default=400)
    parser.add_argument("--px-per-m", type=float, default=400.0)
    parser.add_argument("--origin-u", type=float, default=240.0)
    parser.add_argument("--origin-v", type=float, default=600.0)
    parser.add_argument("--ransac-px", type=float, default=2.0)
    parser.add_argument("--fit-rms-limit", type=float, default=1.5)
    parser.add_argument("--fit-max-limit", type=float, default=3.0)
    parser.add_argument("--cross-rms-limit", type=float, default=2.0)
    parser.add_argument("--cross-max-limit", type=float, default=4.0)
    return parser.parse_args()


class LatestImage(object):
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.frame = None
        self.stamp = None
        self.subscriber = rospy.Subscriber(
            topic, Image, self._callback, queue_size=1
        )

    def _callback(self, message):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
            self.stamp = time.time()
        except Exception as exc:
            rospy.logerr_throttle(2.0, "image conversion failed: %s", str(exc))


def write_candidate(path, data):
    ensure_parent(path)
    with open(path, "w") as stream:
        yaml.safe_dump(data, stream, default_flow_style=False)


def main():
    args = parse_args()
    if args.pattern_cols < 2 or args.pattern_rows < 2:
        raise RuntimeError("checkerboard pattern must be at least 2x2")
    if args.square_m <= 0.0 or args.px_per_m <= 0.0:
        raise RuntimeError("square-m and px-per-m must be positive")

    poses = list(args.pose) if args.pose else list(DEFAULT_POSES)
    if len(poses) < 2:
        raise RuntimeError("at least two board poses are required")

    K, D = load_intrinsics(args.intrinsics)
    pattern_size = (args.pattern_cols, args.pattern_rows)

    rospy.init_node("front_bev_calibrate", anonymous=True)
    latest = LatestImage(args.topic)

    window = "front_bev_calibrate"
    cv2.namedWindow(window)
    samples = []
    current_corners = None
    current_frame = None
    detection_time = 0.0

    print("Board setup:")
    print("  11-corner direction: vehicle left/right")
    print("  8-corner direction: vehicle forward")
    print("  board centre: centre of the inner-corner rectangle")
    print("Keys: SPACE=capture, R=reset all, Q/ESC=quit")

    while not rospy.is_shutdown() and len(samples) < len(poses):
        if latest.frame is None:
            key = cv2.waitKey(30) & 0xff
            if key == ord("q") or key == 27:
                return 1
            continue

        frame = latest.frame.copy()
        now = time.time()
        if now - detection_time >= 0.15:
            current_corners = detect_corners(frame, pattern_size)
            current_frame = frame.copy()
            detection_time = now

        display = frame.copy()
        if current_corners is not None:
            cv2.drawChessboardCorners(
                display,
                pattern_size,
                current_corners.astype(np.float32),
                True,
            )

        pose = poses[len(samples)]
        line_one = "Pose %d/%d centre x=%.3f m y=%.3f m" % (
            len(samples) + 1,
            len(poses),
            pose[0],
            pose[1],
        )
        line_two = "DETECTED - SPACE to capture" if current_corners is not None else "NOT DETECTED"
        cv2.putText(
            display, line_one, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (0, 255, 255), 2
        )
        cv2.putText(
            display, line_two, (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (0, 255, 0) if current_corners is not None else (0, 0, 255), 2
        )
        cv2.imshow(window, display)

        key = cv2.waitKey(30) & 0xff
        if key == ord("q") or key == 27:
            return 1
        if key == ord("r"):
            samples = []
            print("All samples reset.")
            continue
        if key == 32:
            if current_corners is None:
                print("Capture rejected: checkerboard not detected.")
                continue
            samples.append({
                "pose": pose,
                "corners": current_corners.copy(),
                "frame": current_frame.copy(),
            })
            print("Captured pose %d/%d: x=%.3f y=%.3f" % (
                len(samples), len(poses), pose[0], pose[1]
            ))
            current_corners = None
            rospy.sleep(0.2)

    if len(samples) != len(poses):
        raise RuntimeError("calibration capture incomplete")

    height, width = samples[0]["frame"].shape[:2]
    if (width, height) != (640, 360):
        raise RuntimeError(
            "front calibration requires 640x360, got %dx%d" % (width, height)
        )

    new_K, _ = cv2.getOptimalNewCameraMatrix(
        K, D, (width, height), 1.0, (width, height)
    )

    all_source = []
    all_destination = []
    all_ground = []
    sample_slices = []
    offset = 0

    for sample in samples:
        rectified = cv2.undistortPoints(
            sample["corners"], K, D, P=new_K
        ).reshape(-1, 2)
        ground = ground_points_for_pose(
            sample["pose"][0],
            sample["pose"][1],
            args.pattern_cols,
            args.pattern_rows,
            args.square_m,
        )
        destination = vehicle_points_to_bev(
            ground, args.origin_u, args.origin_v, args.px_per_m
        )
        all_source.append(rectified)
        all_destination.append(destination)
        all_ground.append(ground)
        sample_slices.append((offset, offset + len(rectified)))
        offset += len(rectified)

    all_source = np.vstack(all_source)
    all_destination = np.vstack(all_destination)
    all_ground = np.vstack(all_ground)

    H, inliers = fit_homography(
        all_source, all_destination, args.ransac_px
    )
    errors = evaluate_homography(H, all_source, all_destination)
    inlier_errors = errors[inliers]
    fit_rms = float(np.sqrt(np.mean(inlier_errors * inlier_errors)))
    fit_max = float(np.max(inlier_errors))
    inlier_count = int(np.count_nonzero(inliers))
    inlier_ratio = float(inlier_count) / float(len(inliers))

    cross_errors = leave_one_pose_out(samples, K, D, new_K, args)
    cross_rms = None
    cross_max = None
    if cross_errors is not None and len(cross_errors):
        cross_rms = float(np.sqrt(np.mean(cross_errors * cross_errors)))
        cross_max = float(np.max(cross_errors))

    passed = (
        fit_rms <= args.fit_rms_limit and
        fit_max <= args.fit_max_limit and
        inlier_ratio >= 0.90 and
        cross_rms is not None and
        cross_rms <= args.cross_rms_limit and
        cross_max <= args.cross_max_limit
    )

    np.set_printoptions(precision=10, suppress=False)
    print("")
    print("========== FRONT METRIC BEV RESULT ==========")
    print("H =")
    print(H)
    print("fit RMS = %.3f px" % fit_rms)
    print("fit MAX = %.3f px" % fit_max)
    print("inliers = %d/%d (%.1f%%)" % (
        inlier_count, len(inliers), inlier_ratio * 100.0
    ))
    if cross_rms is not None:
        print("leave-one-pose-out RMS = %.3f px" % cross_rms)
        print("leave-one-pose-out MAX = %.3f px" % cross_max)

    for index, bounds in enumerate(sample_slices):
        sample_errors = errors[bounds[0]:bounds[1]]
        sample_rms = float(np.sqrt(np.mean(sample_errors * sample_errors)))
        print("pose %d x=%.3f y=%.3f RMS=%.3f MAX=%.3f" % (
            index + 1,
            poses[index][0],
            poses[index][1],
            sample_rms,
            float(np.max(sample_errors)),
        ))

    print("ground coverage x=[%.3f, %.3f] m y=[%.3f, %.3f] m" % (
        float(np.min(all_ground[:, 0])),
        float(np.max(all_ground[:, 0])),
        float(np.min(all_ground[:, 1])),
        float(np.max(all_ground[:, 1])),
    ))
    print("STATUS = %s" % ("PASS" if passed else "RECHECK"))

    candidate = {
        "status": "PASS" if passed else "RECHECK",
        "image_width": int(width),
        "image_height": int(height),
        "pattern_inner_corners": [
            int(args.pattern_cols), int(args.pattern_rows)
        ],
        "square_size_m": float(args.square_m),
        "poses_vehicle_xy_m": [list(item) for item in poses],
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.reshape(-1).tolist(),
        "rectified_camera_matrix": new_K.tolist(),
        "homography_rectified_to_bev": H.tolist(),
        "bev_width": int(args.bev_width),
        "bev_height": int(args.bev_height),
        "pixels_per_metre": float(args.px_per_m),
        "origin_u": float(args.origin_u),
        "origin_v": float(args.origin_v),
        "fit_rms_error_px": fit_rms,
        "fit_max_error_px": fit_max,
        "inlier_count": inlier_count,
        "point_count": int(len(inliers)),
        "inlier_ratio": inlier_ratio,
        "leave_one_pose_out_rms_px": cross_rms,
        "leave_one_pose_out_max_px": cross_max,
    }

    if args.out:
        write_candidate(args.out, candidate)
        print("candidate YAML: %s" % os.path.abspath(args.out))

    if args.preview:
        ensure_parent(args.preview)
        undistorted = cv2.undistort(
            samples[-1]["frame"], K, D, None, new_K
        )
        preview = cv2.warpPerspective(
            undistorted,
            H,
            (args.bev_width, args.bev_height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        if not cv2.imwrite(args.preview, preview):
            raise RuntimeError("cannot write preview: %s" % args.preview)
        print("preview: %s" % os.path.abspath(args.preview))

    print("Runtime H files were not modified.")
    cv2.destroyAllWindows()
    return 0 if passed else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % str(exc), file=sys.stderr)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        sys.exit(1)
