#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""Create a rear-camera ground-plane homography from one checkerboard image.

The checkerboard must be flat on the ground and aligned with the vehicle:
the 10-square edge is perpendicular to the vehicle centre line and the
7-square edge points away from the camera.  For a board printed as 10 x 7
squares, OpenCV sees 9 x 6 inner corners.

The output YAML is intentionally a small, self-contained file consumed by
rear_bev_node.py.  It stores the intrinsic calibration, the rectified camera
matrix, and a metric pixel-to-ground homography.
"""

from __future__ import print_function

import argparse
import os
import sys

import cv2
import numpy as np
import yaml


DEFAULT_COLS = 9
DEFAULT_ROWS = 6
DEFAULT_BEV_WIDTH = 480
DEFAULT_BEV_HEIGHT = 400
DEFAULT_PX_PER_M = 400.0


def _as_float_array(values, shape=None):
    array = np.asarray(values, dtype=np.float64)
    if shape is not None:
        array = array.reshape(shape)
    return array


def load_intrinsics(path):
    """Read the ROS camera_calibration YAML format."""
    try:
        with open(path, "r") as stream:
            data = yaml.safe_load(stream)
    except Exception as exc:
        raise RuntimeError("cannot read intrinsic YAML %s: %s" % (path, exc))

    if not isinstance(data, dict):
        raise RuntimeError("intrinsic YAML is not a mapping: %s" % path)

    # camera_calibration writes nested {rows, cols, data} mappings.
    try:
        camera_matrix = data["camera_matrix"]["data"]
        distortion = data["distortion_coefficients"]["data"]
    except (KeyError, TypeError):
        raise RuntimeError(
            "expected camera_matrix.data and distortion_coefficients.data in %s"
            % path
        )

    K = _as_float_array(camera_matrix, (3, 3))
    D = _as_float_array(distortion).reshape(-1, 1)

    if not np.isfinite(K).all() or not np.isfinite(D).all():
        raise RuntimeError("intrinsic YAML contains NaN or Inf: %s" % path)

    return K, D


def detect_corners(image, pattern_size):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
             cv2.CALIB_CB_NORMALIZE_IMAGE)

    # The rear board occupies only a small part of a 640x480 frame.  Try the
    # original image and enlarged copies, but always return coordinates in the
    # original image so the intrinsic matrix and runtime resolution remain
    # unchanged.  FAST_CHECK is intentionally omitted: the OpenCV 4.1 build
    # on the Jetson raises a binding TypeError when it is passed here.
    for scale in (1.0, 2.0, 3.0):
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
                    40,
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
            return corners.reshape(-1, 1, 2)

    return None


def make_ground_points(cols, rows, square_m, near_inner_y_m,
                       board_center_x_m, reverse_depth, mirror_x):
    """Return metric (x, y) points matching findChessboardCorners order.

    x is left/right on the ground plane; y is distance behind the camera,
    positive away from the vehicle.  near_inner_y_m is the distance from the
    camera ground projection to the nearest row of inner corners.
    """
    points = []
    centre_col = (cols - 1) / 2.0

    for row in range(rows):
        if reverse_depth:
            depth_index = row
        else:
            # In a rear-facing image, the top row is normally farther away
            # and the bottom row is normally nearer to the vehicle.
            depth_index = rows - 1 - row

        y = near_inner_y_m + depth_index * square_m

        for col in range(cols):
            x = (col - centre_col) * square_m + board_center_x_m
            if mirror_x:
                x = -x
            points.append((x, y))

    return np.asarray(points, dtype=np.float64)


def ground_to_bev(points_xy, width, height, px_per_m):
    """Map ground metres to an image with x centred and y farther at the top."""
    x_min = -float(width) / (2.0 * px_per_m)
    y_min = 0.0
    y_max = float(height) / px_per_m

    dst = np.empty((len(points_xy), 2), dtype=np.float64)
    dst[:, 0] = (points_xy[:, 0] - x_min) * px_per_m
    dst[:, 1] = (y_max - points_xy[:, 1]) * px_per_m

    return dst, x_min, y_min, y_max


def ensure_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


def write_yaml(path, data):
    ensure_parent(path)
    with open(path, "w") as stream:
        yaml.safe_dump(data, stream, default_flow_style=False)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate a rear-camera ground-plane BEV from a checkerboard"
    )
    parser.add_argument("--image", required=True,
                        help="one captured rear-camera image with the board on the ground")
    parser.add_argument("--intrinsics", required=True,
                        help="ROS camera_calibration YAML, for example ost.yaml")
    parser.add_argument("--out", required=True,
                        help="output rear BEV YAML")
    parser.add_argument("--preview", default="",
                        help="optional output BEV preview image")
    parser.add_argument("--square-m", type=float, required=True,
                        help="actual checkerboard square side in metres")
    parser.add_argument("--near-inner-y-m", type=float, required=True,
                        help="distance from camera ground projection to nearest inner-corner row")
    parser.add_argument("--board-center-x-m", type=float, default=0.0,
                        help="board centre x relative to camera, metres; default 0")
    parser.add_argument("--pattern-cols", type=int, default=DEFAULT_COLS,
                        help="inner-corner columns; 10x7 squares means 9")
    parser.add_argument("--pattern-rows", type=int, default=DEFAULT_ROWS,
                        help="inner-corner rows; 10x7 squares means 6")
    parser.add_argument("--bev-width", type=int, default=DEFAULT_BEV_WIDTH)
    parser.add_argument("--bev-height", type=int, default=DEFAULT_BEV_HEIGHT)
    parser.add_argument("--px-per-m", type=float, default=DEFAULT_PX_PER_M,
                        help="BEV scale; 400 means 4 pixels per centimetre")
    parser.add_argument("--reverse-depth", action="store_true",
                        help="use image top row as nearest instead of farthest")
    parser.add_argument("--mirror-x", action="store_true",
                        help="flip ground x if the camera image is horizontally mirrored")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.square_m <= 0 or args.near_inner_y_m < 0:
        raise RuntimeError("square-m must be > 0 and near-inner-y-m must be >= 0")
    if args.pattern_cols < 2 or args.pattern_rows < 2:
        raise RuntimeError("pattern must contain at least 2 x 2 inner corners")
    if args.bev_width <= 0 or args.bev_height <= 0 or args.px_per_m <= 0:
        raise RuntimeError("BEV dimensions and px-per-m must be positive")

    K, D = load_intrinsics(args.intrinsics)
    image = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot open image: %s" % args.image)

    height, width = image.shape[:2]
    pattern_size = (args.pattern_cols, args.pattern_rows)
    corners = detect_corners(image, pattern_size)
    if corners is None:
        raise RuntimeError(
            "checkerboard not detected in %s; expected %d x %d inner corners"
            % (args.image, args.pattern_cols, args.pattern_rows)
        )

    new_K, _ = cv2.getOptimalNewCameraMatrix(
        K, D, (width, height), 1.0, (width, height)
    )

    # H is fitted in the same rectified pixel coordinate system used by the
    # runtime node, rather than in distorted raw-image coordinates.
    rectified_corners = cv2.undistortPoints(
        corners, K, D, P=new_K
    ).reshape(-1, 2)

    ground_points = make_ground_points(
        args.pattern_cols,
        args.pattern_rows,
        args.square_m,
        args.near_inner_y_m,
        args.board_center_x_m,
        args.reverse_depth,
        args.mirror_x,
    )
    bev_points, x_min, y_min, y_max = ground_to_bev(
        ground_points,
        args.bev_width,
        args.bev_height,
        args.px_per_m,
    )

    H, status = cv2.findHomography(
        rectified_corners,
        bev_points,
        cv2.RANSAC,
        3.0,
    )
    if H is None:
        raise RuntimeError("findHomography failed")
    H = H / H[2, 2]

    projected = cv2.perspectiveTransform(
        rectified_corners.reshape(-1, 1, 2).astype(np.float64), H
    ).reshape(-1, 2)
    errors = np.linalg.norm(projected - bev_points, axis=1)
    rms_error = float(np.sqrt(np.mean(errors * errors)))
    max_error = float(np.max(errors))
    inliers = int(np.count_nonzero(status)) if status is not None else len(errors)

    undistorted = cv2.undistort(image, K, D, None, new_K)
    bev_preview = cv2.warpPerspective(
        undistorted,
        H,
        (args.bev_width, args.bev_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    if args.preview:
        ensure_parent(args.preview)
        if not cv2.imwrite(args.preview, bev_preview):
            raise RuntimeError("cannot write preview: %s" % args.preview)

    data = {
        "image_width": int(width),
        "image_height": int(height),
        "pattern_inner_corners": [int(args.pattern_cols), int(args.pattern_rows)],
        "square_size_m": float(args.square_m),
        "near_inner_y_m": float(args.near_inner_y_m),
        "board_center_x_m": float(args.board_center_x_m),
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.reshape(-1).tolist(),
        "rectified_camera_matrix": new_K.tolist(),
        "homography_rectified_to_bev": H.tolist(),
        "bev_width": int(args.bev_width),
        "bev_height": int(args.bev_height),
        "pixels_per_metre": float(args.px_per_m),
        "bev_x_min_m": float(x_min),
        "bev_y_min_m": float(y_min),
        "bev_y_max_m": float(y_max),
        "depth_order": "top_row_far_to_bottom_row_near" if not args.reverse_depth
                       else "top_row_near_to_bottom_row_far",
        "mirror_x": bool(args.mirror_x),
        "homography_rms_error_px": rms_error,
        "homography_max_error_px": max_error,
        "homography_inliers": inliers,
        "source_image": os.path.abspath(args.image),
        "source_intrinsics": os.path.abspath(args.intrinsics),
    }
    write_yaml(args.out, data)

    print("rear BEV calibration written: %s" % os.path.abspath(args.out))
    print("image size: %d x %d" % (width, height))
    print("inner corners: %d x %d (%d points)" % (
        args.pattern_cols, args.pattern_rows, len(corners)
    ))
    print("homography RMS error: %.3f px; max: %.3f px; inliers: %d/%d" % (
        rms_error, max_error, inliers, len(errors)
    ))
    if args.preview:
        print("BEV preview: %s" % os.path.abspath(args.preview))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        sys.exit(2)
