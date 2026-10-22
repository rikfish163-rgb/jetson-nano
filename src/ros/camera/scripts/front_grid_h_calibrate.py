#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
"""Calibrate the front Metric BEV from a measured 3x3 ground grid.

The GUI displays the rectified 640x360 front image.  Freeze a frame and click
the nine grid intersections from near-left to far-right.  Each click is locally
refined with cornerSubPix, then a robust homography and leave-one-point-out
consistency errors are calculated.

This script only writes candidate output files.  It never updates production
source/configuration and never publishes a control command.
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


def parse_float_triplet(text, name):
    try:
        values = [float(item.strip()) for item in text.split(",")]
    except (TypeError, ValueError):
        raise RuntimeError("%s must contain three comma-separated numbers" % name)
    if len(values) != 3:
        raise RuntimeError("%s must contain exactly three numbers" % name)
    for value in values:
        if math.isnan(value) or math.isinf(value):
            raise RuntimeError("%s contains NaN or Inf" % name)
    return values


def load_intrinsics(path):
    try:
        with open(path, "r") as stream:
            data = yaml.safe_load(stream)
        K = np.asarray(data["camera_matrix"]["data"], dtype=np.float64)
        D = np.asarray(
            data["distortion_coefficients"]["data"], dtype=np.float64
        )
    except Exception as exc:
        raise RuntimeError("cannot load intrinsic YAML %s: %s" % (path, exc))
    K = K.reshape(3, 3)
    D = D.reshape(-1, 1)
    if not np.isfinite(K).all() or not np.isfinite(D).all():
        raise RuntimeError("intrinsic YAML contains NaN or Inf")
    return K, D


def make_bev_targets(x_rows, y_cols, origin_u, origin_v, px_per_m):
    targets = []
    for x_forward in x_rows:
        for y_left in y_cols:
            targets.append((
                origin_u - px_per_m * y_left,
                origin_v - px_per_m * x_forward,
            ))
    return np.asarray(targets, dtype=np.float64)


def refine_click(gray, point, window_size, max_shift):
    initial = np.asarray([[[point[0], point[1]]]], dtype=np.float32)
    refined = initial.copy()
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        50,
        0.001,
    )
    try:
        cv2.cornerSubPix(
            gray,
            refined,
            (window_size, window_size),
            (-1, -1),
            criteria,
        )
    except Exception:
        return (float(point[0]), float(point[1])), 0.0, False

    candidate = refined.reshape(2).astype(np.float64)
    if not np.isfinite(candidate).all():
        return (float(point[0]), float(point[1])), 0.0, False
    shift = float(np.linalg.norm(candidate - initial.reshape(2)))
    if shift > max_shift:
        return (float(point[0]), float(point[1])), shift, False
    return (float(candidate[0]), float(candidate[1])), shift, True


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

    H_refined, _ = cv2.findHomography(
        source[inliers], destination[inliers], 0
    )
    if H_refined is not None:
        H = H_refined
    H = H / H[2, 2]
    return H, inliers


def project_errors(H, source, destination):
    predicted = cv2.perspectiveTransform(
        source.reshape(-1, 1, 2).astype(np.float64), H
    ).reshape(-1, 2)
    return np.linalg.norm(predicted - destination, axis=1)


def leave_one_out_errors(source, destination):
    errors = []
    count = len(source)
    for held_index in range(count):
        mask = np.ones(count, dtype=np.bool_)
        mask[held_index] = False
        H, _ = cv2.findHomography(source[mask], destination[mask], 0)
        if H is None:
            errors.append(float("inf"))
            continue
        H = H / H[2, 2]
        error = project_errors(
            H,
            source[held_index:held_index + 1],
            destination[held_index:held_index + 1],
        )[0]
        errors.append(float(error))
    return np.asarray(errors, dtype=np.float64)


def ensure_parent(path):
    parent = os.path.dirname(os.path.abspath(path))
    if parent and not os.path.isdir(parent):
        os.makedirs(parent)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Calibrate front Metric BEV from a 3x3 ground grid"
    )
    parser.add_argument("--topic", default="/front/usb_cam/image_raw")
    parser.add_argument("--intrinsics", required=True)
    parser.add_argument(
        "--x-rows", default="0.30,0.60,0.90",
        help="near,middle,far forward distances in metres",
    )
    parser.add_argument(
        "--y-cols", default="0.30,0.00,-0.30",
        help="left,middle,right offsets in metres",
    )
    parser.add_argument("--bev-width", type=int, default=480)
    parser.add_argument("--bev-height", type=int, default=400)
    parser.add_argument("--px-per-m", type=float, default=400.0)
    parser.add_argument("--origin-u", type=float, default=240.0)
    parser.add_argument("--origin-v", type=float, default=600.0)
    parser.add_argument("--zoom", type=float, default=1.5)
    parser.add_argument("--corner-window", type=int, default=7)
    parser.add_argument("--max-refine-shift", type=float, default=6.0)
    parser.add_argument("--no-refine", action="store_true")
    parser.add_argument("--ransac-px", type=float, default=3.0)
    parser.add_argument("--fit-rms-limit", type=float, default=1.5)
    parser.add_argument("--fit-max-limit", type=float, default=3.0)
    parser.add_argument("--loo-rms-limit", type=float, default=2.0)
    parser.add_argument("--loo-max-limit", type=float, default=4.0)
    parser.add_argument("--out", default="")
    parser.add_argument("--preview", default="")
    return parser.parse_args()


class LatestImage(object):
    def __init__(self, topic):
        self.bridge = CvBridge()
        self.frame = None
        self.received_at = None
        self.subscriber = rospy.Subscriber(
            topic, Image, self._callback, queue_size=1
        )

    def _callback(self, message):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
            self.received_at = time.time()
        except Exception as exc:
            rospy.logerr_throttle(2.0, "image conversion failed: %s", str(exc))


def save_candidate(path, data):
    ensure_parent(path)
    with open(path, "w") as stream:
        yaml.safe_dump(data, stream, default_flow_style=False)


def draw_target_preview(rectified, H, targets, args):
    max_target_v = float(np.max(targets[:, 1]))
    preview_height = max(
        int(args.bev_height), int(math.ceil(max_target_v + 40.0))
    )
    preview = cv2.warpPerspective(
        rectified,
        H,
        (int(args.bev_width), preview_height),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    for point in targets:
        centre = (int(round(point[0])), int(round(point[1])))
        cv2.circle(preview, centre, 5, (0, 255, 255), -1)
    for u_value in sorted(set(targets[:, 0].tolist())):
        u_int = int(round(u_value))
        cv2.line(
            preview,
            (u_int, 0),
            (u_int, preview_height - 1),
            (0, 255, 0),
            1,
        )
    for v_value in sorted(set(targets[:, 1].tolist())):
        v_int = int(round(v_value))
        if 0 <= v_int < preview_height:
            cv2.line(preview, (0, v_int), (args.bev_width - 1, v_int),
                     (0, 255, 0), 1)
    return preview


def solve_and_report(points, frozen, targets, K, D, new_K, args):
    source = np.asarray(points, dtype=np.float64)
    H, inliers = fit_homography(source, targets, args.ransac_px)
    errors = project_errors(H, source, targets)
    inlier_errors = errors[inliers]
    fit_rms = float(np.sqrt(np.mean(inlier_errors * inlier_errors)))
    fit_max = float(np.max(inlier_errors))
    all_max = float(np.max(errors))
    loo_errors = leave_one_out_errors(source, targets)
    loo_rms = float(np.sqrt(np.mean(loo_errors * loo_errors)))
    loo_max = float(np.max(loo_errors))
    inlier_count = int(np.count_nonzero(inliers))

    passed = (
        inlier_count >= 8 and
        fit_rms <= args.fit_rms_limit and
        fit_max <= args.fit_max_limit and
        all_max <= args.fit_max_limit and
        loo_rms <= args.loo_rms_limit and
        loo_max <= args.loo_max_limit
    )

    np.set_printoptions(precision=10, suppress=False)
    print("")
    print("========== FRONT GRID H RESULT ==========")
    print("rectified points =")
    print(source)
    print("targets =")
    print(targets)
    print("H =")
    print(H)
    print("fit errors =")
    print(errors)
    print("fit RMS = %.3f px" % fit_rms)
    print("fit MAX = %.3f px" % all_max)
    print("inliers = %d/9" % inlier_count)
    print("leave-one-out errors =")
    print(loo_errors)
    print("leave-one-out RMS = %.3f px" % loo_rms)
    print("leave-one-out MAX = %.3f px" % loo_max)
    print("STATUS = %s" % ("PASS_CANDIDATE" if passed else "RECHECK"))

    candidate = {
        "status": "PASS_CANDIDATE" if passed else "RECHECK",
        "image_width": 640,
        "image_height": 360,
        "x_rows_m": list(args.x_rows_values),
        "y_cols_m": list(args.y_cols_values),
        "camera_matrix": K.tolist(),
        "distortion_coefficients": D.reshape(-1).tolist(),
        "rectified_camera_matrix": new_K.tolist(),
        "rectified_image_points": source.tolist(),
        "bev_target_points": targets.tolist(),
        "homography_rectified_to_bev": H.tolist(),
        "bev_width": int(args.bev_width),
        "bev_height": int(args.bev_height),
        "pixels_per_metre": float(args.px_per_m),
        "origin_u": float(args.origin_u),
        "origin_v": float(args.origin_v),
        "fit_errors_px": errors.tolist(),
        "fit_rms_px": fit_rms,
        "fit_max_px": all_max,
        "inliers": inlier_count,
        "leave_one_out_errors_px": loo_errors.tolist(),
        "leave_one_out_rms_px": loo_rms,
        "leave_one_out_max_px": loo_max,
    }

    if args.out:
        save_candidate(args.out, candidate)
        print("candidate YAML: %s" % os.path.abspath(args.out))

    preview = draw_target_preview(frozen, H, targets, args)
    if args.preview:
        ensure_parent(args.preview)
        if not cv2.imwrite(args.preview, preview):
            raise RuntimeError("cannot write preview: %s" % args.preview)
        print("extended BEV preview: %s" % os.path.abspath(args.preview))
    return H, preview, passed


def main():
    args = parse_args()
    args.x_rows_values = parse_float_triplet(args.x_rows, "x-rows")
    args.y_cols_values = parse_float_triplet(args.y_cols, "y-cols")
    if not (
        args.x_rows_values[0] < args.x_rows_values[1] < args.x_rows_values[2]
    ):
        raise RuntimeError("x-rows must be ordered near to far")
    if not (
        args.y_cols_values[0] > args.y_cols_values[1] > args.y_cols_values[2]
    ):
        raise RuntimeError("y-cols must be ordered left to right")
    if args.zoom <= 0.0 or args.px_per_m <= 0.0:
        raise RuntimeError("zoom and px-per-m must be positive")
    if args.corner_window < 2:
        raise RuntimeError("corner-window must be at least 2")

    K, D = load_intrinsics(args.intrinsics)
    new_K, _ = cv2.getOptimalNewCameraMatrix(
        K, D, (640, 360), 1.0, (640, 360)
    )
    map_x, map_y = cv2.initUndistortRectifyMap(
        K, D, None, new_K, (640, 360), cv2.CV_32FC1
    )
    targets = make_bev_targets(
        args.x_rows_values,
        args.y_cols_values,
        args.origin_u,
        args.origin_v,
        args.px_per_m,
    )

    rospy.init_node("front_grid_h_calibrate", anonymous=True)
    latest = LatestImage(args.topic)
    state = {
        "frozen": None,
        "points": [],
        "solved": False,
        "preview": None,
    }

    window = "front_grid_h_calibrate"
    cv2.namedWindow(window)

    def mouse_callback(event, x_value, y_value, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if state["frozen"] is None or len(state["points"]) >= 9:
            return
        point = (
            float(x_value) / args.zoom,
            float(y_value) / args.zoom,
        )
        height, width = state["frozen"].shape[:2]
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            return
        if args.no_refine:
            refined = point
            shift = 0.0
            accepted = False
        else:
            gray = cv2.cvtColor(state["frozen"], cv2.COLOR_BGR2GRAY)
            refined, shift, accepted = refine_click(
                gray, point, args.corner_window, args.max_refine_shift
            )
        state["points"].append(refined)
        index = len(state["points"])
        print("P%d click=(%.3f,%.3f) refined=(%.3f,%.3f) shift=%.3f used=%s" % (
            index,
            point[0], point[1],
            refined[0], refined[1],
            shift,
            str(accepted),
        ))

    cv2.setMouseCallback(window, mouse_callback)

    print("SPACE: freeze current rectified frame")
    print("Click P1..P9: near row left/centre/right, then middle, then far")
    print("U: undo last point, R: clear points, L: return to live, Q/ESC: quit")

    while not rospy.is_shutdown():
        if state["frozen"] is None:
            if latest.frame is None:
                key = cv2.waitKey(30) & 0xff
                if key == ord("q") or key == 27:
                    break
                continue
            height, width = latest.frame.shape[:2]
            if (width, height) != (640, 360):
                raise RuntimeError(
                    "front grid calibration requires 640x360, got %dx%d" % (
                        width, height
                    )
                )
            rectified = cv2.remap(
                latest.frame,
                map_x,
                map_y,
                interpolation=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
            )
            base = rectified
            status_text = "LIVE - SPACE to freeze"
        else:
            base = state["frozen"]
            status_text = "FROZEN - click P%d/9" % min(
                len(state["points"]) + 1, 9
            )

        display = cv2.resize(
            base,
            None,
            fx=args.zoom,
            fy=args.zoom,
            interpolation=cv2.INTER_CUBIC,
        )
        for index, point in enumerate(state["points"]):
            shown = (
                int(round(point[0] * args.zoom)),
                int(round(point[1] * args.zoom)),
            )
            cv2.circle(display, shown, 5, (0, 0, 255), -1)
            cv2.putText(
                display,
                "P%d" % (index + 1),
                (shown[0] + 7, shown[1] - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )
        cv2.putText(
            display,
            status_text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )
        cv2.imshow(window, display)

        if len(state["points"]) == 9 and not state["solved"]:
            _, preview, _ = solve_and_report(
                state["points"],
                state["frozen"],
                targets,
                K,
                D,
                new_K,
                args,
            )
            state["preview"] = preview
            state["solved"] = True
            cv2.imshow("front_grid_bev_preview", preview)

        key = cv2.waitKey(30) & 0xff
        if key == ord("q") or key == 27:
            break
        if key == 32 and state["frozen"] is None and latest.frame is not None:
            state["frozen"] = rectified.copy()
            state["points"] = []
            state["solved"] = False
            print("Frame frozen. Click P1..P9.")
        elif key == ord("u") and state["points"]:
            removed = state["points"].pop()
            state["solved"] = False
            print("Undo point (%.3f,%.3f)." % removed)
        elif key == ord("r"):
            state["points"] = []
            state["solved"] = False
            print("Points cleared; frozen frame retained.")
        elif key == ord("l"):
            state["frozen"] = None
            state["points"] = []
            state["solved"] = False
            print("Returned to live view.")

    rospy.signal_shutdown("finished")
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("ERROR: %s" % exc, file=sys.stderr)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        sys.exit(1)
