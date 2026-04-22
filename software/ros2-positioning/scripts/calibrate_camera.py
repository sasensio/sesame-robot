#!/usr/bin/env python3
"""
calibrate_camera.py
====================

Calibrate the Sesame Robot's ESP32-CAM using a checkerboard pattern and save
the result to camera_calibration.yaml (compatible with the aruco_node).

Requirements
------------
  pip install opencv-python numpy pyyaml

  OR if using ROS 2 (recommended):
  sudo apt install python3-opencv python3-numpy python3-yaml

Usage
-----
  # Option A: capture frames from a running /camera/image_raw topic
  python3 calibrate_camera.py --ros

  # Option B: calibrate from a directory of pre-captured JPEG files
  python3 calibrate_camera.py --image-dir ./calibration_images/

  # Option C: capture from a local USB webcam (for testing)
  python3 calibrate_camera.py --device 0

Common options
--------------
  --board-width   N   inner corner count along board width  (default 7)
  --board-height  N   inner corner count along board height (default 5)
  --square-size   F   checkerboard square size in metres     (default 0.025)
  --output        F   output YAML file path (default camera_calibration.yaml)
  --min-frames    N   minimum frames to use for calibration  (default 20)
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
import yaml


# ---------------------------------------------------------------------------
# Checkerboard detection
# ---------------------------------------------------------------------------

def find_corners(
    image: np.ndarray,
    board_size: Tuple[int, int],
) -> Tuple[bool, np.ndarray]:
    """Find sub-pixel checkerboard corners in a grayscale or colour image."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if found:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return found, corners


def collect_from_directory(
    image_dir: str,
    board_size: Tuple[int, int],
) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    """Load images from a directory and extract checkerboard corners."""
    obj_points: List[np.ndarray] = []
    img_points: List[np.ndarray] = []
    img_size: Tuple[int, int] = (0, 0)

    # 3-D object points in the checkerboard's own coordinate system
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    files = sorted(Path(image_dir).glob("*"))
    files = [f for f in files if f.suffix.lower() in exts]

    if not files:
        print(f"No images found in {image_dir}")
        sys.exit(1)

    good = 0
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            continue
        img_size = (img.shape[1], img.shape[0])
        found, corners = find_corners(img, board_size)
        if found:
            obj_points.append(objp)
            img_points.append(corners)
            good += 1
            print(f"  ✓ {f.name}")
        else:
            print(f"  ✗ {f.name} (no corners found)")

    print(f"\n{good}/{len(files)} frames usable for calibration")
    return obj_points, img_points, img_size


def collect_from_device(
    device: int,
    board_size: Tuple[int, int],
    min_frames: int,
) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    """Interactively capture checkerboard frames from a video device."""
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"Cannot open device {device}")
        sys.exit(1)

    obj_points: List[np.ndarray] = []
    img_points: List[np.ndarray] = []
    img_size: Tuple[int, int] = (0, 0)

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    print("Point the checkerboard at the camera.")
    print("Press SPACE to capture a frame, ESC to finish early.")

    while len(obj_points) < min_frames:
        ret, frame = cap.read()
        if not ret:
            break
        img_size = (frame.shape[1], frame.shape[0])
        found, corners = find_corners(frame, board_size)
        display = frame.copy()
        cv2.drawChessboardCorners(display, board_size, corners, found)
        status = f"Captured: {len(obj_points)}/{min_frames}"
        cv2.putText(display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imshow("Calibration — SPACE=capture  ESC=done", display)
        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            break
        if key == 32 and found:  # SPACE
            obj_points.append(objp)
            img_points.append(corners)
            print(f"  Captured frame {len(obj_points)}")
            time.sleep(0.5)

    cap.release()
    cv2.destroyAllWindows()
    return obj_points, img_points, img_size


def collect_from_ros(
    board_size: Tuple[int, int],
    min_frames: int,
) -> Tuple[List[np.ndarray], List[np.ndarray], Tuple[int, int]]:
    """Capture checkerboard frames from a ROS 2 /camera/image_raw topic."""
    try:
        import rclpy
        from sensor_msgs.msg import CompressedImage
    except ImportError:
        print("rclpy not found. Install ROS 2 or use --device / --image-dir.")
        sys.exit(1)

    obj_points: List[np.ndarray] = []
    img_points: List[np.ndarray] = []
    img_size: Tuple[int, int] = (0, 0)

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    rclpy.init()
    node = rclpy.create_node("calibrate_camera_node")
    last_capture = [0.0]

    def image_cb(msg: CompressedImage) -> None:
        nonlocal img_size
        if len(obj_points) >= min_frames:
            return
        np_arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        img_size = (frame.shape[1], frame.shape[0])
        found, corners = find_corners(frame, board_size)
        if not found:
            return
        now = time.monotonic()
        if now - last_capture[0] < 1.0:  # at most 1 capture/second
            return
        last_capture[0] = now
        obj_points.append(objp)
        img_points.append(corners)
        print(f"  Auto-captured frame {len(obj_points)}/{min_frames}")

    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST, depth=1
    )
    node.create_subscription(CompressedImage, "/camera/image_raw", image_cb, qos)

    print(f"Listening on /camera/image_raw — need {min_frames} frames with visible checkerboard...")
    while rclpy.ok() and len(obj_points) < min_frames:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    return obj_points, img_points, img_size


# ---------------------------------------------------------------------------
# Calibration and output
# ---------------------------------------------------------------------------

def calibrate_and_save(
    obj_points: List[np.ndarray],
    img_points: List[np.ndarray],
    img_size: Tuple[int, int],
    square_size: float,
    output_path: str,
) -> None:
    if len(obj_points) < 5:
        print("Not enough frames for calibration (need at least 5).")
        sys.exit(1)

    # Scale object points to real-world units (metres)
    obj_points_scaled = [op * square_size for op in obj_points]

    print(f"\nRunning calibration on {len(obj_points)} frames...")
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points_scaled, img_points, img_size, None, None
    )
    print(f"RMS reprojection error: {rms:.4f} px")
    if rms > 1.5:
        print("WARNING: high RMS error — try more diverse poses or a better printed board")

    # Build YAML output compatible with camera_calibration.yaml format
    data = {
        "image_width":  img_size[0],
        "image_height": img_size[1],
        "camera_name":  "esp32cam_ov2640",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix.flatten().tolist(),
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1,
            "cols": int(dist_coeffs.shape[1]),
            "data": dist_coeffs.flatten().tolist(),
        },
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": np.eye(3).flatten().tolist(),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": np.hstack([camera_matrix, np.zeros((3, 1))]).flatten().tolist(),
        },
        "rms_reprojection_error": float(rms),
    }

    with open(output_path, "w") as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False)

    print(f"Calibration saved to: {output_path}")
    print(f"  fx={camera_matrix[0,0]:.2f}  fy={camera_matrix[1,1]:.2f}")
    print(f"  cx={camera_matrix[0,2]:.2f}  cy={camera_matrix[1,2]:.2f}")
    print(f"  dist={dist_coeffs.flatten().tolist()}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawTextHelpFormatter)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--ros",       action="store_true",
                        help="capture from /camera/image_raw ROS 2 topic")
    source.add_argument("--image-dir", metavar="DIR",
                        help="load images from a directory")
    source.add_argument("--device",    type=int, metavar="N",
                        help="OpenCV video device index")
    parser.add_argument("--board-width",  type=int, default=7,
                        help="inner corners along board width (default 7)")
    parser.add_argument("--board-height", type=int, default=5,
                        help="inner corners along board height (default 5)")
    parser.add_argument("--square-size",  type=float, default=0.025,
                        help="square side length in metres (default 0.025)")
    parser.add_argument("--output",       default="camera_calibration.yaml",
                        help="output file path")
    parser.add_argument("--min-frames",   type=int, default=20,
                        help="minimum usable frames (default 20)")
    args = parser.parse_args()

    board_size = (args.board_width, args.board_height)

    if args.image_dir:
        obj_pts, img_pts, img_sz = collect_from_directory(args.image_dir, board_size)
    elif args.device is not None:
        obj_pts, img_pts, img_sz = collect_from_device(args.device, board_size, args.min_frames)
    else:
        # Default: ROS topic
        obj_pts, img_pts, img_sz = collect_from_ros(board_size, args.min_frames)

    calibrate_and_save(obj_pts, img_pts, img_sz, args.square_size, args.output)


if __name__ == "__main__":
    main()
