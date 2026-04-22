"""
sesame_aruco.aruco_node
=======================

ROS 2 node that subscribes to camera frames published by the Sesame Robot
micro-ROS firmware, detects ArUco markers, and estimates the robot's pose
in the world frame.

Topics
------
Subscribes:
  /camera/image_raw          sensor_msgs/CompressedImage   JPEG from ESP32-CAM

Publishes:
  /sesame/pose               geometry_msgs/PoseStamped     robot pose in world frame
  /aruco/markers             visualization_msgs/MarkerArray  rviz2 visualisation
  /sesame/debug_image        sensor_msgs/CompressedImage   annotated frame (optional)

Parameters
----------
  camera_calibration_file  (str)  path to camera_calibration.yaml
  marker_map_file          (str)  path to marker_map.yaml
  marker_length            (float) physical marker side length in metres (default 0.10)
  aruco_dict               (str)  ArUco dictionary name (default "DICT_4X4_50")
  publish_debug_image      (bool) whether to republish annotated frames (default False)
  robot_marker_id          (int)  ID of the marker attached to the robot (default 0)

Marker Map File Format (marker_map.yaml)
-----------------------------------------
  # Maps ArUco marker IDs to their known pose in the world frame.
  # Position in metres, orientation as roll/pitch/yaw in degrees.
  markers:
    - id: 1
      x: 0.0
      y: 0.0
      yaw: 0.0
    - id: 2
      x: 1.0
      y: 0.5
      yaw: 90.0

Usage
-----
  ros2 run sesame_aruco aruco_node \\
    --ros-args \\
    -p camera_calibration_file:=/path/to/camera_calibration.yaml \\
    -p marker_map_file:=/path/to/marker_map.yaml \\
    -p marker_length:=0.10 \\
    -p robot_marker_id:=0
"""

import math
import os
from typing import Dict, List, Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Vector3,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _yaw_to_quaternion(yaw_deg: float) -> Quaternion:
    """Convert a yaw angle (degrees, around Z) to a geometry_msgs Quaternion."""
    half = math.radians(yaw_deg) / 2.0
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def _rvec_tvec_to_pose(rvec: np.ndarray, tvec: np.ndarray) -> Pose:
    """Convert OpenCV (rvec, tvec) to geometry_msgs/Pose."""
    rot_mat, _ = cv2.Rodrigues(rvec)
    pose = Pose()
    pose.position.x = float(tvec[0])
    pose.position.y = float(tvec[1])
    pose.position.z = float(tvec[2])

    # Convert rotation matrix to quaternion
    trace = rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        pose.orientation.w = 0.25 / s
        pose.orientation.x = (rot_mat[2, 1] - rot_mat[1, 2]) * s
        pose.orientation.y = (rot_mat[0, 2] - rot_mat[2, 0]) * s
        pose.orientation.z = (rot_mat[1, 0] - rot_mat[0, 1]) * s
    elif rot_mat[0, 0] > rot_mat[1, 1] and rot_mat[0, 0] > rot_mat[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot_mat[0, 0] - rot_mat[1, 1] - rot_mat[2, 2])
        pose.orientation.w = (rot_mat[2, 1] - rot_mat[1, 2]) / s
        pose.orientation.x = 0.25 * s
        pose.orientation.y = (rot_mat[0, 1] + rot_mat[1, 0]) / s
        pose.orientation.z = (rot_mat[0, 2] + rot_mat[2, 0]) / s
    elif rot_mat[1, 1] > rot_mat[2, 2]:
        s = 2.0 * math.sqrt(1.0 + rot_mat[1, 1] - rot_mat[0, 0] - rot_mat[2, 2])
        pose.orientation.w = (rot_mat[0, 2] - rot_mat[2, 0]) / s
        pose.orientation.x = (rot_mat[0, 1] + rot_mat[1, 0]) / s
        pose.orientation.y = 0.25 * s
        pose.orientation.z = (rot_mat[1, 2] + rot_mat[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + rot_mat[2, 2] - rot_mat[0, 0] - rot_mat[1, 1])
        pose.orientation.w = (rot_mat[1, 0] - rot_mat[0, 1]) / s
        pose.orientation.x = (rot_mat[0, 2] + rot_mat[2, 0]) / s
        pose.orientation.y = (rot_mat[1, 2] + rot_mat[2, 1]) / s
        pose.orientation.z = 0.25 * s
    return pose


def _aruco_dict_from_name(name: str):
    """Return the cv2.aruco dict constant for a dictionary name string."""
    mapping = {
        "DICT_4X4_50":       aruco.DICT_4X4_50,
        "DICT_4X4_100":      aruco.DICT_4X4_100,
        "DICT_4X4_250":      aruco.DICT_4X4_250,
        "DICT_4X4_1000":     aruco.DICT_4X4_1000,
        "DICT_5X5_50":       aruco.DICT_5X5_50,
        "DICT_5X5_100":      aruco.DICT_5X5_100,
        "DICT_6X6_50":       aruco.DICT_6X6_50,
        "DICT_6X6_100":      aruco.DICT_6X6_100,
        "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
    }
    return mapping.get(name.upper(), aruco.DICT_4X4_50)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ArucoNode(Node):
    """Detect ArUco markers and publish the robot's estimated world pose."""

    def __init__(self) -> None:
        super().__init__("aruco_node")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("camera_calibration_file", "")
        self.declare_parameter("marker_map_file", "")
        self.declare_parameter("marker_length", 0.10)
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter("robot_marker_id", 0)

        cal_file    = self.get_parameter("camera_calibration_file").value
        map_file    = self.get_parameter("marker_map_file").value
        marker_len  = float(self.get_parameter("marker_length").value)
        dict_name   = str(self.get_parameter("aruco_dict").value)
        self._pub_debug = bool(self.get_parameter("publish_debug_image").value)
        self._robot_id  = int(self.get_parameter("robot_marker_id").value)
        self._marker_length = marker_len

        # ── Camera calibration ────────────────────────────────────────────────
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs:   Optional[np.ndarray] = None
        self._load_calibration(cal_file)

        # ── Marker map (world positions of fixed reference markers) ───────────
        # Dict[marker_id] → (x_world, y_world, yaw_world_deg)
        self._marker_map: Dict[int, Tuple[float, float, float]] = {}
        self._load_marker_map(map_file)

        # ── ArUco detector ────────────────────────────────────────────────────
        aruco_dict = aruco.Dictionary_get(_aruco_dict_from_name(dict_name))
        self._aruco_params = aruco.DetectorParameters_create()
        self._aruco_dict   = aruco_dict

        # ── QoS ──────────────────────────────────────────────────────────────
        # Best-effort matches the micro-ROS firmware publisher QoS
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriber ───────────────────────────────────────────────────────
        self._image_sub = self.create_subscription(
            CompressedImage,
            "/camera/image_raw",
            self._image_callback,
            qos_be,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._pose_pub = self.create_publisher(
            PoseStamped, "/sesame/pose", 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/aruco/markers", 10
        )
        if self._pub_debug:
            self._debug_pub = self.create_publisher(
                CompressedImage, "/sesame/debug_image", qos_be
            )

        self.get_logger().info(
            f"ArUco node ready. Dict={dict_name}, "
            f"marker_length={marker_len:.3f} m, "
            f"robot_marker_id={self._robot_id}, "
            f"calibration={'loaded' if self._camera_matrix is not None else 'MISSING'}, "
            f"reference_markers={list(self._marker_map.keys())}"
        )

    # ── Configuration loaders ─────────────────────────────────────────────────

    def _load_calibration(self, path: str) -> None:
        """Load camera_matrix and dist_coeffs from a YAML calibration file."""
        if not path or not os.path.isfile(path):
            self.get_logger().warning(
                f"Camera calibration file not found: '{path}'. "
                "Pose estimates will be inaccurate. "
                "Run calibrate_camera.py to generate one."
            )
            return
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        try:
            self._camera_matrix = np.array(
                data["camera_matrix"]["data"], dtype=np.float64
            ).reshape((3, 3))
            self._dist_coeffs = np.array(
                data["distortion_coefficients"]["data"], dtype=np.float64
            )
            self.get_logger().info(f"Camera calibration loaded from {path}")
        except (KeyError, ValueError) as exc:
            self.get_logger().error(f"Calibration file parse error: {exc}")

    def _load_marker_map(self, path: str) -> None:
        """Load the known world positions of fixed reference markers."""
        if not path or not os.path.isfile(path):
            self.get_logger().warning(
                f"Marker map file not found: '{path}'. "
                "World-frame pose will not be computed."
            )
            return
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        for entry in data.get("markers", []):
            mid  = int(entry["id"])
            x    = float(entry.get("x", 0.0))
            y    = float(entry.get("y", 0.0))
            yaw  = float(entry.get("yaw", 0.0))
            self._marker_map[mid] = (x, y, yaw)
        self.get_logger().info(
            f"Marker map loaded: {len(self._marker_map)} fixed markers"
        )

    # ── Image callback ────────────────────────────────────────────────────────

    def _image_callback(self, msg: CompressedImage) -> None:
        # Decode JPEG
        np_arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning("Failed to decode image frame")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params
        )

        if ids is None or len(ids) == 0:
            return

        ids_flat: List[int] = ids.flatten().tolist()

        # ── Pose estimation (only if calibration is available) ────────────────
        robot_pose_stamped: Optional[PoseStamped] = None
        marker_array = MarkerArray()

        if self._camera_matrix is not None and self._dist_coeffs is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                self._marker_length,
                self._camera_matrix,
                self._dist_coeffs,
            )

            stamp = self._ros_time_from_header(msg.header.stamp)

            for i, mid in enumerate(ids_flat):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Build rviz2 visualisation marker
                viz_marker = self._make_viz_marker(mid, tvec, rvec, stamp)
                marker_array.markers.append(viz_marker)

                # If this is the robot marker, compute world pose
                if mid == self._robot_id:
                    robot_pose_stamped = self._compute_world_pose(
                        rvec, tvec, ids_flat, rvecs, tvecs, stamp
                    )

                # Annotate debug frame
                if self._pub_debug:
                    aruco.drawAxis(
                        frame, self._camera_matrix, self._dist_coeffs,
                        rvec, tvec, self._marker_length * 0.5
                    )

            if marker_array.markers:
                self._marker_pub.publish(marker_array)

            if robot_pose_stamped is not None:
                self._pose_pub.publish(robot_pose_stamped)

        # Annotate corners regardless of calibration
        if self._pub_debug:
            aruco.drawDetectedMarkers(frame, corners, ids)
            self._publish_debug(frame, msg.header)

    # ── World pose computation ────────────────────────────────────────────────

    def _compute_world_pose(
        self,
        robot_rvec: np.ndarray,
        robot_tvec: np.ndarray,
        all_ids: List[int],
        all_rvecs: np.ndarray,
        all_tvecs: np.ndarray,
        stamp: Time,
    ) -> Optional[PoseStamped]:
        """
        Compute the robot's pose in the world frame using the fixed reference
        markers defined in the marker map.

        Strategy: use the first visible fixed marker to transform the robot's
        camera-frame pose into the world frame.  If no fixed markers are
        visible, fall back to the camera-frame pose with frame_id="camera".
        """
        # Try to find a fixed reference marker in the current frame
        for ref_id, (wx, wy, wyaw) in self._marker_map.items():
            if ref_id not in all_ids:
                continue
            j = all_ids.index(ref_id)
            ref_rvec = all_rvecs[j][0]
            ref_tvec = all_tvecs[j][0]

            # Transform robot pose relative to reference marker
            world_pose = self._transform_to_world(
                robot_rvec, robot_tvec,
                ref_rvec, ref_tvec,
                wx, wy, wyaw,
            )
            ps = PoseStamped()
            ps.header.stamp    = stamp
            ps.header.frame_id = "map"
            ps.pose            = world_pose
            return ps

        # No reference marker visible — publish camera-relative pose
        ps = PoseStamped()
        ps.header.stamp    = stamp
        ps.header.frame_id = "camera"
        ps.pose            = _rvec_tvec_to_pose(robot_rvec, robot_tvec)
        return ps

    @staticmethod
    def _transform_to_world(
        robot_rvec: np.ndarray,
        robot_tvec: np.ndarray,
        ref_rvec: np.ndarray,
        ref_tvec: np.ndarray,
        ref_world_x: float,
        ref_world_y: float,
        ref_world_yaw_deg: float,
    ) -> Pose:
        """
        Given the camera-frame pose of both the robot marker and a reference
        marker (whose world position is known), compute the robot's world pose.

        The reference marker lies in the ground plane (z=0) and its yaw gives
        its orientation relative to the world X-axis.
        """
        # Camera → reference marker transform
        ref_rot, _ = cv2.Rodrigues(ref_rvec)
        # Vector from reference marker to robot in camera frame
        cam_to_ref  = ref_tvec
        cam_to_robot = robot_tvec
        rel_tvec = cam_to_robot - cam_to_ref

        # Rotate relative vector into the reference marker's frame
        local_pos = ref_rot.T @ rel_tvec

        # Apply world rotation (yaw around Z)
        yaw_rad = math.radians(ref_world_yaw_deg)
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
        world_x = ref_world_x + cos_y * local_pos[0] - sin_y * local_pos[1]
        world_y = ref_world_y + sin_y * local_pos[0] + cos_y * local_pos[1]

        # Combine yaw from reference and robot-to-camera rotation
        robot_rot, _ = cv2.Rodrigues(robot_rvec)
        combined_rot = ref_rot.T @ robot_rot
        rel_rvec, _ = cv2.Rodrigues(combined_rot)
        robot_pose_rel = _rvec_tvec_to_pose(
            rel_rvec,
            np.array([local_pos[0], local_pos[1], local_pos[2]])
        )
        pose = Pose()
        pose.position.x    = world_x
        pose.position.y    = world_y
        pose.position.z    = 0.0
        pose.orientation   = robot_pose_rel.orientation
        return pose

    # ── Visualisation helpers ─────────────────────────────────────────────────

    def _make_viz_marker(
        self,
        marker_id: int,
        tvec: np.ndarray,
        rvec: np.ndarray,
        stamp: Time,
    ) -> Marker:
        """Build a CUBE Marker for rviz2 at the detected marker location."""
        m = Marker()
        m.header.stamp    = stamp
        m.header.frame_id = "camera"
        m.ns              = "aruco"
        m.id              = marker_id
        m.type            = Marker.CUBE
        m.action          = Marker.ADD
        m.scale           = Vector3(
            x=self._marker_length,
            y=self._marker_length,
            z=0.005,
        )
        m.color = ColorRGBA(
            r=0.0,
            g=1.0 if marker_id == self._robot_id else 0.0,
            b=1.0 if marker_id != self._robot_id else 0.0,
            a=0.8,
        )
        m.pose = _rvec_tvec_to_pose(rvec, tvec)
        m.lifetime.sec    = 1
        m.lifetime.nanosec = 0
        return m

    def _publish_debug(self, frame: np.ndarray, header) -> None:
        """Encode annotated frame as JPEG and publish."""
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        out = CompressedImage()
        out.header = header
        out.format = "jpeg"
        out.data   = buf.tobytes()
        self._debug_pub.publish(out)

    @staticmethod
    def _ros_time_from_header(stamp) -> Time:
        t = Time()
        t.sec     = stamp.sec
        t.nanosec = stamp.nanosec
        return t


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
