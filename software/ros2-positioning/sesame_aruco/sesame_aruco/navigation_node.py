"""
sesame_aruco.navigation_node
=============================

Proportional heading + distance feedback controller for the Sesame Robot.
Receives the robot's world pose from the ArUco detection node and commands
the robot to navigate through a sequence of 2-D waypoints.

Topics
------
Subscribes:
  /sesame/pose        geometry_msgs/PoseStamped   robot pose from ArUco node

Publishes:
  /sesame/cmd         std_msgs/String             motion command to robot
  /sesame/odom_path   nav_msgs/Path               visualisation in rviz2

Services
--------
  /sesame/go_to       sesame_aruco/GoTo            load new waypoint list
  (see GoTo.srv — single float64[] waypoints field, pairs of x,y values)

  NOTE: Because custom service interfaces require a full ROS 2 package build,
  a simpler alternative is provided: pass a YAML waypoints file via the
  ``waypoints_file`` parameter and the node will execute it automatically.

Parameters
----------
  waypoints_file     (str)   path to a YAML file with waypoints (optional)
  goal_tolerance     (float) distance to waypoint at which it is considered
                             reached, in metres (default 0.10)
  heading_kp         (float) proportional gain for heading controller (default 1.5)
  distance_kp        (float) proportional gain for distance controller (default 1.0)
  max_heading_err    (float) heading error (deg) above which the robot turns
                             in-place before translating (default 25.0)
  cmd_rate_hz        (float) how often to re-evaluate and publish commands
                             (default 5.0 Hz)
  pose_timeout_sec   (float) stop robot if no pose received for this many
                             seconds (default 2.0)

Waypoints File Format (waypoints.yaml)
---------------------------------------
  waypoints:
    - x: 0.5
      y: 0.0
    - x: 0.5
      y: 0.5
    - x: 0.0
      y: 0.0

Commands sent to /sesame/cmd
-----------------------------
  "forward"   — move forward one step
  "left"      — turn left one step
  "right"     — turn right one step
  "stop"      — stop all motion
"""

import math
import os
from typing import List, Optional, Tuple

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quat_to_yaw(q) -> float:
    """Extract yaw (radians) from a geometry_msgs Quaternion."""
    # yaw = atan2(2(wz + xy), 1 - 2(y² + z²))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed difference between two angles (radians), result in [-π, π]."""
    diff = a - b
    while diff >  math.pi: diff -= 2.0 * math.pi
    while diff < -math.pi: diff += 2.0 * math.pi
    return diff


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class NavigationNode(Node):
    """Proportional heading + distance controller for waypoint navigation."""

    def __init__(self) -> None:
        super().__init__("navigation_node")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("waypoints_file",   "")
        self.declare_parameter("goal_tolerance",   0.10)
        self.declare_parameter("heading_kp",       1.5)
        self.declare_parameter("distance_kp",      1.0)
        self.declare_parameter("max_heading_err",  25.0)
        self.declare_parameter("cmd_rate_hz",      5.0)
        self.declare_parameter("pose_timeout_sec", 2.0)

        wp_file        = str(self.get_parameter("waypoints_file").value)
        self._goal_tol = float(self.get_parameter("goal_tolerance").value)
        self._hdg_kp   = float(self.get_parameter("heading_kp").value)
        self._dist_kp  = float(self.get_parameter("distance_kp").value)
        self._max_herr = math.radians(float(self.get_parameter("max_heading_err").value))
        rate_hz        = float(self.get_parameter("cmd_rate_hz").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout_sec").value)

        # ── State ────────────────────────────────────────────────────────────
        self._waypoints: List[Tuple[float, float]] = []
        self._current_wp_idx: int = 0
        self._latest_pose: Optional[PoseStamped] = None
        self._last_pose_time: float = 0.0
        self._active: bool = False
        self._path_msg = Path()
        self._path_msg.header.frame_id = "map"

        # ── Load waypoints from file if provided ──────────────────────────────
        if wp_file:
            self._load_waypoints(wp_file)

        # ── QoS ──────────────────────────────────────────────────────────────
        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Subscriber ───────────────────────────────────────────────────────
        self._pose_sub = self.create_subscription(
            PoseStamped, "/sesame/pose",
            self._pose_callback, qos_be
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(String, "/sesame/cmd", 10)
        self._path_pub = self.create_publisher(Path, "/sesame/odom_path", 10)

        # ── Control timer ────────────────────────────────────────────────────
        period = 1.0 / max(1.0, rate_hz)
        self._timer = self.create_timer(period, self._control_step)

        self.get_logger().info(
            f"Navigation node ready. "
            f"Waypoints={len(self._waypoints)}, "
            f"goal_tol={self._goal_tol:.3f} m, "
            f"rate={rate_hz:.1f} Hz"
        )
        if self._waypoints:
            self._active = True
            self.get_logger().info(
                f"Auto-starting navigation: first waypoint "
                f"({self._waypoints[0][0]:.2f}, {self._waypoints[0][1]:.2f})"
            )

    # ── Waypoint loading ──────────────────────────────────────────────────────

    def _load_waypoints(self, path: str) -> None:
        if not os.path.isfile(path):
            self.get_logger().warning(f"Waypoints file not found: {path}")
            return
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        for wp in data.get("waypoints", []):
            self._waypoints.append((float(wp["x"]), float(wp["y"])))
        self.get_logger().info(
            f"Loaded {len(self._waypoints)} waypoints from {path}"
        )

    def set_waypoints(self, waypoints: List[Tuple[float, float]]) -> None:
        """Replace the waypoint list and (re)start navigation."""
        self._waypoints      = list(waypoints)
        self._current_wp_idx = 0
        self._active         = len(waypoints) > 0
        self._path_msg.poses.clear()
        self.get_logger().info(f"New waypoints set: {waypoints}")

    # ── Pose subscriber ───────────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseStamped) -> None:
        self._latest_pose   = msg
        self._last_pose_time = self.get_clock().now().nanoseconds * 1e-9

        # Append to rviz2 path
        self._path_msg.header.stamp = msg.header.stamp
        self._path_msg.poses.append(msg)
        if len(self._path_msg.poses) > 500:  # keep last 500 poses
            self._path_msg.poses.pop(0)
        self._path_pub.publish(self._path_msg)

    # ── Control step ─────────────────────────────────────────────────────────

    def _control_step(self) -> None:
        """Called at cmd_rate_hz. Compute and publish the next motion command."""
        # Publish path even when not navigating
        if self._path_msg.poses:
            self._path_pub.publish(self._path_msg)

        if not self._active or not self._waypoints:
            return

        # Check pose freshness
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._latest_pose is None or (now - self._last_pose_time) > self._pose_timeout:
            self._publish_cmd("stop")
            if self._latest_pose is not None:
                self.get_logger().warning(
                    "Pose timeout — stopping robot"
                )
            return

        # Current robot state
        pose = self._latest_pose.pose
        rx   = pose.position.x
        ry   = pose.position.y
        ryaw = _quat_to_yaw(pose.orientation)

        # Current waypoint
        if self._current_wp_idx >= len(self._waypoints):
            self.get_logger().info("All waypoints reached!")
            self._active = False
            self._publish_cmd("stop")
            return

        wx, wy = self._waypoints[self._current_wp_idx]
        dx     = wx - rx
        dy     = wy - ry
        dist   = math.hypot(dx, dy)

        # Reached waypoint?
        if dist < self._goal_tol:
            self.get_logger().info(
                f"Waypoint {self._current_wp_idx} reached "
                f"({wx:.2f}, {wy:.2f}). dist={dist:.3f} m"
            )
            self._current_wp_idx += 1
            if self._current_wp_idx >= len(self._waypoints):
                self.get_logger().info("All waypoints reached — stopping.")
                self._active = False
                self._publish_cmd("stop")
                return
            wx, wy = self._waypoints[self._current_wp_idx]
            dx, dy = wx - rx, wy - ry
            dist   = math.hypot(dx, dy)

        # Desired heading toward waypoint
        desired_hdg = math.atan2(dy, dx)
        hdg_err     = _angle_diff(desired_hdg, ryaw)

        # ── P-controller ─────────────────────────────────────────────────────
        #
        # If heading error is large, turn in place.
        # Otherwise move forward (with residual heading correction ignored —
        # the robot's straight-line pose corrections come from future steps).
        #
        if abs(hdg_err) > self._max_herr:
            cmd = "right" if hdg_err < 0 else "left"
        else:
            cmd = "forward"

        self._log_nav_state(rx, ry, ryaw, wx, wy, dist, hdg_err, cmd)
        self._publish_cmd(cmd)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _publish_cmd(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self._cmd_pub.publish(msg)

    def _log_nav_state(
        self,
        rx: float, ry: float, ryaw: float,
        wx: float, wy: float,
        dist: float, hdg_err: float,
        cmd: str,
    ) -> None:
        self.get_logger().debug(
            f"WP[{self._current_wp_idx}]=({wx:.2f},{wy:.2f}) "
            f"robot=({rx:.2f},{ry:.2f},yaw={math.degrees(ryaw):.1f}°) "
            f"dist={dist:.3f} m hdg_err={math.degrees(hdg_err):.1f}° "
            f"cmd={cmd}"
        )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
