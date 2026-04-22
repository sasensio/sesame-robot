"""
sesame_positioning.launch.py
==============================

Launches the complete Sesame Robot positioning stack:
  1. micro-ROS Agent (UDP bridge between ROS 2 and the ESP32-CAM firmware)
  2. ArUco detection node   (/camera/image_raw → /sesame/pose)
  3. Navigation node        (/sesame/pose → /sesame/cmd)

Usage
-----
  # Basic launch (agent + detection only, no navigation):
  ros2 launch sesame_aruco sesame_positioning.launch.py

  # With navigation and custom waypoints:
  ros2 launch sesame_aruco sesame_positioning.launch.py \\
    waypoints_file:=/path/to/waypoints.yaml

  # With calibration + map:
  ros2 launch sesame_aruco sesame_positioning.launch.py \\
    camera_calibration_file:=/path/to/camera_calibration.yaml \\
    marker_map_file:=/path/to/marker_map.yaml \\
    marker_length:=0.10 \\
    robot_marker_id:=0

Launch Arguments
----------------
  agent_port              micro-ROS agent UDP port (default 8888)
  camera_calibration_file path to camera_calibration.yaml
  marker_map_file         path to marker_map.yaml
  marker_length           ArUco marker side length in metres (default 0.10)
  robot_marker_id         ID of marker on the robot (default 0)
  aruco_dict              ArUco dictionary (default DICT_4X4_50)
  publish_debug_image     publish annotated frames on /sesame/debug_image
  waypoints_file          YAML waypoints file; empty = navigation node not started
  goal_tolerance          waypoint reached threshold in metres (default 0.10)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("sesame_aruco")
    default_cal  = os.path.join(pkg_share, "config", "camera_calibration.yaml")
    default_map  = os.path.join(pkg_share, "config", "marker_map.yaml")

    # ── Declare launch arguments ──────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "agent_port", default_value="8888",
            description="micro-ROS agent UDP port"
        ),
        DeclareLaunchArgument(
            "camera_calibration_file", default_value=default_cal,
            description="Path to camera_calibration.yaml"
        ),
        DeclareLaunchArgument(
            "marker_map_file", default_value=default_map,
            description="Path to marker_map.yaml"
        ),
        DeclareLaunchArgument(
            "marker_length", default_value="0.10",
            description="Physical ArUco marker side length in metres"
        ),
        DeclareLaunchArgument(
            "robot_marker_id", default_value="0",
            description="ArUco marker ID attached to the robot"
        ),
        DeclareLaunchArgument(
            "aruco_dict", default_value="DICT_4X4_50",
            description="ArUco dictionary name"
        ),
        DeclareLaunchArgument(
            "publish_debug_image", default_value="false",
            description="Republish annotated frames on /sesame/debug_image"
        ),
        DeclareLaunchArgument(
            "waypoints_file", default_value="",
            description="YAML waypoints file; leave empty to skip navigation node"
        ),
        DeclareLaunchArgument(
            "goal_tolerance", default_value="0.10",
            description="Waypoint reached threshold in metres"
        ),
    ]

    # ── 1. micro-ROS Agent ────────────────────────────────────────────────────
    # Runs the micro-ROS Agent as a subprocess (requires micro_ros_agent to be
    # installed:  sudo apt install ros-${ROS_DISTRO}-micro-ros-agent
    # or via Docker — see setup/install_agent.sh).
    micro_ros_agent = ExecuteProcess(
        cmd=[
            "ros2", "run", "micro_ros_agent", "micro_ros_agent",
            "udp4", "--port", LaunchConfiguration("agent_port"),
        ],
        output="screen",
        name="micro_ros_agent",
    )

    # ── 2. ArUco detection node ───────────────────────────────────────────────
    aruco_node = Node(
        package="sesame_aruco",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=[{
            "camera_calibration_file": LaunchConfiguration("camera_calibration_file"),
            "marker_map_file":         LaunchConfiguration("marker_map_file"),
            "marker_length":           LaunchConfiguration("marker_length"),
            "aruco_dict":              LaunchConfiguration("aruco_dict"),
            "publish_debug_image":     LaunchConfiguration("publish_debug_image"),
            "robot_marker_id":         LaunchConfiguration("robot_marker_id"),
        }],
    )

    # ── 3. Navigation node (only when waypoints_file is non-empty) ────────────
    nav_node = Node(
        package="sesame_aruco",
        executable="navigation_node",
        name="navigation_node",
        output="screen",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "goal_tolerance": LaunchConfiguration("goal_tolerance"),
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration("waypoints_file"), "' != ''"])
        ),
    )

    hint = LogInfo(msg=(
        "Sesame positioning stack launched.\n"
        "  micro-ROS agent: udp4 port " + str(8888) + "\n"
        "  Make sure the ESP32-CAM firmware is running and connected to the "
        "same WiFi network.\n"
        "  Topics: /camera/image_raw  /sesame/pose  /sesame/cmd\n"
        "  Run 'ros2 topic list' to verify topics are available."
    ))

    return LaunchDescription(args + [micro_ros_agent, aruco_node, nav_node, hint])
