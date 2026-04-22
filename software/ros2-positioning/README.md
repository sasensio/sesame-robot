# micro-ROS + ArUco Positioning for Sesame Robot

This directory contains the host-side ROS 2 software that complements the
`esp32cam_ros` micro-ROS firmware running on the robot's ESP32-CAM.

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│  Host Computer / Raspberry Pi 5                                  │
│                                                                  │
│  ┌─────────────────┐  ┌──────────────────┐  ┌───────────────┐  │
│  │ micro-ROS Agent │  │  aruco_node      │  │navigation_node│  │
│  │ (UDP port 8888) │◄─│  /camera/image   │─►│ /sesame/pose  │  │
│  └────────┬────────┘  │  → /sesame/pose  │  │ → /sesame/cmd │  │
│           │           └──────────────────┘  └───────────────┘  │
└───────────┼─────────────────────────────────────────────────────┘
            │  WiFi UDP (micro-XRCE-DDS)
            ▼
┌───────────────────────────────────────────────────────────────────┐
│  ESP32-CAM  (firmware: esp32cam_ros)                              │
│  Pub: /camera/image_raw  /sesame/status                           │
│  Sub: /sesame/cmd  /sesame/pose                                   │
└───────────────────────────────────────────────────────────────────┘
```

### What each piece does

| Component | Role |
|-----------|------|
| `esp32cam_ros` firmware | Publishes JPEG frames + status; subscribes to motion commands and pose |
| micro-ROS Agent | Bridges ESP32 micro-XRCE-DDS ↔ host ROS 2 DDS |
| `aruco_node` | Detects ArUco markers, estimates robot world pose |
| `navigation_node` | Proportional controller: drives robot through waypoints |

---

## Prerequisites

### Hardware
- AI-Thinker ESP32-CAM (or ESP32-S3-CAM) with PCA9685 servo driver
- Host: Ubuntu 22.04+ with ROS 2 Humble (or Raspberry Pi 5 with ROS 2)
- Printed ArUco markers (one on the robot, fixed ones in the environment)
- Shared WiFi network

### Software
```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# Python dependencies for the ROS package
pip install opencv-python numpy pyyaml

# Or install via apt (recommended in ROS 2 environment)
sudo apt install python3-opencv python3-numpy python3-yaml
```

---

## Step 1: Flash the micro-ROS Firmware

```bash
# From the repo root
pio run -e esp32cam_ros -t upload
```

After flashing, open a serial terminal (115200 baud) to configure WiFi and
the agent IP:

```
set ssid  YourNetworkName
set pass  YourPassword
set agent 192.168.1.100       # IP of the computer running the micro-ROS agent
set port  8888
save
reboot
```

> **Note:** If the robot previously ran the standard firmware and WiFi
> credentials were saved, they are reused automatically (same NVS namespace).

---

## Step 2: Start the micro-ROS Agent

```bash
cd setup/
./install_agent.sh
```

Or run the agent directly with Docker:

```bash
docker run --rm --net=host \
  microros/micro-ros-agent:humble \
  udp4 --port 8888
```

Or natively (requires `ros-humble-micro-ros-agent`):

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Verify the ESP32 is connected:

```bash
ros2 topic list
# Should show: /camera/image_raw  /sesame/status  /sesame/cmd  /sesame/pose
```

---

## Step 3: Camera Calibration

Accurate pose estimation requires camera intrinsic calibration.

1. Print a 7×5 checkerboard with 25 mm squares.
2. Run the calibration script:

```bash
# Capture from /camera/image_raw while agent is running
python3 scripts/calibrate_camera.py --ros \
  --board-width 7 --board-height 5 --square-size 0.025 \
  --output config/camera_calibration.yaml
```

Move the checkerboard through 20+ diverse poses (tilt, rotate, different
distances) until the script completes.

---

## Step 4: Marker Setup

1. Print ArUco markers from `DICT_4X4_50` at a known physical size:

```python
import cv2.aruco as a, cv2
d = a.Dictionary_get(a.DICT_4X4_50)
# Marker 0 goes on the robot; 1, 2, 3 are fixed in the environment
for i in [0, 1, 2, 3]:
    img = a.drawMarker(d, i, 400)
    cv2.imwrite(f'marker_{i}.png', img)
```

2. Attach marker **ID 0** to the top of the Sesame Robot (visible from above).
3. Place markers **1, 2, 3** at known positions in the room.
4. Measure their world-frame positions (metres) and edit
   `config/marker_map.yaml`.

---

## Step 5: Build and Launch

```bash
# From the ros2-positioning directory (or your colcon workspace src/)
cd sesame_aruco/
colcon build --packages-select sesame_aruco
source install/local_setup.bash

# Launch everything (agent + detection + navigation)
ros2 launch sesame_aruco sesame_positioning.launch.py \
  camera_calibration_file:=$(pwd)/config/camera_calibration.yaml \
  marker_map_file:=$(pwd)/config/marker_map.yaml \
  marker_length:=0.10 \
  robot_marker_id:=0 \
  waypoints_file:=$(pwd)/config/waypoints.yaml
```

### Example waypoints.yaml

```yaml
waypoints:
  - x: 0.5
    y: 0.0
  - x: 0.5
    y: 0.5
  - x: 0.0
    y: 0.0
```

---

## Visualisation in rviz2

```bash
rviz2
```

Add these displays:
- **MarkerArray** → Topic: `/aruco/markers`
- **Path** → Topic: `/sesame/odom_path`
- **PoseStamped** → Topic: `/sesame/pose`
- **Image** → Topic: `/sesame/debug_image` *(enable `publish_debug_image:=true`)*

---

## Topics Reference

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/camera/image_raw` | `sensor_msgs/CompressedImage` | ESP32 → host | JPEG frames |
| `/sesame/status` | `std_msgs/String` | ESP32 → host | JSON status blob |
| `/sesame/cmd` | `std_msgs/String` | host → ESP32 | Motion command |
| `/sesame/pose` | `geometry_msgs/PoseStamped` | host → ESP32 | Estimated world pose |
| `/aruco/markers` | `visualization_msgs/MarkerArray` | host | rviz2 visualisation |
| `/sesame/odom_path` | `nav_msgs/Path` | host | Travelled path |
| `/sesame/debug_image` | `sensor_msgs/CompressedImage` | host | Annotated frames |

### Motion commands

| Command | Action |
|---------|--------|
| `forward` | Walk forward (continuous until changed) |
| `backward` | Walk backward |
| `left` | Turn left |
| `right` | Turn right |
| `rest` | Rest pose (one-shot) |
| `stand` | Stand pose + idle (one-shot) |
| `wave`, `dance`, `swim`, etc. | Animations (one-shot) |
| `stop` | Clear current command |

---

## Architecture Notes

### FreeRTOS Task Layout (ESP32)

| Task | Core | Priority | Responsibility |
|------|------|----------|----------------|
| `camera_task` | 0 | 3 | Grab JPEG frames → shared buffer |
| `ros_spin_task` | 1 | 5 | micro-ROS executor, publish camera + status |
| `movement_task` | 1 | 2 | Execute servo poses, OLED animation |

The ROS spin task has higher priority than the movement task so that incoming
commands and pose updates are processed promptly even during long animations.
Animations yield via `vTaskDelay()` inside `delayWithFace()`, allowing the
spin task to run.

### micro-ROS Reconnection

The firmware implements the standard micro-ROS agent lifecycle:

```
WAITING_AGENT → AGENT_AVAILABLE → AGENT_CONNECTED → AGENT_DISCONNECTED → WAITING_AGENT
```

The robot stops moving and waits for reconnection if the agent disappears.

### Large Message Support (Camera Frames)

The default micro-ROS transport MTU may be too small for full QVGA JPEG
frames. The included `colcon.meta` file at the repo root configures an 8 KB
MTU. To apply it, rebuild the micro-ROS static library:

```bash
pio run -e esp32cam_ros --target clean
pio run -e esp32cam_ros
```

If you still see serialisation errors, reduce the frame size in the firmware:

```cpp
// firmware/sesame-firmware-ros/sesame-firmware-ros.ino
#define CAM_FRAMESIZE  FRAMESIZE_QQVGA   // 160×120 — ~2 KB JPEG
```
