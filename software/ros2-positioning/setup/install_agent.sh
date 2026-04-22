#!/usr/bin/env bash
# =============================================================================
# install_agent.sh — micro-ROS Agent setup for the Sesame Robot
# =============================================================================
#
# The micro-ROS Agent is a lightweight UDP bridge that translates between the
# robot's micro-XRCE-DDS protocol and the full ROS 2 DDS graph on the host.
# Every ROS 2 topic published or subscribed by the ESP32-CAM firmware becomes
# a regular ROS 2 topic on the host after the agent is running.
#
# Prerequisites
# -------------
#   • Ubuntu 22.04+ or equivalent with ROS 2 Humble (or Iron/Jazzy)
#   • Docker (Option A) OR a colcon workspace (Option B)
#   • ESP32-CAM flashed with sesame-firmware-ros and connected to WiFi
#
# Usage
# -----
#   chmod +x install_agent.sh
#   ./install_agent.sh
#
# Then choose an option from the menu.
# =============================================================================
set -euo pipefail

AGENT_PORT="${AGENT_PORT:-8888}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# ── Colour helpers ─────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()    { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()    { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error()   { echo -e "${RED}[ERROR]${NC} $*" >&2; }
section() { echo -e "\n${GREEN}══ $* ══${NC}"; }

# ── Menu ───────────────────────────────────────────────────────────────────────
section "micro-ROS Agent Setup for Sesame Robot"
echo "Choose an installation method:"
echo "  1) Docker (recommended — no ROS installation needed)"
echo "  2) Native (requires ROS 2 ${ROS_DISTRO} to be sourced)"
echo "  3) Show manual run commands only"
echo ""
read -rp "Enter choice [1-3]: " CHOICE

case "$CHOICE" in

  # ── Option A: Docker ────────────────────────────────────────────────────────
  1)
    section "Option A: Docker"

    if ! command -v docker &> /dev/null; then
      error "Docker not found.  Install from https://docs.docker.com/get-docker/"
      exit 1
    fi

    info "Pulling micro-ROS Agent Docker image..."
    docker pull microros/micro-ros-agent:"${ROS_DISTRO}"

    # Create a systemd user service so the agent auto-starts on login
    SERVICE_DIR="${HOME}/.config/systemd/user"
    SERVICE_FILE="${SERVICE_DIR}/micro-ros-agent.service"
    mkdir -p "${SERVICE_DIR}"

    cat > "${SERVICE_FILE}" <<EOF
[Unit]
Description=micro-ROS Agent for Sesame Robot
After=network-online.target docker.service
Wants=network-online.target

[Service]
ExecStart=docker run --rm --net=host \\
  microros/micro-ros-agent:${ROS_DISTRO} \\
  udp4 --port ${AGENT_PORT}
ExecStop=docker stop micro-ros-agent
Restart=on-failure
RestartSec=5s

[Install]
WantedBy=default.target
EOF

    systemctl --user daemon-reload
    systemctl --user enable micro-ros-agent.service
    systemctl --user start  micro-ros-agent.service

    info "micro-ROS Agent service installed and started."
    info "Status: systemctl --user status micro-ros-agent"
    info "Logs:   journalctl --user -u micro-ros-agent -f"

    echo ""
    info "To run manually (without the service):"
    echo "  docker run --rm --net=host \\"
    echo "    microros/micro-ros-agent:${ROS_DISTRO} \\"
    echo "    udp4 --port ${AGENT_PORT}"
    ;;

  # ── Option B: Native colcon build ───────────────────────────────────────────
  2)
    section "Option B: Native ROS 2 build"

    if [ -z "${ROS_DISTRO:-}" ]; then
      error "ROS_DISTRO is not set.  Source your ROS 2 setup.bash first:"
      echo "  source /opt/ros/humble/setup.bash"
      exit 1
    fi

    WS="${HOME}/microros_ws"
    info "Building micro-ROS Agent in ${WS} ..."

    mkdir -p "${WS}/src"
    cd "${WS}"

    # Clone micro-ROS setup if not already present
    if [ ! -f "${WS}/src/micro_ros_setup/package.xml" ]; then
      git clone \
        --branch "${ROS_DISTRO}" \
        https://github.com/micro-ROS/micro_ros_setup.git \
        "${WS}/src/micro_ros_setup"
    fi

    source "/opt/ros/${ROS_DISTRO}/setup.bash"

    # Install dependencies
    rosdep update --rosdistro "${ROS_DISTRO}"
    rosdep install --from-paths src --ignore-src -y

    # Build setup package
    colcon build --packages-select micro_ros_setup
    # shellcheck disable=SC1091
    source install/local_setup.bash

    # Download + build agent
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh

    info "Build complete.  To run the agent:"
    echo "  source ${WS}/install/local_setup.bash"
    echo "  ros2 run micro_ros_agent micro_ros_agent udp4 --port ${AGENT_PORT}"
    ;;

  # ── Option C: Print commands only ────────────────────────────────────────────
  3)
    section "Manual run commands"

    echo ""
    echo "── Docker (simplest) ────────────────────────────────────────────────"
    echo "  docker run --rm --net=host \\"
    echo "    microros/micro-ros-agent:${ROS_DISTRO} \\"
    echo "    udp4 --port ${AGENT_PORT}"
    echo ""
    echo "── Native (ROS 2 ${ROS_DISTRO} installed) ──────────────────────────"
    echo "  sudo apt install ros-${ROS_DISTRO}-micro-ros-agent"
    echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
    echo "  ros2 run micro_ros_agent micro_ros_agent udp4 --port ${AGENT_PORT}"
    echo ""
    echo "── Verify topics after ESP32-CAM connects ───────────────────────────"
    echo "  ros2 topic list"
    echo "  # Expected:"
    echo "  #   /camera/image_raw"
    echo "  #   /sesame/cmd"
    echo "  #   /sesame/pose"
    echo "  #   /sesame/status"
    echo ""
    ;;

  *)
    error "Invalid choice.  Please run the script again."
    exit 1
    ;;

esac

section "Next steps"
echo "  1. Flash the ESP32-CAM with the micro-ROS firmware:"
echo "       pio run -e esp32cam_ros -t upload"
echo ""
echo "  2. Configure WiFi + agent IP on the ESP32 (serial, 115200 baud):"
echo "       set ssid  <your-network>"
echo "       set pass  <your-password>"
echo "       set agent <this-computer-IP>"
echo "       set port  ${AGENT_PORT}"
echo "       save"
echo "       reboot"
echo ""
echo "  3. Launch the full positioning stack:"
echo "       ros2 launch sesame_aruco sesame_positioning.launch.py \\"
echo "         camera_calibration_file:=\$(pwd)/config/camera_calibration.yaml \\"
echo "         marker_map_file:=\$(pwd)/config/marker_map.yaml \\"
echo "         waypoints_file:=\$(pwd)/config/waypoints.yaml"
echo ""
echo "  4. Visualise in rviz2:"
echo "       rviz2"
echo "       # Add: MarkerArray (/aruco/markers)"
echo "       # Add: Path        (/sesame/odom_path)"
echo "       # Add: PoseStamped (/sesame/pose)"
