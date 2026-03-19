#!/usr/bin/env bash
# Usage: source tools/activate.sh
set -euo pipefail

_sourced=0
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  _sourced=1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$REPO_ROOT"

# ROS 2 Humble underlay
set +u
source /opt/ros/humble/setup.bash

# px4_msgs / px4_ros_com overlay
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
else
  echo "[activate] WARN: ~/ros2_ws/install/setup.bash bulunamadı (px4_msgs eksik)"
fi
set -u

# Burkut-sim workspace overlay
if [[ -f "$WS/install/setup.bash" ]]; then
  set +u
  source "$WS/install/setup.bash"
  set -u
else
  echo "[activate] WARN: Workspace henüz derlenmemiş."
  echo "[activate] Çözüm: cd '$WS' && colcon build"
fi

export BURKUT_SIM_ACTIVE=1
export BURKUT_SIM_ROOT="$REPO_ROOT"
export PX4_DIR="$HOME/PX4-Autopilot"
export PATH="$PATH:$HOME/PX4-Autopilot/Micro-XRCE-DDS-Agent/build"
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/src/burkut_worlds/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "[activate] OK: ROS_DISTRO=$ROS_DISTRO | ROOT=$REPO_ROOT"
