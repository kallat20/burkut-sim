#!/bin/bash
# Burkut-Sim çalışma ortamını aktive eder
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
    echo "[burkut-sim] workspace aktif: $WS_DIR"
else
    echo "[burkut-sim] henüz derlenmemiş, önce 'colcon build' çalıştır"
fi

export PX4_DIR="$HOME/PX4-Autopilot"
export GZ_SIM_RESOURCE_PATH="$WS_DIR/src/burkut_worlds/worlds:$GZ_SIM_RESOURCE_PATH"
export PATH="$PATH:$HOME/PX4-Autopilot/Micro-XRCE-DDS-Agent/build"

echo "[burkut-sim] PX4_DIR=$PX4_DIR"
