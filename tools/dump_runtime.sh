#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/tools/activate.sh"

OUT_DIR="$REPO_ROOT/runs/$(date +%F-%H-%M-%S)"
mkdir -p "$OUT_DIR"

{
  echo "timestamp=$(date -Iseconds)"
  echo "git=$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo none)"
  echo "git_branch=$(git -C "$REPO_ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null || echo none)"
  echo "ros_distro=${ROS_DISTRO:-unknown}"
  echo "px4_dir=${PX4_DIR:-unknown}"
} > "$OUT_DIR/meta.txt"

env | sort                                         > "$OUT_DIR/env.txt"              2>&1 || true
ros2 node list                                     > "$OUT_DIR/ros2_nodes.txt"       2>&1 || true
ros2 topic list                                    > "$OUT_DIR/ros2_topics.txt"      2>&1 || true
ros2 topic echo /clock --once                      > "$OUT_DIR/clock_once.yaml"      2>&1 || true
ros2 topic echo /fmu/out/vehicle_status --once     > "$OUT_DIR/vehicle_status.yaml"  2>&1 || true
ros2 topic echo /burkut/obstacles --once           > "$OUT_DIR/obstacles_once.yaml"  2>&1 || true
gz topic -l                                        > "$OUT_DIR/gz_topics.txt"        2>&1 || true

echo "[dump_runtime] Kaydedildi: $OUT_DIR"
