#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/tools/activate.sh"

t() { timeout 3 bash -c "$*" || true; }

echo "== NODES =="
t "ros2 node list | sort"

echo "== PX4 TOPICS =="
t "ros2 topic list | sort | grep -E 'fmu|vehicle_status|vehicle_local_pos' || echo '[!] PX4 topic yok — DDS agent çalışıyor mu?'"

echo "== CLOCK =="
t "ros2 topic echo /clock --once"

echo "== VEHICLE STATUS =="
t "ros2 topic echo /fmu/out/vehicle_status --once >/dev/null && echo OK || echo '[!] vehicle_status yok'"

echo "== OBSTACLES TOPIC =="
t "ros2 topic echo /burkut/obstacles --once >/dev/null && echo OK || echo '[!] /burkut/obstacles yok — perception node çalışıyor mu?'"

echo "== WAYPOINTS TOPIC =="
t "ros2 topic echo /burkut/waypoints --once >/dev/null && echo OK || echo '[!] /burkut/waypoints yok — planning node çalışıyor mu?'"

echo "== GZ TOPICS =="
t "gz topic -l | grep -E 'camera|lidar|imu' | head -10 || true"
