#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="burkut_sim"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

SIM_CMD="source '$REPO_ROOT/tools/activate.sh' && ros2 launch burkut_bringup main.launch.xml"
DDS_CMD="MicroXRCEAgent udp4 -p 8888"
PX4_CMD="source '$REPO_ROOT/tools/activate.sh' \
  && cd '$PX4_DIR' \
  && export PX4_GZ_STANDALONE=1 \
  && export PX4_SYS_AUTOSTART=4008 \
  && export PX4_GZ_WORLD=test \
  && export PX4_GZ_MODEL_NAME=advanced_plane \
  && ./build/px4_sitl_default/bin/px4"
HEALTH_CMD="source '$REPO_ROOT/tools/activate.sh' && set +e && while true; do '$REPO_ROOT/tools/check_health.sh' || true; sleep 5; done"
SHELL_CMD="source '$REPO_ROOT/tools/activate.sh' && cd '$REPO_ROOT' && exec bash -i"

if [ -n "${TMUX-}" ]; then
  echo "[run_sim] Zaten tmux içindesin. Çıkıp yeniden çalıştır ya da:"
  echo "  tmux attach -t $SESSION"
  exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "[run_sim] Session zaten var: $SESSION (attach ediyorum)"
  exec tmux attach -t "$SESSION"
fi

# Core (sol üst): Gazebo + ROS 2 launch
tmux new-session -d -s "$SESSION" -n "Core" -c "$REPO_ROOT" \
  "bash -c \"$SIM_CMD; echo '[sim] çıktı - ENTER ile kapat'; read\""
tmux set-option -t "$SESSION" remain-on-exit on

# Sağ üst: DDS agent
tmux split-window -h -t "$SESSION:0" \
  "bash -c \"$DDS_CMD; echo '[dds] çıktı - ENTER ile kapat'; read\""

# Sağ alt: PX4 SITL (Gazebo'nun ayağa kalkması için 4sn bekle)
tmux split-window -v -t "$SESSION:0.1" \
  "bash -c \"sleep 4 && $PX4_CMD; echo '[px4] çıktı - ENTER ile kapat'; read\""

# Sol alt: interaktif shell
tmux split-window -v -t "$SESSION:0.0" \
  "bash -c \"$SHELL_CMD\""

tmux select-layout -t "$SESSION:0" tiled

# Window 2: Health monitor
tmux new-window -t "$SESSION:1" -n "Health" \
  "bash -c \"$HEALTH_CMD\""

tmux select-window -t "$SESSION:0"
exec tmux attach -t "$SESSION"

<<'EXIT'
# Session davranışı
tmux set-option -t "$SESSION" remain-on-exit off
tmux set-option -t "$SESSION" destroy-unattached on

# Core main pane'i oluştur (ilk pane)
tmux new-session -d -s "$SESSION" -n "Core" -c "$REPO_ROOT" "bash -c \"$SIM_CMD\""

# Core pane id'sini al
CORE_PANE_ID="$(tmux display-message -p -t "$SESSION:0.0" "#{pane_id}")"

# Core pane kapanırsa tüm session'ı kapat
tmux set-hook -t "$SESSION" pane-exited \
  "if -F '#{==:#{hook_pane},$CORE_PANE_ID}' 'kill-session -t $SESSION'"
EXIT
