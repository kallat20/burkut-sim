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
QGC_PATH="${QGC_PATH:-$HOME/QGroundControl.AppImage}"

if [[ "${1-}" == "--stop" || "${1-}" == "stop" ]]; then
  echo "[run_sim] Tüm sim bileşenleri kapatılıyor..."

  tmux kill-session -t "$SESSION" 2>/dev/null && echo "  [ok] tmux session '$SESSION' kapatıldı" || echo "  [--] tmux session bulunamadı"

  for proc in \
    "gz sim" "gz" "gzserver" "gzclient" "ruby.*gz" \
    "px4" \
    "MicroXRCEAgent" \
    "ros2" "ros2_node" \
    "QGroundControl" \
    "check_health"
  do
    pkill -f "$proc" 2>/dev/null && echo "  [ok] '$proc' öldürüldü" || true
  done

  sleep 1

  # Hala ayakta kalanları SIGKILL ile bitir
  for proc in \
    "gz sim" "gz" "gzserver" "gzclient" \
    "px4" \
    "MicroXRCEAgent" \
    "ros2" \
    "QGroundControl"
  do
    pkill -9 -f "$proc" 2>/dev/null || true
  done

  echo "[run_sim] Temizlik tamamlandı."
  exit 0
fi

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

# Log klasörünü oluştur ve Core/DDS/PX4/Shell pane'lerini yönlendir
LOG_DIR="$REPO_ROOT/runs/$(date +%F-%H-%M-%S)"
mkdir -p "$LOG_DIR"
tmux pipe-pane -o -t "$SESSION:0.0" "cat >> '$LOG_DIR/core.log'"
tmux pipe-pane -o -t "$SESSION:0.1" "cat >> '$LOG_DIR/dds.log'"
tmux pipe-pane -o -t "$SESSION:0.2" "cat >> '$LOG_DIR/px4.log'"
tmux pipe-pane -o -t "$SESSION:0.3" "cat >> '$LOG_DIR/shell.log'"
echo "[run_sim] Loglar: $LOG_DIR"

# Window 2: Health monitor (pipe-pane window oluşturulduktan sonra)
tmux new-window -t "$SESSION:1" -n "Health" \
  "bash -c \"$HEALTH_CMD\""
tmux pipe-pane -o -t "$SESSION:1.0" "cat >> '$LOG_DIR/health.log'"

# Window 3: QGroundControl (PX4 ayağa kalksın diye 8sn bekle)
tmux new-window -t "$SESSION:2" -n "QGC" \
  "bash -c \"sleep 8 && '$QGC_PATH'; echo '[qgc] çıktı - ENTER ile kapat'; read\""
tmux pipe-pane -o -t "$SESSION:2.0" "cat >> '$LOG_DIR/qgc.log'"

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
