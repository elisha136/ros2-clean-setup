#!/usr/bin/env bash
#
# ROS 2 Humble clean setup for Ubuntu 22.04 (jammy)
# Idempotent: safe to re-run. Designed to avoid permanently polluting your login shell.
# Usage:
#   bash setup_ros2_humble_ubuntu22.sh
#
# Common heredoc pitfall note:
#   If you ever see a secondary shell prompt '>' appearing while pasting this file manually,
#   it means a heredoc terminator (EOF) was not included. This version keeps heredocs minimal
#   and properly terminated.

set -euo pipefail

ROSDISTRO=humble
REQUIRED_CODENAME=jammy

log() { printf "\n=== %s ===\n" "$*"; }
step() { printf "[*] %s\n" "$*"; }

trap 'echo "[ERROR] Script aborted (line $LINENO)." >&2' ERR

log "ROS 2 $ROSDISTRO setup for Ubuntu 22.04 ($REQUIRED_CODENAME)"

# 0) Verify Ubuntu version
source /etc/os-release
if [[ "${UBUNTU_CODENAME:-}" != "$REQUIRED_CODENAME" ]]; then
  echo "This script is for Ubuntu 22.04 ($REQUIRED_CODENAME). Detected: ${UBUNTU_CODENAME:-unknown}" >&2
  exit 1
fi

# 1) Essentials
step "Installing base packages"
sudo apt update
sudo apt install -y curl gnupg lsb-release software-properties-common build-essential cmake git

# 2) Add ROS 2 apt repo & key (idempotent)
step "Adding ROS 2 apt repository"
sudo add-apt-repository universe -y || true
sudo mkdir -p /usr/share/keyrings
if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $REQUIRED_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# 3) Install ROS 2 desktop + dev tools
step "Installing ROS 2 $ROSDISTRO desktop & dev tools"
sudo apt install -y ros-$ROSDISTRO-desktop \
  python3-rosdep python3-colcon-common-extensions python3-vcstool
sudo rosdep init 2>/dev/null || true
rosdep update

# 4) Visualization / extras
step "Installing visualization & example tools"
sudo apt install -y \
  ros-$ROSDISTRO-rqt \
  ros-$ROSDISTRO-rqt-common-plugins \
  ros-$ROSDISTRO-rqt-graph \
  ros-$ROSDISTRO-rqt-image-view \
  ros-$ROSDISTRO-image-tools \
  ros-$ROSDISTRO-turtlesim
sudo apt install -y ros-$ROSDISTRO-plotjuggler-ros || true

# 5) Create an empty workspace (first build optional)
step "Creating ~/ros2_ws (if missing)"
mkdir -p "$HOME/ros2_ws/src"
if [[ ! -d "$HOME/ros2_ws/build" ]]; then
  ( cd "$HOME/ros2_ws" && colcon build || true )
fi

# 6) Helper functions in ~/.bashrc (only add once)
step "Ensuring helper functions exist in ~/.bashrc"
BASHRC="$HOME/.bashrc"
add_if_missing() {
  local marker="$1"; shift
  local block="$1"
  grep -Fq "$marker" "$BASHRC" 2>/dev/null || {
    printf "\n%s\n%s\n" "$marker" "$block" >> "$BASHRC"
    echo "  - Added: $marker"
  }
}

ros2shell_block=$'# Open a subshell with ROS 2 Humble + your workspace sourced (no global pollution)\nros2shell() {\n  source /opt/ros/humble/setup.bash\n  [ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"\n  echo "ROS 2 environment loaded (exit shell to leave)"\n}'
add_if_missing "# >>> ros2shell helper >>>" "$ros2shell_block"

ros2ws_block=$'# Load ROS 2 + a specific workspace into the current shell (defaults to ~/ros2_ws)\nros2ws () {\n  local wsdir="${1:-$HOME/ros2_ws}"\n  source /opt/ros/humble/setup.bash\n  if [ -f "$wsdir/install/setup.bash" ]; then\n    source "$wsdir/install/setup.bash"\n    echo "Loaded ROS 2 Humble with workspace: $wsdir"\n  else\n    echo "Workspace not built yet: $wsdir"\n    echo "Run: cd \"$wsdir\" && colcon build"\n  fi\n}'
add_if_missing "# >>> ros2ws helper >>>" "$ros2ws_block"

# 7) Dev clean script (heredoc properly closed with EOF)
step "Installing ~/bin/dev-clean.sh"
mkdir -p "$HOME/bin"
cat > "$HOME/bin/dev-clean.sh" <<'EOF'
#!/usr/bin/env bash
set -e
echo "[apt] cleaning package cache…"; sudo apt clean
echo "[pip] purging cache…"; python3 -m pip cache purge 2>/dev/null || true
echo "[npm] cleaning cache…"; command -v npm >/dev/null && npm cache clean --force || true
echo "[conda] cleaning caches…"; command -v conda >/dev/null && conda clean -a -y || true
echo "[ROS] removing old logs…"; rm -rf ~/.ros/log/* 2>/dev/null || true
if [ -d "$HOME/ros2_ws" ]; then
  echo "[colcon] cleaning build/install/log…"
  rm -rf "$HOME/ros2_ws"/{build,install,log}
fi
echo "Done."
EOF
chmod +x "$HOME/bin/dev-clean.sh"

# 8) Light system hygiene
step "Applying light system hygiene"
sudo journalctl --vacuum-size=200M || true
echo 'APT::Periodic::AutocleanInterval "7";' | sudo tee /etc/apt/apt.conf.d/10periodic > /dev/null

log "All done. Open a new terminal or: source ~/.bashrc"
