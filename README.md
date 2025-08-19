# ROS 2 Humble Clean Setup (Ubuntu 22.04)

A reproducible, beginner-friendly ROS 2 Humble setup on Ubuntu 22.04 (Jammy) that avoids environment interference and cache headaches.

## What this gives you
- **ROS 2 Humble (desktop)**: `rviz2`, `rqt`, demos, common tools
- **Dev tools**: `colcon`, `vcstool`, `rosdep`
- **Visualization extras**: `rqt-*`, `rqt-image-view`, `image-tools`, `plotjuggler-ros`, `turtlesim`
- **Clean workspace**: `~/ros2_ws`
- **No global sourcing** — environment stays clean
- **Helpers** in `~/.bashrc`: `ros2shell` and `ros2ws`
- **Hygiene**: journald limit, weekly apt autoclean, `~/bin/dev-clean.sh`

---

## Quick start
```bash
# Clone and run
git clone https://github.com/<YOUR_GH_USERNAME>/ros2-clean-setup.git
cd ros2-clean-setup
bash setup_ros2_humble_ubuntu22.sh

# Open a new terminal or reload shell
source ~/.bashrc
```

### Daily usage (cheat sheet)
Start a ROS session (default workspace):
```bash
ros2shell
```
Leave the ROS environment (close terminal or):
```bash
exec bash -l
```
Create a second workspace (no interference):
```bash
mkdir -p ~/ros2_ws_nav/src
cd ~/ros2_ws_nav
colcon build
```
Use that workspace in a fresh terminal:
```bash
ros2ws ~/ros2_ws_nav
```
Build and run examples:
```bash
cd ~/ros2_ws
colcon build
ros2 run demo_nodes_cpp talker
rviz2
```
Verify ROS is loaded:
```bash
echo $ROS_DISTRO                      # -> humble
echo $AMENT_PREFIX_PATH | tr ':' '\n' # top entry = active workspace
```

Keep caches tidy — run the safe cleaner anytime:
```bash
~/bin/dev-clean.sh
```
It performs: apt clean, pip cache purge, optional npm & conda cache clean, removes `~/.ros/log/*`, and clears `build/ install/ log/` in `~/ros2_ws`.

### One-time system limits (applied by the script)
* Journald capped at 200MB
* Weekly apt autoclean

### Notes to avoid interference
* Do not globally source ROS in `~/.bashrc`.
* Use `ros2shell` (default ws) or `ros2ws <path>` (specific ws).
* One workspace per terminal.
* Keep Conda off by default; use per-project envs if needed (not for ROS itself).

### Uninstall / reset (optional)
```bash
sudo apt purge -y 'ros-*' && sudo apt autoremove -y
rm -rf ~/ros2_ws
```

---
Happy robotics hacking!
