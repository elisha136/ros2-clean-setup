# ROS 2 Humble Clean Setup (Ubuntu 22.04)

This repository provides a single idempotent script to prepare a clean Ubuntu 22.04 (jammy) environment for ROS 2 Humble development without permanently polluting your default shell environment.

## Script
`setup_ros2_humble_ubuntu22.sh`

Run it (safe to re-run):
```bash
bash setup_ros2_humble_ubuntu22.sh
```

What it does:
1. Verifies you are on Ubuntu 22.04 (jammy).
2. Installs essential build & developer packages.
3. Adds the official ROS 2 APT repository & key.
4. Installs ROS 2 Humble desktop and core dev tools (colcon, rosdep, vcstool).
5. Installs useful visualization & example tools (rqt suite, turtlesim, PlotJuggler, image tools).
6. Creates an empty workspace at `~/ros2_ws` and performs an initial (tolerant) build.
7. Adds two helper shell functions to `~/.bashrc` if not already present.
8. Installs a `dev-clean.sh` utility script in `~/bin` to reclaim disk space & clean build artifacts.
9. Applies light system hygiene (journal size, APT autoclean interval).

## Helper Functions
After running the script and opening a new terminal:

- `ros2shell`
  Opens a subshell with ROS 2 Humble and `~/ros2_ws` (if built) sourced. Exit that shell to leave the ROS 2 environment.

- `ros2ws [optional_workspace_path]`
  Sources ROS 2 Humble plus the specified (or default `~/ros2_ws`) workspace into the *current* shell.

## Workspace Tips
Create a sample package:
```bash
ros2shell   # or: ros2ws
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake demo_example
colcon build
ros2 run demo_example <node_name>
```

If you add packages, re-run:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

## Cleaning Up
Use the provided cleaner:
```bash
~/bin/dev-clean.sh
```
This removes apt/pip/npm/conda caches (if tools are installed), deletes old ROS logs, and purges `build/ install/ log/` in `~/ros2_ws`.

## Common Issues
| Symptom | Cause | Fix |
|---------|-------|-----|
| `>` secondary prompt while pasting | Unclosed heredoc in a manual paste | Use the provided complete script or ensure the terminating `EOF`/`EOF'` line is included |
| `rosdep init` warning already initialized | Re-run of script | Ignore (script is idempotent) |
| Missing workspace overlay | You haven't built yet | `cd ~/ros2_ws && colcon build` then re-run `ros2shell` or `ros2ws` |

## Re-running Safely
The script is idempotent: re-running will refresh apt metadata, ensure functions exist, and skip already-created artifacts where appropriate.

## Minimal One-Liner (already done by the script)
```bash
bash setup_ros2_humble_ubuntu22.sh
```

## Contributing
Feel free to open issues or PRs for improvements (additional tools, robustness tweaks, etc.).

## License
No license file yet. If you plan to publish / share broadly, consider adding a permissive license (e.g., MIT or Apache-2.0).

---
Happy robotics hacking!
