# Claude Memory - Ainex Robot (Raspberry Pi)

## Projects

### Ainex Humanoid Robot
- Robot runs ROS Noetic inside Docker container named `ainex`
- Docker mounts: `/home/pi/docker/src` → `/home/ubuntu/share/src`, `/home/pi/docker/ros_ws_src` → `/home/ubuntu/ros_ws/src`
- ROS source editable on host: `/home/pi/docker/ros_ws_src/` (mounted into container)
- Container image for recreation: `ainex-backup:20260308`
- Main launch: `roslaunch ainex_bringup bringup.launch`
- **17 ROS packages** — full inventory in `ainex_architecture.md`
- **24 DOF humanoid**: 12 leg + 10 arm + 2 head servos (RS485 via STM32, /dev/ttyAMA0)
- Key walking service: `rosservice call /walking/command "command: 'enable_control'"` then `rosservice call /walking/command "command: 'start'"` — **NOT `/ainex/set_walking_command`** (deprecated, does not exist)
- Servo IDs 1–12 legs (interleaved L/R, odd=L even=R, ankle→hip order), 13–22 arms, 23–24 head — **canonical table in `ainex_truth_spec.md`** (`ainex_architecture.md` servo table is WRONG)
- Gait config: `ainex_driver/ainex_kinematics/config/walking_param.yaml`
- Missing in repo (need to create): `ainex_control` (safety/watchdog), `ainex_perception` (unified vision), `ainex_navigation` (gait commander)
- `ainex_behavior` now exists (marathon behavior tree via py_trees, added Mar 9 2026)
- Competition code: `hurocup2025/` (marathon, penalty kick, sprint, triple jump, weight lift)
- Simulation: `ainex_simulations/ainex_gazebo/` + `ainex_description/`, flag `gazebo_sim:=true`

### Ainex Controller GUI
- Source: `/home/ubuntu/software/ainex_controller/main.py`
- **Manual button work in progress** — see `ainex_manual_button.md`

- rqt config: `/home/ubuntu/.config/ros.org/rqt_gui.ini` — perspectives set via Python QSettings, details in `ainex_rqt_perspectives.md`

## Topic Files
- `ainex_docker_mount.md` — Docker mount setup (COMPLETED), container recreation command, optional software/ mount
- `ainex_display_fix.md` — X11/rqt display fix: DISPLAY=:1 (XWayland), LIBGL_ALWAYS_SOFTWARE=1 (Mesa v3d workaround), all modified files
- `ainex_manual_button.md` — Manual button in Ainex Controller GUI, ROS walking API, servo IDs
- `ainex_architecture.md` — Full repo inventory, package list, node table, TF tree, config locations, proposed production architecture, MVP launch sequence
- **`ainex_truth_spec.md`** — CANONICAL source of truth: topic table, service table, servo ID table (authoritative)
- `ainex_conflict_matrix.md` — All conflicts between docs, decisions, and dispositions
- `ainex_migration_map.md` — Legacy-to-canonical name mapping
- `ainex_validation_checklist.md` — 24-command acceptance checklist (30 min bringup validation)

## Git Repo
- Build system: **`catkin build`** (NOT `catkin_make` — workspace uses catkin_tools)
- Single repo at `/home/pi` (master branch) tracking ROS source + Claude config
- **GitHub remote**: `origin` → `https://github.com/xzheng2/ainex_xyz.git` (private)
- GitHub user: `xzheng2` (xzheng2@laurentian.ca)
- Auth: credential helper `store` (`~/.git-credentials`, not tracked)
- Allowlist `.gitignore`: only `docker/ros_ws_src/**` and `.claude/` memory/settings are tracked
- 411 files, excludes `__pycache__`, `.pyc`, `.zip`, `.bag`, credentials, session logs
