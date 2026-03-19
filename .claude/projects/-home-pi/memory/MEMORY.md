# Claude Memory - Ainex Robot (Raspberry Pi)

## Projects

### Ainex Humanoid Robot
- Robot runs ROS Noetic inside Docker container named `ainex`
- Docker mounts: `/home/pi/docker/src` → `/home/ubuntu/share/src`, `/home/pi/docker/ros_ws_src` → `/home/ubuntu/ros_ws/src`, `/home/pi/docker/ros_log` → `/home/ubuntu/.ros/log` (+ symlink `/root/.ros/log` → same)
- ROS source editable on host: `/home/pi/docker/ros_ws_src/` (mounted into container)
- Container image for recreation: `ainex-backup:20260308`
- Main launch: `roslaunch ainex_bringup bringup.launch`
- **17 ROS packages** — full inventory in `ainex_architecture.md`
- **24 DOF humanoid**: 12 leg + 10 arm + 2 head servos (RS485 via STM32, /dev/ttyAMA0)
- Key walking service: `rosservice call /walking/command "command: 'enable_control'"` then `rosservice call /walking/command "command: 'start'"` — **NOT `/ainex/set_walking_command`** (deprecated, does not exist)
- Servo IDs 1–12 legs (interleaved L/R, odd=L even=R, ankle→hip order), 13–22 arms, 23–24 head — **canonical table in `ainex_truth_spec.md`** (`ainex_architecture.md` servo table is WRONG)
- Gait config: `ainex_driver/ainex_kinematics/config/walking_param.yaml`
- Missing in repo (need to create): `ainex_control` (safety/watchdog), `ainex_perception` (unified vision), `ainex_navigation` (gait commander)
- `ainex_behavior` — marathon behavior tree via py_trees (added Mar 9 2026)
- `ainex_bt_edu` — educational BT framework (added Mar 16 2026), details in `ainex_bt_edu.md`
- `rosa-agent` container added (Mar 12 2026): NASA JPL ROSA read-only diagnostic agent at `/home/pi/docker/rosa-agent/`; docker-compose at `/home/pi/docker/docker-compose.yml`; **host networking** (`network_mode: host`), `ROS_MASTER_URI=http://127.0.0.1:11311`; no bridge network needed; **8 tools** including `read_last_run_summary` (log summarizer, added Mar 14); mounts ainex ros_log read-only
- Competition code: `hurocup2025/` (marathon, penalty kick, sprint, triple jump, weight lift)
- Simulation: `ainex_simulations/ainex_gazebo/` + `ainex_description/`, flag `gazebo_sim:=true`

### Ainex Controller GUI
- Source: `/home/ubuntu/software/ainex_controller/main.py`
- **Manual button work in progress** — see `ainex_manual_button.md`

- rqt config: `/home/ubuntu/.config/ros.org/rqt_gui.ini` — perspectives set via Python QSettings, details in `ainex_rqt_perspectives.md`

## ROS System
- `/opt/ros/noetic` is the ROS Noetic base install inside the container (295 packages)
- All `import rospy`, `from std_msgs/sensor_msgs/geometry_msgs.msg import ...` resolve to `/opt/ros/noetic/lib/python3/dist-packages/`
- `usb_cam_node`, `apriltag_ros`, `image_proc` are compiled C++ binaries living in `/opt/ros/noetic/lib/`, not in the workspace
- `py_trees` is NOT in noetic — verify with `docker exec ainex python3 -c "import py_trees"`
- Full details: **`ainex_ros_noetic.md`**

## Servo Feedback
- Service: `/ros_robot_controller/bus_servo/get_state` — query per-servo: position, voltage (mV), temperature (°C), offset, limits, torque state
- Full details + example cmds + bug fixes: **`ainex_servo_feedback.md`**

## YOLO Vision
- YOLOv8n runs on **host** (not container): ultralytics 8.4.22, NCNN backend, **6.7 FPS @ 320x320**
- Host↔ROS bridge: `roslibpy` (host) → `rosbridge_server` (container, port 9090)
- Full details: **`ainex_yolo.md`**

## Topic Files
- `ainex_docker_mount.md` — Docker mount setup (COMPLETED), container recreation command (includes ros_log mount), optional software/ mount
- `ainex_display_fix.md` — X11/rqt display fix: DISPLAY=:1 (XWayland), LIBGL_ALWAYS_SOFTWARE=1 (Mesa v3d workaround), all modified files
- `ainex_manual_button.md` — Manual button in Ainex Controller GUI, ROS walking API, servo IDs
- `ainex_architecture.md` — Full repo inventory, package list, node table, TF tree, config locations, proposed production architecture, MVP launch sequence
- **`ainex_truth_spec.md`** — CANONICAL source of truth: topic table, service table, servo ID table (authoritative)
- `ainex_rosa_agent.md` — ROSA agent integration: directory layout, tool table (8 tools incl. log summarizer), Dockerfile notes, LLM config, build/run commands
- `ainex_conflict_matrix.md` — All conflicts between docs, decisions, and dispositions
- `ainex_migration_map.md` — Legacy-to-canonical name mapping
- `ainex_validation_checklist.md` — 24-command acceptance checklist (30 min bringup validation)
- `ainex_bt_edu.md` — ainex_bt_edu package: structure, 36 nodes (L1:11/L2:14/L3:11), BB keys, ROSA integration

## Git Repo
- Build system: **`catkin build`** (NOT `catkin_make` — workspace uses catkin_tools)
- Single repo at `/home/pi` (master branch) tracking ROS source + Claude config
- **GitHub remote**: `origin` → `https://github.com/xzheng2/ainex_xyz.git` (private)
- GitHub user: `xzheng2` (xzheng2@laurentian.ca)
- Auth: credential helper `store` (`~/.git-credentials`, not tracked)
- Allowlist `.gitignore`: only `docker/ros_ws_src/**` and `.claude/` memory/settings are tracked
- 411 files, excludes `__pycache__`, `.pyc`, `.zip`, `.bag`, credentials, session logs
