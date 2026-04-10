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
- `ainex_behavior` — marathon behavior tree via py_trees (added Mar 9 2026); publishes `py_trees_msgs/BehaviourTree` on `~log/tree` for rqt_py_trees + ROSA (added Mar 25 2026)
  - **Marathon BT restructured (Apr 10 2026)**: layered package under `marathon/`; entry: `app/marathon_bt_node.py`; tree: `tree/marathon_bt.py`; actions/conditions: `behaviours/`; algorithm: `algorithms/visual_patrol.py` (pure, no manager); semantics: `semantics/semantic_facade.py`; facade: `comm/comm_facade.py`; infra: `infra/{bb_ros_bridge,bt_exec_controller,tree_publisher,infra_manifest}.py`; CI: `check_imports.py` (AST-based); imports via `rospkg.RosPack().get_path('ainex_behavior')` (not `__file__`); `ManagerProxy` removed from business path; `ros_topic_received` → `ros_in` with `adapter` field; `infra_comm_manifest_lastrun.json` written at startup
  - **BT observability system** (added Apr 5 2026): `bt_observability/` shared module at `ainex_behavior/bt_observability/`; topics `/bt_debug` + `/bt_ros_comm_debug` (JSON String); logs to `marathon/log/` (4 JSONL files: last-run full + recent rolling); `DebugEventLogger` + `BTDebugVisitor` + `ROSCommTracer`; logger=None is zero-cost fallback; `marathon/log/` excluded from git
    - **Rolling files** always newest-first (reversed on each `end_tick()` flush). **Lastrun files** are append-only (oldest-first on disk during run); reversed to newest-first once in `close()` on clean shutdown. Crash leaves lastrun oldest-first.
    - **`max_rolling_ticks` = 30** (recent files keep last 30 ticks)
    - `read_bt_obs` (unified tool) re-reverses lastrun lines before passing to summarizers (which expect oldest-first)
  - `FindLineHeadSweep` (head-sweep mode): two-phase SWEEP→ALIGN state machine; writes `/head_pan_pos` BB key; `IsHeadCentered` gates `FollowLine` until body aligned
    - `initialise()` called every tick (memory=False Selector) — use `_fresh_start` bool flag (not value check) to guard direction reset; only set True in `__init__` + on ALIGN SUCCESS
    - `gait_manager.disable()` in `initialise()` guarded by `line_data is None` — prevents gait stop/restart at 30 Hz during ALIGN turns
    - ALIGN turn uses `set_step()` with go/turn gait config switch at threshold=2°; SWEEP_STEP must not be a factor of 200 (half-range) to avoid exact center hit
    - Full tick-interruption fix pattern documented in `ainex_bt_edu.md`
  - **BT exec controller** (`bt_exec_controller.py`, added Mar 25 2026): RUN/PAUSE/STEP mode gate inside `self.start`; services `~bt/run`, `~bt/pause`, `~bt/step` (all `std_srvs/Empty`); topic `~bt/mode` (String latched); launch param `~bt_mode` (default `'run'`); `~stop`/`~start` unchanged
- `ainex_bt_edu` — educational BT framework (added Mar 16 2026), details in `ainex_bt_edu.md`
- `rosa-agent` container added (Mar 12 2026): NASA JPL ROSA read-only diagnostic agent at `/home/pi/docker/rosa-agent/`; docker-compose at `/home/pi/docker/docker-compose.yml`; **host networking** (`network_mode: host`), `ROS_MASTER_URI=http://127.0.0.1:11311`; no bridge network needed; **11 tools** including `get_bt_status` (BT monitor, Mar 25), `read_last_run_summary` (log summarizer, Mar 14), `read_bt_obs` (BT observability JSONL, Apr 5; unified auto-routing); mounts ainex ros_log + marathon/log (`/opt/ainex_bt_log`) read-only; full details: **`ainex_rosa_agent.md`**
  - **`get_bt_status`** reads `tick_id` + `camera_lost_count` from BB bridge topics; `_BB_KEYS = ['tick_id', 'robot_state', 'line_data', 'last_line_x', 'camera_lost_count']`; no JSONL file read
  - **`read_bt_obs` is in PRIORITY 1** of `about_your_capabilities` prompt — auto-selects _recent pair (live) if BT node running, _lastrun pair (full session) if not; LLM routes tick_id / per-tick decision queries to this tool
  - **BB bridge** (`bb_ros_bridge.py`) publishes 5 keys: `robot_state`, `line_data`, `last_line_x`, `camera_lost_count`, `tick_id` → `/bt/marathon/bb/*` (10 Hz)
  - **`line_lost_count` renamed → `camera_lost_count`** everywhere (ROS1 + ROS2): counts consecutive camera frames (30 Hz) without line detection — distinct from BT `tick_id` (15 Hz iteration counter); `tick_id` written to BB inside `if should_tick():` block
- **ROS2 migration complete (Stage 2, Apr 2 2026)**: `ainex2` service (ROS Humble, host network, privileged, `group_add: ["1001"]` for GPIO); bringup: `ros2 launch ainex_bringup bringup.launch.py [use_imu:=true] [use_camera:=true]`; **always source `setup.zsh` not `setup.bash` in zsh**; full details in **`ainex_ros2.md`**
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
- `py_trees` 2.1.6 installed via pip (NOT apt); `py_trees_msgs`, `uuid_msgs`, `unique_id`, `rqt_py_trees` built from source in workspace (cloned to `ros_ws_src/`)
- ROS Noetic apt repo arm64 packages are **gone** (404) — must build from source
- **Do NOT install `ros-noetic-py-trees-ros`** — it requires py_trees 0.7.x, incompatible with 2.1.6 used by marathon BT and ainex_bt_edu
- After container recreation, reinstall: `apt-get install -y python3-pygraphviz python3-termcolor`
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
- `ainex_rosa_agent.md` — ROSA agent integration: directory layout, tool table (11 tools), Dockerfile notes, LLM config, build/run commands
- `ainex_bt_observability.md` — BT observability system: 4 JSONL files (paths, event schemas for both streams), module structure (DebugEventLogger/BTDebugVisitor/ROSCommTracer), marathon integration points, ROSA tool (`read_bt_obs` — unified auto-routing), ROS topics
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
- Excludes: `__pycache__`, `.pyc`, `.zip`, `.7z`, `.bag`, `.so`, `build/`, `core` dumps, `*_this_session.md`, `*.egg-info/`, credentials, BT session logs
- **All new files in `ros_ws_src/` should be committed** — new files show as `??` (untracked, not ignored); run `git add docker/ros_ws_src/` + commit + push when wrapping up sessions or adding new source files
