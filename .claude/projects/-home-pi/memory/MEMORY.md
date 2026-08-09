# Claude Memory - Ainex Robot (Raspberry Pi)

> **Package renamed Aug 8 2026**: `xyz_bt_edu` → `xyz_bt_lib` (dir, python module, imports, launch refs, hooks `xyz_bt_lib_guard.py`/`xyz_bt_lib_pre_guard.py`, skills `/xyz-bt-lib-node` + `/xyz-bt-lib-adapter`). Older entries below saying `xyz_bt_edu`/`ainex_bt_edu` refer to the same package pre-rename; topic file `ainex_bt_edu.md` removed Aug 9 2026 (was historical; content fully superseded by xyz_bt_lib docstrings, see [[docstring-is-the-spec]]).

> **Competition events WIPED Aug 8 2026** for clean agent rebuild: all `xyz_behavior` project dirs, event launch files, hurocup2025 legacy scripts, event memories/docs deleted; git history squashed to a single clean-baseline commit. **When rebuilding events, do NOT consult old designs, the backup bundle, or any git history** — rebuild from the `xyz_bt_lib` framework only. Kept: xyz_behavior skeleton, bt_observability, ActionGroups motions (hand-tuned assets), hurocup2025/scripts/tools calibration utilities.

## Working preferences

- [Plan summary preference](plan-summary-preference.md) — show only the current round's changes for approval, not the whole plan file
- [BT steady-confirm = LatchedDwellDecorator](bt-steady-confirm-memory-true.md) — wrap the pure L1 condition in BB-backed LatchedDwellDecorator; built-in node dwell + DwellDecorator removed Aug 2026
- [Falling-edge hysteresis gap](hysteresis-falling-edge-gap.md) — old fail_dwell_ticks (1-enter/N-exit debounce) has no decorator replacement; add HysteresisDecorator only when a real tree needs it
- [Extend, don't duplicate BT nodes](bt-extend-not-new-node.md) — add a tunable config param (no-op default) to an existing node instead of a near-duplicate node
- [Docstring is the spec](docstring-is-the-spec.md) — xyz_bt_lib docs/spec.md deleted Aug 9 2026; module docstrings are the authoritative spec, never recreate a central spec doc

## Projects

- [Camera View Banding](camera-view-banding.md) — servo-24 driven upper/lower 640x640 gemini_color via gemini305_view_bridge.py + ZMQ servo state on :5555
- [Web Data Viewer](web-data-viewer.md) — /home/pi/ros_launcher/ browser dashboard on :8090 (Launch control + /viewer/ data viewer over rosbridge:9090); 8081/web_data_viewer retired & folded in

### Ainex Humanoid Robot
- Robot runs ROS Noetic inside Docker container named `ainex`
- Docker mounts: `/home/pi/docker/src` → `/home/ubuntu/share/src`, `/home/pi/docker/ros_ws_src` → `/home/ubuntu/ros_ws/src`, `/home/pi/docker/ros_log` → `/home/ubuntu/.ros/log` (+ symlink `/root/.ros/log` → same)
- ROS source editable on host: `/home/pi/docker/ros_ws_src/` (mounted into container)
- Container image for recreation: `ainex-backup:20260630` (bakes in built ROS workspace + py_trees 2.1.6 + pygraphviz/termcolor + software/lab_tool bind mount)
- Main launch: `roslaunch ainex_bringup bringup.launch`
- **17 ROS packages** — full inventory in `ainex_architecture.md`
- **24 DOF humanoid**: 12 leg + 10 arm + 2 head servos (RS485 via STM32, /dev/ttyAMA0)
- Key walking service: `rosservice call /walking/command "command: 'enable_control'"` then `... "command: 'start'"` — **NOT `/ainex/set_walking_command`** (deprecated, does not exist)
- Servo IDs 1–12 legs (interleaved L/R, odd=L even=R, ankle→hip order), 13–22 arms, 23–24 head — **canonical table in `ainex_truth_spec.md`** (`ainex_architecture.md` servo table is WRONG)
- Gait config: `ainex_driver/ainex_kinematics/config/walking_param.yaml`
- Missing in repo (need to create): `ainex_control` (safety/watchdog), `ainex_perception` (unified vision), `ainex_navigation` (gait commander)
- BT observability (input adapters, two-phase latch, JSONL logging, exec controller): full details in `ainex_bt_observability.md` — note: file paths there predate the xyz_bt_lib rename, verify against current tree before relying on them
- ROSA agent (directory layout, tools, Dockerfile, LLM config): full details in `ainex_rosa_agent.md` — same rename caveat; `read_bt_obs` described there no longer exists, superseded by `session_digest`/`bt_tick_analysis`
- **ROS2 REMOVED (Aug 8 2026)**: `ainex2` container + `ainex2:humble` image deleted, `/home/pi/docker/ros2_ws_src` deleted (was untracked — unrecoverable); robot is ROS1-only again
- Event projects + hurocup2025 legacy scripts deleted Aug 8 2026 (see wipe note at top); `action_path='/home/ubuntu/ros_ws/src/ActionGroups'` for RunAction nodes
- Simulation: `ainex_simulations/ainex_gazebo/` + `ainex_description/`, flag `gazebo_sim:=true`

### Ainex Controller GUI
- Source: `/home/ubuntu/software/ainex_controller/main.py`
- **Manual button work in progress** — see `ainex_manual_button.md`
- Container File Browser (PyQt5, inside `ainex`): `/home/ubuntu/software/file_browser.py`; desktop shortcut `/home/pi/Desktop/ainex_file_browser.desktop`
- rqt config: `/home/ubuntu/.config/ros.org/rqt_gui.ini` — perspectives set via Python QSettings

## ROS System
- `/opt/ros/noetic` is the ROS Noetic base install inside the container (295 packages)
- `py_trees` 2.1.6 installed via pip (NOT apt); `py_trees_msgs`, `uuid_msgs`, `unique_id`, `rqt_py_trees` built from source in workspace (cloned to `ros_ws_src/`)
- ROS Noetic apt repo arm64 packages are **gone** (404) — must build from source
- **Do NOT install `ros-noetic-py-trees-ros`** — requires py_trees 0.7.x, incompatible with 2.1.6 used by the BT stack
- After container recreation, reinstall: `apt-get install -y python3-pygraphviz python3-termcolor`

## Servo 24 (Head Tilt) Range Limit — May 2026
Clamped to **[280, 550]** in `ainex_kinematics/config/servo_controller.yaml` (+`angle2pulse()`) and in `ros_robot_controller_node.py` `set_bus_servo_position()` (direct bus-servo topic; lazy-loads same YAML limits). `move_head()` goes through the second path. Hardware EEPROM (0x30) not implemented by STM32 firmware.

## Servo Feedback
- Service: `/ros_robot_controller/bus_servo/get_state` — query per-servo: position, voltage (mV), temperature (°C), offset, limits, torque state

## Camera Frame Transport — /dev/shm zero-copy (Jun 16 2026)
- Gemini 305 frames move host→container via **shared memory + ZMQ notify** (was: ZMQ TCP raw-BGR, ~5 copies + 3 hops). Producer writes each frame once into a `/dev/shm` double buffer; ZMQ carries only an 18-byte frame-ready notice.
- **`--ipc host` REQUIRED** on the `ainex` container so it shares host `/dev/shm`; container is manually started (not compose) — recreate cmd in `ainex_docker_mount.md`.
- `shm_frame.py` lives on the shared bind mount `/home/pi/docker/tmp` so host py3.11 + container py3.8 import the SAME file (`DoubleBuffer.create/attach/write/read_copy/view`; raw `os.open`+`mmap`, not `multiprocessing.shared_memory`). Segments: `/dev/shm/gemini_color` (uint8 BGR), `/dev/shm/gemini_depth` (uint16 mm).
- Producer `/home/pi/gemini305_bridge.py` (host); consumers: `colour_ros_bridge_node.py`/`depth_ros_bridge_node.py` (container), `/home/pi/YOLO/yolo_zmq_direct.py` (host).
- **bridge.py retired** (image relay obsolete). Results-only `ainex_peripherals/scripts/yolo_results_bridge.py` (PULL 5551 → `/yolo/detections`).

## YOLO Vision
- **YOLO26n** runs on **host** (not container): ultralytics 8.4.40, NCNN backend, **6.7 FPS @ 320x320**
- Model: `/home/pi/yolo26n_ncnn_model/`; script: `/home/pi/yolo_camera.py`; publishes `/yolo/detections` — **NOT `/object/pixel_coords`** (line detection only; mixing corrupted `camera_lost_count`)
- Host↔ROS bridge: `roslibpy` (host) → `rosbridge_server` (container, port 9090)
- Full details: **`ainex_yolo.md`**

## Topic Files
- `ainex_docker_mount.md` — Docker mount setup (COMPLETED), container recreation command (includes ros_log mount), optional software/ mount
- `ainex_manual_button.md` — Manual button in Ainex Controller GUI, ROS walking API, servo IDs
- `ainex_architecture.md` — Full repo inventory, package list, node table, TF tree, config locations, proposed production architecture, MVP launch sequence
- **`ainex_truth_spec.md`** — CANONICAL source of truth: topic table, service table, servo ID table (authoritative)
- `ainex_rosa_agent.md` — ROSA agent integration: directory layout, tool table, Dockerfile notes, LLM config, build/run commands (rename caveat, see above)
- `ainex_bt_observability.md` — BT observability system: 4 JSONL files, event schemas, module structure (rename caveat, see above)

## Git Repo
- Build system: **`catkin build`** (NOT `catkin_make` — workspace uses catkin_tools)
- Single repo at `/home/pi` (master branch) tracking ROS source + Claude config
- **GitHub remote**: `origin` → `https://github.com/xzheng2/ainex_xyz.git` (private); user `xzheng2` (xzheng2@laurentian.ca); auth via credential helper `store` (`~/.git-credentials`, not tracked)
- Allowlist `.gitignore`: only `docker/ros_ws_src/**` and `.claude/` memory/settings are tracked
- Excludes: `__pycache__`, `.pyc`, `.zip`, `.7z`, `.bag`, `.so`, `build/`, `core` dumps, `*_this_session.md`, `*.egg-info/`, credentials, BT session logs
- **All new files in `ros_ws_src/` or `rosa_agent/` should be committed** — new files show as `??` (untracked, not ignored); stage with `git add` then commit + push when wrapping up
- **Commit checklist**: before any `git commit`, run `git status --short -- docker/ros_ws_src/ docker/rosa_agent/` and stage ALL untracked files in those dirs that belong to the repo
- A `PostToolUse` hook (`git_untracked_check.py`) auto-warns after each commit if `??` files remain in those two dirs
- **Claude Code hooks** (all in `~/.claude/hooks/`): `xyz_bt_lib_pre_guard.py`/`xyz_bt_lib_guard.py` (L1/L2/adapter/core files), `xyz_bt_tree_pre_guard.py` (tree wiring: bans raw composites + non-literal `state_key`), `xyz_bt_l1_running_guard.py` (L1 purity — no `Status.RUNNING`, scans behaviours + base class), `xyz_behavior_pre_guard.py`/`xyz_behavior_guard.py` (all `xyz_behavior/` project files), `bt_log_read_guard.py`, `command_explainer.py`, `git_untracked_check.py`
