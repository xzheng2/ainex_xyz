# Claude Memory - Ainex Robot (Raspberry Pi)

> This bank holds **only what cannot be re-derived from code or a live system**. Anything
> answerable by `ls`, a docstring, `rostopic list`, or `CLAUDE.md` was deleted Aug 15 2026
> (12 files → 4). Recover any of it with
> `git show HEAD~1:.claude/projects/-home-pi/memory/<file>`. Do not re-add derivable content:
> it decays silently and then misleads.

> **Package renamed Aug 8 2026**: `xyz_bt_edu` → `xyz_bt_lib` (dir, python module, imports,
> launch refs, hooks `xyz_bt_lib_guard.py`/`xyz_bt_lib_pre_guard.py`, skills `/xyz-bt-lib-node`
> + `/xyz-bt-lib-adapter`). Older references to `xyz_bt_edu`/`ainex_bt_edu` mean the same
> package pre-rename.

> **Competition events WIPED Aug 8 2026** for clean agent rebuild: all `xyz_behavior` project
> dirs, event launch files, legacy event scripts, event memories/docs deleted; git history
> squashed to a single clean-baseline commit. **When rebuilding events, do NOT consult old
> designs or any git history** — rebuild from the `xyz_bt_lib` framework only. Kept:
> xyz_behavior skeleton, bt_observability, ActionGroups motions (hand-tuned assets).

> **Framework-usage conclusions are not kept as memories.** How to use `xyz_bt_lib` —
> which node or decorator fits a situation and why — lives in the code that carries it:
> the module docstrings under `xyz_bt_lib/src/xyz_bt_lib/core/`, and the runnable
> `examples/demo_*.py` proofs that `validate_engine` executes on every push. Read those.
> Do not distil them back into this bank: a digested answer here short-circuits the
> reasoning the docstrings and demos exist to support.

## Topic files

- [Ablation experiment layout](ablation-experiment-layout.md) — results in a SECOND repo
  `/home/pi/experiments/ainex_xyz_result`, partitioned by body_id **AND study AND lane**;
  four lanes share one robot; parallel session-transcript pipeline for agent process data
- [catkin build needs ROS sourced](catkin-build-needs-ros-sourced.md) — clean rebuilds fail
  without an explicit `source /opt/ros/noetic/setup.bash`; incremental builds hide it
- [Plan summary preference](plan-summary-preference.md) — show only the current round's changes
  for approval, not the whole plan file

Environment, bind mounts, the four-package architecture and the git workflow are in
`/home/pi/CLAUDE.md` and are deliberately not repeated here.

## Container recreation

```bash
docker run \
  --name ainex --hostname raspberrypi \
  --privileged --net host --ipc host \
  --restart=unless-stopped \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/pi/docker/src:/home/ubuntu/share/src \
  -v /home/pi/docker/tmp:/home/ubuntu/share/tmp \
  -v /home/pi/docker/ros_ws_src:/home/ubuntu/ros_ws/src \
  -v /home/pi/docker/ros_log:/home/ubuntu/.ros/log \
  -v /home/pi/docker/software/lab_tool:/home/ubuntu/software/lab_tool \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -itd ainex-backup:20260814-clean /bin/bash
```

`DISPLAY=:1` because the desktop is Wayland/Wayfire with XWayland on `:1`, not `:0`.
`LIBGL_ALWAYS_SOFTWARE=1` because the container's Mesa is too old for the Pi 5 v3d driver.

**Then rebuild the workspace — the baked `devel`/`build` are stale AND root-owned:**

```bash
docker exec ainex bash -lc \
  'cd /home/ubuntu/ros_ws && rm -rf build devel logs .catkin_tools'
docker exec -u ubuntu ainex bash -lc \
  'cd /home/ubuntu/ros_ws && source /opt/ros/noetic/setup.bash && catkin build'
```

Traps, each one cost real time:

- **`.catkin_tools` must be in that `rm`.** Its top level is ubuntu-owned so it looks fine, but
  the `profiles/default/packages/*/package.xml` underneath are root's. Leaving it behind gets a
  build that reaches 15 of 23 packages then dies with `PermissionError … uuid_msgs/package.xml`
  in `cache-manifest`, abandoning everything downstream.
- **Never `catkin build` as root.** Root-owned artifacts in `build/`/`devel/` make the next
  ubuntu build fail with a misleading `cmake.lock creation failed (check permissions)`.
  Fix: `docker exec -u root ainex chown -R ubuntu:ubuntu /home/ubuntu/ros_ws/{build,devel,logs}`
- **`docker exec ainex` runs as root**, which does not see ubuntu's `~/.local` user-site pip
  packages (py_trees, zmq, scipy). Always use `-u ubuntu` to check runtime deps.
- **Post-create:** redo the ActionGroups symlink — the controller GUI resolves its action dir
  from its own script location, so it writes to a path that is neither bind-mounted nor in git:
  ```bash
  docker exec -u ubuntu ainex bash -lc '
    cd /home/ubuntu/software/ainex_controller
    [ -L ActionGroups ] || mv ActionGroups ActionGroups.preunify
    ln -sfn /home/ubuntu/ros_ws/src/ActionGroups ActionGroups'
  ```

After any significant in-container change: `docker commit ainex ainex-backup:<date>` and bump
the tag above, or the change is lost on the next recreation.

## Hardware conventions

Facts with no home in code. For anything else — servo IDs, topic names, service signatures —
read `ainex_driver/ainex_kinematics/config/servo_controller.yaml` or introspect the live graph.

- **IMU orientation**: the sensor's Y-axis is up, so **roll ≈ 90° when the robot is upright**,
  not 0°. Fall check is `abs(roll-90) < 30 and abs(pitch) < 30`. `/imu` carries a real fused
  orientation quaternion; `/imu_corrected` is bias-corrected accel+gyro only.
- **Walking**: `/walking/command` with `enable_control` **then** `start`. The `/ainex/set_*`
  service names appear in old docs and **do not exist**.
- **Servo 24 (head tilt) is clamped [280, 550]** in two independent code paths:
  `servo_controller.yaml` + `angle2pulse()`, and `set_bus_servo_position()` in
  `ros_robot_controller_node.py` (the path `move_head()` actually takes). Hardware EEPROM
  register 0x30 is not implemented by the STM32 firmware, so the clamp must stay in software.
- **Arm servos: L = odd ID, R = even ID.** Legs run ankle→hip, distal to proximal.
- **`.d6a` action files are SQLite databases**, table `ActionGroup`. Each row is a keyframe:
  col[1] = duration (ms), col[2+i] = servo i+1 position. Inspect with
  `docker exec ainex sqlite3 <file>.d6a ".schema" "SELECT * FROM ActionGroup LIMIT 3;"`

## Camera and YOLO transport

- Gemini 305 frames move host→container by **`/dev/shm` double buffer + ZMQ notify** — the
  frame is written once and ZMQ carries only an 18-byte frame-ready notice. Segments:
  `/dev/shm/gemini_color` (uint8 BGR), `/dev/shm/gemini_depth` (uint16 mm).
- **`--ipc host` is what makes this work.** Without it the container's `/dev/shm` is private and
  the colour/depth bridges see nothing.
- `shm_frame.py` lives on the `/home/pi/docker/tmp` bind mount so host py3.11 and container
  py3.8 import the **same file**. Raw `os.open` + `mmap`, deliberately not
  `multiprocessing.shared_memory`.
- Producer: `/home/pi/gemini305_bridge.py` (host). Consumers: `colour_ros_bridge_node.py` /
  `depth_ros_bridge_node.py` (container), `/home/pi/YOLO/yolo_zmq_direct.py` (host).
- **YOLO runs on the host, not in the container** (host py3.11 is much faster than container
  py3.8 for inference). YOLO26n, NCNN backend, 320×320: **~25 FPS with ROS down, ~16 FPS with
  ROS running** (Aug 15 2026). Keep input at 320 — 640 roughly halves it.
- Live script `/home/pi/YOLO/yolo_zmq_direct.py`: SUB notify :5552, PUSH detections :5551,
  `CONF_THRESHOLD` 0.5. `MODEL_PATH` is set at the top of that script. Base model and developer
  copies are under `/home/pi/YOLO/for_developer/`.
- Detections publish to **`/yolo/detections`** — **not** `/object/pixel_coords`, which is
  line-detection only; reusing it corrupts `camera_lost_count`.
- **Camera view banding rendering rule (user requirement, do NOT change):** the 640×640
  composite persists across frames; a band never visited stays black; a band not updated this
  frame keeps its last content. No mirror-to-both fallback.

## Environment gotchas

- ROS Noetic **arm64 apt packages are gone (404)** — anything missing must be built from source.
- **Do NOT install `ros-noetic-py-trees-ros`** — it requires py_trees 0.7.x and is incompatible
  with the 2.1.6 the BT stack pins.
- After container recreation, reinstall: `apt-get install -y python3-pygraphviz python3-termcolor`.
- `py_trees` 2.1.6 comes from **pip, not apt**; `py_trees_msgs`, `uuid_msgs`, `unique_id` and
  `rqt_py_trees` are built from source in the workspace (`rqt_py_trees` carries a local patch —
  see `docker/ros_ws_src/VENDORED.md`).
