## Environment

**Host**: Raspberry Pi 5, Linux 6.12 rpi-2712, user `pi`, home `/home/pi`, shell `zsh`

### ROS1 Container (`ainex`)
**Image**: recreate from `ainex-backup:20260630` (the container currently running may be on an
older tag — check with `docker ps`), user `ubuntu`, home `/home/ubuntu`

**User**: robot runs as `ubuntu` (pip `--user` pkgs — py_trees **2.1.6**, zmq, scipy — live in
`/home/ubuntu/.local`; py_trees is pinned via pip, not apt, and `rqt_py_trees` carries a local
patch for it — see `docker/ros_ws_src/VENDORED.md`); `docker exec ainex` defaults to **root**
which can't see them — use `docker exec -u ubuntu ainex` to check runtime deps.

**Bind mounts (host → container)**:
- `/home/pi/docker/ros_ws_src` → `/home/ubuntu/ros_ws/src` (ROS1 workspace source)
- `/home/pi/docker/ros_log` → `/home/ubuntu/.ros/log` (ROS log dir)
- `/home/pi/docker/src` → `/home/ubuntu/share/src`
- `/home/pi/docker/tmp` → `/home/ubuntu/share/tmp`
- `/home/pi/docker/software/lab_tool` → `/home/ubuntu/software/lab_tool`
- `/dev` → `/dev`
- `/tmp/.X11-unix` → `/tmp/.X11-unix`

**`--ipc host` is REQUIRED** — the container shares host `/dev/shm` for zero-copy Gemini 305 camera frames. Container is started manually (not compose); include this flag on recreation.

**ROS workspace**: built inside container at `/home/ubuntu/ros_ws/devel/setup.bash`; source edited on host at `/home/pi/docker/ros_ws_src/`

**Rules**:
- ROS nodes run inside the container, not on the host
- Never assume a Python package is missing until verified inside the container:
  `docker exec ainex python3 -c "import <module>"`
- **Never execute `roslaunch` or `rosrun` commands** (directly or via `docker exec`) to run the robot. Instead, show the command and ask the user to run it themselves. Example: _"Run this to start the node:"_ followed by the command in a code block.

## Architecture

Four packages under `docker/ros_ws_src/`:

- `xyz_bt_lib/` — portable BT library, no robot-specific code. Under `src/xyz_bt_lib/` each
  kind of module has exactly one home, and a subdirectory is created when its first module
  arrives: shared machinery (node/adapter/facade bases, composite factories, `latched_dwell` +
  `hysteresis` gates) in `core/`; blackboard keys and state in `blackboard/`; the tree's ROS
  subscribers in `adapters/`; every behaviour in `behaviours/` under its layer —
  `L1_perception`, `L2_locomotion` or `L3_system`.
- `xyz_perception/` — standalone detection / nav-planning nodes (apriltag / color /
  depth_nav / yolo) plus `DepthNavState.msg` and their launch + config.
- `xyz_run_lab/` — run bookkeeping: `run_lab/run_context.py` (run identity, per-run log
  directories, `run_meta.json` provenance) and `run_lab/run_metrics.py` (metric reduction),
  plus `config/bodies/` (per-robot calibration).
- `xyz_behavior/` — competition projects, plus the shared `launch/`, `log/`,
  `bt_observability/` and `tools/` they all use.

**Why four and not two.** Every package must be shippable without its former host.
Dependencies therefore run one way only: `xyz_bt_lib → xyz_perception`,
`xyz_behavior → {xyz_bt_lib, xyz_perception, xyz_run_lab}`. Nothing points back.

`xyz_perception` nodes are **separate ROS processes**, launched alongside the BT node.
Inside the BT process itself there is exactly one ROS ingress and one egress:

    ROS topic → adapters/ → Blackboard → L1 (pure predicate) / L2 (action)
              → RuntimeFacade → _RuntimeIO → ROS topic → robot

`adapters/` holds the only `rospy.Subscriber` in the tree process; `_RuntimeIO` is the only
publisher. L1 and L2 nodes never touch ROS directly — that decoupling is what lets the tree
run against `core/stub_facade.py` with no robot attached.

File-level detail is deliberately not restated here. The component-layer authority is
`xyz_bt_lib/src/xyz_bt_lib/README.md` plus each module's docstring ("docstring 即规格").

## Git workflow

- **Monorepo, and it must stay one.** `.claude/` and `docker/ros_ws_src/` reference each other
  by hardcoded path in both directions (`xyz_bt_lib/tools/validate_engine.py` → `.claude/skills`,
  `.claude/hooks/*` → `xyz_bt_lib/src/...` path patterns). Splitting it breaks both sides at once —
  never propose it.
- **Trunk-based**: commit straight to `master`. No Git Flow, no release branches.

Everything operational — the pre-push gate, branch/tag conventions, `package.xml` versioning,
the staging checklist — is injected by `.claude/hooks/git_workflow_guard.py` when a git command
actually runs, and is not repeated here.
