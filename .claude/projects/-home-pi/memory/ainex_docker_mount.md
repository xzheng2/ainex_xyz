# Ainex Docker Mount Setup

## Current Mounts (as of 2026-03-14)

| Host Path | Container Path | Contents |
|-----------|---------------|----------|
| `/home/pi/docker/src` | `/home/ubuntu/share/src` | IPC signals (`.halt.txt`, `.led.txt`) |
| `/home/pi/docker/tmp` | `/home/ubuntu/share/tmp` | Scratch files |
| `/home/pi/docker/ros_ws_src` | `/home/ubuntu/ros_ws/src` | **All ROS packages** (11 pkgs + third_party, 17 catkin packages) |
| `/home/pi/docker/ros_log` | `/home/ubuntu/.ros/log` | **ROS log files** — symlink `/root/.ros/log` → `/home/ubuntu/.ros/log` so both users share the mount; also mounted read-only into rosa-agent at `/root/.ros/log` |
| `/dev` | `/dev` | Device passthrough |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 display forwarding |
| `/etc/localtime` | `/etc/localtime` (ro) | Host timezone (America/Toronto) |
| `/etc/timezone` | `/etc/timezone` (ro) | Host timezone name |

## ros_ws/src Mount — COMPLETED

The ROS workspace source is now host-mounted. Claude Code can directly read/edit files under `/home/pi/docker/ros_ws_src/`.

- Migrated 2026-03-08 using test-container approach
- Backup images: `:20260308` and `:20260630` were both DELETED Aug 14 2026 — each still
  baked in material covered by the wipe note at the top of MEMORY.md, so **do not restore
  either from any external copy**. `:20260814-clean` is the only ainex-backup image left,
  and there is no rollback image behind it: it was verified by recreating the container
  from it and getting a 23/23 catkin build plus a fully green `validate_engine`, which is
  the evidence that stands in for a rollback point.
- `catkin build` verified: all 17 packages succeed
- `rospack list` verified: all ainex packages discoverable
- UIDs match (pi=ubuntu=1000), no permission issues

## Auto-Start on Boot

The container restart policy was set so it starts automatically with the Pi:

```bash
docker update --restart=unless-stopped ainex
```

This means the container starts on boot and restarts on crash, but stays stopped if manually stopped with `docker stop ainex`.

## Container Recreation

**Recreate from `ainex-backup:20260814-clean`** (Aug 14 2026 — supersedes `:20260630`,
which was deleted, and the stale `:20260308`). The new image bakes in the freshly-built ROS workspace
(`devel/`/`build/`) + `py_trees==2.1.6` (pip --user, in `/home/ubuntu/.local`) +
`python3-pygraphviz`/`python3-termcolor` (apt), so recreating from it does NOT need
a py_trees reinstall.

> **It DOES need a catkin build now, and one extra step first (verified Aug 14 2026).**
> The baked `devel/`/`build/` are from Jun 30 and the workspace source has moved on a
> long way since, so the bake is stale — `devel/setup.bash` carries a hardcoded
> `ROS_PACKAGE_PATH` from that date. Worse, the baked dirs are **root-owned**, so the
> obvious `docker exec -u ubuntu ... rm -rf build devel` fails with several hundred
> `Permission denied` lines and the build then fails. Remove them as root, build as
> ubuntu:
>
> ```bash
> docker exec ainex bash -lc \
>   'cd /home/ubuntu/ros_ws && rm -rf build devel logs .catkin_tools'
> docker exec -u ubuntu ainex bash -lc \
>   'cd /home/ubuntu/ros_ws && source /opt/ros/noetic/setup.bash && catkin build'
> ```
>
> **`.catkin_tools` must be in that list.** Its top level is `ubuntu`-owned so it looks
> fine, but the baked `profiles/default/packages/*/package.xml` underneath are root's.
> Leaving it behind gets you a build that reaches 15 of 23 packages and then dies with
> `PermissionError ... .catkin_tools/profiles/default/packages/uuid_msgs/package.xml`
> in the `cache-manifest` stage, abandoning the 7 packages downstream of it.

If the container needs to be recreated again, include
`--restart=unless-stopped`. **`--ipc host` is required** for the /dev/shm zero-copy
camera transport (gemini305_bridge + shm_frame.py double buffer) — without it the
container's /dev/shm is private and the colour/depth bridges cannot see the host's
shared frame buffers:

```bash
docker run \
  --name ainex \
  --hostname raspberrypi \
  --privileged \
  --net host \
  --ipc host \
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
  -e DISPLAY=:1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -itd \
  ainex-backup:20260814-clean \
  /bin/bash
```
Note: `DISPLAY=:1` because the desktop is Wayland/Wayfire with XWayland on `:1` (not `:0`).
Note: `LIBGL_ALWAYS_SOFTWARE=1` because the container's Mesa is too old to support the Pi 5's v3d GPU driver.

### Required post-create step: ActionGroups symlink (Aug 12 2026)

The controller GUI resolves its action directory from its own script location
(`main.py` `self.path`, `action_group_controller.py` `os.path.realpath(__file__)`), so it
always writes to `software/ainex_controller/ActionGroups` — which is **not** bind-mounted
and **not** in git. `MotionManager`'s default `action_path` now points at the tracked
`ros_ws/src/ActionGroups` instead, so the GUI directory must be redirected to the same
place or GUI-authored actions become invisible to every ROS node again:

```bash
docker exec -u ubuntu ainex bash -lc '
  cd /home/ubuntu/software/ainex_controller
  [ -L ActionGroups ] || mv ActionGroups ActionGroups.preunify
  ln -sfn /home/ubuntu/ros_ws/src/ActionGroups ActionGroups'
```

The symlink lives in the writable layer, so **redo it on every recreation** (until a newer
`docker commit` bakes it in).

## software/lab_tool bind mount (Jun 30 2026)

`/home/ubuntu/software/lab_tool` is now bind-mounted from host
`/home/pi/docker/software/lab_tool` (included in the recreation cmd above). Edit
lab_tool on the host; the fix survives reboot AND container recreation. Done to
make the bgr8 red/blue inversion fix in `camera_thread.py` permanent
(`get_ros_image` flips `raw[:, :, ::-1]` so downstream `COLOR_RGB2LAB` +
`Format_RGB888` get RGB order). Only `lab_tool/` is mounted — the rest of the
image's `software/` tree (ainex_controller, servo_tool, etc.) is unshadowed.

## Container writable-layer reset on recreation (fixed by the baked image)

Everything NOT on a bind mount lives in the container **writable layer** and is
reset to the image bake on recreation. Two things bit us recreating from the stale
`:20260308` (Jun 30 2026):
1. **ROS workspace** — `ros_ws/devel` + `ros_ws/build` (only `ros_ws/src` is mounted)
   reverted to the **May 19 2025** bake; `devel/setup.bash` had a hardcoded
   `ROS_PACKAGE_PATH` so `roslaunch`/`rospack` couldn't find packages added since
   (`xyz_behavior`, `xyz_bt_lib`, …) — only `ainex_bringup` resolved.
2. **`py_trees==2.1.6`** — pip-installed AFTER the bake, so gone → BT nodes died with
   `ModuleNotFoundError: No module named 'py_trees'`.

`:20260814-clean` inherits that bake, so **(2) py_trees and the apt packages are still
covered** — no reinstall needed. **(1) is NOT**: the workspace bake is from Jun 30 and the
source has changed a great deal since, so a rebuild is required. See the recreation
section above for the exact two commands (root removes `build devel logs .catkin_tools`,
then ubuntu builds).

Kept for reference, in case a future image bake is ever stale in the other direction too:
```bash
docker exec -u ubuntu ainex bash -lc 'python3 -m pip install --user py_trees==2.1.6'
docker exec ainex apt-get install -y python3-pygraphviz python3-termcolor
```
**Diagnostic gotcha:** `docker exec ainex …` runs as **root**, which does NOT see
ubuntu's `~/.local` user-site pip packages (zmq, scipy, py_trees, …). Always check
robot-runtime deps with `docker exec -u ubuntu ainex …`.

**Corollary — never `catkin build` as root.** A root-run build leaves root-owned files
in `build/`/`devel/`, and the next build as `ubuntu` dies with a misleading
`atomic_configure_file.cmake … cmake.lock creation failed (check permissions)`.
Found Aug 12 2026: 881 root-owned artifacts (from an Aug 11 root build) blocked
`ainex_peripherals` and abandoned 13 packages. Fix — artifacts are all regenerable:
```bash
docker exec -u root ainex chown -R ubuntu:ubuntu /home/ubuntu/ros_ws/{build,devel,logs}
``` After any future
significant in-container change, `docker commit ainex ainex-backup:<date>` and bump
the recreation tag so it isn't lost.

## Optional: Also mount the rest of software/

Not yet done. Could mount all of `/home/ubuntu/software` for direct GUI tool editing:
```bash
docker cp ainex:/home/ubuntu/software /home/pi/docker/software
# Add: -v /home/pi/docker/software:/home/ubuntu/software
```
