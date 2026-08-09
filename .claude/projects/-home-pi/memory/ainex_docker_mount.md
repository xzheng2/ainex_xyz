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
- Backup image: `ainex-backup:20260308` (11.6GB) — safe to remove after extended validation
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

**Recreate from `ainex-backup:20260630`** (committed Jun 30 2026 — supersedes the
stale `:20260308`). The new image bakes in the freshly-built ROS workspace
(`devel/`/`build/`) + `py_trees==2.1.6` (pip --user, in `/home/ubuntu/.local`) +
`python3-pygraphviz`/`python3-termcolor` (apt), so recreating from it does NOT need
a `catkin build` or py_trees reinstall (those steps were only needed when recreating
from the old `:20260308`). If the container needs to be recreated again, include
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
  ainex-backup:20260630 \
  /bin/bash
```
Note: `DISPLAY=:1` because the desktop is Wayland/Wayfire with XWayland on `:1` (not `:0`).
Note: `LIBGL_ALWAYS_SOFTWARE=1` because the container's Mesa is too old to support the Pi 5's v3d GPU driver.

## software/lab_tool bind mount (Jun 30 2026)

`/home/ubuntu/software/lab_tool` is now bind-mounted from host
`/home/pi/docker/software/lab_tool` (included in the recreation cmd above). Edit
lab_tool on the host; the fix survives reboot AND container recreation. Done to
make the bgr8 red/blue inversion fix in `camera_thread.py` permanent
(`get_ros_image` flips `raw[:, :, ::-1]` so downstream `COLOR_RGB2LAB` +
`Format_RGB888` get RGB order). Only `lab_tool/` is mounted — the rest of the
image's `software/` tree (ainex_controller, servo_tool, etc.) is unshadowed.

## Container writable-layer reset on recreation (fixed by `:20260630` image)

Everything NOT on a bind mount lives in the container **writable layer** and is
reset to the image bake on recreation. Two things bit us recreating from the stale
`:20260308` (Jun 30 2026):
1. **ROS workspace** — `ros_ws/devel` + `ros_ws/build` (only `ros_ws/src` is mounted)
   reverted to the **May 19 2025** bake; `devel/setup.bash` had a hardcoded
   `ROS_PACKAGE_PATH` so `roslaunch`/`rospack` couldn't find packages added since
   (`xyz_behavior`, `hurocup2025`, `xyz_bt_lib`, …) — only `ainex_bringup` resolved.
2. **`py_trees==2.1.6`** — pip-installed AFTER the bake, so gone → BT nodes died with
   `ModuleNotFoundError: No module named 'py_trees'`.

**Now baked into `ainex-backup:20260630`**, so recreating from it needs NEITHER step.
Only if you ever recreate from `:20260308` again, redo them:
```bash
docker exec ainex bash -lc 'source /opt/ros/noetic/setup.bash && cd /home/ubuntu/ros_ws && catkin build'   # ~27 s warm
docker exec -u ubuntu ainex bash -lc 'python3 -m pip install --user py_trees==2.1.6'
docker exec ainex apt-get install -y python3-pygraphviz python3-termcolor
```
**Diagnostic gotcha:** `docker exec ainex …` runs as **root**, which does NOT see
ubuntu's `~/.local` user-site pip packages (zmq, scipy, py_trees, …). Always check
robot-runtime deps with `docker exec -u ubuntu ainex …`. After any future
significant in-container change, `docker commit ainex ainex-backup:<date>` and bump
the recreation tag so it isn't lost.

## Optional: Also mount the rest of software/

Not yet done. Could mount all of `/home/ubuntu/software` for direct GUI tool editing:
```bash
docker cp ainex:/home/ubuntu/software /home/pi/docker/software
# Add: -v /home/pi/docker/software:/home/ubuntu/software
```
