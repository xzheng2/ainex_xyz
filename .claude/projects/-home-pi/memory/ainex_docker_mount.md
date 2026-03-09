# Ainex Docker Mount Setup

## Current Mounts (as of 2026-03-08)

| Host Path | Container Path | Contents |
|-----------|---------------|----------|
| `/home/pi/docker/src` | `/home/ubuntu/share/src` | IPC signals (`.halt.txt`, `.led.txt`) |
| `/home/pi/docker/tmp` | `/home/ubuntu/share/tmp` | Scratch files |
| `/home/pi/docker/ros_ws_src` | `/home/ubuntu/ros_ws/src` | **All ROS packages** (11 pkgs + third_party, 17 catkin packages) |
| `/dev` | `/dev` | Device passthrough |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 display forwarding |

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

The container was recreated from `ainex-backup:20260308` with the additional `-v /home/pi/docker/ros_ws_src:/home/ubuntu/ros_ws/src` mount. If the container needs to be recreated again, include `--restart=unless-stopped`:

```bash
docker run \
  --name ainex \
  --hostname raspberrypi \
  --privileged \
  --net host \
  --restart=unless-stopped \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/pi/docker/src:/home/ubuntu/share/src \
  -v /home/pi/docker/tmp:/home/ubuntu/share/tmp \
  -v /home/pi/docker/ros_ws_src:/home/ubuntu/ros_ws/src \
  -e DISPLAY=:1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -itd \
  ainex-backup:20260308 \
  /bin/bash
```
Note: `DISPLAY=:1` because the desktop is Wayland/Wayfire with XWayland on `:1` (not `:0`).
Note: `LIBGL_ALWAYS_SOFTWARE=1` because the container's Mesa is too old to support the Pi 5's v3d GPU driver.

## Optional: Also mount software/

Not yet done. Could mount `/home/ubuntu/software` for direct GUI tool editing:
```bash
docker cp ainex:/home/ubuntu/software /home/pi/docker/software
# Add: -v /home/pi/docker/software:/home/ubuntu/software
```
