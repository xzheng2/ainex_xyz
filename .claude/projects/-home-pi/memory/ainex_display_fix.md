# Ainex Display Fix (rqt / Qt X11)

## Root Cause

- Desktop is **Wayland/Wayfire** with **XWayland on `:1`** (not `:0`)
- Container was started with `DISPLAY=:0` → wrong display, connection fails
- Container's Mesa is too old to support the Pi 5's **v3d GPU driver**
  → Qt/OpenCV crash with `libGL error: failed to load driver: v3d`

## Fix Summary

Two env vars must be set everywhere:
- `DISPLAY=:1` — correct XWayland display under Wayfire
- `LIBGL_ALWAYS_SOFTWARE=1` — bypass v3d, use llvmpipe software rendering

## Files Modified

### `/etc/systemd/system/start_node.service` (and `/home/pi/ainex/start_node.service`)
```ini
Environment="DISPLAY=:1"
Environment="LIBGL_ALWAYS_SOFTWARE=1"
```
(replaced `Environment="DISPLAY=:0"`)

### `/home/pi/ainex/start_node.sh`
```bash
docker exec -u ubuntu -w /home/ubuntu -e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1 ainex ...
```
(added `-e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1` to the docker exec call)

### `/home/pi/.zshrc`
```bash
export DISPLAY=:1
alias ainex='docker exec -e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1 -it -u ubuntu ainex bash'
```

### Inside container: `/home/ubuntu/ros_ws/.zshrc`
```bash
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
```
(ubuntu's default shell is zsh; `~/.zshrc` sources this file, so all zsh sessions pick it up)
Note: `.bashrc` was NOT used — ubuntu uses zsh, bash is never auto-sourced.

## Container Recreation Note
When recreating the container, include:
```
-e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1
```
(see `ainex_docker_mount.md` for full `docker run` command)

## Verification
```bash
# X11 connection test from inside container:
docker exec -e DISPLAY=:1 -u ubuntu ainex xdpyinfo -display :1 | head -3
# rqt launch test:
docker exec -e DISPLAY=:1 -e LIBGL_ALWAYS_SOFTWARE=1 -u ubuntu ainex bash -c \
  'source /opt/ros/noetic/setup.bash && rqt'
```
