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

## Container Recreation

The container was recreated from `ainex-backup:20260308` with the additional `-v /home/pi/docker/ros_ws_src:/home/ubuntu/ros_ws/src` mount. If the container needs to be recreated again:

```bash
docker run \
  --name ainex \
  --hostname raspberrypi \
  --privileged \
  --net host \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/pi/docker/src:/home/ubuntu/share/src \
  -v /home/pi/docker/tmp:/home/ubuntu/share/tmp \
  -v /home/pi/docker/ros_ws_src:/home/ubuntu/ros_ws/src \
  -e DISPLAY=:0 \
  -itd \
  ainex-backup:20260308 \
  /bin/bash
```

## Optional: Also mount software/

Not yet done. Could mount `/home/ubuntu/software` for direct GUI tool editing:
```bash
docker cp ainex:/home/ubuntu/software /home/pi/docker/software
# Add: -v /home/pi/docker/software:/home/ubuntu/software
```
