# AINEX Runtime Debug Page

Browser-based runtime debugging UI combining rqt (via noVNC) and ROSA CLI (via ttyd) in one page.

```
┌──────────────────────────────────────────────┐
│  rqt / py_trees via noVNC iframe (port 6080) │
├──────────────────────────────────────────────┤
│  tick: 1618 │ source: /bt/marathon/bb/tick_id │
├──────────────────────────────────────────────┤
│  ROSA CLI via ttyd iframe (port 7681)        │
└──────────────────────────────────────────────┘
```

## Ports

| Port | Service | Container |
|------|---------|-----------|
| 5910 | x11vnc (internal VNC) | ainex |
| 6080 | noVNC websockify → browser | ainex |
| 7681 | ttyd ROSA terminal | rosa-agent |
| 8090 | page backend + static | rosa-agent |

> Port 5900 (default VNC) is permanently occupied by `wayvnc` on the host Pi. x11vnc uses 5910 instead.

## Prerequisites

### 1. ainex container: install noVNC stack (one-time per container lifetime)

These packages are NOT in the `ainex-backup:20260308` image. Install once into the running container:

```bash
docker exec ainex apt-get update
docker exec ainex apt-get install -y xvfb x11vnc novnc python3-websockify
```

> **Note**: These installs are ephemeral — lost if the container is recreated.
> After you finish debugging and stabilising this page, run `docker commit ainex ainex-backup:novnc`
> to create a new image that includes them. Until then, re-run the apt step if ainex is recreated.

### 2. rosa-agent container: rebuild with ttyd + fastapi

The Dockerfile now includes `ttyd` (aarch64 binary) and `fastapi`/`uvicorn`. Rebuild once:

```bash
cd /home/pi/docker
docker compose build rosa-agent
docker compose up -d rosa-agent
```

## Start

```bash
cd /home/pi
./runtime_debug_page/start_all.sh
```

This starts all services and runs the page backend in the foreground. Open:

```
http://<pi-ip>:8090
```

### Manual step-by-step (alternative to start_all.sh)

```bash
# noVNC stack (ainex)
# Note: port 5910 used for x11vnc — host wayvnc permanently occupies 5900
docker exec -d ainex bash -c 'Xvfb :99 -screen 0 1600x900x24 &'
docker exec -d ainex bash -c 'sleep 1 && DISPLAY=:99 rqt &'
docker exec -d ainex bash -c 'sleep 2 && x11vnc -display :99 -forever -shared -nopw -rfbport 5910 &'
docker exec -d ainex bash -c 'sleep 3 && websockify --web /usr/share/novnc 6080 localhost:5910 &'

# ROSA terminal (rosa-agent)
docker exec -d rosa-agent ttyd -p 7681 python3.9 /opt/rosa-agent/ainex_agent.py

# Page backend (rosa-agent)
docker exec -it rosa-agent python3.9 /opt/ainex_page/app.py
```

## Verification

```bash
# 1. ROS topic check
docker exec rosa-agent bash -c \
  'source /opt/ros/noetic/setup.bash && rostopic echo -n 1 /bt/marathon/bb/tick_id'
# Expected: data: "1618"

# 2. Backend API
curl http://localhost:8090/api/current_tick
# Expected: {"ok": true, "tick_id": 1618, "status": "live", ...}

# 3. noVNC
curl -I http://localhost:6080/vnc.html
# Expected: HTTP/1.1 200 OK

# 4. ttyd terminal
curl -I http://localhost:7681
# Expected: HTTP/1.1 200 OK
```

## rqt view control (CLI)

Change rqt layout without touching the browser page — the noVNC iframe updates automatically:

```bash
# Restart rqt with default view
docker exec ainex bash -c 'pkill -f rqt || true; DISPLAY=:99 rqt &'

# Restart rqt with a saved perspective
docker exec ainex bash -c \
  'pkill -f rqt || true; DISPLAY=:99 rqt --perspective-file /root/.config/ainex/rqt_tree.perspective &'

# Restart noVNC bridge if iframe disconnects
docker exec ainex bash -c \
  'pkill -f websockify || true; websockify --web /usr/share/novnc 6080 localhost:5910 &'
```

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| noVNC iframe blank | Xvfb / rqt / x11vnc / websockify not running in ainex; x11vnc failed to bind (check port 5910 free); wrong URL |
| ROSA terminal blank | ttyd not running; ainex_agent.py startup failed; missing API key |
| tick shows `waiting` | BT node not running; BB bridge not publishing; wrong ROS_MASTER_URI |
| tick shows `paused/stale` | Normal in pause/step mode — only investigate if run mode expected |

## Architecture Notes

- Page backend runs **inside `rosa-agent`** (not on the Pi host), because the host has no `rospy`.
- `rosa-agent` uses host networking, so it reaches the Ainex ROS master at `127.0.0.1:11311` directly.
- Volume mount: `/home/pi/runtime_debug_page` → `/opt/ainex_page:ro` (edit on host, instant in container).
- `disable_signals=True` in `rospy.init_node` prevents rospy from conflicting with uvicorn's signal handlers.
- x11vnc uses port **5910** (not 5900) — `wayvnc` on the host Pi permanently occupies 5900; host networking means the conflict is shared across all containers.
