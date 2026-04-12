# ROSA + RQT noVNC Single Page Design

> Goal: Put Ainex rqt runtime display and ROSA interaction into one lightweight browser page.
> UI strategy: Embed existing rqt through noVNC iframe, embed ROSA CLI through web terminal iframe.
> Tick source: subscribe to ROS topic `/bt/marathon/bb/tick_id`, not screen-scrape rqt and not read JSONL for the status bar.
> Deployment: the page backend runs inside the **rosa-agent container** (host Pi has no `rospy`); ainex ROS and ROSA run in separate Docker containers.

**Status: MVP implemented (Apr 12 2026). Start: `./runtime_debug_page/start_all.sh`**

---

## 1. Objective

Create a minimal runtime debugging page with three vertical areas:

```text
┌──────────────────────────────────────────────┐
│ rqt / py_trees / plugin via noVNC iframe     │
├──────────────────────────────────────────────┤
│ narrow status bar                            │
│ [tick_id] [source] [connection/status] ...   │
├──────────────────────────────────────────────┤
│ ROSA CLI via web terminal iframe             │
└──────────────────────────────────────────────┘
```

The page should let a weak-programming user:

1. Watch rqt tree view live.
2. See the current BT tick id in a narrow synchronized status row.
3. Interact with ROSA in a terminal below the rqt view.
4. Use the tick id shown in the status bar when asking ROSA to analyze a pause/step tick.

---

## 2. Design Decision

### 2.1 Use noVNC for rqt

rqt is a Qt desktop application. The lightest browser integration is to keep rqt unchanged and remote-display it:

```text
rqt -> Xvfb DISPLAY -> x11vnc -> websockify/noVNC -> browser iframe
```

Do not rewrite rqt as a web UI for MVP.

### 2.2 Use Web Terminal for ROSA

ROSA MVP interaction uses the existing CLI:

```text
ainex_agent.py interactive CLI -> ttyd -> browser iframe
```

`ttyd` chosen over `wetty` (simpler binary, no Node.js dependency). This avoids writing a chat backend for MVP.

### 2.3 Status Bar Tick Source: ROS Topic

The status bar should not parse noVNC pixels and should not inspect rqt UI internals.

It should subscribe to:

```text
/bt/marathon/bb/tick_id
```

Message type:

```text
std_msgs/String
```

The value is expected to be JSON text from the BB bridge, for example:

```text
1618
```

or a JSON string representation depending on bridge output. The backend parses both plain numeric strings and JSON values.

Why this is preferred:

```text
1. rqt tree view and /bt/marathon/bb/tick_id are both driven by the same BT runtime.
2. pause/step mode naturally freezes both the rqt view and tick_id.
3. The page avoids fragile screen scraping.
4. It provides the current tick id even when the noVNC iframe is not inspectable from JavaScript.
```

---

## 3. High-Level Architecture

```text
Host Raspberry Pi
  ├── Browser
  │   ├── iframe: noVNC rqt desktop (port 6080)
  │   ├── status bar: tick from page backend API
  │   └── iframe: ROSA web terminal (port 7681)
  │
  └── Docker CLI access
      ├── starts/stops rqt/noVNC processes in ainex container
      └── starts/stops ROSA terminal + page backend in rosa-agent container

Ainex Docker Container (host network)
  ├── ROS master / Ainex BT node
  ├── publishes /bt/marathon/bb/tick_id
  ├── Xvfb :99
  ├── rqt (DISPLAY=:99)
  ├── x11vnc (port 5900)
  └── websockify/noVNC (port 6080)

ROSA Agent Docker Container (host network)
  ├── ainex_agent.py interactive CLI
  ├── ttyd (port 7681) — wraps ainex_agent.py
  └── app.py — FastAPI page backend (port 8090)
      ├── rospy subscriber: /bt/marathon/bb/tick_id
      ├── GET /api/current_tick
      └── serves static/ (index.html, style.css, app.js)
```

Logical browser layout:

```text
Browser Page  (served from port 8090)
  ├── iframe: noVNC rqt desktop (port 6080)
  ├── status bar: polls /api/current_tick every 500 ms
  └── iframe: ROSA web terminal (port 7681)
```

### 3.1 Container Boundary

The page backend runs inside the **rosa-agent container**, not on the host Pi.

Reason: the host Pi does not have `rospy` installed, and the ROS Noetic apt repo arm64 packages are no longer available (404). Installing ROS on the host is not feasible.

The rosa-agent container already provides:

```text
- Python 3.9
- rospy (via PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages)
- host networking (shares 127.0.0.1 with ROS master at port 11311)
- fastapi + uvicorn (added to requirements.txt)
```

The page source lives on the host at `/home/pi/runtime_debug_page/` and is volume-mounted read-only into the container at `/opt/ainex_page/`. Edits on the host are visible immediately inside the container.

```yaml
# docker-compose.yml volume entry (rosa-agent service):
- /home/pi/runtime_debug_page:/opt/ainex_page:ro
```

`rospy.init_node` uses `disable_signals=True` so rospy does not interfere with uvicorn's signal handlers.

---

## 4. Files

### 4.1 runtime_debug_page/ (host, git-tracked)

```text
runtime_debug_page/
├── app.py            ← FastAPI backend (rospy subscriber + API + static serve)
├── static/
│   ├── index.html    ← 3-panel layout
│   ├── style.css     ← CSS grid
│   └── app.js        ← 500 ms tick polling + hostname-aware iframe URLs
├── start_all.sh      ← one-shot startup script (runs all services from host)
└── README.md         ← startup, verification, rqt control, troubleshooting
```

### 4.2 Container changes

```text
docker/rosa-agent/Dockerfile
  - added: ttyd 1.7.7 aarch64 binary (/usr/local/bin/ttyd)
  - source: https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64

docker/rosa-agent/requirements.txt
  - added: fastapi>=0.111.0
  - added: uvicorn>=0.30.0

docker/docker-compose.yml
  - added volume: /home/pi/runtime_debug_page:/opt/ainex_page:ro
```

### 4.3 Future (not MVP)

```text
runtime_debug_page/rqt_control.py  ← CLI wrapper for rqt perspective switching
```

---

## 5. Browser Layout

### 5.1 HTML Structure

```html
<!doctype html>
<html>
  <head>
    <meta charset="utf-8">
    <title>AINEX Runtime Debug</title>
    <link rel="stylesheet" href="/static/style.css">
  </head>
  <body>
    <div id="layout">
      <iframe id="rqt-frame" title="rqt via noVNC" allowfullscreen></iframe>

      <div id="status-bar">
        <div class="status-cell tick-cell">
          tick: <span id="tick-id">?</span>
        </div>
        <div class="status-cell">
          source: <span id="tick-source">/bt/marathon/bb/tick_id</span>
        </div>
        <div class="status-cell">
          status: <span id="tick-status">connecting</span>
        </div>
        <div class="status-cell">
          age: <span id="tick-age">?</span>
        </div>
      </div>

      <iframe id="rosa-terminal" title="ROSA CLI terminal"></iframe>
    </div>
    <script src="/static/app.js"></script>
  </body>
</html>
```

`app.js` sets iframe URLs using the browser's current hostname so the page works both on the Pi and from another machine on the same network:

```js
const host = window.location.hostname;
document.getElementById("rqt-frame").src =
  "http://" + host + ":6080/vnc.html?autoconnect=true&resize=scale";
document.getElementById("rosa-terminal").src =
  "http://" + host + ":7681";
```

Do not hardcode `localhost` in the iframe URLs.

### 5.2 CSS Layout

Use CSS grid:

```css
html,
body {
  height: 100%;
  margin: 0;
  overflow: hidden;
}

#layout {
  height: 100vh;
  display: grid;
  grid-template-rows: minmax(360px, 1fr) 32px 260px;
  background: #0f1115;
}

#rqt-frame,
#rosa-terminal {
  width: 100%;
  height: 100%;
  border: 0;
}

#status-bar {
  display: grid;
  grid-template-columns: 180px 300px 1fr 120px;
  height: 32px;
  border-top: 1px solid #30343b;
  border-bottom: 1px solid #30343b;
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 14px;
  background: #151922;
  color: #e6e8ee;
}

.status-cell {
  display: flex;
  align-items: center;
  min-width: 0;
  padding: 0 10px;
  border-right: 1px solid #30343b;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.tick-cell {
  font-weight: 700;
  color: #7ee787;
}

.status-warn {
  color: #f2cc60;
}

.status-error {
  color: #ff7b72;
}
```

---

## 6. Page Backend

### 6.1 Framework

FastAPI + uvicorn, running inside the rosa-agent container.

```text
/opt/ainex_page/app.py  (host: runtime_debug_page/app.py)
```

Responsibilities:

1. Serve `/` and `/static/*`.
2. Initialize `rospy` in a daemon thread (`disable_signals=True`).
3. Subscribe to `/bt/marathon/bb/tick_id`.
4. Store the latest tick in memory (thread-safe with `threading.Lock`).
5. Expose `GET /api/current_tick`.

### 6.2 Backend State

```python
LATEST_TICK = {
    "ok": False,
    "tick_id": None,
    "raw": None,
    "source": "/bt/marathon/bb/tick_id",
    "updated_at": None,
    "error": "no tick received yet",
}
```

### 6.3 ROS Subscriber

Topic:

```text
/bt/marathon/bb/tick_id
```

Type:

```text
std_msgs/String
```

Parsing algorithm:

```text
1. Strip whitespace.
2. Try json.loads(data).
3. If result is int -> use it.
4. If result is str containing integer -> int(result).
5. If json.loads fails, try int(data).
6. Otherwise mark parse error.
```

### 6.4 API: `GET /api/current_tick`

Response (live):

```json
{
  "ok": true,
  "tick_id": 1618,
  "raw": "1618",
  "source": "/bt/marathon/bb/tick_id",
  "age_seconds": 0.4,
  "stale": false,
  "status": "live"
}
```

Response (no tick received yet):

```json
{
  "ok": false,
  "tick_id": null,
  "source": "/bt/marathon/bb/tick_id",
  "age_seconds": null,
  "stale": true,
  "status": "waiting",
  "error": "no tick received yet"
}
```

Stale threshold:

```text
STALE_SECONDS = 10.0
```

Stale is allowed because pause/step mode may freeze the tick.

Status interpretation:

```text
age_seconds <= STALE_SECONDS:  status = "live"
age_seconds > STALE_SECONDS:   status = "paused/stale"
no tick received:              status = "waiting"
```

Do not treat stale as an error.

---

## 7. Frontend Tick Polling

### 7.1 JavaScript

```js
async function refreshTick() {
  const tickId     = document.getElementById("tick-id");
  const tickStatus = document.getElementById("tick-status");
  const tickAge    = document.getElementById("tick-age");

  try {
    const res  = await fetch("/api/current_tick", { cache: "no-store" });
    const data = await res.json();

    tickStatus.classList.remove("status-warn", "status-error");

    if (!data.ok) {
      tickId.textContent     = "?";
      tickAge.textContent    = "?";
      tickStatus.textContent = data.error || data.status || "unavailable";
      tickStatus.classList.add("status-error");
      return;
    }

    tickId.textContent = data.tick_id;
    tickAge.textContent =
      data.age_seconds == null ? "?" : data.age_seconds.toFixed(1) + "s";

    if (data.stale) {
      tickStatus.textContent = "paused/stale";
      tickStatus.classList.add("status-warn");
    } else {
      tickStatus.textContent = "live";
    }
  } catch (err) {
    tickId.textContent     = "?";
    tickAge.textContent    = "?";
    tickStatus.textContent = "api error";
    tickStatus.classList.add("status-error");
  }
}

setInterval(refreshTick, 500);
refreshTick();
```

Polling every 500 ms is enough. The page does not need 15 Hz UI refresh.

---

## 8. Runtime Services

### 8.1 rqt via noVNC (ainex container)

The rqt/noVNC stack runs in the ainex Docker container because rqt needs the ROS packages and message types from the ainex ROS workspace.

**noVNC packages are not in the `ainex-backup:20260308` image. Install once into the running container:**

```bash
docker exec ainex apt-get update
docker exec ainex apt-get install -y xvfb x11vnc novnc python3-websockify
```

These installs are **ephemeral** — lost if the container is recreated. After debugging stabilises, run:

```bash
docker commit ainex ainex-backup:novnc
```

to preserve the install in a new image snapshot.

Start commands (from host):

```bash
docker exec -d ainex bash -c 'Xvfb :99 -screen 0 1600x900x24 &'
docker exec -d ainex bash -c 'sleep 1 && DISPLAY=:99 rqt &'
docker exec -d ainex bash -c 'sleep 2 && x11vnc -display :99 -forever -shared -nopw -rfbport 5900 &'
docker exec -d ainex bash -c 'sleep 3 && websockify --web /usr/share/novnc 6080 localhost:5900 &'
```

The page iframe points to:

```text
http://<pi-host>:6080/vnc.html?autoconnect=true&resize=scale
```

noVNC static path confirmed at `/usr/share/novnc/vnc.html` in the installed package.

### 8.2 ROSA via Web Terminal (rosa-agent container)

`ttyd` is used. `wetty` not chosen (requires Node.js).

`ttyd` is not in the Ubuntu 20.04 arm64 apt repos. It is installed in the Dockerfile as a prebuilt binary:

```dockerfile
RUN curl -fsSL \
      "https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64" \
      -o /usr/local/bin/ttyd \
    && chmod +x /usr/local/bin/ttyd
```

Note: the binary filename is `ttyd.aarch64`, not `ttyd_linux.aarch64`.

Start command (from host):

```bash
docker exec -d rosa-agent ttyd -p 7681 python3.9 /opt/rosa-agent/ainex_agent.py
```

The page iframe points to:

```text
http://<pi-host>:7681
```

### 8.3 Page Backend (rosa-agent container)

The backend runs inside rosa-agent, not on the host Pi. The source is volume-mounted from the host:

```bash
docker exec -it rosa-agent python3.9 /opt/ainex_page/app.py
```

Port: `8090`

Browser opens:

```text
http://<pi-host>:8090
```

No `ROS_MASTER_URI` export needed on the host — the container already has `ROS_MASTER_URI=http://127.0.0.1:11311` set in its environment (docker-compose.yml).

### 8.4 One-Shot Startup

```bash
./runtime_debug_page/start_all.sh
```

This runs all four service groups sequentially and leaves the page backend in the foreground (Ctrl-C to stop).

---

## 9. Runtime rqt Page Control via CLI

The MVP leaves room to modify the rqt view while the page is running.

Do not implement browser buttons for this in MVP. Use host CLI commands that control the rqt process inside the ainex Docker container. Because noVNC displays the same virtual X display, changing/restarting rqt inside the container is immediately reflected in the iframe.

### 9.1 Control Model

```text
Host CLI
  -> docker exec ainex ...
  -> starts/stops/restarts rqt on DISPLAY=:99
  -> noVNC iframe keeps showing DISPLAY=:99
```

The browser page remains unchanged. Only the content displayed in the noVNC iframe changes.

### 9.2 Suggested CLI Wrapper (not MVP)

Later create:

```text
runtime_debug_page/rqt_control.py
```

Suggested commands:

```bash
python3 runtime_debug_page/rqt_control.py status
python3 runtime_debug_page/rqt_control.py restart-default
python3 runtime_debug_page/rqt_control.py restart-tree
python3 runtime_debug_page/rqt_control.py restart-perspective <path>
python3 runtime_debug_page/rqt_control.py stop-rqt
```

### 9.3 Example Commands

Restart default rqt:

```bash
docker exec ainex bash -c 'pkill -f rqt || true; DISPLAY=:99 rqt &'
```

Restart rqt with a saved perspective:

```bash
docker exec ainex bash -c \
  'pkill -f rqt || true; DISPLAY=:99 rqt --perspective-file /root/.config/ainex/rqt_tree.perspective &'
```

Restart only noVNC bridge if the iframe disconnects:

```bash
docker exec ainex bash -c \
  'pkill -f websockify || true; websockify --web /usr/share/novnc 6080 localhost:5900 &'
```

### 9.4 Safety Rules

```text
Only provide predefined commands for rqt/noVNC display control.
Do not expose arbitrary shell execution through the browser page.
Do not let ROSA execute these commands automatically in read-only mode.
```

---

## 10. Synchronization Semantics

The status bar tick id and rqt tree view are not synchronized by reading from the same UI. They are synchronized by being driven by the same BT runtime.

Expected runtime flow:

```text
BT tick loop
  -> updates /tick_id blackboard key
  -> BB bridge publishes /bt/marathon/bb/tick_id
  -> tree publisher updates /marathon_bt/log/tree and rqt view
  -> page backend (in rosa-agent) receives tick_id via rospy subscriber
  -> browser status bar polls /api/current_tick and updates
```

In pause/step mode:

```text
BT stops advancing
rqt tree view stays on the paused state
/bt/marathon/bb/tick_id stays at the paused tick
status bar shows the same tick and transitions to paused/stale after 10 s
```

This is desired behavior.

---

## 11. User Workflow

1. User opens the page at `http://<pi-ip>:8090`.
2. User watches rqt in the noVNC iframe.
3. User sees current tick id in the narrow status bar.
4. User uses ROSA terminal below.
5. Example prompt:

```text
分析 tick 1618。机器人现实表现：还在往前走，头没有动。rqt 显示 FindLine。
```

or:

```text
分析当前 tick。机器人现实表现：停止了，头在扫，但地上有线没有进入 FollowLine。
```

ROSA should use `analyze_bt_tick` for this query.

---

## 12. Failure Modes

### 12.1 noVNC iframe blank

Likely causes:

```text
Xvfb not running in ainex
rqt not running on DISPLAY=:99
x11vnc not attached to DISPLAY=:99
websockify/noVNC not running
noVNC packages not installed (apt-get step skipped after container recreation)
wrong noVNC URL (path confirmed: /usr/share/novnc/vnc.html)
```

### 12.2 ROSA terminal blank

Likely causes:

```text
ttyd not running (start_all.sh not executed)
ainex_agent.py failed to start
missing OPENAI/API config in .env
ROS master not reachable from rosa-agent container
```

### 12.3 Tick shows waiting

Likely causes:

```text
BT node not running
MarathonBBBridge not publishing /bt/marathon/bb/tick_id
app.py not running in rosa-agent
rospy.init_node failed (check container logs)
topic type mismatch
```

### 12.4 Tick shows paused/stale

This can be normal in pause/step mode.

Investigate only if the user expects continuous run mode.

---

## 13. Verification

### 13.1 ROS Topic Check

```bash
docker exec rosa-agent bash -c \
  'source /opt/ros/noetic/setup.bash && rostopic echo -n 1 /bt/marathon/bb/tick_id'
```

Expected:

```text
data: "1618"
```

### 13.2 Backend Check

```bash
curl http://localhost:8090/api/current_tick
```

Expected (BT running):

```json
{"ok": true, "tick_id": 1618, "status": "live", ...}
```

Expected (BT not running):

```json
{"ok": false, "tick_id": null, "status": "waiting", ...}
```

### 13.3 Service Health Check

```bash
curl -s -o /dev/null -w "%{http_code}" http://localhost:6080/vnc.html   # 200
curl -s -o /dev/null -w "%{http_code}" http://localhost:7681             # 200
curl -s -o /dev/null -w "%{http_code}" http://localhost:8090/            # 200
```

### 13.4 ROSA One-Tick Query Check

In the embedded terminal:

```text
分析当前 tick。机器人现实表现：头没有动，rqt 显示 FindLine。
```

Expected ROSA behavior:

```text
calls analyze_bt_tick
outputs explain_tick / compare_tick / diagnose_tick
does not treat ros_out as physical execution proof
```

---

## 14. Future Enhancements

After MVP:

```text
1. Add active leaf to status bar by subscribing /marathon_bt/log/tree or /marathon_bt/ascii/snapshot.
2. Add BT mode by subscribing /marathon_bt/bt/mode.
3. Add a "copy prompt" button:
   "分析 tick <id>。机器人现实表现：..."
4. Replace ROSA terminal iframe with a proper chat panel.
5. Add a one-click Analyze Current Tick form that calls analyze_bt_tick directly.
6. Commit ainex container as ainex-backup:novnc after noVNC install is stable.
7. Add ROSBridge/WebSocket frontend path if direct browser ROS subscriptions are preferred later.
```

MVP should avoid these until the basic page is stable.

---

## 15. Implementation Notes (Apr 12 2026)

Deviations from the original design and rationale:

### 15.1 Page backend location

Original design said: "page runs on the host Raspberry Pi".

Actual: **runs inside rosa-agent container**.

Reason: host Pi has no `rospy`. ROS Noetic arm64 apt packages are unavailable (404). rosa-agent already has Python 3.9, rospy via PYTHONPATH, and host networking — it is the natural host for the backend.

The source is edited on the host and volume-mounted into the container; there is no workflow change for development.

### 15.2 ttyd binary filename

`ttyd.aarch64`, not `ttyd_linux.aarch64`. The GitHub release asset name differs from what the docs implied. Confirmed from the GitHub API: `https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64`.

### 15.3 noVNC install is ephemeral

`Xvfb`, `x11vnc`, `novnc`, `python3-websockify` are installed via `apt-get` into the running `ainex` container. They are not in the base image `ainex-backup:20260308`. They will be lost on container recreation. Plan: run `docker commit ainex ainex-backup:novnc` after the page is confirmed stable.

### 15.4 rospy + uvicorn coexistence

`rospy.spin()` runs in a `threading.Thread(daemon=True)`. `rospy.init_node` uses `disable_signals=True` to prevent rospy from overriding uvicorn's SIGINT/SIGTERM handlers. Without this flag, Ctrl-C would be intercepted by rospy before uvicorn can handle graceful shutdown.

### 15.5 start_all.sh

The `start_runtime_page.sh` suggested in §4 was implemented as `start_all.sh` covering all four service groups (Xvfb, rqt, x11vnc, websockify + ttyd + app.py).
