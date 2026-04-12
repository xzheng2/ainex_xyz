# ROSA + RQT noVNC Single Page Design

> Goal: Put Ainex rqt runtime display and ROSA interaction into one lightweight browser page.
> UI strategy: Embed existing rqt through noVNC iframe, embed ROSA CLI through web terminal iframe.
> Tick source: subscribe to ROS topic `/bt/marathon/bb/tick_id`, not screen-scrape rqt and not read JSONL for the status bar.
> Deployment: the page runs on the host Raspberry Pi; Ainex ROS and ROSA run in separate Docker containers.

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
ainex_agent.py interactive CLI -> ttyd / wetty -> browser iframe
```

This avoids writing a chat backend for MVP. The user gets the same ROSA CLI in the page.

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

or a JSON string representation depending on bridge output. The backend should parse both plain numeric strings and JSON values.

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
  │   ├── iframe: noVNC rqt desktop
  │   ├── status bar: current tick from host page backend
  │   └── iframe: ROSA web terminal
  │
  ├── Page Backend
  │   ├── serves static HTML/CSS/JS
  │   ├── subscribes /bt/marathon/bb/tick_id from Ainex ROS master
  │   ├── exposes GET /api/current_tick
  │   └── optional CLI control endpoints for rqt runtime page changes
  │
  └── Docker CLI access
      ├── starts/stops rqt/noVNC processes in the Ainex container
      └── starts/stops ROSA terminal in the ROSA container

Ainex Docker Container
  ├── ROS master / Ainex BT node
  ├── publishes /bt/marathon/bb/tick_id
  ├── Xvfb
  ├── rqt
  ├── x11vnc
  └── websockify/noVNC

ROSA Docker Container
  ├── ainex_agent.py interactive CLI
  └── ttyd or wetty web terminal
```

Logical browser layout:

```text
Browser Page
  ├── iframe: noVNC rqt desktop
  ├── status bar: current tick from page backend
  └── iframe: ROSA web terminal
```

### 3.1 Container Boundary

The page process is hosted on the Pi, not inside either Docker container.

The page does not need to own ROSA or rqt directly. It embeds:

```text
noVNC URL from Ainex container, e.g. http://<pi-host>:6080/...
ROSA terminal URL from ROSA container, e.g. http://<pi-host>:7681/...
```

The host page backend needs read access to the Ainex ROS graph for `/bt/marathon/bb/tick_id`.

MVP assumption:

```text
The Ainex ROS container uses host networking, or otherwise exposes ROS master and topic transport so the host backend can subscribe with rospy.
```

If the host does not have ROS Python available, use a tiny tick bridge in the Ainex container later. MVP keeps the selected source as the ROS topic and documents the host ROS requirement.

---

## 4. Files to Add

Suggested location:

```text
ainex_xyz-master/runtime_debug_page/
```

Create:

```text
runtime_debug_page/
├── app.py
├── static/
│   ├── index.html
│   ├── style.css
│   └── app.js
└── README.md
```

Optional later:

```text
runtime_debug_page/start_runtime_page.sh
runtime_debug_page/rqt_control.py
```

For MVP, a documented command sequence is enough.

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
      <iframe
        id="rqt-frame"
        src="">
      </iframe>

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

      <iframe
        id="rosa-terminal"
        src="">
      </iframe>
    </div>

    <script src="/static/app.js"></script>
  </body>
</html>
```

`app.js` should set iframe URLs using the browser's current host name, so the page works both on the Pi itself and from another computer on the same network:

```js
const host = window.location.hostname;
document.getElementById("rqt-frame").src =
  `http://${host}:6080/vnc.html?autoconnect=true&resize=scale`;
document.getElementById("rosa-terminal").src =
  `http://${host}:7681`;
```

Do not hardcode `localhost` in the iframe URLs unless the browser always runs directly on the Pi.

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

### 6.1 Recommended Framework

Use FastAPI because it is small and easy to serve static files plus JSON endpoints.

```text
runtime_debug_page/app.py
```

Responsibilities:

1. Serve `/` and `/static/*`.
2. Initialize `rospy` once.
3. Subscribe to `/bt/marathon/bb/tick_id`.
4. Store the latest tick in memory.
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

Parsing policy:

```python
def parse_tick_value(data: str) -> int:
    # Accept:
    #   "1618"
    #   1618 encoded as JSON
    #   "\"1618\"" encoded as JSON string
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

Response:

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

If no message received:

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
age_seconds <= STALE_SECONDS:
  status = "live"

age_seconds > STALE_SECONDS:
  status = "paused/stale"

no tick received:
  status = "waiting"
```

Do not treat stale as an error.

---

## 7. Frontend Tick Polling

### 7.1 JavaScript

```js
async function refreshTick() {
  const tickId = document.getElementById("tick-id");
  const tickStatus = document.getElementById("tick-status");
  const tickAge = document.getElementById("tick-age");

  try {
    const res = await fetch("/api/current_tick", { cache: "no-store" });
    const data = await res.json();

    tickStatus.classList.remove("status-warn", "status-error");

    if (!data.ok) {
      tickId.textContent = "?";
      tickAge.textContent = "?";
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
    tickId.textContent = "?";
    tickAge.textContent = "?";
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

### 8.1 rqt via noVNC

The rqt/noVNC stack runs in the Ainex Docker container because rqt needs the ROS packages and message types from the Ainex ROS workspace.

Typical host-side commands:

```bash
docker exec ainex bash -lc 'Xvfb :99 -screen 0 1600x900x24 &'
docker exec ainex bash -lc 'DISPLAY=:99 rqt &'
docker exec ainex bash -lc 'x11vnc -display :99 -forever -shared -rfbport 5900 &'
docker exec ainex bash -lc 'websockify --web /usr/share/novnc 6080 localhost:5900 &'
```

The page iframe points to:

```text
http://<pi-host>:6080/vnc.html?autoconnect=true&resize=scale
```

If noVNC uses a different path in the installed package, update `index.html`.

### 8.2 ROSA via Web Terminal

Use `ttyd` or `wetty`.

Recommended first choice:

```bash
docker exec rosa-agent bash -lc 'ttyd -p 7681 python3.9 /opt/rosa-agent/ainex_agent.py'
```

The page iframe points to:

```text
http://<pi-host>:7681
```

If `python3.9` is not the runtime executable in the container, use the environment's normal Python command.

### 8.3 Page Backend

Example:

```bash
cd ainex_xyz-master
python3 runtime_debug_page/app.py
```

Suggested port:

```text
8090
```

Browser opens:

```text
http://<pi-host>:8090
```

The page backend runs on the host Pi. It must be launched in an environment that can reach the Ainex ROS master and import `rospy`.

Expected environment:

```bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=<pi-host-ip>
```

Adjust `ROS_MASTER_URI` if the Ainex ROS master is not exposed through host networking.

---

## 9. Runtime rqt Page Control via CLI

The MVP should leave room to modify the rqt view while the page is running.

Do not implement browser buttons for this in MVP. Use host CLI commands that control the rqt process inside the Ainex Docker container. Because noVNC displays the same virtual X display, changing/restarting rqt inside the container is immediately reflected in the iframe.

### 9.1 Control Model

```text
Host CLI
  -> docker exec ainex ...
  -> starts/stops/restarts rqt on DISPLAY=:99
  -> noVNC iframe keeps showing DISPLAY=:99
```

The browser page remains unchanged. Only the content displayed in the noVNC iframe changes.

### 9.2 Suggested CLI Wrapper

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

MVP can start with documented shell commands instead of implementing this wrapper immediately.

### 9.3 Example Commands

Restart default rqt:

```bash
docker exec ainex bash -lc 'pkill -f "rqt" || true; DISPLAY=:99 rqt &'
```

Restart rqt with a saved perspective:

```bash
docker exec ainex bash -lc 'pkill -f "rqt" || true; DISPLAY=:99 rqt --perspective-file /root/.config/ainex/rqt_tree.perspective &'
```

Restart only noVNC bridge if the iframe disconnects:

```bash
docker exec ainex bash -lc 'pkill -f "websockify" || true; websockify --web /usr/share/novnc 6080 localhost:5900 &'
```

### 9.4 Safety Rules

CLI control should be allowlisted.

Recommended MVP rule:

```text
Only provide predefined commands for rqt/noVNC display control.
Do not expose arbitrary shell execution through the browser page.
Do not let ROSA execute these commands automatically in read-only mode.
```

The user or developer can run these commands manually on the host Pi when changing the rqt page layout.

---

## 10. Synchronization Semantics

The status bar tick id and rqt tree view are not synchronized by reading from the same UI. They are synchronized by being driven by the same BT runtime.

Expected runtime flow:

```text
BT tick loop
  -> updates /tick_id blackboard key
  -> BB bridge publishes /bt/marathon/bb/tick_id
  -> tree publisher updates /marathon_bt/log/tree and rqt view
  -> page backend receives tick_id
  -> browser status bar updates
```

In pause/step mode:

```text
BT stops advancing
rqt tree view stays on the paused state
/bt/marathon/bb/tick_id stays at the paused tick
status bar shows the same tick and may become paused/stale
```

This is desired behavior.

---

## 11. User Workflow

1. User opens the page.
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
Xvfb not running
rqt not running on DISPLAY=:99
x11vnc not attached to DISPLAY=:99
websockify/noVNC not running
wrong noVNC URL
```

### 12.2 ROSA terminal blank

Likely causes:

```text
ttyd/wetty not running
wrong iframe URL
ainex_agent.py failed to start
missing OPENAI/API config
ROS master not reachable
```

### 12.3 Tick shows waiting

Likely causes:

```text
BT node not running
MarathonBBBridge not publishing /bt/marathon/bb/tick_id
ROS_MASTER_URI wrong for page backend
backend did not initialize rospy
topic type mismatch
```

### 12.4 Tick shows paused/stale

This can be normal in pause/step mode.

Investigate only if the user expects continuous run mode.

---

## 13. MVP Verification

### 13.1 ROS Topic Check

Inside the ROS environment:

```bash
rostopic echo -n 1 /bt/marathon/bb/tick_id
```

Expected:

```text
data: "1618"
```

or equivalent JSON/string numeric representation.

### 13.2 Backend Check

```bash
curl http://<pi-host>:8090/api/current_tick
```

Expected:

```json
{"ok": true, "tick_id": 1618, ...}
```

### 13.3 Page Check

Expected:

```text
rqt iframe visible
status bar tick id updates or stays paused
ROSA terminal interactive
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
6. Add ROSBridge/WebSocket frontend path if direct browser ROS subscriptions are preferred later.
```

MVP should avoid these until the basic page is stable.
