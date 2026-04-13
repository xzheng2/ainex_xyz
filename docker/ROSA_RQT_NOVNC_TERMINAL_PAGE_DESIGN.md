# ROSA + RQT noVNC Single Page Design

> Goal: Put Ainex rqt runtime display and ROSA interaction into one lightweight browser page.
> UI strategy: Embed rqt through noVNC iframe; embed ROSA interaction as an inline chat panel (not an iframe).
> Tick source: subscribe to ROS topic `/bt/marathon/bb/tick_id`, not screen-scrape rqt and not read JSONL for the status bar.
> Deployment: the page backend runs inside the **rosa-agent container** (host Pi has no `rospy`); ainex ROS and ROSA run in separate Docker containers.

**Status: MVP implemented and working (Apr 12 2026). Start: `./runtime_debug_page/start_all.sh`**

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
│ ROSA chat panel (inline, not iframe)         │
└──────────────────────────────────────────────┘
```

The page should let a weak-programming user:

1. Watch rqt tree view live.
2. See the current BT tick id in a narrow synchronized status row.
3. Interact with ROSA in a chat panel below the rqt view.
4. Use the tick id shown in the status bar when asking ROSA to analyze a pause/step tick.

---

## 2. Design Decision

### 2.1 Use noVNC for rqt

rqt is a Qt desktop application. The lightest browser integration is to keep rqt unchanged and remote-display it:

```text
rqt -> Xvfb DISPLAY -> x11vnc -> websockify/noVNC -> browser iframe
```

Do not rewrite rqt as a web UI for MVP.

### 2.2 Use Inline Chat Panel for ROSA

ROSA interaction is implemented as an inline HTML chat panel — a `<pre>` output area plus a text `<input>` box — connected via a WebSocket to a PTY subprocess spawned by the backend.

```text
browser input box
  -> WebSocket /terminal/ws (same-origin, port 8090)
  -> app.py: pty.openpty() + subprocess.Popen(ainex_agent.py)
  -> PTY output streamed back via WebSocket
  -> displayed in <pre id="rosa-output">
```

**No iframe, no ttyd.** The backend spawns `ainex_agent.py` directly using Python's `pty` module. The browser JS uses the same ttyd wire protocol (`"0"` prefix = data) for compatibility, but there is no ttyd binary involved at runtime.

This avoids two problems that affected the iframe approach:
- Cross-origin keyboard blocking (iframe on port 7681 ≠ page on port 8090).
- ttyd 1.7.7 not spawning the child process on this container (confirmed by inspection — ttyd accepted WebSocket connections but never executed the child command).

### 2.3 Status Bar Tick Source: ROS Topic

The status bar subscribes to:

```text
/bt/marathon/bb/tick_id
```

Type: `std_msgs/String`. The backend parses both plain numeric strings and JSON values.

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
  │   └── #rosa-chat div: WebSocket to /terminal/ws
  │
  └── Docker CLI access
      ├── starts/stops rqt/noVNC processes in ainex container
      └── starts/stops page backend in rosa-agent container

Ainex Docker Container (host network)
  ├── ROS master / Ainex BT node
  ├── publishes /bt/marathon/bb/tick_id
  ├── Xvfb :99
  ├── rqt (DISPLAY=:99)
  ├── x11vnc (port 5910)
  └── websockify/noVNC (port 6080)

ROSA Agent Docker Container (host network)
  ├── ainex_agent.py (PID 1, entrypoint)
  └── app.py — FastAPI page backend (port 8090)
      ├── rospy subscriber: /bt/marathon/bb/tick_id
      ├── GET /api/current_tick
      ├── WebSocket /terminal/ws — spawns ainex_agent.py in PTY
      └── serves static/ (index.html, style.css, app.js)
```

Logical browser layout:

```text
Browser Page  (served from port 8090)
  ├── iframe: noVNC rqt desktop (port 6080)
  ├── status bar: polls /api/current_tick every 500 ms
  └── #rosa-chat: WebSocket /terminal/ws (same-origin)
```

### 3.1 Container Boundary

The page backend runs inside the **rosa-agent container**, not on the host Pi.

Reason: the host Pi does not have `rospy` installed, and the ROS Noetic apt repo arm64 packages are no longer available (404). Installing ROS on the host is not feasible.

The rosa-agent container provides:

```text
- Python 3.9
- rospy (via PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages)
- host networking (shares 127.0.0.1 with ROS master at port 11311)
- fastapi + uvicorn (requirements.txt)
- ainex_agent.py at /opt/rosa-agent/ainex_agent.py
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
├── app.py            ← FastAPI backend (rospy subscriber + API + PTY terminal WebSocket)
├── static/
│   ├── index.html    ← 3-panel layout (rqt iframe + status bar + #rosa-chat div)
│   ├── style.css     ← CSS grid + chat panel styles
│   └── app.js        ← tick polling + WebSocket ROSA chat
├── start_all.sh      ← one-shot startup script (runs all services from host)
└── README.md         ← startup, verification, rqt control, troubleshooting
```

### 4.2 Container changes

```text
docker/rosa-agent/requirements.txt
  - added: fastapi>=0.111.0
  - added: uvicorn>=0.30.0

docker/docker-compose.yml
  - added volume: /home/pi/runtime_debug_page:/opt/ainex_page:ro
```

No additional packages are needed for the PTY terminal. Python's `pty`, `fcntl`, `termios`, `struct`, `subprocess` are all standard library.

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

      <!-- ROSA chat panel — inline div, not an iframe -->
      <div id="rosa-chat">
        <pre id="rosa-output"></pre>
        <div id="rosa-input-bar">
          <input id="rosa-input" type="text"
                 placeholder="Type your ROSA query and press Enter…"
                 autocomplete="off" autofocus />
          <button id="rosa-send">Send ↵</button>
        </div>
      </div>
    </div>
    <script src="/static/app.js"></script>
  </body>
</html>
```

`app.js` sets the rqt iframe URL using the browser's current hostname:

```js
const host = window.location.hostname;
document.getElementById("rqt-frame").src =
  "http://" + host + ":6080/vnc.html?autoconnect=true&resize=scale";
```

The ROSA chat panel connects via WebSocket to the same origin (port 8090):

```js
const proto = location.protocol === "https:" ? "wss:" : "ws:";
const wsUrl  = proto + "//" + location.host + "/terminal/ws";
ws = new WebSocket(wsUrl, ["tty"]);
```

### 5.2 CSS Layout

```css
#layout {
  height: 100vh;
  display: grid;
  grid-template-rows: minmax(360px, 1fr) 32px 260px;
  background: #0f1115;
}

#rosa-chat {
  display: flex;
  flex-direction: column;
  background: #0d1117;
}

#rosa-output {
  flex: 1;
  overflow-y: auto;
  padding: 8px 12px;
  font-family: ui-monospace, monospace;
  font-size: 13px;
  color: #c9d1d9;
  white-space: pre-wrap;
}

#rosa-input-bar {
  display: flex;
  height: 36px;
  border-top: 1px solid #30343b;
  background: #151922;
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
6. Expose `WebSocket /terminal/ws` — spawns `ainex_agent.py` in a PTY and proxies I/O.

### 6.2 PTY Terminal WebSocket

```python
_AGENT_CMD = ["python3.9", "/opt/rosa-agent/ainex_agent.py"]

@app.websocket("/terminal/ws")
async def terminal_ws_pty(ws: WebSocket):
    await ws.accept(subprotocol="tty")

    master_fd, slave_fd = pty.openpty()
    fcntl.ioctl(master_fd, termios.TIOCSWINSZ, struct.pack("HHHH", 24, 80, 0, 0))
    proc = subprocess.Popen(
        _AGENT_CMD,
        stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
        close_fds=True, start_new_session=True,
    )
    os.close(slave_fd)

    loop = asyncio.get_event_loop()

    async def forward_output():
        while True:
            try:
                data = await loop.run_in_executor(None, lambda: os.read(master_fd, 4096))
                if data:
                    await ws.send_text("0" + data.decode("utf-8", errors="replace"))
            except OSError:
                break

    fwd = asyncio.create_task(forward_output())
    try:
        while True:
            msg = await ws.receive()
            if msg["type"] == "websocket.disconnect":
                break
            text = msg.get("text") or ""
            if not text:
                continue
            cmd, body = text[0], text[1:]
            if cmd == "0":       # input from browser
                os.write(master_fd, body.encode("utf-8"))
            elif cmd == "1":     # terminal resize
                size = json.loads(body)
                fcntl.ioctl(master_fd, termios.TIOCSWINSZ,
                            struct.pack("HHHH", size.get("rows", 24), size.get("cols", 80), 0, 0))
    finally:
        fwd.cancel()
        proc.kill(); proc.wait(timeout=3)
        os.close(master_fd)
```

Wire protocol (ttyd-compatible):
- Server → client: `"0"` + UTF-8 text = terminal output
- Client → server: `"0"` + text = user input (append `"\r"` for Enter)
- Client → server: `"1"` + JSON `{"cols": N, "rows": M}` = resize

### 6.3 Backend State

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

Stale threshold: `STALE_SECONDS = 10.0`. Stale is allowed (pause/step mode).

---

## 7. Frontend JavaScript

### 7.1 Tick Polling

```js
async function refreshTick() {
  const res  = await fetch("/api/current_tick", { cache: "no-store" });
  const data = await res.json();
  // update #tick-id, #tick-status, #tick-age
}
setInterval(refreshTick, 500);
```

### 7.2 ROSA Chat Panel WebSocket

```js
const proto = location.protocol === "https:" ? "wss:" : "ws:";
const wsUrl  = proto + "//" + location.host + "/terminal/ws";
ws = new WebSocket(wsUrl, ["tty"]);
ws.binaryType = "arraybuffer";

ws.onopen = function () {
  ws.send("1" + JSON.stringify({ cols: 90, rows: 20 }));  // resize
};

ws.onmessage = function (e) {
  const str = e.data instanceof ArrayBuffer
    ? new TextDecoder().decode(e.data) : e.data;
  if (str[0] === "0") appendOutput(str.slice(1));
};

function sendInput() {
  const text = input.value;
  input.value = "";
  appendOutput(text + "\n");            // local echo
  ws.send("0" + text + "\r");          // "0" = input; \r = Enter
}
```

ANSI codes from the PTY are stripped by `stripAnsi()` before displaying in the `<pre>`. `\r\n` is normalized to `\n`.

---

## 8. Runtime Services

### 8.1 rqt via noVNC (ainex container)

**noVNC packages are not in the `ainex-backup:20260308` image. Install once into the running container:**

```bash
docker exec ainex apt-get update
docker exec ainex apt-get install -y xvfb x11vnc novnc python3-websockify
```

These installs are **ephemeral** — lost if the container is recreated. After debugging stabilises, commit:

```bash
docker commit ainex ainex-backup:novnc
```

Start commands (from host):

```bash
docker exec -d ainex bash -c 'Xvfb :99 -screen 0 1600x900x24 &'
docker exec -d ainex bash -c 'sleep 1 && DISPLAY=:99 rqt &'
docker exec -d ainex bash -c 'sleep 2 && x11vnc -display :99 -forever -shared -nopw -rfbport 5910 &'
docker exec -d ainex bash -c 'sleep 3 && websockify --web /usr/share/novnc 6080 localhost:5910 &'
```

**Note**: port 5910 is used instead of the default 5900. The host Pi runs `wayvnc` which permanently occupies 5900. x11vnc binds to 5910; websockify proxies from 5910.

### 8.2 ROSA Chat Panel (rosa-agent container)

The chat panel backend is built into `app.py` — it spawns `ainex_agent.py` directly in a PTY when a browser WebSocket connects. No separate service or port needed.

### 8.3 Page Backend (rosa-agent container)

```bash
docker exec -it rosa-agent python3.9 /opt/ainex_page/app.py
```

Port: `8090`. Browser opens `http://<pi-host>:8090`.

No `ROS_MASTER_URI` export needed — the container already has it set.

### 8.4 One-Shot Startup

```bash
./runtime_debug_page/start_all.sh
```

Starts four service groups: Xvfb, rqt, x11vnc, websockify. Then runs the page backend in the foreground (Ctrl-C to stop).

---

## 9. Runtime rqt Page Control via CLI

The MVP leaves room to modify the rqt view while the page is running.

Do not implement browser buttons for this in MVP. Use host CLI commands:

```text
Host CLI
  -> docker exec ainex ...
  -> starts/stops/restarts rqt on DISPLAY=:99
  -> noVNC iframe keeps showing DISPLAY=:99
```

### 9.1 Example Commands

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
  'pkill -f websockify || true; websockify --web /usr/share/novnc 6080 localhost:5910 &'
```

---

## 10. Synchronization Semantics

The status bar tick id and rqt tree view are driven by the same BT runtime, not synchronized via UI inspection.

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
4. User types queries in the ROSA input box below.
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

```text
Xvfb not running in ainex
rqt not running on DISPLAY=:99
x11vnc not attached to DISPLAY=:99
websockify/noVNC not running
noVNC packages not installed (apt-get step skipped after container recreation)
wrong noVNC URL (path confirmed: /usr/share/novnc/vnc.html)
```

### 12.2 ROSA chat panel blank / "[Connecting…]" stays

```text
app.py not running in rosa-agent container
ainex_agent.py failed to start (check container logs)
missing OPENAI/API config in .env
ROS master not reachable from rosa-agent container
```

### 12.3 Tick shows waiting

```text
BT node not running
MarathonBBBridge not publishing /bt/marathon/bb/tick_id
app.py not running in rosa-agent
rospy.init_node failed (check container logs)
topic type mismatch
```

### 12.4 Tick shows paused/stale

Normal in pause/step mode. Investigate only if continuous run mode is expected.

---

## 13. Verification

### 13.1 ROS Topic Check

```bash
docker exec rosa-agent bash -c \
  'source /opt/ros/noetic/setup.bash && rostopic echo -n 1 /bt/marathon/bb/tick_id'
```

### 13.2 Backend Check

```bash
curl http://localhost:8090/api/current_tick
```

### 13.3 Service Health Check

```bash
curl -s -o /dev/null -w "%{http_code}" http://localhost:6080/vnc.html   # 200
curl -s -o /dev/null -w "%{http_code}" http://localhost:8090/            # 200
```

### 13.4 ROSA Chat Check

Open browser at `http://<pi-ip>:8090`. The ROSA banner should appear in the chat panel within a few seconds of page load. Type a query and press Enter — the response should stream in.

---

## 14. Future Enhancements

```text
1. Add active leaf to status bar by subscribing /marathon_bt/log/tree or /marathon_bt/ascii/snapshot.
2. Add BT mode by subscribing /marathon_bt/bt/mode.
3. Add a "copy prompt" button: "分析 tick <id>。机器人现实表现：..."
4. Add a one-click Analyze Current Tick form that calls analyze_bt_tick directly.
5. Commit ainex container as ainex-backup:novnc after noVNC install is stable.
6. Add ROSBridge/WebSocket frontend path if direct browser ROS subscriptions are preferred later.
```

---

## 15. Implementation Notes

### 15.1 Page backend in container

Host Pi has no `rospy`. ROS Noetic arm64 apt packages are unavailable (404). rosa-agent already has Python 3.9, rospy via PYTHONPATH, and host networking — it is the natural host for the backend. Source edited on host, mounted read-only into container; no workflow change for development.

### 15.2 noVNC install is ephemeral

`Xvfb`, `x11vnc`, `novnc`, `python3-websockify` installed via `apt-get` into running `ainex` container; not in base image `ainex-backup:20260308`. Run `docker commit ainex ainex-backup:novnc` after stable.

### 15.3 rospy + uvicorn coexistence

`rospy.spin()` runs in a `threading.Thread(daemon=True)`. `rospy.init_node` uses `disable_signals=True` to prevent rospy from overriding uvicorn's SIGINT/SIGTERM handlers.

### 15.4 Port 5910 for x11vnc

Default VNC port 5900 is permanently occupied by `wayvnc` on the host Pi. All containers use host networking, so x11vnc uses 5910 instead; websockify proxies from 5910.

### 15.5 PTY approach instead of ttyd

**Original plan**: embed ROSA as a ttyd iframe at port 7681.

**Problems encountered**:
1. **Cross-origin keyboard block**: iframe on port 7681 ≠ page on port 8090 → browsers block keyboard events in cross-origin iframes.
2. **ttyd 1.7.7 does not spawn the child process**: ttyd accepted WebSocket connections and sent the title message but never executed `ainex_agent.py`. No child process appeared in `ps`. The cause was confirmed by running test cases with both `echo` and `bash` commands — same result. The ttyd binary is present in the container but non-functional as a process spawner on this platform.

**Fix**: bypass ttyd entirely. `app.py` spawns `ainex_agent.py` directly via `pty.openpty()` + `subprocess.Popen` in a FastAPI WebSocket handler. The ROSA UI is an inline HTML chat panel (`<div id="rosa-chat">`) connected to `/terminal/ws` on the same origin (port 8090). Confirmed working: ROSA banner appears within ~2 seconds of WebSocket connect; queries typed in the input box are sent and responses stream back.
