"""
AINEX Runtime Debug Page — FastAPI backend.

Runs inside the rosa-agent container (Python 3.9, rospy on PYTHONPATH).
  - Subscribes /bt/marathon/bb/tick_id (std_msgs/String)
  - Exposes GET /api/current_tick
  - Serves static/ (index.html, style.css, app.js)
  - Spawns ainex_agent.py in a PTY at /terminal/ws (direct, no ttyd dependency)

Start:
    python3.9 /opt/ainex_page/app.py
Port: 8090
"""

import asyncio
import fcntl
import json
import os
import pty
import struct
import subprocess
import termios
import threading
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

# ── ROS subscriber ────────────────────────────────────────────────────────────

STALE_SECONDS = 10.0
_SOURCE = "/bt/marathon/bb/tick_id"

LATEST_TICK: dict = {
    "ok": False,
    "tick_id": None,
    "raw": None,
    "source": _SOURCE,
    "updated_at": None,
    "error": "no tick received yet",
}
_lock = threading.Lock()


def _parse_tick(data: str):
    """Return int tick or raise ValueError."""
    data = data.strip()
    try:
        val = json.loads(data)
        if isinstance(val, int):
            return val
        if isinstance(val, str):
            return int(val)
        # e.g. float — truncate
        return int(val)
    except (json.JSONDecodeError, ValueError):
        pass
    return int(data)


def _tick_callback(msg):
    raw = msg.data
    now = time.time()
    try:
        tick_id = _parse_tick(raw)
        with _lock:
            LATEST_TICK.update({
                "ok": True,
                "tick_id": tick_id,
                "raw": raw,
                "source": _SOURCE,
                "updated_at": now,
                "error": None,
            })
    except (ValueError, TypeError) as exc:
        with _lock:
            LATEST_TICK.update({
                "ok": False,
                "tick_id": None,
                "raw": raw,
                "source": _SOURCE,
                "updated_at": now,
                "error": f"parse error: {exc}",
            })


def _start_ros():
    import rospy
    from std_msgs.msg import String  # noqa: PLC0415

    rospy.init_node("ainex_page_backend", anonymous=True, disable_signals=True)
    rospy.Subscriber(_SOURCE, String, _tick_callback)
    try:
        rospy.spin()
    except Exception:
        pass


# ── FastAPI app ───────────────────────────────────────────────────────────────

app = FastAPI(title="AINEX Runtime Debug", docs_url=None, redoc_url=None)

_static_dir = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(_static_dir)), name="static")


@app.get("/")
async def index():
    from fastapi.responses import FileResponse  # noqa: PLC0415
    return FileResponse(str(_static_dir / "index.html"))


@app.get("/api/current_tick")
async def current_tick():
    with _lock:
        snap = dict(LATEST_TICK)

    now = time.time()
    updated_at = snap.get("updated_at")

    if updated_at is None:
        age_seconds = None
        stale = True
        status = "waiting"
    else:
        age_seconds = round(now - updated_at, 2)
        stale = age_seconds > STALE_SECONDS
        status = "paused/stale" if stale else "live"

    return JSONResponse({
        "ok": snap["ok"],
        "tick_id": snap["tick_id"],
        "raw": snap["raw"],
        "source": snap["source"],
        "age_seconds": age_seconds,
        "stale": stale,
        "status": status,
        "error": snap.get("error"),
    })


# ── ROSA agent PTY terminal ───────────────────────────────────────────────────
# Spawns ainex_agent.py directly in a PTY — no ttyd dependency.
# Browser JS sends "1"+JSON for resize, "0"+text for input.
# Server sends "0"+text for output (ttyd-compatible wire format).

_AGENT_CMD = ["python3.9", "/opt/rosa-agent/ainex_agent.py"]


@app.websocket("/terminal/ws")
async def terminal_ws_pty(ws: WebSocket):
    """Spawn ainex_agent.py in a PTY and proxy I/O to the browser."""
    await ws.accept(subprotocol="tty")

    # Allocate PTY and spawn agent
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
        """Read PTY output in executor thread and forward to browser."""
        while True:
            try:
                data = await loop.run_in_executor(
                    None, lambda: os.read(master_fd, 4096)
                )
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
            if cmd == "0":          # input from browser
                try:
                    os.write(master_fd, body.encode("utf-8"))
                except OSError:
                    break
            elif cmd == "1":        # terminal resize
                try:
                    size = json.loads(body)
                    rows = int(size.get("rows", 24))
                    cols = int(size.get("cols", 80))
                    fcntl.ioctl(master_fd, termios.TIOCSWINSZ,
                                struct.pack("HHHH", rows, cols, 0, 0))
                except Exception:
                    pass
    finally:
        fwd.cancel()
        try:
            proc.kill()
            proc.wait(timeout=3)
        except Exception:
            pass
        try:
            os.close(master_fd)
        except Exception:
            pass


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ros_thread = threading.Thread(target=_start_ros, daemon=True, name="ros-spin")
    ros_thread.start()
    uvicorn.run(app, host="0.0.0.0", port=8090, log_level="info")
