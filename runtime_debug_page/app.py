"""
AINEX Runtime Debug Page — FastAPI backend.

Runs inside the rosa-agent container (Python 3.9, rospy on PYTHONPATH).
  - Subscribes /bt/marathon/bb/tick_id (std_msgs/String)
  - Exposes GET /api/current_tick
  - Serves static/ (index.html, style.css, app.js)

Start:
    python3.9 /opt/ainex_page/app.py
Port: 8090
"""

import json
import threading
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI
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


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    ros_thread = threading.Thread(target=_start_ros, daemon=True, name="ros-spin")
    ros_thread.start()
    uvicorn.run(app, host="0.0.0.0", port=8090, log_level="info")
