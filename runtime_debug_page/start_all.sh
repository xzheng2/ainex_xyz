#!/usr/bin/env bash
# start_all.sh — start the full AINEX runtime debug page stack
#
# Run from the host Pi. Requires:
#   - ainex container running (ROS + BT node active)
#   - rosa-agent container running (rebuilt with ttyd + fastapi)
#
# Services started:
#   Port 6080  noVNC (rqt via Xvfb in ainex container)
#   Port 7681  ttyd ROSA terminal (in rosa-agent container)
#   Port 8090  runtime debug page backend (in rosa-agent container)
#
# Open browser: http://<pi-ip>:8090

set -e

# ── 1. noVNC stack in ainex ───────────────────────────────────────────────────
echo "[1/5] Starting Xvfb in ainex..."
docker exec -d ainex bash -c 'Xvfb :99 -screen 0 1600x900x24 &' 2>/dev/null || true

echo "[2/5] Starting rqt on DISPLAY=:99 in ainex..."
docker exec -d ainex bash -c 'sleep 1 && DISPLAY=:99 rqt &' 2>/dev/null || true

echo "[3/5] Starting x11vnc in ainex..."
# Port 5910 used instead of default 5900 — wayvnc on the host occupies 5900.
docker exec -d ainex bash -c 'sleep 2 && x11vnc -display :99 -forever -shared -nopw -rfbport 5910 &' 2>/dev/null || true

echo "[4/5] Starting websockify/noVNC in ainex (port 6080)..."
docker exec -d ainex bash -c 'sleep 3 && websockify --web /usr/share/novnc 6080 localhost:5910 &' 2>/dev/null || true

# ── 2. ROSA web terminal ──────────────────────────────────────────────────────
echo "[5/5] Starting ttyd ROSA terminal (port 7681) in rosa-agent..."
docker exec -d rosa-agent ttyd -p 7681 python3.9 /opt/rosa-agent/ainex_agent.py 2>/dev/null || true

# ── 3. Page backend ───────────────────────────────────────────────────────────
# Run in foreground so logs are visible. Ctrl-C to stop.
echo ""
echo "Starting page backend (port 8090) in rosa-agent — Ctrl-C to stop"
echo "Open:  http://$(hostname -I | awk '{print $1}'):8090"
echo ""
docker exec -it rosa-agent python3.9 /opt/ainex_page/app.py
