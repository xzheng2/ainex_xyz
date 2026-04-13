#!/usr/bin/env bash
# launch_debug_page.sh — open lxterminal running start_all.sh, then open browser

# Open terminal with start_all.sh (logs visible, Ctrl-C to stop backend)
lxterminal --title="AINEX Debug Page" \
  -e bash -c "/home/pi/runtime_debug_page/start_all.sh; read -p 'Backend stopped. Press Enter to close...'" &

# Wait for backend to come up, then open browser
sleep 5
chromium-browser http://localhost:8090 &
