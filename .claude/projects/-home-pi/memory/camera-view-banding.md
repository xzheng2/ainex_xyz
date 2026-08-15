---
name: camera-view-banding
description: Servo-driven upper/lower camera view banding system (gemini305_view_bridge.py + ZMQ servo state)
metadata: 
  node_type: memory
  type: project
  originSessionId: ab66531b-2f87-4591-9d55-80dc425c8dcb
---

Single Gemini 305 on a tilting head (servo 24) is split into two logical views composited into ONE 640x640 `/dev/shm/gemini_color`:
- upper/front view = rows 0..479 when servo 24 > 320
- lower/down view = rows 160..639 when servo 24 <= 320
- overlap rows 160..479 = last-writer-wins
Threshold 320 with ±5 hysteresis (`TILT_THRESHOLD`/`TILT_HYST`).

**Host producer** `/home/pi/gemini305_view_bridge.py` — fork of `gemini305_bridge.py` (which stays untouched as reference). Run INSTEAD of it (one camera owner). Keeps `DoubleBuffer` ("01") format, depth (gemini_depth 640x480) + intrinsics + notify ports 5552/5553/5554 identical. Color is an in-RAM 640x640 composite written whole each frame; band chosen by servo 24.

**Rendering semantics (user requirement, do NOT change):** composite persists across frames; a band never visited stays BLACK (initial zeros); a band not updated this frame KEEPS its last frame. No mirror-to-both fallback.

**Servo transport = ZMQ, NOT rosbridge** (rosbridge/roslibpy was dropped — it was fragile; rosbridge being down left the top band black). New container node `ainex_peripherals/scripts/servo_state_zmq_bridge.py` subscribes `/ros_robot_controller/bus_servo/state`, PUB connects out to `tcp://172.17.0.1:5555`, sends JSON `{"ts":..,"pos":{"<id>":pos}}`. Host `gemini305_view_bridge.py` BINDs SUB on `tcp://*:5555`, drains-to-latest each frame, caches servo 24 (last-known; None only before first packet). Added to `ainex_peripherals/launch/usb_cam.launch` (respawn). Container zmq lives in `/home/ubuntu/.local` (ubuntu user) — roslaunch runs nodes as ubuntu so it resolves; `docker exec` as root does NOT find zmq.

`ros_robot_controller_node.py` `~state_rate_hz` default raised 10.0 → 50.0 (fresh servo-24 value per 30fps frame); needs catkin build + controller restart to apply.

Caveats: `head_camera.yaml` still 640x480 so `/camera/camera_info` mismatches the 640x640 image (fine for display/YOLO, wrong for 3D projection); `gemini_depth` 640x480 no longer pixel-aligned to banded color. YOLO 640x640→320x320 letterbox is zero-pad (square). Related: the "Camera Frame Transport — /dev/shm zero-copy" section of [[MEMORY]] if present.
