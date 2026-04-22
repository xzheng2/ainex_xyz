# YOLO on Ainex Robot (Raspberry Pi 5)

## Hardware
- Pi 5: 4x Cortex-A76 @ 2.4GHz, 8GB RAM, NEON FP16, **no GPU/NPU**

## Installation (Host, NOT container)
- **Runs on host** (Debian 12, Python 3.11) — NOT inside ainex container
- Ultralytics **8.4.40**, PyTorch 2.1.2, OpenCV 4.9.0, NCNN 1.0.20260316
- Model: `yolo26n.pt` (5.3 MB) at `/home/pi/yolo26n.pt` (YOLO26n, Jan 2026, 2.4M params, mAP 40.9)
- NCNN export: `/home/pi/yolo26n_ncnn_model/` (9.3 MB, exported at imgsz=320)
- **Replaced yolov8n (Apr 20 2026)**: YOLO26n is NMS-free, ~43% faster, higher accuracy

## Performance (320x320)

| Backend | Speed | FPS | Accuracy |
|---------|-------|-----|----------|
| **NCNN** | **149ms** | **6.7** | Correct |
| PyTorch | 776ms | 1.3 | Correct |

- NCNN is **5.2x faster** than PyTorch on this hardware
- Keep input at 320x320 for real-time; 640x640 will halve FPS

## ROS Bridge (Host → Container)
- `roslibpy 2.0.0` installed on host (pure Python, no ROS needed)
- `rosbridge_server` already installed in container (`ros-noetic-rosbridge-server`)
- Start bridge: `docker exec ainex bash -c "source /opt/ros/noetic/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch"`
- Connect from host: `roslibpy.Ros(host='127.0.0.1', port=9090)`
- Latency: ~1–5ms local WebSocket

## Camera Script
- **Script**: `/home/pi/yolo_camera.py` — subscribes to ROS camera via rosbridge, runs YOLO, shows bounding boxes
- Camera topic: `/camera/image_raw/compressed` (throttled: `throttle_rate=100`, `queue_length=1`)
- **Drop-frame pattern**: callback only stores latest frame, main loop grabs newest → no lag
- `CONF_THRESHOLD = 0.4` — minimum confidence
- `DETECT_CLASSES` list filters which objects to detect (subset of 80 COCO classes, passed as `classes=DETECT_INDICES` to model)
- `ALL_COCO_CLASSES` has full 80-class reference
- **Topic**: `DETECTIONS_TOPIC = "/yolo/detections"` (NOT `/object/pixel_coords` — that's exclusively for LineDetectionAdapter; using it caused camera_lost_count corruption)
- `ANNOTATED_TOPIC = "/yolo/image_annotated/compressed"` (unchanged)
- Check rosbridge running: `docker exec ainex bash -c "source /opt/ros/noetic/setup.bash && rosnode list | grep rosbridge"`
- Start rosbridge: `docker exec ainex bash -c "kill \$(lsof -ti:9090) 2>/dev/null; sleep 1; source /opt/ros/noetic/setup.bash && nohup roslaunch rosbridge_server rosbridge_websocket.launch > /tmp/rosbridge.log 2>&1 &"`
- Run: `python3 ~/yolo_camera.py` (press `q` or Ctrl+C to stop)

## BT Integration (Phase 2, Apr 20 2026)
- **`ObjectDetectionAdapter`**: `ainex_bt_edu/input_adapters/object_detection_adapter.py`
  - Subscribes to `/yolo/detections`, filters out `type == 'line'` objects
  - Two-phase latch: `snapshot_and_reset()` under lock, `write_snapshot()` after
  - Writes: `BB.DETECTED_OBJECTS` (`/latched/detected_objects`, list), `BB.DETECTED_COUNT` (`/latched/detected_count`, int)
- **`L1_Vision_IsObjectDetected`**: `ainex_bt_edu/behaviours/L1_perception/L1_Vision_IsObjectDetected.py`
  - Pure BB read, no rospy.Subscriber
  - `detected_count > 0` → SUCCESS
- **BB keys** in `blackboard_keys.py`: `DETECTED_OBJECTS_KEY='detected_objects'`, `DETECTED_COUNT_KEY='detected_count'`, `DETECTED_OBJECTS='/latched/detected_objects'`, `DETECTED_COUNT='/latched/detected_count'`
- **marathon_bt_node.py**: wired with 3rd adapter (import + `__init__` + both `run()` phases)
- **bb_ros_bridge.py**: `/latched/detected_count` → `/bt/marathon/bb/detected_count` (count only; list too large)
- **infra_manifest.py**: `/yolo/detections` topic_sub record added

## Key Lessons
- **ultralytics 8.1.2 NCNN export was broken** — produced garbage detections (hundreds of false positives, all conf=1.00). Upgrading to 8.4.22 fixed it
- ONNX export failed due to `typing_extensions` / `onnx` version conflict (not critical since NCNN is faster)
- Host Python 3.11 is significantly faster than container Python 3.8 for inference

## D6A Action Files
- `.d6a` files are **SQLite databases** with table `ActionGroup`
- Each row = keyframe: col[1] = duration (ms), col[2+i] = servo i+1 position
- Reader: `/home/ubuntu/software/ainex_controller/action_group_controller.py`
- Action files: `/home/ubuntu/software/ainex_controller/ActionGroups/`
- Inspect: `docker exec ainex sqlite3 <file>.d6a ".schema" "SELECT * FROM ActionGroup LIMIT 3;"`
