---
name: web-data-viewer
description: "Browser ROS dashboard on :8090 (ros_launcher) — launch control + generic data viewer (rosbridge); 8081 retired"
metadata: 
  node_type: memory
  type: project
  originSessionId: 08c652ab-4469-4400-9c3f-a34d615e88a8
---

Single browser dashboard on host port **8090**: `/home/pi/ros_launcher/` — `launch_server.py`
(stdlib `ThreadingHTTPServer`) + `index.html` (its own Launch UI). Two tabs:
**Launch** (start/stop roslaunch) + **Data viewer** (generic rosbridge topic viewer, embedded via
`<iframe src="/viewer/">`). Tailscale-reachable (`http://ainex-pi:8090/`, `0.0.0.0`).
Run: `nohup bash /home/pi/ros_launcher/serve.sh >/tmp/ros_launcher.log 2>&1 &`.
Desktop shortcut: `/home/pi/Desktop/ros_launcher.desktop` → `start_desktop.sh` (idempotent start + chromium).

**The old standalone :8081 (`/home/pi/web_data_viewer/`) was RETIRED Jul 31 2026** — folded into 8090
and the folder deleted. Viewer files now live in **`/home/pi/ros_launcher/viewer/`** (`index.html`,
`bt_tree.js`, `roslib.min.js`), served by launch_server at `/viewer/…` (`VIEWER_DIR`, realpath-guarded).

## Data viewer (`ros_launcher/viewer/index.html`, served at 8090 `/viewer/`)
- `?topic=/name` (comma-separate → one panel each). roslibjs → `ws://<location.hostname>:9090`
  (no backend; relies on `rosbridge_server` on `0.0.0.0:9090`). Auto-detects type via
  `ros.getTopicType`; renders raw JSON + a horizontal-bar pointer (angle wrapped ±180°) only when
  `msgType==='std_msgs/Float64'`. No `?topic=` → index page (`ros.getTopics`) with big shortcut
  buttons (IMU axes, Camera, Battery, Servo state) + dynamic Behavior-tree buttons; full list under
  a collapsed `<details>`. Header has a ⟳ button (`location.reload()`) beside the conn badge.
- **sensor_msgs/Image** → renders `<img src="http://<host>:8080/stream?topic=<name>">` (web_video_server
  MJPEG; literal slashes, NOT encodeURIComponent) instead of dumping pixels over rosbridge.
- **py_trees_msgs/BehaviourTree** → live node-link SVG (rqt-style, no pixels): shapes from
  `rqt_py_trees/dotcode_behaviour.py` (box=Sequence, octagon=Selector/Parallel, ellipse=leaf/Decorator),
  fill = rqt (is_active,status) hex map. UUIDs arrive base64 (not int array) — handled in `bt_tree.js`.
  `bt_tree.js` = UMD: `BTTree.parse(behaviours)`→forest (children from child_ids, root=zero parent_id),
  `BTTree.layout()`→{nodes,edges,bbox}. Every BT node publishes latched `~log/tree` via `TreeROSPublisher`.
- IMU topics from `ainex_peripherals/scripts/imu_gui_node.py` (`std_msgs/Float64`, deg):
  `/imu_gui`=heading (`gyro_ang[0]`; depth_nav subscribes — keep name), `/imu_gui_pitch`=`gyro_ang[1]`,
  `/imu_gui_roll`=`gyro_ang[2]` (renamed from `/imu_gui_yaw`). Viewer shows `/imu_gui` as `(yaw)` via
  display-only `AXIS_LABEL` map. Shortcut = `/imu_gui,/imu_gui_pitch,/imu_gui_roll`.

## Launch control (`ros_launcher/launch_server.py`, 8090)
- Collapsible group per package: **xyz_behavior / ainex_peripherals / ainex_app** (`PKGS`), folded.
  Jobs keyed by `pkg/launch`; runs `docker exec -u ubuntu ainex … roslaunch <pkg> <file>` in its own
  session (`start_new_session=True`); Stop = `killpg(SIGINT)`. Each Start also pops an `lxterminal`
  (`DISPLAY=:1`) tailing the job log (per-job logs in `/tmp/ros_launcher/`).
- **⛔ Stop ROS** button (top of Launch tab) → `POST /api/stop_all` runs `~/.stop_ros.sh`
  (docker exec ainex: rosnode kill -a → SIGINT roslaunch → force-kill roslaunch/rosmaster/rosout/rosbridge).
- API: `GET /api/launches`(groups+jobs)`|/api/jobs|/api/log?id=`, `POST /api/launch{pkg,launch}|/api/stop{id}|/api/stop_all`.
  Launch tab polls `/api/launches` every 2s (0.5 Hz); data viewer is push (rosbridge sub at topic rate).
- SECURITY: anyone reaching :8090 can start the robot. Ports avoided: 5900(wayvnc)/6080(noVNC)/8080(web_video_server)/9090(rosbridge).

Related: [[camera-view-banding]]; the FastAPI+rospy alternative is `/home/pi/docker/runtime_debug_page/`.
