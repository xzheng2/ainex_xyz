# Ainex ROS2 Migration

## Status
- **Stage 1 complete** (Apr 1 2026): `ainex2` service added to docker-compose; base image built
- **Stage 2 complete** (Apr 2 2026): all 10 packages ported and built ✓
- **Bringup working** (Apr 2 2026): all 4 base nodes + IMU filter + camera stack verified

## Container: `ainex2`
- Image: `ainex2:humble` (built from `Dockerfile.ros2`)
- Base: `ros:humble-ros-base` (Ubuntu 22.04, aarch64)
- User: `ubuntu`, home `/home/ubuntu`
- Network: `host` (DDS, no ROS_MASTER_URI needed), `ROS_DOMAIN_ID=0`
- `privileged: true`, `group_add: ["1001"]` (host gpiochip device group for GPIO access)

## Key Files
- `Dockerfile`: `/home/pi/docker/Dockerfile.ros2`
- Entrypoint: `/home/pi/docker/ros2_entrypoint.sh` (sources humble + workspace setup.bash)
- Workspace src (host): `/home/pi/docker/ros2_ws_src/` → container `/home/ubuntu/ros2_ws/src`
- docker-compose: `/home/pi/docker/docker-compose.yml`

## Shell & Sourcing
- **Always use `setup.zsh`, NEVER `setup.bash` in zsh** — bash scripts use `BASH_SOURCE[0]` which is empty in zsh, causing wrong COLCON_CURRENT_PREFIX (falls back to CWD)
- Source: `source /home/ubuntu/ros2_ws/install/setup.zsh`
- colcon build must be run from workspace root: `cd /home/ubuntu/ros2_ws && colcon build ...`

## Build & Run
```bash
docker compose -f /home/pi/docker/docker-compose.yml build ainex2   # ~15–25 min
docker compose -f /home/pi/docker/docker-compose.yml run --rm ainex2
```

## Bringup
```bash
# Base (4 nodes)
ros2 launch ainex_bringup bringup.launch.py

# With IMU filter
ros2 launch ainex_bringup bringup.launch.py use_imu:=true

# With camera + rectify + web video
ros2 launch ainex_bringup bringup.launch.py use_camera:=true

# All
ros2 launch ainex_bringup bringup.launch.py use_imu:=true use_camera:=true
```

### Node inventory
| Node | Package | Arg |
|---|---|---|
| `/ainex_controller` | ainex_kinematics | always |
| `/ros_robot_controller` | ros_robot_controller | always |
| `/rosbridge_websocket` | rosbridge_server (port 9090) | always |
| `/sensor` | ainex_sdk | always |
| `/imu_filter` | imu_complementary_filter | `use_imu:=true` |
| `/camera/usb_cam` | usb_cam | `use_camera:=true` |
| `/camera/rectify_color` | image_proc | `use_camera:=true` |
| `/web_video_server` | web_video_server (port 8080) | `use_camera:=true` |

### IMU notes
- `/ros_robot_controller/imu_raw` — raw accel + gyro only; **orientation fields are zeros** (hardcoded 0,0,0,0 in node)
- `/imu` — fused orientation from complementary filter (only with `use_imu:=true`)
- `imu_calib` not ported (not in apt); imu_raw fed directly to imu_filter (skip calibration step)
- Plot live IMU: use `linear_acceleration/x,y,z` or `angular_velocity/x,y,z` from imu_raw; use `orientation` from `/imu`

### Known shutdown noise (cosmetic)
On Ctrl+C, 3 nodes print `RCLError: rcl_shutdown already called` — double-shutdown race between launch SIGINT handler and node cleanup. Nodes ran correctly; fix is `try/except` around `rclpy.shutdown()` calls in sensor_node.py, ainex_controller.py, ros_robot_controller_node.py.

## Installed Packages (Dockerfile)
### ROS Humble (apt)
- `usb-cam`, `image-proc`, `web-video-server`
- `image-transport`, `image-transport-plugins`, `compressed-image-transport`
- `cv-bridge`, `tf2-tools`, `tf2-ros`
- `joint-state-publisher`, `robot-state-publisher`, `xacro`
- `imu-tools` (includes `imu-complementary-filter`, `imu-filter-madgwick`)
- `py-trees-ros` (**compatible with py_trees 2.2.x**)
- `apriltag-ros`, `diagnostic-updater`, `diagnostic-aggregator`
- `rosbridge-server`, `rqt`, `rqt-common-plugins`, `rqt-graph`, `rqt-image-view`

### Python (pip)
- `py_trees==2.2.3`, `Cython==0.29.37`, `PyYAML`, `pyserial`, `gpiod` (2.4.1, 2.x API)

### GPIO note
- `button.py` and `led.py` use **gpiod 2.x API** (`gpiod.Chip()`, `chip.request_lines()`, `gpiod.LineSettings()`)
- Old gpiod 1.x API (`gpiod.chip()`, `gpiod.line_request()`) is incompatible with 2.x
- `/dev/gpiochip0` owned by GID 1001 (unnamed on Pi 5); `group_add: ["1001"]` in docker-compose gives access

## Key Differences from ROS1
| Topic | ROS1 (Noetic) | ROS2 (Humble) |
|---|---|---|
| Build | `catkin build` | `colcon build` (from workspace root) |
| Node API | `rospy` | `rclpy` |
| Launch | `.launch` XML | `.launch.py` Python |
| py_trees | 2.1.6 via pip (apt broken) | 2.2.3 via pip + `ros-humble-py-trees-ros` |
| Master | `roscore` required | DDS, no master |
| Params | `rosparam` | `ros2 param` |
| Bags | `.bag` (rosbag) | `.db3` (ros2 bag) |
| GPIO | gpiod 1.x API | gpiod 2.x API (breaking change) |
| Covariance | `int` zeros ok | must be `float` (`0.0` not `0`) |
| Image proc | nodelets | composable nodes / standalone `rectify_node` |
| Shell source | setup.bash | **setup.zsh** (in zsh shell) |

## Bind Mounts
- `/home/pi/docker/ros2_ws_src` → `/home/ubuntu/ros2_ws/src`
- `/home/pi/docker/src` → `/home/ubuntu/share/src`
- `/home/pi/docker/tmp` → `/home/ubuntu/share/tmp`
- `/dev`, `/tmp/.X11-unix` (display)

## Full Package Inventory

| ROS2 Package | Location | Status |
|---|---|---|
| `ros_robot_controller_msgs` | `ros2_ws_src/ainex_driver/ros_robot_controller_msgs/` | Built ✓ |
| `ainex_interfaces` | `ros2_ws_src/ainex_interfaces/` | Built ✓ |
| `ros_robot_controller` | `ros2_ws_src/ainex_driver/ros_robot_controller/` | Built ✓ |
| `ainex_sdk` | `ros2_ws_src/ainex_driver/ainex_sdk/` | Built ✓ |
| `ainex_kinematics` | `ros2_ws_src/ainex_driver/ainex_kinematics/` | Built ✓ (Cython .so) |
| `ainex_bringup` | `ros2_ws_src/ainex_bringup/` | Built ✓ (launch-only) |
| `ainex_bt_edu_msgs` | `ros2_ws_src/ainex_bt_edu_msgs/` | Built ✓ |
| `ainex_bt_edu` | `ros2_ws_src/ainex_bt_edu/` | Built ✓ (36 nodes) |
| `ainex_behavior` | `ros2_ws_src/ainex_behavior/` | Built ✓ (marathon BT) |
| `hurocup2025` | `ros2_ws_src/hurocup2025/` | Built ✓ |

### Folder structure mirrors ros_ws_src/
- `ainex_driver/` nests: `ainex_kinematics`, `ainex_sdk`, `ros_robot_controller`, `ros_robot_controller_msgs`
- `ainex_interfaces`, `ainex_bringup`, `ainex_bt_edu_msgs`, `ainex_bt_edu`, `ainex_behavior`, `hurocup2025` at top level

### Key porting decisions
- `ros_robot_controller_sdk.py` copied as-is (pure Python, no ROS)
- `sensor_msgs_ext/magnetometer` → `sensor_msgs/MagneticField` for mag_raw
- `MotionManager` now takes a `node` argument (rclpy node reference)
- `ainex_controller.py` uses a background thread for the walk loop (tight timing)
- Cython `.so` at: `install/ainex_kinematics/local/lib/python3.10/dist-packages/ainex_kinematics/`
- `BTExecController` services: ROS2 callbacks take `(req, resp)` and return `resp`
- `MarathonBBBridge`: `node.create_timer()` instead of `rospy.Timer`; QoS TRANSIENT_LOCAL instead of latch=True
- `hurocup2025`: `visual_patrol.py` loads calib.yaml from `ament_index_python` share dir

### Build order (colcon, from /home/ubuntu/ros2_ws)
```bash
colcon build --packages-select ros_robot_controller_msgs ainex_interfaces
colcon build --packages-select ros_robot_controller ainex_sdk
colcon build --packages-select ainex_kinematics
colcon build --packages-select ainex_bringup ainex_bt_edu_msgs ainex_bt_edu ainex_behavior hurocup2025
```

## Entry Points
```bash
ros2 launch ainex_bringup bringup.launch.py [use_imu:=true] [use_camera:=true]
ros2 launch ainex_behavior marathon_bt_node.launch.py
ros2 launch hurocup2025 marathon.launch.py
ros2 run ainex_behavior marathon_bt_node
ros2 run hurocup2025 marathon_node.py
ros2 run hurocup2025 fall_rise_node.py
```

## Deferred (not yet ported)
- `ainex_app` + `ainex_example` (color_detection, face_detect) — no ROS2 port
- `ainex_peripherals` joystick_control — not needed for competition
- `imu_calib` — not in apt, not ported; raw IMU used directly
- `ainex_description` / URDF (inside `ainex_simulations/`)
- Competition scripts: `penalty_kick`, `sprint`, `triplejump`, `weightlift`
