# Ainex Robot — Full Architecture & Repository Analysis

_Generated: 2026-03-03 from deep repository scan of `/home/pi/docker/src`_

---

## ROS Version

**ROS Noetic (confirmed)** — catkin build system, rospy/roscpp, .launch XML format.
No ROS2 artifacts anywhere. Docker container: `ainex`, distro: `noetic`.

---

## Repository Root

- **Host path**: `/home/pi/docker/src`
- **Container path**: `/home/ubuntu/share/src` (mounted)
- **ROS workspace**: `/home/ubuntu/ros_ws/`
- **Build**: `catkin build` from `/home/ubuntu/ros_ws`
- **Source**: `source /home/ubuntu/ros_ws/devel/setup.bash`
- **Main launch**: `roslaunch ainex_bringup bringup.launch`

---

## Package Inventory (17 packages)

### Core Packages

| Package | Path | Purpose |
|---------|------|---------|
| `ainex_bringup` | `ainex_bringup/` | Master launch orchestration |
| `ros_robot_controller` | `ainex_driver/ros_robot_controller/` | STM32 serial bridge (hardware) |
| `ainex_sdk` | `ainex_driver/ainex_sdk/` | Onboard Python SDK (LED, audio, ASR, button) |
| `ainex_kinematics` | `ainex_driver/ainex_kinematics/` | IK engine + gait controller (24 DOF) |
| `ainex_interfaces` | `ainex_interfaces/` | Custom msgs (12) + srvs (10) |
| `ainex_peripherals` | `ainex_peripherals/` | USB camera, IMU filter, joystick |
| `ainex_calibration` | `ainex_calibration/` | IMU, mag, camera calibration data |
| `ainex_app` | `ainex_app/` | Web/mobile app, rosbridge |
| `ainex_example` | `ainex_example/` | 30+ task demo scripts |
| `ainex_tutorial` | `ainex_tutorial/` | Tutorial code |

### Simulation Packages

| Package | Path | Purpose |
|---------|------|---------|
| `ainex_description` | `ainex_simulations/ainex_description/` | URDF/xacro + 27 STL meshes |
| `ainex_gazebo` | `ainex_simulations/ainex_gazebo/` | Gazebo worlds, controllers |

### Third-Party Packages

| Package | Purpose |
|---------|---------|
| `apriltag_ros` | AprilTag visual fiducial detection |
| `imu_calib` | IMU bias/scale calibration |
| `calibration_imu` | ROS wrapper for IMU calib |
| `sensor_msgs_ext` | Extended sensor message types |

### Competition Package

| Package | Path | Purpose |
|---------|------|---------|
| `hurocup2025` | `hurocup2025/` | HuroCup 2025: marathon, penalty kick, sprint, triple jump, weight lift |

---

## Hardware Architecture

### Servo System (24 DOF)
- **Connection**: RS485 on `/dev/ttyAMA0`, 115200 baud, via STM32
- **Update rate**: 40 Hz
- **Servo IDs and joint names**:

| ID | Joint | Side | Group |
|----|-------|------|-------|
| 1 | hip_yaw | R | Leg |
| 2 | hip_roll | R | Leg |
| 3 | hip_pitch | R | Leg |
| 4 | knee | R | Leg |
| 5 | ank_pitch | R | Leg |
| 6 | ank_roll | R | Leg |
| 7 | hip_yaw | L | Leg |
| 8 | hip_roll | L | Leg |
| 9 | hip_pitch | L | Leg |
| 10 | knee | L | Leg |
| 11 | ank_pitch | L | Leg |
| 12 | ank_roll | L | Leg |
| 13 | sho_pitch | R | Arm |
| 14 | sho_pitch | L | Arm |
| 15 | sho_roll | R | Arm |
| 16 | sho_roll | L | Arm |
| 17 | el_pitch | R | Arm |
| 18 | el_pitch | L | Arm |
| 19 | el_yaw | R | Arm |
| 20 | el_yaw | L | Arm |
| 21 | gripper | R | Arm |
| 22 | gripper | L | Arm |
| 23 | head_pan | — | Head |
| 24 | head_tilt | — | Head |

### Sensors
- **IMU**: 9-axis (accel + gyro + mag) @ 100 Hz via STM32
- **Camera**: USB camera `/dev/video0`, 640×480 @ 30fps
- **Button**: Onboard button via GPIO
- **Sonar**: Optional ultrasonic sensor
- **Joystick**: `/dev/input/js0` (SBUS remote or USB gamepad)

### Onboard Peripherals
- RGB LED, Buzzer, OLED display, Speaker (TTS), Microphone (ASR)

---

## Key Source Files

### Driver Nodes
| File | Node Name | Function |
|------|-----------|----------|
| `ainex_driver/ros_robot_controller/scripts/ros_robot_controller_node.py` | `ros_robot_controller_node` | STM32 serial bridge — publishes IMU, battery, button; subscribes servo/LED/buzzer cmds |
| `ainex_driver/ainex_sdk/scripts/sensor_node.py` | `sensor_node` | Button polling + LED control |
| `ainex_driver/ainex_kinematics/scripts/ainex_controller.py` | `ainex_controller` | Motion controller — IK, gait, 24 servo coordination |
| `ainex_peripherals/scripts/joystick_control.py` | `joystick_control` | Gamepad → walking commands |
| `ainex_peripherals/scripts/tf_broadcaster_imu.py` | `tf_broadcaster_imu` | Publishes static TF frames (imu_link, camera_link) |

### SDK Modules (`ainex_driver/ainex_sdk/src/ainex_sdk/`)
`button.py`, `led.py`, `gpio.py`, `asr.py`, `tts.py`, `voice_play.py`, `pid.py`, `sonar.py`, `misc.py`, `common.py`, `fps.py`, `board_sensor_check.py`

### Kinematics Libraries (`ainex_driver/ainex_kinematics/src/ainex_kinematics/`)
- `kinematics.so` — compiled IK library
- `walking_module.so` — compiled walking gait generator
- `gait_manager.py` — gait parameter management
- `motion_manager.py` — motion sequencing

### Perception Utilities (`ainex_example/src/ainex_example/`)
- `color_common.py` — OpenCV HSV color detection
- `face_common.py` — MediaPipe face detection
- `apriltag_common.py` — AprilTag detection utilities
- `pid_track.py` — PID-based visual tracking
- `visual_patrol.py` — Line following logic
- `approach_object.py` — Object approach kinematics

### Example Task Scripts (`ainex_example/scripts/`)
kick_ball, face_detect, hand_gesture_control, apriltag_track, fall_rise, color_track, climb_stairs, hurdles, crawl_under, autonomous_transport, color_sort, hand_trajectory_control, cross_detect, visual_patrol, multi_control

---

## Config File Locations

| File | Path | Contents |
|------|------|----------|
| `servo_controller.yaml` | `ainex_driver/ainex_kinematics/config/` | 24 servo defs: IDs, joint names, limits, speed |
| `init_pose.yaml` | `ainex_driver/ainex_kinematics/config/` | Standing posture (24 joint angles in radians) |
| `walking_param.yaml` | `ainex_driver/ainex_kinematics/config/` | Real robot gait: step height, stride, period_time=400ms |
| `walking_param_sim.yaml` | `ainex_driver/ainex_kinematics/config/` | Gazebo-optimized gait parameters |
| `walking_offset.yaml` | `ainex_driver/ainex_kinematics/config/` | Gait offset adjustments |
| `mag_calib.yaml` | `ainex_calibration/config/` | 4×4 magnetometer calibration matrix |
| `imu_calib.yaml` | `ainex_calibration/config/` | IMU bias calibration |
| `face_track_pid.yaml` | `ainex_example/config/` | PID gains for face tracking |
| `color_track_pid.yaml` | `ainex_example/config/` | PID gains for color tracking |
| `apriltag_track_pid.yaml` | `ainex_example/config/` | PID gains for AprilTag tracking |
| `tags.yaml` | `ainex_example/config/` | AprilTag ID definitions |
| `calib.yaml` | `ainex_example/config/` | Camera calibration offsets |

---

## Launch File Hierarchy

```
bringup.launch
├── usb_cam_with_calib.launch      (ainex_peripherals)
├── base.launch
│   ├── sensor_node.launch         (ainex_sdk)
│   ├── ainex_controller.launch    (ainex_kinematics)
│   │   └── ros_robot_controller_node.launch
│   └── imu.launch                 (ainex_peripherals)
├── joystick_control.launch        (ainex_peripherals)
├── web_video_server               (port 8080)
├── rosbridge.launch               (ainex_app, port 9090)
└── start.launch                   (ainex_app game nodes)
```

---

## ROS Topics (Key)

| Topic | Type | Publisher | Rate |
|-------|------|-----------|------|
| `/ainex/imu/raw` | `sensor_msgs/Imu` | `ros_robot_controller_node` | 100 Hz |
| `/ainex/imu/data` | `sensor_msgs/Imu` | `imu_filter_node` (Madgwick) | 100 Hz |
| `/ainex/battery` | `std_msgs/Float32` | `ros_robot_controller_node` | ~1 Hz |
| `/ainex/usb_cam/image_raw` | `sensor_msgs/Image` | `usb_cam_node` | 30 Hz |
| `/ainex/joint_states` | `sensor_msgs/JointState` | `ainex_controller` | 40 Hz |
| `/ainex/walking_state` | custom | `ainex_controller` | on change |
| `/sensor/button/get_button_state` | custom | `sensor_node` | ~50 Hz |

---

## ROS Services (Key)

| Service | Type | Server | Function |
|---------|------|--------|----------|
| `/ainex/set_walking_command` | `SetWalkingCommand` | `ainex_controller` | Walk: forward/backward/left/right/turn/stop |
| `/ainex/set_walking_param` | `SetWalkingParam` | `ainex_controller` | Update gait parameters |
| `/ainex/get_walking_param` | `GetWalkingParam` | `ainex_controller` | Read current gait params |
| `/ainex/get_walking_state` | `GetWalkingState` | `ainex_controller` | Query walking state |
| `GetBusServoState` | custom | `ros_robot_controller_node` | Read servo position/state |
| `GetBusServosPosition` | custom | `ros_robot_controller_node` | Read all servo positions |

### SetWalkingCommand Usage
```python
from ainex_interfaces.srv import SetWalkingCommand
rospy.wait_for_service('/ainex/set_walking_command')
srv = rospy.ServiceProxy('/ainex/set_walking_command', SetWalkingCommand)
srv(command='forward', step_num=10)
# commands: 'forward', 'backward', 'left', 'right', 'turn_left', 'turn_right', 'stop'
```

---

## Custom Messages (ainex_interfaces)

### Motion Messages
- `WalkingParam.msg` — 25-field gait control struct
- `AppWalkingParam.msg` — simplified app-facing gait params
- `WalkingOffset.msg` — gait offset adjustments
- `HeadState.msg` — pan/tilt angles

### Vision Messages
- `ColorDetect.msg` — single color object detection
- `ColorsDetect.msg` — multiple colors
- `ObjectInfo.msg` — detected object (x, y, width, height, color)
- `ObjectsInfo.msg` — array of objects
- `PixelPosition.msg` — 2D pixel coordinates
- `FingerPosition.msg` — finger detection result
- `ROI.msg` — region of interest
- `LineROI.msg` — line detection region

---

## TF Frame Tree

```
world
└── odom
    └── base_link  (body_link — torso center)
        ├── imu_link
        ├── camera_link → camera_rgb_optical_frame
        ├── head_pan_link → head_tilt_link
        ├── r_hip_yaw_link → r_hip_roll → r_hip_pitch → r_knee
        │                                               → r_ank_pitch → r_ank_roll → r_foot_link
        ├── l_hip_yaw_link → (mirror of right)
        ├── r_sho_pitch_link → r_sho_roll → r_el_pitch → r_el_yaw → r_gripper_link
        └── l_sho_pitch_link → (mirror of right)
```

Static transforms published by `tf_broadcaster_imu.py`:
- `base_link` → `imu_link`: `[0, 0, 0.15]`
- `base_link` → `camera_link`: `[0.04, 0, 0.25]`

---

## URDF Details

- **File**: `ainex_simulations/ainex_description/urdf/ainex.urdf.xacro`
- **Includes**: `ainex.xacro`, `materials.xacro`, `transmissions.xacro`, `gazebo.xacro`
- **Meshes**: 27 STL files in `ainex_simulations/ainex_description/meshes/`
- **Motor spec**: 6 Nm torque, 100 rad/s max velocity
- **Base link**: `body_link`

---

## Proposed Production Architecture

### Missing Packages (need to create)

| Package | Purpose | Priority |
|---------|---------|---------|
| `ainex_control` | Safety monitor, watchdog, fall detector, diagnostics | HIGH |
| `ainex_perception` | Unified perception pipeline (consolidate from ainex_example) | HIGH |
| `ainex_navigation` | Gait commander, visual patrol, object approach | MEDIUM |
| `ainex_behavior` | SMACH FSM task manager | MEDIUM |
| `ainex_msgs` | Consolidated messages (merge ainex_interfaces + hw msgs) | MEDIUM |

### Reorganization Map

| Current | Proposed | Action |
|---------|----------|--------|
| `ainex_driver/` (meta) | `ainex_drivers/` | Rename |
| `ainex_interfaces/` | `ainex_msgs/` | Rename + consolidate |
| `ainex_simulations/ainex_description/` | `ainex_description/` | Promote to top-level |
| `ainex_simulations/ainex_gazebo/` | `ainex_simulation/ainex_gazebo/` | Rename container |
| `ainex_example/src/.../color_common.py` etc | `ainex_perception/src/` | Move |
| `ainex_example/src/.../visual_patrol.py` etc | `ainex_navigation/src/` | Move |
| `ainex_example/scripts/fall_rise/` | `ainex_behavior/scripts/` | Move |
| `hurocup2025/` | `ainex_competition/` | Rename |
| `imu_calib/`, `apriltag_ros/`, etc | `third_party/` | Move |

---

## MVP Launch Sequence (Minimal Robot)

```bash
# In Docker container
source /home/ubuntu/ros_ws/devel/setup.bash

# Full bringup
roslaunch ainex_bringup bringup.launch

# Smoke tests
rostopic hz /ainex/imu/raw           # expect ~100 Hz
rostopic echo /ainex/battery --once  # expect > 10.5V
rosservice call /ainex/get_walking_state "{}"

# Walk forward 5 steps
rosservice call /ainex/set_walking_command "{command: 'forward', step_num: 5}"

# Stop
rosservice call /ainex/set_walking_command "{command: 'stop', step_num: 0}"
```

---

## Walking Parameters (walking_param.yaml key values)

- `trajectory_step_s`: 0.008
- `servo_control_cycle`: 0.02
- `period_time`: 400 ms
- `step_height`: tunable
- `stride`: tunable
- `arm_swing_gain`: tunable
- Simulation version: `walking_param_sim.yaml`

---

## Competition Tasks (hurocup2025)

| Task | Script | Launch |
|------|--------|--------|
| Marathon | `marathon/marathon_node.py` | `marathon_node.launch` |
| Penalty Kick | `penalty_kick/penalty_kick_node.py` | `penalty_kick_node.launch` |
| Sprint | `sprint/sprint_node.py` | `sprint_node.launch` |
| Triple Jump | `triplejump/triplejump_node.py` | — |
| Weight Lift | `weightlift/weightlift_node.py` | — |

---

## Key Dependencies

```bash
# ROS packages (apt)
ros-noetic-rosbridge-server
ros-noetic-web-video-server
ros-noetic-imu-filter-madgwick
ros-noetic-usb-cam
ros-noetic-joy
ros-noetic-smach
ros-noetic-smach-ros
ros-noetic-diagnostic-updater

# Python (pip3)
mediapipe
opencv-python
numpy
scipy
PyYAML
```

---

## Namespacing Convention

```
/ainex/                  # all hardware topics
/ainex/imu/              # IMU sub-namespace
/ainex/usb_cam/          # camera sub-namespace
/ainex/color/            # color perception
/ainex/face/             # face perception
/ainex/apriltag/         # fiducial detection
/ainex/task/             # behavior commands/status
/ainex/robot/            # robot health/status
/sensor/                 # SDK sensors (button, LED)
```

---

## Safety & Control Notes

- No watchdog node exists in current repo — if controller crashes, servos hold last position
- Fall detection logic exists only in `ainex_example/scripts/fall_rise/` — not in a proper control layer
- No `ros_control` hardware interface — direct Python-to-serial control
- Emergency stop: no unified ESTOP topic in current repo
- Battery low warning: voltage monitoring in `ros_robot_controller_node.py`

---

## Simulation Notes

- Gazebo launch: `roslaunch ainex_simulations ainex_description/launch/gazebo.launch`
  or via bringup: `roslaunch ainex_bringup bringup.launch sim:=true gazebo:=true`
- Sim uses `walking_param_sim.yaml` (different gait params from real robot)
- Controller flag: `<arg name="gazebo_sim" value="true"/>` in `ainex_controller.launch`
- Position controllers defined in `ainex_gazebo/config/position_controller.yaml`

---

## Dynamic Reconfigure

- `ainex_example` uses `dynamic_reconfigure` for runtime PID tuning
- Config: `ainex_example/cfg/PID.cfg`
- Access: `rosrun rqt_reconfigure rqt_reconfigure`

---

## Audio Files

- Location: `ainex_driver/ainex_sdk/audio/`
- Files: `running.wav`, `warnning.wav`, `english/running.wav`, `english/warnning.wav`
- Playback via: `ainex_sdk.voice_play.VoicePlay`
