# Ainex Robot — Production-Grade ROS Framework
_Generated: 2026-03-03_

---

## PHASE 1 — ROS Version Detection

**Verdict: ROS 1 Noetic (confirmed, unambiguous)**

| File | Evidence |
|------|----------|
| `ainex_bringup/package.xml` | `<buildtool_depend>catkin</buildtool_depend>` |
| `ainex_kinematics/CMakeLists.txt` | `find_package(catkin REQUIRED)` |
| `bringup.launch` | `.launch` XML format (not `.launch.py`) |
| `ros_robot_controller_node.py` | `import rospy` (not rclpy) |
| `ainex_interfaces/CMakeLists.txt` | `add_message_files()`, `generate_messages()` (catkin pattern) |
| Container runtime | `ROS_DISTRO=noetic` in Docker |
| `apriltag_ros/package.xml` | Format 2, `<buildtool_depend>catkin</buildtool_depend>` |

No ROS2 artifacts (`ament`, `rclcpp`, `rclpy`, `.launch.py`, `colcon`) found anywhere.

---

## PHASE 2 — Architecture Design

### Layer Map

```
+---------------------------------------------------------------------+
|  BEHAVIOR LAYER     FSM / Task Manager / Competition Logic          |
|  ainex_behavior     ainex_competition                               |
+---------------------------------------------------------------------+
|  PERCEPTION LAYER   Vision / Detection / Tracking                   |
|  ainex_perception   apriltag_ros   (mediapipe)                      |
+---------------------------------------------------------------------+
|  PLANNING/NAV       Gait Planning / Path Following                  |
|  ainex_navigation   ainex_kinematics                                |
+---------------------------------------------------------------------+
|  CONTROL LAYER      Servo Controller / Safety / Diagnostics         |
|  ainex_control      ainex_kinematics   ros_robot_controller         |
+---------------------------------------------------------------------+
|  INTERFACE LAYER    Joystick / Web App / Web Video                  |
|  ainex_app          ainex_peripherals  rosbridge                    |
+---------------------------------------------------------------------+
|  DRIVER LAYER       STM32 Serial / Camera / IMU / Peripherals       |
|  ros_robot_controller   ainex_sdk   ainex_peripherals               |
+---------------------------------------------------------------------+
|  DESCRIPTION/CALIB  URDF / Meshes / Calibration Data               |
|  ainex_description  ainex_calibration                               |
+---------------------------------------------------------------------+
|  SIMULATION         Gazebo / RViz                                   |
|  ainex_gazebo       ainex_description                               |
+---------------------------------------------------------------------+
```

### Component Inventory

| Layer | Component | Status | Notes |
|-------|-----------|--------|-------|
| Hardware/Drivers | STM32 serial bridge | EXISTS | `ros_robot_controller_node.py` via `/dev/ttyAMA0` |
| Hardware/Drivers | Servo controller (24 DOF) | EXISTS | RS485 bus servos IDs 1-24 |
| Hardware/Drivers | USB camera | EXISTS | `usb_cam_with_calib.launch` |
| Hardware/Drivers | IMU (9-axis) | EXISTS | Onboard via STM32, Madgwick filter |
| Hardware/Drivers | Sonar sensor | EXISTS | `sonar.py` in ainex_sdk |
| Hardware/Drivers | Speaker / ASR | EXISTS | `tts.py`, `asr.py`, `voice_play.py` |
| Hardware/Drivers | LED / Buzzer / OLED | EXISTS | Via STM32 messages |
| Robot Description | URDF/xacro (24 DOF) | EXISTS | `ainex_description` with 27 STL meshes |
| Robot Description | Calibration data | EXISTS | `ainex_calibration` (IMU, mag, camera) |
| Control | Joint position controllers | EXISTS | `servo_controller.yaml`, 40 Hz |
| Control | Gait / IK engine | EXISTS | `kinematics.so`, `walking_module.so` |
| Control | Safety / Watchdog | MISSING | [ASSUMPTION] No watchdog node detected |
| Control | Fall detection | PARTIAL | `fall_rise_node.py` in example, not in control layer |
| Control | Diagnostics | MISSING | No `diagnostic_updater` found |
| Perception | Color detection | EXISTS | `color_common.py`, OpenCV HSV |
| Perception | Face detection | EXISTS | MediaPipe via `face_common.py` |
| Perception | Hand gesture | EXISTS | MediaPipe via `hand_gesture_detect_node.py` |
| Perception | AprilTag fiducial | EXISTS | Full `apriltag_ros` package |
| Perception | Line following | EXISTS | `visual_patrol.py` |
| Perception | Ball detection | EXISTS | `kick_ball_node.py` with color segmentation |
| Perception | LiDAR | NOT PRESENT | No LiDAR packages found |
| Behavior | FSM / Task Manager | PARTIAL | Scattered in `ainex_example`; no unified FSM |
| Behavior | Fall recovery FSM | EXISTS | `fall_rise_node.py` |
| Behavior | Competition tasks | EXISTS | `hurocup2025` package |
| Navigation | Gait-based walking | EXISTS | Via `SetWalkingCommand.srv` |
| Navigation | SLAM | NOT PRESENT | [ASSUMPTION] No SLAM packages |
| Navigation | Path planning | NOT PRESENT | [ASSUMPTION] No move_base/Nav2 |
| Interfaces | Joystick/gamepad | EXISTS | `joystick_control.py` |
| Interfaces | Web video stream | EXISTS | `web_video_server` |
| Interfaces | WebSocket bridge | EXISTS | `rosbridge_websocket` |
| Interfaces | Mobile app | EXISTS | `ainex_app` |
| Simulation | Gazebo model | EXISTS | `ainex_gazebo`, `ainex_description` |
| Simulation | RViz config | EXISTS | Multiple `.rviz` files |
| Logging | rosbag | MISSING | No automated bag recording |
| Logging | Metrics | MISSING | No monitoring framework |

---

## PHASE 3 — Workspace Structure

```
ainex_ws/
├── src/
│   ├── ainex_bringup/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   └── launch/
│   │       ├── bringup.launch
│   │       ├── base.launch
│   │       ├── drivers.launch
│   │       ├── perception.launch
│   │       ├── simulation.launch
│   │       └── navigation.launch
│   │
│   ├── ainex_msgs/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── msg/
│   │   │   ├── WalkingParam.msg
│   │   │   ├── HeadState.msg
│   │   │   ├── ObjectInfo.msg
│   │   │   ├── ObjectsInfo.msg
│   │   │   ├── ColorDetect.msg
│   │   │   ├── ColorsDetect.msg
│   │   │   ├── PixelPosition.msg
│   │   │   ├── FingerPosition.msg
│   │   │   ├── ROI.msg
│   │   │   ├── LineROI.msg
│   │   │   ├── RobotStatus.msg       [NEW]
│   │   │   └── TaskStatus.msg        [NEW]
│   │   └── srv/
│   │       ├── SetWalkingCommand.srv
│   │       ├── SetWalkingParam.srv
│   │       ├── GetWalkingParam.srv
│   │       ├── GetWalkingState.srv
│   │       ├── SetWalkingOffset.srv
│   │       ├── GetWalkingOffset.srv
│   │       ├── SetString.srv
│   │       ├── SetInt.srv
│   │       ├── SetFloat.srv
│   │       └── SetPoint.srv
│   │
│   ├── ainex_description/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── urdf/
│   │   │   ├── ainex.urdf.xacro
│   │   │   ├── ainex.xacro
│   │   │   ├── materials.xacro
│   │   │   ├── transmissions.xacro
│   │   │   └── gazebo.xacro
│   │   ├── meshes/
│   │   │   └── [27 STL files]
│   │   ├── launch/
│   │   │   └── display.launch
│   │   └── rviz/
│   │       └── urdf.rviz
│   │
│   ├── ainex_drivers/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   │
│   │   ├── ros_robot_controller/
│   │   │   ├── package.xml
│   │   │   ├── CMakeLists.txt
│   │   │   ├── launch/
│   │   │   │   └── ros_robot_controller_node.launch
│   │   │   ├── scripts/
│   │   │   │   └── ros_robot_controller_node.py
│   │   │   ├── src/ros_robot_controller/
│   │   │   │   └── ros_robot_controller_sdk.py
│   │   │   ├── msg/   [17 hardware messages]
│   │   │   └── srv/   [3 hardware services]
│   │   │
│   │   ├── ainex_sdk/
│   │   │   ├── package.xml
│   │   │   ├── CMakeLists.txt
│   │   │   ├── launch/
│   │   │   │   └── sensor_node.launch
│   │   │   ├── scripts/
│   │   │   │   └── sensor_node.py
│   │   │   ├── src/ainex_sdk/
│   │   │   │   ├── button.py
│   │   │   │   ├── led.py
│   │   │   │   ├── gpio.py
│   │   │   │   ├── asr.py
│   │   │   │   ├── tts.py
│   │   │   │   ├── voice_play.py
│   │   │   │   ├── pid.py
│   │   │   │   ├── sonar.py
│   │   │   │   └── common.py
│   │   │   └── audio/
│   │   │       └── [WAV files]
│   │   │
│   │   └── ainex_kinematics/
│   │       ├── package.xml
│   │       ├── CMakeLists.txt
│   │       ├── launch/
│   │       │   └── ainex_controller.launch
│   │       ├── scripts/
│   │       │   └── ainex_controller.py
│   │       ├── src/ainex_kinematics/
│   │       │   ├── kinematics.so
│   │       │   ├── walking_module.so
│   │       │   ├── gait_manager.py
│   │       │   └── motion_manager.py
│   │       └── config/
│   │           ├── servo_controller.yaml
│   │           ├── init_pose.yaml
│   │           ├── walking_param.yaml
│   │           ├── walking_param_sim.yaml
│   │           └── walking_offset.yaml
│   │
│   ├── ainex_control/              [NEW]
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   └── control.launch
│   │   ├── scripts/
│   │   │   ├── safety_monitor.py
│   │   │   ├── fall_detector.py
│   │   │   └── diagnostics_node.py
│   │   └── config/
│   │       └── control.yaml
│   │
│   ├── ainex_peripherals/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   ├── usb_cam.launch
│   │   │   ├── usb_cam_with_calib.launch
│   │   │   ├── imu.launch
│   │   │   └── joystick_control.launch
│   │   ├── scripts/
│   │   │   ├── joystick_control.py
│   │   │   └── tf_broadcaster_imu.py
│   │   └── config/
│   │       └── sensors.yaml
│   │
│   ├── ainex_calibration/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── config/
│   │   │   ├── imu_calib.yaml
│   │   │   └── mag_calib.yaml
│   │   └── launch/
│   │       ├── camera_calibration.launch
│   │       ├── imu_calibration.launch
│   │       └── mag_calibration.launch
│   │
│   ├── ainex_perception/           [NEW]
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   ├── perception.launch
│   │   │   ├── color_detect.launch
│   │   │   ├── face_detect.launch
│   │   │   └── apriltag.launch
│   │   ├── scripts/
│   │   │   ├── color_detect_node.py
│   │   │   ├── face_detect_node.py
│   │   │   ├── hand_gesture_node.py
│   │   │   ├── ball_detect_node.py
│   │   │   └── line_detect_node.py
│   │   ├── src/ainex_perception/
│   │   │   ├── color_common.py
│   │   │   ├── face_common.py
│   │   │   ├── apriltag_common.py
│   │   │   └── pid_track.py
│   │   └── config/
│   │       ├── color_detect.yaml
│   │       ├── face_detect.yaml
│   │       └── apriltag.yaml
│   │
│   ├── ainex_navigation/           [NEW]
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   └── navigation.launch
│   │   ├── scripts/
│   │   │   ├── gait_commander.py
│   │   │   ├── visual_patrol_node.py
│   │   │   └── approach_object_node.py
│   │   ├── src/ainex_navigation/
│   │   │   ├── visual_patrol.py
│   │   │   └── approach_object.py
│   │   └── config/
│   │       └── navigation.yaml
│   │
│   ├── ainex_behavior/             [NEW]
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   └── behavior.launch
│   │   ├── scripts/
│   │   │   ├── task_manager_node.py
│   │   │   ├── fall_rise_node.py
│   │   │   └── idle_behavior_node.py
│   │   ├── src/ainex_behavior/
│   │   │   ├── robot_fsm.py
│   │   │   └── states/
│   │   │       ├── idle_state.py
│   │   │       ├── walking_state.py
│   │   │       ├── fallen_state.py
│   │   │       └── task_state.py
│   │   └── config/
│   │       └── behavior.yaml
│   │
│   ├── ainex_app/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   ├── rosbridge.launch
│   │   │   └── start.launch
│   │   └── scripts/
│   │       └── app_node.py
│   │
│   ├── ainex_simulation/
│   │   ├── package.xml
│   │   └── ainex_gazebo/
│   │       ├── package.xml
│   │       ├── CMakeLists.txt
│   │       ├── launch/
│   │       │   ├── empty_world.launch
│   │       │   ├── worlds.launch
│   │       │   ├── spawn_model.launch
│   │       │   └── position_controller.launch
│   │       ├── config/
│   │       │   └── position_controller.yaml
│   │       ├── worlds/
│   │       │   ├── empty.world
│   │       │   └── hurocup_arena.world  [NEW]
│   │       └── rviz/
│   │           └── urdf.rviz
│   │
│   ├── ainex_competition/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   ├── marathon.launch
│   │   │   ├── penalty_kick.launch
│   │   │   ├── sprint.launch
│   │   │   └── triple_jump.launch
│   │   └── scripts/
│   │       ├── marathon/
│   │       ├── penalty_kick/
│   │       ├── sprint/
│   │       └── triplejump/
│   │
│   └── third_party/
│       ├── imu_calib/
│       ├── calibration_imu/
│       ├── sensor_msgs_ext/
│       └── apriltag_ros/
│
├── .github/
│   └── workflows/
│       └── ci.yml
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
├── README.md
└── ARCHITECTURE.md
```

---

## PHASE 4 — Required Deliverables

### ainex_msgs/package.xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>ainex_msgs</name>
  <version>1.0.0</version>
  <description>Custom ROS messages and services for the Ainex humanoid robot</description>
  <maintainer email="robot@ainex.dev">Ainex Team</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
</package>
```

### ainex_msgs/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(ainex_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs geometry_msgs sensor_msgs
)

add_message_files(FILES
  WalkingParam.msg AppWalkingParam.msg WalkingOffset.msg HeadState.msg
  ColorDetect.msg ColorsDetect.msg ObjectInfo.msg ObjectsInfo.msg
  PixelPosition.msg FingerPosition.msg ROI.msg LineROI.msg
  RobotStatus.msg TaskStatus.msg
)

add_service_files(FILES
  SetWalkingCommand.srv SetWalkingParam.srv GetWalkingParam.srv
  GetWalkingState.srv SetWalkingOffset.srv GetWalkingOffset.srv
  SetString.srv SetInt.srv SetFloat.srv SetPoint.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)
```

---

### ainex_bringup/launch/bringup.launch

```xml
<?xml version="1.0"?>
<launch>
  <arg name="sim"       default="false"/>
  <arg name="gazebo"    default="false"/>
  <arg name="joystick"  default="true"/>
  <arg name="web_video" default="true"/>
  <arg name="app"       default="true"/>
  <arg name="robot_ns"  default="ainex"/>

  <!-- Robot Description -->
  <include file="$(find ainex_description)/launch/display.launch">
    <arg name="use_gui" value="false"/>
  </include>

  <!-- Hardware Drivers -->
  <group unless="$(arg sim)">
    <include file="$(find ainex_bringup)/launch/drivers.launch">
      <arg name="robot_ns" value="$(arg robot_ns)"/>
    </include>
  </group>

  <!-- Peripherals -->
  <include file="$(find ainex_peripherals)/launch/usb_cam_with_calib.launch"/>
  <include file="$(find ainex_peripherals)/launch/imu.launch"/>

  <!-- Control -->
  <include file="$(find ainex_control)/launch/control.launch">
    <arg name="robot_ns" value="$(arg robot_ns)"/>
  </include>

  <!-- Behavior -->
  <include file="$(find ainex_behavior)/launch/behavior.launch">
    <arg name="robot_ns" value="$(arg robot_ns)"/>
  </include>

  <!-- Joystick -->
  <group if="$(arg joystick)">
    <include file="$(find ainex_peripherals)/launch/joystick_control.launch"/>
  </group>

  <!-- Web Video Server -->
  <group if="$(arg web_video)">
    <node name="web_video_server" pkg="web_video_server" type="web_video_server">
      <param name="port" value="8080"/>
      <param name="default_stream_type" value="mjpeg"/>
    </node>
  </group>

  <!-- App / ROS Bridge -->
  <group if="$(arg app)">
    <include file="$(find ainex_app)/launch/rosbridge.launch"/>
    <include file="$(find ainex_app)/launch/start.launch"/>
  </group>
</launch>
```

---

### ainex_bringup/launch/drivers.launch

```xml
<?xml version="1.0"?>
<launch>
  <arg name="robot_ns"    default="ainex"/>
  <arg name="serial_port" default="/dev/ttyAMA0"/>
  <arg name="baud_rate"   default="115200"/>
  <arg name="imu_freq"    default="100"/>

  <group ns="$(arg robot_ns)">
    <!-- STM32 Serial Hardware Bridge -->
    <include file="$(find ros_robot_controller)/launch/ros_robot_controller_node.launch">
      <arg name="port"      value="$(arg serial_port)"/>
      <arg name="baudrate"  value="$(arg baud_rate)"/>
      <arg name="freq"      value="$(arg imu_freq)"/>
      <arg name="imu_frame" value="imu_link"/>
    </include>

    <!-- SDK Sensor Node -->
    <include file="$(find ainex_sdk)/launch/sensor_node.launch"/>

    <!-- Kinematics / Motion Controller -->
    <include file="$(find ainex_kinematics)/launch/ainex_controller.launch">
      <arg name="gazebo_sim" value="false"/>
    </include>
  </group>
</launch>
```

---

### ainex_bringup/launch/simulation.launch

```xml
<?xml version="1.0"?>
<launch>
  <arg name="world_name" default="empty"/>
  <arg name="paused"     default="false"/>
  <arg name="gui"        default="true"/>
  <arg name="rviz"       default="true"/>
  <arg name="robot_ns"   default="ainex"/>

  <include file="$(find ainex_gazebo)/launch/empty_world.launch">
    <arg name="world_name" value="$(find ainex_gazebo)/worlds/$(arg world_name).world"/>
    <arg name="paused"     value="$(arg paused)"/>
    <arg name="gui"        value="$(arg gui)"/>
  </include>

  <include file="$(find ainex_description)/launch/display.launch">
    <arg name="use_gui" value="false"/>
  </include>

  <include file="$(find ainex_gazebo)/launch/spawn_model.launch">
    <arg name="robot_ns" value="$(arg robot_ns)"/>
    <arg name="x" value="0.0"/>
    <arg name="y" value="0.0"/>
    <arg name="z" value="0.36"/>
  </include>

  <include file="$(find ainex_gazebo)/launch/position_controller.launch">
    <arg name="robot_ns" value="$(arg robot_ns)"/>
  </include>

  <include file="$(find ainex_kinematics)/launch/ainex_controller.launch">
    <arg name="gazebo_sim" value="true"/>
  </include>

  <include file="$(find ainex_control)/launch/control.launch"/>
  <include file="$(find ainex_behavior)/launch/behavior.launch"/>

  <group if="$(arg rviz)">
    <node name="rviz" pkg="rviz" type="rviz"
          args="-d $(find ainex_description)/rviz/urdf.rviz"/>
  </group>
</launch>
```

---

### ainex_bringup/launch/navigation.launch

```xml
<?xml version="1.0"?>
<launch>
  <arg name="mode"     default="patrol"/>
  <arg name="robot_ns" default="ainex"/>

  <group ns="$(arg robot_ns)">
    <node name="gait_commander" pkg="ainex_navigation"
          type="gait_commander.py" output="screen">
      <rosparam file="$(find ainex_navigation)/config/navigation.yaml"/>
    </node>

    <group if="$(eval arg('mode') == 'patrol')">
      <node name="visual_patrol" pkg="ainex_navigation"
            type="visual_patrol_node.py" output="screen">
        <rosparam file="$(find ainex_navigation)/config/navigation.yaml"/>
      </node>
    </group>

    <group if="$(eval arg('mode') == 'approach')">
      <node name="approach_object" pkg="ainex_navigation"
            type="approach_object_node.py" output="screen">
        <rosparam file="$(find ainex_navigation)/config/navigation.yaml"/>
      </node>
    </group>
  </group>
</launch>
```

---

### ainex_control/config/control.yaml

```yaml
control:
  watchdog_timeout_s: 2.0

  joint_limits:
    position_tolerance_rad: 0.1
    velocity_limit_rad_s: 5.24

  fall_detection:
    enabled: true
    imu_topic: "/ainex/imu/data"
    roll_threshold_deg: 45.0
    pitch_threshold_deg: 50.0
    debounce_time_s: 0.2

  recovery:
    auto_recover: true
    recovery_timeout_s: 30.0
    max_attempts: 3

  diagnostics:
    publish_rate_hz: 1.0
    battery_warn_voltage: 10.5
    battery_crit_voltage: 10.0

  estop:
    topic: "/ainex/estop"
    requires_reset: true
```

---

### ainex_peripherals/config/sensors.yaml

```yaml
camera:
  device: "/dev/video0"
  frame_rate: 30
  image_width: 640
  image_height: 480
  pixel_format: "yuyv"
  camera_name: "ainex_camera"
  camera_frame: "camera_rgb_optical_frame"
  calibration_file: "$(find ainex_calibration)/config/camera_calib.yaml"
  auto_exposure: true
  exposure: 100
  brightness: 50

imu:
  topic_raw: "/ainex/imu/raw"
  topic_filtered: "/ainex/imu/data"
  frame_id: "imu_link"
  filter_type: "madgwick"
  madgwick_gain: 0.1
  publish_rate_hz: 100
  world_frame: "enu"
  calib_file: "$(find ainex_calibration)/config/imu_calib.yaml"

magnetometer:
  enabled: true
  frame_id: "imu_link"
  calib_file: "$(find ainex_calibration)/config/mag_calib.yaml"

sonar:
  enabled: false
  topic: "/ainex/sonar/range"
  frame_id: "sonar_link"
  min_range_m: 0.02
  max_range_m: 4.0
  field_of_view_deg: 15.0

joystick:
  device: "/dev/input/js0"
  deadzone: 0.1
  axis_walk_x: 1
  axis_walk_y: 0
  axis_turn: 3
  axis_head_pan: 2
  btn_stand: 0
  btn_sit: 1
  btn_wave: 3
  btn_estop: 9
```

---

### ainex_navigation/config/navigation.yaml

```yaml
gait_commander:
  service_topic: "/ainex/set_walking_command"
  default_step_length_mm: 20.0
  default_lateral_step_mm: 10.0
  default_turn_angle_deg: 10.0
  cmd_timeout_s: 0.5

visual_patrol:
  image_topic: "/ainex/usb_cam/image_raw"
  roi_top: 0.6
  roi_bottom: 0.9
  line_color_h_min: 0
  line_color_h_max: 180
  line_color_s_min: 0
  line_color_s_max: 60
  line_color_v_min: 0
  line_color_v_max: 60
  pid:
    kp: 0.3
    ki: 0.0
    kd: 0.05
  walk:
    step_length_mm: 15.0
    max_turn_angle_deg: 8.0

approach_object:
  image_topic: "/ainex/usb_cam/image_raw"
  target_pixel_width: 120
  approach_step_mm: 10.0
  lateral_step_mm: 5.0
  lateral_pid:
    kp: 0.2
    ki: 0.0
    kd: 0.02
  distance_pid:
    kp: 0.15
    ki: 0.0
    kd: 0.01
```

---

### ainex_behavior/config/behavior.yaml

```yaml
task_manager:
  initial_state: "IDLE"
  timeouts:
    idle_to_active_s: 0.0
    active_to_idle_s: 5.0
    fallen_recovery_s: 30.0
  task_command_topic: "/ainex/task/command"
  task_status_topic:  "/ainex/task/status"
  robot_status_topic: "/ainex/robot/status"
  startup:
    play_sound: true
    sound_file: "running.wav"
    blink_led: true

states:
  idle:
    head_default_pan_deg: 0.0
    head_default_tilt_deg: -10.0
    led_color: [0, 0, 255]
  walking:
    led_color: [0, 255, 0]
    max_duration_s: 60.0
  fallen:
    led_color: [255, 0, 0]
    buzzer_hz: 880
    auto_recover: true
  task_executing:
    led_color: [255, 165, 0]

fall_rise:
  fall_event_topic: "/ainex/fall_event"
  rise_from_back_motion: "rise_back"
  rise_from_front_motion: "rise_front"
  confirmation_delay_s: 1.0
```

---

### ainex_control/scripts/safety_monitor.py (stub)

```python
#!/usr/bin/env python3
"""Safety monitor node — watchdog, joint limits, ESTOP, diagnostics."""
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from ainex_msgs.msg import RobotStatus


class SafetyMonitor:
    def __init__(self):
        rospy.init_node('safety_monitor')
        self.watchdog_timeout = rospy.get_param('~watchdog_timeout_s', 2.0)
        self.last_imu_time = rospy.Time.now()
        self.estop_active = False

        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)
        self.estop_pub = rospy.Publisher('/ainex/estop', Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher('/ainex/robot/status', RobotStatus, queue_size=10)

        rospy.Subscriber('/ainex/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/ainex/estop', Bool, self.estop_callback)
        rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)
        rospy.loginfo('[SafetyMonitor] Initialized')

    def imu_callback(self, msg):
        self.last_imu_time = rospy.Time.now()

    def estop_callback(self, msg):
        if msg.data and not self.estop_active:
            rospy.logwarn('[SafetyMonitor] ESTOP activated!')
            self.estop_active = True

    def publish_diagnostics(self, event):
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = 'Ainex Safety Monitor'
        status.hardware_id = 'ainex_robot'
        imu_age = (rospy.Time.now() - self.last_imu_time).to_sec()
        if imu_age > self.watchdog_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = f'IMU timeout: {imu_age:.1f}s'
        elif self.estop_active:
            status.level = DiagnosticStatus.WARN
            status.message = 'ESTOP active'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        status.values = [
            KeyValue('imu_age_s', f'{imu_age:.2f}'),
            KeyValue('estop', str(self.estop_active)),
        ]
        array.status.append(status)
        self.diag_pub.publish(array)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    SafetyMonitor().run()
```

---

### ainex_behavior/src/ainex_behavior/robot_fsm.py (stub)

```python
#!/usr/bin/env python3
"""Ainex Robot FSM using SMACH. States: IDLE -> ACTIVE -> FALLEN -> RECOVERY -> IDLE"""
import rospy
import smach
import smach_ros
from ainex_msgs.msg import TaskStatus, RobotStatus
from ainex_msgs.srv import SetWalkingCommand


class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['activate', 'preempted'],
                             input_keys=['command_in'])

    def execute(self, userdata):
        rospy.loginfo('[FSM] IDLE state')
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if userdata.command_in and userdata.command_in != 'idle':
                return 'activate'
            rate.sleep()
        return 'preempted'


class ActiveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'fallen', 'preempted'],
                             input_keys=['command_in'])

    def execute(self, userdata):
        rospy.loginfo(f'[FSM] ACTIVE: executing {userdata.command_in}')
        rospy.sleep(1.0)
        return 'done'


class FallenState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['recovered', 'failed'])

    def execute(self, userdata):
        rospy.logwarn('[FSM] FALLEN — attempting recovery')
        rospy.sleep(2.0)
        return 'recovered'


def build_fsm():
    sm = smach.StateMachine(outcomes=['shutdown'])
    sm.userdata.command_in = 'idle'
    with sm:
        smach.StateMachine.add('IDLE', IdleState(),
            transitions={'activate': 'ACTIVE', 'preempted': 'shutdown'})
        smach.StateMachine.add('ACTIVE', ActiveState(),
            transitions={'done': 'IDLE', 'fallen': 'FALLEN', 'preempted': 'shutdown'})
        smach.StateMachine.add('FALLEN', FallenState(),
            transitions={'recovered': 'IDLE', 'failed': 'IDLE'})
    return sm
```

---

### README.md

```markdown
# Ainex Humanoid Robot — ROS Noetic Workspace

24-DOF humanoid robot running ROS Noetic in Docker on Raspberry Pi 5.

## Prerequisites
- Docker Engine >= 24.0, docker-compose >= 2.0
- Python 3.8+: mediapipe, opencv-python, numpy, PyYAML

## Installation
    git clone https://github.com/your-org/ainex_ws.git
    cd ainex_ws/docker && docker-compose up -d
    docker exec -it ainex bash
    cd /home/ubuntu/ros_ws && catkin build
    source devel/setup.bash

## Launch Sequence

### Real Robot
    roslaunch ainex_bringup bringup.launch

### Simulation
    roslaunch ainex_bringup simulation.launch gui:=true

### Specific Mode
    roslaunch ainex_bringup bringup.launch app:=false joystick:=false
    roslaunch ainex_bringup navigation.launch mode:=patrol

### Competition
    roslaunch ainex_competition marathon.launch
    roslaunch ainex_competition penalty_kick.launch

## Package Index
| Package | Description |
|---------|-------------|
| ainex_bringup | Master launch orchestration |
| ainex_msgs | Custom messages and services |
| ainex_description | URDF, meshes, robot model |
| ainex_drivers/ros_robot_controller | STM32 serial hardware bridge |
| ainex_drivers/ainex_sdk | Onboard Python SDK |
| ainex_drivers/ainex_kinematics | IK engine and gait controller |
| ainex_control [NEW] | Safety monitor, watchdog, diagnostics |
| ainex_peripherals | Camera, IMU filter, joystick |
| ainex_calibration | Calibration data |
| ainex_perception [NEW] | Unified vision pipeline |
| ainex_navigation [NEW] | Gait-based navigation |
| ainex_behavior [NEW] | FSM task manager |
| ainex_app | Web/mobile app interface |
| ainex_simulation | Gazebo worlds and controllers |
| ainex_competition | HuroCup 2025 tasks |

## Troubleshooting
| Symptom | Fix |
|---------|-----|
| Servos not responding | Check ls /dev/ttyAMA*, verify udev rules |
| Robot falls immediately | Tune walking_param.yaml, test in sim first |
| IMU NaN values | Re-run imu_calibration.launch |
| Camera not opening | Check ls /dev/video*, set device in sensors.yaml |
| Web video not streaming | Check netstat -tulnp | grep 8080 |
| Rosbridge not connecting | Check Docker port 9090 forwarding |
| catkin build fails | sudo apt install ros-noetic-<package> |
```

---

## PHASE 5 — Repository Mapping

### Current → Proposed Path

| Current Path | Proposed Path | Action |
|---|---|---|
| `ainex_bringup/` | `ainex_bringup/` | Refactor launch args |
| `ainex_driver/ros_robot_controller/` | `ainex_drivers/ros_robot_controller/` | Move |
| `ainex_driver/ainex_sdk/` | `ainex_drivers/ainex_sdk/` | Move |
| `ainex_driver/ainex_kinematics/` | `ainex_drivers/ainex_kinematics/` | Move |
| `ainex_interfaces/` | `ainex_msgs/` | Rename + consolidate |
| `ainex_simulations/ainex_description/` | `ainex_description/` | Promote to top-level |
| `ainex_simulations/ainex_gazebo/` | `ainex_simulation/ainex_gazebo/` | Rename |
| `ainex_peripherals/` | `ainex_peripherals/` | Add sensors.yaml |
| `ainex_calibration/` | `ainex_calibration/` | Keep |
| `ainex_app/` | `ainex_app/` | Keep |
| `ainex_example/src/.../color_common.py` | `ainex_perception/src/ainex_perception/` | Move |
| `ainex_example/src/.../face_common.py` | `ainex_perception/src/ainex_perception/` | Move |
| `ainex_example/src/.../apriltag_common.py` | `ainex_perception/src/ainex_perception/` | Move |
| `ainex_example/src/.../pid_track.py` | `ainex_perception/src/ainex_perception/` | Move |
| `ainex_example/src/.../visual_patrol.py` | `ainex_navigation/src/ainex_navigation/` | Move |
| `ainex_example/src/.../approach_object.py` | `ainex_navigation/src/ainex_navigation/` | Move |
| `ainex_example/scripts/fall_rise/` | `ainex_behavior/scripts/fall_rise_node.py` | Move |
| `ainex_example/scripts/` (all task nodes) | `ainex_behavior/scripts/` | Move |
| `hurocup2025/` | `ainex_competition/` | Rename |
| `imu_calib/` | `third_party/imu_calib/` | Move |
| `calibration_imu/` | `third_party/calibration_imu/` | Move |
| `sensor_msgs_ext/` | `third_party/sensor_msgs_ext/` | Move |
| `apriltag_ros/` | `third_party/apriltag_ros/` | Move |

### Missing (create new)

- `ainex_control/` — safety_monitor.py, fall_detector.py, diagnostics_node.py
- `ainex_perception/` — unified vision package
- `ainex_navigation/` — gait commander + visual patrol wrapper
- `ainex_behavior/` — SMACH FSM task manager
- `ainex_msgs/msg/RobotStatus.msg` — overall robot health message
- `ainex_msgs/msg/TaskStatus.msg` — behavior task status message

---

## PHASE 6 — MVP Path

### Minimum Package Set (walk + joystick)

1. `ainex_msgs`
2. `ros_robot_controller`
3. `ainex_sdk`
4. `ainex_kinematics`
5. `ainex_calibration`
6. `ainex_peripherals`
7. `ainex_bringup`

### Exact Launch + Test Sequence

```bash
# Enter container
docker exec -it ainex bash
source /home/ubuntu/ros_ws/devel/setup.bash

# Launch (minimal)
roslaunch ainex_bringup bringup.launch app:=false web_video:=false joystick:=true

# --- In new terminals ---

# Check IMU
rostopic hz /ainex/imu/raw                    # expect ~100 Hz

# Check battery
rostopic echo /ainex/battery --once           # expect > 10.5V

# Check walking service
rosservice call /ainex/get_walking_state "{}" # expect state: "stopped"

# Check camera
rostopic hz /ainex/usb_cam/image_raw          # expect ~30 Hz

# Check TF
rosrun tf tf_echo base_link imu_link          # expect [0, 0, 0.15]

# Walk test
rosservice call /ainex/set_walking_command "{command: 'forward', step_num: 5}"
rosservice call /ainex/set_walking_command "{command: 'stop', step_num: 0}"
```

### Smoke-Test Checklist

| Test | Expected |
|------|----------|
| `rostopic list \| grep /ainex/imu` | Topic listed |
| `rostopic hz /ainex/imu/raw` | ~100 Hz |
| `rostopic echo /ainex/battery --once` | voltage > 10.5 V |
| `rosservice call /ainex/get_walking_state "{}"` | `state: "stopped"` |
| `rostopic hz /ainex/usb_cam/image_raw` | ~30 Hz |
| `rostopic hz /ainex/imu/data` | ~100 Hz |
| `rosrun tf tf_echo base_link imu_link` | `[0, 0, 0.15]` |
| Walk forward 3 steps | Robot physically moves |
| Walk stop | Robot halts |
| Joystick left stick | `rostopic echo /joy` shows axis changes |

---

## PHASE 7 — Engineering Best Practices

### YAML Conventions
- Always suffix units: `_s`, `_ms`, `_hz`, `_m`, `_mm`, `_deg`, `_rad`, `_v`
- Namespace by package: `ainex_control.safety.watchdog_timeout_s`
- Group related params under sub-keys (not flat lists)
- Document every parameter with a comment

### Namespacing Strategy
```
/ainex/              hardware topics
/ainex/imu/          IMU namespace
/ainex/usb_cam/      camera namespace
/ainex/color/        color perception
/ainex/face/         face perception
/ainex/apriltag/     fiducial detection
/ainex/task/         behavior commands/status
/ainex/robot/        robot health/status
/sensor/             SDK sensors (button, LED)
```

### TF Naming Rules
- snake_case for all frame names (base_link, r_hip_yaw_link)
- Suffixes: `_link` = rigid body, `_frame` = abstract, `_optical` = camera optical frame
- Side prefixes: `r_` (right), `l_` (left)
- Never use `/` in frame names
- Static transforms in URDF; dynamic in nodes

### Coding Standards
```python
#!/usr/bin/env python3
"""One-line module docstring."""
import rospy

class MyNode:
    def __init__(self):
        rospy.init_node('my_node')
        self.rate_hz = rospy.get_param('~rate_hz', 10.0)
        self.pub = rospy.Publisher('topic', MsgType, queue_size=10)
        rospy.Subscriber('input', MsgType, self.callback)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.timer_cb)

    def callback(self, msg): pass
    def timer_cb(self, event): pass
    def run(self): rospy.spin()

if __name__ == '__main__':
    MyNode().run()

# Rules:
# - 4-space indent, max 100 chars/line
# - All nodes as classes (not procedural)
# - Log with rospy.loginfo/warn/err (never print)
# - All scripts: chmod +x, #!/usr/bin/env python3
```

### GitHub Actions CI

```yaml
name: Ainex Robot CI
on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main]

jobs:
  build-and-test:
    runs-on: ubuntu-20.04
    container:
      image: ros:noetic-ros-base
    steps:
      - uses: actions/checkout@v3
        with:
          path: src/ainex_ws

      - name: Install dependencies
        run: |
          apt-get update
          rosdep update
          rosdep install --from-paths src --ignore-src -r -y
          apt-get install -y python3-catkin-tools

      - name: Install Python deps
        run: pip3 install numpy opencv-python PyYAML

      - name: Build workspace
        run: |
          source /opt/ros/noetic/setup.bash
          catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release

      - name: Run unit tests
        run: |
          source /opt/ros/noetic/setup.bash
          source devel/setup.bash
          catkin test ainex_msgs ainex_control ainex_perception

      - name: Lint Python
        run: |
          pip3 install flake8
          flake8 src/ainex_ws/src/ --max-line-length=100 --exclude=third_party,*.so
```

---

## TF Frame Tree

```
world
└── odom
    └── base_link  (body_link — torso center)
        ├── imu_link
        ├── camera_link
        │   └── camera_rgb_optical_frame
        ├── head_pan_link  (servo 23)
        │   └── head_tilt_link  (servo 24)
        ├── r_hip_yaw_link  (servo 1)
        │   └── r_hip_roll_link  (servo 2)
        │       └── r_hip_pitch_link  (servo 3)
        │           └── r_knee_link  (servo 4)
        │               └── r_ank_pitch_link  (servo 5)
        │                   └── r_ank_roll_link  (servo 6)
        │                       └── r_foot_link
        ├── l_hip_yaw_link  (servo 7) ... [mirror of right]
        ├── r_sho_pitch_link  (servo 13)
        │   └── r_sho_roll_link  (servo 15)
        │       └── r_el_pitch_link  (servo 17)
        │           └── r_el_yaw_link  (servo 19)
        │               └── r_gripper_link  (servo 21)
        └── l_sho_pitch_link  (servo 14) ... [mirror of right]
```

Static transforms (published by tf_broadcaster_imu.py):

| Parent | Child | Translation |
|--------|-------|-------------|
| base_link | imu_link | [0, 0, 0.15] |
| base_link | camera_link | [0.04, 0, 0.25] |

---

## System Architecture Diagram

```
[Joystick]  [Web App]  [ROS Bridge]
     |            |          |
     +-----> [Behavior FSM (SMACH)] <------+
                  |                        |
     +------------+------------+           |
     |            |            |           |
[Gait Cmd]  [Task Nodes]  [Fall Rise]      |
     |                                     |
     v                                     |
[ainex_controller] <-- SetWalkingCommand   |
     |                                     |
     v                               [Perception]
[ros_robot_controller]              color/face/april
     |                                     ^
     v                                     |
[STM32 MCU]                         [USB Camera]
  |      |
[Servos][IMU]
```

---

## Node Summary Table

| Node | Package | Publishes | Subscribes | Services |
|------|---------|-----------|------------|----------|
| ros_robot_controller_node | ros_robot_controller | /ainex/imu/raw, /ainex/battery, /ainex/sbus, /ainex/button_state | /ainex/led/set, /ainex/buzzer/set, /ainex/servo/set_position | GetBusServoState, GetPWMServoState, GetBusServosPosition |
| sensor_node | ainex_sdk | /sensor/button/get_button_state | /sensor/led/set_led_state | — |
| ainex_controller | ainex_kinematics | /ainex/joint_states, /ainex/walking_state | /ainex/head/cmd | SetWalkingCommand, SetWalkingParam, GetWalkingParam, GetWalkingState |
| usb_cam_node | usb_cam | /ainex/usb_cam/image_raw, /ainex/usb_cam/camera_info | — | — |
| imu_filter_node | imu_filter_madgwick | /ainex/imu/data | /ainex/imu/raw, /ainex/mag/data | — |
| safety_monitor [NEW] | ainex_control | /ainex/robot/status, /diagnostics | /ainex/imu/data, /ainex/joint_states | — |
| fall_detector [NEW] | ainex_control | /ainex/fall_event | /ainex/imu/data | — |
| color_detect_node [NEW] | ainex_perception | /ainex/color/objects | /ainex/usb_cam/image_raw | — |
| face_detect_node [NEW] | ainex_perception | /ainex/face/detections | /ainex/usb_cam/image_raw | — |
| apriltag_ros_continuous | apriltag_ros | /ainex/apriltag/detections, /tf | /ainex/usb_cam/image_raw | — |
| task_manager_node [NEW] | ainex_behavior | /ainex/task/status | /ainex/task/command, /ainex/fall_event | — |
| fall_rise_node | ainex_behavior | /ainex/task/status | /ainex/fall_event, /ainex/imu/data | SetWalkingCommand |
| gait_commander [NEW] | ainex_navigation | /ainex/gait/cmd | /ainex/task/command | SetWalkingCommand |
| visual_patrol_node | ainex_navigation | /ainex/gait/cmd | /ainex/line/error | SetWalkingCommand |
| joystick_control | ainex_peripherals | /ainex/joy_cmd | /joy | SetWalkingCommand |
| tf_broadcaster_imu | ainex_peripherals | /tf_static | /ainex/imu/data | — |
| web_video_server | web_video_server | HTTP:8080 | /ainex/usb_cam/image_raw | — |
| rosbridge_websocket | rosbridge_server | WS:9090 | — | — |
| app_node | ainex_app | /ainex/task/command | /ainex/task/status | — |
