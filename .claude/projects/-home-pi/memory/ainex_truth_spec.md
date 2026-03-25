# Ainex Robot — Canonical Source-of-Truth Specification

---

## Version Metadata

| Field | Value |
|-------|-------|
| **Document version** | 1.0.0 |
| **Date** | 2026-03-06 |
| **ROS version** | ROS Noetic (Ubuntu 20.04) |
| **Applicable repo** | `/home/pi/docker/src` (container: `/home/ubuntu/ros_ws/src`) |
| **Repo commit** | pending — run `git -C /home/pi/docker/src log --oneline -1` to record |
| **Authority** | This document supersedes all memory files and inline comments for Topics, Services, and Servo IDs |
| **Status** | CANONICAL — derived from `servo_controller.yaml` + live code scan |

---

## Naming Conventions

### Unified Prefix Rules

| Subsystem | Prefix | Examples |
|-----------|--------|---------|
| Walking / Gait | `/walking/` | `/walking/command`, `/walking/get_param` |
| Sensor peripherals | `/sensor/` | `/sensor/button/get_button_state` |
| Camera | `/camera/` | `/camera/image_raw` |
| IMU (raw) | `/ros_robot_controller/` | `/ros_robot_controller/imu_raw` |
| IMU (calibrated) | `/` (root) | `/imu_corrected` (from `imu_calib` node) |
| IMU (filtered) | `/` (root) | `/imu` (from `imu_filter` node) |
| Hardware driver (STM32) | `/ros_robot_controller/` | `/ros_robot_controller/bus_servo/set_position` |
| Application layer | `/app/` | `/app/set_walking_param` |
| Vision detectors | `/<detector>/` | `/color_detection/image_result` |
| Unified vision output | `/object/` | `/object/pixel_coords` |

### Rules
1. All canonical topic/service paths begin with `/` (absolute, not relative).
2. `/ainex/set_*` prefixes for walking services are **DEPRECATED** — use `/walking/*`.
3. Robot-private node topics prefixed `~` (tilde) are internal; do not subscribe from external nodes.
4. Arm servo side order: **Left (L) ID is always ODD, Right (R) ID is always EVEN** in each joint pair.
5. Leg servo order: Ankle → Knee → Hip (distal to proximal). IDs 1-2 ankle, 3-4 ankle pitch, 5-6 knee, 7-8 hip pitch, 9-10 hip roll, 11-12 hip yaw.

---

## Canonical Topic Table

| # | Topic Path | Message Type | Publisher Node | Subscriber Node(s) | Rate | Purpose |
|---|-----------|-------------|----------------|-------------------|------|---------|
| T01 | `/ros_robot_controller/imu_raw` | `sensor_msgs/Imu` | `ros_robot_controller` | `imu_calib` | 100 Hz | Raw accel+gyro from STM32 (no orientation) |
| T02 | `/imu_corrected` | `sensor_msgs/Imu` | `imu_calib` | `imu_filter` | 100 Hz | Bias-corrected accel+gyro (no orientation quaternion — `imu_calib` only removes bias/scale) |
| T03 | `/imu` | `sensor_msgs/Imu` | `imu_filter` | perception, gait | 100 Hz | Fused IMU from `imu_complementary_filter`; **orientation quaternion IS in msg**; sensor Y-axis is up so roll≈90° upright (not 0°); fall check: `abs(roll-90)<30 and abs(pitch)<30` |
| T04 | `/ros_robot_controller/mag` | `sensor_msgs/MagneticField` | `ros_robot_controller` | `imu_filter` | 100 Hz | Raw magnetometer for heading fusion |
| T05 | `/ros_robot_controller/battery` | `std_msgs/UInt16` | `ros_robot_controller_node` | monitor, app | ~1 Hz | Battery voltage (mV) |
| T06 | `/camera/image_raw` | `sensor_msgs/Image` | `usb_cam_node` | vision nodes | 30 Hz | Raw RGB camera image (640×480) |
| T07 | ~~`/ainex/joint_states`~~ | — | — | — | — | **NOT IMPLEMENTED** — no publisher exists |
| T08 | `/walking/is_walking` | `std_msgs/Bool` | `ainex_controller` | app, behavior | on change | Gait engine on/off state |
| T09 | `/sensor/button/get_button_state` | `std_msgs/Bool` | `sensor_node` | behavior nodes | ~50 Hz | Onboard button press (True=pressed) |
| T10 | `/sensor/led/set_led_state` | `std_msgs/Bool` | behavior nodes | `sensor_node` | on demand | LED on/off |
| T11 | `/app/set_walking_param` | `ainex_interfaces/AppWalkingParam` | app, joystick | `ainex_controller` | on demand | High-level gait: x/y/angle/height |
| T12 | `/ros_robot_controller/bus_servo/set_position` | `ros_robot_controller/SetBusServosPosition` | `ainex_controller` | `ros_robot_controller_node` | 40 Hz | Multi-servo position command |
| T13 | `/ros_robot_controller/bus_servo/set_state` | `ros_robot_controller/SetBusServoState` | config nodes | `ros_robot_controller_node` | on demand | Servo enable/disable/config |
| T14 | `/ros_robot_controller/set_buzzer` | `ros_robot_controller/BuzzerState` | app nodes | `ros_robot_controller_node` | on demand | Buzzer (freq, duty, repeat) |
| T15 | `/ros_robot_controller/set_rgb` | `ros_robot_controller/RGBsState` | app nodes | `ros_robot_controller_node` | on demand | RGB LED array |
| T16 | `/ros_robot_controller/set_led` | `ros_robot_controller/LedState` | app nodes | `ros_robot_controller_node` | on demand | Binary LED |
| T17 | `/ros_robot_controller/set_oled` | `ros_robot_controller/OLEDState` | app nodes | `ros_robot_controller_node` | on demand | OLED display text |
| T18 | `/object/pixel_coords` | `ainex_interfaces/ObjectsInfo` | vision nodes | behavior, nav | on detection | Unified detected object list |
| T19 | `/color_detection/image_result` | `sensor_msgs/Image` | `color_detection_node` | web_video_server | 30 Hz | Annotated color-segmented image |
| T20 | `/face_detect/image_result` | `sensor_msgs/Image` | `face_detect_node` | web_video_server | 30 Hz | Annotated face detection image |
| T21 | `/color_detection/update_detect` | `ainex_interfaces/ColorsDetect` | app node | `color_detection_node` | on demand | Color detection configuration |
| T22 | `/tag_detections` | `apriltag_ros/AprilTagDetectionArray` | `apriltag_ros` | behavior nodes | 30 Hz | AprilTag detections |
| T23 | `/marathon_bt/log/tree` | `py_trees_msgs/BehaviourTree` | `marathon_bt` | rqt_py_trees, ROSA | per tick (30 Hz), latched | Full BT snapshot for rqt visualization |
| T24 | `/marathon_bt/ascii/snapshot` | `std_msgs/String` | `marathon_bt` | ROSA, debug | per tick, latched | Human-readable ASCII tree rendering |
| T25 | `/marathon_bt/tip` | `py_trees_msgs/Behaviour` | `marathon_bt` | ROSA, debug | per tick, latched | Current tip (deepest active) node |
| T26 | `/bt/marathon/bb/robot_state` | `std_msgs/String` | `marathon_bt` | ROSA | 10 Hz, latched | BB mirror: 'stand', 'lie_to_stand', 'recline_to_stand' (JSON) |
| T27 | `/bt/marathon/bb/line_data` | `std_msgs/String` | `marathon_bt` | ROSA | 10 Hz, latched | BB mirror: line detection object or null (JSON) |
| T28 | `/bt/marathon/bb/last_line_x` | `std_msgs/String` | `marathon_bt` | ROSA | 10 Hz, latched | BB mirror: last known line x-position (JSON) |
| T29 | `/bt/marathon/bb/line_lost_count` | `std_msgs/String` | `marathon_bt` | ROSA | 10 Hz, latched | BB mirror: consecutive frames without line (JSON) |
| T30 | `/bt_node_events` | `ainex_bt_edu/BTNodeEvent` | ainex_bt_edu nodes | ROSA | on state change | Per-node state transitions (prev/curr status, BB snapshot) |
| T31 | `/bt_run_complete` | `ainex_bt_edu/BTRunComplete` | `AinexBTRunner` | ROSA | on completion, latched | Session-level BT result (status, duration, tick count) |
| T32 | `/bt/bb/*` | `std_msgs/String` | `BlackboardROSBridge` | ROSA | 10 Hz | ainex_bt_edu BB mirrors (11 keys, JSON) |

---

## Canonical Service Table

| # | Service Path | srv Type | Server Node | Request Fields | Response Fields | Example Call |
|---|-------------|---------|-------------|----------------|-----------------|-------------|
| S01 | `/walking/command` | `ainex_interfaces/SetWalkingCommand` | `ainex_controller` | `string command` | `bool result` | `rosservice call /walking/command "command: 'enable_control'"` |
| S02 | `/walking/command` (start) | same as S01 | `ainex_controller` | `string command` | `bool result` | `rosservice call /walking/command "command: 'start'"` |
| S03 | `/walking/command` (stop) | same as S01 | `ainex_controller` | `string command` | `bool result` | `rosservice call /walking/command "command: 'stop'"` |
| S04 | `/walking/init_pose` | `std_srvs/Empty` | `ainex_controller` | — | — | `rosservice call /walking/init_pose "{}"` |
| S05 | `/walking/is_walking` | `ainex_interfaces/GetWalkingState` | `ainex_controller` | — | `bool state, string message` | `rosservice call /walking/is_walking "{}"` |
| S06 | `/walking/get_param` | `ainex_interfaces/GetWalkingParam` | `ainex_controller` | `bool get_param` | `WalkingParam parameters` | `rosservice call /walking/get_param "get_param: true"` |
| S07 | `/walking/set_param` | `ainex_interfaces/SetWalkingParam` | `ainex_controller` | `WalkingParam parameters` | `bool result` | See AppWalkingParam topic (T09) for real-time use |
| S08 | `/walking/get_offset` | `ainex_interfaces/GetWalkingOffset` | `ainex_controller` | `bool get_param` | `WalkingOffset parameters` | `rosservice call /walking/get_offset "get_param: true"` |
| S09 | `/walking/set_offset` | `ainex_interfaces/SetWalkingOffset` | `ainex_controller` | `WalkingOffset parameters` | `bool result` | See gait_manager.py for field list |
| S10 | `/walking/save_offset` | `std_srvs/Empty` | `ainex_controller` | — | — | `rosservice call /walking/save_offset "{}"` |
| S11 | `/ros_robot_controller/bus_servo/get_position` | `ros_robot_controller/GetBusServosPosition` | `ros_robot_controller_node` | `uint8[] id` | `bool success, BusServoPosition[] position` | `rosservice call /ros_robot_controller/bus_servo/get_position "{id: [23, 24]}"` |
| S12 | `/ros_robot_controller/bus_servo/get_state` | `ros_robot_controller/GetBusServoState` | `ros_robot_controller_node` | `GetBusServoCmd[] cmd` | `bool success, BusServoState[] state` | See ros_robot_controller srv definition |
| S13 | `/ros_robot_controller/pwm_servo/get_state` | `ros_robot_controller/GetPWMServoState` | `ros_robot_controller_node` | `GetPWMServoCmd[] cmd` | `bool success, PWMServoState[] state` | See ros_robot_controller srv definition |
| S14 | `/sensor/button/enable` | `std_srvs/SetBool` | `sensor_node` | `bool data` | `bool success, string message` | `rosservice call /sensor/button/enable "data: true"` |
| S15 | `/app_node/enter` | `ainex_interfaces/SetInt` | `app_node` | `int64 data` | `bool success, string message` | `rosservice call /app_node/enter "data: 1"` |
| S16 | `/app_node/set_running` | `std_srvs/SetBool` | `app_node` | `bool data` | `bool success, string message` | `rosservice call /app_node/set_running "data: true"` |

### Walking Command Values (S01-S03)
| Command String | Effect |
|---------------|--------|
| `enable_control` | Arm the gait engine (must call before start) |
| `disable_control` | Disarm gait engine |
| `start` | Begin walking with current AppWalkingParam |
| `stop` | Halt walking, hold pose |

---

## Canonical Servo ID Table

**Authority:** `ainex_driver/ainex_kinematics/config/servo_controller.yaml`
**Protocol:** RS485 @ 115200 baud via STM32 on `/dev/ttyAMA0`
**Pulse range:** 0–1000 (500 = mechanical zero for most joints)
**Arm rule:** L=ODD ID, R=EVEN ID. Leg rule: 1-2 ankle-roll, 3-4 ankle-pitch, 5-6 knee, 7-8 hip-pitch, 9-10 hip-roll, 11-12 hip-yaw.

### Leg Servos (IDs 1–12)

| ID | Joint Name | Side | Joint Group | Init Pulse | Min | Max | Direction Notes |
|----|-----------|------|-------------|-----------|-----|-----|----------------|
| 1 | ank_roll | L | Leg | 500 | 0 | 1000 | Standard |
| 2 | ank_roll | R | Leg | 500 | 0 | 1000 | Standard |
| 3 | ank_pitch | L | Leg | 500 | 0 | 1000 | Standard |
| 4 | ank_pitch | R | Leg | 500 | 0 | 1000 | Standard |
| 5 | knee | L | Leg | 240 | 0 | 1000 | Non-center init |
| 6 | knee | R | Leg | 760 | 0 | 1000 | Non-center init (mirror) |
| 7 | hip_pitch | L | Leg | 500 | 0 | 1000 | Standard |
| 8 | hip_pitch | R | Leg | 500 | 0 | 1000 | Standard |
| 9 | hip_roll | L | Leg | 500 | 0 | 1000 | Standard |
| 10 | hip_roll | R | Leg | 500 | 0 | 1000 | Standard |
| 11 | hip_yaw | L | Leg | 500 | 0 | 1000 | Standard |
| 12 | hip_yaw | R | Leg | 500 | 0 | 1000 | Standard |

### Arm Servos (IDs 13–22)

| ID | Joint Name | Side | Joint Group | Init Pulse | Min | Max | Direction Notes |
|----|-----------|------|-------------|-----------|-----|-----|----------------|
| 13 | sho_pitch | L | Arm | 875 | 1000 | 0 | **Reversed range** |
| 14 | sho_pitch | R | Arm | 125 | 1000 | 0 | **Reversed range** |
| 15 | sho_roll | L | Arm | 500 | 0 | 1000 | Standard |
| 16 | sho_roll | R | Arm | 500 | 0 | 1000 | Standard |
| 17 | el_pitch | L | Arm | 500 | 0 | 1000 | Standard |
| 18 | el_pitch | R | Arm | 500 | 0 | 1000 | Standard |
| 19 | el_yaw | L | Arm | 500 | 0 | 1000 | Standard |
| 20 | el_yaw | R | Arm | 500 | 0 | 1000 | Standard |
| 21 | gripper | L | Arm | 500 | 0 | 1000 | Standard |
| 22 | gripper | R | Arm | 500 | 0 | 1000 | Standard |

### Head Servos (IDs 23–24)

| ID | Joint Name | Side | Joint Group | Init Pulse | Min | Max | Direction Notes |
|----|-----------|------|-------------|-----------|-----|-----|----------------|
| 23 | head_pan | — | Head | 500 | 0 | 1000 | Standard (500=center) |
| 24 | head_tilt | — | Head | 500 | 0 | 1000 | Standard (500=level) |

---

## Compatibility Strategy

### Deprecated Names → Canonical Replacements

| Deprecated | Canonical | Status | Migration |
|-----------|----------|--------|-----------|
| `/ainex/set_walking_command` | `/walking/command` | DEPRECATED | Alias or remap in launch file |
| `/ainex/set_walking_param` | `/walking/set_param` (srv) or `/app/set_walking_param` (topic) | DEPRECATED | Use topic T09 for real-time |
| `/ainex/get_walking_param` | `/walking/get_param` | DEPRECATED | Update all callers |
| `/ainex/get_walking_state` | `/walking/is_walking` | DEPRECATED | Update all callers |
| `GetBusServoState` (bare) | `/ros_robot_controller/bus_servo/get_state` | INCOMPLETE — add prefix | Update all callers |
| `GetBusServosPosition` (bare) | `/ros_robot_controller/bus_servo/get_position` | INCOMPLETE — add prefix | Update all callers |

### Servo ID Deprecated Interpretations

The following servo ID↔joint assignments in `ainex_architecture.md` are INCORRECT and must not be used:

| ID (arch.md — WRONG) | ID (canonical — CORRECT) |
|---------------------|--------------------------|
| 1 = hip_yaw R | 1 = ank_roll L |
| 2 = hip_roll R | 2 = ank_roll R |
| IDs 1-6 = Right leg | IDs 1,3,5,7,9,11 = Left; 2,4,6,8,10,12 = Right |
| 13 = sho_pitch R | 13 = sho_pitch **L** |
| 14 = sho_pitch L | 14 = sho_pitch **R** |

---

## Teaching Usage Notes

### Minimum Interface Set (Student Must Know)

**Level 1 — Observation (read-only)**
```bash
# Monitor raw IMU (no orientation)
rostopic echo /ros_robot_controller/imu_raw

# Monitor calibrated IMU (has valid orientation quaternion)
rostopic echo /imu_corrected

# Monitor filtered IMU (fused quaternion orientation IS in msg; roll/pitch≈0° upright)
rostopic echo /imu

# Monitor battery (UInt16, millivolts)
rostopic echo /ros_robot_controller/battery

# Check walking state
rostopic echo /walking/is_walking
```

**Level 2 — Walking Control (service calls)**
```python
from ainex_interfaces.srv import SetWalkingCommand
import rospy

rospy.init_node('student_node')
cmd = rospy.ServiceProxy('/walking/command', SetWalkingCommand)

cmd('enable_control')   # arm gait engine
cmd('start')            # begin walking
# ... publish AppWalkingParam to /app/set_walking_param to control direction ...
cmd('stop')             # halt
```

**Level 3 — Gait Tuning (publish AppWalkingParam)**
```python
from ainex_interfaces.msg import AppWalkingParam
import rospy

pub = rospy.Publisher('/app/set_walking_param', AppWalkingParam, queue_size=1)
msg = AppWalkingParam()
msg.x = 0.02       # forward stride (m)
msg.y = 0.0        # lateral stride (m)
msg.angle = 5.0    # turn angle (deg)
msg.height = 0.025 # step height (m)
pub.publish(msg)
```

**Level 4 — Direct Servo Read (diagnostic only)**
```bash
rosservice call /ros_robot_controller/bus_servo/get_position "{id: [23, 24]}"
```

### Minimum Bringup Validation (≤30 min)
See `ainex_validation_checklist.md` for the ordered 30-minute procedure.

---

## Custom Message Reference

### ainex_interfaces Messages
| Message | Key Fields | Used By |
|---------|-----------|---------|
| `AppWalkingParam` | `speed, x, y, angle, height` | T09, joystick_control |
| `WalkingParam` | 25 gait fields (see walking_param.yaml) | S06, S07 |
| `ObjectsInfo` | `objects[]` (ObjectInfo: label, x, y, width, height) | T16 |
| `ColorsDetect` | `data[]` (ColorDetect: color_name, detect_ranges) | T19 |
| `FingerPosition` | `label, points[]` | hand_gesture |
| `HeadState` | `yaw, pitch, duration` | head control |

### py_trees_msgs Messages (built from source in workspace)
| Message | Key Fields | Used By |
|---------|-----------|---------|
| `BehaviourTree` | `header, behaviours[]` (Behaviour array) | T23 — rqt_py_trees visualization |
| `Behaviour` | `name, class_name, own_id, parent_id, child_ids[], tip_id, type, blackbox_level, status, message, is_active` | T23, T25 — per-node state |

### ainex_bt_edu Messages
| Message | Key Fields | Used By |
|---------|-----------|---------|
| `BTNodeEvent` | `header, node_name, level, prev_status, curr_status, tick_count, session_id, bb_snapshot` | T30 — per-node state transitions |
| `BTRunComplete` | `header, session_id, status, duration_sec, tick_count, tree_name` | T31 — session completion |

### ros_robot_controller Messages
| Message | Key Fields | Used By |
|---------|-----------|---------|
| `SetBusServosPosition` | `duration, position[]` (id+position) | T10 |
| `BuzzerState` | `freq, on_time, off_time, repeat` | T12 |
| `RGBsState` | `data[]` (RGBState: r, g, b, id) | T13 |
| `ButtonState` | `id, state` | T07 (internal) |
