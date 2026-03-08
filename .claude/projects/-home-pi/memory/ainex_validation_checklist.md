# Ainex Validation Checklist

**Generated:** 2026-03-06
**Purpose:** Pre-class / pre-release acceptance validation. A teaching assistant can complete this within 30 minutes.
**Prerequisites:** Docker container `ainex` running. Robot powered on with battery > 10.5V.

---

## PHASE 0 — Environment Setup (5 min)

```bash
# Verify container is running
docker ps | grep ainex

# Enter container
docker exec -it -u ubuntu ainex bash

# Source ROS workspace
source /home/ubuntu/ros_ws/devel/setup.bash

# Verify ROS master
rostopic list | head -5
```

| Check | Command | Expected | Pass/Fail |
|-------|---------|----------|-----------|
| P0-1 | `docker ps \| grep ainex` | Container listed, STATUS=Up | |
| P0-2 | `echo $ROS_MASTER_URI` | `http://localhost:11311` | |
| P0-3 | `rosnode list` | Lists at least `ros_robot_controller_node`, `ainex_controller`, `sensor_node` | |

---

## PHASE 1 — Hardware Sensor Validation (10 min)

### V01 — IMU Data Flow
```bash
rostopic hz /ainex/imu/raw
```
**Expected:** ~100 Hz (95–105 Hz acceptable)
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V02 — IMU Filtered Data
```bash
rostopic echo /ainex/imu/data --once
```
**Expected:** Non-zero quaternion values, no NaN fields
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V03 — Battery Voltage
```bash
rostopic echo /ainex/battery --once
```
**Expected:** Float32 > 10.5 (Volts). STOP if < 10.0V.
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V04 — Camera Image
```bash
rostopic hz /ainex/usb_cam/image_raw
```
**Expected:** ~30 Hz (25–35 Hz acceptable)
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V05 — Joint States
```bash
rostopic echo /ainex/joint_states --once
```
**Expected:** 24 joint names, 24 position values
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V06 — Button Topic
```bash
rostopic echo /sensor/button/get_button_state
# Physically press the onboard button
```
**Expected:** Bool value toggles on press
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

---

## PHASE 2 — Service Availability Validation (5 min)

### V07 — Walking Services Listed
```bash
rosservice list | grep walking
```
**Expected output MUST include:**
```
/walking/command
/walking/get_offset
/walking/get_param
/walking/init_pose
/walking/is_walking
/walking/save_offset
/walking/set_offset
/walking/set_param
```
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

**FAIL criteria:** If `/ainex/set_walking_command` appears instead of `/walking/command` — legacy code is running.

### V08 — Hardware Servo Services Listed
```bash
rosservice list | grep ros_robot_controller
```
**Expected MUST include:**
```
/ros_robot_controller/bus_servo/get_position
/ros_robot_controller/bus_servo/get_state
/ros_robot_controller/pwm_servo/get_state
```
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V09 — Walking State Query
```bash
rosservice call /walking/is_walking "{}"
```
**Expected:** `state: False, message: ""` (robot not walking)
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V10 — Gait Parameters Read
```bash
rosservice call /walking/get_param "get_param: true"
```
**Expected:** WalkingParam struct with period_time ~400, z_move_amplitude ~0.02
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

---

## PHASE 3 — Servo ID Validation (5 min)

**SAFETY:** Ensure robot is on the floor, not on a stand, before moving servos.

### V11 — Read Head Servo Positions (Safe — small range)
```bash
rosservice call /ros_robot_controller/bus_servo/get_position "{id: [23, 24]}"
```
**Expected:** 2 positions, each ~500 (center)
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V12 — Read All Leg Servo Positions
```bash
rosservice call /ros_robot_controller/bus_servo/get_position "{id: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]}"
```
**Expected:** 12 positions. Knee IDs 5 (L knee) ~240, 6 (R knee) ~760.
**Cross-verify:** ID 5 position ≈ 240 and ID 6 ≈ 760 confirms canonical leg mapping is correct.
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

**CONFLICT VERIFICATION:** If ID 5 ≈ 500 (not 240), the servo_controller.yaml mapping may not be loaded — investigate config load path.

### V13 — Read All Arm Servo Positions
```bash
rosservice call /ros_robot_controller/bus_servo/get_position "{id: [13, 14, 15, 16, 17, 18, 19, 20, 21, 22]}"
```
**Expected:** Shoulder pitch L (ID 13) ≈ 875, shoulder pitch R (ID 14) ≈ 125 (reversed range).
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

---

## PHASE 4 — Walking Control Validation (5 min)

**SAFETY:** Stand clear. Robot on floor. Have E-stop ready (kill roslaunch).

### V14 — Enable Walking Engine
```bash
rosservice call /walking/command "command: 'enable_control'"
```
**Expected:** `result: True`
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V15 — Init Pose
```bash
rosservice call /walking/init_pose "{}"
```
**Expected:** Robot moves to standing pose. No response fields.
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V16 — Walking State Reflects Enabled
```bash
rosservice call /walking/is_walking "{}"
```
**Expected:** `state: False` (enabled but not yet started)
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V17 — Publish Gait Parameters (Forward)
```bash
# In a second terminal (or after V14):
rostopic pub -1 /app/set_walking_param ainex_interfaces/AppWalkingParam \
  "speed: 3
   x: 0.02
   y: 0.0
   angle: 0.0
   height: 0.025"
```
**Expected:** Parameter published. Robot does NOT move yet (needs 'start' command).
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V18 — Start Walking
```bash
rosservice call /walking/command "command: 'start'"
```
**Expected:** `result: True`. Robot begins walking forward.
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V19 — Stop Walking
```bash
rosservice call /walking/command "command: 'stop'"
```
**Expected:** `result: True`. Robot halts.
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V20 — Negative Test: Legacy Service Does Not Exist
```bash
rosservice call /ainex/set_walking_command "command: 'stop'" 2>&1
```
**Expected:** ERROR `[rosservice] ERROR: service [/ainex/set_walking_command] unavailable`
**Purpose:** Confirms deprecated path is not active. If it returns True — legacy code is running; investigate ainex_controller.py service registration.
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

---

## PHASE 5 — Namespace & Topic Integrity (5 min)

### V21 — No Topics Under Wrong Namespace
```bash
rostopic list | grep '/ainex/set_'
```
**Expected:** No output. Any `/ainex/set_*` topic indicates legacy publisher.
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V22 — IMU Rate Verification
```bash
rostopic hz /ainex/imu/raw -w 5
```
**Expected:** 100 Hz ± 5 Hz
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V23 — Service Type Verification
```bash
rosservice info /walking/command
```
**Expected:**
```
Node: /ainex_controller
URI: ...
Type: ainex_interfaces/SetWalkingCommand
Args: command
```
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

### V24 — Message Type Verification for Button Topic
```bash
rostopic info /sensor/button/get_button_state
```
**Expected:** `Type: std_msgs/Bool`
**Status:** [ ] PASS / [ ] FAIL / [ ] NOT VALIDATED

---

## Validation Status Summary Table

| ID | Description | Method | Expected | Status |
|----|-------------|--------|----------|--------|
| V01 | IMU raw @ 100 Hz | `rostopic hz` | 95-105 Hz | |
| V02 | IMU filtered data | `rostopic echo` | Valid quaternion | |
| V03 | Battery voltage | `rostopic echo` | >10.5V | |
| V04 | Camera @ 30 Hz | `rostopic hz` | 25-35 Hz | |
| V05 | 24 joint states | `rostopic echo` | 24 joints | |
| V06 | Button toggle | Physical press | Bool change | |
| V07 | Walking services | `rosservice list` | /walking/* listed | |
| V08 | Servo services | `rosservice list` | /ros_robot_controller/* | |
| V09 | Walking state = False | `rosservice call` | state: False | |
| V10 | Gait param read | `rosservice call` | period_time ~400 | |
| V11 | Head servo positions | `rosservice call` | ID 23,24 ≈ 500 | |
| V12 | Leg servo positions | `rosservice call` | ID 5≈240, ID 6≈760 | |
| V13 | Arm servo positions | `rosservice call` | ID 13≈875, ID 14≈125 | |
| V14 | Enable walking | `rosservice call` | result: True | |
| V15 | Init pose | `rosservice call` | Robot stands | |
| V16 | Walking state after enable | `rosservice call` | state: False | |
| V17 | Publish gait params | `rostopic pub` | No error | |
| V18 | Start walking | `rosservice call` | Robot moves | |
| V19 | Stop walking | `rosservice call` | Robot halts | |
| V20 | Legacy service absent | `rosservice call` | ERROR returned | |
| V21 | No legacy `/ainex/set_*` topics | `rostopic list` | Empty | |
| V22 | IMU rate | `rostopic hz` | 100±5 Hz | |
| V23 | Walking service type | `rosservice info` | ainex_interfaces/SetWalkingCommand | |
| V24 | Button topic type | `rostopic info` | std_msgs/Bool | |

---

## Acceptance Criteria (Quality Gates)

| Gate | Requirement | Min Pass Count |
|------|------------|---------------|
| QG1 | All sensor topics (V01-V05) pass | 5/5 |
| QG2 | All walking services (V07-V10) pass | 4/4 |
| QG3 | Servo ID positions match canonical init values (V11-V13) | 3/3 |
| QG4 | Full walk cycle (V14-V19) completes without crash | 6/6 |
| QG5 | Legacy namespace absence confirmed (V20-V21) | 2/2 |
| QG6 | Type verification (V22-V24) passes | 3/3 |

**Minimum to proceed with class:** QG1 + QG2 + QG3 + QG4 must all pass.
**Conflict still pending verification:** C05 (walking command values), C06 (IMU topic remap).

---

## Blocker Log

If any check fails, log here before proceeding:

| Check | Failure Observed | Suspected Cause | Resolution |
|-------|-----------------|----------------|------------|
| | | | |
