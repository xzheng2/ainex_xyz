# Marathon Source Code Import Map (Full Multi-Layer)

## Directory Structure

```
/home/pi/docker/ros_ws_src/
├── ainex_behavior/
│   ├── setup.py
│   └── marathon/
│       ├── marathon_bt_node.py      ← entry point (main ROS node)
│       ├── marathon_bt.py           ← BT factory
│       ├── bb_ros_bridge.py         ← blackboard → ROS topic bridge
│       ├── tree_publisher.py        ← py_trees → rqt_py_trees publisher
│       └── behaviours/
│           ├── __init__.py          ← empty
│           ├── actions.py
│           └── conditions.py
├── ainex_driver/
│   ├── ainex_sdk/src/ainex_sdk/
│   │   ├── __init__.py
│   │   ├── common.py
│   │   ├── misc.py
│   │   ├── pid.py
│   │   ├── gpio.py
│   │   ├── led.py
│   │   ├── fps.py
│   │   ├── asr.py
│   │   ├── tts.py
│   │   ├── sonar.py
│   │   ├── voice_play.py
│   │   ├── board_sensor_check.py
│   │   └── button.py
│   └── ainex_kinematics/src/ainex_kinematics/
│       ├── gait_manager.py
│       └── motion_manager.py
├── ainex_example/src/ainex_example/
│   ├── color_common.py
│   ├── face_common.py
│   ├── apriltag_common.py
│   ├── approach_object.py
│   ├── visual_patrol.py
│   └── pid_track.py
└── hurocup2025/scripts/marathon/
    ├── visual_patrol.py             ← injected via sys.path (used by BT)
    ├── visual_patrol_copy.py        ← tuning variant (not used by BT)
    ├── visual_patrol_original.py    ← archived original (not used by BT)
    ├── marathon_node.py             ← legacy two-node approach (not used by BT)
    └── fall_rise_node.py            ← legacy fall recovery node (not used by BT)
```

---

## Layer 1 — Entry Point

### `marathon_bt_node.py`
`/home/pi/docker/ros_ws_src/ainex_behavior/marathon/marathon_bt_node.py`

| Import | Resolved Path |
|--------|---------------|
| `os`, `sys`, `math`, `time`, `signal` | stdlib |
| `rospkg`, `rospy` | ROS external |
| `py_trees` | external |
| `std_msgs.msg.String` | ROS message |
| `sensor_msgs.msg.Imu` | ROS message |
| `ainex_interfaces.msg.ObjectsInfo, ColorDetect` | ROS message |
| `ros_robot_controller.msg.BuzzerState` | ROS message |
| `ainex_sdk.common` | `ainex_driver/ainex_sdk/src/ainex_sdk/common.py` |
| `ainex_example.color_common.Common` | `ainex_example/src/ainex_example/color_common.py` |
| `visual_patrol.VisualPatrol` | `hurocup2025/scripts/marathon/visual_patrol.py` (**sys.path inject**) |
| `marathon_bt.bootstrap` | `ainex_behavior/marathon/marathon_bt.py` |
| `tree_publisher.TreeROSPublisher` | `ainex_behavior/marathon/tree_publisher.py` (**sys.path inject**) |
| `bb_ros_bridge.MarathonBBBridge` | `ainex_behavior/marathon/bb_ros_bridge.py` (**sys.path inject**) |

**sys.path injection:**
```python
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))           # marathon/ dir
sys.path.insert(0, os.path.join(_hurocup_path, 'scripts', 'marathon'))   # hurocup2025
```

---

## Layer 2 — Direct Dependencies

### `marathon_bt.py`
`/home/pi/docker/ros_ws_src/ainex_behavior/marathon/marathon_bt.py`

| Import | Resolved Path |
|--------|---------------|
| `py_trees` | external |
| `behaviours.conditions.IsRobotStanding, IsLineDetected` | `marathon/behaviours/conditions.py` |
| `behaviours.actions.RecoverFromFall, FollowLine, StopWalking, FindLine` | `marathon/behaviours/actions.py` |

---

### `bb_ros_bridge.py`
`/home/pi/docker/ros_ws_src/ainex_behavior/marathon/bb_ros_bridge.py`

| Import | Resolved Path |
|--------|---------------|
| `json` | stdlib |
| `rospy` | ROS external |
| `py_trees` | external |
| `std_msgs.msg.String` | ROS message — **LEAF** |

---

### `tree_publisher.py`
`/home/pi/docker/ros_ws_src/ainex_behavior/marathon/tree_publisher.py`

| Import | Resolved Path |
|--------|---------------|
| `uuid` | stdlib |
| `rospy` | ROS external |
| `py_trees` | external |
| `unique_id` | ROS external (uuid_msgs companion) |
| `py_trees_msgs.msg` | ROS message — **LEAF** |
| `std_msgs.msg.String` | ROS message — **LEAF** |

---

### `color_common.py`
`/home/pi/docker/ros_ws_src/ainex_example/src/ainex_example/color_common.py`

| Import | Resolved Path |
|--------|---------------|
| `time`, `threading.RLock` | stdlib |
| `rospy` | ROS external |
| `std_srvs.srv.Empty, EmptyResponse` | ROS service |
| `ainex_interfaces.msg.ColorsDetect` | ROS message |
| `ainex_sdk.common` | `ainex_driver/ainex_sdk/src/ainex_sdk/common.py` |
| `ainex_kinematics.gait_manager.GaitManager` | `ainex_driver/ainex_kinematics/src/ainex_kinematics/gait_manager.py` |
| `ainex_kinematics.motion_manager.MotionManager` | `ainex_driver/ainex_kinematics/src/ainex_kinematics/motion_manager.py` |

---

### `visual_patrol.py` (hurocup2025 — the one actually used)
`/home/pi/docker/ros_ws_src/hurocup2025/scripts/marathon/visual_patrol.py`

| Import | Resolved Path |
|--------|---------------|
| `math` | stdlib |
| `ainex_sdk.misc` | `ainex_driver/ainex_sdk/src/ainex_sdk/misc.py` |
| `ainex_sdk.common` | `ainex_driver/ainex_sdk/src/ainex_sdk/common.py` |

---

## Layer 3 — Secondary Dependencies

### `behaviours/conditions.py`
`/home/pi/docker/ros_ws_src/ainex_behavior/marathon/behaviours/conditions.py`

| Import | Resolved Path |
|--------|---------------|
| `py_trees`, `py_trees.common.Access` | external — **LEAF** |

---

### `behaviours/actions.py`
`/home/pi/docker/ros_ws_src/ainex_behavior/marathon/behaviours/actions.py`

Exports: `StopWalking`, `FollowLine`, `FindLine`, `FindLineHeadSweep`, `RecoverFromFall`

| Import | Resolved Path |
|--------|---------------|
| `time` | stdlib |
| `rospy` | ROS external |
| `py_trees`, `py_trees.common.Access` | external |
| `ros_robot_controller.msg.BuzzerState` | ROS message — **LEAF** |

---

### `gait_manager.py`
`/home/pi/docker/ros_ws_src/ainex_driver/ainex_kinematics/src/ainex_kinematics/gait_manager.py`

| Import | Resolved Path |
|--------|---------------|
| `time`, `math` | stdlib |
| `rospy` | ROS external |
| `std_msgs.msg.Bool` | ROS message |
| `ainex_interfaces.msg.WalkingParam` | ROS message |
| `ainex_interfaces.srv.GetWalkingParam, SetWalkingCommand` | ROS service — **LEAF** |

---

### `motion_manager.py`
`/home/pi/docker/ros_ws_src/ainex_driver/ainex_kinematics/src/ainex_kinematics/motion_manager.py`

| Import | Resolved Path |
|--------|---------------|
| `os`, `time`, `sqlite3` | stdlib |
| `rospy` | ROS external |
| `ros_robot_controller.srv.GetBusServosPosition` | ROS service |
| `ros_robot_controller.msg.SetBusServosPosition, BusServoPosition` | ROS message — **LEAF** |

---

### `ainex_sdk/common.py`
`/home/pi/docker/ros_ws_src/ainex_driver/ainex_sdk/src/ainex_sdk/common.py`

| Import | Resolved Path |
|--------|---------------|
| `math`, `random` | stdlib |
| `cv2` | external (OpenCV) |
| `yaml` | external (PyYAML) |
| `numpy` | external (NumPy) |
| `rospy` | ROS external |
| `std_msgs.msg.Header` | ROS message |
| `sensor_msgs.msg.Image` | ROS message |
| `geometry_msgs.msg.Pose, Quaternion` | ROS message — **LEAF** |

---

### `ainex_sdk/misc.py`
`/home/pi/docker/ros_ws_src/ainex_driver/ainex_sdk/src/ainex_sdk/misc.py`

No imports — pure utility functions (`val_map`, `set_range`, `empty_func`) — **LEAF**

---

## Layer 4 — Full ainex_sdk Module

### `ainex_sdk/__init__.py`
`/home/pi/docker/ros_ws_src/ainex_driver/ainex_sdk/src/ainex_sdk/__init__.py`

| Import | Resolved Path |
|--------|---------------|
| `ros_robot_controller.ros_robot_controller_sdk.Board` | ROS external — **LEAF** |

---

### Other ainex_sdk files (not in marathon critical path, listed for completeness)

| File | Imports | Notes |
|------|---------|-------|
| `pid.py` | `time` | LEAF |
| `gpio.py` | none | LEAF (constants only) |
| `led.py` | `time`, `gpiod` | LEAF |
| `asr.py` | `time`, `smbus` | LEAF |
| `tts.py` | `time`, `smbus` | LEAF |
| `sonar.py` | `sys`, `time`, `smbus2` | LEAF |
| `voice_play.py` | `os` | LEAF |
| `fps.py` | `cv2`, `time`, `ainex_sdk.common` | circular (benign) |
| `button.py` | `time`, `gpiod`, `ainex_sdk.gpio` | → `gpio.py` (LEAF) |
| `board_sensor_check.py` | `time`, `threading`, `ainex_sdk.{button,led,Board}` | → `button.py`, `led.py` |

---

## Legacy Two-Node Files (hurocup2025 — not used by BT node)

These files implement the original two-process architecture that `marathon_bt_node.py` replaces.

### `marathon_node.py`
`/home/pi/docker/ros_ws_src/hurocup2025/scripts/marathon/marathon_node.py`

| Import | Resolved Path |
|--------|---------------|
| `time`, `signal` | stdlib |
| `rospy` | ROS external |
| `std_msgs.msg.String` | ROS message |
| `ainex_sdk.common` | `ainex_driver/ainex_sdk/src/ainex_sdk/common.py` |
| `ainex_example.color_common.Common` | `ainex_example/src/ainex_example/color_common.py` |
| `visual_patrol.VisualPatrol` | `hurocup2025/scripts/marathon/visual_patrol.py` (sys.path inject) |
| `ainex_interfaces.srv.SetString` | ROS service |
| `ainex_interfaces.msg.ObjectsInfo, ColorDetect` | ROS message |

---

### `fall_rise_node.py`
`/home/pi/docker/ros_ws_src/hurocup2025/scripts/marathon/fall_rise_node.py`

| Import | Resolved Path |
|--------|---------------|
| `cv2`, `time`, `math`, `signal` | stdlib / external (OpenCV) |
| `numpy` | external (NumPy) |
| `rospy` | ROS external |
| `threading.RLock` | stdlib |
| `sensor_msgs.msg.Imu` | ROS message |
| `std_srvs.srv.Empty, EmptyResponse` | ROS service |
| `ros_robot_controller.msg.BuzzerState` | ROS message |
| `ainex_kinematics.gait_manager.GaitManager` | `ainex_driver/ainex_kinematics/src/ainex_kinematics/gait_manager.py` |
| `ainex_kinematics.motion_manager.MotionManager` | `ainex_driver/ainex_kinematics/src/ainex_kinematics/motion_manager.py` |
| `ainex_sdk.common` | `ainex_driver/ainex_sdk/src/ainex_sdk/common.py` |
| `std_msgs.msg.String` | ROS message |

---

### `visual_patrol_copy.py` / `visual_patrol_original.py`
Both have identical imports to `visual_patrol.py`: `math`, `ainex_sdk.misc`, `ainex_sdk.common`. They differ only in gait tuning parameters.

---

## Full Dependency Tree

```
marathon_bt_node.py  [ENTRY]
├── ainex_sdk.common
│   └── cv2, yaml, numpy, rospy, ROS messages  [LEAF]
├── ainex_example.color_common.Common
│   ├── ainex_sdk.common  [LEAF - see above]
│   ├── ainex_kinematics.gait_manager
│   │   └── time, math, rospy, ROS messages/services  [LEAF]
│   └── ainex_kinematics.motion_manager
│       └── os, time, sqlite3, rospy, ROS messages/services  [LEAF]
├── visual_patrol.VisualPatrol  (hurocup2025, via sys.path)
│   ├── ainex_sdk.misc  [LEAF - no imports]
│   └── ainex_sdk.common  [LEAF - see above]
├── marathon_bt.bootstrap
│   ├── behaviours.conditions
│   │   └── py_trees  [LEAF]
│   └── behaviours.actions  (StopWalking, FollowLine, FindLine, RecoverFromFall)
│       └── time, rospy, py_trees, ros_robot_controller.msg  [LEAF]
├── tree_publisher.TreeROSPublisher  (marathon/ dir, via sys.path)
│   └── uuid, rospy, py_trees, unique_id, py_trees_msgs.msg, std_msgs  [LEAF]
└── bb_ros_bridge.MarathonBBBridge  (marathon/ dir, via sys.path)
    └── json, rospy, py_trees, std_msgs  [LEAF]
```

---

## All Resolved Absolute Paths (Custom Python Only)

| Absolute Path | Role |
|---------------|------|
| `.../ainex_behavior/marathon/marathon_bt_node.py` | Entry point |
| `.../ainex_behavior/marathon/marathon_bt.py` | BT factory |
| `.../ainex_behavior/marathon/bb_ros_bridge.py` | Blackboard → ROS topic bridge |
| `.../ainex_behavior/marathon/tree_publisher.py` | py_trees → rqt_py_trees publisher |
| `.../ainex_behavior/marathon/behaviours/actions.py` | Action behaviors (StopWalking, FollowLine, FindLine, FindLineHeadSweep, RecoverFromFall) |
| `.../ainex_behavior/marathon/behaviours/conditions.py` | Condition behaviors |
| `.../ainex_behavior/marathon/behaviours/__init__.py` | Empty module init |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/common.py` | Core util (cv2, yaml, numpy, geometry) |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/misc.py` | Math utils — no deps |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/__init__.py` | SDK init → Board |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/pid.py` | PID controller |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/gpio.py` | GPIO constants |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/led.py` | LED (gpiod) |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/fps.py` | FPS counter |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/asr.py` | Speech recognition (smbus) |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/tts.py` | Text-to-speech (smbus) |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/sonar.py` | Ultrasonic sensor (smbus2) |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/voice_play.py` | Voice playback |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/board_sensor_check.py` | Board sensor test |
| `.../ainex_driver/ainex_sdk/src/ainex_sdk/button.py` | Button (gpiod) |
| `.../ainex_driver/ainex_kinematics/src/ainex_kinematics/gait_manager.py` | Gait param management |
| `.../ainex_driver/ainex_kinematics/src/ainex_kinematics/motion_manager.py` | Servo/action group control |
| `.../ainex_example/src/ainex_example/color_common.py` | Common game base (color) |
| `.../ainex_example/src/ainex_example/face_common.py` | Common game base (face) |
| `.../ainex_example/src/ainex_example/apriltag_common.py` | Common game base (apriltag) |
| `.../ainex_example/src/ainex_example/approach_object.py` | Object approach logic |
| `.../ainex_example/src/ainex_example/visual_patrol.py` | Line following (unused by marathon) |
| `.../ainex_example/src/ainex_example/pid_track.py` | PID tracking — no deps |
| `.../hurocup2025/scripts/marathon/visual_patrol.py` | Line following (used by BT) |
| `.../hurocup2025/scripts/marathon/visual_patrol_copy.py` | Tuning variant (not used by BT) |
| `.../hurocup2025/scripts/marathon/visual_patrol_original.py` | Archived original (not used by BT) |
| `.../hurocup2025/scripts/marathon/marathon_node.py` | Legacy line-follow node (not used by BT) |
| `.../hurocup2025/scripts/marathon/fall_rise_node.py` | Legacy fall recovery node (not used by BT) |

> All paths prefixed with `/home/pi/docker/ros_ws_src/`

---

## Notes

- **Circular import:** `fps.py` imports `ainex_sdk.common` from within the same package — benign at runtime since `fps.py` is not imported by marathon.
- **Two `visual_patrol.py` copies exist:** marathon uses the `hurocup2025` version (injected via `sys.path`), not `ainex_example`'s copy.
- **`motion_manager.py` uses SQLite** to load servo action group files from disk (`.db` files).
- **External packages required inside container:** `py_trees`, `py_trees_msgs`, `unique_id`, `cv2`, `yaml`, `numpy`, `gpiod`, `smbus`, `smbus2`.
- **`FindLine` action** was added to `behaviours/actions.py`; `marathon_bt.py` now imports it alongside the original three actions.
- **`tree_publisher.py`** publishes `py_trees_msgs/BehaviourTree` and `py_trees_msgs/Behaviour` on `~log/tree`, `~ascii/snapshot`, and `~tip` for `rqt_py_trees` and ROSA monitoring.
- **`bb_ros_bridge.py`** mirrors blackboard keys (`/robot_state`, `/line_data`, `/last_line_x`, `/camera_lost_count`, `/tick_id`) to `/bt/marathon/bb/*` ROS topics at 10 Hz.
- **Legacy two-node files** (`marathon_node.py`, `fall_rise_node.py`) remain in the repo as reference but are superseded by `marathon_bt_node.py`.

---

## Tuning Log

### `FindLineHeadSweep` — `behaviours/actions.py`

| # | Change | Detail |
|---|--------|--------|
| 1 | **`_fresh_start` flag** | Replaced `if self._head_pan == HEAD_PAN_CENTER:` guard in `initialise()` with explicit `_fresh_start` bool. Prevents `_sweep_dir` reset when sweep passes through exactly 500 (triggered by step sizes that are factors of 200, e.g. 10, 20). `_fresh_start` set True in `__init__` and on ALIGN SUCCESS; cleared on first use in `initialise()`. |
| 2 | **Conditional `disable()` in `initialise()`** | `gait_manager.disable()` now only called when `line_data is None` (SWEEP mode). Previously called unconditionally every tick, stopping and restarting the gait at 30 Hz during ALIGN, causing stuttery turns. |
| 3 | **ALIGN: `move()` → `set_step()`** | ALIGN phase replaced `gait_manager.move(2, 0, 0, gait_yaw)` with `set_step()` using explicit `go_gait_param`/`go_dsp` vs `turn_gait_param`/`turn_dsp` switching. Threshold: `abs(gait_yaw) < 2` → go config, else → turn config. `x=0` throughout (body rotation only). |

### `VisualPatrol` — `hurocup2025/scripts/marathon/visual_patrol.py`

| # | Change | Detail |
|---|--------|--------|
| 4 | **`go_gait_param['hip_pitch_offset']`** | Changed 15 → 25 to match `turn_gait_param`, giving better forward lean for both gait types during ALIGN. |
