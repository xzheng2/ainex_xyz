# Marathon Source Code Import Map (Full Multi-Layer)

## Directory Structure

```
/home/pi/docker/ros_ws_src/
├── ainex_behavior/
│   ├── setup.py
│   └── marathon/
│       ├── marathon_bt_node.py      ← entry point (main ROS node)
│       ├── marathon_bt.py           ← BT factory
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
    └── visual_patrol.py             ← injected via sys.path
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
| `behaviours.actions.RecoverFromFall, FollowLine, StopWalking` | `marathon/behaviours/actions.py` |

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
└── marathon_bt.bootstrap
    ├── behaviours.conditions
    │   └── py_trees  [LEAF]
    └── behaviours.actions
        └── time, rospy, py_trees, ros_robot_controller.msg  [LEAF]
```

---

## All Resolved Absolute Paths (Custom Python Only)

| Absolute Path | Role |
|---------------|------|
| `.../ainex_behavior/marathon/marathon_bt_node.py` | Entry point |
| `.../ainex_behavior/marathon/marathon_bt.py` | BT factory |
| `.../ainex_behavior/marathon/behaviours/actions.py` | Action behaviors |
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
| `.../hurocup2025/scripts/marathon/visual_patrol.py` | Line following (used by marathon) |

> All paths prefixed with `/home/pi/docker/ros_ws_src/`

---

## Notes

- **Circular import:** `fps.py` imports `ainex_sdk.common` from within the same package — benign at runtime since `fps.py` is not imported by marathon.
- **Two `visual_patrol.py` copies exist:** marathon uses the `hurocup2025` version (injected via `sys.path`), not `ainex_example`'s copy.
- **`motion_manager.py` uses SQLite** to load servo action group files from disk (`.db` files).
- **External packages required inside container:** `py_trees`, `cv2`, `yaml`, `numpy`, `gpiod`, `smbus`, `smbus2`.
