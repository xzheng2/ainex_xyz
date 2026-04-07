# ainex_bt_edu — Educational Behavior Tree Framework

## Overview
- **Package path**: `/home/pi/docker/ros_ws_src/ainex_bt_edu/` (host) → `/home/ubuntu/ros_ws/src/ainex_bt_edu/` (container)
- **Spec**: `ainex_bt_edu_spec.md` in package root (v1.2.0, 2026-03-16)
- **51 files total**: 10 framework/build + 36 behaviour nodes + 4 `__init__.py` + 1 demo script
- **Build**: `catkin build ainex_bt_edu` inside container
- **Depends on**: py_trees >= 2.1, rospy, ainex_interfaces, ainex_example, ainex_kinematics, ros_robot_controller

## Architecture
- `AinexBTNode` base class (extends `py_trees.behaviour.Behaviour`) — auto-publishes structured events to `/bt_node_events` on every state transition
- `AinexBTRunner` — lifecycle manager, publishes `/bt_run_complete` (latch) when tree terminates
- `BlackboardROSBridge` — mirrors BB keys to `/bt/bb/*` topics (std_msgs/String JSON) at 10Hz for ROSA
- `BB` class in `blackboard_keys.py` — all BB key constants + `ROSA_TOPIC_MAP`

## Custom Messages
- `BTNodeEvent.msg`: header, node_name, level, prev_status, curr_status, tick_count, session_id, bb_snapshot (JSON)
- `BTRunComplete.msg`: header, session_id, status, duration_sec, tick_count, tree_name

## Three-Level Node System
- **L1 (perception/condition, 11 nodes)**: read-only checks, subscriber-based
- **L2 (action, 14 nodes)**: single-step hardware commands
- **L3 (composite/mission, 11 nodes)**: multi-step tasks combining L1/L2

## Node Inventory

### L1 Perception (behaviours/L1_perception/) — 11 condition nodes
| Node | Key Topic/BB | Logic |
|------|-------------|-------|
| L1_Balance_IsStanding | BB ROBOT_STATE | == 'stand' → SUCCESS |
| L1_Balance_IsFallen | Sub /imu, BB ROBOT_STATE | IMU angle thresholds, writes fall state |
| L1_Vision_IsLineDetected | BB LINE_DATA | not None → SUCCESS |
| L1_Vision_IsObjectDetected | Sub /object/pixel_coords | writes BB DETECTED_OBJECTS |
| L1_Vision_IsFaceDetected | Sub /object/pixel_coords | filters type=='face' |
| L1_System_IsButtonPressed | Sub /sensor/button/get_button_state | Bool → SUCCESS |
| L1_Battery_IsVoltageOk | Sub /ainex/battery (UInt16) | >= threshold_mv |
| L1_Motion_ListActions | Enumerates .d6a files | writes BB AVAILABLE_ACTIONS |
| L1_Vision_IsPatternDetected | BB DETECTED_OBJECTS | filters by obj.type |
| L1_Vision_IsObjectStill | BB DETECTED_OBJECTS | position buffer stability |
| L1_Vision_IsTargetOnLeft | BB TARGET_PIXEL_X | < center → SUCCESS |

### L2 Locomotion (behaviours/L2_locomotion/) — 14 action nodes
| Node | API | Logic |
|------|-----|-------|
| L2_Gait_Enable | gait_manager.enable() | writes BB GAIT_ENABLED=True |
| L2_Gait_Disable | gait_manager.disable() | always SUCCESS |
| L2_Gait_Start | ServiceProxy walking/command 'start' | |
| L2_Gait_Stop | gait_manager.stop() | |
| L2_Gait_SetParam | Pub AppWalkingParam | reads BB walk_x/y/angle/body_height |
| L2_Gait_FollowLine | visual_patrol.process() | reads BB LINE_DATA |
| L2_Balance_RecoverFromFall | buzz→sleep→disable→action | blocking, writes BB 'stand' |
| L2_Head_MoveTo | motion_manager.set_servos_position | servo 23/24, constructor values |
| L2_Head_TrackObject | PIDTrack → servo 23/24 | reads BB target_pixel_x/y |
| L2_Motion_PlayAction | motion_manager.run_action(name) | blocking, constructor name |
| L2_Motion_PlayFromBB | motion_manager.run_action(bb) | reads BB ACTION_NAME |
| L2_Vision_SetColorTarget | Pub ColorsDetect | configures color detection |
| L2_Vision_UpdateROI | Pub ColorsDetect | reads BB DETECT_ROI |
| L2_Arm_MoveToPreset | motion_manager.set_servos_position | arm servo presets |

### L3 Mission (behaviours/L3_mission/) — 11 composite nodes
| Node | Logic |
|------|-------|
| L3_Vision_TrackColorLine | Full _set_color config from BB TARGET_COLOR |
| L3_Vision_TrackColorObject | PID head tracking + color detection |
| L3_Vision_TrackFace | PID head tracking for faces |
| L3_Vision_StartColorDetectFromBB | Dynamic ColorsDetect (use_name True/False) |
| L3_Balance_SafeWalk | Internal sub-tree: IsStanding → Disable+Recover |
| L3_Gesture_RecognizeAndAct | Sub FingerPosition, writes BB GESTURE_LABEL |
| L3_Arm_PickObject | State machine: open→close gripper |
| L3_Arm_PlaceObject | open gripper→retract |
| L3_Mission_TagNavigate | AprilTag detect + PID track + walk |
| L3_Mission_ExecuteSequence | ROSA-driven: reads action list from BB |
| L3_Gait_ApproachObject | ApproachObject PID walk-toward |

## Blackboard Keys (BB class)
- `/perception/`: detected_objects, line_data, target_pixel_x/y, face_detected, gesture_label, target_world_pos, color_lab_min/max, color_detect_type, detect_roi
- `/locomotion/`: robot_state, gait_enabled, walk_x/y/angle, walk_body_height
- `/manipulation/`: gripper_state, approach_done
- `/mission/`: current_task, session_id, target_color, action_name, available_actions
- `robot_state` values: 'stand', 'walking', 'lie_to_stand', 'recline_to_stand'

## ROSA Integration
- `ROSA_TOPIC_MAP` (11 entries) mirrors key BB values to `/bt/bb/*` as JSON strings
- ROSA agent subscribes via standard ROS topics

## Key Design Decisions
- `behaviours/` lives under `src/ainex_bt_edu/behaviours/` (not package root) for clean Python imports
- All nodes extend `AinexBTNode`, declare `LEVEL` and `BB_LOG_KEYS` class attrs
- Subclasses only implement `update()` — logging is fully automatic via `tick()` override
- Reuses existing libs (GaitManager, MotionManager, PIDTrack, ApproachObject, VisualPatrol) — does NOT recreate them

## Fixing Action Interrupted by Tick (memory=False Selector)

With `memory=False` composites, `initialise()` is called **every tick** — the Selector stops and restarts all children each cycle. This breaks multi-tick actions that assume `initialise()` only fires once.

### Pattern: state machine action (e.g. FindLineHeadSweep)

**Problem symptoms:**
- `initialise()` log appears every tick, not just on first activation
- Internal state resets each tick, causing oscillation or direction flip
- Gait disabled and restarted at 30 Hz → stuttery motion

**Root causes and fixes:**

1. **Direction/state reset in `initialise()`** triggered by head position or other runtime value:
   - Fix: use an explicit `_fresh_start` bool flag instead of a value check.
   - Set `True` in `__init__` and when the action genuinely completes (SUCCESS).
   - Clear to `False` inside `initialise()` after using it.
   - Mid-tick reinits see `False` → preserve state unconditionally.

2. **`gait_manager.disable()` called unconditionally in `initialise()`**:
   - Kills the gait every tick during continuous motion phases (e.g. ALIGN turn).
   - Fix: guard with a condition that is only True during the idle/scan phase:
     ```python
     if self.bb.line_data is None:   # only disable during SWEEP (no line)
         self.visual_patrol.gait_manager.disable()
     ```
   - During active motion (line detected → ALIGN), skip disable so gait runs continuously.

3. **State reset in `initialise()` clobbering a sub-phase**:
   - `initialise()` resets `_state = _ST_SWEEP` every tick.
   - Safe because `_update_sweep()` immediately re-transitions to `_ST_ALIGN` when `line_data is not None`.
   - Persistence of `_head_pan` across ticks drives incremental head movement.

**General rule**: In `initialise()`, only perform resets that are safe to do every 30 Hz tick. Gate anything destructive (disable gait, reset direction, clear phase) behind a condition that is True only for genuine restarts.

## py_trees 2.1.6 Compatibility (CRITICAL)
These bugs were found and fixed during implementation. Keep for reference:

1. **`Blackboard.storage` (NOT `._storage`)**: py_trees 2.1.6 uses public `Blackboard.storage` dict. The private `_storage` does NOT exist and causes `AttributeError`.
2. **`tick()` is a generator**: Must use `yield`, not plain return. Override pattern:
   ```python
   def tick(self):
       prev = self.status
       self._tick_count += 1
       for node in super().tick():
           yield node
       if self.status != prev:
           self._emit_event(prev, self.status)
   ```
3. **Init ordering**: py_trees `Behaviour.__init__()` calls `self.tick()` internally (creates iterator). Instance vars accessed in `tick()` must be set BEFORE `super().__init__()`.
4. **BB keys keep leading `/`**: `Blackboard.storage` keys include the leading `/` (e.g. `/locomotion/robot_state`). Do NOT `lstrip('/')` when accessing storage directly. BB client `.get()` works with or without slash.

## Running the Example
```bash
# Dry-run (no hardware):
docker exec ainex bash -c "source /home/ubuntu/ros_ws/devel/setup.bash && rosrun ainex_bt_edu bt_edu_node.py _mode:=dry_run"

# Full patrol (needs bringup):
docker exec ainex bash -c "source /home/ubuntu/ros_ws/devel/setup.bash && rosrun ainex_bt_edu bt_edu_node.py"
```

## Migration Pattern
Old: `class FollowLine(py_trees.behaviour.Behaviour)` → New: `class L2_Gait_FollowLine(AinexBTNode)` with LEVEL='L2', BB_LOG_KEYS, super().setup(). update() body stays identical.

## Marathon BT ROS Publishing (added Mar 25 2026)

The marathon BT uses a **custom publisher shim** (NOT `py_trees_ros`) to publish tree state:

- **Why not py_trees_ros**: `ros-noetic-py-trees-ros` (0.6.x) requires py_trees 0.7.x, incompatible with 2.1.6
- **Shim files**: `marathon/tree_publisher.py` (TreeROSPublisher) + `marathon/bb_ros_bridge.py` (MarathonBBBridge)
- **TreeROSPublisher**: registers as `add_post_tick_handler`, converts py_trees 2.1.6 nodes to `py_trees_msgs/Behaviour` messages, publishes on `~log/tree` (consumed by rqt_py_trees)
- **MarathonBBBridge**: mirrors 4 BB keys to `/bt/marathon/bb/*` at 10 Hz (same pattern as ainex_bt_edu's `BlackboardROSBridge`)
- **Dependencies built from source**: `py_trees_msgs`, `uuid_msgs`, `unique_id`, `rqt_py_trees` (ROS Noetic arm64 apt packages 404/EOL)
- **rqt_py_trees**: auto-discovers any topic of type `py_trees_msgs/BehaviourTree` — no fixed topic name required
- **rqt_py_trees also imports** `py_trees.common.VisibilityLevel` and `BlackBoxLevel` — both exist in 2.1.6
