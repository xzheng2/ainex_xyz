# ainex_bt_edu — Educational Behavior Tree Framework

## Overview (v2.0.0, 2026-04-15)
- **Package path**: `/home/pi/docker/ros_ws_src/ainex_bt_edu/` (host) → `/home/ubuntu/ros_ws/src/ainex_bt_edu/` (container)
- **Spec**: `ainex_bt_edu_spec.md` in package root (v2.0.0, 2026-04-15)
- **Active behaviours/ nodes**: **7 total** (L1: 2, L2: 5, L3: 0)
- **behaviours_tem/**: archive of 31 old-style nodes (v1.2.1, not importable — use for reference only)
- **Build**: `catkin build ainex_bt_edu ainex_behavior` inside container
- **Depends on**: py_trees >= 2.1, rospy, ainex_interfaces (ainex_kinematics/etc via facade only)

## Architecture (Facade Pattern, v2.0.0)
- `AinexBTFacade` (`base_facade.py`) — abstract interface; all project SemanticFacades must inherit
- `AinexBTNode` (`base_node.py`) — base class; auto-publishes `/bt_node_events` on status transition
- `AinexBTRunner` (`bt_runner.py`) — lifecycle manager, publishes `/bt_run_complete` (latch)
- `BlackboardROSBridge` (`bb_ros_bridge.py`) — mirrors BB keys to `/bt/bb/*` (10Hz, ROSA)
- `BB` class in `blackboard_keys.py` — all BB key constants + `ROSA_TOPIC_MAP`

## Key Design Rules (v2.0.0)
- Nodes hold only `AinexBTFacade`; NO direct ROS I/O (publisher/service/gait_manager)
- BB keys use `/latched/` namespace — latched once per tick before tree.tick()
- Constructor signature: `(name, facade=None, logger=None, tick_id_getter=None)`
- `setup()` must call `super().setup(**kwargs)` first

## Active Node Inventory (behaviours/)

### L1 Perception — 2 condition nodes
| Node | BB Read | Logic |
|------|---------|-------|
| L1_Balance_IsStanding | /latched/robot_state | == 'stand' → SUCCESS |
| L1_Vision_IsLineDetected | /latched/line_data | not None → SUCCESS |

### L2 Locomotion — 5 action nodes
| Node | BB R/W | Logic |
|------|--------|-------|
| L2_Gait_Stop | — | facade.stop_walking(); always SUCCESS |
| L2_Gait_FollowLine | R /latched/line_data | facade.move_head(500) then facade.follow_line(); always SUCCESS |
| L2_Balance_RecoverFromFall | R+W /latched/robot_state | facade.recover_from_fall(); writes 'stand'; calls robot_state_setter |
| L2_Gait_FindLine | R /latched/last_line_x, camera_lost_count | facade.search_line(); always RUNNING |
| L2_Head_FindLineSweep | R /latched/line_data, last_line_x; W /head_pan_pos | SWEEP→ALIGN state machine; SUCCESS when head centred |

### L3 Mission — 0 nodes (all in behaviours_tem/)

## Blackboard Keys (current)
- `/latched/robot_state` — str: 'stand' | 'lie_to_stand' | 'recline_to_stand'
- `/latched/line_data` — ObjectInfo | None
- `/latched/last_line_x` — float | None
- `/latched/camera_lost_count` — int
- `/head_pan_pos` — int (absolute, written by L2_Head_FindLineSweep; read by IsHeadCentered)
- `/tick_id` — int (written by MarathonBTNode, not in /latched)

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
