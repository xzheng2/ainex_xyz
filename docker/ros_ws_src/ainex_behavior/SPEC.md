# ainex_behavior — Behavior Tree Package Specification

## Context

The Ainex humanoid robot runs ROS 1 (catkin workspace at `ros_ws/src`).
The existing marathon task lives in `hurocup2025/scripts/marathon/` and uses a
**two-node, state-machine** design:

| Node | Role |
|------|------|
| `fall_rise_node.py` | Reads `/imu`, detects falls, publishes `/fall_rise/state` |
| `marathon_node.py`  | Reads `/object/pixel_coords` + `/fall_rise/state`, runs line-follow loop |

Control logic is encoded as `if/else` inside `run()` loops; priority is implicit
and the two processes are coupled through a ROS topic.

**Goal:** Rewrite the marathon task — and future tasks — using a
**py_trees behavior tree** architecture inside a new ROS package `ainex_behavior`.

---

## Target Repository Layout

```
ros_ws/src/
└── ainex_behavior/
    ├── CMakeLists.txt
    ├── package.xml
    ├── setup.py
    ├── SPEC.md                        ← this file
    └── marathon/
        ├── marathon_bt_node.py        ← main ROS node (entry point)
        ├── marathon_bt.py             ← behavior tree definition
        ├── marathon_bt_node.launch    ← ROS launch file
        └── behaviours/
            ├── __init__.py
            ├── conditions.py          ← Condition nodes
            └── actions.py            ← Action nodes
```

---

## Behavior Tree Structure

```
[-] MarathonBT                  [Sequence, memory=False]
    [o] SafetyGate              [Selector, memory=False]
        --> IsRobotStanding     [Condition]   SUCCESS if blackboard.robot_state == 'stand'
        [-] Recovery            [Sequence, memory=False]
            --> StopWalking     [Action]      calls gait_manager.disable()
            --> RecoverFromFall [Action]      plays lie/recline_to_stand action, resets state
    [o] PatrolControl           [Selector, memory=False]
        [-] LineFollowing       [Sequence, memory=False]
            --> IsLineDetected  [Condition]   SUCCESS if blackboard.line_data is not None
            --> FollowLine      [Action]      calls visual_patrol.process(x, width)
        --> StopWalking         [Action]      fallback: stop if no line visible
```

**Priority guarantee:** `SafetyGate` is the first child of the root `Sequence`.
If the robot has fallen, `SafetyGate` blocks execution of `PatrolControl` until
recovery succeeds — enforced by tree structure, no `if` statements required.

---

## Blackboard Keys

| Key | Type | Writer | Readers |
|-----|------|--------|---------|
| `robot_state` | `str` | `_imu_callback` in main node, `RecoverFromFall` action | `IsRobotStanding`, `RecoverFromFall` |
| `line_data` | `ObjectInfo \| None` | `_objects_callback` in main node | `IsLineDetected`, `FollowLine` |

`robot_state` values: `'stand'` · `'lie_to_stand'` · `'recline_to_stand'`

---

## Implementation Tasks

### 1. ROS Package Boilerplate

Create `CMakeLists.txt`, `package.xml`, and `setup.py` for the `ainex_behavior`
catkin package. It must declare:
- `rospy`, `std_msgs`, `sensor_msgs`, `ainex_interfaces`, `ainex_example` as
  dependencies.
- `scripts/marathon/marathon_bt_node.py` as an installed executable.
- No C++ targets needed.

### 2. `behaviours/conditions.py`

Implement two `py_trees.behaviour.Behaviour` subclasses:

**`IsRobotStanding`**
- Attaches blackboard client, registers `robot_state` (READ).
- `update()`: return `SUCCESS` if `robot_state == 'stand'`, else `FAILURE`.

**`IsLineDetected`**
- Attaches blackboard client, registers `line_data` (READ).
- `update()`: return `SUCCESS` if `line_data is not None`, else `FAILURE`.

### 3. `behaviours/actions.py`

Implement three `py_trees.behaviour.Behaviour` subclasses.
Constructor arguments provide hardware handles (no global state).

**`RecoverFromFall(name, motion_manager, gait_manager, buzzer_pub)`**
- Registers `robot_state` (READ_WRITE).
- `update()`:
  1. Publish buzzer alert (`BuzzerState(freq=1900, on_time=0.1, off_time=0.01, repeat=1)`).
  2. `time.sleep(2)` — wait for robot to settle.
  3. `gait_manager.disable()`.
  4. Run `motion_manager.run_action('lie_to_stand')` or `'recline_to_stand'`
     based on `robot_state`.
  5. `time.sleep(0.5)`.
  6. Write `robot_state = 'stand'` to blackboard.
  7. Return `SUCCESS`.

**`FollowLine(name, visual_patrol)`**
- Registers `line_data` (READ).
- `update()`: call `visual_patrol.process(line_data.x, line_data.width)`,
  return `SUCCESS`. Return `FAILURE` if `line_data` is `None`.

**`StopWalking(name, gait_manager)`**
- No blackboard keys.
- `update()`: `gait_manager.disable()`, return `SUCCESS`.

### 4. `marathon_bt.py`

Define `MarathonBT(py_trees.composites.Sequence)`:
- Constructor takes `(motion_manager, gait_manager, visual_patrol, buzzer_pub)`.
- `_build_tree()` wires the four composites and six leaf nodes as shown in the
  tree diagram above using `add_children()`.
- Expose a `bootstrap(...)` factory function for use by the main node.

### 5. `marathon_bt_node.py`

Define `MarathonBTNode(Common)` (inherits `ainex_example.color_common.Common`):

**`__init__`**
1. `rospy.init_node(name)`.
2. Set `head_pan_init = 500`, `head_tilt_init = 340`.
3. Call `super().__init__(name, head_pan_init, head_tilt_init)` — provides
   `self.gait_manager`, `self.motion_manager`, `self.detect_pub`.
4. Instantiate `VisualPatrol(self.gait_manager)`.
5. Create `buzzer_pub` publisher on `/ros_robot_controller/set_buzzer`.
6. Enable py_trees blackboard activity stream; create a writer client;
   initialise `robot_state = 'stand'` and `line_data = None`.
7. Initialise fall-detection counters `_count_lie = 0`, `_count_recline = 0`
   with threshold `FALL_COUNT_THRESHOLD = 100`.
8. Build BT via `bootstrap(...)`, call `bt.setup(timeout=15)`.
9. Subscribe `/imu` → `_imu_callback`, `/object/pixel_coords` → `_objects_callback`.
10. Advertise `~set_color` service → `_set_color_srv_callback`.
11. `signal.signal(SIGINT, _shutdown)`.
12. `motion_manager.run_action('walk_ready')`.
13. If ROS param `~start` is True: call `enter_func`, `_set_color_srv_callback`,
    `start_srv_callback`.

**`_imu_callback(msg: Imu)`**
- Compute `angle = abs(degrees(atan2(linear_acceleration.y, linear_acceleration.z)))`.
- Under `self.lock`, only act when `robot_state == 'stand'`:
  - `angle < 30`  → increment `_count_lie`, else reset.
  - `angle > 150` → increment `_count_recline`, else reset.
  - If `_count_lie >= FALL_COUNT_THRESHOLD`: write `robot_state = 'lie_to_stand'`.
  - If `_count_recline >= FALL_COUNT_THRESHOLD`: write `robot_state = 'recline_to_stand'`.

**`_objects_callback(msg: ObjectsInfo)`**
- Find first `obj` where `obj.type == 'line'`; write to `blackboard.line_data`.

**`_set_color_srv_callback(msg)`**
- Build `ColorDetect` param with three ROI bands (up/center/down) from
  `line_roi` list and `image_process_size = [160, 120]`.
- Publish via `self.detect_pub`.

**`run()`**
- `rospy.Rate(30)` loop while `self.running and not rospy.is_shutdown()`.
- If `self.start`: call `self.bt.tick(post_tick_handler=self._print_tree)`.
- On exit: `init_action`, `stop_srv_callback`, `rospy.signal_shutdown`.

**`_print_tree(tree)`**
- `rospy.loginfo_throttle(2, py_trees.display.unicode_tree(..., show_status=True))`.

### 6. `marathon_bt_node.launch`

```xml
<launch>
  <arg name="color" default="black"/>
  <arg name="start" default="true"/>
  <include file="$(find ainex_example)/scripts/color_detection/color_detection_node.launch"/>
  <include file="$(find ainex_bringup)/launch/base.launch"/>
  <node pkg="ainex_behavior" type="marathon_bt_node.py" name="marathon_bt" output="screen">
    <param name="start" value="$(arg start)"/>
    <param name="color" value="$(arg color)"/>
  </node>
</launch>
```

---

## Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| Single ROS node | Eliminates inter-process topic latency; fall detection and patrol share the same hardware handles. |
| `memory=False` on all composites | Every tick re-evaluates conditions from scratch; stale state never silently persists. |
| Hardware handles passed as constructor args | Behaviour nodes are fully testable without ROS; no singleton/global managers. |
| `StopWalking` as Selector fallback | Makes the PatrolControl subtree always return SUCCESS, preventing spurious tree failures when the line is temporarily lost. |
| `FALL_COUNT_THRESHOLD = 100` debounce | Prevents single noisy IMU frames from triggering recovery; matches original `fall_rise_node.py` logic. |
| `RecoverFromFall` blocks synchronously | Recovery is a hard real-time constraint; the BT intentionally stalls until the robot is upright before any patrol resumes. |

---

## Dependencies

```
Python  : py_trees >= 2.0
ROS 1   : rospy, std_msgs, sensor_msgs, ainex_interfaces, ainex_example,
          ainex_kinematics, ros_robot_controller
```

Install py_trees if not present:
```bash
pip install py_trees
```

---

## Files NOT Modified

The original `hurocup2025/scripts/marathon/` files (`marathon_node.py`,
`fall_rise_node.py`, `visual_patrol.py`) are left untouched. The new package
`ainex_behavior` is a parallel implementation.
