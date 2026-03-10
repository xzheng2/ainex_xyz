# Marathon BT — File Descriptions

## ROS node files (`*_node.py`)

There is one ROS node file in this folder — identified by the `_node` suffix,
it calls `rospy.init_node()`, is marked executable (`chmod +x`), and is
registered in `CMakeLists.txt` under `catkin_install_python`.

### `marathon_bt_node.py` ★ ROS node
The main ROS executable. Inherits from `Common` (ainex_example) so it gets
`gait_manager`, `motion_manager`, color-detection publisher, and the
standard start/stop/enter/exit services for free.

Responsibilities:
- Initialises the ROS node and the py_trees blackboard writer
- Subscribes to `/imu` → detects falls, writes `robot_state` to blackboard
- Subscribes to `/object/pixel_coords` → finds the line object, writes
  `line_data` to blackboard
- Publishes colour-detection parameters (ROI, colour name) to the camera
  pipeline via `_set_color()`
- Creates the `VisualPatrol` helper and the buzzer publisher, then calls
  `bootstrap()` to build the behaviour tree
- Runs the 30 Hz tick loop: tree is ticked only while `self.start` is True

---

## Helper / library files (not ROS nodes)

### `marathon_bt.py` — library module
Wires the full behaviour tree and exposes a `bootstrap()` factory function.

- `MarathonBT` — root `Sequence` (memory=False) that contains `SafetyGate`
  and `PatrolControl` as children
- `bootstrap(motion_manager, gait_manager, visual_patrol, buzzer_pub)` —
  builds the `MarathonBT` root, wraps it in a `BehaviourTree`, calls
  `tree.setup(timeout=5)`, and returns the ready-to-tick tree

### `marathon_bt_node.launch` — launch file
ROS launch file. Starts a single `marathon_bt_node.py` node with parameters:
- `start` (default `true`) — auto-start colour detection and walking on launch
- `color` (default `black`) — colour of the line to follow

---

## `behaviours/` package

### `behaviours/__init__.py`
Empty file. Makes `behaviours/` a Python package so the node can import from
it via `from behaviours.conditions import ...`.

### `behaviours/conditions.py`
Two read-only condition behaviours that check blackboard state each tick.

| Class | Returns SUCCESS when |
|---|---|
| `IsRobotStanding` | `blackboard.robot_state == 'stand'` |
| `IsLineDetected` | `blackboard.line_data is not None` |

Both register `/robot_state` or `/line_data` with `Access.READ` in `setup()`.

### `behaviours/actions.py`
Three action behaviours that interact with hardware or the blackboard.

| Class | What it does |
|---|---|
| `StopWalking` | Calls `gait_manager.disable()`, always returns SUCCESS. No blackboard access. Used as the fallback leaf in both the Recovery sequence and the PatrolControl selector. |
| `FollowLine` | Reads `line_data` from the blackboard and calls `visual_patrol.process(x, width)` to steer the gait. Returns SUCCESS. |
| `RecoverFromFall` | Reads `robot_state` to pick the right stand-up action (`lie_to_stand` or `recline_to_stand`). Publishes a buzzer beep, waits 2 s, disables gait, runs the motion action, waits 0.5 s, then writes `robot_state = 'stand'` back to the blackboard. Returns SUCCESS. |
