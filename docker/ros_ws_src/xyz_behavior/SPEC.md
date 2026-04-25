# xyz_behavior — Marathon BT Package Spec

> 最后更新：2026-04-17

## Overview

HuroCup marathon implementation as a `py_trees` behavior tree.
Safety priority is enforced through tree structure (SafetyGate blocks PatrolControl)
rather than if/else conditional logic.

## Dependencies

- `py_trees >= 2.0` (2.x blackboard Client API)
- `ros_robot_controller` (BuzzerState msg)
- `ainex_example` (Common base class)
- `ainex_interfaces` (ObjectsInfo, ColorDetect, ColorsInfo msgs)
- `xyz_bt_edu` (standard BT nodes + input_adapters)
- `bt_observability` (DebugEventLogger + BTDebugVisitor, shared module)

## Behavior Tree Structure

```
[-] MarathonBT (Sequence, memory=False)
    [o] SafetyGate (Selector, memory=False)
        --> IsRobotStanding     SUCCESS if /latched/robot_state == 'stand'
        [-] Recovery (Sequence, memory=False)
            --> StopWalking         gait_manager.disable(), always SUCCESS
            --> RecoverFromFall     buzzer→sleep(2)→disable→action→sleep(0.5)→write 'stand'→SUCCESS
    [o] PatrolControl (Selector, memory=False)
        [-] LineFollowing (Sequence, memory=False)
            --> IsLineDetected      SUCCESS if /latched/line_data is not None
            --> FindLine | FindLineHeadSweep   (selected via ~find_line param)
            --> FollowLine          visual_patrol.process(x, width), SUCCESS
        --> StopWalking             gait_manager.disable(), always SUCCESS
```

## Input Adapters (xyz_bt_edu)

Sensor subscriptions live in `xyz_bt_edu/input_adapters/` (shared layer), **not** in `app/`.
Two-phase latch per tick (in `run()`):

```
Phase 1 (under lock):  snapshot_and_reset()  ← atomic snapshot of all adapters
Phase 2 (after lock):  write_snapshot()      ← writes BB + emits ros_in / input_state
tree.tick()
```

| Adapter | Subscribes | Latched BB Keys |
|---|---|---|
| `ImuBalanceStateAdapter` | `/imu` | `/latched/robot_state` |
| `LineDetectionAdapter` | `/object/pixel_coords` | `/latched/line_data`, `/latched/last_line_x`, `/latched/camera_lost_count` |

## Blackboard Keys

| Key | Type | Writer | Readers |
|---|---|---|---|
| `/latched/robot_state` | str | `ImuBalanceStateAdapter.write_snapshot()`, `RecoverFromFall` (via `robot_state_setter`) | `IsRobotStanding`, `RecoverFromFall` |
| `/latched/line_data` | ObjectInfo \| None | `LineDetectionAdapter.write_snapshot()` | `IsLineDetected`, `FollowLine`, `FindLine` |
| `/latched/last_line_x` | float \| None | `LineDetectionAdapter.write_snapshot()` | `FindLine`, `FollowLine` |
| `/latched/camera_lost_count` | int | `LineDetectionAdapter.write_snapshot()` | `FindLine` |
| `/tick_id` | int | `marathon_bt_node.run()` | BB bridge (→ `/bt/marathon/bb/tick_id`) |

## IMU Fall Detection

Handled by `ImuBalanceStateAdapter`:
- `angle = abs(int(degrees(atan2(imu.linear_acceleration.y, imu.linear_acceleration.z))))`
- Falls detected only when `robot_state == 'stand'`
- `angle < 30` for N consecutive messages → `robot_state = 'lie_to_stand'`
- `angle > 150` for N consecutive messages → `robot_state = 'recline_to_stand'`
- `RecoverFromFall` resets state to `'stand'` via `robot_state_setter` hook (= `ImuAdapter.force_state`)
- Threshold: `FALL_COUNT_THRESHOLD = 100`

## Observability

`bt_observability/` shared module — logs BT decisions + ROS communication per tick:

| File | Content |
|---|---|
| `log/bt_debug_lastrun.jsonl` | Full session BT events |
| `log/bt_debug_recent.jsonl` | Last 30 ticks (rolling) |
| `log/bt_ros_comm_debug_lastrun.jsonl` | Full session comm events (`ros_in`, `input_state`, `ros_out`) |
| `log/bt_ros_comm_debug_recent.jsonl` | Last 30 ticks comm (rolling) |
| `log/infra_comm_manifest_lastrun.json` | Static infra interface manifest (written on startup) |

ROS topics: `/bt_debug` (String), `/bt_ros_comm_debug` (String), `~log/tree` (py_trees_msgs).

## Launch

```bash
roslaunch xyz_behavior marathon_bt_node.launch
```

Params:
- `start:=true` — auto-start BT ticking
- `color:=black` — line color to detect
- `find_line:=gait` — FindLine variant (`gait` or `head_sweep`)
- `bt_mode:=run` — exec controller mode (`run` / `pause` / `step`)

## Verification

```bash
# Build
docker exec ainex bash -c "cd /home/ubuntu/ros_ws && catkin build xyz_behavior"

# Import check
docker exec ainex python3 -c "from marathon.tree.marathon_bt import bootstrap; print('OK')"

# Import layer check
docker exec ainex python3 /home/ubuntu/ros_ws/src/xyz_behavior/marathon/check_imports.py

# Full launch (requires bringup)
roslaunch ainex_bringup bringup.launch
roslaunch xyz_behavior marathon_bt_node.launch
```
