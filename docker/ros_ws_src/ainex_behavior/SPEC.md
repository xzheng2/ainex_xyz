# ainex_behavior — Marathon BT Package Spec

## Overview

Rewrite of the HuroCup marathon implementation as a `py_trees` behavior tree.
Safety priority is enforced through tree structure (SafetyGate blocks PatrolControl)
rather than if/else conditional logic.

## Dependencies

- `py_trees >= 2.0` (2.x blackboard Client API)
- `ros_robot_controller` (BuzzerState msg)
- `ainex_example` (Common base class)
- `ainex_interfaces` (ObjectsInfo, ColorDetect, ColorsDetect msgs)
- `hurocup2025` package (VisualPatrol — loaded dynamically via rospkg)

## Behavior Tree Structure

```
[-] MarathonBT (Sequence, memory=False)
    [o] SafetyGate (Selector, memory=False)
        --> IsRobotStanding     SUCCESS if bb.robot_state == 'stand'
        [-] Recovery (Sequence, memory=False)
            --> StopWalking         gait_manager.disable(), always SUCCESS
            --> RecoverFromFall     buzzer→sleep(2)→disable→action→sleep(0.5)→write 'stand'→SUCCESS
    [o] PatrolControl (Selector, memory=False)
        [-] LineFollowing (Sequence, memory=False)
            --> IsLineDetected      SUCCESS if bb.line_data is not None
            --> FollowLine          visual_patrol.process(x, width), SUCCESS
        --> StopWalking             gait_manager.disable(), always SUCCESS
```

## Blackboard Keys

| Key | Type | Writer | Readers |
|---|---|---|---|
| `/robot_state` | str | `_imu_callback` in node, `RecoverFromFall.update()` | `IsRobotStanding`, `RecoverFromFall` |
| `/line_data` | ObjectInfo \| None | `_objects_callback` in node | `IsLineDetected`, `FollowLine` |

## IMU Fall Detection

- `angle = abs(int(degrees(atan2(imu.linear_acceleration.y, imu.linear_acceleration.z))))`
- Falls detected only when `robot_state == 'stand'`
- `angle < 30` for N consecutive ticks → `robot_state = 'lie_to_stand'`
- `angle > 150` for N consecutive ticks → `robot_state = 'recline_to_stand'`
- Threshold: `FALL_COUNT_THRESHOLD = 100`

## Launch

```bash
roslaunch ainex_behavior marathon_bt_node.launch
```

Params: `start:=true` (auto-start), `color:=black` (line color to detect)

## Verification

```bash
# Build
cd /home/ubuntu/ros_ws && catkin build ainex_behavior

# Import check
python3 -c "import py_trees; print(py_trees.__version__)"

# Full launch (requires bringup)
roslaunch ainex_bringup bringup.launch
roslaunch ainex_behavior marathon_bt_node.launch
```
