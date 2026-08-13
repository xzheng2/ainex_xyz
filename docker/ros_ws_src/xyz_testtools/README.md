# xyz_testtools

Hardware bring-up and calibration scripts. Standalone — run them directly, never
through `roslaunch` or `rosrun`.

`CATKIN_IGNORE` sits next to this file, so `catkin build` skips the directory: it
is not a package and has no `package.xml`. That is deliberate. These are one-off
diagnostics, not something any node depends on.

## Running them

Four of the six talk to the servo bus through `ainex_sdk`, and `frame_viewer.py`
subscribes to a camera topic, so the workspace has to be sourced first:

```bash
docker exec -u ubuntu -it ainex bash -lc \
  'source /home/ubuntu/ros_ws/devel/setup.bash && \
   python3 /home/ubuntu/ros_ws/src/xyz_testtools/<script>.py'
```

`Lab.py` is the exception — pure OpenCV, no ROS, no servo bus.

## Inventory

| Script | What it does | Needs |
|---|---|---|
| `Lab.py` | LAB colour-threshold picker with trackbars | OpenCV only |
| `frame_viewer.py` | Views a ROS camera topic and measures FPS | roscore + camera |
| `lookdown.py` | Points the head down (servo 23/24) | servo bus |
| `lookstraight.py` | Returns the head to level | servo bus |
| `test_all_servo_ids.py` | Sweeps every servo ID and reports which answer | servo bus |
| `test_servo_mapping.py` | Verifies joint→servo ID mapping against `walking_module.so` | servo bus |

The scripts that drive servos move the robot. Have it held or lying down before
running them.
