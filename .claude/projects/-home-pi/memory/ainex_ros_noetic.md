# /opt/ros/noetic — ROS Base Installation (Container)

## What it is
The **ROS Noetic Ninjemys system installation** inside the `ainex` container.
It is the foundation layer the entire ainex ROS workspace depends on — 295 packages.

```
/opt/ros/noetic/
├── lib/python3/dist-packages/   ← Python packages (rospy, msg types, tools)
├── lib/                         ← C++ .so libraries + compiled node binaries
├── include/                     ← C++ headers
├── share/                       ← package manifests, .msg/.srv defs, launch files
└── setup.bash                   ← sourced by workspace devel/setup.bash
```

## Python import resolution

Every non-local import in the workspace resolves here:

| Import statement | Resolved path (in container) |
|-----------------|------------------------------|
| `import rospy` | `/opt/ros/noetic/lib/python3/dist-packages/rospy/` |
| `from std_msgs.msg import Header` | `.../std_msgs/msg/_Header.py` |
| `from sensor_msgs.msg import Image, Imu` | `.../sensor_msgs/msg/_Image.py`, `_Imu.py` |
| `from geometry_msgs.msg import Pose, Quaternion` | `.../geometry_msgs/msg/_Pose.py`, `_Quaternion.py` |
| `from std_srvs.srv import Empty` | `.../std_srvs/srv/_Empty.py` |
| `from ainex_interfaces.msg import ...` | `/home/ubuntu/ros_ws/devel/` (workspace, NOT noetic) |
| `from ros_robot_controller.msg import ...` | `/home/ubuntu/ros_ws/devel/` (workspace, NOT noetic) |
| `import rospkg` | `.../rospkg/` |
| `import message_filters` | `.../message_filters/` |
| `import py_trees` | **NOT in noetic** — installed separately (`pip3`) |

## Compiled node binaries inside noetic

These are C++ nodes launched by `.launch` files — not Python, not in the workspace:

| Node | Binary path | Launched by |
|------|-------------|-------------|
| `usb_cam_node` | `/opt/ros/noetic/lib/usb_cam/usb_cam_node` | `ainex_peripherals/launch/usb_cam.launch` |
| `apriltag_ros` nodelet | `/opt/ros/noetic/lib/libapriltag.so` | `apriltag_detection_node.launch` |
| `image_proc` rectify nodelet | `/opt/ros/noetic/lib/image_proc/` | `usb_cam_with_calib.launch` |
| `roscore` / `rosmaster` | `/opt/ros/noetic/bin/roscore` | startup |

These publish the topics that workspace Python nodes subscribe to:
- `usb_cam_node` → `/camera/image_raw`
- `image_proc` → `/camera/image_rect_color`
- `apriltag_ros` → `/tag_detections` (contains `geometry_msgs/Pose` + `Quaternion`)

## Sourcing chain

```
/home/ubuntu/ros_ws/devel/setup.bash   ← workspace overlay
    └─ sources /opt/ros/noetic/setup.bash
                └─ PYTHONPATH += /opt/ros/noetic/lib/python3/dist-packages
                └─ LD_LIBRARY_PATH += /opt/ros/noetic/lib
                └─ PATH += /opt/ros/noetic/bin
```

Without sourcing, `import rospy` fails. All nodes require this to be sourced first.

## Rule
Never assume a ROS package (rospy, std_msgs, sensor_msgs, etc.) is missing.
These are always present at `/opt/ros/noetic/lib/python3/dist-packages/` inside the container.
Only `py_trees` and other non-ROS packages need separate verification:
`docker exec ainex python3 -c "import py_trees"`
